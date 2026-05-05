// GC9A01 free-rotation viewer with BLE image upload
// Inherits the 64-step gravity-following rotation from gc9a01_rotate.
// Adds a BLE peripheral that accepts a custom multi-frame image payload
// (GIF-converted) and swaps the active animation.
//
// BLE protocol:
//   Service:  5550EEC9-B00F-4444-9876-AB12CD34EF01
//   Upload  (Write):       5550EEC9-B00F-4444-9876-AB12CD34EF02
//   Commit  (Write):       5550EEC9-B00F-4444-9876-AB12CD34EF03
//   Status  (Read/Notify): 5550EEC9-B00F-4444-9876-AB12CD34EF04
//
//   Upload: chunked writes appended in order to a 32 KB buffer.
//   Commit: 0x01 = validate & swap, 0x02 = buffer reset, 0x03 = revert default.
//   Status: 1 byte (0=idle, 1=receiving, 2=success, 3=error).
//
// Payload (sent via Upload, raw bytes appended in order):
//   [0..1]    Magic 'PP' (0x50 0x50)
//   [2]       Version = 1
//   [3]       Frame count N (1..16)
//   [4..35]   Palette: 16 x uint16 RGB565, little-endian (32 B)
//   [36..]    Per frame:
//               uint16 duration_ms (LE)
//               uint8  data[2048]   (4-bit packed 64x64; even idx = high nibble)
//   Total = 4 + 32 + N*2050 bytes  (max 32868 at N=16)
//
// BLE is active only while awake. Deep-sleep entry stops the radio implicitly.
//
// Persistence: a successful commit writes the upload buffer to LittleFS at
// /img.bin. On boot, the file is loaded back and validated before showing.
// Revert (0x03) removes the file. Requires a partition scheme that includes
// a SPIFFS/LittleFS partition (e.g. "Minimal SPIFFS").

#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---- BLE constants ----
#define BLE_DEVICE_NAME       "puchi-pix"
#define BLE_SERVICE_UUID      "5550eec9-b00f-4444-9876-ab12cd34ef01"
#define BLE_UPLOAD_UUID       "5550eec9-b00f-4444-9876-ab12cd34ef02"
#define BLE_COMMIT_UUID       "5550eec9-b00f-4444-9876-ab12cd34ef03"
#define BLE_STATUS_UUID       "5550eec9-b00f-4444-9876-ab12cd34ef04"

static constexpr uint8_t MAX_UPLOADED_FRAMES = 16;

static constexpr uint8_t STATUS_IDLE      = 0;
static constexpr uint8_t STATUS_RECEIVING = 1;
static constexpr uint8_t STATUS_SUCCESS   = 2;
static constexpr uint8_t STATUS_ERROR     = 3;

// Result of loadPersistedImage(). Declared up here so the Arduino auto-prototype
// generator sees the type before any function signatures that use it.
enum LoadResult : uint8_t {
  LOAD_OK       = 0,  // image restored
  LOAD_NONE     = 1,  // no persisted image (first boot / after revert)
  LOAD_FS_FAIL  = 2,  // filesystem mount or read failure
  LOAD_INVALID  = 3,  // file exists but is corrupt / wrong format
};

// ---- Pin assignments ----
static constexpr uint8_t TFT_SCK  = 4;
static constexpr uint8_t TFT_MOSI = 6;
static constexpr uint8_t TFT_CS   = 7;
static constexpr uint8_t TFT_DC   = 3;
static constexpr uint8_t TFT_RST  = 2;
static constexpr uint8_t TFT_BL   = 8;

static constexpr uint8_t I2C_SDA  = 20;
static constexpr uint8_t I2C_SCL  = 21;
static constexpr uint8_t KXTJ3_INT_PIN = 5;

static constexpr int WIDTH  = 240;
static constexpr int HEIGHT = 240;

// ---- Default image data (PROGMEM fallback) ----
#include "../puchi_pix/frame.h"
#include "../puchi_pix/icon_original.h"

static constexpr uint16_t FRAME_BYTES = (uint16_t)IMG_W * IMG_H / 2;  // 2048 for 64x64 4-bit
static constexpr uint16_t FRAME_REC   = FRAME_BYTES + 2;              // + uint16 duration
static constexpr size_t UPLOAD_BUF_SIZE = 4 + 32 + (size_t)MAX_UPLOADED_FRAMES * FRAME_REC;  // 32868

// 64x64 source -> 128x128 visible (fits 240 disc with rotation, diagonal 181)
static constexpr uint8_t SCALE_SHIFT = 1;
static constexpr int16_t OUT_SIZE = (IMG_W << SCALE_SHIFT) + 64;  // 192
static constexpr int16_t OUT_HALF = OUT_SIZE / 2;
static constexpr int16_t SRC_HALF = IMG_W / 2;

// ---- 64-step rotation table (Q15) ----
static const int16_t COS64[64] = {
   32767,  32610,  32138,  31357,  30274,  28898,  27246,  25330,
   23170,  20788,  18205,  15447,  12540,   9512,   6393,   3212,
       0,  -3212,  -6393,  -9512, -12540, -15447, -18205, -20788,
  -23170, -25330, -27246, -28898, -30274, -31357, -32138, -32610,
  -32767, -32610, -32138, -31357, -30274, -28898, -27246, -25330,
  -23170, -20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,
       0,   3212,   6393,   9512,  12540,  15447,  18205,  20788,
   23170,  25330,  27246,  28898,  30274,  31357,  32138,  32610,
};
static const int16_t SIN64[64] = {
       0,   3212,   6393,   9512,  12540,  15447,  18205,  20788,
   23170,  25330,  27246,  28898,  30274,  31357,  32138,  32610,
   32767,  32610,  32138,  31357,  30274,  28898,  27246,  25330,
   23170,  20788,  18205,  15447,  12540,   9512,   6393,   3212,
       0,  -3212,  -6393,  -9512, -12540, -15447, -18205, -20788,
  -23170, -25330, -27246, -28898, -30274, -31357, -32138, -32610,
  -32767, -32610, -32138, -31357, -30274, -28898, -27246, -25330,
  -23170, -20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,
};

// ---- SPI helpers ----

static inline void cmdByte(uint8_t cmd) {
  digitalWrite(TFT_DC, LOW);
  digitalWrite(TFT_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(TFT_CS, HIGH);
}

static inline void cmdWithData(uint8_t cmd, const uint8_t* data, uint8_t n) {
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(cmd);
  if (n) {
    digitalWrite(TFT_DC, HIGH);
    while (n--) SPI.transfer(*data++);
  }
  digitalWrite(TFT_CS, HIGH);
}

// ---- GC9A01 init ----

static void gc9a01Init() {
  static const uint8_t initTable[] PROGMEM = {
    0xEF, 0,
    0xEB, 1, 0x14,
    0xFE, 0,
    0xEF, 0,
    0xEB, 1, 0x14,
    0x84, 1, 0x40,  0x85, 1, 0xFF,  0x86, 1, 0xFF,  0x87, 1, 0xFF,
    0x88, 1, 0x0A,  0x89, 1, 0x21,  0x8A, 1, 0x00,  0x8B, 1, 0x80,
    0x8C, 1, 0x01,  0x8D, 1, 0x01,  0x8E, 1, 0xFF,  0x8F, 1, 0xFF,
    0xB6, 2, 0x00, 0x00,
    0x36, 1, 0x48,
    0x3A, 1, 0x05,
    0x90, 4, 0x08, 0x08, 0x08, 0x08,
    0xBD, 1, 0x06,  0xBC, 1, 0x00,
    0xFF, 3, 0x60, 0x01, 0x04,
    0xC3, 1, 0x13,  0xC4, 1, 0x13,  0xC9, 1, 0x22,  0xBE, 1, 0x11,
    0xE1, 2, 0x10, 0x0E,
    0xDF, 3, 0x21, 0x0C, 0x02,
    0xF0, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    0xF1, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    0xF2, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    0xF3, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    0xED, 2, 0x1B, 0x0B,
    0xAE, 1, 0x77,  0xCD, 1, 0x63,
    0x70, 9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03,
    0xE8, 1, 0x34,
    0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
    0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
    0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07,
    0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00,
    0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98,
    0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00,
    0x98, 2, 0x3E, 0x07,
    0x35, 0,
    0x21, 0,
    0x11, 0x80, 120,
    0x29, 0x80, 20,
  };
  const uint8_t* p = initTable;
  const uint8_t* end = initTable + sizeof(initTable);
  while (p < end) {
    uint8_t cmd = pgm_read_byte(p++);
    uint8_t nd = pgm_read_byte(p++);
    uint8_t n = nd & 0x7F;
    uint8_t buf[12];
    for (uint8_t i = 0; i < n; i++) buf[i] = pgm_read_byte(p++);
    cmdWithData(cmd, buf, n);
    if (nd & 0x80) delay(pgm_read_byte(p++));
  }
}

// ---- Drawing primitives ----

static void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  cmdByte(0x2A);
  uint8_t cx[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_CS, LOW);
  for (uint8_t i = 0; i < 4; i++) SPI.transfer(cx[i]);
  digitalWrite(TFT_CS, HIGH);

  cmdByte(0x2B);
  uint8_t cy[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_CS, LOW);
  for (uint8_t i = 0; i < 4; i++) SPI.transfer(cy[i]);
  digitalWrite(TFT_CS, HIGH);

  cmdByte(0x2C);
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_CS, LOW);
}

static inline void writePixel(uint16_t c) {
  SPI.transfer(c >> 8);
  SPI.transfer(c & 0xFF);
}

static inline uint8_t read4bit(const uint8_t* data, uint16_t idx) {
  uint8_t b = pgm_read_byte(&data[idx >> 1]);
  return (idx & 1) ? (b & 0x0F) : (b >> 4);
}

static inline uint8_t read4bitRam(const uint8_t* data, uint16_t idx) {
  uint8_t b = data[idx >> 1];
  return (idx & 1) ? (b & 0x0F) : (b >> 4);
}

// ---- Upload buffer + state ----
static uint8_t uploadBuf[UPLOAD_BUF_SIZE];
static volatile uint32_t uploadPos = 0;
static volatile uint8_t uploadStatus = STATUS_IDLE;
static volatile bool bleConnected = false;
static volatile bool uploadCommittedFlag = false;
static volatile bool uploadedActive = false;
static volatile uint8_t uploadedFrameCount = 0;

// Deferred persistence flags (set in BLE callbacks, applied in loop()).
static volatile bool persistRequested = false;
static volatile bool clearPersistRequested = false;

#define IMG_PATH "/img.bin"

// Accessors over uploadBuf
static inline const uint16_t* uploadedPalette() {
  return (const uint16_t*)&uploadBuf[4];
}
static inline const uint8_t* uploadedFrameData(uint8_t idx) {
  return &uploadBuf[4 + 32 + (uint32_t)idx * FRAME_REC + 2];
}
static inline uint16_t uploadedFrameDuration(uint8_t idx) {
  const uint8_t* p = &uploadBuf[4 + 32 + (uint32_t)idx * FRAME_REC];
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ---- Compose dispatchers ----

static void composeFrameFromProgmem(uint8_t frameIdx, uint16_t* buf) {
  uint8_t ftype = pgm_read_byte(&frames[frameIdx].type);
  const uint8_t* data = (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);

  if (ftype == 0) {
    for (uint16_t i = 0; i < IMG_W * IMG_H; i++)
      buf[i] = pgm_read_word(&palette[read4bit(data, i)]);
  } else {
    uint8_t ref = pgm_read_byte(&frames[frameIdx].ref);
    const uint8_t* refData = (const uint8_t*)pgm_read_ptr(&frames[ref].data);
    for (uint16_t i = 0; i < IMG_W * IMG_H; i++)
      buf[i] = pgm_read_word(&palette[read4bit(refData, i)]);

    uint8_t rx = pgm_read_byte(&frames[frameIdx].rx);
    uint8_t ry = pgm_read_byte(&frames[frameIdx].ry);
    uint8_t rw = pgm_read_byte(&frames[frameIdx].rw);
    uint8_t rh = pgm_read_byte(&frames[frameIdx].rh);
    for (uint8_t dy = 0; dy < rh; dy++) {
      for (uint8_t dx = 0; dx < rw; dx++) {
        uint16_t srcIdx = (uint16_t)dy * rw + dx;
        uint16_t dstIdx = (uint16_t)(ry + dy) * IMG_W + (rx + dx);
        buf[dstIdx] = pgm_read_word(&palette[read4bit(data, srcIdx)]);
      }
    }
  }
}

static void composeFrameFromUpload(uint8_t frameIdx, uint16_t* buf) {
  if (frameIdx >= uploadedFrameCount) frameIdx = 0;
  const uint16_t* pal = uploadedPalette();
  const uint8_t* data = uploadedFrameData(frameIdx);
  for (uint16_t i = 0; i < IMG_W * IMG_H; i++) {
    buf[i] = pal[read4bitRam(data, i)];
  }
}

static void composeFrame(uint8_t frameIdx, uint16_t* buf) {
  if (uploadedActive) composeFrameFromUpload(frameIdx, buf);
  else                composeFrameFromProgmem(frameIdx, buf);
}

// Frame metadata accessors (dispatch source).
static uint16_t getDuration(uint8_t idx) {
  if (uploadedActive) {
    if (idx >= uploadedFrameCount) idx = 0;
    return uploadedFrameDuration(idx);
  }
  return pgm_read_word(&frames[idx].duration_ms);
}

static uint8_t getNextFrame(uint8_t idx) {
  if (uploadedActive) {
    uint8_t n = uploadedFrameCount;
    if (n == 0) return 0;
    return (uint8_t)((idx + 1) % n);
  }
  return pgm_read_byte(&frames[idx].next);
}

// Reverse-rotate source buffer onto a centered OUT_SIZE x OUT_SIZE region.
static void drawFrameRotated(const uint16_t* src, uint8_t bin) {
  const int32_t cosT = COS64[bin & 0x3F];
  const int32_t sinT = SIN64[bin & 0x3F];

  const uint16_t ox0 = (WIDTH - OUT_SIZE) / 2;
  const uint16_t oy0 = (HEIGHT - OUT_SIZE) / 2;
  setWindow(ox0, oy0, ox0 + OUT_SIZE - 1, oy0 + OUT_SIZE - 1);

  const int8_t SHIFT = 15 + SCALE_SHIFT;

  for (int16_t oy = 0; oy < OUT_SIZE; oy++) {
    int32_t cy = oy - OUT_HALF;
    int32_t cySin = cy * sinT;
    int32_t cyCos = cy * cosT;
    for (int16_t ox = 0; ox < OUT_SIZE; ox++) {
      int32_t cx = ox - OUT_HALF;
      int16_t sx = (int16_t)(((cx * cosT) + cySin) >> SHIFT) + SRC_HALF;
      int16_t sy = (int16_t)(((-cx * sinT) + cyCos) >> SHIFT) + SRC_HALF;
      uint16_t c;
      if ((uint16_t)sx < (uint16_t)IMG_W && (uint16_t)sy < (uint16_t)IMG_H) {
        c = src[(uint16_t)sy * IMG_W + sx];
      } else {
        c = 0x0000;
      }
      writePixel(c);
    }
  }
  digitalWrite(TFT_CS, HIGH);
}

static void fillScreen(uint16_t color) {
  setWindow(0, 0, WIDTH - 1, HEIGHT - 1);
  for (uint32_t i = 0; i < (uint32_t)WIDTH * HEIGHT; i++)
    writePixel(color);
  digitalWrite(TFT_CS, HIGH);
}

// ---- Text rendering (5x7 ASCII font) ----
// Each char is 5 columns x 7 rows. Bytes encode columns; bit 0 = top row.
// Cell layout: 5 col + 1 spacing col, 7 row + 1 spacing row -> 6x8 cell.
static const uint8_t font5x7[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00,  // 0x20 ' '
  0x00, 0x00, 0x5F, 0x00, 0x00,  // 0x21 '!'
  0x00, 0x07, 0x00, 0x07, 0x00,  // 0x22 '"'
  0x14, 0x7F, 0x14, 0x7F, 0x14,  // 0x23 '#'
  0x24, 0x2A, 0x7F, 0x2A, 0x12,  // 0x24 '$'
  0x23, 0x13, 0x08, 0x64, 0x62,  // 0x25 '%'
  0x36, 0x49, 0x55, 0x22, 0x50,  // 0x26 '&'
  0x00, 0x05, 0x03, 0x00, 0x00,  // 0x27 '\''
  0x00, 0x1C, 0x22, 0x41, 0x00,  // 0x28 '('
  0x00, 0x41, 0x22, 0x1C, 0x00,  // 0x29 ')'
  0x14, 0x08, 0x3E, 0x08, 0x14,  // 0x2A '*'
  0x08, 0x08, 0x3E, 0x08, 0x08,  // 0x2B '+'
  0x00, 0x50, 0x30, 0x00, 0x00,  // 0x2C ','
  0x08, 0x08, 0x08, 0x08, 0x08,  // 0x2D '-'
  0x00, 0x60, 0x60, 0x00, 0x00,  // 0x2E '.'
  0x20, 0x10, 0x08, 0x04, 0x02,  // 0x2F '/'
  0x3E, 0x51, 0x49, 0x45, 0x3E,  // 0x30 '0'
  0x00, 0x42, 0x7F, 0x40, 0x00,  // 0x31 '1'
  0x42, 0x61, 0x51, 0x49, 0x46,  // 0x32 '2'
  0x21, 0x41, 0x45, 0x4B, 0x31,  // 0x33 '3'
  0x18, 0x14, 0x12, 0x7F, 0x10,  // 0x34 '4'
  0x27, 0x45, 0x45, 0x45, 0x39,  // 0x35 '5'
  0x3C, 0x4A, 0x49, 0x49, 0x30,  // 0x36 '6'
  0x01, 0x71, 0x09, 0x05, 0x03,  // 0x37 '7'
  0x36, 0x49, 0x49, 0x49, 0x36,  // 0x38 '8'
  0x06, 0x49, 0x49, 0x29, 0x1E,  // 0x39 '9'
  0x00, 0x36, 0x36, 0x00, 0x00,  // 0x3A ':'
  0x00, 0x56, 0x36, 0x00, 0x00,  // 0x3B ';'
  0x00, 0x08, 0x14, 0x22, 0x41,  // 0x3C '<'
  0x14, 0x14, 0x14, 0x14, 0x14,  // 0x3D '='
  0x41, 0x22, 0x14, 0x08, 0x00,  // 0x3E '>'
  0x02, 0x01, 0x51, 0x09, 0x06,  // 0x3F '?'
  0x32, 0x49, 0x79, 0x41, 0x3E,  // 0x40 '@'
  0x7E, 0x11, 0x11, 0x11, 0x7E,  // 0x41 'A'
  0x7F, 0x49, 0x49, 0x49, 0x36,  // 0x42 'B'
  0x3E, 0x41, 0x41, 0x41, 0x22,  // 0x43 'C'
  0x7F, 0x41, 0x41, 0x22, 0x1C,  // 0x44 'D'
  0x7F, 0x49, 0x49, 0x49, 0x41,  // 0x45 'E'
  0x7F, 0x09, 0x09, 0x01, 0x01,  // 0x46 'F'
  0x3E, 0x41, 0x41, 0x51, 0x32,  // 0x47 'G'
  0x7F, 0x08, 0x08, 0x08, 0x7F,  // 0x48 'H'
  0x00, 0x41, 0x7F, 0x41, 0x00,  // 0x49 'I'
  0x20, 0x40, 0x41, 0x3F, 0x01,  // 0x4A 'J'
  0x7F, 0x08, 0x14, 0x22, 0x41,  // 0x4B 'K'
  0x7F, 0x40, 0x40, 0x40, 0x40,  // 0x4C 'L'
  0x7F, 0x02, 0x04, 0x02, 0x7F,  // 0x4D 'M'
  0x7F, 0x04, 0x08, 0x10, 0x7F,  // 0x4E 'N'
  0x3E, 0x41, 0x41, 0x41, 0x3E,  // 0x4F 'O'
  0x7F, 0x09, 0x09, 0x09, 0x06,  // 0x50 'P'
  0x3E, 0x41, 0x51, 0x21, 0x5E,  // 0x51 'Q'
  0x7F, 0x09, 0x19, 0x29, 0x46,  // 0x52 'R'
  0x46, 0x49, 0x49, 0x49, 0x31,  // 0x53 'S'
  0x01, 0x01, 0x7F, 0x01, 0x01,  // 0x54 'T'
  0x3F, 0x40, 0x40, 0x40, 0x3F,  // 0x55 'U'
  0x1F, 0x20, 0x40, 0x20, 0x1F,  // 0x56 'V'
  0x3F, 0x40, 0x38, 0x40, 0x3F,  // 0x57 'W'
  0x63, 0x14, 0x08, 0x14, 0x63,  // 0x58 'X'
  0x07, 0x08, 0x70, 0x08, 0x07,  // 0x59 'Y'
  0x61, 0x51, 0x49, 0x45, 0x43,  // 0x5A 'Z'
  0x00, 0x7F, 0x41, 0x41, 0x00,  // 0x5B '['
  0x02, 0x04, 0x08, 0x10, 0x20,  // 0x5C '\\'
  0x00, 0x41, 0x41, 0x7F, 0x00,  // 0x5D ']'
  0x04, 0x02, 0x01, 0x02, 0x04,  // 0x5E '^'
  0x40, 0x40, 0x40, 0x40, 0x40,  // 0x5F '_'
  0x00, 0x01, 0x02, 0x04, 0x00,  // 0x60 '`'
  0x20, 0x54, 0x54, 0x54, 0x78,  // 0x61 'a'
  0x7F, 0x48, 0x44, 0x44, 0x38,  // 0x62 'b'
  0x38, 0x44, 0x44, 0x44, 0x20,  // 0x63 'c'
  0x38, 0x44, 0x44, 0x48, 0x7F,  // 0x64 'd'
  0x38, 0x54, 0x54, 0x54, 0x18,  // 0x65 'e'
  0x08, 0x7E, 0x09, 0x01, 0x02,  // 0x66 'f'
  0x0C, 0x52, 0x52, 0x52, 0x3E,  // 0x67 'g'
  0x7F, 0x08, 0x04, 0x04, 0x78,  // 0x68 'h'
  0x00, 0x44, 0x7D, 0x40, 0x00,  // 0x69 'i'
  0x20, 0x40, 0x44, 0x3D, 0x00,  // 0x6A 'j'
  0x7F, 0x10, 0x28, 0x44, 0x00,  // 0x6B 'k'
  0x00, 0x41, 0x7F, 0x40, 0x00,  // 0x6C 'l'
  0x7C, 0x04, 0x18, 0x04, 0x78,  // 0x6D 'm'
  0x7C, 0x08, 0x04, 0x04, 0x78,  // 0x6E 'n'
  0x38, 0x44, 0x44, 0x44, 0x38,  // 0x6F 'o'
  0x7C, 0x14, 0x14, 0x14, 0x08,  // 0x70 'p'
  0x08, 0x14, 0x14, 0x18, 0x7C,  // 0x71 'q'
  0x7C, 0x08, 0x04, 0x04, 0x08,  // 0x72 'r'
  0x48, 0x54, 0x54, 0x54, 0x20,  // 0x73 's'
  0x04, 0x3F, 0x44, 0x40, 0x20,  // 0x74 't'
  0x3C, 0x40, 0x40, 0x20, 0x7C,  // 0x75 'u'
  0x1C, 0x20, 0x40, 0x20, 0x1C,  // 0x76 'v'
  0x3C, 0x40, 0x30, 0x40, 0x3C,  // 0x77 'w'
  0x44, 0x28, 0x10, 0x28, 0x44,  // 0x78 'x'
  0x0C, 0x50, 0x50, 0x50, 0x3C,  // 0x79 'y'
  0x44, 0x64, 0x54, 0x4C, 0x44,  // 0x7A 'z'
  0x00, 0x08, 0x36, 0x41, 0x00,  // 0x7B '{'
  0x00, 0x00, 0x7F, 0x00, 0x00,  // 0x7C '|'
  0x00, 0x41, 0x36, 0x08, 0x00,  // 0x7D '}'
  0x08, 0x04, 0x08, 0x10, 0x08,  // 0x7E '~'
};

// Stream a single character cell (6 cols x 8 rows, scaled). Window must already
// be set to cover this cell.
static void streamCharGlyph(char c, uint16_t color, uint16_t bg, uint8_t scale) {
  if (c < 0x20 || c > 0x7E) c = '?';
  const uint8_t* glyph = &font5x7[(c - 0x20) * 5];
  for (uint8_t row = 0; row < 8; row++) {
    uint8_t mask = (row < 7) ? (1 << row) : 0;
    for (uint8_t sy = 0; sy < scale; sy++) {
      for (uint8_t col = 0; col < 6; col++) {
        uint8_t bits = (col < 5) ? pgm_read_byte(&glyph[col]) : 0;
        uint16_t pixel = (bits & mask) ? color : bg;
        for (uint8_t sx = 0; sx < scale; sx++) writePixel(pixel);
      }
    }
  }
}

static void drawChar(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale) {
  uint16_t w = 6 * scale;
  uint16_t h = 8 * scale;
  if (x < 0 || y < 0 || x + w > WIDTH || y + h > HEIGHT) return;
  setWindow(x, y, x + w - 1, y + h - 1);
  streamCharGlyph(c, color, bg, scale);
  digitalWrite(TFT_CS, HIGH);
}

static void drawTextCentered(int16_t y, const char* s, uint16_t color, uint16_t bg, uint8_t scale) {
  uint16_t n = strlen(s);
  uint16_t w = n * 6 * scale;
  if (w > WIDTH) w = WIDTH;
  int16_t x = (WIDTH - w) / 2;
  while (*s) {
    drawChar(x, y, *s++, color, bg, scale);
    x += 6 * scale;
  }
}

// ---- KXTJ3 ----

static const uint8_t KXTJ3_ADDR     = 0x0E;
static const uint8_t REG_CTRL1      = 0x1B;
static const uint8_t REG_CTRL2      = 0x1D;
static const uint8_t REG_INT_CTRL1  = 0x1E;
static const uint8_t REG_INT_CTRL2  = 0x1F;
static const uint8_t REG_DATA_CTRL  = 0x21;
static const uint8_t REG_XOUT_L     = 0x06;
static const uint8_t REG_INT_REL    = 0x1A;
static const uint8_t REG_WU_COUNTER = 0x29;
static const uint8_t REG_WUTH_H     = 0x6A;
static const uint8_t REG_WUTH_L     = 0x6B;

static inline void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(KXTJ3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static void setWakeupThreshold(uint16_t counts12) {
  counts12 &= 0x0FFF;
  write8(REG_WUTH_H, (counts12 >> 4) & 0xFF);
  write8(REG_WUTH_L, (counts12 & 0x0F) << 4);
}

static void clearLatchedInterrupt() {
  Wire.beginTransmission(KXTJ3_ADDR);
  Wire.write(REG_INT_REL);
  Wire.endTransmission(false);
  Wire.requestFrom(KXTJ3_ADDR, (uint8_t)1);
  (void)Wire.read();
}

static void enableWakeupInterrupt() {
  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_CTRL2, 0x04);
  write8(REG_WU_COUNTER, 2);
  setWakeupThreshold(64);
  write8(REG_INT_CTRL2, 0x3F);
  write8(REG_INT_CTRL1, 0x20);
  write8(REG_CTRL1, 0x82); delay(50);
  clearLatchedInterrupt();
}

static bool readXYZRaw(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(KXTJ3_ADDR);
  Wire.write(REG_XOUT_L);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(KXTJ3_ADDR, (uint8_t)6) != 6) return false;

  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  uint8_t zl = Wire.read(), zh = Wire.read();
  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
  return true;
}

static void calibrateKXTJ3(int16_t &xRef, int16_t &yRef, int16_t &zRef) {
  long sx = 0, sy = 0, sz = 0;
  uint8_t got = 0;
  for (uint8_t i = 0; i < 20; i++) {
    int16_t rx, ry, rz;
    if (readXYZRaw(rx, ry, rz)) { sx += rx; sy += ry; sz += rz; got++; }
    delay(10);
  }
  if (!got) return;
  xRef = (int16_t)(sx / got);
  yRef = (int16_t)(sy / got);
  zRef = (int16_t)(sz / got);
}

// ---- Tilt-to-bin ----

static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;
static constexpr int BIN_DIR_SIGN = -1;
static constexpr uint8_t MOUNT_NEUTRAL_BIN = 48;

static constexpr uint8_t LP_SHIFT = 3;
static constexpr int32_t MAG2_MIN = 24000000L;
static constexpr uint8_t HYSTERESIS_SHIFT = 8;

// ---- Sleep ----

static volatile uint32_t lastActivityMs = 0;

static void displayOff() {
  analogWrite(TFT_BL, 0);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
  gpio_hold_en((gpio_num_t)TFT_BL);

  cmdWithData(0x28, nullptr, 0);  // DISPOFF
  cmdWithData(0x10, nullptr, 0);  // SLPIN

  digitalWrite(TFT_RST, LOW);
  gpio_hold_en((gpio_num_t)TFT_RST);

  gpio_deep_sleep_hold_en();
}

// ---- Timeouts ----
static constexpr uint32_t DIM_TIMEOUT_MS   = 10000;
static constexpr uint32_t SLEEP_TIMEOUT_MS = 30000;

// ---- State ----
static uint8_t curFrame = 0;
static uint8_t curAbsBin = MOUNT_NEUTRAL_BIN;
static uint8_t currentBin = 0;
static uint32_t lastFrameTime = 0;
static uint16_t frameBuf[IMG_W * IMG_H];
static int16_t xRef = 0, yRef = 0, zRef = 0;
static int32_t fxLp = 0, fyLp = 0;

static uint8_t computeBin(int32_t fx, int32_t fy, uint8_t curBin) {
  uint8_t bestBin = 0;
  int32_t bestScore = INT32_MIN;
  int32_t curScore = INT32_MIN;
  for (uint8_t b = 0; b < 64; b++) {
    int32_t score = fx * (int32_t)COS64[b] + fy * (int32_t)SIN64[b];
    if (b == curBin) curScore = score;
    if (score > bestScore) {
      bestScore = score;
      bestBin = b;
    }
  }
  int32_t margin = curScore > 0 ? (curScore >> HYSTERESIS_SHIFT) : 0;
  return (bestScore > curScore + margin) ? bestBin : curBin;
}

// ---- BLE ----

static BLEServer* pServer = nullptr;
static BLECharacteristic* pStatusChar = nullptr;

static void setStatus(uint8_t s) {
  uploadStatus = s;
  if (pStatusChar) {
    uint8_t v = s;
    pStatusChar->setValue(&v, 1);
    pStatusChar->notify();
  }
}

static bool validateUpload() {
  if (uploadPos < 4) return false;
  if (uploadBuf[0] != 'P' || uploadBuf[1] != 'P') return false;
  if (uploadBuf[2] != 1) return false;
  uint8_t n = uploadBuf[3];
  if (n == 0 || n > MAX_UPLOADED_FRAMES) return false;
  uint32_t expected = 4 + 32 + (uint32_t)n * FRAME_REC;
  return uploadPos == expected;
}

// ---- Persistence (LittleFS) ----

static bool persistImage() {
  if (!LittleFS.begin(true)) return false;
  File f = LittleFS.open(IMG_PATH, "w");
  if (!f) return false;
  size_t n = f.write(uploadBuf, uploadPos);
  f.close();
  return n == uploadPos;
}

static bool clearPersistedImage() {
  if (!LittleFS.begin(true)) return false;
  if (!LittleFS.exists(IMG_PATH)) return true;
  return LittleFS.remove(IMG_PATH);
}

// Load /img.bin into uploadBuf and validate. Sets uploadedActive on success.
// Auto-removes the file on LOAD_INVALID so a corrupt blob doesn't persist
// across reboots.
static LoadResult loadPersistedImage() {
  if (!LittleFS.begin(true)) return LOAD_FS_FAIL;
  if (!LittleFS.exists(IMG_PATH)) return LOAD_NONE;
  File f = LittleFS.open(IMG_PATH, "r");
  if (!f) return LOAD_FS_FAIL;
  size_t size = f.size();
  if (size < 4 || size > UPLOAD_BUF_SIZE) {
    f.close();
    LittleFS.remove(IMG_PATH);
    return LOAD_INVALID;
  }
  size_t n = f.read(uploadBuf, size);
  f.close();
  if (n != size) return LOAD_FS_FAIL;
  uploadPos = size;
  if (!validateUpload()) {
    uploadPos = 0;
    LittleFS.remove(IMG_PATH);
    return LOAD_INVALID;
  }
  uploadedFrameCount = uploadBuf[3];
  uploadedActive = true;
  return LOAD_OK;
}

// Brief full-screen splash with a text label. Color-coded background:
//   red    = file existed but was corrupt (was auto-removed)
//   orange = filesystem mount or read failure (no auto-recovery available)
static void showLoadError(LoadResult r) {
  uint16_t bg;
  const char* line1;
  const char* line2;
  switch (r) {
    case LOAD_INVALID:
      bg = 0xF800;       // red
      line1 = "BAD";
      line2 = "FILE";
      break;
    case LOAD_FS_FAIL:
      bg = 0xFD20;       // orange
      line1 = "FS";
      line2 = "ERROR";
      break;
    default: return;
  }
  fillScreen(bg);
  // Two lines, scale 4 (char cell 24x32, glyph 20x28). Centered vertically
  // with a 12px gap between lines. Total height = 32 + 12 + 32 = 76.
  const uint8_t scale = 4;
  const int16_t lineH = 8 * scale;     // 32
  const int16_t gap   = 12;
  const int16_t total = lineH * 2 + gap;
  const int16_t y0    = (HEIGHT - total) / 2;
  drawTextCentered(y0,                  line1, 0xFFFF, bg, scale);
  drawTextCentered(y0 + lineH + gap,    line2, 0xFFFF, bg, scale);
  delay(2000);
  fillScreen(0x0000);
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    uploadPos = 0;
    setStatus(STATUS_IDLE);
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

class UploadCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();
    size_t len = v.length();
    if (len == 0) return;
    const uint8_t* data = (const uint8_t*)v.c_str();
    if (uploadPos + len > UPLOAD_BUF_SIZE) {
      setStatus(STATUS_ERROR);
      return;
    }
    memcpy(&uploadBuf[uploadPos], data, len);
    uploadPos += len;
    setStatus(STATUS_RECEIVING);
  }
};

class CommitCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();
    if (v.length() < 1) return;
    uint8_t cmd = ((const uint8_t*)v.c_str())[0];
    switch (cmd) {
      case 0x01:  // commit
        if (validateUpload()) {
          uploadedFrameCount = uploadBuf[3];
          uploadedActive = true;
          uploadCommittedFlag = true;
          persistRequested = true;
          setStatus(STATUS_SUCCESS);
        } else {
          setStatus(STATUS_ERROR);
        }
        break;
      case 0x02:  // reset upload buffer
        uploadPos = 0;
        setStatus(STATUS_IDLE);
        break;
      case 0x03:  // revert to default
        uploadedActive = false;
        uploadCommittedFlag = true;
        clearPersistRequested = true;
        setStatus(STATUS_IDLE);
        break;
    }
  }
};

static void bleSetup() {
  BLEDevice::init(BLE_DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  BLECharacteristic* pUpload = pService->createCharacteristic(
    BLE_UPLOAD_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pUpload->setCallbacks(new UploadCB());

  BLECharacteristic* pCommit = pService->createCharacteristic(
    BLE_COMMIT_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCommit->setCallbacks(new CommitCB());

  pStatusChar = pService->createCharacteristic(
    BLE_STATUS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pStatusChar->addDescriptor(new BLE2902());
  uint8_t s0 = STATUS_IDLE;
  pStatusChar->setValue(&s0, 1);

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void setup() {
  gpio_hold_dis((gpio_num_t)TFT_RST);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  digitalWrite(TFT_RST, HIGH); delay(10);
  digitalWrite(TFT_RST, LOW);  delay(10);
  digitalWrite(TFT_RST, HIGH); delay(120);

  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

  gc9a01Init();

  gpio_hold_dis((gpio_num_t)TFT_BL);
  pinMode(TFT_BL, OUTPUT);
  analogWrite(TFT_BL, 255);

  fillScreen(0x0000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  write8(REG_CTRL1, 0xC0); delay(50);

  calibrateKXTJ3(xRef, yRef, zRef);

  enableWakeupInterrupt();
  esp_deep_sleep_enable_gpio_wakeup(BIT(KXTJ3_INT_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);

  curFrame = 0;
  curAbsBin = MOUNT_NEUTRAL_BIN;
  currentBin = 0;
  fxLp = (int32_t)xRef * ACCEL_X_SIGN;
  fyLp = (int32_t)yRef * ACCEL_Y_SIGN;
  lastFrameTime = millis();
  lastActivityMs = millis();

  // Restore persisted image (if present and valid). Falls back to default.
  // Surfaces filesystem / corruption errors via a colored splash.
  LoadResult lr = loadPersistedImage();
  showLoadError(lr);

  bleSetup();

  composeFrame(curFrame, frameBuf);
  drawFrameRotated(frameBuf, currentBin);
}

void loop() {
  uint32_t now = millis();

  // Suppress sleep timer while a host is connected or a transfer is mid-flight.
  if (bleConnected || uploadStatus == STATUS_RECEIVING) {
    lastActivityMs = now;
  }
  uint32_t elapsed = now - lastActivityMs;

  if (elapsed >= SLEEP_TIMEOUT_MS) {
    displayOff();
    clearLatchedInterrupt();
    esp_deep_sleep_start();
  }

  analogWrite(TFT_BL, (elapsed >= DIM_TIMEOUT_MS) ? 40 : 255);

  // Apply pending image source switch (commit or revert).
  if (uploadCommittedFlag) {
    uploadCommittedFlag = false;
    curFrame = 0;
    lastFrameTime = now;
    composeFrame(curFrame, frameBuf);
    drawFrameRotated(frameBuf, currentBin);
  }

  // Apply pending persistence operations (deferred from BLE callbacks so the
  // BLE task isn't blocked by Flash writes).
  if (persistRequested) {
    persistRequested = false;
    persistImage();
  }
  if (clearPersistRequested) {
    clearPersistRequested = false;
    clearPersistedImage();
  }

  // Update lowpass on raw gravity vector
  int16_t rx, ry, rz;
  if (readXYZRaw(rx, ry, rz)) {
    int32_t fx = (int32_t)rx * ACCEL_X_SIGN;
    int32_t fy = (int32_t)ry * ACCEL_Y_SIGN;
    fxLp = (fxLp * ((1 << LP_SHIFT) - 1) + fx) >> LP_SHIFT;
    fyLp = (fyLp * ((1 << LP_SHIFT) - 1) + fy) >> LP_SHIFT;
  }

  int32_t mag2 = fxLp * fxLp + fyLp * fyLp;
  if (mag2 > MAG2_MIN) {
    curAbsBin = computeBin(fxLp, fyLp, curAbsBin);
  }
  int8_t diff = (int8_t)curAbsBin - (int8_t)MOUNT_NEUTRAL_BIN;
  diff *= BIN_DIR_SIGN;
  uint8_t newBin = (uint8_t)diff & 0x3F;

  bool binChanged = (newBin != currentBin);
  if (binChanged) {
    currentBin = newBin;
    lastActivityMs = now;
  }

  uint16_t duration = getDuration(curFrame);
  bool frameAdvance = (now - lastFrameTime >= duration);
  if (frameAdvance) {
    lastFrameTime = now;
    curFrame = getNextFrame(curFrame);
  }

  if (frameAdvance || binChanged) {
    composeFrame(curFrame, frameBuf);
    drawFrameRotated(frameBuf, currentBin);
  }
}
