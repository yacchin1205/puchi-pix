// Puchi-Pix ESP32 (ESPr Developer C3): GC9A01 free-rotation viewer
// with BLE image upload
// 64-step (5.625deg) rotation following gravity tilt via KXTJ3.
// A BLE peripheral accepts a custom multi-frame image payload
// (GIF-converted) and swaps the active animation.
//
// BLE protocol:
//   Service:  5550EEC9-B00F-4444-9876-AB12CD34EF01
//   Upload  (Write):       5550EEC9-B00F-4444-9876-AB12CD34EF02
//   Commit  (Write):       5550EEC9-B00F-4444-9876-AB12CD34EF03
//   Status  (Read/Notify): 5550EEC9-B00F-4444-9876-AB12CD34EF04
//   Proto   (Read):        5550EEC9-B00F-4444-9876-AB12CD34EF06
//
//   Upload: chunked writes appended in order to a 32 KB buffer.
//   Commit: 0x01 = validate & swap, 0x02 = buffer reset, 0x03 = revert default.
//   Status: 1 byte (0=idle, 1=receiving, 2=success, 3=error).
//   Proto: 4 bytes [payloadVerMax, payloadVerMin, legacyMotionCfgLen,
//     reserved]. Read by the uploader on connect so a web/firmware protocol
//     mismatch is reported explicitly instead of failing as a generic
//     upload error. legacyMotionCfgLen is 0: since v6 the motion config is
//     embedded in the payload and the Motion characteristic is gone.
//   Motion block (16 bytes, embedded in v6 payloads):
//     [enabled, mainStart lo/hi, loopStart lo/hi, shakeStart lo/hi,
//      outStart lo/hi, exitStart lo/hi, speed, gap lo/hi, bg565 lo/hi].
//     Splits the animation into up to six segments:
//       entry walk  [0, mainStart)         loops while walking in
//       intro trans [mainStart, loopStart) plays once after arriving
//       main loop   [loopStart, shakeStart | outStart | exitStart | N)
//                                          loops while handling is detected
//       shake       [shakeStart, outStart | exitStart | N)
//                                          plays once when the device is
//                                          shaken during the main loop,
//                                          then the main loop resumes
//       outro trans [outStart, exitStart | N) plays once before leaving
//       exit walk   [exitStart, N)         loops while walking out
//     Sentinels: mainStart = 0 -> no entry walk (the main loop itself plays
//     while walking in/out; intro is forced off). loopStart <= mainStart ->
//     no intro. shakeStart = 0 -> no shake segment. outStart = 0 -> no
//     outro. exitStart = 0 -> exit reuses the entry walk frames (or the
//     main loop when there is no entry).
//     When enabled, the sprite walks in from the right translating at
//     `speed` source px/s, then the screen stays empty for `gap` ms after
//     the exit before the next entry. bg565 fills everything outside the
//     sprite (RGB565), matching the uploader's compositing background.
//     Because the block lives inside the payload, image and presentation
//     are committed, validated and persisted atomically (/img.bin only).
//
// Payload (sent via Upload, raw bytes appended in order):
//   v1 (64x64 fixed):
//     [0..1]  Magic 'PP' (0x50 0x50)
//     [2]     Version = 1
//     [3]     Frame count N (1..16)
//     [4..35] Palette: 16 x uint16 RGB565, little-endian (32 B)
//     [36..]  Per frame: uint16 duration_ms (LE) + data[2048] (4-bit packed)
//   v2 (64x64 or 128x128, 16 or 256 colors):
//     [0..1]  Magic 'PP'
//     [2]     Version = 2
//     [3]     Frame count N
//     [4]     Image width  (64 or 128)
//     [5]     Image height (must equal width)
//     [6]     Bits per pixel: 0 or 4 = 16-color, 8 = 256-color
//     [7]     Reserved (0)
//     [8..]   Palette: 16 (4bpp, 32 B) or 256 (8bpp, 512 B) x uint16 RGB565 LE
//     then    Per frame: uint16 duration_ms (LE) + data[W*H*bpp/8]
//             (4bpp: packed 2px/byte, even idx = high nibble; 8bpp: 1px/byte)
//   v3 (diff chain; 64x64, 120x120 or 128x128, 16 or 256 colors):
//     [0..1]  Magic 'PP'
//     [2]     Version = 3
//     [3..4]  Frame count N (uint16 LE, >= 1)
//     [5]     Image size (64, 120 or 128, square)
//     [6]     Bits per pixel: 4 = 16-color, 8 = 256-color
//     [7]     Reserved (0)
//     [8..]   Palette: 16 (4bpp, 32 B) or 256 (8bpp, 512 B) x uint16 RGB565 LE
//     then    Per frame: uint16 duration_ms (LE) + uint8 flags (bit0 = keyframe)
//             keyframe: data[W*H*bpp/8] (full frame)
//             diff:     uint8 rectCount + rectCount x
//                       { rx, ry, rw, rh (uint8) + data[ceil(rw*rh*bpp/8)] }
//             Diffs apply onto the previously composed frame; frame 0 must be
//             a keyframe. Playback is sequential and wraps to frame 0.
//   v4 (RLE diff chain): identical to v3 except every pixel-data block is
//     PackBits RLE compressed and prefixed with its uint16 LE byte length:
//     keyframe: uint16 rleLen + rle[rleLen]   (decodes to W*H*bpp/8 bytes)
//     rect:     rx, ry, rw, rh, uint16 rleLen + rle[rleLen]
//               (decodes to ceil(rw*rh*bpp/8) bytes)
//     PackBits: control c < 128 = copy next c+1 bytes verbatim; c > 128 =
//     repeat next byte (257 - c) times; c == 128 = no-op. 4bpp data is RLE'd
//     on the packed bytes.
//   v6: v5 with a 16-byte motion block between the header and the palette
//     (palette starts at byte 24); see "Motion block" above. Validated as
//     part of commit; v3-v5 payloads simply play as a plain loop.
//   v5: v4 plus referenced-diff records (flags bit1). Instead of diffing
//     against the previous frame, the rects patch a copy of an arbitrary
//     EARLIER frame; an exact repeat is simply a referenced diff with zero
//     rects:
//       uint16 duration + flags(0x02) + uint16 refFrame LE + uint8 rectCount
//       + rects (v4 layout)
//     To keep decode single-level, refFrame must itself be composable
//     without touching any referenced-diff record (its replay window from
//     the nearest keyframe contains none); validation enforces this and the
//     uploader only picks refs satisfying it.
//   v1/v2 frame count is additionally bounded by the upload buffer (98368 B):
//   64x64 -> 16 frames either depth; 128x128 -> 12 (16-color) / 5 (256-color).
//   v3/v4 have no fixed frame cap; the payload just has to fit the buffer.
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
#define BLE_PROTO_UUID        "5550eec9-b00f-4444-9876-ab12cd34ef06"

static constexpr uint8_t PAYLOAD_VER_MIN = 1;
static constexpr uint8_t PAYLOAD_VER_MAX = 6;

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

// PackBits reader emitting one decoded byte per call (v4 payloads). The
// stream is validated up front, so no bounds checks during decode. Declared
// up here for the same auto-prototype reason as LoadResult.
struct RleReader {
  const uint8_t* p;
  uint8_t runByte;
  uint8_t runLeft;
  uint8_t litLeft;
};

// Motion (walk-in/out) configuration; see the Motion characteristic notes in
// the header comment. Declared up here for the auto-prototype generator.
struct MotionConfig {
  uint8_t  enabled;    // 0 = plain in-place loop (legacy behavior)
  uint16_t mainStart;  // end of entry walk = start of intro; 0 = no entry
  uint16_t loopStart;  // first frame of the main loop; <= mainStart = no intro
  uint16_t shakeStart; // first frame of the shake segment; 0 = none
  uint16_t outStart;   // first frame of the outro transition; 0 = none
  uint16_t exitStart;  // first frame of the exit walk; 0 = reuse entry
  uint8_t  speed;      // traverse speed, source px/s
  uint16_t gapMs;      // empty-screen time after exit before re-entry
  uint16_t bg;         // RGB565 fill outside the sprite / for empty screen
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

// Status LED D1: 3.3V -> 4.7k -> LED -> GPIO0, so LOW = lit. Shows an
// active BLE connection.
static constexpr uint8_t STATUS_LED = 0;

static constexpr int WIDTH  = 240;
static constexpr int HEIGHT = 240;

// ---- Default image data (PROGMEM fallback) ----
#include "../puchi_pix/frame.h"
#include "../puchi_pix/icon_original.h"

static constexpr uint16_t MAX_IMG_SIZE = 128;

// 12 frames of 128x128 (or 16 of 64x64) + v2 header. Kept below the full
// 16x128x128 (131 KB) so enough heap remains for the Bluedroid BLE stack.
static constexpr size_t UPLOAD_BUF_SIZE = 8 + 32 + 12u * ((size_t)MAX_IMG_SIZE * MAX_IMG_SIZE / 2 + 2);  // 98368

// All sources are pixel-doubled (shift 1).
// 64x64  -> 128x128 visible; rotated corners sweep a 181px diagonal, so the
//           output window is 192x192 (black outside the sprite erases trails).
// 120x120 -> 240x240 visible, exactly filling the window. The 240px disc is
//           the square's inscribed circle (radius 120 = half-side) at every
//           rotation angle, so nothing is cropped and no black corners appear.
// 128x128 -> 256x256 visible, cropped to the full 240x240 window. Its inscribed
//           circle (256px) always covers the 240px disc, so no black corners
//           and every visible pixel maps inside the source (radius 120/2 = 60 < 64).
static constexpr uint8_t SCALE_SHIFT = 1;

// Active source dimensions (updated when the image source switches)
static uint8_t srcSize  = IMG_W;  // 64, 120 or 128
static int16_t outSize  = 192;    // output window: 192 (64src) or 240 (120/128src)

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

// Shared row buffer for bulk transfers. SPI.writePixels() sends each 16-bit
// value MSB-first (same wire order as writePixel), so pixels are stored in
// native endianness. One row of the full panel width is the largest unit.
static uint16_t txRowBuf[WIDTH];

static inline uint8_t read4bit(const uint8_t* data, uint16_t idx) {
  uint8_t b = pgm_read_byte(&data[idx >> 1]);
  return (idx & 1) ? (b & 0x0F) : (b >> 4);
}

static inline uint8_t read4bitRam(const uint8_t* data, uint32_t idx) {
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
static volatile uint16_t uploadedFrameCount = 0;

// Deferred persistence flags (set in BLE callbacks, applied in loop()).
static volatile bool persistRequested = false;
static volatile bool clearPersistRequested = false;

#define IMG_PATH "/img.bin"

// Layout of the validated upload (set by validateUpload)
static uint8_t  upVer      = 2;     // payload version (1, 2 or 3)
static uint8_t  upSize     = 64;    // image dimension (square)
static uint8_t  upBpp      = 4;     // bits per pixel (4 or 8)
static uint32_t upPalOff   = 4;     // palette byte offset
static uint32_t upDataOff  = 36;    // first frame record byte offset
static uint32_t upFrameRec = 2050;  // v1/v2: bytes per frame record (duration + data)
static uint32_t upCurRec   = 0;     // v3: byte offset of the current frame's record

// Accessors over uploadBuf
static inline const uint16_t* uploadedPalette() {
  return (const uint16_t*)&uploadBuf[upPalOff];
}
static inline const uint8_t* uploadedFrameData(uint8_t idx) {
  return &uploadBuf[upDataOff + (uint32_t)idx * upFrameRec + 2];
}
static inline uint16_t uploadedFrameDuration(uint8_t idx) {
  const uint8_t* p = &uploadBuf[upDataOff + (uint32_t)idx * upFrameRec];
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ---- Motion (walk-in / walk-out) config ----
// Set over BLE, persisted to /motion.bin, cleared whenever the image source
// changes (frame indices are payload-specific).
static MotionConfig motionCfg = { 0, 1, 0, 0, 0, 0, 60, 1000, 0x0000 };

static constexpr uint32_t MOTION_BLOCK_OFF = 8;
static constexpr uint32_t MOTION_BLOCK_LEN = 16;

static inline bool motionOn() { return motionCfg.enabled && uploadedActive; }

// Point the renderer at the active source's dimensions. Motion needs the full
// 240px window regardless of source size so the traverse path isn't cropped.
static void applyActiveSource() {
  srcSize = uploadedActive ? upSize : IMG_W;
  outSize = (srcSize >= 120 || motionOn()) ? 240 : 192;
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

static void composeFrameFromUpload(uint16_t frameIdx, uint16_t* buf) {
  if (frameIdx >= uploadedFrameCount) frameIdx = 0;
  const uint16_t* pal = uploadedPalette();
  const uint8_t* data = uploadedFrameData(frameIdx);
  uint32_t n = (uint32_t)upSize * upSize;
  if (upBpp == 8) {
    for (uint32_t i = 0; i < n; i++) buf[i] = pal[data[i]];
  } else {
    for (uint32_t i = 0; i < n; i++) buf[i] = pal[read4bitRam(data, i)];
  }
}

static void composeFrame(uint16_t frameIdx, uint16_t* buf) {
  if (uploadedActive) composeFrameFromUpload(frameIdx, buf);
  else                composeFrameFromProgmem((uint8_t)frameIdx, buf);
}

// ---- v3/v4 (diff chain) record helpers ----
// Record offsets passed here are trusted: validateUpload() walks the full
// record chain on commit and rejects any out-of-bounds rect, bad RLE stream
// or truncation.

static inline uint32_t chainFrameBytes() {
  return (uint32_t)upSize * upSize * upBpp / 8;
}

static inline uint32_t chainRectDataBytes(uint8_t rw, uint8_t rh) {
  uint32_t px = (uint32_t)rw * rh;
  return (upBpp == 8) ? px : ((px + 1) >> 1);
}

static inline uint16_t rdU16(uint32_t off) {
  return (uint16_t)uploadBuf[off] | ((uint16_t)uploadBuf[off + 1] << 8);
}

static inline void rleInit(RleReader& r, const uint8_t* p) {
  r.p = p;
  r.runLeft = 0;
  r.litLeft = 0;
}

static uint8_t rleNextByte(RleReader& r) {
  if (r.runLeft) { r.runLeft--; return r.runByte; }
  if (r.litLeft) { r.litLeft--; return *r.p++; }
  for (;;) {
    uint8_t c = *r.p++;
    if (c < 128) { r.litLeft = c; return *r.p++; }
    if (c > 128) {
      r.runByte = *r.p++;
      r.runLeft = (uint8_t)(256 - c);   // total 257-c, one emitted now
      return r.runByte;
    }
    // c == 128: no-op
  }
}

// Verify a PackBits stream of `len` bytes decodes to exactly `expect` bytes.
static bool rleCheck(const uint8_t* p, uint32_t len, uint32_t expect) {
  const uint8_t* end = p + len;
  uint32_t out = 0;
  while (p < end) {
    uint8_t c = *p++;
    if (c < 128) {
      uint32_t n = (uint32_t)c + 1;
      if (p + n > end) return false;
      p += n;
      out += n;
    } else if (c > 128) {
      if (p >= end) return false;
      p++;
      out += 257 - c;
    }
  }
  return out == expect;
}

static uint32_t chainNextRecord(uint32_t off) {
  if (upVer >= 5 && (uploadBuf[off + 2] & 0x02)) {   // referenced diff
    uint8_t rectCount = uploadBuf[off + 5];
    uint32_t p = off + 6;
    for (uint8_t r = 0; r < rectCount; r++)
      p += 6 + rdU16(p + 4);
    return p;
  }
  if (upVer >= 4) {
    if (uploadBuf[off + 2] & 0x01) return off + 5 + rdU16(off + 3);
    uint8_t rectCount = uploadBuf[off + 3];
    uint32_t p = off + 4;
    for (uint8_t r = 0; r < rectCount; r++)
      p += 6 + rdU16(p + 4);
    return p;
  }
  if (uploadBuf[off + 2] & 0x01) return off + 3 + chainFrameBytes();
  uint8_t rectCount = uploadBuf[off + 3];
  uint32_t p = off + 4;
  for (uint8_t r = 0; r < rectCount; r++)
    p += 4 + chainRectDataBytes(uploadBuf[p + 2], uploadBuf[p + 3]);
  return p;
}

static uint16_t chainDuration(uint32_t off) {
  return rdU16(off);
}

// Apply `rectCount` RLE rect patches starting at byte offset p onto buf.
static void applyRleRects(uint32_t p, uint8_t rectCount, uint16_t* buf) {
  const uint16_t* pal = uploadedPalette();
  for (uint8_t rc = 0; rc < rectCount; rc++) {
    uint8_t rx = uploadBuf[p], ry = uploadBuf[p + 1];
    uint8_t rw = uploadBuf[p + 2], rh = uploadBuf[p + 3];
    uint16_t rl = rdU16(p + 4);
    RleReader r;
    rleInit(r, &uploadBuf[p + 6]);
    uint32_t i = 0;
    uint8_t b = 0;
    for (uint8_t dy = 0; dy < rh; dy++) {
      uint16_t* row = &buf[(uint32_t)(ry + dy) * upSize + rx];
      for (uint8_t dx = 0; dx < rw; dx++, i++) {
        if (upBpp == 8) row[dx] = pal[rleNextByte(r)];
        else {
          if (!(i & 1)) b = rleNextByte(r);
          row[dx] = pal[(i & 1) ? (b & 0x0F) : (b >> 4)];
        }
      }
    }
    p += 6 + rl;
  }
}

// Keyframe: full decode into buf. Diff: rect patches applied over buf, which
// must still hold the previously composed frame. A v5 referenced diff first
// composes its (validated ref-free) source frame, so it nests at most one
// composeChainInto level. v4 reads pixel data through the RLE reader; v3
// reads it raw.
static void chainDecode(uint32_t off, uint16_t* buf) {
  const uint16_t* pal = uploadedPalette();
  const bool rle = (upVer >= 4);
  if (upVer >= 5 && (uploadBuf[off + 2] & 0x02)) {
    composeChainInto(rdU16(off + 3), buf);
    applyRleRects(off + 6, uploadBuf[off + 5], buf);
    return;
  }
  if (uploadBuf[off + 2] & 0x01) {
    uint32_t n = (uint32_t)upSize * upSize;
    if (rle) {
      RleReader r;
      rleInit(r, &uploadBuf[off + 5]);
      uint8_t b = 0;
      for (uint32_t i = 0; i < n; i++) {
        if (upBpp == 8) buf[i] = pal[rleNextByte(r)];
        else {
          if (!(i & 1)) b = rleNextByte(r);
          buf[i] = pal[(i & 1) ? (b & 0x0F) : (b >> 4)];
        }
      }
    } else {
      const uint8_t* data = &uploadBuf[off + 3];
      if (upBpp == 8) {
        for (uint32_t i = 0; i < n; i++) buf[i] = pal[data[i]];
      } else {
        for (uint32_t i = 0; i < n; i++) buf[i] = pal[read4bitRam(data, i)];
      }
    }
    return;
  }
  uint8_t rectCount = uploadBuf[off + 3];
  uint32_t p = off + 4;
  for (uint8_t rc = 0; rc < rectCount; rc++) {
    uint8_t rx = uploadBuf[p], ry = uploadBuf[p + 1];
    uint8_t rw = uploadBuf[p + 2], rh = uploadBuf[p + 3];
    const uint8_t* data = &uploadBuf[p + (rle ? 6 : 4)];
    RleReader r;
    if (rle) rleInit(r, data);
    uint32_t i = 0;
    uint8_t b = 0;
    for (uint8_t dy = 0; dy < rh; dy++) {
      uint16_t* row = &buf[(uint32_t)(ry + dy) * upSize + rx];
      for (uint8_t dx = 0; dx < rw; dx++, i++) {
        uint8_t v;
        if (rle) {
          if (upBpp == 8) v = rleNextByte(r);
          else {
            if (!(i & 1)) b = rleNextByte(r);
            v = (i & 1) ? (b & 0x0F) : (b >> 4);
          }
        } else {
          v = (upBpp == 8) ? data[i] : read4bitRam(data, i);
        }
        row[dx] = pal[v];
      }
    }
    p += rle ? (6 + rdU16(p + 4)) : (4 + chainRectDataBytes(rw, rh));
  }
}

// Frame metadata accessors (dispatch source).
static uint16_t getDuration(uint16_t idx) {
  if (uploadedActive) {
    if (upVer >= 3) return chainDuration(upCurRec);
    if (idx >= uploadedFrameCount) idx = 0;
    return uploadedFrameDuration(idx);
  }
  return pgm_read_word(&frames[idx].duration_ms);
}

static uint16_t getNextFrame(uint16_t idx) {
  if (uploadedActive) {
    uint16_t n = uploadedFrameCount;
    if (n == 0) return 0;
    return (uint16_t)((idx + 1) % n);
  }
  return pgm_read_byte(&frames[idx].next);
}

// Reverse-rotate source buffer onto a centered OUT_SIZE x OUT_SIZE region.
// walkShift displaces the sprite along its own horizontal axis (source px,
// positive = right); the axis rotates with the tilt bins, so the sprite
// always walks perpendicular to gravity.
static void drawFrameRotated(const uint16_t* src, uint8_t bin, int16_t walkShift) {
  const int32_t cosT = COS64[bin & 0x3F];
  const int32_t sinT = SIN64[bin & 0x3F];

  const int16_t outHalf = outSize / 2;
  const uint16_t ox0 = (WIDTH - outSize) / 2;
  const uint16_t oy0 = (HEIGHT - outSize) / 2;
  setWindow(ox0, oy0, ox0 + outSize - 1, oy0 + outSize - 1);

  const int8_t SHIFT = 15 + SCALE_SHIFT;
  const int16_t srcHalf = srcSize / 2;
  const int16_t srcCX = srcHalf - walkShift;
  const uint16_t sz = srcSize;

  for (int16_t oy = 0; oy < outSize; oy++) {
    int32_t cy = oy - outHalf;
    int32_t cySin = cy * sinT;
    int32_t cyCos = cy * cosT;
    for (int16_t ox = 0; ox < outSize; ox++) {
      int32_t cx = ox - outHalf;
      int16_t sx = (int16_t)(((cx * cosT) + cySin) >> SHIFT) + srcCX;
      int16_t sy = (int16_t)(((-cx * sinT) + cyCos) >> SHIFT) + srcHalf;
      uint16_t c;
      if ((uint16_t)sx < sz && (uint16_t)sy < sz) {
        c = src[(uint32_t)sy * sz + sx];
      } else {
        c = motionCfg.bg;
      }
      txRowBuf[ox] = c;
    }
    SPI.writePixels(txRowBuf, (uint32_t)outSize * 2);
  }
  digitalWrite(TFT_CS, HIGH);
}

static void fillScreen(uint16_t color) {
  setWindow(0, 0, WIDTH - 1, HEIGHT - 1);
  for (int16_t i = 0; i < WIDTH; i++) txRowBuf[i] = color;
  for (int16_t y = 0; y < HEIGHT; y++)
    SPI.writePixels(txRowBuf, (uint32_t)WIDTH * 2);
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

// Board is mounted flipped (component side reversed): X mirrors, Z unused.
static constexpr int ACCEL_X_SIGN = -1;
static constexpr int ACCEL_Y_SIGN = 1;
static constexpr int BIN_DIR_SIGN = -1;
static constexpr uint8_t MOUNT_NEUTRAL_BIN = 0;

// Gravity lowpass time constant. Loop period varies widely (sub-ms idle,
// tens of ms during a redraw), so the filter scales its blend factor by
// elapsed time instead of using a fixed per-sample weight. 200 ms attenuates
// hand tremor and linear-acceleration wobble that the old ~10 ms-equivalent
// EMA passed straight through.
static constexpr int32_t LP_TAU_MS = 200;
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
static uint16_t curFrame = 0;
static uint8_t curAbsBin = MOUNT_NEUTRAL_BIN;
static uint8_t currentBin = 0;
static uint32_t lastFrameTime = 0;
static uint16_t frameBuf[(uint32_t)MAX_IMG_SIZE * MAX_IMG_SIZE];
static int16_t xRef = 0, yRef = 0, zRef = 0;
static int32_t fxLp = 0, fyLp = 0;
static uint32_t lastLpUpdateMs = 0;

// Reset playback to frame 0 and compose it into frameBuf.
static void animReset() {
  curFrame = 0;
  if (uploadedActive && upVer >= 3) {
    upCurRec = upDataOff;
    chainDecode(upCurRec, frameBuf);
  } else {
    composeFrame(0, frameBuf);
  }
}

// Advance to the next frame. frameBuf always holds the current composed
// frame, so v3 only patches the diff rects; other sources recompose fully.
static void animAdvance() {
  if (uploadedActive && upVer >= 3) {
    if ((uint16_t)(curFrame + 1) < uploadedFrameCount) {
      curFrame++;
      upCurRec = chainNextRecord(upCurRec);
    } else {
      curFrame = 0;
      upCurRec = upDataOff;
    }
    chainDecode(upCurRec, frameBuf);
  } else {
    curFrame = getNextFrame(curFrame);
    composeFrame(curFrame, frameBuf);
  }
}

// Compose an arbitrary frame into frameBuf. For v3 this replays the diff
// chain from the nearest keyframe at or before the target (frame 0 is always
// a keyframe, so this terminates). Walk segments are short, so the replay on
// a segment jump costs a few small decodes at most.
// Compose `target` into buf by replaying the chain from the nearest
// keyframe. Does not touch playback state, so referenced-diff decoding can
// use it for its source frame mid-replay.
static void composeChainInto(uint16_t target, uint16_t* buf) {
  uint32_t off = upDataOff;
  uint32_t keyOff = upDataOff;
  uint16_t keyIdx = 0;
  for (uint16_t i = 1; i <= target; i++) {
    off = chainNextRecord(off);
    if (uploadBuf[off + 2] & 0x01) { keyOff = off; keyIdx = i; }
  }
  uint32_t p = keyOff;
  chainDecode(p, buf);
  for (uint16_t i = keyIdx; i < target; i++) {
    p = chainNextRecord(p);
    chainDecode(p, buf);
  }
}

static void animJumpTo(uint16_t target) {
  if (target >= uploadedFrameCount) target = 0;
  if (uploadedActive && upVer >= 3) {
    composeChainInto(target, frameBuf);
    uint32_t off = upDataOff;
    for (uint16_t i = 0; i < target; i++) off = chainNextRecord(off);
    upCurRec = off;
  } else {
    composeFrame(target, frameBuf);
  }
  curFrame = target;
}

// Advance within [first, last] (inclusive), wrapping back to first.
static void animAdvanceRange(uint16_t first, uint16_t last) {
  if (curFrame >= last) {
    animJumpTo(first);
    return;
  }
  if (uploadedActive && upVer >= 3) {
    curFrame++;
    upCurRec = chainNextRecord(upCurRec);
    chainDecode(upCurRec, frameBuf);
  } else {
    curFrame++;
    composeFrame(curFrame, frameBuf);
  }
}

// ---- Motion (walk-in / walk-out) runtime ----

// Device-handling detection: raw accel deviating from the 200 ms lowpass by
// more than ~0.04 g (600 counts at 16384/g) on either axis counts as
// handling; it is considered ongoing for HANDLE_HOLD_MS after the last hit.
static constexpr int32_t HANDLE_THRESH = 600;
static constexpr uint32_t HANDLE_HOLD_MS = 2000;
static uint32_t lastHandledMs = 0;

// Shake detection: a much larger deviation (~0.5 g) than plain handling.
// Only shakes seen while the main loop is on screen arm the shake segment,
// so a vigorous pickup doesn't fire it on entry.
static constexpr int32_t SHAKE_THRESH = 8000;
static bool shakePending = false;

enum MotionPhase : uint8_t {
  PH_GAP, PH_ENTER, PH_TRANS_IN, PH_MAIN, PH_SHAKE, PH_TRANS_OUT, PH_EXIT
};
static MotionPhase motionPhase = PH_GAP;
static int32_t motionPosMpx = 0;    // sprite offset, milli-source-px, + = right
static uint32_t motionLastMs = 0;   // last position integration time
static uint32_t gapReadyMs = 0;     // earliest allowed re-entry
static int16_t lastWalkShift = 0;

static inline bool handledRecently(uint32_t now) {
  return (uint32_t)(now - lastHandledMs) < HANDLE_HOLD_MS;
}

// Offset at which the sprite is fully outside the output window even when
// rotated 45deg (source half-diagonal ~= srcSize * 1.5 / 2 on screen).
static inline int16_t motionMaxShift() {
  return (int16_t)((outSize / 2 + (int16_t)srcSize * 3 / 2) / 2 + 1);
}

// Segment boundaries derived from the config (all inclusive last indices).
// See the header comment for the five-segment layout and its sentinels.
static inline bool hasEntrySeg() { return motionCfg.mainStart > 0; }
static inline uint16_t loopFirstFrame() {
  return (motionCfg.loopStart > motionCfg.mainStart) ? motionCfg.loopStart
                                                     : motionCfg.mainStart;
}
static inline bool hasIntroSeg() { return loopFirstFrame() > motionCfg.mainStart; }
static inline bool hasShakeSeg() { return motionCfg.shakeStart > 0; }
static inline uint16_t loopLastFrame() {
  if (motionCfg.shakeStart) return motionCfg.shakeStart - 1;
  if (motionCfg.outStart) return motionCfg.outStart - 1;
  return motionCfg.exitStart ? (motionCfg.exitStart - 1)
                             : (uploadedFrameCount - 1);
}
static inline uint16_t shakeLastFrame() {
  if (motionCfg.outStart) return motionCfg.outStart - 1;
  return motionCfg.exitStart ? (motionCfg.exitStart - 1)
                             : (uploadedFrameCount - 1);
}
static inline uint16_t outLastFrame() {
  return motionCfg.exitStart ? (motionCfg.exitStart - 1)
                             : (uploadedFrameCount - 1);
}
static inline uint16_t enterLastFrame() {
  return hasEntrySeg() ? (motionCfg.mainStart - 1) : loopLastFrame();
}
static inline uint16_t exitFirstFrame() {
  return motionCfg.exitStart ? motionCfg.exitStart : 0;
}
static inline uint16_t exitLastFrame() {
  if (motionCfg.exitStart) return uploadedFrameCount - 1;
  return hasEntrySeg() ? (motionCfg.mainStart - 1) : loopLastFrame();
}

static void parseMotionCfg(const uint8_t* p, MotionConfig& c) {
  c.enabled    = p[0] ? 1 : 0;
  c.mainStart  = (uint16_t)p[1] | ((uint16_t)p[2] << 8);
  c.loopStart  = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
  c.shakeStart = (uint16_t)p[5] | ((uint16_t)p[6] << 8);
  c.outStart   = (uint16_t)p[7] | ((uint16_t)p[8] << 8);
  c.exitStart  = (uint16_t)p[9] | ((uint16_t)p[10] << 8);
  c.speed      = p[11];
  c.gapMs      = (uint16_t)p[12] | ((uint16_t)p[13] << 8);
  c.bg         = (uint16_t)p[14] | ((uint16_t)p[15] << 8);
}

// Strict validation against a payload with n frames. Any inconsistency
// rejects the whole config — nothing is silently collapsed or defaulted.
static bool motionCfgValid(const MotionConfig& c, uint16_t n) {
  if (c.speed == 0) return false;
  if (!c.enabled) return true;         // frame fields unused when disabled
  if (c.mainStart >= n) return false;
  if (c.mainStart == 0 && c.loopStart != 0) return false;  // intro needs entry
  if (c.loopStart < c.mainStart || c.loopStart >= n) return false;
  const uint16_t lf = c.loopStart;     // == loopFirstFrame() for a valid cfg
  const uint16_t exitB = c.exitStart ? c.exitStart : n;
  const uint16_t outB = c.outStart ? c.outStart : exitB;
  if (c.exitStart && (c.exitStart <= lf || c.exitStart >= n)) return false;
  if (c.outStart && (c.outStart <= lf || c.outStart >= exitB)) return false;
  if (c.shakeStart && (c.shakeStart <= lf || c.shakeStart >= outB)) return false;
  return true;
}

// Adopt the presentation config carried by the active payload (v6); older
// payloads and the built-in default play as a plain loop. The payload was
// validated at commit/load time, motion block included.
static void adoptPayloadMotion() {
  if (uploadedActive && upVer >= 6) {
    parseMotionCfg(&uploadBuf[MOTION_BLOCK_OFF], motionCfg);
  } else {
    motionCfg = MotionConfig{ 0, 1, 0, 0, 0, 0, 60, 1000, 0x0000 };
  }
  applyActiveSource();
}

// Begin the walk-out. When the exit reuses frames that are not currently
// playing, the chain is replayed to the segment start; without entry and
// exit segments the main loop simply keeps playing while translating.
static void startExit() {
  motionPhase = PH_EXIT;
  motionPosMpx = 0;
  if (motionCfg.exitStart || hasEntrySeg()) {
    animJumpTo(exitFirstFrame());
  }
}

// Per-loop motion driver: advances phase/position/frames and redraws when
// the frame, tilt bin or walk offset changed.
static void runMotion(uint32_t now, bool binChanged) {
  bool redraw = binChanged;
  int32_t dt = (int32_t)(now - motionLastMs);
  motionLastMs = now;
  const int32_t maxMpx = (int32_t)motionMaxShift() * 1000;

  switch (motionPhase) {
    case PH_GAP:
      if (handledRecently(now) && (int32_t)(now - gapReadyMs) >= 0) {
        motionPhase = PH_ENTER;
        motionPosMpx = maxMpx;
        animJumpTo(0);
        lastFrameTime = now;
        redraw = true;
      }
      break;

    case PH_ENTER:
    case PH_EXIT:
      motionPosMpx -= (int32_t)motionCfg.speed * dt;
      if (now - lastFrameTime >= getDuration(curFrame)) {
        lastFrameTime = now;
        if (motionPhase == PH_ENTER) animAdvanceRange(0, enterLastFrame());
        else                         animAdvanceRange(exitFirstFrame(), exitLastFrame());
        redraw = true;
      }
      if (motionPhase == PH_ENTER && motionPosMpx <= 0) {
        motionPosMpx = 0;
        // Without an entry segment the main loop is already playing, so its
        // frame position is kept instead of snapping back to the start.
        if (hasEntrySeg()) {
          if (hasIntroSeg()) {
            motionPhase = PH_TRANS_IN;
            animJumpTo(motionCfg.mainStart);
          } else {
            motionPhase = PH_MAIN;
            animJumpTo(loopFirstFrame());
          }
          lastFrameTime = now;
        } else {
          motionPhase = PH_MAIN;
        }
        redraw = true;
      } else if (motionPhase == PH_EXIT && motionPosMpx <= -maxMpx) {
        motionPhase = PH_GAP;
        gapReadyMs = now + motionCfg.gapMs;
        fillScreen(motionCfg.bg);
        return;
      }
      break;

    case PH_TRANS_IN:   // one-shot [mainStart, loopStart); ends inside the loop
      if (now - lastFrameTime >= getDuration(curFrame)) {
        lastFrameTime = now;
        animAdvanceRange(motionCfg.mainStart, loopLastFrame());
        if (curFrame >= loopFirstFrame()) motionPhase = PH_MAIN;
        redraw = true;
      }
      break;

    case PH_MAIN:
      if (now - lastFrameTime >= getDuration(curFrame)) {
        lastFrameTime = now;
        if (shakePending && hasShakeSeg()) {
          // Switch on the frame boundary: the current frame plays out fully
          // so the authored connection from loop into shake stays intact.
          shakePending = false;
          motionPhase = PH_SHAKE;
          animJumpTo(motionCfg.shakeStart);
          redraw = true;
          break;
        }
        if (curFrame >= loopLastFrame()) {
          if (handledRecently(now)) {
            animJumpTo(loopFirstFrame());      // keep looping while handled
          } else if (motionCfg.outStart) {
            motionPhase = PH_TRANS_OUT;
            animAdvanceRange(motionCfg.outStart, outLastFrame());
          } else {
            startExit();
          }
        } else {
          animAdvanceRange(loopFirstFrame(), loopLastFrame());
        }
        redraw = true;
      }
      break;

    case PH_SHAKE:  // one-shot [shakeStart, outStart | exitStart | N)
      shakePending = false;                    // ignore re-shakes mid-play
      if (now - lastFrameTime >= getDuration(curFrame)) {
        lastFrameTime = now;
        if (curFrame >= shakeLastFrame()) {
          motionPhase = PH_MAIN;
          animJumpTo(loopFirstFrame());
        } else {
          animAdvanceRange(motionCfg.shakeStart, shakeLastFrame());
        }
        redraw = true;
      }
      break;

    case PH_TRANS_OUT:  // one-shot [outStart, exitStart | N)
      if (now - lastFrameTime >= getDuration(curFrame)) {
        lastFrameTime = now;
        if (curFrame >= outLastFrame()) startExit();
        else animAdvanceRange(motionCfg.outStart, outLastFrame());
        redraw = true;
      }
      break;
  }

  if (motionPhase == PH_GAP) return;
  int16_t shift = (int16_t)(motionPosMpx / 1000);
  if (shift != lastWalkShift) redraw = true;
  if (redraw) {
    lastWalkShift = shift;
    drawFrameRotated(frameBuf, currentBin, shift);
  }
}

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
  uint8_t ver = uploadBuf[2];

  uint16_t n;
  uint16_t size;
  uint8_t bpp;
  uint32_t palOff;
  if (ver == 1) {           // legacy: 64x64, 16-color fixed
    n = uploadBuf[3];
    if (n == 0 || n > MAX_UPLOADED_FRAMES) return false;
    size = 64; bpp = 4; palOff = 4;
  } else if (ver == 2) {    // sized: 64 or 128, square, 16 or 256 colors
    if (uploadPos < 8) return false;
    n = uploadBuf[3];
    if (n == 0 || n > MAX_UPLOADED_FRAMES) return false;
    size = uploadBuf[4];
    if ((size != 64 && size != 128) || uploadBuf[5] != size) return false;
    bpp = uploadBuf[6];
    if (bpp == 0) bpp = 4;
    if (bpp != 4 && bpp != 8) return false;
    palOff = 8;
  } else if (ver >= 3 && ver <= 6) {  // diff chain (v4 +RLE, v5 +ref, v6 +motion)
    if (uploadPos < 8) return false;
    n = (uint16_t)uploadBuf[3] | ((uint16_t)uploadBuf[4] << 8);
    if (n == 0) return false;
    size = uploadBuf[5];
    if (size != 64 && size != 120 && size != 128) return false;
    bpp = uploadBuf[6];
    if (bpp != 4 && bpp != 8) return false;
    palOff = (ver >= 6) ? (MOTION_BLOCK_OFF + MOTION_BLOCK_LEN) : 8;
    if (ver >= 6) {
      if (uploadPos < palOff) return false;
      MotionConfig mc;
      parseMotionCfg(&uploadBuf[MOTION_BLOCK_OFF], mc);
      if (!motionCfgValid(mc, n)) return false;
    }
  } else {
    return false;
  }

  uint32_t palBytes = (bpp == 8) ? 512 : 32;
  uint32_t dataOff = palOff + palBytes;

  if (ver >= 3) {
    // Walk every record; playback trusts these offsets, rect bounds,
    // (v4) RLE stream integrity and (v5) referenced-frame composability.
    // refFreeBits marks frames whose replay window holds no referenced
    // diff, the only legal targets for a reference (single-level decode).
    static uint8_t refFreeBits[256];               // up to 2048 frames
    if (ver >= 5) {
      if (n > 2048) return false;
      memset(refFreeBits, 0, sizeof(refFreeBits));
    }
    bool windowHasRef = false;
    uint32_t frameBytes = (uint32_t)size * size * bpp / 8;
    uint32_t off = dataOff;
    for (uint16_t i = 0; i < n; i++) {
      if (off + 3 > uploadPos) return false;
      uint8_t rflags = uploadBuf[off + 2];
      if ((ver >= 5) && (rflags & 0x02) && !(rflags & 0x01)) {
        // referenced diff
        if (i == 0) return false;
        if (off + 6 > uploadPos) return false;
        uint16_t ref = (uint16_t)uploadBuf[off + 3] | ((uint16_t)uploadBuf[off + 4] << 8);
        if (ref >= i) return false;
        if (!(refFreeBits[ref >> 3] & (1 << (ref & 7)))) return false;
        uint8_t rectCount = uploadBuf[off + 5];
        off += 6;
        for (uint8_t r = 0; r < rectCount; r++) {
          if (off + 6 > uploadPos) return false;
          uint8_t rw = uploadBuf[off + 2], rh = uploadBuf[off + 3];
          if (rw == 0 || rh == 0) return false;
          if ((uint16_t)uploadBuf[off] + rw > size) return false;
          if ((uint16_t)uploadBuf[off + 1] + rh > size) return false;
          uint32_t px = (uint32_t)rw * rh;
          uint32_t raw = (bpp == 8) ? px : ((px + 1) >> 1);
          uint32_t rl = (uint32_t)uploadBuf[off + 4] | ((uint32_t)uploadBuf[off + 5] << 8);
          if (off + 6 + rl > uploadPos) return false;
          if (!rleCheck(&uploadBuf[off + 6], rl, raw)) return false;
          off += 6 + rl;
        }
        windowHasRef = true;
        if (off > uploadPos) return false;
        continue;
      }
      if (uploadBuf[off + 2] & 0x01) {
        if (ver >= 4) {
          if (off + 5 > uploadPos) return false;
          uint32_t rl = (uint32_t)uploadBuf[off + 3] | ((uint32_t)uploadBuf[off + 4] << 8);
          if (off + 5 + rl > uploadPos) return false;
          if (!rleCheck(&uploadBuf[off + 5], rl, frameBytes)) return false;
          off += 5 + rl;
        } else {
          off += 3 + frameBytes;
        }
        windowHasRef = false;
        if (ver >= 5) refFreeBits[i >> 3] |= (uint8_t)(1 << (i & 7));
      } else {
        if (i == 0) return false;  // frame 0 must be a keyframe
        if (off + 4 > uploadPos) return false;
        uint8_t rectCount = uploadBuf[off + 3];
        off += 4;
        for (uint8_t r = 0; r < rectCount; r++) {
          uint32_t rectHdr = (ver >= 4) ? 6 : 4;
          if (off + rectHdr > uploadPos) return false;
          uint8_t rw = uploadBuf[off + 2], rh = uploadBuf[off + 3];
          if (rw == 0 || rh == 0) return false;
          if ((uint16_t)uploadBuf[off] + rw > size) return false;
          if ((uint16_t)uploadBuf[off + 1] + rh > size) return false;
          uint32_t px = (uint32_t)rw * rh;
          uint32_t raw = (bpp == 8) ? px : ((px + 1) >> 1);
          if (ver >= 4) {
            uint32_t rl = (uint32_t)uploadBuf[off + 4] | ((uint32_t)uploadBuf[off + 5] << 8);
            if (off + 6 + rl > uploadPos) return false;
            if (!rleCheck(&uploadBuf[off + 6], rl, raw)) return false;
            off += 6 + rl;
          } else {
            off += 4 + raw;
          }
        }
        if (ver >= 5 && !windowHasRef) {
          refFreeBits[i >> 3] |= (uint8_t)(1 << (i & 7));
        }
      }
      if (off > uploadPos) return false;
    }
    if (off != uploadPos) return false;
  } else {
    uint32_t rec = (uint32_t)size * size * bpp / 8 + 2;
    uint32_t expected = dataOff + (uint32_t)n * rec;
    if (expected > UPLOAD_BUF_SIZE || uploadPos != expected) return false;
    upFrameRec = rec;
  }

  upVer = ver;
  upSize = size;
  upBpp = bpp;
  upPalOff = palOff;
  upDataOff = dataOff;
  uploadedFrameCount = n;
  return true;
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
  uploadedActive = true;
  applyActiveSource();
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
    digitalWrite(STATUS_LED, LOW);
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    digitalWrite(STATUS_LED, HIGH);
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
    // The incoming payload overwrites the active image's backing store, so
    // playback must not keep reading a half-written buffer (v3 record offsets
    // would walk garbage). Fall back to the default animation until commit.
    if (uploadPos == 0 && uploadedActive) {
      uploadedActive = false;
      applyActiveSource();
      uploadCommittedFlag = true;
    }
    memcpy(&uploadBuf[uploadPos], data, len);
    uploadPos += len;
    setStatus(STATUS_RECEIVING);
  }
};

class CommitCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();
    if (v.length() < 1) {
      setStatus(STATUS_ERROR);
      return;
    }
    uint8_t cmd = ((const uint8_t*)v.c_str())[0];
    switch (cmd) {
      case 0x01:  // commit
        if (validateUpload()) {
          uploadedActive = true;
          applyActiveSource();
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
        applyActiveSource();
        uploadCommittedFlag = true;
        clearPersistRequested = true;
        setStatus(STATUS_IDLE);
        break;
      default:
        setStatus(STATUS_ERROR);
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

  BLECharacteristic* pProto = pService->createCharacteristic(
    BLE_PROTO_UUID, BLECharacteristic::PROPERTY_READ);
  uint8_t proto[4] = { PAYLOAD_VER_MAX, PAYLOAD_VER_MIN, 0, 0 };
  pProto->setValue(proto, 4);

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void setup() {
  gpio_hold_dis((gpio_num_t)TFT_RST);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);   // off until a BLE central connects

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

  curAbsBin = MOUNT_NEUTRAL_BIN;
  currentBin = 0;
  fxLp = (int32_t)xRef * ACCEL_X_SIGN;
  fyLp = (int32_t)yRef * ACCEL_Y_SIGN;
  lastFrameTime = millis();
  lastActivityMs = millis();
  lastLpUpdateMs = millis();

  // Restore persisted image (if present and valid). Falls back to default.
  // Surfaces filesystem / corruption errors via a colored splash.
  LoadResult lr = loadPersistedImage();
  showLoadError(lr);
  adoptPayloadMotion();

  bleSetup();

  if (motionOn()) {
    // Start hidden with the handling timer seeded, so the sprite walks in
    // immediately after the wake-by-motion.
    motionPhase = PH_GAP;
    gapReadyMs = millis();
    lastHandledMs = millis();
    motionLastMs = millis();
  } else {
    animReset();
    drawFrameRotated(frameBuf, currentBin, 0);
  }
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

  // Apply pending image source switch (commit or revert). The motion config
  // indexes frames of the previous payload, so it is cleared here; the
  // uploader re-sends it after a commit if the user wants motion.
  if (uploadCommittedFlag) {
    uploadCommittedFlag = false;
    adoptPayloadMotion();
    lastFrameTime = now;
    motionLastMs = now;
    fillScreen(motionCfg.bg);  // clear stale pixels outside the new window
    if (motionOn()) {
      // Start hidden; seed the handling timer so the sprite walks in right
      // away (a fresh commit implies the user is present).
      motionPhase = PH_GAP;
      gapReadyMs = now;
      lastHandledMs = now;
    } else {
      animReset();
      drawFrameRotated(frameBuf, currentBin, 0);
    }
  }

  // Apply pending persistence operations (deferred from BLE callbacks so the
  // BLE task isn't blocked by Flash writes).
  if (persistRequested) {
    persistRequested = false;
    if (!persistImage()) setStatus(STATUS_ERROR);
  }
  if (clearPersistRequested) {
    clearPersistRequested = false;
    if (!clearPersistedImage()) setStatus(STATUS_ERROR);
  }

  // Update lowpass on raw gravity vector. Blend factor is dt/tau so the
  // effective time constant stays ~LP_TAU_MS whether the loop is spinning
  // at sub-ms (no redraw) or stalled tens of ms by a frame write.
  int16_t rx, ry, rz;
  if (readXYZRaw(rx, ry, rz)) {
    int32_t fx = (int32_t)rx * ACCEL_X_SIGN;
    int32_t fy = (int32_t)ry * ACCEL_Y_SIGN;
    // Handling detection: raw deviating from the lowpass = the device is
    // being moved by hand. Counts as activity (keeps the display awake).
    int32_t adx = labs(fx - fxLp);
    int32_t ady = labs(fy - fyLp);
    if (adx > HANDLE_THRESH || ady > HANDLE_THRESH) {
      lastHandledMs = now;
      lastActivityMs = now;
    }
    if ((adx > SHAKE_THRESH || ady > SHAKE_THRESH) &&
        motionOn() && motionPhase == PH_MAIN && hasShakeSeg()) {
      shakePending = true;
    }
    int32_t dt = (int32_t)(now - lastLpUpdateMs);
    if (dt > 0) {
      if (dt > LP_TAU_MS) dt = LP_TAU_MS;
      lastLpUpdateMs = now;
      fxLp += (fx - fxLp) * dt / LP_TAU_MS;
      fyLp += (fy - fyLp) * dt / LP_TAU_MS;
    }
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

  if (motionOn()) {
    runMotion(now, binChanged);
    return;
  }

  uint16_t duration = getDuration(curFrame);
  bool frameAdvance = (now - lastFrameTime >= duration);
  if (frameAdvance) {
    lastFrameTime = now;
    animAdvance();
  }

  if (frameAdvance || binChanged) {
    drawFrameRotated(frameBuf, currentBin, 0);
  }
}
