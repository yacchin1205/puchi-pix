// GC9A01 free-rotation viewer for ESPr Developer C3 (ESP32-C3)
// 64-step rotation (5.625deg) following gravity tilt via KXTJ3
#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

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

// ---- Image data ----
#include "../puchi_pix/frame.h"
#include "../puchi_pix/icon_original.h"

// 64x64 source -> 128x128 visible (fits 240 disc with rotation, diagonal 181)
static constexpr uint8_t SCALE_SHIFT = 1;  // SCALE = 1 << SCALE_SHIFT = 2
static constexpr int16_t OUT_SIZE = (IMG_W << SCALE_SHIFT) + 64;  // 128 + 64 BG margin = 192
static constexpr int16_t OUT_HALF = OUT_SIZE / 2;
static constexpr int16_t SRC_HALF = IMG_W / 2;

// ---- 64-step rotation table (Q15) ----
// COS64[i] = cos(i*5.625deg) * 32768  (saturated at +/-32767)
// SIN64[i] = sin(i*5.625deg) * 32768  (saturated at +/-32767)
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

// ---- Drawing ----

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

// Compose frame to 64x64 source-coord buffer (no rotation)
static void composeFrame(uint8_t frameIdx, uint16_t* buf) {
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

// Reverse-rotate source buffer onto a centered OUT_SIZE x OUT_SIZE region.
// bin in 0..63 selects the rotation angle (bin*5.625deg).
static void drawFrameRotated(const uint16_t* src, uint8_t bin) {
  const int32_t cosT = COS64[bin & 0x3F];
  const int32_t sinT = SIN64[bin & 0x3F];

  const uint16_t ox0 = (WIDTH - OUT_SIZE) / 2;
  const uint16_t oy0 = (HEIGHT - OUT_SIZE) / 2;
  setWindow(ox0, oy0, ox0 + OUT_SIZE - 1, oy0 + OUT_SIZE - 1);

  // For each output pixel (ox,oy) in ROI, compute source coord:
  //   sx = ((cx*cos + cy*sin) >> 15) / SCALE + SRC_HALF
  //   sy = ((-cx*sin + cy*cos) >> 15) / SCALE + SRC_HALF
  // Combined Q15 unscale + SCALE divide = >> (15 + SCALE_SHIFT)
  const int8_t SHIFT = 15 + SCALE_SHIFT;

  for (int16_t oy = 0; oy < OUT_SIZE; oy++) {
    int32_t cy = oy - OUT_HALF;
    // Per-row constants (cy*sinT, cy*cosT) hoisted out of inner loop
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
// Use raw gravity direction (rx, ry) in chip frame. Display bin is the
// gravity-direction bin minus a fixed mounting-offset constant. Independent
// of boot-time orientation. KXTJ3 16-bit raw at +/-2g: 1g ~= 16384 counts.
// In-plane gravity = sin(tilt_off_z) * 16384.

// Sign flips: tweak if rotation direction is inverted vs. physical rotation.
static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;
// Sign of bin delta direction (flip if icon rotates opposite to device).
static constexpr int BIN_DIR_SIGN = -1;
// Bin value that gravity (fxLp, fyLp) points to when the device is held in
// its natural "display upright" orientation. Hardware-mounting constant.
// Range 0..63, each step = 5.625 degrees.
// Tune by trial: pick a value, flash, hold the device upright, observe the
// icon angle, then offset MOUNT_NEUTRAL_BIN by that many bins.
static constexpr uint8_t MOUNT_NEUTRAL_BIN = 48;

// IIR lowpass weight: out = (out*(N-1) + sample) / N   with N = 1<<LP_SHIFT
static constexpr uint8_t LP_SHIFT = 3;

// Minimum in-plane gravity magnitude squared. Below this, rotation is held.
// 0.3g threshold: |fxy| > ~4900, mag2 > ~2.4e7  (device must tilt >~17deg off Z)
static constexpr int32_t MAG2_MIN = 24000000L;

// Hysteresis: rival bin must beat current bin's score by (curScore >> SHIFT).
// Larger SHIFT = lighter hysteresis (smaller angular dead zone).
//   SHIFT=7 -> ~4.6deg per side, 8 -> ~2.3deg, 9 -> ~1.1deg, 10 -> ~0.6deg
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
static uint8_t curAbsBin = MOUNT_NEUTRAL_BIN;  // gravity-direction bin (hysteretic)
static uint8_t currentBin = 0;                 // display bin (derived from curAbsBin)
static uint32_t lastFrameTime = 0;
static uint16_t frameBuf[IMG_W * IMG_H];
static int16_t xRef = 0, yRef = 0, zRef = 0;
static int32_t fxLp = 0, fyLp = 0;

// Argmax of dot product with 64 unit vectors, with proportional hysteresis.
// Returns curBin unless another bin's score exceeds curBin's score by
// (curScore >> HYSTERESIS_SHIFT). Margin scales with tilt magnitude so the
// angular dead zone is roughly constant at any tilt.
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

  // Calibration only used to seed the IIR filter; rotation reference is the
  // compile-time MOUNT_NEUTRAL_BIN constant, not the boot-time orientation.
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

  composeFrame(curFrame, frameBuf);
  drawFrameRotated(frameBuf, currentBin);
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - lastActivityMs;

  if (elapsed >= SLEEP_TIMEOUT_MS) {
    displayOff();
    clearLatchedInterrupt();
    esp_deep_sleep_start();
  }

  analogWrite(TFT_BL, (elapsed >= DIM_TIMEOUT_MS) ? 40 : 255);

  // Update lowpass on raw gravity vector (chip frame, with sign flips)
  int16_t rx, ry, rz;
  if (readXYZRaw(rx, ry, rz)) {
    int32_t fx = (int32_t)rx * ACCEL_X_SIGN;
    int32_t fy = (int32_t)ry * ACCEL_Y_SIGN;
    fxLp = (fxLp * ((1 << LP_SHIFT) - 1) + fx) >> LP_SHIFT;
    fyLp = (fyLp * ((1 << LP_SHIFT) - 1) + fy) >> LP_SHIFT;
  }

  // Hysteretic bin update: only when in-plane gravity is strong enough.
  int32_t mag2 = fxLp * fxLp + fyLp * fyLp;
  if (mag2 > MAG2_MIN) {
    curAbsBin = computeBin(fxLp, fyLp, curAbsBin);
  }
  // Derive display bin from absolute bin every loop (cheap).
  int8_t diff = (int8_t)curAbsBin - (int8_t)MOUNT_NEUTRAL_BIN;
  diff *= BIN_DIR_SIGN;
  uint8_t newBin = (uint8_t)diff & 0x3F;

  bool binChanged = (newBin != currentBin);
  if (binChanged) {
    currentBin = newBin;
    lastActivityMs = now;
  }

  // Frame advance
  uint16_t duration = pgm_read_word(&frames[curFrame].duration_ms);
  bool frameAdvance = (now - lastFrameTime >= duration);
  if (frameAdvance) {
    lastFrameTime = now;
    curFrame = pgm_read_byte(&frames[curFrame].next);
  }

  if (frameAdvance || binChanged) {
    composeFrame(curFrame, frameBuf);
    drawFrameRotated(frameBuf, currentBin);
  }
}
