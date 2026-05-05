// GC9A01 animation viewer for ESPr Developer C3 (ESP32-C3)
// with KXTJ3 accelerometer + sleep support
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

// Pixel scale factor: 64x64 source -> 192x192 on screen
static constexpr uint8_t SCALE = 3;

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

static void drawFrame(const uint16_t* buf) {
  const uint16_t scaledW = (uint16_t)IMG_W * SCALE;
  const uint16_t scaledH = (uint16_t)IMG_H * SCALE;
  uint16_t ox = (WIDTH - scaledW) / 2;
  uint16_t oy = (HEIGHT - scaledH) / 2;
  setWindow(ox, oy, ox + scaledW - 1, oy + scaledH - 1);
  for (uint8_t y = 0; y < IMG_H; y++) {
    const uint16_t* row = &buf[(uint16_t)y * IMG_W];
    for (uint8_t r = 0; r < SCALE; r++) {
      for (uint8_t x = 0; x < IMG_W; x++) {
        uint16_t c = row[x];
        for (uint8_t s = 0; s < SCALE; s++) writePixel(c);
      }
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

// ---- Orientation detection ----

static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;

static uint8_t detectOrientation(uint8_t current, int16_t xRef, int16_t yRef) {
  int16_t rx, ry, rz;
  if (!readXYZRaw(rx, ry, rz)) return current;

  int16_t dx = (xRef - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - yRef) * ACCEL_Y_SIGN;

  const int16_t thresholdIn = 8000;
  const int16_t thresholdOut = 4000;

  int16_t absDx = dx > 0 ? dx : -dx;
  int16_t absDy = dy > 0 ? dy : -dy;
  int16_t maxTilt = absDx > absDy ? absDx : absDy;

  if (maxTilt > thresholdOut) {
    switch (current) {
      case 1:  if (absDx > absDy && dx < 0) return 1; break;
      case 2:  if (absDx > absDy && dx > 0) return 2; break;
      case 3:  if (absDy > absDx && dy < 0) return 3; break;
      case 0:  if (absDy > absDx && dy > 0) return 0; break;
    }
  } else {
    return current;
  }

  if (maxTilt > thresholdIn) {
    if (absDx > absDy) return (dx > 0) ? 2 : 1;
    else return (dy < 0) ? 3 : 0;
  }

  return current;
}

static inline uint16_t getSrcIndex(uint8_t x, uint8_t y, uint8_t orient, uint8_t w, uint8_t h) {
  uint8_t srcX, srcY;
  switch (orient) {
    case 1:  srcX = y; srcY = (h - 1) - x; break;
    case 2:  srcX = (w - 1) - y; srcY = x; break;
    case 3:  srcX = (w - 1) - x; srcY = (h - 1) - y; break;
    default: srcX = x; srcY = y; break;
  }
  return (uint16_t)srcY * w + srcX;
}

static void composeFrameOriented(uint8_t frameIdx, uint8_t orient, uint16_t* buf) {
  uint8_t ftype = pgm_read_byte(&frames[frameIdx].type);
  const uint8_t* data;
  if (ftype == 0) {
    data = (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);
  } else {
    uint8_t ref = pgm_read_byte(&frames[frameIdx].ref);
    data = (const uint8_t*)pgm_read_ptr(&frames[ref].data);
  }

  // Draw base frame with orientation
  for (uint8_t dy = 0; dy < IMG_H; dy++) {
    for (uint8_t dx = 0; dx < IMG_W; dx++) {
      uint16_t srcIdx = getSrcIndex(dx, dy, orient, IMG_W, IMG_H);
      uint8_t palIdx = read4bit(data, srcIdx);
      buf[(uint16_t)dy * IMG_W + dx] = pgm_read_word(&palette[palIdx]);
    }
  }

  // Apply overlay if needed
  if (ftype == 1) {
    const uint8_t* ovData = (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);
    uint8_t rx = pgm_read_byte(&frames[frameIdx].rx);
    uint8_t ry = pgm_read_byte(&frames[frameIdx].ry);
    uint8_t rw = pgm_read_byte(&frames[frameIdx].rw);
    uint8_t rh = pgm_read_byte(&frames[frameIdx].rh);

    for (uint8_t dy = 0; dy < rh; dy++) {
      for (uint8_t dx = 0; dx < rw; dx++) {
        uint16_t srcIdx = getSrcIndex(dx, dy, orient, rw, rh);
        uint8_t palIdx = read4bit(ovData, srcIdx);

        // Map overlay dest coords with orientation
        uint8_t dstX, dstY;
        switch (orient) {
          case 1:  dstX = (IMG_H - 1) - (ry + rh - 1) + dy; dstY = rx + dx; break;
          case 2:  dstX = ry + dy; dstY = (IMG_W - 1) - (rx + rw - 1) + dx; break;
          case 3:  dstX = (IMG_W - 1) - (rx + rw - 1) + dx; dstY = (IMG_H - 1) - (ry + rh - 1) + dy; break;
          default: dstX = rx + dx; dstY = ry + dy; break;
        }
        buf[(uint16_t)dstY * IMG_W + dstX] = pgm_read_word(&palette[palIdx]);
      }
    }
  }
}

// ---- Sleep ----

static volatile uint32_t lastActivityMs = 0;

static void displayOff() {
  analogWrite(TFT_BL, 0);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
  gpio_hold_en((gpio_num_t)TFT_BL);

  cmdWithData(0x28, nullptr, 0);  // DISPOFF
  cmdWithData(0x10, nullptr, 0);  // SLPIN

  // Hold RST asserted so the panel sits in reset (lower leakage than SLPIN)
  digitalWrite(TFT_RST, LOW);
  gpio_hold_en((gpio_num_t)TFT_RST);

  gpio_deep_sleep_hold_en();
}

// ---- Timeouts ----
static constexpr uint32_t DIM_TIMEOUT_MS   = 10000;
static constexpr uint32_t SLEEP_TIMEOUT_MS = 30000;

// ---- State ----
static uint8_t curFrame = 0;
static uint8_t currentOrient = 0;
static uint32_t lastFrameTime = 0;
static uint16_t frameBuf[IMG_W * IMG_H];
static int16_t xRef = 0, yRef = 0, zRef = 0;

// Orientation cooldown
static constexpr uint32_t ORIENT_COOLDOWN_MS = 1500;
static uint32_t lastOrientChangeMs = 0;
static int8_t pendingOrient = -1;

void setup() {
  // Release any hold left by the previous deep-sleep cycle
  gpio_hold_dis((gpio_num_t)TFT_RST);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  // Hardware reset
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

  // I2C + KXTJ3
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  write8(REG_CTRL1, 0xC0); delay(50);

  calibrateKXTJ3(xRef, yRef, zRef);
  enableWakeupInterrupt();
  esp_deep_sleep_enable_gpio_wakeup(BIT(KXTJ3_INT_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);

  curFrame = 0;
  currentOrient = 0;
  lastFrameTime = millis();
  lastActivityMs = millis();

  composeFrameOriented(curFrame, currentOrient, frameBuf);
  drawFrame(frameBuf);
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - lastActivityMs;

  // Deep sleep
  if (elapsed >= SLEEP_TIMEOUT_MS) {
    displayOff();
    clearLatchedInterrupt();
    esp_deep_sleep_start();
    // ここには戻らない。復帰は setup() から再実行
  }

  // Dim
  if (elapsed >= DIM_TIMEOUT_MS) {
    analogWrite(TFT_BL, 40);
  } else {
    analogWrite(TFT_BL, 255);
  }

  // Orientation detection with cooldown
  uint8_t orient = detectOrientation(currentOrient, xRef, yRef);
  bool inCooldown = (now - lastOrientChangeMs) < ORIENT_COOLDOWN_MS;

  if (orient != currentOrient) {
    if (inCooldown) {
      pendingOrient = orient;
    } else {
      lastActivityMs = now;
      lastOrientChangeMs = now;
      pendingOrient = -1;
      currentOrient = orient;
      curFrame = 0;
      lastFrameTime = now;
      fillScreen(0x0000);
      composeFrameOriented(curFrame, currentOrient, frameBuf);
      drawFrame(frameBuf);
      return;
    }
  } else if (!inCooldown && pendingOrient >= 0 && (uint8_t)pendingOrient != currentOrient) {
    orient = (uint8_t)pendingOrient;
    pendingOrient = -1;
    lastActivityMs = now;
    lastOrientChangeMs = now;
    currentOrient = orient;
    curFrame = 0;
    lastFrameTime = now;
    fillScreen(0x0000);
    composeFrameOriented(curFrame, currentOrient, frameBuf);
    drawFrame(frameBuf);
    return;
  }

  // Frame advance
  uint16_t duration = pgm_read_word(&frames[curFrame].duration_ms);
  if (now - lastFrameTime >= duration) {
    lastFrameTime = now;
    uint8_t nextFrame = pgm_read_byte(&frames[curFrame].next);
    curFrame = nextFrame;
    composeFrameOriented(curFrame, currentOrient, frameBuf);
    drawFrame(frameBuf);
  }

}
