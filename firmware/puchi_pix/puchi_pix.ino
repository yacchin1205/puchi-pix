// NOTE: Build with "Tools > U(S)ART support > Disabled" to fit in 32KB flash
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// =====================
// 画像データ (アイコン差し替えはここだけ変更)
// =====================
#include "frame.h"
#include "icon_IMG_1534.h"

// =====================
// ディスプレイ選択: いずれか1つをdefine (両方コメントアウトでST7735 TFT)
// =====================
// #define USE_SSD1351
#define USE_SSD1331

// =====================
// 共通ピン定義
//   SCL -> PA_5 (SPI SCK)
//   SDA -> PA_7 (SPI MOSI)
//   DC  -> PA_1
//   CS  -> PA_0
// =====================
static constexpr uint8_t TFT_CS = PA_0;
static constexpr uint8_t TFT_DC = PA_1;
static constexpr uint8_t TFT_RST = PA_2;

#if defined(USE_SSD1351)
// SSD1351 OLED 128x128
static constexpr int TFT_W = 128;
static constexpr int TFT_H = 128;
static constexpr uint8_t X_OFFSET = 0;
static constexpr uint8_t Y_OFFSET = 0;
// 加速度センサー座標反転 (SSD1351用)
static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;
#elif defined(USE_SSD1331)
// SSD1331 OLED 96x64
static constexpr int TFT_W = 96;
static constexpr int TFT_H = 64;
static constexpr uint8_t X_OFFSET = 0;
static constexpr uint8_t Y_OFFSET = 0;
// 加速度センサー座標反転 (SSD1331用)
static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;
#else
// ST7735 TFT 128x160
static constexpr int TFT_W = 128;
static constexpr int TFT_H = 160;
static constexpr uint8_t X_OFFSET = 2;
static constexpr uint8_t Y_OFFSET = 1;
static constexpr uint8_t TFT_BL = PA_8;
// 加速度センサー座標反転 (ST7735用)
static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;
#endif

// =====================
// KXTJ3 (I2C)
//   SDA -> PB_7
//   SCL -> PB_6
//   INT -> PA_4
// =====================
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

static constexpr uint32_t KXTJ3_INT_PIN = PA_4;
#if defined(USE_SSD1331) || defined(USE_SSD1351)
static constexpr uint32_t STAT_LED_PIN  = PA_8;
#endif

// アニメーション状態
static uint8_t  curFrame = 0;
static int8_t   lastDrawnFrame = -1;
static uint8_t  currentOrient = 0;
static uint32_t lastFrameTime = 0;

// ---------- KXTJ3 low-level ----------
static inline void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(KXTJ3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// ---------- Display Power / Sleep ----------
static constexpr uint32_t DIM_TIMEOUT_MS   = 10000;
static constexpr uint32_t SLEEP_TIMEOUT_MS = 30000;

static uint32_t lastActivityMs = 0;

#if defined(USE_SSD1351) || defined(USE_SSD1331)
// OLED用: 前方宣言
static void oledDisplayOn();
static void oledDisplayOff();
static void oledSetContrast(uint8_t level);

static inline void backlightFull() { oledDisplayOn(); oledSetContrast(0x06); }
static inline void backlightDim()  { oledSetContrast(0x01); }
static inline void backlightOff()  { oledDisplayOff(); }
#else
// TFT用: PWMバックライト
static inline void backlightFull() { analogWrite(TFT_BL, 0); }
static inline void backlightDim()  { analogWrite(TFT_BL, 210); }
static inline void backlightOff()  { analogWrite(TFT_BL, 255); }
#endif

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

void wakeupCallback() {
  lastActivityMs = millis();
}

// ---------- TFT low-level ----------
static inline void dcCommand() { digitalWrite(TFT_DC, LOW); }
static inline void dcData()    { digitalWrite(TFT_DC, HIGH); }

static inline void tftWriteCommand(uint8_t cmd) {
  dcCommand();
  digitalWrite(TFT_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(TFT_CS, HIGH);
}

static inline void tftWriteDataN(const uint8_t* p, uint8_t n) {
  dcData();
  digitalWrite(TFT_CS, LOW);
  while (n--) SPI.transfer(*p++);
  digitalWrite(TFT_CS, HIGH);
}

static inline void tftWriteCommandData(uint8_t cmd, const uint8_t* data, uint8_t n) {
  digitalWrite(TFT_CS, LOW);
  dcCommand();
  SPI.transfer(cmd);
  if (n) {
    dcData();
    while (n--) SPI.transfer(*data++);
  }
  digitalWrite(TFT_CS, HIGH);
}

// ---------- Address Window ----------
#if defined(USE_SSD1351)
static inline void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t dataX[2] = { (uint8_t)(x0 + X_OFFSET), (uint8_t)(x1 + X_OFFSET) };
  tftWriteCommandData(0x15, dataX, 2);
  uint8_t dataY[2] = { (uint8_t)(y0 + Y_OFFSET), (uint8_t)(y1 + Y_OFFSET) };
  tftWriteCommandData(0x75, dataY, 2);
  tftWriteCommand(0x5C);
}
#elif defined(USE_SSD1331)
static inline void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  // SSD1331: all bytes with D/C=LOW
  digitalWrite(TFT_CS, LOW);
  dcCommand();
  SPI.transfer(0x15);
  SPI.transfer((uint8_t)(x0 + X_OFFSET));
  SPI.transfer((uint8_t)(x1 + X_OFFSET));
  digitalWrite(TFT_CS, HIGH);

  digitalWrite(TFT_CS, LOW);
  dcCommand();
  SPI.transfer(0x75);
  SPI.transfer((uint8_t)(y0 + Y_OFFSET));
  SPI.transfer((uint8_t)(y1 + Y_OFFSET));
  digitalWrite(TFT_CS, HIGH);
}
#else
static inline void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  x0 += X_OFFSET; x1 += X_OFFSET;
  y0 += Y_OFFSET; y1 += Y_OFFSET;
  tftWriteCommand(0x2A);
  uint8_t dataX[4] = { 0, (uint8_t)x0, 0, (uint8_t)x1 };
  tftWriteDataN(dataX, 4);
  tftWriteCommand(0x2B);
  uint8_t dataY[4] = { 0, (uint8_t)y0, 0, (uint8_t)y1 };
  tftWriteDataN(dataY, 4);
  tftWriteCommand(0x2C);
}
#endif

// ---------- Color / Fill ----------
#if defined(USE_SSD1351) || defined(USE_SSD1331)
// RGB565 (16bit)
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void tftFillScreen888(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = rgb565(r, g, b);
  uint8_t hi = c >> 8, lo = c & 0xFF;

#if defined(USE_SSD1351)
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(0x15);
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(0x00);
  SPI.transfer(0x7F);
  digitalWrite(TFT_CS, HIGH);

  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(0x75);
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(0x00);
  SPI.transfer(0x7F);
  digitalWrite(TFT_CS, HIGH);

  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(0x5C);
  digitalWrite(TFT_DC, HIGH);
#else
  // SSD1331: use tftSetAddrWindow for consistency
  tftSetAddrWindow(0, 0, TFT_W - 1, TFT_H - 1);
  dcData();
  digitalWrite(TFT_CS, LOW);
#endif
  for (uint32_t i = 0; i < (uint32_t)TFT_W * TFT_H; i++) {
    SPI.transfer(hi);
    SPI.transfer(lo);
  }
  digitalWrite(TFT_CS, HIGH);
}

static inline void tftHLine888(int x, int y, int w, uint8_t r, uint8_t g, uint8_t b) {
  if ((uint16_t)y >= TFT_H || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > TFT_W) w = TFT_W - x;
  if (w <= 0) return;

  uint16_t c = rgb565(r, g, b);
  uint8_t hi = c >> 8, lo = c & 0xFF;

  tftSetAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)(x + w - 1), (uint16_t)y);
  dcData();
  digitalWrite(TFT_CS, LOW);
  while (w--) {
    SPI.transfer(hi);
    SPI.transfer(lo);
  }
  digitalWrite(TFT_CS, HIGH);
}
#else  // ST7735
// RGB888 (18bit)
static void tftFillScreen888(uint8_t r, uint8_t g, uint8_t b) {
  tftSetAddrWindow(0, 0, TFT_W - 1, TFT_H - 1);
  dcData();
  digitalWrite(TFT_CS, LOW);
  for (uint32_t i = 0; i < (uint32_t)TFT_W * TFT_H; i++) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }
  digitalWrite(TFT_CS, HIGH);
}

static inline void tftHLine888(int x, int y, int w, uint8_t r, uint8_t g, uint8_t b) {
  if ((uint16_t)y >= TFT_H || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > TFT_W) w = TFT_W - x;
  if (w <= 0) return;

  tftSetAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)(x + w - 1), (uint16_t)y);
  dcData();
  digitalWrite(TFT_CS, LOW);
  while (w--) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }
  digitalWrite(TFT_CS, HIGH);
}
#endif

static void tftFillCircle888(int x0, int y0, int rad, uint8_t r8, uint8_t g8, uint8_t b8) {
  if (rad <= 0) return;

  int f = 1 - rad;
  int ddF_x = 1;
  int ddF_y = -2 * rad;
  int x = 0;
  int y = rad;

  tftHLine888(x0 - rad, y0, 2 * rad + 1, r8, g8, b8);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    tftHLine888(x0 - x, y0 + y, 2 * x + 1, r8, g8, b8);
    tftHLine888(x0 - x, y0 - y, 2 * x + 1, r8, g8, b8);
    tftHLine888(x0 - y, y0 + x, 2 * y + 1, r8, g8, b8);
    tftHLine888(x0 - y, y0 - x, 2 * y + 1, r8, g8, b8);
  }
}

// ---------- 矩形塗りつぶし (画像表示用) ----------
#if defined(USE_SSD1351) || defined(USE_SSD1331)
static void tftFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  if (x >= TFT_W || y >= TFT_H) return;
  if (x + w > TFT_W) w = TFT_W - x;
  if (y + h > TFT_H) h = TFT_H - y;

  uint8_t hi = color >> 8, lo = color & 0xFF;
  tftSetAddrWindow(x, y, x + w - 1, y + h - 1);
  dcData();
  digitalWrite(TFT_CS, LOW);
  for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
    SPI.transfer(hi);
    SPI.transfer(lo);
  }
  digitalWrite(TFT_CS, HIGH);
}
#else  // ST7735
static void tftFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  if (x >= TFT_W || y >= TFT_H) return;
  if (x + w > TFT_W) w = TFT_W - x;
  if (y + h > TFT_H) h = TFT_H - y;

  // RGB565 -> RGB888
  uint8_t r = (color >> 8) & 0xF8;
  uint8_t g = (color >> 3) & 0xFC;
  uint8_t b = (color << 3) & 0xF8;

  tftSetAddrWindow(x, y, x + w - 1, y + h - 1);
  dcData();
  digitalWrite(TFT_CS, LOW);
  for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }
  digitalWrite(TFT_CS, HIGH);
}
#endif

// ---------- Frame描画 ----------
// Orientation: 0=normal, 1=90°CW(左傾き), 2=90°CCW(右傾き), 3=180°(上傾き)

// 回転に応じた開始X座標
static inline uint16_t getStartX(uint8_t orient) {
  if (orient == 1) return 0;                    // 90°CW: 左端
  if (orient == 2) return TFT_W - IMG_W;        // 90°CCW: 右端
  return (TFT_W - IMG_W) / 2;                   // 0°/180°: 中央
}

// 回転に応じたソースピクセル座標変換
static inline uint16_t getSrcIndex(uint8_t x, uint8_t y, uint8_t orient, uint8_t w, uint8_t h) {
  uint8_t srcX, srcY;
  switch (orient) {
    case 1:  srcX = y; srcY = (h - 1) - x; break;        // 90°CW
    case 2:  srcX = (w - 1) - y; srcY = x; break;        // 90°CCW
    case 3:  srcX = (w - 1) - x; srcY = (h - 1) - y; break; // 180°
    default: srcX = x; srcY = y; break;                  // 0°
  }
  return (uint16_t)srcY * w + srcX;
}

// 4-bit packed array から palette index を読み出す
static inline uint8_t read4bit(const uint8_t* data, uint16_t pixelIdx) {
  uint8_t b = pgm_read_byte(&data[pixelIdx >> 1]);
  return (pixelIdx & 1) ? (b & 0x0F) : (b >> 4);
}

// palette index から色を転送
static inline void transferColor(uint16_t color) {
#if defined(USE_SSD1351) || defined(USE_SSD1331)
  SPI.transfer(color >> 8);
  SPI.transfer(color & 0xFF);
#else
  SPI.transfer((color >> 8) & 0xF8);
  SPI.transfer((color >> 3) & 0xFC);
  SPI.transfer((color << 3) & 0xF8);
#endif
}

// フルフレーム描画
static void drawFullFrame(const uint8_t* data, uint8_t orient) {
  uint16_t screenX = getStartX(orient);
  for (uint8_t dy = 0; dy < IMG_H; dy++) {
    tftSetAddrWindow(screenX, dy, screenX + IMG_W - 1, dy);
    dcData();
    digitalWrite(TFT_CS, LOW);
    for (uint8_t dx = 0; dx < IMG_W; dx++) {
      uint16_t srcIdx = getSrcIndex(dx, dy, orient, IMG_W, IMG_H);
      uint8_t idx = read4bit(data, srcIdx);
      transferColor(pgm_read_word(&palette[idx]));
    }
    digitalWrite(TFT_CS, HIGH);
  }
}

// オーバーレイ領域描画 (overlay data は srcW x srcH ピクセル)
static void drawOverlay(const uint8_t* data, uint8_t orient,
                        uint8_t srcX, uint8_t srcY, uint8_t srcW, uint8_t srcH) {
  uint8_t dstX, dstY, dstW, dstH;
  switch (orient) {
    case 1:  dstX = (IMG_H-1)-(srcY+srcH-1); dstY = srcX; dstW = srcH; dstH = srcW; break;
    case 2:  dstX = srcY; dstY = (IMG_W-1)-(srcX+srcW-1); dstW = srcH; dstH = srcW; break;
    case 3:  dstX = (IMG_W-1)-(srcX+srcW-1); dstY = (IMG_H-1)-(srcY+srcH-1); dstW = srcW; dstH = srcH; break;
    default: dstX = srcX; dstY = srcY; dstW = srcW; dstH = srcH; break;
  }
  uint16_t screenX = getStartX(orient) + dstX;
  for (uint8_t dy = 0; dy < dstH; dy++) {
    tftSetAddrWindow(screenX, dstY + dy, screenX + dstW - 1, dstY + dy);
    dcData();
    digitalWrite(TFT_CS, LOW);
    for (uint8_t dx = 0; dx < dstW; dx++) {
      uint16_t srcIdx = getSrcIndex(dx, dy, orient, srcW, srcH);
      uint8_t idx = read4bit(data, srcIdx);
      transferColor(pgm_read_word(&palette[idx]));
    }
    digitalWrite(TFT_CS, HIGH);
  }
}

// フルフレームから指定領域を描画 (ソース座標系)
static void drawRegionFromFull(const uint8_t* fullData, uint8_t orient,
                                uint8_t rx, uint8_t ry, uint8_t rw, uint8_t rh) {
  uint8_t dstX, dstY, dstW, dstH;
  switch (orient) {
    case 1:  dstX = (IMG_H-1)-(ry+rh-1); dstY = rx; dstW = rh; dstH = rw; break;
    case 2:  dstX = ry; dstY = (IMG_W-1)-(rx+rw-1); dstW = rh; dstH = rw; break;
    case 3:  dstX = (IMG_W-1)-(rx+rw-1); dstY = (IMG_H-1)-(ry+rh-1); dstW = rw; dstH = rh; break;
    default: dstX = rx; dstY = ry; dstW = rw; dstH = rh; break;
  }
  uint16_t screenX = getStartX(orient) + dstX;
  for (uint8_t dy = 0; dy < dstH; dy++) {
    tftSetAddrWindow(screenX, dstY + dy, screenX + dstW - 1, dstY + dy);
    dcData();
    digitalWrite(TFT_CS, LOW);
    for (uint8_t dx = 0; dx < dstW; dx++) {
      uint16_t srcIdx = getSrcIndex(dstX + dx, dstY + dy, orient, IMG_W, IMG_H);
      uint8_t idx = read4bit(fullData, srcIdx);
      transferColor(pgm_read_word(&palette[idx]));
    }
    digitalWrite(TFT_CS, HIGH);
  }
}

// 余白を黒で塗りつぶす
static void clearMargins(uint8_t orient) {
  uint16_t imgX = getStartX(orient);
  uint16_t rightX = imgX + IMG_W;
  for (uint8_t y = 0; y < TFT_H; y++) {
    // 左余白
    if (imgX > 0) {
      tftSetAddrWindow(0, y, imgX - 1, y);
      dcData();
      digitalWrite(TFT_CS, LOW);
      for (uint8_t x = 0; x < imgX; x++) {
        SPI.transfer(0x00); SPI.transfer(0x00);
      }
      digitalWrite(TFT_CS, HIGH);
    }
    // 右余白
    if (rightX < TFT_W) {
      tftSetAddrWindow(rightX, y, TFT_W - 1, y);
      dcData();
      digitalWrite(TFT_CS, LOW);
      for (uint8_t x = rightX; x < TFT_W; x++) {
        SPI.transfer(0x00); SPI.transfer(0x00);
      }
      digitalWrite(TFT_CS, HIGH);
    }
  }
}

// 現在のフルフレームデータを取得 (overlay→ref辿り)
static const uint8_t* getActiveFullFrameData(uint8_t frameIdx) {
  uint8_t ftype = pgm_read_byte(&frames[frameIdx].type);
  if (ftype == 0) return (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);
  uint8_t ref = pgm_read_byte(&frames[frameIdx].ref);
  return (const uint8_t*)pgm_read_ptr(&frames[ref].data);
}

// 統合描画 (type判定、ref参照、差分最適化)
static void drawCurrentFrame(uint8_t frameIdx, uint8_t orient) {
  uint8_t curType = pgm_read_byte(&frames[frameIdx].type);
  const uint8_t* curData = (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);

  if (lastDrawnFrame < 0 || orient != currentOrient) {
    clearMargins(orient);
    currentOrient = orient;
    if (curType == 0) {
      drawFullFrame(curData, orient);
    } else {
      uint8_t curRef = pgm_read_byte(&frames[frameIdx].ref);
      drawFullFrame((const uint8_t*)pgm_read_ptr(&frames[curRef].data), orient);
      drawOverlay(curData, orient,
        pgm_read_byte(&frames[frameIdx].rx), pgm_read_byte(&frames[frameIdx].ry),
        pgm_read_byte(&frames[frameIdx].rw), pgm_read_byte(&frames[frameIdx].rh));
    }
    lastDrawnFrame = frameIdx;
    return;
  }

  // TODO: これがないと青い線が出る。原因未解明
  clearMargins(orient);

  if ((uint8_t)lastDrawnFrame == frameIdx) return;

  uint8_t prevIdx = (uint8_t)lastDrawnFrame;
  uint8_t prevType = pgm_read_byte(&frames[prevIdx].type);

  if (curType == 0) {
    // フル←オーバーレイ: overlay領域だけ復元
    if (prevType == 1 && pgm_read_byte(&frames[prevIdx].ref) == frameIdx) {
      drawRegionFromFull(curData, orient,
        pgm_read_byte(&frames[prevIdx].rx), pgm_read_byte(&frames[prevIdx].ry),
        pgm_read_byte(&frames[prevIdx].rw), pgm_read_byte(&frames[prevIdx].rh));
    } else {
      drawFullFrame(curData, orient);
    }
  } else {
    uint8_t curRef = pgm_read_byte(&frames[frameIdx].ref);
    uint8_t rx = pgm_read_byte(&frames[frameIdx].rx);
    uint8_t ry = pgm_read_byte(&frames[frameIdx].ry);
    uint8_t rw = pgm_read_byte(&frames[frameIdx].rw);
    uint8_t rh = pgm_read_byte(&frames[frameIdx].rh);

    if (prevType == 0 && curRef == prevIdx) {
      // フル→その上にoverlay
      drawOverlay(curData, orient, rx, ry, rw, rh);
    } else if (prevType == 1 && pgm_read_byte(&frames[prevIdx].ref) == curRef) {
      // overlay→同refの別overlay: 前のregionが違えば復元
      uint8_t prevRx = pgm_read_byte(&frames[prevIdx].rx);
      uint8_t prevRy = pgm_read_byte(&frames[prevIdx].ry);
      uint8_t prevRw = pgm_read_byte(&frames[prevIdx].rw);
      uint8_t prevRh = pgm_read_byte(&frames[prevIdx].rh);
      if (prevRx != rx || prevRy != ry || prevRw != rw || prevRh != rh) {
        drawRegionFromFull((const uint8_t*)pgm_read_ptr(&frames[curRef].data),
          orient, prevRx, prevRy, prevRw, prevRh);
      }
      drawOverlay(curData, orient, rx, ry, rw, rh);
    } else {
      // 異なるコンテキスト: ref全描画+overlay
      drawFullFrame((const uint8_t*)pgm_read_ptr(&frames[curRef].data), orient);
      drawOverlay(curData, orient, rx, ry, rw, rh);
    }
  }

  lastDrawnFrame = frameIdx;
}

#if defined(USE_SSD1351) || defined(USE_SSD1331)
// 全コントラスト設定 (SSD1331用)
static void oledSetAllContrast(uint8_t a, uint8_t b, uint8_t c, uint8_t master) {
  tftWriteCommand(0x81); tftWriteCommand(a);       // Contrast A
  tftWriteCommand(0x82); tftWriteCommand(b);       // Contrast B
  tftWriteCommand(0x83); tftWriteCommand(c);       // Contrast C
  tftWriteCommand(0x87); tftWriteCommand(master);  // Master current
}

// フェードトランジション (向き変更時)
static void fadeTransition(uint8_t newOrient) {
  const uint8_t steps = 8;

  // フェードアウト
  for (int i = steps; i >= 0; i--) {
    uint8_t a = (0x91 * i) / steps;
    uint8_t b = (0x50 * i) / steps;
    uint8_t c = (0x7D * i) / steps;
    uint8_t m = (0x06 * i) / steps;
    oledSetAllContrast(a, b, c, m);
    delay(25);
  }

  // リセット+再初期化 (暗い状態で)
  oledHardReset();
  oledInitRegisters();
  oledSetAllContrast(0, 0, 0, 0);

  currentOrient = newOrient;
  curFrame = 0;
  lastDrawnFrame = -1;
  tftFillScreen888(0, 0, 0);

  // フェードイン
  for (int i = 0; i <= steps; i++) {
    uint8_t a = (0x91 * i) / steps;
    uint8_t b = (0x50 * i) / steps;
    uint8_t c = (0x7D * i) / steps;
    uint8_t m = (0x06 * i) / steps;
    oledSetAllContrast(a, b, c, m);
    delay(25);
  }
}
#endif

// ---------- Display Init ----------
#if defined(USE_SSD1351)
static void oledDisplayOn() {
  tftWriteCommandData(0xAF, nullptr, 0);
}

static void oledDisplayOff() {
  tftWriteCommandData(0xAE, nullptr, 0);
  delay(10);  // Wait for SPI completion
}

static void oledSetContrast(uint8_t level) {
  uint8_t d[] = { (uint8_t)(level & 0x0F) };
  tftWriteCommandData(0xC7, d, 1);
}

static void tftInit() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

  { const uint8_t d[] = { 0x12 }; tftWriteCommandData(0xFD, d, 1); }
  { const uint8_t d[] = { 0xB1 }; tftWriteCommandData(0xFD, d, 1); }
  tftWriteCommandData(0xAE, nullptr, 0);
  delay(10);
  { const uint8_t d[] = { 0xF1 }; tftWriteCommandData(0xB3, d, 1); }
  { const uint8_t d[] = { 0x7F }; tftWriteCommandData(0xCA, d, 1); }
  { const uint8_t d[] = { 0x74 }; tftWriteCommandData(0xA0, d, 1); }
  { const uint8_t d[] = { 0x00 }; tftWriteCommandData(0xA1, d, 1); }
  { const uint8_t d[] = { 0x00 }; tftWriteCommandData(0xA2, d, 1); }
  { const uint8_t d[] = { 0x00 }; tftWriteCommandData(0xB5, d, 1); }
  { const uint8_t d[] = { 0x01 }; tftWriteCommandData(0xAB, d, 1); }
  { const uint8_t d[] = { 0x32 }; tftWriteCommandData(0xB1, d, 1); }
  { const uint8_t d[] = { 0x17 }; tftWriteCommandData(0xBB, d, 1); }
  { const uint8_t d[] = { 0x05 }; tftWriteCommandData(0xBE, d, 1); }
  { const uint8_t d[] = { 0xC8, 0x80, 0xC8 }; tftWriteCommandData(0xC1, d, 3); }
  { const uint8_t d[] = { 0x0F }; tftWriteCommandData(0xC7, d, 1); }
  { const uint8_t d[] = { 0x01 }; tftWriteCommandData(0xB6, d, 1); }
  tftWriteCommandData(0xA6, nullptr, 0);
  tftWriteCommandData(0xAF, nullptr, 0);
  delay(100);

  tftFillScreen888(0, 0, 0);
}
#elif defined(USE_SSD1331)
static void oledDisplayOn() {
  tftWriteCommand(0xAF);                          // Display ON
  tftWriteCommand(0x87); tftWriteCommand(0x06);  // Master current = 6 (restore)
}

static void oledDisplayOff() {
  tftWriteCommand(0x87); tftWriteCommand(0x00);  // Master current = 0
  tftWriteCommand(0xAE);                          // Display OFF
  delay(10);
}

static void oledSetContrast(uint8_t level) {
  // SSD1331: Master Current Control (0x87), 0-15
  // Note: SSD1331 requires all bytes sent with D/C=LOW
  tftWriteCommand(0x87);
  tftWriteCommand(level & 0x0F);
}

static void oledHardReset() {
  digitalWrite(TFT_RST, LOW);
  delay(10);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
}

static void oledInitRegisters() {
  tftWriteCommand(0xAE);  // Display OFF
  delay(100);
  tftWriteCommand(0xA0); tftWriteCommand(0x72);  // Remap: 65k, RGB, scan/column remap
  tftWriteCommand(0xA1); tftWriteCommand(0x00);  // Start line 0
  tftWriteCommand(0xA2); tftWriteCommand(0x00);  // Display offset 0
  tftWriteCommand(0xA4);  // Normal display
  tftWriteCommand(0xA8); tftWriteCommand(0x3F);  // MUX ratio: 64
  tftWriteCommand(0xAD); tftWriteCommand(0x8E);  // Master config
  tftWriteCommand(0xB0); tftWriteCommand(0x0B);  // Power save OFF
  tftWriteCommand(0xB1); tftWriteCommand(0x31);  // Phase period adj
  tftWriteCommand(0xB3); tftWriteCommand(0xC0);  // Clock divider
  tftWriteCommand(0x8A); tftWriteCommand(0x64);  // Precharge A
  tftWriteCommand(0x8B); tftWriteCommand(0x78);  // Precharge B
  tftWriteCommand(0x8C); tftWriteCommand(0x64);  // Precharge C
  tftWriteCommand(0xBB); tftWriteCommand(0x3A);  // Precharge level
  tftWriteCommand(0xBE); tftWriteCommand(0x3E);  // VCOMH
  tftWriteCommand(0x87); tftWriteCommand(0x06);  // Master current
  tftWriteCommand(0x81); tftWriteCommand(0x91);  // Contrast A
  tftWriteCommand(0x82); tftWriteCommand(0x50);  // Contrast B
  tftWriteCommand(0x83); tftWriteCommand(0x7D);  // Contrast C
  tftWriteCommand(0xAF);  // Display ON
  delay(100);
}

static void tftInit() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH);
  delay(10);
  oledHardReset();

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  oledInitRegisters();
  tftFillScreen888(0, 0, 0);  // Clear to black
}
#else  // ST7735
static void tftInit() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();

  tftWriteCommandData(0x01, nullptr, 0); delay(150);
  tftWriteCommandData(0x11, nullptr, 0); delay(150);
  { const uint8_t d[] = { 0x06 }; tftWriteCommandData(0x3A, d, 1); }
  delay(10);
  { const uint8_t d[] = { 0x00 }; tftWriteCommandData(0x36, d, 1); }
  delay(10);
  tftWriteCommandData(0x20, nullptr, 0); delay(10);
  tftWriteCommandData(0x13, nullptr, 0); delay(10);
  tftWriteCommandData(0x29, nullptr, 0); delay(120);

  pinMode(TFT_BL, OUTPUT);
  backlightFull();
  tftFillScreen888(0, 0, 0);
}
#endif

// ---------- KXTJ3 ----------
static bool readXYZ_raw(int16_t &x, int16_t &y, int16_t &z) {
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

// ---------- motion ----------
static float pos_x=0, pos_y=0, vel_x=0, vel_y=0;
static int16_t x_ref=0, y_ref=0, z_ref=0;
static uint8_t calibrated=0;
static float fdx=0, fdy=0;
static int prev_cx=-9999, prev_cy=-9999;

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static void calibrateKXTJ3(uint8_t samples=80) {
  long sx=0, sy=0, sz=0;
  uint8_t got=0;
  for (uint8_t i=0; i<samples; i++) {
    int16_t rx, ry, rz;
    if (readXYZ_raw(rx, ry, rz)) { sx += rx; sy += ry; sz += rz; got++; }
    delay(10);
  }
  if (!got) return;
  x_ref = (int16_t)(sx / got);
  y_ref = (int16_t)(sy / got);
  z_ref = (int16_t)(sz / got);
  fdx = fdy = 0;
  calibrated = 1;
}

// 加速度センサーから傾き方向を検出 (ヒステリシス付き)
static uint8_t detectOrientation() {
  int16_t rx, ry, rz;
  if (!calibrated || !readXYZ_raw(rx, ry, rz)) return currentOrient;

  int16_t dx = (x_ref - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - y_ref) * ACCEL_Y_SIGN;

  const int16_t thresholdIn = 5000;   // 入る閾値
  const int16_t thresholdOut = 3000;  // 出る閾値

  int16_t absDx = dx > 0 ? dx : -dx;
  int16_t absDy = dy > 0 ? dy : -dy;
  int16_t maxTilt = absDx > absDy ? absDx : absDy;

  // 現在の向きを維持するか判定
  if (maxTilt > thresholdOut) {
    switch (currentOrient) {
      case 1:  // 左傾き: dx が支配的で負
        if (absDx > absDy && dx < 0) return 1;
        break;
      case 2:  // 右傾き: dx が支配的で正
        if (absDx > absDy && dx > 0) return 2;
        break;
      case 3:  // 上傾き: dy が支配的で負
        if (absDy > absDx && dy < 0) return 3;
        break;
      case 0:  // 下/水平: dy が支配的で正、または傾き小
        if (absDy > absDx && dy > 0) return 0;
        break;
    }
  } else {
    // 傾きが小さい場合は現在の向きを維持
    return currentOrient;
  }

  // 新しい向きへの切り替え判定
  if (maxTilt > thresholdIn) {
    if (absDx > absDy) {
      // X方向が支配的
      return (dx > 0) ? 2 : 1;  // 右 or 左
    } else {
      // Y方向が支配的
      return (dy < 0) ? 3 : 0;  // 上 or 下
    }
  }

  return currentOrient;
}

// 「上」方向インジケーター (4ピクセルの赤い点、傾きで位置が変わる)
static int16_t prevIndX = -1, prevIndY = -1;

// 指定位置の元画像ピクセルを復元
static void restorePixel(int16_t sx, int16_t sy) {
  if (sx < 0 || sx >= TFT_W || sy < 0 || sy >= TFT_H) return;

  uint16_t imgX = getStartX(currentOrient);
  uint16_t color;

  // 画像領域内かチェック
  if (sx >= imgX && sx < imgX + IMG_W && sy < IMG_H) {
    uint8_t localX = sx - imgX;
    uint8_t localY = sy;
    uint16_t srcIdx = getSrcIndex(localX, localY, currentOrient, IMG_W, IMG_H);
    const uint8_t* fullData = getActiveFullFrameData(curFrame);
    uint8_t palIdx = read4bit(fullData, srcIdx);
    color = pgm_read_word(&palette[palIdx]);
  } else {
    color = 0x0000;  // 黒 (余白)
  }

  tftSetAddrWindow(sx, sy, sx, sy);
  dcData();
  digitalWrite(TFT_CS, LOW);
#if defined(USE_SSD1351) || defined(USE_SSD1331)
  SPI.transfer(color >> 8);
  SPI.transfer(color & 0xFF);
#else
  SPI.transfer((color >> 8) & 0xF8);
  SPI.transfer((color >> 3) & 0xFC);
  SPI.transfer((color << 3) & 0xF8);
#endif
  digitalWrite(TFT_CS, HIGH);
}

static void drawUpIndicator() {
  int16_t rx, ry, rz;
  if (!calibrated || !readXYZ_raw(rx, ry, rz)) return;

  int16_t dx = (x_ref - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - y_ref) * ACCEL_Y_SIGN;

  // 傾きを直接位置にマッピング (水準器の泡)
  const int16_t maxTilt = 8000;
  int16_t posX = TFT_W / 2 - (dx * (TFT_W / 2 - 4)) / maxTilt;
  int16_t posY = TFT_H / 2 - (dy * (TFT_H / 2 - 4)) / maxTilt;

  // 画面内にクランプ
  if (posX < 2) posX = 2;
  if (posX > TFT_W - 4) posX = TFT_W - 4;
  if (posY < 2) posY = 2;
  if (posY > TFT_H - 4) posY = TFT_H - 4;

  // 前の位置を復元 (2x2)
  if (prevIndX >= 0 && (prevIndX != posX || prevIndY != posY)) {
    for (int dy = 0; dy < 2; dy++) {
      for (int dx = 0; dx < 2; dx++) {
        restorePixel(prevIndX + dx, prevIndY + dy);
      }
    }
  }

  // 赤色 (RGB565: 0xF800)
  const uint16_t red = 0xF800;
  uint8_t hi = red >> 8, lo = red & 0xFF;

  // 4ピクセル描画 (2x2)
  tftSetAddrWindow(posX, posY, posX + 1, posY + 1);
  dcData();
  digitalWrite(TFT_CS, LOW);
#if defined(USE_SSD1351) || defined(USE_SSD1331)
  for (int i = 0; i < 4; i++) {
    SPI.transfer(hi); SPI.transfer(lo);
  }
#else
  for (int i = 0; i < 4; i++) {
    SPI.transfer((red >> 8) & 0xF8);
    SPI.transfer((red >> 3) & 0xFC);
    SPI.transfer((red << 3) & 0xF8);
  }
#endif
  digitalWrite(TFT_CS, HIGH);

  prevIndX = posX;
  prevIndY = posY;
}

// HSI 8MHz, PLLなし (省電力: CR2032でOLED輝度確保のため)
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

void setup() {
  tftInit();
  tftFillScreen888(255, 255, 255);

  // 加速度センサー初期化
  Wire.setSDA(PB_7);
  Wire.setSCL(PB_6);
  Wire.begin();
  Wire.setClock(100000);

  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  write8(REG_CTRL1, 0xC0); delay(50);  // 動作モード (12bit, 50Hz)

  calibrateKXTJ3(20);
  enableWakeupInterrupt();

  attachInterrupt(digitalPinToInterrupt(KXTJ3_INT_PIN), wakeupCallback, FALLING);

#if defined(USE_SSD1331) || defined(USE_SSD1351)
  pinMode(STAT_LED_PIN, OUTPUT);
  digitalWrite(STAT_LED_PIN, LOW);
#endif

  curFrame = 0;
  lastDrawnFrame = -1;
  tftFillScreen888(0, 0, 0);
  lastActivityMs = millis();
  lastFrameTime = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - lastActivityMs;

  // スリープ
  if (elapsed >= SLEEP_TIMEOUT_MS) {
#if defined(USE_SSD1331) || defined(USE_SSD1351)
    digitalWrite(STAT_LED_PIN, LOW);
#endif
    backlightOff();          // Display OFF + Power Save ON
    clearLatchedInterrupt();
    HAL_SuspendTick();
    __DSB();                 // Data Synchronization Barrier (ARM requirement before WFI)
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    backlightFull();
    lastActivityMs = millis();
    lastFrameTime = millis();
    curFrame = 0;
    lastDrawnFrame = -1;
    tftFillScreen888(0, 0, 0);
    return;
  }

  // ディム
  if (elapsed >= DIM_TIMEOUT_MS) {
    backlightDim();
#if defined(USE_SSD1331) || defined(USE_SSD1351)
    digitalWrite(STAT_LED_PIN, LOW);
#endif
  } else {
    backlightFull();
#if defined(USE_SSD1331) || defined(USE_SSD1351)
    digitalWrite(STAT_LED_PIN, HIGH);
#endif
  }

  // 傾き検出
  uint8_t orient = detectOrientation();
  if (orient != currentOrient) {
    lastActivityMs = now;
#if defined(USE_SSD1351) || defined(USE_SSD1331)
    // Dim状態でなければフェードトランジション
    if (elapsed < DIM_TIMEOUT_MS) {
      fadeTransition(orient);
      lastFrameTime = now;
      return;
    }
#endif
  }

  // フレーム進行
  uint16_t duration = pgm_read_word(&frames[curFrame].duration_ms);
  if (now - lastFrameTime >= duration) {
    lastFrameTime = now;
    curFrame = pgm_read_byte(&frames[curFrame].next);
  }

  // 描画 (変化がなければ内部で早期リターン)
  if (lastDrawnFrame < 0 || (uint8_t)lastDrawnFrame != curFrame || orient != currentOrient) {
    drawCurrentFrame(curFrame, orient);
  }

  // 上方向インジケーター
  drawUpIndicator();
}
