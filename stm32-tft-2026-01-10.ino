// NOTE: Build with "Tools > U(S)ART support > Disabled" to fit in 32KB flash
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

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

#if defined(USE_SSD1351)
// SSD1351 OLED 128x128
static constexpr int TFT_W = 128;
static constexpr int TFT_H = 128;
static constexpr uint8_t X_OFFSET = 0;
static constexpr uint8_t Y_OFFSET = 0;
// 加速度センサー座標反転 (SSD1351用)
static constexpr int ACCEL_X_SIGN = -1;
static constexpr int ACCEL_Y_SIGN = -1;
#elif defined(USE_SSD1331)
// SSD1331 OLED 96x64
static constexpr int TFT_W = 96;
static constexpr int TFT_H = 64;
static constexpr uint8_t X_OFFSET = 0;
static constexpr uint8_t Y_OFFSET = 0;
// 加速度センサー座標反転 (SSD1331用 - 要調整)
static constexpr int ACCEL_X_SIGN = -1;
static constexpr int ACCEL_Y_SIGN = -1;
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

// =====================
// 画像データ (64x64, blink animation with eye region overlay)
// =====================
#include "image_data.h"
static const uint8_t IMG_SCALE = 1;
static const uint8_t IMG_FRAMES = 5;  // blink sequence: 0->1->2->1->0

// Blink sequence: base -> half -> closed -> half -> base
static const uint8_t blinkSequence[5] = { 0, 1, 2, 1, 0 };

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

static inline void backlightFull() { oledDisplayOn(); oledSetContrast(0x0F); }
static inline void backlightDim()  { oledSetContrast(0x05); }
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

// ---------- 画像描画 (瞬きアニメーション対応) ----------
// Orientation: 0=normal, 1=90°CW(左傾き), 2=90°CCW(右傾き), 3=180°(上傾き)
static uint8_t currentOrient = 0;
static int8_t lastDrawnFrame = -1;  // -1 = 未描画

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

// ベースフレームの指定領域を描画
static void drawBaseFrameRegion(uint8_t orient, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  uint16_t screenX = getStartX(orient) + x;
  uint16_t screenY = y;

  for (uint8_t dy = 0; dy < h; dy++) {
    tftSetAddrWindow(screenX, screenY + dy, screenX + w - 1, screenY + dy);
    dcData();
    digitalWrite(TFT_CS, LOW);
    for (uint8_t dx = 0; dx < w; dx++) {
      uint8_t idx = pgm_read_byte(&baseFrame[getSrcIndex(x + dx, y + dy, orient, IMG_W, IMG_H)]);
      uint16_t color = pgm_read_word(&basePalette[idx]);
#if defined(USE_SSD1351) || defined(USE_SSD1331)
      SPI.transfer(color >> 8);
      SPI.transfer(color & 0xFF);
#else
      SPI.transfer((color >> 8) & 0xF8);
      SPI.transfer((color >> 3) & 0xFC);
      SPI.transfer((color << 3) & 0xF8);
#endif
    }
    digitalWrite(TFT_CS, HIGH);
  }
}

// ベースフレーム全体を描画
static void drawBaseFrame(uint8_t orient) {
  drawBaseFrameRegion(orient, 0, 0, IMG_W, IMG_H);
}

// 目領域の描画先座標とサイズを取得
static void getEyeImageRect(uint8_t orient, uint8_t& x, uint8_t& y, uint8_t& w, uint8_t& h) {
  switch (orient) {
    case 1:  x = 63 - (EYE_Y + EYE_H - 1); y = EYE_X; w = EYE_H; h = EYE_W; break;
    case 2:  x = EYE_Y; y = 63 - (EYE_X + EYE_W - 1); w = EYE_H; h = EYE_W; break;
    case 3:  x = 63 - (EYE_X + EYE_W - 1); y = 63 - (EYE_Y + EYE_H - 1); w = EYE_W; h = EYE_H; break;
    default: x = EYE_X; y = EYE_Y; w = EYE_W; h = EYE_H; break;
  }
}


// 目領域のみ描画
static void drawEyeRegion(uint8_t orient, const uint8_t* eyeData, const uint16_t* palette) {
  uint8_t x, y, w, h;
  getEyeImageRect(orient, x, y, w, h);
  uint16_t screenX = getStartX(orient) + x;
  uint16_t screenY = y;

  for (uint8_t dy = 0; dy < h; dy++) {
    tftSetAddrWindow(screenX, screenY + dy, screenX + w - 1, screenY + dy);
    dcData();
    digitalWrite(TFT_CS, LOW);
    for (uint8_t dx = 0; dx < w; dx++) {
      uint8_t idx = pgm_read_byte(&eyeData[getSrcIndex(dx, dy, orient, EYE_W, EYE_H)]);
      uint16_t color = pgm_read_word(&palette[idx]);
#if defined(USE_SSD1351) || defined(USE_SSD1331)
      SPI.transfer(color >> 8);
      SPI.transfer(color & 0xFF);
#else
      SPI.transfer((color >> 8) & 0xF8);
      SPI.transfer((color >> 3) & 0xFC);
      SPI.transfer((color << 3) & 0xF8);
#endif
    }
    digitalWrite(TFT_CS, HIGH);
  }
}

// ベース画像の目領域を復元
static void restoreBaseEyeRegion(uint8_t orient) {
  uint8_t x, y, w, h;
  getEyeImageRect(orient, x, y, w, h);
  drawBaseFrameRegion(orient, x, y, w, h);
}

// 余白を白で塗りつぶす
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
        SPI.transfer(0xFF); SPI.transfer(0xFF);
      }
      digitalWrite(TFT_CS, HIGH);
    }
    // 右余白
    if (rightX < TFT_W) {
      tftSetAddrWindow(rightX, y, TFT_W - 1, y);
      dcData();
      digitalWrite(TFT_CS, LOW);
      for (uint8_t x = rightX; x < TFT_W; x++) {
        SPI.transfer(0xFF); SPI.transfer(0xFF);
      }
      digitalWrite(TFT_CS, HIGH);
    }
  }
}

// フレーム描画: blinkState 0=目開き, 1=半目, 2=閉じ
static void drawFrame(uint8_t seqIdx, uint8_t orient) {
  uint8_t blinkState = blinkSequence[seqIdx];

  if (lastDrawnFrame < 0 || orient != currentOrient) {
    clearMargins(orient);
    drawBaseFrame(orient);
    currentOrient = orient;
  }

  // TODO: これがないと青い線が出る。原因未解明
  clearMargins(orient);

  if (blinkState == 0) {
    restoreBaseEyeRegion(orient);
  } else if (blinkState == 1) {
    drawEyeRegion(orient, eyeHalf, eyeHalfPalette);
  } else {
    drawEyeRegion(orient, eyeClosed, eyeClosedPalette);
  }

  lastDrawnFrame = seqIdx;
}

static uint8_t currentFrame = 0;

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
  // 初期値: A=0x91, B=0x50, C=0x7D, Master=0x06
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

  // 完全消灯
  tftWriteCommand(0xAE);  // Display OFF
  delay(50);

  // 新しい向きで描画
  drawFrame(currentFrame, newOrient);

  // Display ON して フェードイン
  tftWriteCommand(0xAF);  // Display ON
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

static void tftInit() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // SSD1331 init: all bytes sent with D/C=LOW (like Adafruit)
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
  tftWriteCommand(0xB3); tftWriteCommand(0xF0);  // Clock divider
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

static uint32_t lastFrameTime = 0;
static const uint32_t FRAME_INTERVAL_MS = 200;  // 200ms per frame

// 加速度センサーから傾き方向を検出
static uint8_t detectOrientation() {
  int16_t rx, ry, rz;
  if (!calibrated || !readXYZ_raw(rx, ry, rz)) return currentOrient;

  int16_t dx = (x_ref - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - y_ref) * ACCEL_Y_SIGN;

  const int16_t threshold = 5000;

  // X方向優先
  if (dx > threshold) return 2;       // 右傾き → 90°CCW
  if (dx < -threshold) return 1;      // 左傾き → 90°CW

  // Y方向
  if (dy < -threshold) return 3;      // 上傾き → 180°

  return 0;  // 下傾きまたは水平 → 通常
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

  drawFrame(0, detectOrientation());
  lastActivityMs = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - lastActivityMs;

  // スリープ
  if (elapsed >= SLEEP_TIMEOUT_MS) {
    backlightOff();          // Display OFF + Power Save ON
    clearLatchedInterrupt();
    HAL_SuspendTick();
    __DSB();                 // Data Synchronization Barrier (ARM requirement before WFI)
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    backlightFull();
    lastActivityMs = millis();
    lastFrameTime = millis();
    lastDrawnFrame = -1;  // 再描画
    return;
  }

  // ディム
  if (elapsed >= DIM_TIMEOUT_MS) {
    backlightDim();
  } else {
    backlightFull();
  }

  // 傾き検出と描画
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

  if (now - lastFrameTime >= FRAME_INTERVAL_MS) {
    lastFrameTime = now;
    currentFrame = (currentFrame + 1) % IMG_FRAMES;
    drawFrame(currentFrame, orient);
  }

  /* ボールデモ用 (未使用)
  uint32_t elapsed = millis() - lastActivityMs;

  if (elapsed >= SLEEP_TIMEOUT_MS) {
    backlightOff();
    clearLatchedInterrupt();
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    backlightFull();
    lastActivityMs = millis();
    return;
  }

  if (elapsed >= DIM_TIMEOUT_MS) {
    backlightDim();
  } else {
    backlightFull();
  }

  int16_t rx, ry, rz;
  if (!calibrated || !readXYZ_raw(rx, ry, rz)) return;

  int16_t dx = (x_ref - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - y_ref) * ACCEL_Y_SIGN;

  const float alpha = 0.20f;
  fdx = (1.0f - alpha) * fdx + alpha * (float)dx;
  fdy = (1.0f - alpha) * fdy + alpha * (float)dy;

  const float dead = 5000.0f;
  float ux = (fdx > -dead && fdx < dead) ? 0.0f : fdx;
  float uy = (fdy > -dead && fdy < dead) ? 0.0f : fdy;

  if (ux != 0.0f || uy != 0.0f) {
    lastActivityMs = millis();
  }

  const float accel_gain = 0.0008f;
  vel_x += ux * accel_gain;
  vel_y += uy * accel_gain;

  const float friction = 0.92f;
  vel_x *= friction;
  vel_y *= friction;

  const float v_max = 3.0f;
  vel_x = clampf(vel_x, -v_max, v_max);
  vel_y = clampf(vel_y, -v_max, v_max);

  pos_x += vel_x;
  pos_y += vel_y;

  const int r = 10;
  const float x_min = r, y_min = r;
  const float x_max = TFT_W - 1 - r;
  const float y_max = TFT_H - 1 - r;

  if (pos_x < x_min) { pos_x = x_min; vel_x = -vel_x; }
  if (pos_x > x_max) { pos_x = x_max; vel_x = -vel_x; }
  if (pos_y < y_min) { pos_y = y_min; vel_y = -vel_y; }
  if (pos_y > y_max) { pos_y = y_max; vel_y = -vel_y; }

  int cx = (int)(pos_x + 0.5f);
  int cy = (int)(pos_y + 0.5f);

  if (prev_cx > -1000) {
    tftFillCircle888(prev_cx, prev_cy, r + 1, 0, 0, 0);
  }

  tftFillCircle888(cx, cy, r, 255, 0, 0);
  int hr = (r/5) ? (r/5) : 1;
  tftFillCircle888(cx - r/3, cy - r/3, hr, 255, 255, 255);

  prev_cx = cx;
  prev_cy = cy;

  delay(30);
  */
}
