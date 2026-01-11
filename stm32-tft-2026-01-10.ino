#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// =====================
// TFT配線（TFT表記準拠）
//   VCC -> 3.3V
//   GND -> GND
//   SCL -> PA_5 (SPI SCK)
//   SDA -> PA_7 (SPI MOSI)
//   RES -> VCC直結（コードでは未使用）
//   DC  -> PA_1（例：混乱しないGPIOへ
//   CS  -> PA_0（例：混乱しないGPIOへ）
//   BL  -> 3.3V
// =====================
static constexpr uint8_t TFT_CS = PA_0;   // ←あなたの実配線に合わせて
static constexpr uint8_t TFT_DC = PA_1;   // ←あなたの実配線に合わせて

// ST7735 128x160
static constexpr int TFT_W = 128;
static constexpr int TFT_H = 160;

// オフセット（確定）
static constexpr uint8_t X_OFFSET = 2;
static constexpr uint8_t Y_OFFSET = 1;

// =====================
// KXTJ3 (I2C)
//   SDA -> PB_7
//   SCL -> PB_6
// =====================
static const uint8_t KXTJ3_ADDR     = 0x0E;
static const uint8_t REG_CTRL1      = 0x1B;
static const uint8_t REG_DATA_CTRL  = 0x21;
static const uint8_t REG_XOUT_L     = 0x06;

// LED（任意：動いてる時点灯）
static const uint32_t LED_PIN = PA_4;
static inline void ledWrite(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW); // 逆なら入れ替え
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

// CS LOWのまま cmd+data を送る（初期化の安定化）
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

static inline void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  x0 += X_OFFSET; x1 += X_OFFSET;
  y0 += Y_OFFSET; y1 += Y_OFFSET;

  tftWriteCommand(0x2A); // CASET
  uint8_t dataX[4] = { 0, (uint8_t)x0, 0, (uint8_t)x1 };
  tftWriteDataN(dataX, 4);

  tftWriteCommand(0x2B); // RASET
  uint8_t dataY[4] = { 0, (uint8_t)y0, 0, (uint8_t)y1 };
  tftWriteDataN(dataY, 4);

  tftWriteCommand(0x2C); // RAMWR
}

// 18bit(RGB888)で同色塗りつぶし
static void tftFillScreen888(uint8_t r, uint8_t g, uint8_t b) {
  tftSetAddrWindow(0, 0, TFT_W - 1, TFT_H - 1);

  dcData();
  digitalWrite(TFT_CS, LOW);
  for (uint32_t i = 0; i < (uint32_t)TFT_W * (uint32_t)TFT_H; i++) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }
  digitalWrite(TFT_CS, HIGH);
}

// 18bit水平線
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

// 18bit塗りつぶし円（sqrtf無し）
static void tftFillCircle888(int x0, int y0, int r, uint8_t r8, uint8_t g8, uint8_t b8) {
  if (r <= 0) return;

  int f = 1 - r;
  int ddF_x = 1;
  int ddF_y = -2 * r;
  int x = 0;
  int y = r;

  tftHLine888(x0 - r, y0, 2 * r + 1, r8, g8, b8);

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

static void tftInit_ST7735_18bit() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);   // 未接続でOK
  SPI.begin();

  tftWriteCommandData(0x01, nullptr, 0); delay(150); // SWRESET
  tftWriteCommandData(0x11, nullptr, 0); delay(150); // SLPOUT

  // COLMOD: 18-bit
  { const uint8_t d[] = { 0x06 }; tftWriteCommandData(0x3A, d, 1); }
  delay(10);

  // MADCTL: まずは0x00でOK（向きが合わなければ後で調整）
  { const uint8_t d[] = { 0x00 }; tftWriteCommandData(0x36, d, 1); }
  delay(10);

  tftWriteCommandData(0x20, nullptr, 0); delay(10); // INVOFF
  tftWriteCommandData(0x13, nullptr, 0); delay(10); // NORON
  tftWriteCommandData(0x29, nullptr, 0); delay(120); // DISPON
}

// ---------- KXTJ3 ----------
static inline void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(KXTJ3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}
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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  ledWrite(false);

  Wire.setSDA(PB_7);
  Wire.setSCL(PB_6);
  Wire.begin();
  Wire.setClock(100000);

  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  write8(REG_CTRL1, 0x80); delay(50);

  tftInit_ST7735_18bit();

  // 背景を黒に
  tftFillScreen888(0, 0, 0);

  pos_x = TFT_W * 0.5f;
  pos_y = TFT_H * 0.5f;

  calibrateKXTJ3(80);
}

void loop() {
  int16_t rx, ry, rz;
  if (!calibrated || !readXYZ_raw(rx, ry, rz)) return;

  int16_t dx = x_ref - rx;
  int16_t dy = ry - y_ref;

  const float alpha = 0.20f;
  fdx = (1.0f - alpha) * fdx + alpha * (float)dx;
  fdy = (1.0f - alpha) * fdy + alpha * (float)dy;

  const float dead = 300.0f;
  float ux = (fdx > -dead && fdx < dead) ? 0.0f : fdx;
  float uy = (fdy > -dead && fdy < dead) ? 0.0f : fdy;

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

  // 赤いボール
  tftFillCircle888(cx, cy, r, 255, 0, 0);
  // ハイライト
  int hr = (r/5) ? (r/5) : 1;
  tftFillCircle888(cx - r/3, cy - r/3, hr, 255, 255, 255);

  prev_cx = cx;
  prev_cy = cy;

  bool moving = (vel_x > 0.3f || vel_x < -0.3f || vel_y > 0.3f || vel_y < -0.3f);
  ledWrite(moving);

  delay(30);
}
