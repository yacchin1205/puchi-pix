#pragma once
// =====================
// ST7735 TFT 128x160 (RGB888/18bit)
// CAUTION: 未テスト。実機での動作確認が必要。
// =====================

static constexpr int DISPLAY_W = 128;
static constexpr int DISPLAY_H = 160;

// =====================
// 内部定数
// =====================
static constexpr uint8_t DISPLAY_CS  = PA_0;
static constexpr uint8_t DISPLAY_DC  = PA_1;
static constexpr uint8_t DISPLAY_X_OFFSET = 2;
static constexpr uint8_t DISPLAY_Y_OFFSET = 1;
static constexpr uint8_t DISPLAY_BL = PA_8;

// =====================
// 内部SPI低レベル
// =====================
static inline void cmdByte(uint8_t cmd) {
  digitalWrite(DISPLAY_DC, LOW);
  digitalWrite(DISPLAY_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(DISPLAY_CS, HIGH);
}

static inline void cmdWithData(uint8_t cmd, const uint8_t* data, uint8_t n) {
  digitalWrite(DISPLAY_CS, LOW);
  digitalWrite(DISPLAY_DC, LOW);
  SPI.transfer(cmd);
  if (n) {
    digitalWrite(DISPLAY_DC, HIGH);
    while (n--) SPI.transfer(*data++);
  }
  digitalWrite(DISPLAY_CS, HIGH);
}

// =====================
// 共通インターフェース実装
// =====================

static inline void displayWritePixel(uint16_t rgb565) {
  SPI.transfer((rgb565 >> 8) & 0xF8);
  SPI.transfer((rgb565 >> 3) & 0xFC);
  SPI.transfer((rgb565 << 3) & 0xF8);
}

static inline void displayBeginWrite(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint16_t x0 = x + DISPLAY_X_OFFSET;
  uint16_t x1 = x + w - 1 + DISPLAY_X_OFFSET;
  uint16_t y0 = y + DISPLAY_Y_OFFSET;
  uint16_t y1 = y + h - 1 + DISPLAY_Y_OFFSET;

  cmdByte(0x2A);
  { uint8_t d[4] = { 0, (uint8_t)x0, 0, (uint8_t)x1 };
    digitalWrite(DISPLAY_DC, HIGH);
    digitalWrite(DISPLAY_CS, LOW);
    for (uint8_t i = 0; i < 4; i++) SPI.transfer(d[i]);
    digitalWrite(DISPLAY_CS, HIGH); }

  cmdByte(0x2B);
  { uint8_t d[4] = { 0, (uint8_t)y0, 0, (uint8_t)y1 };
    digitalWrite(DISPLAY_DC, HIGH);
    digitalWrite(DISPLAY_CS, LOW);
    for (uint8_t i = 0; i < 4; i++) SPI.transfer(d[i]);
    digitalWrite(DISPLAY_CS, HIGH); }

  cmdByte(0x2C);

  // ピクセルデータ受付開始
  digitalWrite(DISPLAY_DC, HIGH);
  digitalWrite(DISPLAY_CS, LOW);
}

static inline void displayEndWrite() {
  digitalWrite(DISPLAY_CS, HIGH);
}

static void displayFillScreen(uint8_t r, uint8_t g, uint8_t b) {
  displayBeginWrite(0, 0, DISPLAY_W, DISPLAY_H);
  for (uint32_t i = 0; i < (uint32_t)DISPLAY_W * DISPLAY_H; i++) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }
  displayEndWrite();
}

static void displayHLine(int x, int y, int w, uint8_t r, uint8_t g, uint8_t b) {
  if ((uint16_t)y >= DISPLAY_H || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > DISPLAY_W) w = DISPLAY_W - x;
  if (w <= 0) return;

  displayBeginWrite((uint16_t)x, (uint16_t)y, (uint16_t)w, 1);
  while (w--) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }
  displayEndWrite();
}

static void displayBrightness(DisplayBright level) {
  switch (level) {
    case BRIGHT_FULL: analogWrite(DISPLAY_BL, 0);   break;
    case BRIGHT_DIM:  analogWrite(DISPLAY_BL, 210); break;
    case BRIGHT_OFF:  analogWrite(DISPLAY_BL, 255); break;
  }
}

static void displayFade(bool out) {
  (void)out;  // TFT: no-op
}

static void displayReset() {
  // TFT: no-op
}

static void displayInit() {
  pinMode(DISPLAY_CS, OUTPUT);
  pinMode(DISPLAY_DC, OUTPUT);
  digitalWrite(DISPLAY_CS, HIGH);
  digitalWrite(DISPLAY_DC, HIGH);

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();

  cmdWithData(0x01, nullptr, 0); delay(150);  // SWRESET
  cmdWithData(0x11, nullptr, 0); delay(150);  // SLPOUT
  { const uint8_t d[] = { 0x06 }; cmdWithData(0x3A, d, 1); } delay(10);
  { const uint8_t d[] = { 0x00 }; cmdWithData(0x36, d, 1); } delay(10);
  cmdWithData(0x20, nullptr, 0); delay(10);   // INVOFF
  cmdWithData(0x13, nullptr, 0); delay(10);   // NORON
  cmdWithData(0x29, nullptr, 0); delay(120);  // DISPON

  pinMode(DISPLAY_BL, OUTPUT);
  displayBrightness(BRIGHT_FULL);
  displayFillScreen(0, 0, 0);
}
