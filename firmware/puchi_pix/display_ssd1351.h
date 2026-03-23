#pragma once
// =====================
// SSD1351 OLED 128x128 (RGB565)
// CAUTION: 未テスト。実機での動作確認が必要。
// =====================

static constexpr int DISPLAY_W = 128;
static constexpr int DISPLAY_H = 128;

// =====================
// 内部定数
// =====================
static constexpr uint8_t DISPLAY_CS  = PA_0;
static constexpr uint8_t DISPLAY_DC  = PA_1;
static constexpr uint8_t DISPLAY_RST = PA_2;
static constexpr uint8_t DISPLAY_X_OFFSET = 0;
static constexpr uint8_t DISPLAY_Y_OFFSET = 0;
static constexpr uint32_t STAT_LED_PIN = PA_8;

static constexpr uint8_t OLED_MASTER_FULL = 0x0A;
static constexpr uint8_t OLED_MASTER_DIM  = 0x02;
static constexpr uint8_t OLED_CONTRAST_A  = 0x91;
static constexpr uint8_t OLED_CONTRAST_B  = 0x50;
static constexpr uint8_t OLED_CONTRAST_C  = 0x7D;
static constexpr bool OLED_RESET_ON_FADE  = false;

// =====================
// 内部SPI低レベル
// =====================
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

static inline void cmdByte(uint8_t cmd) {
  cmdWithData(cmd, nullptr, 0);
}

static void oledHardReset() {
  digitalWrite(DISPLAY_RST, LOW);
  delay(10);
  digitalWrite(DISPLAY_RST, HIGH);
  delay(10);
}

static void oledInitSequence() {
  { const uint8_t d[] = { 0x12 }; cmdWithData(0xFD, d, 1); }
  { const uint8_t d[] = { 0xB1 }; cmdWithData(0xFD, d, 1); }
  cmdByte(0xAE); delay(10);
  { const uint8_t d[] = { 0xF1 }; cmdWithData(0xB3, d, 1); }
  { const uint8_t d[] = { 0x7F }; cmdWithData(0xCA, d, 1); }
  { const uint8_t d[] = { 0x74 }; cmdWithData(0xA0, d, 1); }
  { const uint8_t d[] = { 0x00 }; cmdWithData(0xA1, d, 1); }
  { const uint8_t d[] = { 0x00 }; cmdWithData(0xA2, d, 1); }
  { const uint8_t d[] = { 0x00 }; cmdWithData(0xB5, d, 1); }
  { const uint8_t d[] = { 0x01 }; cmdWithData(0xAB, d, 1); }
  { const uint8_t d[] = { 0x32 }; cmdWithData(0xB1, d, 1); }
  { const uint8_t d[] = { 0x17 }; cmdWithData(0xBB, d, 1); }
  { const uint8_t d[] = { 0x05 }; cmdWithData(0xBE, d, 1); }
  { const uint8_t d[] = { 0xC8, 0x80, 0xC8 }; cmdWithData(0xC1, d, 3); }
  { const uint8_t d[] = { 0x0F }; cmdWithData(0xC7, d, 1); }
  { const uint8_t d[] = { 0x01 }; cmdWithData(0xB6, d, 1); }
  cmdByte(0xA6);
  cmdByte(0xAF); delay(100);
}

static void oledSetAllContrast(uint8_t a, uint8_t b, uint8_t c, uint8_t master) {
  uint8_t rgb[3] = { a, b, c };
  cmdWithData(0xC1, rgb, 3);
  uint8_t m[1] = { (uint8_t)(master & 0x0F) };
  cmdWithData(0xC7, m, 1);
}

// =====================
// 共通インターフェース実装
// =====================

static inline void displayWritePixel(uint16_t rgb565) {
  SPI.transfer(rgb565 >> 8);
  SPI.transfer(rgb565 & 0xFF);
}

static inline void displayBeginWrite(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint8_t dataX[2] = { (uint8_t)(x + DISPLAY_X_OFFSET), (uint8_t)(x + w - 1 + DISPLAY_X_OFFSET) };
  cmdWithData(0x15, dataX, 2);
  uint8_t dataY[2] = { (uint8_t)(y + DISPLAY_Y_OFFSET), (uint8_t)(y + h - 1 + DISPLAY_Y_OFFSET) };
  cmdWithData(0x75, dataY, 2);

  // Write RAM command + ピクセルデータ受付開始
  digitalWrite(DISPLAY_CS, LOW);
  digitalWrite(DISPLAY_DC, LOW);
  SPI.transfer(0x5C);
  digitalWrite(DISPLAY_DC, HIGH);
}

static inline void displayEndWrite() {
  digitalWrite(DISPLAY_CS, HIGH);
}

static void displayFillScreen(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  displayBeginWrite(0, 0, DISPLAY_W, DISPLAY_H);
  for (uint32_t i = 0; i < (uint32_t)DISPLAY_W * DISPLAY_H; i++) {
    displayWritePixel(c);
  }
  displayEndWrite();
}

static void displayHLine(int x, int y, int w, uint8_t r, uint8_t g, uint8_t b) {
  if ((uint16_t)y >= DISPLAY_H || w <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > DISPLAY_W) w = DISPLAY_W - x;
  if (w <= 0) return;

  uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  displayBeginWrite((uint16_t)x, (uint16_t)y, (uint16_t)w, 1);
  while (w--) displayWritePixel(c);
  displayEndWrite();
}

static void displayBrightness(DisplayBright level) {
  switch (level) {
    case BRIGHT_FULL:
      cmdByte(0xAF);  // Display ON
      { uint8_t m[1] = { OLED_MASTER_FULL };
        cmdWithData(0xC7, m, 1); }
      digitalWrite(STAT_LED_PIN, LOW);
      break;
    case BRIGHT_DIM:
      { uint8_t m[1] = { OLED_MASTER_DIM };
        cmdWithData(0xC7, m, 1); }
      digitalWrite(STAT_LED_PIN, HIGH);
      break;
    case BRIGHT_OFF:
      cmdByte(0xAE);  // Display OFF
      digitalWrite(STAT_LED_PIN, HIGH);
      delay(10);
      break;
  }
}

static void displayFade(bool out) {
  const uint8_t steps = 8;
  if (out) {
    for (int i = steps; i >= 0; i--) {
      oledSetAllContrast(
        (OLED_CONTRAST_A * i) / steps,
        (OLED_CONTRAST_B * i) / steps,
        (OLED_CONTRAST_C * i) / steps,
        (OLED_MASTER_FULL * i) / steps);
      delay(25);
    }
  } else {
    for (int i = 0; i <= steps; i++) {
      oledSetAllContrast(
        (OLED_CONTRAST_A * i) / steps,
        (OLED_CONTRAST_B * i) / steps,
        (OLED_CONTRAST_C * i) / steps,
        (OLED_MASTER_FULL * i) / steps);
      delay(25);
    }
  }
}

static void displayReset() {
  if (OLED_RESET_ON_FADE) {
    oledHardReset();
    oledInitSequence();
  }
  oledSetAllContrast(0, 0, 0, 0);
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
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

  oledInitSequence();
  displayFillScreen(0, 0, 0);

  pinMode(STAT_LED_PIN, OUTPUT);
  digitalWrite(STAT_LED_PIN, LOW);
}
