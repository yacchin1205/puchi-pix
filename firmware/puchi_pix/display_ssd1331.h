#pragma once
// =====================
// SSD1331 OLED 96x64 (RGB565)
// =====================

static constexpr int DISPLAY_W = 96;
static constexpr int DISPLAY_H = 64;

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
static inline void cmdByte(uint8_t b) {
  digitalWrite(DISPLAY_DC, LOW);
  digitalWrite(DISPLAY_CS, LOW);
  SPI.transfer(b);
  digitalWrite(DISPLAY_CS, HIGH);
}

static void oledHardReset() {
  digitalWrite(DISPLAY_RST, LOW);
  delay(10);
  digitalWrite(DISPLAY_RST, HIGH);
  delay(10);
}

static void oledInitRegisters() {
  cmdByte(0xAE);  delay(100);
  cmdByte(0xA0); cmdByte(0x72);
  cmdByte(0xA1); cmdByte(0x00);
  cmdByte(0xA2); cmdByte(0x00);
  cmdByte(0xA4);
  cmdByte(0xA8); cmdByte(0x3F);
  cmdByte(0xAD); cmdByte(0x8E);
  cmdByte(0xB0); cmdByte(0x0B);
  cmdByte(0xB1); cmdByte(0x31);
  cmdByte(0xB3); cmdByte(0xC0);
  cmdByte(0x8A); cmdByte(0x64);
  cmdByte(0x8B); cmdByte(0x78);
  cmdByte(0x8C); cmdByte(0x64);
  cmdByte(0xBB); cmdByte(0x3A);
  cmdByte(0xBE); cmdByte(0x3E);
  cmdByte(0x87); cmdByte(OLED_MASTER_FULL);
  cmdByte(0x81); cmdByte(OLED_CONTRAST_A);
  cmdByte(0x82); cmdByte(OLED_CONTRAST_B);
  cmdByte(0x83); cmdByte(OLED_CONTRAST_C);
  cmdByte(0xAF);  delay(100);
}

static void oledSetAllContrast(uint8_t a, uint8_t b, uint8_t c, uint8_t master) {
  cmdByte(0x81); cmdByte(a);
  cmdByte(0x82); cmdByte(b);
  cmdByte(0x83); cmdByte(c);
  cmdByte(0x87); cmdByte(master);
}

// =====================
// 共通インターフェース実装
// =====================

static inline void displayWritePixel(uint16_t rgb565) {
  SPI.transfer(rgb565 >> 8);
  SPI.transfer(rgb565 & 0xFF);
}

static inline void displayBeginWrite(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  // SSD1331: column/row address set (all bytes with D/C=LOW)
  digitalWrite(DISPLAY_CS, LOW);
  digitalWrite(DISPLAY_DC, LOW);
  SPI.transfer(0x15);
  SPI.transfer((uint8_t)(x + DISPLAY_X_OFFSET));
  SPI.transfer((uint8_t)(x + w - 1 + DISPLAY_X_OFFSET));
  digitalWrite(DISPLAY_CS, HIGH);

  digitalWrite(DISPLAY_CS, LOW);
  digitalWrite(DISPLAY_DC, LOW);
  SPI.transfer(0x75);
  SPI.transfer((uint8_t)(y + DISPLAY_Y_OFFSET));
  SPI.transfer((uint8_t)(y + h - 1 + DISPLAY_Y_OFFSET));
  digitalWrite(DISPLAY_CS, HIGH);

  // ピクセルデータ受付開始
  digitalWrite(DISPLAY_DC, HIGH);
  digitalWrite(DISPLAY_CS, LOW);
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
      cmdByte(0x87); cmdByte(OLED_MASTER_FULL);
      digitalWrite(STAT_LED_PIN, LOW);
      break;
    case BRIGHT_DIM:
      cmdByte(0x87); cmdByte(OLED_MASTER_DIM);
      digitalWrite(STAT_LED_PIN, HIGH);
      break;
    case BRIGHT_OFF:
      cmdByte(0x87); cmdByte(0x00);
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
    oledInitRegisters();
  }
  oledSetAllContrast(0, 0, 0, 0);
}

static void displayInit() {
  pinMode(DISPLAY_CS, OUTPUT);
  pinMode(DISPLAY_DC, OUTPUT);
  digitalWrite(DISPLAY_CS, HIGH);
  digitalWrite(DISPLAY_DC, HIGH);

  pinMode(DISPLAY_RST, OUTPUT);
  digitalWrite(DISPLAY_RST, HIGH);
  delay(10);
  oledHardReset();

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  oledInitRegisters();
  displayFillScreen(0, 0, 0);

  pinMode(STAT_LED_PIN, OUTPUT);
  digitalWrite(STAT_LED_PIN, LOW);
}
