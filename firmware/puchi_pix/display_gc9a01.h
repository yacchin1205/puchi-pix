#pragma once
// =====================
// GC9A01 Round TFT 240x240 (RGB565)
// CAUTION: 未テスト。実機での動作確認が必要。
// =====================

static constexpr int DISPLAY_W = 240;
static constexpr int DISPLAY_H = 240;

// =====================
// 内部定数
// =====================
static constexpr uint8_t DISPLAY_CS  = PA_0;
static constexpr uint8_t DISPLAY_DC  = PA_1;
static constexpr uint8_t DISPLAY_RST = PA_2;
static constexpr uint8_t DISPLAY_X_OFFSET = 0;
static constexpr uint8_t DISPLAY_Y_OFFSET = 0;
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
  SPI.transfer(rgb565 >> 8);
  SPI.transfer(rgb565 & 0xFF);
}

static inline void displayBeginWrite(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint16_t x0 = x + DISPLAY_X_OFFSET;
  uint16_t x1 = x + w - 1 + DISPLAY_X_OFFSET;
  uint16_t y0 = y + DISPLAY_Y_OFFSET;
  uint16_t y1 = y + h - 1 + DISPLAY_Y_OFFSET;

  cmdByte(0x2A);
  { uint8_t d[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
    digitalWrite(DISPLAY_DC, HIGH);
    digitalWrite(DISPLAY_CS, LOW);
    for (uint8_t i = 0; i < 4; i++) SPI.transfer(d[i]);
    digitalWrite(DISPLAY_CS, HIGH); }

  cmdByte(0x2B);
  { uint8_t d[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
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

  pinMode(DISPLAY_RST, OUTPUT);
  digitalWrite(DISPLAY_RST, HIGH); delay(10);
  digitalWrite(DISPLAY_RST, LOW);  delay(10);
  digitalWrite(DISPLAY_RST, HIGH); delay(120);

  SPI.setSCLK(PA_5);
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.begin();

  // GC9A01 init table: {cmd, ndata|0x80 for delay after, data...}
  // ndata bit7 = post-delay flag, bit6..0 = data byte count
  static const uint8_t PROGMEM initTable[] = {
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
    0x11, 0x80, 120,     // SLPOUT + 120ms delay
    0x29, 0x80, 20,      // DISPON + 20ms delay
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

  pinMode(DISPLAY_BL, OUTPUT);
  displayBrightness(BRIGHT_FULL);
  displayFillScreen(0, 0, 0);
}
