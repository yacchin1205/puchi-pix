// NOTE: Build with "Tools > U(S)ART support > Disabled" to fit in 32KB flash
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// =====================
// ディスプレイ選択: コメントアウトでST7735 TFT
//  =====================
#define USE_SSD1351

// =====================
// 共通ピン定義
//   SCL -> PA_5 (SPI SCK)
//   SDA -> PA_7 (SPI MOSI)
//   DC  -> PA_1
//   CS  -> PA_0
// =====================
static constexpr uint8_t TFT_CS = PA_0;
static constexpr uint8_t TFT_DC = PA_1;

#ifdef USE_SSD1351
// SSD1351 OLED 128x128
static constexpr int TFT_W = 128;
static constexpr int TFT_H = 128;
static constexpr uint8_t X_OFFSET = 0;
static constexpr uint8_t Y_OFFSET = 0;
// 加速度センサー座標反転 (SSD1351用)
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
// 画像データ (32x53, 20 colors, 3x scale, 2 frames animation)
// =====================
static const uint8_t IMG_W = 32;
static const uint8_t IMG_H = 53;
static const uint8_t IMG_SCALE = 2;
static const uint8_t IMG_FRAMES = 2;

static const uint16_t imgPalette[20] PROGMEM = {
  0xDF9D,  // 0: (220, 241, 238)
  0xF71A,  // 1: (244, 225, 208)
  0x9EFB,  // 2: (155, 223, 223)
  0x8EFB,  // 3: (140, 220, 220) - background
  0x8EFA,  // 4: (140, 220, 212)
  0xDDF6,  // 5: (218, 191, 182)
  0x9E19,  // 6: (157, 194, 202)
  0x7658,  // 7: (119, 200, 197)
  0xED59,  // 8: (237, 170, 207)
  0xFCFE,  // 9: (252, 156, 246)
  0xF4FE,  // 10: (243, 156, 244)
  0x7516,  // 11: (118, 162, 176)
  0x4D34,  // 12: (76, 164, 162)
  0x2CE6,  // 13: (41, 157, 50)
  0x24E3,  // 14: (36, 156, 28)
  0xEC3B,  // 15: (233, 135, 221)
  0xA3B3,  // 16: (167, 119, 154)
  0xB2B1,  // 17: (176, 85, 142)
  0xC0ED,  // 18: (196, 29, 111)
  0x9049,  // 19: (151, 9, 76)
};

static const uint8_t imgFrame1[1696] PROGMEM = {
  16,19, 2,16,19, 2, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2,
  16,19, 6,16,19, 5, 0, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2,
   5, 5,19,19, 5, 5, 5, 0, 0, 0, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   5, 5,19,16, 5, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   5, 5,19,16, 5, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 3, 3,
   3, 3, 3,16,19, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 3, 3, 3, 3, 3,
   3, 3, 3,16,19, 4, 3, 3, 3, 3, 3, 3, 0, 0, 2, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 6,16, 6, 3, 3, 3, 3, 3, 3, 3, 6,18, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 6,18,18, 6, 3, 3, 3, 3, 3, 3,16,18, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 6,18,18,16, 3, 3, 3, 3, 3, 3,16,18,18, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 6,18,19,16, 5,10,10,10,10,10,15,18,19, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 6,19,18,10, 1, 9,10, 9, 9, 9,15,19,19, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 2,10,15,15,10,10, 5, 9,10, 9, 9,15,19, 6, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 6, 0, 0,10,15,15,10, 5,10,10, 5, 8, 9, 9,18,15, 9, 6, 3, 3, 3, 3, 2, 2, 3, 3, 3, 3, 3, 3,
   3, 3,10, 0, 0, 5,16, 9, 9, 9, 9, 5, 6, 5, 9, 9, 9, 9, 9,10, 5, 6, 3, 2,10,10, 3, 3, 3, 3, 3, 3,
   3, 3,10, 9, 9,15,18,18,18,18,18,18,18,15, 9, 9, 9, 9, 9, 9, 9,10, 5,10,15, 5, 3, 3, 3, 3, 3, 3,
   3, 3,10, 9,15,15,18,18,15,16,16,18,19,16, 9, 9, 9, 9, 9, 9, 9,10,15, 5, 5, 3, 3, 3, 3, 3, 3, 3,
   3, 3,10, 9,15,19,19, 5, 9, 8, 5, 5,19,19, 8, 9, 9,15, 9,15, 9, 9,10,10, 0, 0, 0, 0, 2, 3, 3, 3,
   3, 3,10,15,15,19, 5,19, 5, 5, 5,19, 5,16,16, 9, 9,15, 9,10,15, 9, 9, 9, 9, 9, 9, 9,10,10, 5, 3,
   3, 3, 3,15,15,19, 1,19, 5, 1, 5,19, 1, 5,16, 0, 9,15, 9, 9,10,15, 9, 9,15,15,15,15,15,15, 5, 3,
   3, 3, 3,15,15, 0,15, 1, 1, 1, 1, 0,15,15, 1, 9, 9, 0,15, 9, 9,15,15,15,15, 6, 3, 3, 3, 5,15, 5,
   3, 3, 6,15,15, 1,15, 1, 1, 1, 1, 0,15,15, 1, 9, 0, 0,15,10,15, 9, 9, 9, 9, 5, 6, 6, 3, 3, 3, 5,
   3, 3,15,15,15, 1, 8,16,16,16,16, 1, 1, 1, 1, 9, 9, 0,15,15,10,15,10, 9, 9, 9, 9,10, 6, 3, 3, 3,
   3, 3, 8,15,15, 1, 8,18,18,18,18, 1, 1, 1, 1, 9, 9, 9,15,15, 9,15,15,15,15,15, 9, 9,10, 6, 3, 3,
   3, 3,10,15,15, 1, 5,18,18,18,16, 1, 1, 1, 1, 9, 9, 9, 9,15,15,15, 5, 5,15,15,15,15,15, 5, 3, 3,
   3, 3, 6, 6,15, 5, 1, 5, 5, 5, 1, 1, 1, 1, 5, 5, 9,15,15,15,15,15,16,16, 5, 6, 6, 5,15,10, 6, 3,
  18,18,18, 8,18,18, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5,18,18,18, 5, 5, 5, 3,16,15,15,15, 3, 6,15, 5, 3,
  18,15,15,15,18,18, 4, 4,16,19, 8, 5, 5, 5, 5,16, 8, 1,18, 1, 1, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  18,18,15,18,18,16, 6, 5,18,18, 1, 1, 1, 1,15,18,16, 5,18, 5, 1, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  16,18,18,16,16, 1, 5, 5,18,18,16, 8, 5,16,18,18,18,18,16, 5, 1, 1, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 6,16, 0, 0, 1, 1, 5,16,18,19,18,19,19,18,18,19, 6, 3, 2, 0, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 5, 0, 0, 5, 5, 5, 5,18,16, 8, 8,16,18,18,19, 6, 3,16, 0, 0, 5, 6, 3, 3, 5, 5, 6, 3, 3, 3,
   3, 3, 3, 6, 6, 6, 6, 6, 6,18, 1, 0, 0, 8,18,18,18,18, 6,18, 0, 0,16,16, 3, 5,15,15,15, 6, 3, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 3,18, 1, 0, 0, 8,18,18,18,19, 4, 6,18,18,18,16, 3,10,15, 6, 5,15, 6, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 3,18, 1, 0, 0, 0,16,19,16,19,19,19,18, 6, 4, 4, 3,10,15, 3, 5,15, 5, 3,
   7,12, 4, 3, 3, 3, 4,12, 7, 6, 5, 5, 5, 5, 5, 0,18,18,19,19,18, 3, 3, 3, 3,15,15, 4, 3, 3, 3, 3,
   7, 7, 4, 4, 4, 4, 4, 7, 7,18,18,19,18,18,18,18,18,18,18,18,15, 6, 6, 6,10,15,15, 4, 3, 4, 4, 4,
   3, 3, 3, 4,12, 4, 3, 3,16,18,19,18,18,18,18,18,18,18,19, 6,15,15,15,16,15,15, 3, 3, 3,12,12,12,
   4, 3, 7, 7, 4, 7, 7, 3, 4,16,19,18,18, 5,18,19,18, 6, 3, 6, 4, 7, 4, 4, 7, 7, 3, 7,12, 4, 4, 4,
   7, 4, 7, 7, 4, 7,12, 4, 7,12, 4,18, 1, 1, 1, 5, 5, 6, 6, 6, 6,19,16, 4,12, 7, 4, 7,12, 4, 3, 4,
   7,12, 4, 4,12, 4, 4,12, 7, 3,12, 1, 1, 1, 1, 5, 5, 5, 5, 5,16,19,19,12,12,12,12, 4, 3,12,12,12,
   7, 7, 7, 7,12, 7, 7, 7, 7, 7, 6, 1, 1, 1, 5, 6, 5, 5, 5, 5,16,19,19,16,12,12,12, 7, 7,12,12,12,
   7, 3,12,12,12,12,12, 3, 7, 6, 0, 0, 1, 5, 7,12,16,16,16, 5,16,19,19,16,12,12,12,12,12,12,12,12,
  12, 7,12,12,12,12,16,18,18,18, 1, 0, 1, 5,12,12,12,12,12,12,12,16,16,12,12,12,12,12,12,12,12,12,
  12,12,12,12,12,12,19,18,18,18, 1, 0, 0,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
  12,14,12,12,12,12,12,19,19,18,15, 0, 6,12,12,14,12,12,12,12,12,14,12,12,12,12,14,12,12,12,12,12,
  13,14,12,12,12,12,12,19,19,19,18,16,12,12,12,14,12,12,12,12,13,14,12,12,12,13,14,12,12,12,12,12,
  14,14,12,12,12,14,14,14,19,19,19,19,14,12,12,14,14,12,12,14,14,14,12,12,14,14,14,14,14,12,12,12,
  14,14,13,13,13,14,14,14,14,14,14,14,14,13,13,14,14,13,13,14,14,14,13,13,14,14,14,14,14,13,13,13,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
};

static const uint8_t imgFrame2[1696] PROGMEM = {
   0, 0, 6,19,16, 6,19,16, 0, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   2, 0, 6,19,16, 6,19,16, 1, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 5, 5, 5,19,19, 5, 5, 5, 2, 0, 0, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 5, 5, 5,19,19, 5, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 5, 5, 5,19,19, 5, 5, 5, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3,
   3, 3, 3, 3, 3, 7,19,16, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 3,
   3, 3, 3, 3, 3, 7,19,16, 3, 3, 3, 3, 3, 3, 2, 0, 0, 2, 0, 0, 0, 0, 2, 2, 0, 0, 0, 2, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3,16,16, 4, 3, 3, 3, 3, 3, 3,16, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3,18,18, 4, 3, 3, 3, 3, 3, 3,18,16, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3,18,18,19,19, 3, 3, 3, 3, 3,18,18,16, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3,18,18,19,18, 3, 3, 3, 3, 3,18,19,16, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3,19,19, 0, 0, 9, 9, 9, 9, 9,19,19,18, 6, 3, 3, 3, 3, 2, 0,10, 5, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 2,16,18, 8, 9,10, 8, 9, 9, 9,18,19,18, 9, 8, 6, 6, 6,10,15,15, 5, 3, 6, 6, 3,
   3, 3, 3, 3, 3, 0, 0, 9, 9, 9,10, 8, 8, 9, 8,10, 9,18,15, 9, 9, 9,15,15,15, 5, 5, 3, 6, 5, 5, 3,
   3, 3, 3, 3, 8, 9, 9, 8, 5, 9,10, 8, 9, 8,16, 5, 9, 9, 9, 9, 9, 9,15, 5, 5, 3, 3, 6, 8, 8,10, 3,
   3, 3, 3, 3, 9, 9, 9, 5, 6,10, 9, 9, 9, 5, 6, 5, 9, 9, 9, 9, 9, 9, 9, 8, 8, 5, 8,10, 0, 9,15, 6,
   3, 3, 3, 3, 9, 9,15,18,18,18,18,18,18,18,18,15, 9, 9,10,15, 9, 9, 9, 9, 9, 9, 9, 9, 9,15,15, 6,
   3, 3, 3, 3, 9, 9,15,15,19,15,15,15, 5,19,19,15, 9, 9, 9,15,15, 9, 9, 9, 9, 9,15,15,15,15, 6, 3,
   3, 3, 3, 3,10, 9,16,19,19,15, 9, 8,19,19,19,19,10, 9, 9,15,15, 9, 9, 9, 9,10, 5, 3, 3, 3, 3, 3,
   3, 3, 3, 3, 6,10,15,19, 5,19, 9, 9,16,16, 0,19, 0, 9, 9,10, 9,15,10, 9, 9, 9, 9, 5, 3, 3, 3, 3,
   6, 3, 3, 3, 3, 5,15,16,16, 1, 1, 9,10, 8,15, 1, 0, 0, 0,10,15, 9,15,15,15,10,10, 9, 5, 6, 6, 6,
   3, 3, 3, 3, 3, 5,15, 0,15, 0, 1, 1, 1, 8,15, 1, 9, 0, 0,10,15,15, 9,15,15,15,15,15,15,10, 9, 9,
   3, 3, 3, 3, 6,15,15, 1, 8, 1, 1, 1, 1, 8, 8, 1, 9, 9, 9,10,15,15,15, 9,15, 9, 9,15,15,15,15,15,
   3, 3, 3, 3, 5,15,15, 8, 1,18,18,18,18, 8, 1, 1, 9, 9, 9,15,15,15,15,15, 9, 9,15,15, 9,10,15,15,
   3, 3, 3, 3, 5,15,15, 8, 1,18,18,18,18,16, 1, 1, 9, 9, 9,15,15,15,15,15,15, 9,15,15,15,15,15,15,
   3, 3, 6,18,18,16,18,16, 5, 1, 1, 1, 1, 1, 1, 1, 5, 5,15,15,15,15,15,15,15,15,15,15, 5, 3, 3,16,
   3, 4,18,15,18,15,15,18, 6, 5, 5, 5, 5, 5, 5, 5,16,16,19,19,16, 6, 6, 6, 6, 3, 3, 5,15,15, 6, 3,
   3, 4,18,18,15,15,18,18, 6, 3, 4,16,18, 5, 5, 5,18,18,18,18,18, 5, 5, 5, 5, 6, 3, 3, 6,16, 6, 3,
   3, 3, 6,18,18,18,18, 5, 6, 3, 5,19,18, 1, 5, 1,18,18, 1,16,18, 5, 5, 1, 1, 5, 3, 3, 3, 3, 3, 3,
   3, 3, 6,16,19,18, 8, 1, 1, 5,16,19,19,16,16,16,18,18,18,18,16, 5, 5, 1, 0, 0, 2, 3, 3, 3, 3, 3,
   3, 3, 3, 6,16, 8, 0, 0, 1, 5,16,19,19,19,18,19,18,18,19,18, 6, 3, 3, 5, 0, 0, 8, 6, 3, 3, 3, 3,
   3, 3, 3, 3, 6, 0, 0, 5, 5, 5, 5,18,18, 8, 8, 5,18,18,19,16, 3, 3, 3,18, 0, 0, 8,16, 3, 3, 3, 3,
   3, 3, 3, 3, 3, 3, 3, 3, 6, 6, 6, 6,16, 0, 0, 0,18,18,19,19, 6, 3, 3,19,18,18,18,16, 3, 5,10, 8,
   3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 6,16, 0, 0, 0,18,18,19,19,16, 6, 3, 6, 6, 6, 6, 4, 3, 8, 9,15,
   3, 3, 4,12, 7, 3, 3, 3, 3, 7,12, 6,16, 0, 0, 0,19,19,18,16,19,19,19, 6, 3, 3, 3, 4, 5, 9,15,15,
   4, 4, 4, 7, 4, 3, 4, 4, 3, 7, 7,16,18, 5, 5, 5, 5, 1, 8,18,19,19,19, 6, 4, 4, 3, 4, 5,15, 6, 4,
  12, 4, 3, 3, 3, 3,12, 7, 3, 3, 6,18,18,15, 1, 1,18,18,18,18,18,19, 6, 3, 4,12, 4, 3, 8,10, 6,12,
   4, 7, 7, 3, 7, 7, 4, 7, 7, 4,18,18,18,18,18,18,18,18,18,19,19,16,16,15, 6, 6, 7, 6,10,15, 6, 7,
   4, 7, 7, 4, 7,12, 4, 7,12, 7, 6,18,18,18,18,18,18,18,18,18, 6, 3, 6,16,16,16,16,15, 8, 7,12, 4,
  12, 4, 4,12, 7, 3,12, 7, 3,12, 6, 1,16,18,19,19,19,18,18, 5, 5, 6, 6,16,16,12,12,12,12, 7, 3,12,
  12, 7, 7, 7, 7, 7,12,12, 7, 6, 1, 1, 5, 5, 6,12,12, 5, 5, 5, 5, 1, 5,19,19,16,12,12,12, 7, 7,12,
  12,12, 7, 3, 7,12,12,12, 6, 1, 1, 1, 5, 5,12,12,12, 6, 5, 1, 1, 1, 5,19,19,19,12,12,12,12,12,12,
  12,12,12,12,12,16,18,18, 8, 0, 1, 5, 5,12,12,12,12,12, 6, 5, 1, 1, 5,19,19,19,12,12,12,12,12,12,
  12,12,12,12,16,18,18,18,18, 1, 5, 5,16,12,12,12,12,12,12,12,12,12,12,19,19,19,12,12,12,12,12,12,
  12,12,12,14,13,19,19,18, 8, 0, 5,12,14,12,12,12,14,13,12,12,12,12,13,14,16,12,12,12,14,12,12,12,
  12,12,12,14,12,16,19,19,18,16, 7,13,14,12,12,12,14,13,12,12,12,12,13,14,12,12,12,13,14,13,12,12,
  12,13,14,14,12,12,16,19,19,16,14,14,14,14,13,12,14,14,14,12,12,14,14,14,12,12,14,14,14,14,14,12,
  13,14,14,14,13,13,13,14,14,13,14,14,14,14,14,13,14,14,14,13,13,14,14,14,13,13,14,14,14,14,14,13,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
};

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

#ifdef USE_SSD1351
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
#ifdef USE_SSD1351
static inline void tftSetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t dataX[2] = { (uint8_t)(x0 + X_OFFSET), (uint8_t)(x1 + X_OFFSET) };
  tftWriteCommandData(0x15, dataX, 2);
  uint8_t dataY[2] = { (uint8_t)(y0 + Y_OFFSET), (uint8_t)(y1 + Y_OFFSET) };
  tftWriteCommandData(0x75, dataY, 2);
  tftWriteCommand(0x5C);
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
#ifdef USE_SSD1351
// RGB565 (16bit)
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void tftFillScreen888(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = rgb565(r, g, b);
  uint8_t hi = c >> 8, lo = c & 0xFF;

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
#else
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
#ifdef USE_SSD1351
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
#else
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

// ---------- 画像描画 (3倍拡大、中央配置、行単位バッチ描画) ----------
static const uint8_t* imgFrames[2] = { imgFrame1, imgFrame2 };
static int8_t lastDrawnFrame = -1;  // -1 = 未描画

// 行バッファを使って一括転送
static void drawFrameRow(uint16_t startX, uint16_t py, const uint8_t* rowData, const uint8_t* prevRowData) {
  // 行全体を1回のアドレスウィンドウ設定で転送
  tftSetAddrWindow(startX, py, startX + IMG_W * IMG_SCALE - 1, py + IMG_SCALE - 1);
  dcData();
  digitalWrite(TFT_CS, LOW);

  for (uint8_t sy = 0; sy < IMG_SCALE; sy++) {
    for (uint8_t ix = 0; ix < IMG_W; ix++) {
      uint8_t idx = pgm_read_byte(&rowData[ix]);
      uint16_t color = pgm_read_word(&imgPalette[idx]);
#ifdef USE_SSD1351
      uint8_t hi = color >> 8, lo = color & 0xFF;
      for (uint8_t sx = 0; sx < IMG_SCALE; sx++) {
        SPI.transfer(hi);
        SPI.transfer(lo);
      }
#else
      uint8_t r = (color >> 8) & 0xF8;
      uint8_t g = (color >> 3) & 0xFC;
      uint8_t b = (color << 3) & 0xF8;
      for (uint8_t sx = 0; sx < IMG_SCALE; sx++) {
        SPI.transfer(r);
        SPI.transfer(g);
        SPI.transfer(b);
      }
#endif
    }
  }
  digitalWrite(TFT_CS, HIGH);
}

static void drawFrame(uint8_t frameIdx) {
  uint16_t startX = (TFT_W - (uint16_t)IMG_W * IMG_SCALE) / 2;
  uint16_t startY = (TFT_H - (uint16_t)IMG_H * IMG_SCALE) / 2;

  const uint8_t* frameData = imgFrames[frameIdx];
  const uint8_t* prevData = (lastDrawnFrame >= 0) ? imgFrames[lastDrawnFrame] : nullptr;

  for (uint8_t iy = 0; iy < IMG_H; iy++) {
    const uint8_t* rowData = &frameData[(uint16_t)iy * IMG_W];
    const uint8_t* prevRowData = prevData ? &prevData[(uint16_t)iy * IMG_W] : nullptr;

    // デバッグ: 全行描画（差分チェック無効）
    uint16_t py = startY + (uint16_t)iy * IMG_SCALE;
    drawFrameRow(startX, py, rowData, prevRowData);
  }

  lastDrawnFrame = frameIdx;
}

// ---------- Display Init ----------
#ifdef USE_SSD1351
static void oledDisplayOn() {
  tftWriteCommandData(0xAF, nullptr, 0);
}

static void oledDisplayOff() {
  tftWriteCommandData(0xAE, nullptr, 0);
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
#else
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

static uint8_t currentFrame = 0;
static uint32_t lastFrameTime = 0;
static const uint32_t FRAME_INTERVAL_MS = 200;  // 200ms per frame

void setup() {
  tftInit();
  drawFrame(0);

  // 加速度センサー初期化 (スリープ用)
  Wire.setSDA(PB_7);
  Wire.setSCL(PB_6);
  Wire.begin();
  Wire.setClock(100000);

  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  enableWakeupInterrupt();

  attachInterrupt(digitalPinToInterrupt(KXTJ3_INT_PIN), wakeupCallback, FALLING);

  lastActivityMs = millis();
}

void loop() {
  // スリープ/ディム制御
  uint32_t now = millis();
  uint32_t elapsed = now - lastActivityMs;

  if (elapsed >= SLEEP_TIMEOUT_MS) {
    backlightOff();
    clearLatchedInterrupt();
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    backlightFull();
    lastActivityMs = millis();
    lastFrameTime = millis();  // アニメーション再開用
    return;
  }

  if (elapsed >= DIM_TIMEOUT_MS) {
    backlightDim();
  } else {
    backlightFull();
  }

  // アニメーション (2フレーム切り替え)
  if (now - lastFrameTime >= FRAME_INTERVAL_MS) {
    lastFrameTime = now;
    currentFrame = (currentFrame + 1) % IMG_FRAMES;
    drawFrame(currentFrame);
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
