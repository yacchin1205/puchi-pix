// puchi_pix_card: 16x16 RGB ピクセルアート表示ボード (STM32G0B1CBT6)
// Build: GenG0, pnum=GENERIC_G0B1CBTX, usb=CDCgen (Serial=USB CDC), xserial=none
//
// 実機立ち上げ用スケッチ: テストパターン表示、KXTJ3 による motion wake、
// NOR Flash の JEDEC ID 確認、電池/VBUS 電圧のモニタ。
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "mbi5153.h"

// =====================
// ピン (hardware/puchi_pix_card/mcu.kicad_sch)
// =====================
static constexpr uint32_t PIN_ACC_INT    = PA0;   // KXTJ3 INT (SYS_WKUP1)
static constexpr uint32_t PIN_VBUS_SENSE = PA1;   // VBUS 100k/100k
static constexpr uint32_t PIN_VBAT_ADC   = PB1;   // 電池 4.7M/4.7M
static constexpr uint32_t PIN_BOOST_EN   = PD2;   // TPS61023 EN (1M プルダウン)
static constexpr uint32_t PIN_STATUS_LED = PD3;   // 1k → LED → GND
static constexpr uint32_t PIN_FLASH_CS   = PA4;   // W25Q64 (SPI1: PA5/PA6/PA7)

// =====================
// KXTJ3 (I2C1: PB6/PB7)。レジスタ設定は puchi_pix (v1) と同じ
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

static constexpr uint32_t SLEEP_TIMEOUT_MS = 30000;
static constexpr uint32_t FRAME_MS = 33;
static volatile uint32_t lastActivityMs = 0;
static uint32_t lastFrameMs = 0;
static uint16_t animTick = 0;
static uint16_t gamma14[256];

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

static void setAccelHighRes(bool highRes) {
  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_CTRL1, highRes ? 0xC2 : 0x82); delay(50);
}

static void enableWakeupInterrupt() {
  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_CTRL2, 0x04);
  write8(REG_WU_COUNTER, 2);
  setWakeupThreshold(64);
  write8(REG_INT_CTRL2, 0x3F);
  write8(REG_INT_CTRL1, 0x20);
  write8(REG_CTRL1, 0xC2); delay(50);
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

static void wakeupCallback() {
  lastActivityMs = millis();
}

// =====================
// NOR Flash (W25Q64JV / GD25Q64 互換)
// =====================
static void flashPinsBegin() {
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.setSCLK(PA_5);
  SPI.begin();
  pinMode(PIN_FLASH_CS, OUTPUT);
  digitalWrite(PIN_FLASH_CS, HIGH);
}

// スリープ前に Hi-Z へ。NOR は +3V3P (昇圧断で消える) に載っているため、
// 出力を残すと保護ダイオード経由で未給電の NOR に逆給電する
static void flashPinsEnd() {
  SPI.end();
  pinMode(PIN_FLASH_CS, INPUT);
  pinMode(PA5, INPUT); pinMode(PA6, INPUT); pinMode(PA7, INPUT);
}

static uint32_t flashReadJedecId() {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x9F);
  uint32_t id = (uint32_t)SPI.transfer(0) << 16;
  id |= (uint32_t)SPI.transfer(0) << 8;
  id |= SPI.transfer(0);
  digitalWrite(PIN_FLASH_CS, HIGH);
  SPI.endTransaction();
  return id;   // W25Q64JV: 0xEF4017, GD25Q64: 0xC84017
}

// =====================
// 電源
// =====================
static void powerRailsOn() {
  digitalWrite(PIN_BOOST_EN, HIGH);   // +5V と +3V3P が立ち上がる
  delay(5);                            // TPS61023 ソフトスタート 700µs (p5)
}

static void powerRailsOff() {
  digitalWrite(PIN_BOOST_EN, LOW);
}

static float readVoltage(uint32_t pin, float dividerRatio) {
  return analogRead(pin) * 3.3f / 4095.0f * dividerRatio;   // 分圧はどちらも 1/2
}

// =====================
// テストパターン: 虹のグラデーションが流れ、白い点がラスタ順に走る
// (行/列/チャネル順の写像を目で確認する)
// =====================
static void hsvToRgb(uint8_t h, uint8_t &r, uint8_t &g, uint8_t &b) {
  uint8_t region = h / 43;
  uint8_t rem = (h - region * 43) * 6;
  uint8_t q = 255 - rem, t = rem;
  switch (region) {
    case 0: r = 255; g = t;   b = 0;   break;
    case 1: r = q;   g = 255; b = 0;   break;
    case 2: r = 0;   g = 255; b = t;   break;
    case 3: r = 0;   g = q;   b = 255; break;
    case 4: r = t;   g = 0;   b = 255; break;
    default: r = 255; g = 0;  b = q;   break;
  }
}

static void renderTestPattern(uint16_t tick) {
  uint16_t dot = (tick / 4) % (MATRIX_W * MATRIX_H);
  for (uint8_t y = 0; y < MATRIX_H; y++) {
    for (uint8_t x = 0; x < MATRIX_W; x++) {
      uint8_t r, g, b;
      if (y * MATRIX_W + x == dot) {
        r = g = b = 255;
      } else {
        hsvToRgb((uint8_t)(x * 8 + y * 8 + tick), r, g, b);
        r /= 8; g /= 8; b /= 8;   // 立ち上げ時は暗めに
      }
      mbiGray[0][y][x] = gamma14[r];
      mbiGray[1][y][x] = gamma14[g];
      mbiGray[2][y][x] = gamma14[b];
    }
  }
}

// =====================
// スリープ: 昇圧を止めて +5V/+3V3P ドメインごと落とし、Stop1 で KXTJ3 の
// INT (PA0, EXTI) を待つ。復帰後は MBI の設定が消えているので再初期化する。
// =====================
extern "C" void SystemClock_Config(void);

static void enterSleep() {
  Serial.println("sleep");
  Serial.flush();
  mbiEnd();
  flashPinsEnd();
  powerRailsOff();
  digitalWrite(PIN_STATUS_LED, LOW);
  setAccelHighRes(false);
  clearLatchedInterrupt();

  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  SystemClock_Config();
  HAL_ResumeTick();

  setAccelHighRes(true);
  powerRailsOn();
  flashPinsBegin();
  mbiBegin();
  lastActivityMs = millis();
  lastFrameMs = millis();
}

void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BOOST_EN, OUTPUT);
  digitalWrite(PIN_BOOST_EN, LOW);
  analogReadResolution(12);

  Serial.begin(115200);

  for (uint16_t i = 0; i < 256; i++) {
    gamma14[i] = (uint16_t)(powf(i / 255.0f, 2.2f) * 16383.0f + 0.5f);
  }

  Wire.setSDA(PB_7);
  Wire.setSCL(PB_6);
  Wire.begin();
  Wire.setClock(100000);
  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  enableWakeupInterrupt();
  pinMode(PIN_ACC_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ACC_INT), wakeupCallback, FALLING);

  powerRailsOn();
  flashPinsBegin();
  Serial.print("flash jedec id: 0x");
  Serial.println(flashReadJedecId(), HEX);

  mbiBegin();
  digitalWrite(PIN_STATUS_LED, HIGH);
  lastActivityMs = millis();
  lastFrameMs = millis();
}

void loop() {
  uint32_t now = millis();

  if (now - lastActivityMs >= SLEEP_TIMEOUT_MS) {
    enterSleep();
    return;
  }

  if (now - lastFrameMs >= FRAME_MS) {
    lastFrameMs = now;
    renderTestPattern(animTick++);
    mbiUploadFrame();
  }

  static uint32_t lastReportMs = 0;
  if (now - lastReportMs >= 1000) {
    lastReportMs = now;
    int16_t ax, ay, az;
    bool ok = readXYZRaw(ax, ay, az);
    Serial.print("vbat=");  Serial.print(readVoltage(PIN_VBAT_ADC, 2.0f), 2);
    Serial.print(" vbus="); Serial.print(readVoltage(PIN_VBUS_SENSE, 2.0f), 2);
    Serial.print(" acc=");
    if (ok) { Serial.print(ax); Serial.print(','); Serial.print(ay); Serial.print(','); Serial.print(az); }
    else Serial.print("err");
    Serial.println();
  }
}
