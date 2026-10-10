// puchi_pix_card: 16x16 RGB ピクセルアート表示ボード (STM32G0B1CBT6)
// Build: GenG0, pnum=GENERIC_G0B1CBTX, usb=none (USB stack in src/usb: MSC UF2 drive + CDC log), xserial=none
//
// NOR Flash 上のアニメーションペイロード (USB の UF2 ドライブ経由で書き込む) を
// 表示 (MBI5124, BCM 8bit)、KXTJ3 による天地判定と motion wake、電池/VBUS 電圧のモニタ。
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "mbi5124.h"
#include "w25q.h"
#include "payload.h"
#include "src/usb/puchi_usb.h"

// =====================
// ピン (hardware/puchi_pix_card/mcu.kicad_sch)
// =====================
static constexpr uint32_t PIN_ACC_INT    = PA0;   // KXTJ3 INT (SYS_WKUP1)
static constexpr uint32_t PIN_VBUS_SENSE = PA1;   // VBUS 10k/10k (R6/R7)
static constexpr uint32_t PIN_VBAT_ADC   = PB1;   // 電池 1M/1M (R10/R11) + 100nF
static constexpr uint32_t PIN_BOOST_EN   = PD2;   // TPS61023 EN (1M プルダウン)
static constexpr uint32_t PIN_STATUS_LED = PD3;   // 1k → LED → GND

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

static constexpr uint32_t IDLE_TIMEOUT_MS = 30000;   // 無動作でここまで経つと消灯 (電池ならスリープ)
static constexpr uint32_t FRAME_MS = 33;

// USB 検出: VBUS 3.0V (ピン 1.5V) 以上で「あり」。D1 の逆リークでノードが
// 浮く誤検出対策として閾値を高めに置く (レビュー S6。分圧は 10k/10k)
static constexpr uint16_t VBUS_PRESENT_ADC = (uint16_t)(1.5f / 3.3f * 4095);
static bool vbusPresent = false;
static volatile uint32_t lastActivityMs = 0;
static uint32_t lastFrameMs = 0;
static uint8_t gamma8[256];

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
// NOR 上のペイロード: [0..3] 'PPNR', [4..7] ペイロード長 (LE), [16..] ペイロード本体。
// UF2 ドロップは USB 割り込みから 256B ページ単位で NOR に書かれ、最終ブロックで
// フラグが立つ。読み出しは RAM に丸ごと展開してから再生する
// =====================
static constexpr uint32_t NOR_MAGIC = 0x524E5050;   // 'PPNR'
static constexpr uint32_t NOR_HEADER_LEN = 16;
static bool payloadActive = false;
static volatile bool norDirty = false;
static uint32_t frameStartMs = 0;

// ホストはブロックを順不同で書くことがあるので、受信済みをビットマップで数えて
// 全ブロックが揃った時点で再読み込みする
static constexpr uint32_t UF2_MAX_BLOCKS = (NOR_HEADER_LEN + PAYLOAD_MAX) / NOR_PAGE_SIZE + 1;
static uint32_t uf2Seen[(UF2_MAX_BLOCKS + 31) / 32];
static uint32_t uf2SeenCount = 0;
static uint32_t uf2NumBlocks = 0;

void puchiUsbOnUf2Block(const Uf2Block &block) {
  if (block.payloadSize > NOR_PAGE_SIZE) return;
  if (block.targetAddr % NOR_PAGE_SIZE != 0) return;
  if (block.targetAddr + block.payloadSize > NOR_SIZE) return;
  if (block.numBlocks > UF2_MAX_BLOCKS || block.blockNo >= block.numBlocks) return;
  if (block.targetAddr % NOR_SECTOR_SIZE == 0) flashEraseSector(block.targetAddr);
  flashProgramPage(block.targetAddr, block.data, block.payloadSize);

  if (block.numBlocks != uf2NumBlocks) {
    memset(uf2Seen, 0, sizeof(uf2Seen));
    uf2SeenCount = 0;
    uf2NumBlocks = block.numBlocks;
  }
  uint32_t bit = 1u << (block.blockNo & 31);
  if (!(uf2Seen[block.blockNo >> 5] & bit)) {
    uf2Seen[block.blockNo >> 5] |= bit;
    uf2SeenCount++;
  }
  if (uf2SeenCount == uf2NumBlocks) {
    uf2NumBlocks = 0;
    norDirty = true;
  }
}

static void loadPayloadFromNor() {
  norDirty = false;
  HAL_NVIC_DisableIRQ(USB_UCPD1_2_IRQn);   // NOR は USB 割り込みからも書かれる
  uint8_t hdr[NOR_HEADER_LEN];
  flashRead(0, hdr, sizeof(hdr));
  uint32_t magic, len;
  memcpy(&magic, hdr, 4);
  memcpy(&len, hdr + 4, 4);
  payloadActive = false;
  if (magic == NOR_MAGIC && len <= PAYLOAD_MAX) {
    flashRead(NOR_HEADER_LEN, payloadBuf, len);
    payloadActive = payloadValidate(len);
  }
  HAL_NVIC_EnableIRQ(USB_UCPD1_2_IRQn);
  if (payloadActive) {
    payloadJumpTo(0);
    frameStartMs = millis();
    Serial.print("payload: v"); Serial.print(payVer); Serial.print(' ');
    Serial.print(payFrames); Serial.print(" frames, "); Serial.print(payloadLen); Serial.println(" bytes");
  } else {
    memset(mbiGray, 0, sizeof(mbiGray));
    mbiUploadFrame();
    Serial.println("payload: none");
  }
}

// =====================
// ROM の DFU ブートローダへ: 1200bps タッチでリセットし、起動直後に目印を見て
// システムメモリ (0x1FFF0000) へ飛ぶ。目印はリセットで消えない .noinit に置く
// =====================
static constexpr uint32_t DFU_MAGIC = 0x44465521;   // 'DFU!'
static uint32_t dfuMagic __attribute__((section(".noinit")));

static void jumpToBootloader() {
  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICPR[0] = 0xFFFFFFFF;
  const uint32_t* vec = (const uint32_t*)0x1FFF0000;
  __set_MSP(vec[0]);
  ((void (*)(void))vec[1])();
}

// =====================
// 電源
// =====================
static void powerRailsOn() {
  // SW1 は +3V3S 側だけを切るので昇圧の VIN は常に電池につながっている。
  // USB 給電中に昇圧を入れると電池から給電してしまうため、先に VBUS を見る
  vbusPresent = analogRead(PIN_VBUS_SENSE) > VBUS_PRESENT_ADC;
  if (vbusPresent) return;
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
// 表示: ペイロードのフレーム (RGB565) を重力の向きに合わせて回し、輝度とガンマをかける
// =====================
static constexpr uint8_t IMG_BRIGHTNESS = 128;      // 0..255
static uint8_t imgRot = 0;                          // 時計回り 90° 単位 (0..3)

// 重力の向きから「どの辺が上か」を決めて画像を回す。実測: 上辺が上で ay≈-1g、
// 左辺が上で ax≈-1g (センサ -Y=上辺, -X=左辺)。平置きや傾き不足は直前の向きを保持。
static constexpr int16_t ORIENT_THRESHOLD = 9000;   // ≈0.55g (16384/g)
static void updateOrientation() {
  int16_t ax, ay, az;
  if (!readXYZRaw(ax, ay, az)) return;
  uint8_t cand;
  if (abs(ay) >= abs(ax)) {
    if (abs(ay) < ORIENT_THRESHOLD) return;
    cand = ay < 0 ? 0 : 2;          // 上辺が上 / 下辺が上
  } else {
    if (abs(ax) < ORIENT_THRESHOLD) return;
    cand = ax < 0 ? 3 : 1;          // 左辺が上 / 右辺が上
  }
  static uint8_t pending = 0, count = 0;
  if (cand != pending) { pending = cand; count = 0; }
  if (count < 3) { count++; return; }
  imgRot = cand;
}

static void renderFrame(uint32_t now) {
  if (now - frameStartMs >= payloadDuration()) {
    frameStartMs = now;
    payloadAdvance();
  }
  for (uint8_t y = 0; y < MATRIX_H; y++) {
    for (uint8_t x = 0; x < MATRIX_W; x++) {
      uint8_t sx, sy;
      switch (imgRot) {
        case 1:  sx = y;                sy = MATRIX_W - 1 - x; break;   // 画像の上を右辺へ
        case 2:  sx = MATRIX_W - 1 - x; sy = MATRIX_H - 1 - y; break;
        case 3:  sx = MATRIX_H - 1 - y; sy = x;                break;   // 画像の上を左辺へ
        default: sx = x;                sy = y;                break;
      }
      uint16_t c = payFrame565[sy * PAYLOAD_SIZE + sx];
      uint8_t r = (c >> 8) & 0xF8, g = (c >> 3) & 0xFC, b = (c << 3) & 0xF8;
      mbiGray[0][y][x] = gamma8[(r * IMG_BRIGHTNESS) >> 8];
      mbiGray[1][y][x] = gamma8[(g * IMG_BRIGHTNESS) >> 8];
      mbiGray[2][y][x] = gamma8[(b * IMG_BRIGHTNESS) >> 8];
    }
  }
}

// =====================
// 消灯: 無動作が続いたらマトリクスの駆動を止める。USB 給電中はホストへの応答を
// 保つためここまでで、電池ならさらにスリープへ進む。動きを検出したら再点灯する。
// =====================
static bool displayOn = false;

static void setDisplay(bool on) {
  if (on == displayOn) return;
  displayOn = on;
  if (on) {
    mbiBegin();
    frameStartMs = millis();
    lastFrameMs = frameStartMs;
  } else {
    mbiEnd();
  }
  Serial.println(on ? "display on" : "display off");
}

// =====================
// スリープ: 昇圧を止めて +5V/+3V3P ドメインごと落とし、Stop1 で KXTJ3 の
// INT (PA0, EXTI) を待つ。復帰後は MBI の設定が消えているので再初期化する。
// =====================
extern "C" void SystemClock_Config(void);

static void enterSleep() {
  Serial.println("sleep");
  Serial.flush();
  setDisplay(false);
  flashPinsEnd();
  powerRailsOff();
  digitalWrite(PIN_STATUS_LED, LOW);
  // 8bit モード + ODR 0.781Hz へ (50Hz のままだと ~10µA、0.781Hz で ~1.5µA。
  // Wake-Up 機能は OWUF (CTRL2) 独立なので検出への影響なし。レビュー S8)
  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x08);
  write8(REG_CTRL1, 0x82); delay(50);
  clearLatchedInterrupt();

  // USB 挿入でも起きられるようにする (レビュー M5)。PA1 は普段 ADC 入力
  pinMode(PIN_VBUS_SENSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_VBUS_SENSE), wakeupCallback, RISING);

  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  SystemClock_Config();
  HAL_ResumeTick();

  detachInterrupt(digitalPinToInterrupt(PIN_VBUS_SENSE));
  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02);              // ODR 50Hz へ復帰
  write8(REG_CTRL1, 0xC2); delay(50);
  powerRailsOn();
  flashPinsBegin();
  lastActivityMs = millis();
  setDisplay(true);
}

void setup() {
  if (dfuMagic == DFU_MAGIC) {
    dfuMagic = 0;
    jumpToBootloader();
  }
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BOOST_EN, OUTPUT);
  digitalWrite(PIN_BOOST_EN, LOW);
  analogReadResolution(12);

  puchiUsbBegin();

  for (uint16_t i = 0; i < 256; i++) {
    gamma8[i] = (uint8_t)(powf(i / 255.0f, 2.2f) * 255.0f + 0.5f);
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
  Serial.println("puchi_pix_card build " __DATE__ " " __TIME__);
  Serial.print("flash jedec id: 0x");
  Serial.println(flashReadJedecId(), HEX);

  lastActivityMs = millis();
  setDisplay(true);
  loadPayloadFromNor();
  digitalWrite(PIN_STATUS_LED, HIGH);
}

void loop() {
  uint32_t now = millis();

  // KXTJ3 の INT はラッチ (INT_CTRL1 IEL=0)。解放しないと Low のまま次の FALLING が
  // 来ず、動かしていても lastActivityMs が更新されない。wakeupCallback は FALLING で更新済み
  if (digitalRead(PIN_ACC_INT) == LOW) {
    clearLatchedInterrupt();
  }

  // USB 給電中は昇圧を止めて VBUS 側から給電する (レビュー S3: 昇圧 5.0V >
  // VBUS-VF のため、昇圧を動かしたままだと USB 接続中も電池が減る)
  static uint32_t lastVbusCheckMs = 0;
  if (now - lastVbusCheckMs >= 100) {
    lastVbusCheckMs = now;
    bool present = analogRead(PIN_VBUS_SENSE) > VBUS_PRESENT_ADC;
    if (present != vbusPresent) {
      vbusPresent = present;
      digitalWrite(PIN_BOOST_EN, vbusPresent ? LOW : HIGH);
    }
  }

  if (puchiUsbDfuRequested()) {
    dfuMagic = DFU_MAGIC;
    NVIC_SystemReset();
  }
  if (norDirty) {
    loadPayloadFromNor();
    lastActivityMs = now;   // 書き込まれたものをすぐ見せる
  }

  // USB 接続中はスリープしない (レビュー M5: ホストから見て応答不能になる)
  bool idle = now - lastActivityMs >= IDLE_TIMEOUT_MS;
  if (idle && !vbusPresent) {
    enterSleep();
    return;
  }
  setDisplay(!idle);

  static uint32_t lastOrientMs = 0;
  if (now - lastOrientMs >= 100) {
    lastOrientMs = now;
    updateOrientation();
  }

  if (displayOn && payloadActive && now - lastFrameMs >= FRAME_MS) {
    lastFrameMs = now;
    renderFrame(now);
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
