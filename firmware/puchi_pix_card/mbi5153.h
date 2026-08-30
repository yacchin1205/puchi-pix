// 16x16 RGBマトリクス駆動: MBI5153 x3 (列, R/G/B各1) + 74HCT595 x2 → PFET x16 (行スキャン)
//
// 配線 (hardware/puchi_pix_card/display.kicad_sch):
//   MBIチェーン MCU SDI → U101(R) → U102(G) → U103(B) → MCU SDO
//   COL_x c = MBI OUT c (c=0 が左端), ROW r = 595 SR_Q r (r=0 が最上段)
//   595 QA (SR_Q0) は最後にシフトしたビット。'0' を1個入れて1セクションごとに
//   1ビットずつ送ると、アクティブLow の行選択が上から下へ移動する。
//
// 動作 (MBI5153 datasheet V1.00):
//   - コマンドは LE=High の間の DCLK 立ち上がり数で識別 (p13)
//     1=Data Latch, 2=VSYNC, 4=Write Config1 (要 Pre-Active=14 先行)
//   - 14bit階調モードでは 16bit データの下位2bitを0で送る (p18)
//   - スキャンは 1セクション = 513 GCLK で1行進む (p18)。TIM1 のリピート
//     カウンタ(RCR=512)+ワンパルスで 513 パルスのバーストを出し、バースト終了
//     割込で 595 を1ビット進めて次のバーストを開始する。行切替と MBI の内部
//     ラインカウンタはこれで同期する。
//   - VSYNC は GCLK 停止中に打つ (GCLK→LE↑ 300ns, LE↓→GCLK 1200ns, p9)。
//     バースト間の停止区間で ISR が発行する。
#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>

static constexpr uint8_t MATRIX_W = 16;
static constexpr uint8_t MATRIX_H = 16;

// ---- ピン (回路図ネット名) ----
// GCLK=PA8 (TIM1_CH1), MBI_LE=PB12, MBI_DCLK=PB13, MBI_SDO=PB14, MBI_SDI=PB15
// SR_SER=PC6, SR_SRCLK=PC7, SR_RCLK=PD0, SR_NOE=PD1
#define MBI_LE_BIT   12
#define MBI_DCLK_BIT 13
#define MBI_SDI_BIT  15
#define SR_SER_BIT   6
#define SR_SRCLK_BIT 7
#define SR_RCLK_BIT  0
#define SR_NOE_BIT   1

// 1スキャンライン内の16回の Data Latch が OUT15→OUT0 の順か OUT0→OUT15 の順か。
// データシートに明記がないため実機で確認する (逆なら列が左右反転して見える)。
static constexpr bool MBI_FIRST_LATCH_IS_OUT15 = true;

// Config Register 1 (p16): bitF=下ゴースト除去, bitB-8=スキャンライン数-1,
// bit7=0:14bit階調, bit6=0:GCLK倍増なし, bit5-0=電流ゲイン (既定 0b101011)
static constexpr uint16_t MBI_CONFIG1 = (1u << 15) | ((MATRIX_H - 1) << 8) | 0x2B;

// GCLK = 64MHz / MBI_GCLK_DIV。3.3V動作時の上限 20MHz (p9)。
static constexpr uint32_t MBI_GCLK_DIV = 8;   // 8MHz → 1セクション 64µs, 16行 1.03ms
static constexpr uint32_t MBI_GCLKS_PER_SECTION = 513;

// 階調バッファ: [色][y][x], 14bit (0..16383)。上位側が書き換え、mbiUploadFrame で転送
static uint16_t mbiGray[3][MATRIX_H][MATRIX_W];

static HardwareTimer mbiGclkTimer(TIM1);
static volatile bool    mbiVsyncPending = false;
static volatile uint8_t mbiRow = 0;

// ---------- ビットバンギング (BSRR 直書き) ----------
static inline void mbiPin(GPIO_TypeDef* port, uint8_t bit, bool high) {
  port->BSRR = high ? (1u << bit) : (1u << (bit + 16));
}

// 1ビットを DCLK に乗せる。le は当該クロックでの LE レベル
static inline void mbiClockBit(bool bit, bool le) {
  mbiPin(GPIOB, MBI_SDI_BIT, bit);
  mbiPin(GPIOB, MBI_LE_BIT, le);
  mbiPin(GPIOB, MBI_DCLK_BIT, true);
  mbiPin(GPIOB, MBI_DCLK_BIT, false);
}

// 16bit を MSB first で送る。末尾 leClocks 個のクロックで LE=High (コマンド符号化)
static void mbiShiftWord(uint16_t w, uint8_t leClocks) {
  for (int8_t i = 15; i >= 0; i--) {
    mbiClockBit((w >> i) & 1, i < leClocks);
  }
  mbiPin(GPIOB, MBI_LE_BIT, false);
}

// データを伴わないコマンド: LE=High のまま n クロック
static void mbiCommand(uint8_t nClocks) {
  mbiPin(GPIOB, MBI_SDI_BIT, false);
  mbiPin(GPIOB, MBI_LE_BIT, true);
  for (uint8_t i = 0; i < nClocks; i++) {
    mbiPin(GPIOB, MBI_DCLK_BIT, true);
    mbiPin(GPIOB, MBI_DCLK_BIT, false);
  }
  mbiPin(GPIOB, MBI_LE_BIT, false);
}

// チェーン3個へ同じ語を書き、末尾 leClocks クロックでラッチ
static void mbiChainWrite(uint16_t wR, uint16_t wG, uint16_t wB, uint8_t leClocks) {
  mbiShiftWord(wB, 0);   // 最初に送ったビットが最遠端 (U103=B) に届く
  mbiShiftWord(wG, 0);
  mbiShiftWord(wR, leClocks);
}

// ---------- 74HCT595 ----------
static inline void srShiftBit(bool bit) {
  mbiPin(GPIOC, SR_SER_BIT, bit);
  mbiPin(GPIOC, SR_SRCLK_BIT, true);
  mbiPin(GPIOC, SR_SRCLK_BIT, false);
}

static inline void srLatch() {
  mbiPin(GPIOD, SR_RCLK_BIT, true);
  mbiPin(GPIOD, SR_RCLK_BIT, false);
}

static inline void srRowsEnable(bool enable) {
  mbiPin(GPIOD, SR_NOE_BIT, !enable);   // /OE
}

static void srAllOff() {
  for (uint8_t i = 0; i < MATRIX_H; i++) srShiftBit(true);
  srLatch();
}

// ---------- スキャン ----------
// バースト終了ごとに呼ばれる。GCLK 停止中なので VSYNC はここで安全に打てる
static void mbiSectionEnd() {
  if (mbiRow == 0 && mbiVsyncPending) {
    mbiCommand(2);              // VSYNC: 直前の GCLK からは ISR 遅延で 300ns 以上経過
    mbiVsyncPending = false;
    delayMicroseconds(2);       // LE↓ → 次の GCLK まで 1200ns
  }
  srShiftBit(mbiRow != 0);      // 行0 のときだけ '0' を投入
  srLatch();
  mbiRow = (mbiRow + 1) & (MATRIX_H - 1);
  TIM1->CR1 |= TIM_CR1_CEN;     // 次の 513 パルス
}

// ---------- 公開 API ----------
// フレームバッファを MBI の SRAM へ転送する。表示は次の VSYNC で切り替わる。
// 前回の VSYNC が未発行なら待つ (ISR と DCLK/LE を取り合わないため)。
static void mbiUploadFrame() {
  while (mbiVsyncPending) {}
  for (uint8_t y = 0; y < MATRIX_H; y++) {
    for (uint8_t k = 0; k < MATRIX_W; k++) {
      uint8_t x = MBI_FIRST_LATCH_IS_OUT15 ? (MATRIX_W - 1 - k) : k;
      mbiChainWrite((uint16_t)(mbiGray[0][y][x] << 2),
                    (uint16_t)(mbiGray[1][y][x] << 2),
                    (uint16_t)(mbiGray[2][y][x] << 2), 1);
    }
  }
  mbiVsyncPending = true;
}

// 電源投入後 (+3V3P ドメイン復帰後) 毎回呼ぶ。設定レジスタは電源断で消える。
static void mbiBegin() {
  pinMode(PB12, OUTPUT); pinMode(PB13, OUTPUT); pinMode(PB15, OUTPUT);
  pinMode(PB14, INPUT);
  pinMode(PC6, OUTPUT);  pinMode(PC7, OUTPUT);  pinMode(PD0, OUTPUT);  pinMode(PD1, OUTPUT);
  srRowsEnable(false);
  mbiPin(GPIOB, MBI_LE_BIT, false);
  mbiPin(GPIOB, MBI_DCLK_BIT, false);
  srAllOff();

  mbiCommand(14);                                   // Pre-Active
  mbiChainWrite(MBI_CONFIG1, MBI_CONFIG1, MBI_CONFIG1, 4);  // Write Config1

  memset(mbiGray, 0, sizeof(mbiGray));
  mbiVsyncPending = false;
  mbiUploadFrame();                                 // VSYNC 有効化には 16 回以上の Data Latch が必要

  // TIM1: PA8 に 50% PWM、RCR=512 + OPM で 513 パルスのバースト
  mbiGclkTimer.pause();
  mbiGclkTimer.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA_8);
  mbiGclkTimer.setOverflow(MBI_GCLK_DIV, TICK_FORMAT);
  mbiGclkTimer.setCaptureCompare(1, MBI_GCLK_DIV / 2, TICK_COMPARE_FORMAT);
  TIM1->CR1 |= TIM_CR1_URS;                         // UG で割込を起こさない
  TIM1->RCR = MBI_GCLKS_PER_SECTION - 1;
  mbiGclkTimer.refresh();                           // RCR を取り込む
  TIM1->CR1 |= TIM_CR1_OPM;
  mbiGclkTimer.attachInterrupt(mbiSectionEnd);

  // 最初のバースト (MBI にとっては VSYNC 前の空クロック) が終わった ISR で
  // VSYNC → 行0 → バースト、と揃って始まる
  mbiRow = 0;
  mbiVsyncPending = true;
  mbiGclkTimer.resume();
  srRowsEnable(true);
}

// スリープ前: 行スイッチを全OFFにして GCLK を止める (+3V3P/+5V 断の前に)
static void mbiEnd() {
  srRowsEnable(false);
  mbiGclkTimer.pause();
  mbiGclkTimer.detachInterrupt();
  mbiVsyncPending = false;
  srAllOff();
  pinMode(PA8, INPUT);
  pinMode(PB12, INPUT); pinMode(PB13, INPUT); pinMode(PB15, INPUT);   // 未給電 MBI への逆給電防止
  pinMode(PC6, INPUT);  pinMode(PC7, INPUT);  pinMode(PD0, INPUT);
}
