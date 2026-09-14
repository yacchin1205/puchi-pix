// 16x16 RGBマトリクス駆動: MBI5124 x3 (列, R/G/B各1) + 74HCT595 x2 → PFET x16 (行スキャン)
//
// 配線 (hardware/puchi_pix_card/display.kicad_sch):
//   MBIチェーン MCU SDI → U101(R) → U102(G) → U103(B) → MCU SDO
//   COL_x c = MBI OUT c (c=0 が左端、赤のみ直列180Ω)、ROW r = 595 SR_Q r (r=0 が最上段)
//   595 QA (SR_Q0) は最後にシフトしたビット。'0' を1個入れて1行ごとに
//   1ビットずつ送ると、アクティブLow の行選択が上から下へ移動する。
//   PFET ゲートは 100k で +5V に吊られており、595 が Hi-Z/未給電でも行OFF。
//   /OE (PD1) はオープンドレイン駆動: R104 10k → +5V がプルアップ。リリースで
//   行無効かつ未給電の +5V ドメインへ流れ込まない (レビュー C2)。
//
// 動作 (MBI5124 datasheet V0.01):
//   - MBI5124 は 16bit シフトレジスタ+出力ラッチ+/OE のみで、階調は MCU が作る。
//     方式は BCM: 各行を 8 枚のビットプレーンに分け、プレーン b を 2^b × T0 の
//     時間だけ /OE=Low にして点灯する。プレーン切替 (/OE=High の間) に
//     次プレーンのデータを LE でラッチし、その点灯中に次々プレーンを SPI2 で送る。
//   - /OE は TIM1_CH1 (PA8) の PWM2 + ワンパルスで生成する。CCR1=幅、ARR=幅+1 で
//     開始し、プリロードした CCR1=0 がオーバーフローで有効になるので停止後は
//     /OE=High に留まる。オーバーフロー割込で次プレーンへ進む。
//   - データラッチは LE 単独パルス (CLK なし, p12 "Latch data")。設定レジスタは
//     LE=High の間に CLK 4 発 (p12)。色ごとの推奨値 (p12) を電源投入時に書く。
//   - シフト順: 先に送ったビットが最遠端 (U103=B) の OUT15 に届く (p5 D15→OUT15)。
//   - タイミング: T0=3µs (SPI 16MHz の 48bit=3µs と同じ) → 1行 ≈ 0.8ms、16行 ≈ 12.7ms (≈79Hz)
#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>
#include <SPI.h>

static constexpr uint8_t MATRIX_W = 16;
static constexpr uint8_t MATRIX_H = 16;

// ---- ピン (回路図ネット名) ----
// MBI_OE=PA8 (TIM1_CH1), MBI_LE=PB12, MBI_DCLK=PB13 (SPI2_SCK), MBI_SDO=PB14 (MISO),
// MBI_SDI=PB15 (MOSI), SR_SER=PC6, SR_SRCLK=PC7, SR_RCLK=PD0, SR_NOE=PD1
#define MBI_LE_BIT   12
#define MBI_DCLK_BIT 13
#define MBI_SDI_BIT  15
#define SR_SER_BIT   6
#define SR_SRCLK_BIT 7
#define SR_RCLK_BIT  0
#define SR_NOE_BIT   1

static constexpr uint8_t  MBI_PLANES   = 8;
static constexpr uint32_t MBI_T0_TICKS = 3 * 64;      // TIM1 64MHz: LSB プレーン 3µs
static constexpr uint32_t MBI_SPI_HZ   = 16000000;    // MBI5124 CLK 上限 25MHz (p1)
static constexpr uint8_t  MBI_STREAM_BYTES = 6;       // 3チップ × 16bit

// 設定レジスタ (p12): LED の色ごとのプリチャージ/ゴースト除去設定
static constexpr uint16_t MBI_CONFIG_R = 0x7D6B;
static constexpr uint16_t MBI_CONFIG_G = 0xF16B;
static constexpr uint16_t MBI_CONFIG_B = 0xED6B;

// 階調バッファ: [色][y][x], 8bit。上位側が書き換え、mbiUploadFrame で反映
static uint8_t mbiGray[3][MATRIX_H][MATRIX_W];

// ビットプレーン列: [面][y][プレーン][バイト] (B OUT15..0, G OUT15..0, R OUT15..0)
static uint8_t mbiPlanes[2][MATRIX_H][MBI_PLANES][MBI_STREAM_BYTES];
static volatile uint8_t mbiFront = 0;          // ISR が表示中の面
static volatile bool    mbiSwapPending = false;
static volatile uint8_t mbiRow = 0;            // シフトレジスタに入っているプレーンの行
static volatile uint8_t mbiPlane = 0;          //   同プレーン番号

static HardwareTimer mbiOeTimer(TIM1);
static SPIClass mbiSpi(PB_15, PB_14, PB_13);   // SPI2

// ---------- ビットバンギング (BSRR 直書き) ----------
static inline void mbiPin(GPIO_TypeDef* port, uint8_t bit, bool high) {
  port->BSRR = high ? (1u << bit) : (1u << (bit + 16));
}

// 最適化レベルに依存せずセットアップ/ホールドを確保する短い待ち (M0+ 64MHz: 1 NOP ≈ 16ns)
static inline void mbiNops() {
  __NOP(); __NOP(); __NOP(); __NOP();
}

// 1ビットを DCLK に乗せる。le は当該クロックでの LE レベル (設定書込専用)
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
  mbiNops();
}

// 設定レジスタ書込: チェーン3個へ色別の語を送り、末尾 4 クロックで LE=High (p12)
static void mbiWriteConfig() {
  mbiShiftWord(MBI_CONFIG_B, 0);   // 最初に送った語が最遠端 (U103=B) に届く
  mbiShiftWord(MBI_CONFIG_G, 0);
  mbiShiftWord(MBI_CONFIG_R, 4);
}

// LE 単独パルス: シフトレジスタ → 出力ラッチ (tw(L) ≥20ns, p9)
static inline void mbiLatch() {
  mbiPin(GPIOB, MBI_LE_BIT, true);
  mbiNops();
  mbiPin(GPIOB, MBI_LE_BIT, false);
}

// 48bit を SPI2 で送る (ISR 内から呼ぶ)。転送完了まで戻らない (16MHz で 3µs)。
// 受信側 (MBI_SDO) は使わないので FIFO を捨て、OVR を消す。
static void mbiShift48(const uint8_t* stream) {
  for (uint8_t i = 0; i < MBI_STREAM_BYTES; i++) {
    while (!(SPI2->SR & SPI_SR_TXE)) {}
    *(volatile uint8_t*)&SPI2->DR = stream[i];
  }
  while (SPI2->SR & SPI_SR_BSY) {}
  while (SPI2->SR & SPI_SR_FRLVL) { (void)*(volatile uint8_t*)&SPI2->DR; }
  (void)SPI2->SR;   // OVR クリア (DR 読み → SR 読み)
}

// ---------- 74HCT595 ----------
static inline void srShiftBit(bool bit) {
  mbiPin(GPIOC, SR_SER_BIT, bit);
  mbiNops();   // tsu(DS→SHCP) 20ns
  mbiPin(GPIOC, SR_SRCLK_BIT, true);
  mbiPin(GPIOC, SR_SRCLK_BIT, false);
}

static inline void srLatch() {
  mbiPin(GPIOD, SR_RCLK_BIT, true);
  mbiPin(GPIOD, SR_RCLK_BIT, false);
}

// /OE はオープンドレイン: false = リリース (R104 が +5V に引き上げ → 行OFF)
static inline void srRowsEnable(bool enable) {
  mbiPin(GPIOD, SR_NOE_BIT, !enable);
}

static void srAllOff() {
  for (uint8_t i = 0; i < MATRIX_H; i++) srShiftBit(true);
  srLatch();
}

// ---------- /OE パルス ----------
// width ティックだけ /OE=Low。ARR=width+1 のオーバーフローで割込、同時にプリロード済みの
// CCR1=0 が有効になり出力は High に戻る (PWM2: CNT ≥ CCR1 で active=High)
static inline void mbiOePulse(uint32_t width) {
  TIM1->ARR  = width + 1;
  TIM1->CCR1 = width;
  TIM1->EGR  = TIM_EGR_UG;      // シャドウ更新 + CNT=0 (URS=1 なので割込は出ない)
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->CCR1 = 0;               // プリロード: オーバーフロー後に有効
}

// ---------- スキャン ----------
// プレーン点灯終了 (/OE=High) ごとに呼ばれる。このとき次に表示するプレーンの
// データはシフトレジスタに入っているので、ラッチして点灯を始め、その点灯中に
// さらに次のプレーンを送る。
static void mbiPlaneEnd() {
  if (mbiPlane == 0) {
    srShiftBit(mbiRow != 0);     // 行0 のときだけ '0' を投入
    srLatch();
  }
  mbiLatch();
  mbiOePulse(MBI_T0_TICKS << mbiPlane);

  uint8_t plane = mbiPlane + 1, row = mbiRow;
  if (plane == MBI_PLANES) {
    plane = 0;
    row = (row + 1) & (MATRIX_H - 1);
    if (row == 0 && mbiSwapPending) {
      mbiFront ^= 1;
      mbiSwapPending = false;
    }
  }
  mbiShift48(mbiPlanes[mbiFront][row][plane]);
  mbiPlane = plane;
  mbiRow = row;
}

// ---------- 公開 API ----------
// mbiGray をビットプレーンに展開し、次のフレーム先頭で表示を切り替える。
// 前回の切替が未完了なら待つ (裏面を書き換え中に ISR が取り込まないため)。
static void mbiUploadFrame() {
  while (mbiSwapPending) {}
  uint8_t back = mbiFront ^ 1;
  for (uint8_t y = 0; y < MATRIX_H; y++) {
    for (uint8_t b = 0; b < MBI_PLANES; b++) {
      uint8_t* s = mbiPlanes[back][y][b];
      for (uint8_t c = 0; c < 3; c++) {          // ストリーム順 B, G, R
        const uint8_t* g = mbiGray[2 - c][y];
        uint16_t bits = 0;
        for (uint8_t x = 0; x < MATRIX_W; x++) { // 先頭ビット = OUT15
          bits = (uint16_t)(bits << 1) | ((g[MATRIX_W - 1 - x] >> b) & 1);
        }
        s[c * 2]     = (uint8_t)(bits >> 8);
        s[c * 2 + 1] = (uint8_t)bits;
      }
    }
  }
  mbiSwapPending = true;
}

// 電源投入後 (+3V3P ドメイン復帰後) 毎回呼ぶ。設定レジスタは電源断で消える。
static void mbiBegin() {
  pinMode(PB12, OUTPUT); pinMode(PB13, OUTPUT); pinMode(PB15, OUTPUT);
  pinMode(PB14, INPUT);
  pinMode(PC6, OUTPUT); pinMode(PC7, OUTPUT); pinMode(PD0, OUTPUT);
  pinMode(PD1, OUTPUT_OPEN_DRAIN);              // /OE: リリース側は R104 が 5V に引く
  srRowsEnable(false);
  mbiPin(GPIOB, MBI_LE_BIT, false);
  mbiPin(GPIOB, MBI_DCLK_BIT, false);
  srAllOff();

  // /OE=High で待機 (CCR1=0 → PWM2 active)。MBI 側も内部プルアップで消灯 (p7)
  mbiOeTimer.pause();
  mbiOeTimer.setMode(1, TIMER_OUTPUT_COMPARE_PWM2, PA_8);
  mbiOeTimer.setPrescaleFactor(1);
  mbiOeTimer.setOverflow(MBI_T0_TICKS + 1, TICK_FORMAT);
  mbiOeTimer.setCaptureCompare(1, 0, TICK_COMPARE_FORMAT);
  mbiOeTimer.resume();                          // MOE と出力段を有効化
  mbiOeTimer.pause();
  TIM1->CR1   |= TIM_CR1_OPM | TIM_CR1_URS;
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
  TIM1->EGR    = TIM_EGR_UG;

  mbiWriteConfig();                             // ビットバンギング (LE と CLK の同期が必要)

  mbiSpi.begin();                               // PB13/PB15 を SPI2 へ
  mbiSpi.beginTransaction(SPISettings(MBI_SPI_HZ, MSBFIRST, SPI_MODE0));
  mbiSpi.transfer(0);                           // SPE を立てる。以後は mbiShift48 が DR を直接叩く

  memset(mbiGray, 0, sizeof(mbiGray));
  memset(mbiPlanes, 0, sizeof(mbiPlanes));
  mbiFront = 0;
  mbiSwapPending = false;
  mbiRow = 0;
  mbiPlane = 0;
  mbiShift48(mbiPlanes[0][0][0]);

  mbiOeTimer.setInterruptPriority(0, 0);        // プレーン周期のジッタ防止
  mbiOeTimer.attachInterrupt(mbiPlaneEnd);
  srRowsEnable(true);
  mbiPlaneEnd();                                // 行0 プレーン0 から開始
}

// スリープ前: 行を全OFFにしてから止める。+3V3P/+5V 断の前に呼ぶ。
// USB 給電中はレールが残るため、「消灯した状態」を明示的に作る (S4)。
static void mbiEnd() {
  mbiOeTimer.pause();                           // 以後 ISR は来ない
  mbiOeTimer.detachInterrupt();
  while (TIM1->CR1 & TIM_CR1_CEN) {}            // 進行中の /OE パルスは終わらせる
  mbiSwapPending = false;
  srAllOff();                                   // 全行OFFをラッチしてから
  srRowsEnable(false);                          //   /OE リリース (C2: PD1 はドレイン開放)
  mbiSpi.endTransaction();
  mbiSpi.end();
  pinMode(PA8, INPUT_ANALOG);                   // 未給電 MBI/595 への逆給電防止 (M6)
  pinMode(PB12, INPUT_ANALOG); pinMode(PB13, INPUT_ANALOG); pinMode(PB15, INPUT_ANALOG);
  pinMode(PC6, INPUT_ANALOG); pinMode(PC7, INPUT_ANALOG); pinMode(PD0, INPUT_ANALOG);
}
