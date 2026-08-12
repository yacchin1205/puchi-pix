// NOTE: Build with "Tools > U(S)ART support > Disabled" to fit in 32KB flash
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// =====================
// 画像データ (アイコン差し替えはここだけ変更)
// =====================
#include "frame.h"
#include "icon_original.h"

#ifndef HAS_WALK
#error "icon header predates frame roles; regenerate with resources/generate_image_data.py"
#endif

// =====================
// ディスプレイ抽象化レイヤー
// =====================
#include "display.h"

// デバッグ: loopカウンタをY=0に表示
#define DEBUG_LOOP_COUNTER 0

// 手持ち検出診断: ドリフト量をY=0(X軸)/Y=1(Y軸)にバー表示。
// 閾値=白ティック(x=32)、検出中=右端赤。リリースビルドでは0にする。
#define PUCHI_DIAG_HANDLE 0

// 上方向インジケーター (赤ドット)。フラッシュ節約のためデフォルト無効
#define ENABLE_UP_INDICATOR 0

#if DEBUG_LOOP_COUNTER
static uint16_t dbgLoopCount = 0;

static void drawDebugCounter() {
  uint8_t barLen = dbgLoopCount % DISPLAY_W;
  displayBeginWrite(0, 0, DISPLAY_W, 1);
  for (uint8_t x = 0; x < DISPLAY_W; x++) {
    displayWritePixel(x < barLen ? 0x07E0 : 0x0000);
  }
  displayEndWrite();
}
#endif

// =====================
// KXTJ3 (I2C)
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

// 加速度センサー座標反転
static constexpr int ACCEL_X_SIGN = 1;
static constexpr int ACCEL_Y_SIGN = 1;

// 向き変更後のクールダウン
static constexpr uint32_t ORIENT_COOLDOWN_MS = 1500;
static uint32_t lastOrientChangeMs = 0;
static int8_t pendingOrient = -1;

// アニメーション状態
static uint8_t  curFrame = 0;
static int8_t   lastDrawnFrame = -1;
static uint8_t  currentOrient = 0;
static uint32_t lastFrameTime = 0;

// =====================
// 歩き登場アニメーション
// 右から歩いて入場 → 中割り(入り) → メインループ → 中割り(戻り) → 左へ歩いて退場
// メインは手持ち検出が続く間ループし、ループ境界で退場側へ抜ける。
// =====================
// フレーム役割 (WALK_SEQ / FRAME_INTRO_START / FRAME_MAIN_START /
// FRAME_OUTRO_START) はアイコンヘッダが定義する。FRAME_NONE は省略。
// introチェーンは curFrame が FRAME_MAIN_START に達したところで終端。
// mainチェーンは FRAME_MAIN_START に戻る循環。
// outroチェーンは next が WALK_SEQ[0] に戻ったところで終端。
static uint8_t walkPhase = 0;  // WALK_SEQ内の現在位置

static constexpr int16_t  WALK_CENTER_X = (DISPLAY_W - IMG_W) / 2;
static constexpr int16_t  WALK_ENTER_X  = DISPLAY_W;   // 右端の画面外
static constexpr int16_t  WALK_EXIT_X   = -IMG_W;      // 左端の画面外
static constexpr uint8_t  WALK_STEP_PX  = 4;
static constexpr uint16_t WALK_STEP_MS  = 150;
static constexpr uint16_t WALK_PAUSE_MS = 800;         // 退場後、再入場までの待ち

enum WalkMode : uint8_t {
  MODE_WALK_IN, MODE_INTRO, MODE_MAIN, MODE_OUTRO, MODE_WALK_OUT, MODE_OFFSCREEN
};
static uint8_t walkMode = MODE_WALK_IN;
static int16_t spriteX  = WALK_ENTER_X;  // 見た目上のX位置 (orient 3では鏡映して描画)

// ---------- KXTJ3 low-level ----------
static inline void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(KXTJ3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// ---------- Display Power / Sleep ----------
// HAS_WALK: フェーズ駆動。退場後(OFFSCREEN)は即減光し、OFFSCREEN_SLEEP_MS
// でスリープ。再生中は減光しない。
// 単純ループ (HAS_WALK 0): 従来どおり lastActivityMs からのタイマー駆動。
static constexpr uint32_t DIM_TIMEOUT_MS      = 10000;
static constexpr uint32_t SLEEP_TIMEOUT_MS    = 30000;
static constexpr uint32_t OFFSCREEN_SLEEP_MS  = 10000;

static uint32_t lastActivityMs = 0;
static uint32_t offscreenEnterMs = 0;

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

// 分解能切り替え。手持ち検出には12bitが必要 (8bitは1LSB≈256カウントで、
// 手持ちの微小ドリフトが量子化に埋もれて検出が続かない)。スリープ中は
// 省電力の8bitに戻す (12bit: 155µA / 8bit: 10µA)。ウェイク機能の
// サンプリングはOWUF (CTRL2) 独立なのでRESの影響を受けない。
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

void wakeupCallback() {
  lastActivityMs = millis();
}

// ---------- 図形描画 ----------
static void displayFillCircle(int x0, int y0, int rad, uint8_t r8, uint8_t g8, uint8_t b8) {
  if (rad <= 0) return;

  int f = 1 - rad;
  int ddF_x = 1;
  int ddF_y = -2 * rad;
  int x = 0;
  int y = rad;

  displayHLine(x0 - rad, y0, 2 * rad + 1, r8, g8, b8);

  while (x < y) {
    if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
    x++; ddF_x += 2; f += ddF_x;

    displayHLine(x0 - x, y0 + y, 2 * x + 1, r8, g8, b8);
    displayHLine(x0 - x, y0 - y, 2 * x + 1, r8, g8, b8);
    displayHLine(x0 - y, y0 + x, 2 * y + 1, r8, g8, b8);
    displayHLine(x0 - y, y0 - x, 2 * y + 1, r8, g8, b8);
  }
}

// ---------- Frame描画 ----------
// Orientation: 0=normal, 3=180°(逆さ)。90°(orient 1/2)は非対応。

static inline uint8_t read4bit(const uint8_t* data, uint16_t pixelIdx) {
  uint8_t b = pgm_read_byte(&data[pixelIdx >> 1]);
  return (pixelIdx & 1) ? (b & 0x0F) : (b >> 4);
}

// オーバーレイ連鎖: frameIdx から ref を辿り、ピクセルごとに最初に領域が
// 当たったオーバーレイのデータを使う。当たらなければ refDy だけシフトして
// 次の ref へ。フルフレームまで到達したらそこから読む。
static constexpr uint8_t MAX_OVL_CHAIN = 8;

static uint8_t buildOvlChain(uint8_t frameIdx, OvlDesc* chain, const uint8_t** fullOut) {
  uint8_t depth = 0;
  uint8_t f = frameIdx;
  while (pgm_read_byte(&frames[f].type) == 1 && depth < MAX_OVL_CHAIN) {
    chain[depth].data = (const uint8_t*)pgm_read_ptr(&frames[f].data);
    chain[depth].rx = pgm_read_byte(&frames[f].rx);
    chain[depth].ry = pgm_read_byte(&frames[f].ry);
    chain[depth].rw = pgm_read_byte(&frames[f].rw);
    chain[depth].rh = pgm_read_byte(&frames[f].rh);
    chain[depth].refDy = (int8_t)pgm_read_byte(&frames[f].refDy);
    depth++;
    f = pgm_read_byte(&frames[f].ref);
  }
  *fullOut = (const uint8_t*)pgm_read_ptr(&frames[f].data);
  return depth;
}

static uint8_t chainPixelIdx(const OvlDesc* chain, uint8_t depth, const uint8_t* full,
                             uint8_t srcX, int16_t srcY) {
  for (uint8_t d = 0; d < depth; d++) {
    const OvlDesc& o = chain[d];
    if (srcX >= o.rx && srcX < o.rx + o.rw && srcY >= o.ry && srcY < o.ry + o.rh) {
      return read4bit(o.data, (uint16_t)(srcY - o.ry) * o.rw + (srcX - o.rx));
    }
    srcY -= o.refDy;
    if (srcY < 0 || srcY >= IMG_H) return 0;  // シフトで画像外に出た行は背景
  }
  return read4bit(full, (uint16_t)srcY * IMG_W + srcX);
}

// ---------- 歩きスプライト描画 (任意X位置、画面クリップ付き) ----------
static inline int16_t walkScreenX(uint8_t orient) {
  // orient 3 (180°) は描画が点対称に反転するので、見た目の位置を保つため鏡映する
  return (orient == 3) ? (int16_t)(DISPLAY_W - IMG_W) - spriteX : spriteX;
}

static void drawSpriteAt(uint8_t frameIdx, int16_t posX, uint8_t orient) {
  OvlDesc chain[MAX_OVL_CHAIN];
  const uint8_t* full;
  uint8_t depth = buildOvlChain(frameIdx, chain, &full);
  int16_t sx0 = posX < 0 ? 0 : posX;
  int16_t sx1 = (posX + IMG_W > DISPLAY_W) ? DISPLAY_W : posX + IMG_W;
  if (sx0 >= sx1) return;
  bool flip = (orient == 3);
  for (uint8_t dy = 0; dy < IMG_H; dy++) {
    displayBeginWrite(sx0, dy, sx1 - sx0, 1);
    for (int16_t sx = sx0; sx < sx1; sx++) {
      uint8_t lx = (uint8_t)(sx - posX);
      uint8_t srcX = flip ? (IMG_W - 1 - lx) : lx;
      uint8_t srcY = flip ? (IMG_H - 1 - dy) : dy;
      uint8_t idx = chainPixelIdx(chain, depth, full, srcX, srcY);
      displayWritePixel(pgm_read_word(&palette[idx]));
    }
    displayEndWrite();
  }
}

static void clearColumns(int16_t x0, int16_t x1) {  // [x0, x1) を黒でクリア
  if (x0 < 0) x0 = 0;
  if (x1 > DISPLAY_W) x1 = DISPLAY_W;
  if (x0 >= x1) return;
  for (uint8_t y = 0; y < DISPLAY_H; y++) {
    displayHLine(x0, y, x1 - x0, 0, 0, 0);
  }
}

#if HAS_WALK
// 中央のチェーン再生を終えて左へ歩き出す
static void startWalkOut() {
  walkMode = MODE_WALK_OUT;
  walkPhase = 0;
  curFrame = WALK_SEQ[0];
}
#endif

// アニメーションを初期状態に戻す (HAS_WALK: 右から歩き入場)
static void resetAnimation() {
#if HAS_WALK
  walkPhase = 0;
  curFrame = WALK_SEQ[0];
  walkMode = MODE_WALK_IN;
  spriteX = WALK_ENTER_X;
#else
  curFrame = 0;
  spriteX = WALK_CENTER_X;
#endif
  lastDrawnFrame = -1;
  displayFillScreen(0, 0, 0);
}

#if HAS_WALK
static void walkStep(uint8_t orient) {
  int16_t oldPos = walkScreenX(orient);
  spriteX -= WALK_STEP_PX;
  if (walkMode == MODE_WALK_IN && spriteX < WALK_CENTER_X) spriteX = WALK_CENTER_X;
  int16_t newPos = walkScreenX(orient);
  walkPhase = (uint8_t)((walkPhase + 1) % sizeof(WALK_SEQ));
  curFrame = WALK_SEQ[walkPhase];
  drawSpriteAt(curFrame, newPos, orient);
  // 移動で空いた跡を消す
  if (newPos > oldPos) clearColumns(oldPos, newPos);
  else if (newPos < oldPos) clearColumns(newPos + IMG_W, oldPos + IMG_W);
  lastDrawnFrame = -1;  // 位置が動いたので差分描画の前提を無効化
}
#endif  // HAS_WALK

static void drawCurrentFrame(uint8_t frameIdx, uint8_t orient) {
  currentOrient = orient;
  drawSpriteAt(frameIdx, walkScreenX(orient), orient);
  lastDrawnFrame = frameIdx;
}

// ---------- 向き変更トランジション ----------
static void onOrientChange(uint8_t newOrient) {
  displayFade(true);
  displayReset();

  currentOrient = newOrient;
  resetAnimation();

  displayFade(false);
  delay(250);
  lastOrientChangeMs = millis();
}

// ---------- KXTJ3 ----------
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

// ---------- motion ----------
static int16_t xRef=0, yRef=0, zRef=0;
static uint8_t calibrated=0;

static void calibrateKXTJ3(uint8_t samples=80) {
  long sx=0, sy=0, sz=0;
  uint8_t got=0;
  for (uint8_t i=0; i<samples; i++) {
    int16_t rx, ry, rz;
    if (readXYZRaw(rx, ry, rz)) { sx += rx; sy += ry; sz += rz; got++; }
    delay(10);
  }
  if (!got) return;
  xRef = (int16_t)(sx / got);
  yRef = (int16_t)(sy / got);
  zRef = (int16_t)(sz / got);
  calibrated = 1;
}

static uint8_t classifyOrientation(int16_t rx, int16_t ry) {
  int16_t dx = (xRef - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - yRef) * ACCEL_Y_SIGN;

  const int16_t thresholdIn = 8000;
  const int16_t thresholdOut = 4000;

  // 上下 (orient 0/3) のみ判定。90°横向きは非対応
  int16_t absDx = dx > 0 ? dx : -dx;
  int16_t absDy = dy > 0 ? dy : -dy;

  if (absDy <= thresholdOut || absDx > absDy) return currentOrient;
  if (currentOrient == 3 && dy < 0) return 3;
  if (currentOrient == 0 && dy > 0) return 0;
  if (absDy > thresholdIn) return (dy < 0) ? 3 : 0;
  return currentOrient;
}

// ---------- 手持ち検出 (向きドリフト) ----------
// 速いローパス(τ=200ms)と遅いローパス(τ=2000ms)の差が閾値を超えた状態が
// 続いたら「手に持っている」と判定する。瞬時値では静置ノイズと静かな
// 手持ちを分離できない (ESP32での実測知見)。
// 固定小数点は<<6: diffが最大2^22なのでdtを256msでクランプすれば
// int32乗算に収まる (M0+でint64を避ける)。
static constexpr uint32_t LP_TAU_MS        = 200;
static constexpr uint32_t LP_SLOW_TAU_MS   = 2000;
static constexpr uint32_t HANDLE_SAMPLE_MS = 20;  // KXTJ3 ODR 50Hzに合わせる
static constexpr int32_t  HANDLE_DRIFT_THRESH = 300;
static constexpr uint8_t  HANDLE_DEBOUNCE  = 3;
static constexpr uint32_t HANDLE_HOLD_MS   = 2000;

static int32_t xLpFP, yLpFP, xSlowFP, ySlowFP;  // <<6固定小数点
static bool     lpInit = false;
static uint32_t lastLpMs = 0;
static uint8_t  driftCount = 0;
static uint32_t lastHandleMs = 0;

#if PUCHI_DIAG_HANDLE
static int32_t diagDriftX = 0, diagDriftY = 0;
#endif

static bool handleHeld(uint32_t now) {
  return (now - lastHandleMs) < HANDLE_HOLD_MS;
}

static void updateHandling(int16_t rx, int16_t ry, uint32_t now) {
  if (!lpInit) {
    xLpFP = (int32_t)rx << 6;
    yLpFP = (int32_t)ry << 6;
    xSlowFP = xLpFP;
    ySlowFP = yLpFP;
    lastLpMs = now;
    lpInit = true;
    return;
  }
  uint32_t dt = now - lastLpMs;
  if (dt < HANDLE_SAMPLE_MS) return;
  lastLpMs = now;
  if (dt > 256) dt = 256;
  int32_t dtF = (int32_t)(dt > LP_TAU_MS ? LP_TAU_MS : dt);
  xLpFP += (((int32_t)rx << 6) - xLpFP) * dtF / (int32_t)LP_TAU_MS;
  yLpFP += (((int32_t)ry << 6) - yLpFP) * dtF / (int32_t)LP_TAU_MS;
  xSlowFP += (xLpFP - xSlowFP) * (int32_t)dt / (int32_t)LP_SLOW_TAU_MS;
  ySlowFP += (yLpFP - ySlowFP) * (int32_t)dt / (int32_t)LP_SLOW_TAU_MS;

  int32_t dx = (xLpFP - xSlowFP) >> 6;
  int32_t dy = (yLpFP - ySlowFP) >> 6;
  if (dx < 0) dx = -dx;
  if (dy < 0) dy = -dy;
#if PUCHI_DIAG_HANDLE
  diagDriftX = dx;
  diagDriftY = dy;
#endif
  if (dx > HANDLE_DRIFT_THRESH || dy > HANDLE_DRIFT_THRESH) {
    if (driftCount < HANDLE_DEBOUNCE) driftCount++;
    if (driftCount >= HANDLE_DEBOUNCE) {
      lastHandleMs = now;
      lastActivityMs = now;  // 手持ち中に減光させない
    }
  } else {
    driftCount = 0;
  }
}

#if PUCHI_DIAG_HANDLE
static void drawHandleDiag(bool held) {
  for (uint8_t row = 0; row < 2; row++) {
    int32_t drift = row ? diagDriftY : diagDriftX;
    int32_t len = drift * 32 / HANDLE_DRIFT_THRESH;  // 閾値がx=32に対応
    if (len > DISPLAY_W) len = DISPLAY_W;
    displayBeginWrite(0, row, DISPLAY_W, 1);
    for (uint8_t x = 0; x < DISPLAY_W; x++) {
      uint16_t c = 0x0000;
      if ((int32_t)x < len) c = row ? 0x07FF : 0x07E0;
      if (x == 32) c = 0xFFFF;
      if (held && x >= DISPLAY_W - 4) c = 0xF800;
      displayWritePixel(c);
    }
    displayEndWrite();
  }
}
#endif

// ---------- 上方向インジケーター ----------
#if ENABLE_UP_INDICATOR
static int16_t prevIndX = -1, prevIndY = -1;

static void restorePixel(int16_t sx, int16_t sy) {
  if (sx < 0 || sx >= DISPLAY_W || sy < 0 || sy >= DISPLAY_H) return;

  int16_t imgX = walkScreenX(currentOrient);
  uint16_t color;

  if (sx >= imgX && sx < imgX + IMG_W && sy < IMG_H) {
    uint8_t localX = sx - imgX;
    uint8_t localY = sy;
    uint8_t srcX = (currentOrient == 3) ? (IMG_W - 1 - localX) : localX;
    uint8_t srcY = (currentOrient == 3) ? (IMG_H - 1 - localY) : localY;
    OvlDesc chain[MAX_OVL_CHAIN];
    const uint8_t* full;
    uint8_t depth = buildOvlChain(curFrame, chain, &full);
    uint8_t palIdx = chainPixelIdx(chain, depth, full, srcX, srcY);
    color = pgm_read_word(&palette[palIdx]);
  } else {
    color = 0x0000;
  }

  displayBeginWrite(sx, sy, 1, 1);
  displayWritePixel(color);
  displayEndWrite();
}

static void drawUpIndicator() {
  int16_t rx, ry, rz;
  if (!calibrated || !readXYZRaw(rx, ry, rz)) return;

  int16_t dx = (xRef - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - yRef) * ACCEL_Y_SIGN;

  const int16_t maxTilt = 8000;
  int16_t posX = DISPLAY_W / 2 - (dx * (DISPLAY_W / 2 - 4)) / maxTilt;
  int16_t posY = DISPLAY_H / 2 - (dy * (DISPLAY_H / 2 - 4)) / maxTilt;

  if (posX < 2) posX = 2;
  if (posX > DISPLAY_W - 4) posX = DISPLAY_W - 4;
  if (posY < 2) posY = 2;
  if (posY > DISPLAY_H - 4) posY = DISPLAY_H - 4;

  if (prevIndX >= 0 && (prevIndX != posX || prevIndY != posY)) {
    for (int dy = 0; dy < 2; dy++)
      for (int dx = 0; dx < 2; dx++)
        restorePixel(prevIndX + dx, prevIndY + dy);
  }

  displayBeginWrite(posX, posY, 2, 2);
  for (int i = 0; i < 4; i++) displayWritePixel(0xF800);
  displayEndWrite();

  prevIndX = posX;
  prevIndY = posY;
}
#endif  // ENABLE_UP_INDICATOR

// HSI 8MHz, PLLなし (省電力)
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

void setup() {
  displayInit();

  Wire.setSDA(PB_7);
  Wire.setSCL(PB_6);
  Wire.begin();
  Wire.setClock(100000);

  write8(REG_CTRL1, 0x00); delay(10);
  write8(REG_DATA_CTRL, 0x02); delay(10);
  write8(REG_CTRL1, 0xC0); delay(50);

  calibrateKXTJ3(20);
  enableWakeupInterrupt();
  attachInterrupt(digitalPinToInterrupt(KXTJ3_INT_PIN), wakeupCallback, FALLING);

  resetAnimation();
  lastActivityMs = millis();
  lastFrameTime = millis();
  lastHandleMs = millis() - HANDLE_HOLD_MS;  // 起動直後は「手持ちでない」
}

void loop() {
  uint32_t now = millis();
#if HAS_WALK
  // フェーズ駆動: 退場後のOFFSCREENでのみ減光/スリープする
  bool dimNow = (walkMode == MODE_OFFSCREEN);
  bool sleepNow = dimNow && (now - offscreenEnterMs >= OFFSCREEN_SLEEP_MS);
#else
  uint32_t elapsed = now - lastActivityMs;
  bool dimNow = (elapsed >= DIM_TIMEOUT_MS);
  bool sleepNow = (elapsed >= SLEEP_TIMEOUT_MS);
#endif

  // スリープ
  if (sleepNow) {
    displayBrightness(BRIGHT_OFF);
    setAccelHighRes(false);
    clearLatchedInterrupt();
    HAL_SuspendTick();
    __DSB();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    setAccelHighRes(true);
    displayBrightness(BRIGHT_FULL);
    lastActivityMs = millis();
    lastFrameTime = millis();
    resetAnimation();
    return;
  }

  // ディム
  displayBrightness(dimNow ? BRIGHT_DIM : BRIGHT_FULL);

  // 加速度読み取り (向き検出と手持ち検出で共有)
  int16_t rx, ry, rz;
  bool accelOk = calibrated && readXYZRaw(rx, ry, rz);
  if (accelOk) updateHandling(rx, ry, now);

  // 傾き検出 (クールダウン中はキューに溜める)
  uint8_t orient = accelOk ? classifyOrientation(rx, ry) : currentOrient;
  bool inCooldown = (now - lastOrientChangeMs) < ORIENT_COOLDOWN_MS;

  if (orient != currentOrient) {
    if (inCooldown) {
      pendingOrient = orient;
      orient = currentOrient;
    } else {
      lastActivityMs = now;
      lastOrientChangeMs = now;
      pendingOrient = -1;
      onOrientChange(orient);
      lastFrameTime = millis();
      return;
    }
  } else if (!inCooldown && pendingOrient >= 0 && (uint8_t)pendingOrient != currentOrient) {
    orient = (uint8_t)pendingOrient;
    pendingOrient = -1;
    lastActivityMs = now;
    lastOrientChangeMs = now;
    onOrientChange(orient);
    lastFrameTime = millis();
    return;
  }

  // アニメーション進行
#if HAS_WALK
  if (walkMode == MODE_INTRO || walkMode == MODE_MAIN || walkMode == MODE_OUTRO) {
    // 中央でのフレームチェーン再生
    uint16_t duration = pgm_read_word(&frames[curFrame].duration_ms);
    if (now - lastFrameTime >= duration) {
      lastFrameTime = now;
      uint8_t nextFrame = pgm_read_byte(&frames[curFrame].next);
      if (walkMode == MODE_MAIN && nextFrame == FRAME_MAIN_START && !handleHeld(now)) {
        // ループ境界で手持ちが切れていたら退場側へ
        if (FRAME_OUTRO_START != FRAME_NONE) {
          walkMode = MODE_OUTRO;
          curFrame = FRAME_OUTRO_START;
        } else {
          startWalkOut();
        }
      } else if (walkMode == MODE_OUTRO && nextFrame == WALK_SEQ[0]) {
        // outroチェーン終端 → 左へ歩き出す
        startWalkOut();
      } else {
        curFrame = nextFrame;
        if (walkMode == MODE_INTRO && curFrame == FRAME_MAIN_START) walkMode = MODE_MAIN;
      }
    }
    if (lastDrawnFrame < 0 || (uint8_t)lastDrawnFrame != curFrame) {
      drawCurrentFrame(curFrame, orient);
    }
  } else if (walkMode == MODE_OFFSCREEN) {
    // 手持ち検出があったときだけ次のサイクルへ (WALK_PAUSE_MSは最小間隔)
    if (now - offscreenEnterMs >= WALK_PAUSE_MS && handleHeld(now)) {
      lastFrameTime = now;
      walkMode = MODE_WALK_IN;
      spriteX = WALK_ENTER_X;
      walkPhase = 0;
      curFrame = WALK_SEQ[0];
    }
  } else {  // MODE_WALK_IN / MODE_WALK_OUT
    if (now - lastFrameTime >= WALK_STEP_MS) {
      lastFrameTime = now;
      if (walkMode == MODE_WALK_IN && spriteX <= WALK_CENTER_X) {
        // 中央到達 → 中割り(なければメイン)へ
        walkMode = (FRAME_INTRO_START != FRAME_NONE) ? MODE_INTRO : MODE_MAIN;
        curFrame = (walkMode == MODE_INTRO) ? FRAME_INTRO_START : FRAME_MAIN_START;
        lastDrawnFrame = -1;
        drawCurrentFrame(curFrame, orient);
      } else if (walkMode == MODE_WALK_OUT && spriteX <= WALK_EXIT_X) {
        walkMode = MODE_OFFSCREEN;
        offscreenEnterMs = now;
      } else {
        walkStep(orient);
      }
    }
  }
#else  // 単純ループ: nextチェーンをそのまま再生
  {
    uint16_t duration = pgm_read_word(&frames[curFrame].duration_ms);
    if (now - lastFrameTime >= duration) {
      lastFrameTime = now;
      curFrame = pgm_read_byte(&frames[curFrame].next);
    }
    if (lastDrawnFrame < 0 || (uint8_t)lastDrawnFrame != curFrame) {
      drawCurrentFrame(curFrame, orient);
    }
  }
#endif

#if ENABLE_UP_INDICATOR
  drawUpIndicator();
#endif

#if PUCHI_DIAG_HANDLE
  static uint32_t lastDiagMs = 0;
  if (now - lastDiagMs >= 50) {
    lastDiagMs = now;
    drawHandleDiag(handleHeld(now));
  }
#endif

#if DEBUG_LOOP_COUNTER
  static uint32_t dbgLastTime = 0;
  if (now - dbgLastTime >= 100) {
    dbgLastTime = now;
    dbgLoopCount++;
    drawDebugCounter();
  }
#endif
}
