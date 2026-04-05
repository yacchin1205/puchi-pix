// NOTE: Build with "Tools > U(S)ART support > Disabled" to fit in 32KB flash
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// =====================
// 画像データ (アイコン差し替えはここだけ変更)
// =====================
#include "frame.h"
#include "icon_dotpict2.h"

// =====================
// ディスプレイ抽象化レイヤー
// =====================
#include "display.h"

// デバッグ: loopカウンタをY=0に表示
#define DEBUG_LOOP_COUNTER 1

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
// Orientation: 0=normal, 1=90°CW(左傾き), 2=90°CCW(右傾き), 3=180°(上傾き)

static inline uint16_t getStartX(uint8_t orient) {
  if (orient == 1) return 0;
  if (orient == 2) return DISPLAY_W - IMG_W;
  return (DISPLAY_W - IMG_W) / 2;
}

static inline uint16_t getSrcIndex(uint8_t x, uint8_t y, uint8_t orient, uint8_t w, uint8_t h) {
  uint8_t srcX, srcY;
  switch (orient) {
    case 1:  srcX = y; srcY = (h - 1) - x; break;
    case 2:  srcX = (w - 1) - y; srcY = x; break;
    case 3:  srcX = (w - 1) - x; srcY = (h - 1) - y; break;
    default: srcX = x; srcY = y; break;
  }
  return (uint16_t)srcY * w + srcX;
}

static inline uint8_t read4bit(const uint8_t* data, uint16_t pixelIdx) {
  uint8_t b = pgm_read_byte(&data[pixelIdx >> 1]);
  return (pixelIdx & 1) ? (b & 0x0F) : (b >> 4);
}

static void drawFullFrame(const uint8_t* data, uint8_t orient) {
  uint16_t screenX = getStartX(orient);
  for (uint8_t dy = 0; dy < IMG_H; dy++) {
    displayBeginWrite(screenX, dy, IMG_W, 1);
    for (uint8_t dx = 0; dx < IMG_W; dx++) {
      uint16_t srcIdx = getSrcIndex(dx, dy, orient, IMG_W, IMG_H);
      uint8_t idx = read4bit(data, srcIdx);
      displayWritePixel(pgm_read_word(&palette[idx]));
    }
    displayEndWrite();
  }
}

static void drawOverlay(const uint8_t* data, uint8_t orient,
                        uint8_t srcX, uint8_t srcY, uint8_t srcW, uint8_t srcH) {
  uint8_t dstX, dstY, dstW, dstH;
  switch (orient) {
    case 1:  dstX = (IMG_H-1)-(srcY+srcH-1); dstY = srcX; dstW = srcH; dstH = srcW; break;
    case 2:  dstX = srcY; dstY = (IMG_W-1)-(srcX+srcW-1); dstW = srcH; dstH = srcW; break;
    case 3:  dstX = (IMG_W-1)-(srcX+srcW-1); dstY = (IMG_H-1)-(srcY+srcH-1); dstW = srcW; dstH = srcH; break;
    default: dstX = srcX; dstY = srcY; dstW = srcW; dstH = srcH; break;
  }
  uint16_t screenX = getStartX(orient) + dstX;
  for (uint8_t dy = 0; dy < dstH; dy++) {
    displayBeginWrite(screenX, dstY + dy, dstW, 1);
    for (uint8_t dx = 0; dx < dstW; dx++) {
      uint16_t srcIdx = getSrcIndex(dx, dy, orient, srcW, srcH);
      uint8_t idx = read4bit(data, srcIdx);
      displayWritePixel(pgm_read_word(&palette[idx]));
    }
    displayEndWrite();
  }
}

static void drawRegionFromFull(const uint8_t* fullData, uint8_t orient,
                                uint8_t rx, uint8_t ry, uint8_t rw, uint8_t rh) {
  uint8_t dstX, dstY, dstW, dstH;
  switch (orient) {
    case 1:  dstX = (IMG_H-1)-(ry+rh-1); dstY = rx; dstW = rh; dstH = rw; break;
    case 2:  dstX = ry; dstY = (IMG_W-1)-(rx+rw-1); dstW = rh; dstH = rw; break;
    case 3:  dstX = (IMG_W-1)-(rx+rw-1); dstY = (IMG_H-1)-(ry+rh-1); dstW = rw; dstH = rh; break;
    default: dstX = rx; dstY = ry; dstW = rw; dstH = rh; break;
  }
  uint16_t screenX = getStartX(orient) + dstX;
  for (uint8_t dy = 0; dy < dstH; dy++) {
    displayBeginWrite(screenX, dstY + dy, dstW, 1);
    for (uint8_t dx = 0; dx < dstW; dx++) {
      uint16_t srcIdx = getSrcIndex(dstX + dx, dstY + dy, orient, IMG_W, IMG_H);
      uint8_t idx = read4bit(fullData, srcIdx);
      displayWritePixel(pgm_read_word(&palette[idx]));
    }
    displayEndWrite();
  }
}

static void clearMargins(uint8_t orient) {
  uint16_t imgX = getStartX(orient);
  uint16_t rightX = imgX + IMG_W;
  for (uint8_t y = 0; y < DISPLAY_H; y++) {
    if (imgX > 0) {
      displayBeginWrite(0, y, imgX, 1);
      for (uint8_t x = 0; x < imgX; x++) displayWritePixel(0x0000);
      displayEndWrite();
    }
    if (rightX < DISPLAY_W) {
      displayBeginWrite(rightX, y, DISPLAY_W - rightX, 1);
      for (uint8_t x = rightX; x < DISPLAY_W; x++) displayWritePixel(0x0000);
      displayEndWrite();
    }
  }
}

static const uint8_t* getActiveFullFrameData(uint8_t frameIdx) {
  uint8_t ftype = pgm_read_byte(&frames[frameIdx].type);
  if (ftype == 0) return (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);
  uint8_t ref = pgm_read_byte(&frames[frameIdx].ref);
  return (const uint8_t*)pgm_read_ptr(&frames[ref].data);
}

static void drawCurrentFrame(uint8_t frameIdx, uint8_t orient) {
  uint8_t curType = pgm_read_byte(&frames[frameIdx].type);
  const uint8_t* curData = (const uint8_t*)pgm_read_ptr(&frames[frameIdx].data);

  if (lastDrawnFrame < 0 || orient != currentOrient) {
    clearMargins(orient);
    currentOrient = orient;
    if (curType == 0) {
      drawFullFrame(curData, orient);
    } else {
      uint8_t curRef = pgm_read_byte(&frames[frameIdx].ref);
      drawFullFrame((const uint8_t*)pgm_read_ptr(&frames[curRef].data), orient);
      drawOverlay(curData, orient,
        pgm_read_byte(&frames[frameIdx].rx), pgm_read_byte(&frames[frameIdx].ry),
        pgm_read_byte(&frames[frameIdx].rw), pgm_read_byte(&frames[frameIdx].rh));
    }
    lastDrawnFrame = frameIdx;
    return;
  }

  if ((uint8_t)lastDrawnFrame == frameIdx) return;

  uint8_t prevIdx = (uint8_t)lastDrawnFrame;
  uint8_t prevType = pgm_read_byte(&frames[prevIdx].type);

  if (curType == 0) {
    if (prevType == 1 && pgm_read_byte(&frames[prevIdx].ref) == frameIdx) {
      drawRegionFromFull(curData, orient,
        pgm_read_byte(&frames[prevIdx].rx), pgm_read_byte(&frames[prevIdx].ry),
        pgm_read_byte(&frames[prevIdx].rw), pgm_read_byte(&frames[prevIdx].rh));
    } else {
      drawFullFrame(curData, orient);
    }
  } else {
    uint8_t curRef = pgm_read_byte(&frames[frameIdx].ref);
    uint8_t rx = pgm_read_byte(&frames[frameIdx].rx);
    uint8_t ry = pgm_read_byte(&frames[frameIdx].ry);
    uint8_t rw = pgm_read_byte(&frames[frameIdx].rw);
    uint8_t rh = pgm_read_byte(&frames[frameIdx].rh);

    if (prevType == 0 && curRef == prevIdx) {
      drawOverlay(curData, orient, rx, ry, rw, rh);
    } else if (prevType == 1 && pgm_read_byte(&frames[prevIdx].ref) == curRef) {
      uint8_t prevRx = pgm_read_byte(&frames[prevIdx].rx);
      uint8_t prevRy = pgm_read_byte(&frames[prevIdx].ry);
      uint8_t prevRw = pgm_read_byte(&frames[prevIdx].rw);
      uint8_t prevRh = pgm_read_byte(&frames[prevIdx].rh);
      if (prevRx != rx || prevRy != ry || prevRw != rw || prevRh != rh) {
        drawRegionFromFull((const uint8_t*)pgm_read_ptr(&frames[curRef].data),
          orient, prevRx, prevRy, prevRw, prevRh);
      }
      drawOverlay(curData, orient, rx, ry, rw, rh);
    } else {
      drawFullFrame((const uint8_t*)pgm_read_ptr(&frames[curRef].data), orient);
      drawOverlay(curData, orient, rx, ry, rw, rh);
    }
  }

  lastDrawnFrame = frameIdx;
}

// ---------- 向き変更トランジション ----------
static void onOrientChange(uint8_t newOrient) {
  displayFade(true);
  displayReset();

  currentOrient = newOrient;
  curFrame = 0;
  lastDrawnFrame = -1;
  displayFillScreen(0, 0, 0);

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
static float posX=0, posY=0, velX=0, velY=0;
static int16_t xRef=0, yRef=0, zRef=0;
static uint8_t calibrated=0;
static float fdx=0, fdy=0;
static int prevCx=-9999, prevCy=-9999;

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

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
  fdx = fdy = 0;
  calibrated = 1;
}

static uint8_t detectOrientation() {
  int16_t rx, ry, rz;
  if (!calibrated || !readXYZRaw(rx, ry, rz)) return currentOrient;

  int16_t dx = (xRef - rx) * ACCEL_X_SIGN;
  int16_t dy = (ry - yRef) * ACCEL_Y_SIGN;

  const int16_t thresholdIn = 8000;
  const int16_t thresholdOut = 4000;

  int16_t absDx = dx > 0 ? dx : -dx;
  int16_t absDy = dy > 0 ? dy : -dy;
  int16_t maxTilt = absDx > absDy ? absDx : absDy;

  if (maxTilt > thresholdOut) {
    switch (currentOrient) {
      case 1:  if (absDx > absDy && dx < 0) return 1; break;
      case 2:  if (absDx > absDy && dx > 0) return 2; break;
      case 3:  if (absDy > absDx && dy < 0) return 3; break;
      case 0:  if (absDy > absDx && dy > 0) return 0; break;
    }
  } else {
    return currentOrient;
  }

  if (maxTilt > thresholdIn) {
    if (absDx > absDy) return (dx > 0) ? 2 : 1;
    else return (dy < 0) ? 3 : 0;
  }

  return currentOrient;
}

// ---------- 上方向インジケーター ----------
static int16_t prevIndX = -1, prevIndY = -1;

static void restorePixel(int16_t sx, int16_t sy) {
  if (sx < 0 || sx >= DISPLAY_W || sy < 0 || sy >= DISPLAY_H) return;

  uint16_t imgX = getStartX(currentOrient);
  uint16_t color;

  if (sx >= imgX && sx < imgX + IMG_W && sy < IMG_H) {
    uint8_t localX = sx - imgX;
    uint8_t localY = sy;
    uint16_t srcIdx = getSrcIndex(localX, localY, currentOrient, IMG_W, IMG_H);
    const uint8_t* fullData = getActiveFullFrameData(curFrame);
    uint8_t palIdx = read4bit(fullData, srcIdx);
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
  displayFillScreen(255, 255, 255);

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

  curFrame = 0;
  lastDrawnFrame = -1;
  displayFillScreen(0, 0, 0);
  lastActivityMs = millis();
  lastFrameTime = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - lastActivityMs;

  // スリープ
  if (elapsed >= SLEEP_TIMEOUT_MS) {
    displayBrightness(BRIGHT_OFF);
    clearLatchedInterrupt();
    HAL_SuspendTick();
    __DSB();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    displayBrightness(BRIGHT_FULL);
    lastActivityMs = millis();
    lastFrameTime = millis();
    curFrame = 0;
    lastDrawnFrame = -1;
    displayFillScreen(0, 0, 0);
    return;
  }

  // ディム
  if (elapsed >= DIM_TIMEOUT_MS) {
    displayBrightness(BRIGHT_DIM);
  } else {
    displayBrightness(BRIGHT_FULL);
  }

  // 傾き検出 (クールダウン中はキューに溜める)
  uint8_t orient = detectOrientation();
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

  // フレーム進行
  uint16_t duration = pgm_read_word(&frames[curFrame].duration_ms);
  if (now - lastFrameTime >= duration) {
    lastFrameTime = now;
    curFrame = pgm_read_byte(&frames[curFrame].next);
  }

  // 描画
  if (lastDrawnFrame < 0 || (uint8_t)lastDrawnFrame != curFrame || orient != currentOrient) {
    drawCurrentFrame(curFrame, orient);
  }

  drawUpIndicator();

#if DEBUG_LOOP_COUNTER
  static uint32_t dbgLastTime = 0;
  if (now - dbgLastTime >= 100) {
    dbgLastTime = now;
    dbgLoopCount++;
    drawDebugCounter();
  }
#endif
}
