#pragma once
// =====================
// Display abstraction layer
// =====================
//
// 各デバイスが提供する関数:
//   displayInit()                          - 初期化
//   displayBeginWrite(x,y,w,h)            - ピクセル書き込み開始
//   displayWritePixel(rgb565)             - 1ピクセル転送 (static inline)
//   displayEndWrite()                     - ピクセル書き込み終了
//   displayFillScreen(r,g,b)              - 全画面塗りつぶし
//   displayHLine(x,y,w,r,g,b)            - 水平ライン
//   displayBrightness(level)              - 輝度制御 (BRIGHT_FULL/DIM/OFF)
//   displayFade(out)                      - フェードアウト(true)/イン(false)
//   displayReset()                        - リセット (不要なデバイスではno-op)
//
// 各デバイスが提供する定数:
//   DISPLAY_W, DISPLAY_H                  - 画面サイズ

#include <Arduino.h>
#include <SPI.h>

// =====================
// ディスプレイ選択: いずれか1つをdefine (両方コメントアウトでST7735 TFT)
// =====================
// #define USE_SSD1351
#define USE_SSD1331

// 輝度レベル
enum DisplayBright : uint8_t {
  BRIGHT_OFF  = 0,
  BRIGHT_DIM  = 1,
  BRIGHT_FULL = 2,
};

// =====================
// デバイス固有ヘッダ読み込み
// =====================
#if defined(USE_SSD1351)
  #include "display_ssd1351.h"
#elif defined(USE_SSD1331)
  #include "display_ssd1331.h"
#else
  #include "display_st7735.h"
#endif
