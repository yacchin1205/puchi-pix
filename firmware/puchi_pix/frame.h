#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

typedef struct {
  uint8_t type;           // 0=full, 1=overlay
  uint8_t next;           // next frame index
  uint8_t ref;            // overlay: reference frame index (may itself be an overlay)
  uint8_t rx, ry, rw, rh; // overlay region
  int8_t refDy;           // overlay: vertical shift applied to ref content
                          // (pixel(x,y) outside region samples ref at y-refDy;
                          //  out-of-image rows render as palette index 0)
  uint16_t duration_ms;   // display duration (ms)
  const uint8_t* data;
} Frame;

// Overlay chain entry unpacked from PROGMEM for per-pixel sampling
typedef struct {
  const uint8_t* data;
  uint8_t rx, ry, rw, rh;
  int8_t refDy;
} OvlDesc;

#endif
