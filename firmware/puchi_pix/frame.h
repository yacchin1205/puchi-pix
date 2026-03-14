#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

typedef struct {
  uint8_t type;           // 0=full, 1=overlay
  uint8_t next;           // next frame index
  uint8_t ref;            // overlay: reference frame index
  uint8_t rx, ry, rw, rh; // overlay region
  uint16_t duration_ms;   // display duration (ms)
  const uint8_t* data;
} Frame;

#endif
