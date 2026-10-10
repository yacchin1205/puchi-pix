// アニメーションペイロード (ESP32 版 v3〜v6 と同一形式、正方形サイズは 16 のみ) の
// 検証・展開。フォーマットは firmware/puchi_pix_esp32/puchi_pix_esp32.ino 冒頭の
// "Payload" を参照。v6 のモーションブロックは検証のみで、再生は単純ループ。
#pragma once
#include <Arduino.h>

static constexpr uint32_t PAYLOAD_MAX = 32768;
static constexpr uint8_t PAYLOAD_SIZE = 16;      // 画像の一辺

static uint8_t payloadBuf[PAYLOAD_MAX];
static uint32_t payloadLen = 0;
static uint8_t payVer = 0;
static uint8_t payBpp = 4;
static uint32_t payPalOff = 0;
static uint32_t payDataOff = 0;
static uint16_t payFrames = 0;

static uint16_t payCurFrame = 0;
static uint32_t payCurRec = 0;
static uint16_t payFrame565[PAYLOAD_SIZE * PAYLOAD_SIZE];

struct RleReader {
  const uint8_t* p;
  uint8_t runByte;
  uint8_t runLeft;
  uint8_t litLeft;
};

static inline uint16_t payU16(uint32_t off) {
  return (uint16_t)payloadBuf[off] | ((uint16_t)payloadBuf[off + 1] << 8);
}

static inline const uint16_t* payPalette() {
  return (const uint16_t*)&payloadBuf[payPalOff];
}

static inline uint32_t payFrameBytes() {
  return (uint32_t)PAYLOAD_SIZE * PAYLOAD_SIZE * payBpp / 8;
}

static inline void rleInit(RleReader& r, const uint8_t* p) {
  r.p = p;
  r.runLeft = 0;
  r.litLeft = 0;
}

static uint8_t rleNextByte(RleReader& r) {
  if (r.runLeft) { r.runLeft--; return r.runByte; }
  if (r.litLeft) { r.litLeft--; return *r.p++; }
  for (;;) {
    uint8_t c = *r.p++;
    if (c < 128) { r.litLeft = c; return *r.p++; }
    if (c > 128) {
      r.runByte = *r.p++;
      r.runLeft = (uint8_t)(256 - c);   // total 257-c, one emitted now
      return r.runByte;
    }
  }
}

static bool rleCheck(const uint8_t* p, uint32_t len, uint32_t expect) {
  const uint8_t* end = p + len;
  uint32_t out = 0;
  while (p < end) {
    uint8_t c = *p++;
    if (c < 128) {
      uint32_t n = (uint32_t)c + 1;
      if (p + n > end) return false;
      p += n;
      out += n;
    } else if (c > 128) {
      if (p >= end) return false;
      p++;
      out += 257 - c;
    }
  }
  return out == expect;
}

static inline uint8_t read4bit(const uint8_t* data, uint32_t idx) {
  uint8_t b = data[idx >> 1];
  return (idx & 1) ? (b & 0x0F) : (b >> 4);
}

static uint32_t chainNextRecord(uint32_t off) {
  if (payVer >= 5 && (payloadBuf[off + 2] & 0x02)) {
    uint8_t rectCount = payloadBuf[off + 5];
    uint32_t p = off + 6;
    for (uint8_t r = 0; r < rectCount; r++) p += 6 + payU16(p + 4);
    return p;
  }
  if (payVer >= 4) {
    if (payloadBuf[off + 2] & 0x01) return off + 5 + payU16(off + 3);
    uint8_t rectCount = payloadBuf[off + 3];
    uint32_t p = off + 4;
    for (uint8_t r = 0; r < rectCount; r++) p += 6 + payU16(p + 4);
    return p;
  }
  if (payloadBuf[off + 2] & 0x01) return off + 3 + payFrameBytes();
  uint8_t rectCount = payloadBuf[off + 3];
  uint32_t p = off + 4;
  for (uint8_t r = 0; r < rectCount; r++) {
    uint32_t px = (uint32_t)payloadBuf[p + 2] * payloadBuf[p + 3];
    p += 4 + ((payBpp == 8) ? px : ((px + 1) >> 1));
  }
  return p;
}

static void applyRects(uint32_t p, uint8_t rectCount, bool rle, uint16_t* buf) {
  const uint16_t* pal = payPalette();
  for (uint8_t rc = 0; rc < rectCount; rc++) {
    uint8_t rx = payloadBuf[p], ry = payloadBuf[p + 1];
    uint8_t rw = payloadBuf[p + 2], rh = payloadBuf[p + 3];
    const uint8_t* data = &payloadBuf[p + (rle ? 6 : 4)];
    RleReader r;
    if (rle) rleInit(r, data);
    uint32_t i = 0;
    uint8_t b = 0;
    for (uint8_t dy = 0; dy < rh; dy++) {
      uint16_t* row = &buf[(uint32_t)(ry + dy) * PAYLOAD_SIZE + rx];
      for (uint8_t dx = 0; dx < rw; dx++, i++) {
        uint8_t v;
        if (rle) {
          if (payBpp == 8) v = rleNextByte(r);
          else {
            if (!(i & 1)) b = rleNextByte(r);
            v = (i & 1) ? (b & 0x0F) : (b >> 4);
          }
        } else {
          v = (payBpp == 8) ? data[i] : read4bit(data, i);
        }
        row[dx] = pal[v];
      }
    }
    uint32_t px = (uint32_t)rw * rh;
    p += rle ? (6 + payU16(p + 4)) : (4 + ((payBpp == 8) ? px : ((px + 1) >> 1)));
  }
}

static void composeChainInto(uint16_t target, uint16_t* buf);

// キーフレームは全画素を、差分は buf に残っている前フレームへ矩形を当てる。
// v5 の参照差分は参照先 (参照を含まないことが検証済み) を合成してから当てる
static void chainDecode(uint32_t off, uint16_t* buf) {
  const bool rle = (payVer >= 4);
  if (payVer >= 5 && (payloadBuf[off + 2] & 0x02)) {
    composeChainInto(payU16(off + 3), buf);
    applyRects(off + 6, payloadBuf[off + 5], true, buf);
    return;
  }
  if (payloadBuf[off + 2] & 0x01) {
    const uint16_t* pal = payPalette();
    uint32_t n = (uint32_t)PAYLOAD_SIZE * PAYLOAD_SIZE;
    if (rle) {
      RleReader r;
      rleInit(r, &payloadBuf[off + 5]);
      uint8_t b = 0;
      for (uint32_t i = 0; i < n; i++) {
        if (payBpp == 8) buf[i] = pal[rleNextByte(r)];
        else {
          if (!(i & 1)) b = rleNextByte(r);
          buf[i] = pal[(i & 1) ? (b & 0x0F) : (b >> 4)];
        }
      }
    } else {
      const uint8_t* data = &payloadBuf[off + 3];
      for (uint32_t i = 0; i < n; i++) buf[i] = pal[(payBpp == 8) ? data[i] : read4bit(data, i)];
    }
    return;
  }
  applyRects(off + 4, payloadBuf[off + 3], rle, buf);
}

static void composeChainInto(uint16_t target, uint16_t* buf) {
  uint32_t off = payDataOff;
  uint32_t keyOff = payDataOff;
  uint16_t keyIdx = 0;
  for (uint16_t i = 1; i <= target; i++) {
    off = chainNextRecord(off);
    if (payloadBuf[off + 2] & 0x01) { keyOff = off; keyIdx = i; }
  }
  uint32_t p = keyOff;
  chainDecode(p, buf);
  for (uint16_t i = keyIdx; i < target; i++) {
    p = chainNextRecord(p);
    chainDecode(p, buf);
  }
}

// ESP32 版 validateUpload の v3〜v6 部分。レコード鎖を全て歩き、矩形の範囲、
// RLE の整合、参照先の合成可能性を確認する。通れば再生側は境界チェックを持たない
static bool payloadValidate(uint32_t len) {
  if (len < 8 || payloadBuf[0] != 'P' || payloadBuf[1] != 'P') return false;
  uint8_t ver = payloadBuf[2];
  if (ver < 3 || ver > 6) return false;
  uint16_t n = payU16(3);
  if (n == 0 || n > 2048) return false;
  if (payloadBuf[5] != PAYLOAD_SIZE) return false;
  uint8_t bpp = payloadBuf[6];
  if (bpp != 4 && bpp != 8) return false;
  uint32_t palOff = (ver >= 6) ? 24 : 8;
  if (ver >= 6) {
    const uint8_t* m = &payloadBuf[8];
    if (m[0]) {
      uint16_t mainStart = m[1] | (m[2] << 8), loopStart = m[3] | (m[4] << 8);
      uint16_t shakeStart = m[5] | (m[6] << 8), outStart = m[7] | (m[8] << 8);
      uint16_t exitStart = m[9] | (m[10] << 8);
      uint8_t speed = m[11];
      if (speed == 0 && (mainStart != 0 || exitStart != 0)) return false;
      if (mainStart >= n) return false;
      if (mainStart == 0 && loopStart != 0) return false;
      if (loopStart < mainStart || loopStart >= n) return false;
      uint16_t exitB = exitStart ? exitStart : n;
      uint16_t outB = outStart ? outStart : exitB;
      if (exitStart && (exitStart <= loopStart || exitStart >= n)) return false;
      if (outStart && (outStart <= loopStart || outStart >= exitB)) return false;
      if (shakeStart && (shakeStart <= loopStart || shakeStart >= outB)) return false;
    }
  }
  uint32_t dataOff = palOff + ((bpp == 8) ? 512 : 32);
  uint32_t frameBytes = (uint32_t)PAYLOAD_SIZE * PAYLOAD_SIZE * bpp / 8;

  static uint8_t refFreeBits[256];
  memset(refFreeBits, 0, sizeof(refFreeBits));
  bool windowHasRef = false;
  uint32_t off = dataOff;
  for (uint16_t i = 0; i < n; i++) {
    if (off + 3 > len) return false;
    uint8_t rflags = payloadBuf[off + 2];
    if ((ver >= 5) && (rflags & 0x02) && !(rflags & 0x01)) {
      if (i == 0) return false;
      if (off + 6 > len) return false;
      uint16_t ref = payU16(off + 3);
      if (ref >= i) return false;
      if (!(refFreeBits[ref >> 3] & (1 << (ref & 7)))) return false;
      uint8_t rectCount = payloadBuf[off + 5];
      off += 6;
      for (uint8_t r = 0; r < rectCount; r++) {
        if (off + 6 > len) return false;
        uint8_t rw = payloadBuf[off + 2], rh = payloadBuf[off + 3];
        if (rw == 0 || rh == 0) return false;
        if ((uint16_t)payloadBuf[off] + rw > PAYLOAD_SIZE) return false;
        if ((uint16_t)payloadBuf[off + 1] + rh > PAYLOAD_SIZE) return false;
        uint32_t px = (uint32_t)rw * rh;
        uint32_t raw = (bpp == 8) ? px : ((px + 1) >> 1);
        uint32_t rl = payU16(off + 4);
        if (off + 6 + rl > len) return false;
        if (!rleCheck(&payloadBuf[off + 6], rl, raw)) return false;
        off += 6 + rl;
      }
      windowHasRef = true;
      if (off > len) return false;
      continue;
    }
    if (rflags & 0x01) {
      if (ver >= 4) {
        if (off + 5 > len) return false;
        uint32_t rl = payU16(off + 3);
        if (off + 5 + rl > len) return false;
        if (!rleCheck(&payloadBuf[off + 5], rl, frameBytes)) return false;
        off += 5 + rl;
      } else {
        off += 3 + frameBytes;
      }
      windowHasRef = false;
      refFreeBits[i >> 3] |= (uint8_t)(1 << (i & 7));
    } else {
      if (i == 0) return false;
      if (off + 4 > len) return false;
      uint8_t rectCount = payloadBuf[off + 3];
      off += 4;
      for (uint8_t r = 0; r < rectCount; r++) {
        uint32_t rectHdr = (ver >= 4) ? 6 : 4;
        if (off + rectHdr > len) return false;
        uint8_t rw = payloadBuf[off + 2], rh = payloadBuf[off + 3];
        if (rw == 0 || rh == 0) return false;
        if ((uint16_t)payloadBuf[off] + rw > PAYLOAD_SIZE) return false;
        if ((uint16_t)payloadBuf[off + 1] + rh > PAYLOAD_SIZE) return false;
        uint32_t px = (uint32_t)rw * rh;
        uint32_t raw = (bpp == 8) ? px : ((px + 1) >> 1);
        if (ver >= 4) {
          uint32_t rl = payU16(off + 4);
          if (off + 6 + rl > len) return false;
          if (!rleCheck(&payloadBuf[off + 6], rl, raw)) return false;
          off += 6 + rl;
        } else {
          off += 4 + raw;
        }
      }
      if (!windowHasRef) refFreeBits[i >> 3] |= (uint8_t)(1 << (i & 7));
    }
    if (off > len) return false;
  }
  if (off != len) return false;

  payloadLen = len;
  payVer = ver;
  payBpp = bpp;
  payPalOff = palOff;
  payDataOff = dataOff;
  payFrames = n;
  return true;
}

static void payloadJumpTo(uint16_t target) {
  composeChainInto(target, payFrame565);
  uint32_t off = payDataOff;
  for (uint16_t i = 0; i < target; i++) off = chainNextRecord(off);
  payCurRec = off;
  payCurFrame = target;
}

static void payloadAdvance() {
  if (payCurFrame + 1 >= payFrames) {
    payloadJumpTo(0);
    return;
  }
  payCurFrame++;
  payCurRec = chainNextRecord(payCurRec);
  chainDecode(payCurRec, payFrame565);
}

static inline uint16_t payloadDuration() {
  return payU16(payCurRec);
}
