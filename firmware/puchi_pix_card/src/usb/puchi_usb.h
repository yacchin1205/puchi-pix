#pragma once
#include <Arduino.h>

// USB composite device: MSC (UF2 drop drive over the NOR flash) + CDC (log).
void puchiUsbBegin();

// True once a host opened the CDC port at 1200 bps (Arduino-style request to
// reboot into the ROM DFU bootloader).
bool puchiUsbDfuRequested();

// CDC serial. Output is dropped while no host has opened the port.
class PuchiUsbSerial : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *buf, size_t len) override;
  void flush();
};
extern PuchiUsbSerial Serial;

// UF2 block handed over by the drive (payload already validated by magic/family).
struct Uf2Block {
  uint32_t targetAddr;
  uint32_t payloadSize;
  uint32_t blockNo;
  uint32_t numBlocks;
  const uint8_t *data;
};
// Called from the USB interrupt for every UF2 block written to the drive.
void puchiUsbOnUf2Block(const Uf2Block &block);
