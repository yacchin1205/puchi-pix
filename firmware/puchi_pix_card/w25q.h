// W25Q64JV (GD25Q64 互換) NOR Flash、SPI1: CS=PA4, SCK=PA5, MISO=PA6, MOSI=PA7。
// 4KB セクタ消去 (tSE 45〜400ms) と 256B ページプログラム (tPP 0.4〜3ms)。
#pragma once
#include <Arduino.h>
#include <SPI.h>

static constexpr uint32_t PIN_FLASH_CS = PA4;
static constexpr uint32_t NOR_PAGE_SIZE = 256;
static constexpr uint32_t NOR_SECTOR_SIZE = 4096;
static constexpr uint32_t NOR_SIZE = 8u * 1024 * 1024;

static const SPISettings flashSpi(8000000, MSBFIRST, SPI_MODE0);

static void flashPinsBegin() {
  SPI.setMOSI(PA_7);
  SPI.setMISO(PA_6);
  SPI.setSCLK(PA_5);
  SPI.begin();
  pinMode(PIN_FLASH_CS, OUTPUT);
  digitalWrite(PIN_FLASH_CS, HIGH);
}

// スリープ前に Hi-Z へ。NOR は +3V3P (昇圧断で消える) に載っているため、
// 出力を残すと保護ダイオード経由で未給電の NOR に逆給電する
static void flashPinsEnd() {
  SPI.end();
  pinMode(PIN_FLASH_CS, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG); pinMode(PA6, INPUT_ANALOG); pinMode(PA7, INPUT_ANALOG);
}

static void flashCmdAddr(uint8_t cmd, uint32_t addr) {
  SPI.transfer(cmd);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
}

static uint32_t flashReadJedecId() {
  SPI.beginTransaction(flashSpi);
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x9F);
  uint32_t id = (uint32_t)SPI.transfer(0) << 16;
  id |= (uint32_t)SPI.transfer(0) << 8;
  id |= SPI.transfer(0);
  digitalWrite(PIN_FLASH_CS, HIGH);
  SPI.endTransaction();
  return id;   // W25Q64JV: 0xEF4017, GD25Q64: 0xC84017
}

static void flashRead(uint32_t addr, uint8_t* buf, uint32_t len) {
  SPI.beginTransaction(flashSpi);
  digitalWrite(PIN_FLASH_CS, LOW);
  flashCmdAddr(0x03, addr);
  for (uint32_t i = 0; i < len; i++) buf[i] = SPI.transfer(0);
  digitalWrite(PIN_FLASH_CS, HIGH);
  SPI.endTransaction();
}

static void flashWaitBusy() {
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x05);
  while (SPI.transfer(0) & 0x01) {}
  digitalWrite(PIN_FLASH_CS, HIGH);
}

static void flashWriteEnable() {
  digitalWrite(PIN_FLASH_CS, LOW);
  SPI.transfer(0x06);
  digitalWrite(PIN_FLASH_CS, HIGH);
}

static void flashEraseSector(uint32_t addr) {
  SPI.beginTransaction(flashSpi);
  flashWriteEnable();
  digitalWrite(PIN_FLASH_CS, LOW);
  flashCmdAddr(0x20, addr);
  digitalWrite(PIN_FLASH_CS, HIGH);
  flashWaitBusy();
  SPI.endTransaction();
}

// addr から len バイト (ページ境界をまたがないこと) をプログラムする
static void flashProgramPage(uint32_t addr, const uint8_t* data, uint32_t len) {
  SPI.beginTransaction(flashSpi);
  flashWriteEnable();
  digitalWrite(PIN_FLASH_CS, LOW);
  flashCmdAddr(0x02, addr);
  for (uint32_t i = 0; i < len; i++) SPI.transfer(data[i]);
  digitalWrite(PIN_FLASH_CS, HIGH);
  flashWaitBusy();
  SPI.endTransaction();
}
