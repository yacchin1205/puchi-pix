// MSC storage backend: a synthesized FAT16 volume ("ghost FAT"). Reads return
// a fixed boot sector / FAT / root directory / INFO_UF2.TXT; writes are
// scanned for UF2 blocks and everything else (host metadata files) is
// discarded. No real file system lives on the NOR, so host-side caching and
// write ordering cannot corrupt anything.
#include "puchi_usb.h"
#include "st/usbd_msc.h"

#define UF2_MAGIC_START0    0x0A324655u
#define UF2_MAGIC_START1    0x9E5D5157u
#define UF2_MAGIC_END       0x0AB16F30u
#define UF2_FLAG_FAMILY_ID  0x00002000u
#define UF2_FAMILY_PUCHIPIX 0x50784361u

#define SECTOR_SIZE     512u
#define TOTAL_SECTORS   16384u          // 8 MiB volume
#define FAT_SECTORS     64u             // 16384 FAT16 entries * 2 B
#define ROOT_SECTORS    16u             // 256 entries * 32 B
#define LBA_FAT         1u
#define LBA_ROOT        (LBA_FAT + FAT_SECTORS)
#define LBA_DATA        (LBA_ROOT + ROOT_SECTORS)

static const char infoText[] =
  "UF2 Bootloader v0.1\r\n"
  "Model: Puchi-Pix card\r\n"
  "Board-ID: STM32G0B1-PUCHIPIX\r\n";

static const uint8_t bootSector[] = {
  0xEB, 0x3C, 0x90, 'P', 'U', 'C', 'H', 'I', 'P', 'I', 'X',
  0x00, 0x02,             // bytes per sector 512
  0x01,                   // sectors per cluster
  0x01, 0x00,             // reserved sectors
  0x01,                   // FATs
  0x00, 0x01,             // root entries 256
  (uint8_t)(TOTAL_SECTORS & 0xFF), (uint8_t)(TOTAL_SECTORS >> 8),
  0xF8,                   // media
  FAT_SECTORS, 0x00,      // sectors per FAT
  0x01, 0x00,             // sectors per track
  0x01, 0x00,             // heads
  0x00, 0x00, 0x00, 0x00, // hidden sectors
  0x00, 0x00, 0x00, 0x00, // total sectors (32-bit, unused)
  0x80, 0x00, 0x29,       // drive number, reserved, boot signature
  0x50, 0x58, 0x43, 0x41, // volume id
  'P', 'U', 'C', 'H', 'I', 'P', 'I', 'X', ' ', ' ', ' ',
  'F', 'A', 'T', '1', '6', ' ', ' ', ' ',
};

static void dirEntry(uint8_t *e, const char *name83, uint8_t attr, uint16_t cluster, uint32_t size)
{
  memcpy(e, name83, 11);
  e[11] = attr;
  e[26] = cluster & 0xFF;
  e[27] = cluster >> 8;
  e[28] = size & 0xFF;
  e[29] = (size >> 8) & 0xFF;
  e[30] = (size >> 16) & 0xFF;
  e[31] = size >> 24;
}

static void readSector(uint32_t lba, uint8_t *buf)
{
  memset(buf, 0, SECTOR_SIZE);
  if (lba == 0) {
    memcpy(buf, bootSector, sizeof(bootSector));
    buf[510] = 0x55;
    buf[511] = 0xAA;
  } else if (lba == LBA_FAT) {
    buf[0] = 0xF8; buf[1] = 0xFF;   // media / reserved
    buf[2] = 0xFF; buf[3] = 0xFF;
    buf[4] = 0xFF; buf[5] = 0xFF;   // cluster 2: INFO_UF2.TXT, end of chain
  } else if (lba == LBA_ROOT) {
    dirEntry(buf, "PUCHIPIX   ", 0x08, 0, 0);
    dirEntry(buf + 32, "INFO_UF2TXT", 0x01, 2, sizeof(infoText) - 1);
  } else if (lba == LBA_DATA) {
    memcpy(buf, infoText, sizeof(infoText) - 1);
  }
}

static void writeSector(const uint8_t *buf)
{
  const uint32_t *w = (const uint32_t *)buf;
  if (w[0] != UF2_MAGIC_START0 || w[1] != UF2_MAGIC_START1 || w[127] != UF2_MAGIC_END) return;
  if (!(w[2] & UF2_FLAG_FAMILY_ID) || w[7] != UF2_FAMILY_PUCHIPIX) return;
  Uf2Block block = {w[3], w[4], w[5], w[6], buf + 32};
  if (block.payloadSize > 476) return;
  puchiUsbOnUf2Block(block);
}

static int8_t storageInit(uint8_t lun)
{
  (void)lun;
  return 0;
}

static int8_t storageGetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  (void)lun;
  *block_num = TOTAL_SECTORS;
  *block_size = SECTOR_SIZE;
  return 0;
}

static int8_t storageIsReady(uint8_t lun)
{
  (void)lun;
  return 0;
}

static int8_t storageIsWriteProtected(uint8_t lun)
{
  (void)lun;
  return 0;
}

static int8_t storageRead(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  (void)lun;
  for (uint16_t i = 0; i < blk_len; i++) {
    readSector(blk_addr + i, buf + i * SECTOR_SIZE);
  }
  return 0;
}

static int8_t storageWrite(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  (void)lun; (void)blk_addr;
  for (uint16_t i = 0; i < blk_len; i++) {
    writeSector(buf + i * SECTOR_SIZE);
  }
  return 0;
}

static int8_t storageGetMaxLun()
{
  return 0;
}

static int8_t inquiry[] = {
  0x00, (int8_t)0x80, 0x02, 0x02, 36 - 5, 0x00, 0x00, 0x00,
  'P', 'u', 'c', 'h', 'i', 'P', 'i', 'x',                                          // vendor (8)
  'c', 'a', 'r', 'd', ' ', 'U', 'F', '2', ' ', 'd', 'r', 'i', 'v', 'e', ' ', ' ',  // product (16)
  '0', '.', '1', ' ',                                                              // revision (4)
};

USBD_StorageTypeDef uf2DriveStorage = {
  storageInit, storageGetCapacity, storageIsReady, storageIsWriteProtected,
  storageRead, storageWrite, storageGetMaxLun, inquiry,
};
