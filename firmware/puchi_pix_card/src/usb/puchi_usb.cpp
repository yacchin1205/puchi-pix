#include "puchi_usb.h"
#include "usbd_core.h"
// usbd_composite_builder.h is not C++-clean (a parameter named "class").
extern "C" uint32_t USBD_CMPSIT_SetClassID(USBD_HandleTypeDef *pdev, USBD_CompositeClassTypeDef Class, uint32_t Instance);
#include "st/usbd_msc.h"
#include "st/usbd_cdc.h"

extern USBD_DescriptorsTypeDef puchiUsbDescriptors;
extern USBD_StorageTypeDef uf2DriveStorage;

static USBD_HandleTypeDef usbDev;
static uint8_t mscEps[] = {0x81, 0x01};
static uint8_t cdcEps[] = {0x82, 0x02, 0x83};
static uint32_t cdcClassId;

// ---- CDC interface ----
static USBD_CDC_LineCodingTypeDef lineCoding = {115200, 0, 0, 8};
static uint8_t cdcRxBuf[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t cdcTxBuf[256];
static volatile bool cdcDtr = false;
// Output written before a host opens the port (boot log) is kept here and
// sent first once DTR is asserted.
static uint8_t backlog[1024];
static size_t backlogLen = 0;
static volatile bool dfuRequested = false;

bool puchiUsbDfuRequested()
{
  return dfuRequested;
}

static int8_t cdcInit()
{
  USBD_CDC_SetRxBuffer(&usbDev, cdcRxBuf);
  return USBD_OK;
}

static int8_t cdcDeInit()
{
  cdcDtr = false;
  return USBD_OK;
}

static int8_t cdcControl(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  (void)length;
  switch (cmd) {
    case CDC_SET_LINE_CODING:
      memcpy(&lineCoding, pbuf, sizeof(lineCoding));
      if (lineCoding.bitrate == 1200) dfuRequested = true;
      break;
    case CDC_GET_LINE_CODING:
      memcpy(pbuf, &lineCoding, sizeof(lineCoding));
      break;
    case CDC_SET_CONTROL_LINE_STATE:
      cdcDtr = ((USBD_SetupReqTypedef *)pbuf)->wValue & 0x01;
      break;
    default:
      break;
  }
  return USBD_OK;
}

static int8_t cdcReceive(uint8_t *buf, uint32_t *len)
{
  (void)buf; (void)len;
  USBD_CDC_SetRxBuffer(&usbDev, cdcRxBuf);
  USBD_CDC_ReceivePacket(&usbDev);
  return USBD_OK;
}

static int8_t cdcTransmitCplt(uint8_t *buf, uint32_t *len, uint8_t epnum)
{
  (void)buf; (void)len; (void)epnum;
  return USBD_OK;
}

static USBD_CDC_ItfTypeDef cdcItf = {cdcInit, cdcDeInit, cdcControl, cdcReceive, cdcTransmitCplt};

static bool cdcTxBusy()
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)usbDev.pClassDataCmsit[cdcClassId];
  return hcdc == NULL || hcdc->TxState != 0;
}

static void cdcSend(const uint8_t *buf, size_t len)
{
  size_t sent = 0;
  while (sent < len) {
    uint32_t t0 = millis();
    while (cdcTxBusy()) {
      if (millis() - t0 > 20) return;   // host stalled: drop the rest
    }
    size_t n = min(len - sent, sizeof(cdcTxBuf));
    memcpy(cdcTxBuf, buf + sent, n);
    USBD_CDC_SetTxBuffer(&usbDev, cdcTxBuf, n, cdcClassId);
    USBD_CDC_TransmitPacket(&usbDev, cdcClassId);
    sent += n;
  }
}

size_t PuchiUsbSerial::write(const uint8_t *buf, size_t len)
{
  if (usbDev.dev_state != USBD_STATE_CONFIGURED || !cdcDtr) {
    if (len >= sizeof(backlog)) {
      memcpy(backlog, buf + len - sizeof(backlog), sizeof(backlog));
      backlogLen = sizeof(backlog);
    } else {
      if (backlogLen + len > sizeof(backlog)) {
        size_t drop = backlogLen + len - sizeof(backlog);
        memmove(backlog, backlog + drop, backlogLen - drop);
        backlogLen -= drop;
      }
      memcpy(backlog + backlogLen, buf, len);
      backlogLen += len;
    }
    return len;
  }
  if (backlogLen) {
    size_t n = backlogLen;
    backlogLen = 0;
    cdcSend(backlog, n);
  }
  cdcSend(buf, len);
  return len;
}

void PuchiUsbSerial::flush()
{
  uint32_t t0 = millis();
  while (cdcDtr && cdcTxBusy() && millis() - t0 < 20) {}
}

PuchiUsbSerial Serial;

// ---- device ----
void puchiUsbBegin()
{
  USBD_Init(&usbDev, &puchiUsbDescriptors, 0);
  USBD_RegisterClassComposite(&usbDev, USBD_MSC_CLASS, CLASS_TYPE_MSC, mscEps);
  USBD_RegisterClassComposite(&usbDev, USBD_CDC_CLASS, CLASS_TYPE_CDC, cdcEps);
  USBD_CMPSIT_SetClassID(&usbDev, CLASS_TYPE_MSC, 0);
  USBD_MSC_RegisterStorage(&usbDev, &uf2DriveStorage);
  cdcClassId = USBD_CMPSIT_SetClassID(&usbDev, CLASS_TYPE_CDC, 0);
  USBD_CDC_RegisterInterface(&usbDev, &cdcItf);
  USBD_Start(&usbDev);
}
