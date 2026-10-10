// USB device descriptor and strings for the CDC+MSC composite device.
#include "usbd_core.h"

#define USBD_VID            0x0483
#define USBD_PID            0x5740
#define USBD_LANGID_STRING  0x0409

__ALIGN_BEGIN static uint8_t deviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  USB_LEN_DEV_DESC, USB_DESC_TYPE_DEVICE,
  0x00, 0x02,                 // bcdUSB 2.00
  0xEF, 0x02, 0x01,           // Miscellaneous / Common / IAD
  USB_MAX_EP0_SIZE,
  LOBYTE(USBD_VID), HIBYTE(USBD_VID),
  LOBYTE(USBD_PID), HIBYTE(USBD_PID),
  0x00, 0x01,                 // bcdDevice 1.00
  USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION,
};

__ALIGN_BEGIN static uint8_t langIdDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING),
};

__ALIGN_BEGIN static uint8_t strDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *getDeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  *length = sizeof(deviceDesc);
  return deviceDesc;
}

static uint8_t *getLangIdStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  *length = sizeof(langIdDesc);
  return langIdDesc;
}

static uint8_t *getManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  USBD_GetString((uint8_t *)"Puchi-Pix", strDesc, length);
  return strDesc;
}

static uint8_t *getProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  USBD_GetString((uint8_t *)"Puchi-Pix card", strDesc, length);
  return strDesc;
}

static uint8_t *getSerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  char serial[25];
  snprintf(serial, sizeof(serial), "%08lX%08lX%08lX",
           (unsigned long)HAL_GetUIDw0(), (unsigned long)HAL_GetUIDw1(), (unsigned long)HAL_GetUIDw2());
  USBD_GetString((uint8_t *)serial, strDesc, length);
  return strDesc;
}

static uint8_t *getConfigurationStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  USBD_GetString((uint8_t *)"Puchi-Pix card", strDesc, length);
  return strDesc;
}

static uint8_t *getInterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  (void)speed;
  USBD_GetString((uint8_t *)"Puchi-Pix card", strDesc, length);
  return strDesc;
}

USBD_DescriptorsTypeDef puchiUsbDescriptors = {
  getDeviceDescriptor,
  getLangIdStrDescriptor,
  getManufacturerStrDescriptor,
  getProductStrDescriptor,
  getSerialStrDescriptor,
  getConfigurationStrDescriptor,
  getInterfaceStrDescriptor,
};
