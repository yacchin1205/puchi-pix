// USB device low-level glue for the STM32G0B1 USB_DRD_FS peripheral
// (PCD HAL <-> ST USB Device Library). The Arduino core's USBDevice library
// is not built (usb=none); this file plays its usbd_conf.c role.
#include "usbd_core.h"
#include "stm32yyxx_ll_pwr.h"
#include "PeripheralPins.h"
#include "pinconfig.h"

PCD_HandleTypeDef g_hpcd;

// Endpoint layout shared with puchi_usb.cpp: EP1 = MSC bulk, EP2 = CDC bulk,
// EP3 = CDC notification. Packet memory offsets leave 8 bytes per endpoint
// (DEV_NUM_EP) for the buffer descriptor table at the start of the PMA.
#define DEV_NUM_EP 4U
static const struct { uint16_t addr; uint16_t pma; } ep_def[] = {
  {0x00, 0x040}, {0x80, 0x080},
  {0x01, 0x0C0}, {0x81, 0x100},
  {0x02, 0x140}, {0x82, 0x180},
  {0x83, 0x1C0},
};

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  (void)hpcd;
  // Only DM/DP: the variant's PinMap_USB also lists PA4/PA13/PA15 as USB_NOE,
  // which would steal the NOR chip select and SWDIO.
  pin_function(PA_11, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, GPIO_AF_NONE));
  pin_function(PA_12, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, GPIO_AF_NONE));
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_NVIC_SetPriority(USB_IRQn, USBD_IRQ_PRIO, USBD_IRQ_SUBPRIO);
  HAL_NVIC_EnableIRQ(USB_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  (void)hpcd;
  __HAL_RCC_USB_CLK_DISABLE();
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF(hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Reset(hpcd->pData);
  USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Suspend(hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Resume(hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}

void USB_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&g_hpcd);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  g_hpcd.Instance = USB;
  g_hpcd.Init.dev_endpoints = DEV_NUM_EP;
  g_hpcd.Init.ep0_mps = EP_MPS_64;
  g_hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
  g_hpcd.Init.speed = PCD_SPEED_FULL;
  g_hpcd.Init.lpm_enable = DISABLE;
  g_hpcd.Init.battery_charging_enable = DISABLE;
  g_hpcd.Init.low_power_enable = DISABLE;
  g_hpcd.Init.Sof_enable = DISABLE;
  g_hpcd.pData = pdev;
  pdev->pData = &g_hpcd;

  if (HAL_PCD_Init(&g_hpcd) != HAL_OK) {
    Error_Handler();
  }
  for (uint32_t i = 0; i < sizeof(ep_def) / sizeof(ep_def[0]); i++) {
    HAL_PCDEx_PMAConfig(&g_hpcd, ep_def[i].addr, PCD_SNG_BUF, ep_def[i].pma);
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  if ((ep_addr & 0x80) == 0x80) {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
