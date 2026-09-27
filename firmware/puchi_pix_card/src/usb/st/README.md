# STM32 USB Device Library (vendored)

Unmodified copies from the STM32 USB Device Middleware V2.11.3
(`stm32_mw_usb_device`), taken from Arduino_Core_STM32 2.12.0
(`system/Middlewares/ST/STM32_USB_Device_Library/`). License: ST Ultimate
Liberty (SLA0044), see `LICENSE.md`.

| Files | Module |
|---|---|
| usbd_core.c, usbd_ctlreq.c, usbd_ioreq.c | Core (enumeration, standard requests) |
| usbd_composite_builder.c/.h | Class/CompositeBuilder (several classes in one device) |
| usbd_msc*.c/.h | Class/MSC (mass storage, SCSI/BOT) |
| usbd_cdc.c/.h | Class/CDC (virtual serial) |

They live in the sketch because the Arduino core only wires the library up
as a single CDC or HID device (`usb=CDCgen` / `usb=HID`) and its include
paths do not reach the MSC and composite sources. The sketch builds with
`usb=none`; `build_opt.h` sets `USBCON`, `USE_USBD_COMPOSITE` and the class
activation flags, and `../usbd_conf.c` / `../usbd_desc.c` supply the PCD
glue and descriptors that the core's `libraries/USBDevice` would otherwise
provide.

To update: copy the same files from a newer core and check
`usbd_composite_builder.h` is still not included from C++ (it uses `class`
as a parameter name; `puchi_usb.cpp` declares the one function it needs).
