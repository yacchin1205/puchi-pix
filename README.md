# Puchi-Pix (ぷちぴく)

A tiny pixel art display kit powered by STM32G030F6P, with a 96x64 OLED (SSD1331) or TFT display.

## STM32G030 BOOT0 pin issue

The factory default option bytes have `nBOOT_SEL=1`, which ignores the physical BOOT0 pin. A fresh (empty flash) chip boots into the bootloader automatically, but once firmware is written, the BOOT button has no effect.

Before the first firmware upload, set `nBOOT_SEL=0` via STM32CubeProgrammer CLI:

```bash
STM32_Programmer_CLI -c port=/dev/cu.usbserial-XXXXX br=115200 -ob nBOOT_SEL=0
```

Verify with:

```bash
STM32_Programmer_CLI -c port=/dev/cu.usbserial-XXXXX br=115200 -ob displ
```

`nBOOT_SEL : 0x0 (BOOT0 signal is defined by BOOT0 pin value (legacy mode))` means BOOT+RST will work.
