#!/usr/bin/env python3
"""
Generate image_data.h for blink animation.
Source images should be in the same directory as this script.
"""
from PIL import Image
import numpy as np
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(SCRIPT_DIR, '..', 'image_data.h')

# Source images
BASE_IMG = os.path.join(SCRIPT_DIR, 'IMG_1508.PNG')
HALF_IMG = os.path.join(SCRIPT_DIR, 'gemini2_64.png')    # half-closed eyes
CLOSED_IMG = os.path.join(SCRIPT_DIR, 'gemini1_64.png')  # fully closed eyes

# Eye region coordinates (excluding mouth)
EYE_X, EYE_Y = 17, 29
EYE_W, EYE_H = 29, 18


def rgb_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def quantize_to_palette(img_arr, max_colors=256):
    img = Image.fromarray(img_arr)
    quantized = img.quantize(colors=max_colors, method=Image.MEDIANCUT)
    palette_data = quantized.getpalette()[:max_colors * 3]

    palette_rgb565 = []
    for i in range(0, len(palette_data), 3):
        r, g, b = palette_data[i], palette_data[i + 1], palette_data[i + 2]
        palette_rgb565.append(rgb_to_rgb565(r, g, b))

    while len(palette_rgb565) < 256:
        palette_rgb565.append(0)

    indexed = np.array(quantized)
    return palette_rgb565, indexed


def write_palette(f, name, palette):
    f.write(f'static const uint16_t {name}[256] PROGMEM = {{\n')
    for i in range(0, 256, 8):
        line = ', '.join(f'0x{palette[j]:04X}' for j in range(i, min(i + 8, 256)))
        f.write(f'  {line},\n')
    f.seek(f.tell() - 2)
    f.write('\n};\n\n')


def write_data(f, name, data):
    flat = data.flatten()
    f.write(f'static const uint8_t {name}[{len(flat)}] PROGMEM = {{\n')
    for i in range(0, len(flat), 16):
        line = ', '.join(f'0x{flat[j]:02X}' for j in range(i, min(i + 16, len(flat))))
        f.write(f'  {line},\n')
    f.seek(f.tell() - 2)
    f.write('\n};\n\n')


def main():
    # Load images
    base_arr = np.array(Image.open(BASE_IMG).convert('RGB'))
    half_arr = np.array(Image.open(HALF_IMG).convert('RGB'))
    closed_arr = np.array(Image.open(CLOSED_IMG).convert('RGB'))

    # Quantize base frame
    base_palette, base_indexed = quantize_to_palette(base_arr)

    # Extract and quantize eye regions
    eye_half = half_arr[EYE_Y:EYE_Y + EYE_H, EYE_X:EYE_X + EYE_W]
    eye_closed = closed_arr[EYE_Y:EYE_Y + EYE_H, EYE_X:EYE_X + EYE_W]

    half_palette, half_indexed = quantize_to_palette(eye_half)
    closed_palette, closed_indexed = quantize_to_palette(eye_closed)

    # Generate header file
    with open(OUTPUT_FILE, 'w') as f:
        f.write('// Auto-generated image data for blink animation\n')
        f.write('// Run: python3 resources/generate_image_data.py\n\n')
        f.write(f'static const uint8_t IMG_W = {base_arr.shape[1]};\n')
        f.write(f'static const uint8_t IMG_H = {base_arr.shape[0]};\n')
        f.write(f'static const uint8_t EYE_X = {EYE_X};\n')
        f.write(f'static const uint8_t EYE_Y = {EYE_Y};\n')
        f.write(f'static const uint8_t EYE_W = {EYE_W};\n')
        f.write(f'static const uint8_t EYE_H = {EYE_H};\n\n')

        write_palette(f, 'basePalette', base_palette)
        write_data(f, 'baseFrame', base_indexed)
        write_palette(f, 'eyeHalfPalette', half_palette)
        write_data(f, 'eyeHalf', half_indexed)
        write_palette(f, 'eyeClosedPalette', closed_palette)
        write_data(f, 'eyeClosed', closed_indexed)

    # Summary
    total = 512 + base_indexed.size + 512 + half_indexed.size + 512 + closed_indexed.size
    print(f"Generated: {OUTPUT_FILE}")
    print(f"Base: {base_arr.shape[1]}x{base_arr.shape[0]} = {base_indexed.size} bytes")
    print(f"Eye region: {EYE_W}x{EYE_H} = {half_indexed.size} bytes each")
    print(f"Total: {total} bytes")


if __name__ == '__main__':
    main()
