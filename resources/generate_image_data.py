#!/usr/bin/env python3
"""
Generate image_data.h for entrance + blink animation from a GIF.

GIF frame layout:
  0: entrance frame 0 (ears peeking)
  1: entrance frame 1 (half head)
  2: base frame (full character, eyes open)
  3: half-closed eyes
  4: closed eyes
  5: half-closed eyes (same as 3, ignored)

All frames share a single 16-color palette.
Base frame and eye regions use 4-bit packed pixel data (2 pixels/byte).
Entrance frames use RLE compression with 4-bit palette indices.

Run: python3 resources/generate_image_data.py [gif_path]
"""
from PIL import Image
import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(SCRIPT_DIR, '..', 'image_data.h')

# Default GIF path (can be overridden via command line)
DEFAULT_GIF = os.path.join(SCRIPT_DIR, '..', 'animation.gif')

# Eye region coordinates
EYE_X, EYE_Y = 15, 43
EYE_W, EYE_H = 33, 14

# Entrance animation
ENTRANCE_FRAMES = 2
ENTRANCE_INTERVAL_MS = 200

PALETTE_SIZE = 16


def rgb_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def quantize_to_palette(img_arr, max_colors=PALETTE_SIZE):
    img = Image.fromarray(img_arr)
    quantized = img.quantize(colors=max_colors, method=Image.MEDIANCUT)
    palette_data = quantized.getpalette()[:max_colors * 3]

    palette_rgb565 = []
    for i in range(0, len(palette_data), 3):
        r, g, b = palette_data[i], palette_data[i + 1], palette_data[i + 2]
        palette_rgb565.append(rgb_to_rgb565(r, g, b))

    indexed = np.array(quantized)
    return palette_rgb565, indexed


def pack_4bit(data):
    """Pack palette indices (0-15) into 4-bit pairs, 2 pixels per byte."""
    flat = data.flatten().tolist()
    packed = []
    for i in range(0, len(flat), 2):
        hi = flat[i] & 0x0F
        lo = flat[i + 1] & 0x0F if i + 1 < len(flat) else 0
        packed.append((hi << 4) | lo)
    return packed


def rle_encode(data):
    """RLE encode: pairs of (count, value). count max 255, value 0-15."""
    flat = data.flatten().tolist()
    encoded = []
    i = 0
    while i < len(flat):
        val = flat[i]
        count = 1
        while i + count < len(flat) and flat[i + count] == val and count < 255:
            count += 1
        encoded.append(count)
        encoded.append(val)
        i += count
    return encoded


def write_c_array(f, type_str, name, data, per_line=16):
    """Write a generic C array."""
    f.write(f'static const {type_str} {name}[{len(data)}] PROGMEM = {{\n')
    fmt = '0x{:04X}' if '16' in type_str else '0x{:02X}'
    for i in range(0, len(data), per_line):
        line = ', '.join(fmt.format(data[j]) for j in range(i, min(i + per_line, len(data))))
        f.write(f'  {line},\n')
    f.seek(f.tell() - 2)
    f.write('\n};\n\n')


def extract_gif_frames(gif_path):
    """Extract all frames from a GIF as RGB numpy arrays."""
    gif = Image.open(gif_path)
    frames = []
    i = 0
    while True:
        try:
            gif.seek(i)
            frames.append(np.array(gif.convert('RGB')))
            i += 1
        except EOFError:
            break
    return frames


def main():
    gif_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GIF

    if not os.path.exists(gif_path):
        print(f"Error: GIF not found: {gif_path}")
        sys.exit(1)

    print(f"Reading GIF: {gif_path}")
    frames = extract_gif_frames(gif_path)
    print(f"Extracted {len(frames)} frames")

    if len(frames) < 5:
        print(f"Error: Expected at least 5 frames, got {len(frames)}")
        sys.exit(1)

    # Replace background color with black
    # Detect from base frame (frame 2) - most common color
    flat = frames[2].reshape(-1, 3)
    unique, counts = np.unique(flat, axis=0, return_counts=True)
    bg_color = unique[np.argmax(counts)]
    total_replaced = 0
    for i in range(len(frames)):
        mask = np.all(frames[i] == bg_color, axis=2)
        frames[i][mask] = [0, 0, 0]
        total_replaced += mask.sum()
    print(f"Background {bg_color} -> black ({total_replaced} px total)")

    # Frame assignments
    entrance0_arr = frames[0]
    entrance1_arr = frames[1]
    base_arr = frames[2]
    half_arr = frames[3]
    closed_arr = frames[4]

    img_w, img_h = base_arr.shape[1], base_arr.shape[0]
    print(f"Image size: {img_w}x{img_h}")

    # Extract eye regions
    eye_half = half_arr[EYE_Y:EYE_Y + EYE_H, EYE_X:EYE_X + EYE_W]
    eye_closed = closed_arr[EYE_Y:EYE_Y + EYE_H, EYE_X:EYE_X + EYE_W]

    # Pad eye regions to img_w width for vstack, then quantize all together
    def pad_to_width(arr, w):
        if arr.shape[1] == w:
            return arr
        padded = np.zeros((arr.shape[0], w, 3), dtype=arr.dtype)
        padded[:, :arr.shape[1]] = arr
        return padded

    combined = np.vstack([entrance0_arr, entrance1_arr, base_arr,
                          pad_to_width(eye_half, img_w),
                          pad_to_width(eye_closed, img_w)])
    palette, combined_indexed = quantize_to_palette(combined, PALETTE_SIZE)

    y = 0
    ent0_indexed = combined_indexed[y:y + img_h]; y += img_h
    ent1_indexed = combined_indexed[y:y + img_h]; y += img_h
    base_indexed = combined_indexed[y:y + img_h]; y += img_h
    half_indexed = combined_indexed[y:y + EYE_H, :EYE_W]; y += EYE_H
    closed_indexed = combined_indexed[y:y + EYE_H, :EYE_W]

    print(f"Palette: {len(palette)} colors")

    # Encode data (all 4-bit packed for random access / rotation support)
    ent0_packed = pack_4bit(ent0_indexed)
    ent1_packed = pack_4bit(ent1_indexed)
    base_packed = pack_4bit(base_indexed)
    half_packed = pack_4bit(half_indexed)
    closed_packed = pack_4bit(closed_indexed)

    # Generate header file
    with open(OUTPUT_FILE, 'w') as f:
        f.write('// Auto-generated image data for entrance + blink animation\n')
        f.write(f'// Palette: {PALETTE_SIZE} colors, shared across all frames\n')
        f.write('// All frames: 4-bit packed (2 pixels/byte)\n')
        f.write('// Run: python3 resources/generate_image_data.py\n\n')
        f.write(f'static const uint8_t IMG_W = {img_w};\n')
        f.write(f'static const uint8_t IMG_H = {img_h};\n')
        f.write(f'static const uint8_t EYE_X = {EYE_X};\n')
        f.write(f'static const uint8_t EYE_Y = {EYE_Y};\n')
        f.write(f'static const uint8_t EYE_W = {EYE_W};\n')
        f.write(f'static const uint8_t EYE_H = {EYE_H};\n')
        f.write(f'static const uint8_t ENTRANCE_FRAMES = {ENTRANCE_FRAMES};\n')
        f.write(f'static const uint16_t ENTRANCE_INTERVAL_MS = {ENTRANCE_INTERVAL_MS};\n\n')

        # Shared palette (16 colors)
        write_c_array(f, 'uint16_t', 'palette', palette, per_line=8)

        # Entrance frames - 4-bit packed
        write_c_array(f, 'uint8_t', 'entranceFrame0', ent0_packed)
        write_c_array(f, 'uint8_t', 'entranceFrame1', ent1_packed)

        # Base frame - 4-bit packed
        write_c_array(f, 'uint8_t', 'baseFrame', base_packed)

        # Eye overlays - 4-bit packed
        write_c_array(f, 'uint8_t', 'eyeHalf', half_packed)
        write_c_array(f, 'uint8_t', 'eyeClosed', closed_packed)

    # Summary
    pal_bytes = PALETTE_SIZE * 2
    ent_bytes = len(ent0_packed) + len(ent1_packed)
    base_bytes = len(base_packed)
    eye_bytes = len(half_packed) + len(closed_packed)
    total = pal_bytes + ent_bytes + base_bytes + eye_bytes

    print(f"\nGenerated: {OUTPUT_FILE}")
    print(f"Palette: {PALETTE_SIZE} colors = {pal_bytes} bytes")
    print(f"Entrance 4-bit: {len(ent0_packed)} + {len(ent1_packed)} = {ent_bytes} bytes")
    print(f"Base 4-bit: {base_bytes} bytes")
    print(f"Eyes 4-bit: {len(half_packed)} + {len(closed_packed)} = {eye_bytes} bytes")
    print(f"Total image data: {total} bytes")


if __name__ == '__main__':
    main()
