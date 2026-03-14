#!/usr/bin/env python3
"""
Generate image_data.h for Puchi-Pix animation from a GIF.

Each non-base frame is automatically stored as either:
  - Full frame (4-bit packed) if diff from base is large
  - Overlay region (4-bit packed, bounding box only) if diff is small

The threshold for overlay vs full is configurable (OVERLAY_THRESHOLD).

Run: python3 resources/generate_image_data.py [gif_path]
"""
from PIL import Image
import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(SCRIPT_DIR, '..', 'firmware', 'puchi_pix', 'image_data.h')

# Default GIF path (can be overridden via command line)
DEFAULT_GIF = os.path.join(SCRIPT_DIR, '..', 'animation.gif')

# Frame assignments
ENTRANCE_FRAME_INDICES = [0, 1, 2]
BASE_FRAME_INDEX = 3
BLINK_FRAME_INDICES = [4, 5]  # overlay frames for blink animation

# If overlay region occupies less than this fraction of full frame,
# store as overlay. Otherwise store as full frame.
OVERLAY_THRESHOLD = 0.5

ENTRANCE_INTERVAL_MS = 200
PALETTE_SIZE = 16


def rgb_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def quantize_to_palette(img_arr, max_colors=PALETTE_SIZE):
    img = Image.fromarray(img_arr)
    quantized = img.quantize(colors=max_colors, method=Image.MEDIANCUT)
    palette_data = quantized.getpalette()[:max_colors * 3]

    palette_rgb565 = []
    palette_rgb = []
    for i in range(0, len(palette_data), 3):
        r, g, b = palette_data[i], palette_data[i + 1], palette_data[i + 2]
        palette_rgb565.append(rgb_to_rgb565(r, g, b))
        palette_rgb.append([r, g, b])

    indexed = np.array(quantized)
    return palette_rgb565, palette_rgb, indexed


def pack_4bit(data):
    """Pack palette indices (0-15) into 4-bit pairs, 2 pixels per byte."""
    flat = data.flatten().tolist()
    packed = []
    for i in range(0, len(flat), 2):
        hi = flat[i] & 0x0F
        lo = flat[i + 1] & 0x0F if i + 1 < len(flat) else 0
        packed.append((hi << 4) | lo)
    return packed


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


def compute_diff_region(base, frame, img_w, img_h):
    """Compute bounding box of pixel differences between base and frame.
    Returns (x, y, w, h) with 1px margin and even width, or None if identical."""
    diff = np.any(base != frame, axis=2)
    if not diff.any():
        return None
    ys, xs = np.where(diff)
    x = max(0, int(xs.min()) - 1)
    y = max(0, int(ys.min()) - 1)
    w = min(img_w, int(xs.max()) + 2) - x
    h = min(img_h, int(ys.max()) + 2) - y
    if w % 2 != 0:
        w = min(img_w - x, w + 1)
    return (x, y, w, h)


def should_use_overlay(region, img_w, img_h):
    """Decide whether to store as overlay (True) or full frame (False)."""
    if region is None:
        return True  # identical to base, store as tiny overlay
    x, y, w, h = region
    region_pixels = w * h
    full_pixels = img_w * img_h
    return (region_pixels / full_pixels) < OVERLAY_THRESHOLD


def main():
    gif_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GIF

    if not os.path.exists(gif_path):
        print(f"Error: GIF not found: {gif_path}")
        sys.exit(1)

    print(f"Reading GIF: {gif_path}")
    frames = extract_gif_frames(gif_path)
    print(f"Extracted {len(frames)} frames")

    all_indices = ENTRANCE_FRAME_INDICES + [BASE_FRAME_INDEX] + BLINK_FRAME_INDICES
    min_frames = max(all_indices) + 1
    if len(frames) < min_frames:
        print(f"Error: Expected at least {min_frames} frames, got {len(frames)}")
        sys.exit(1)

    # Replace background color with black using flood fill from edges
    # This preserves interior pixels of the same color (e.g. white highlights)
    from scipy import ndimage
    flat = frames[BASE_FRAME_INDEX].reshape(-1, 3)
    unique, counts = np.unique(flat, axis=0, return_counts=True)
    bg_color = unique[np.argmax(counts)]
    total_replaced = 0
    for i in range(len(frames)):
        bg_mask = np.all(frames[i] == bg_color, axis=2)
        # Label connected components of background-colored pixels
        labeled, num_features = ndimage.label(bg_mask)
        # Find labels that touch any edge
        edge_labels = set()
        h, w = bg_mask.shape
        for edge in [labeled[0, :], labeled[h-1, :], labeled[:, 0], labeled[:, w-1]]:
            edge_labels.update(edge[edge > 0].tolist())
        # Only replace background pixels connected to edges
        exterior_mask = np.isin(labeled, list(edge_labels))
        frames[i][exterior_mask] = [0, 0, 0]
        total_replaced += exterior_mask.sum()
    print(f"Background {bg_color} -> black (flood fill, {total_replaced} px total)")

    base_arr = frames[BASE_FRAME_INDEX]
    img_h, img_w = base_arr.shape[:2]
    print(f"Image size: {img_w}x{img_h}")

    # Analyze all non-base frames: decide overlay vs full
    non_base_indices = ENTRANCE_FRAME_INDICES + BLINK_FRAME_INDICES
    frame_info = {}  # index -> { 'mode': 'full'|'overlay', 'region': (x,y,w,h)|None }
    for i in non_base_indices:
        region = compute_diff_region(base_arr, frames[i], img_w, img_h)
        use_overlay = should_use_overlay(region, img_w, img_h)
        mode = 'overlay' if use_overlay else 'full'
        frame_info[i] = {'mode': mode, 'region': region}
        if region:
            x, y, w, h = region
            savings = (img_w * img_h // 2) - (w * h // 2) if use_overlay else 0
            print(f"  Frame {i}: {mode} (diff region {w}x{h} at ({x},{y}), {savings}B saved)" if use_overlay
                  else f"  Frame {i}: {mode} (diff region {w}x{h}, too large for overlay)")
        else:
            print(f"  Frame {i}: overlay (identical to base)")

    # Determine blink overlay region (union of all blink frame diffs)
    blink_regions = [frame_info[i]['region'] for i in BLINK_FRAME_INDICES if frame_info[i]['region']]
    if blink_regions:
        bx = min(r[0] for r in blink_regions)
        by = min(r[1] for r in blink_regions)
        bx2 = max(r[0] + r[2] for r in blink_regions)
        by2 = max(r[1] + r[3] for r in blink_regions)
        bw = bx2 - bx
        bh = by2 - by
        if bw % 2 != 0:
            bw = min(img_w - bx, bw + 1)
        EYE_X, EYE_Y, EYE_W, EYE_H = bx, by, bw, bh
    else:
        EYE_X, EYE_Y, EYE_W, EYE_H = 0, 0, 2, 2
    print(f"Blink overlay region: ({EYE_X},{EYE_Y}) {EYE_W}x{EYE_H}")

    # Collect entrance overlay regions (for overlay-mode entrance frames)
    entrance_overlay_regions = {}
    for i in ENTRANCE_FRAME_INDICES:
        if frame_info[i]['mode'] == 'overlay' and frame_info[i]['region']:
            entrance_overlay_regions[i] = frame_info[i]['region']

    # Build combined image for unified palette quantization
    # Include: all full entrance frames, base, all overlay regions
    parts = []
    part_map = []  # track what's where

    # Full entrance frames
    for i in ENTRANCE_FRAME_INDICES:
        if frame_info[i]['mode'] == 'full':
            parts.append(frames[i])
            part_map.append(('entrance_full', i, img_h))

    # Base frame
    parts.append(base_arr)
    part_map.append(('base', BASE_FRAME_INDEX, img_h))

    # Entrance overlay regions
    for i in ENTRANCE_FRAME_INDICES:
        if frame_info[i]['mode'] == 'overlay' and frame_info[i]['region']:
            x, y, w, h = frame_info[i]['region']
            region = frames[i][y:y+h, x:x+w]
            padded = np.zeros((h, img_w, 3), dtype=region.dtype)
            padded[:, :w] = region
            parts.append(padded)
            part_map.append(('entrance_overlay', i, h))

    # Blink overlay regions
    for i in BLINK_FRAME_INDICES:
        region = frames[i][EYE_Y:EYE_Y+EYE_H, EYE_X:EYE_X+EYE_W]
        padded = np.zeros((EYE_H, img_w, 3), dtype=region.dtype)
        padded[:, :EYE_W] = region
        parts.append(padded)
        part_map.append(('blink_overlay', i, EYE_H))

    combined = np.vstack(parts)
    palette, quantized_palette_rgb, combined_indexed = quantize_to_palette(combined, PALETTE_SIZE)
    print(f"Palette: {len(palette)} colors")

    # Extract indexed data from combined
    y_offset = 0
    indexed_data = {}
    for kind, idx, height in part_map:
        if kind == 'entrance_full':
            indexed_data[('entrance_full', idx)] = combined_indexed[y_offset:y_offset+height]
        elif kind == 'base':
            indexed_data[('base', idx)] = combined_indexed[y_offset:y_offset+height]
        elif kind == 'entrance_overlay':
            x, _, w, h = frame_info[idx]['region']
            indexed_data[('entrance_overlay', idx)] = combined_indexed[y_offset:y_offset+height, :w]
        elif kind == 'blink_overlay':
            indexed_data[('blink_overlay', idx)] = combined_indexed[y_offset:y_offset+height, :EYE_W]
        y_offset += height

    # Pack all data
    packed = {}
    for key, data in indexed_data.items():
        packed[key] = pack_4bit(data)

    # Count entrance frames by type
    entrance_full_count = sum(1 for i in ENTRANCE_FRAME_INDICES if frame_info[i]['mode'] == 'full')
    entrance_overlay_count = sum(1 for i in ENTRANCE_FRAME_INDICES if frame_info[i]['mode'] == 'overlay')

    # Generate header file
    with open(OUTPUT_FILE, 'w') as f:
        f.write('// Auto-generated image data for Puchi-Pix animation\n')
        f.write(f'// Palette: {PALETTE_SIZE} colors, shared across all frames\n')
        f.write('// Full frames: 4-bit packed (2 pixels/byte)\n')
        f.write('// Overlay frames: 4-bit packed, region only\n')
        f.write('// Run: python3 resources/generate_image_data.py\n\n')
        f.write(f'static const uint8_t IMG_W = {img_w};\n')
        f.write(f'static const uint8_t IMG_H = {img_h};\n')
        f.write(f'static const uint8_t EYE_X = {EYE_X};\n')
        f.write(f'static const uint8_t EYE_Y = {EYE_Y};\n')
        f.write(f'static const uint8_t EYE_W = {EYE_W};\n')
        f.write(f'static const uint8_t EYE_H = {EYE_H};\n')

        # Entrance info
        total_entrance = len(ENTRANCE_FRAME_INDICES)
        f.write(f'static const uint8_t ENTRANCE_FRAMES = {total_entrance};\n')
        f.write(f'static const uint16_t ENTRANCE_INTERVAL_MS = {ENTRANCE_INTERVAL_MS};\n\n')

        # For each entrance frame, record whether it's full or overlay
        # Generate a flag array: 0 = full frame, 1 = overlay on base
        entrance_flags = []
        for i in ENTRANCE_FRAME_INDICES:
            entrance_flags.append(0 if frame_info[i]['mode'] == 'full' else 1)
        write_c_array(f, 'uint8_t', 'entranceIsOverlay', entrance_flags, per_line=16)

        # For overlay entrance frames, record the region
        overlay_entrance_regions = []
        for i in ENTRANCE_FRAME_INDICES:
            if frame_info[i]['mode'] == 'overlay' and frame_info[i]['region']:
                x, y, w, h = frame_info[i]['region']
                overlay_entrance_regions.extend([x, y, w, h])
            else:
                overlay_entrance_regions.extend([0, 0, 0, 0])
        write_c_array(f, 'uint8_t', 'entranceOverlayRegion', overlay_entrance_regions, per_line=4)

        # Shared palette
        write_c_array(f, 'uint16_t', 'palette', palette, per_line=8)

        # Entrance frames
        entrance_idx = 0
        for i in ENTRANCE_FRAME_INDICES:
            if frame_info[i]['mode'] == 'full':
                write_c_array(f, 'uint8_t', f'entranceFrame{entrance_idx}',
                              packed[('entrance_full', i)])
            else:
                write_c_array(f, 'uint8_t', f'entranceFrame{entrance_idx}',
                              packed[('entrance_overlay', i)])
            entrance_idx += 1

        # Base frame
        write_c_array(f, 'uint8_t', 'baseFrame', packed[('base', BASE_FRAME_INDEX)])

        # Blink overlays
        blink_names = ['eyeHalf', 'eyeClosed']
        for name, i in zip(blink_names, BLINK_FRAME_INDICES):
            write_c_array(f, 'uint8_t', name, packed[('blink_overlay', i)])

    # Summary
    pal_bytes = PALETTE_SIZE * 2
    ent_bytes = sum(len(packed[k]) for k in packed if k[0].startswith('entrance'))
    base_bytes = len(packed[('base', BASE_FRAME_INDEX)])
    blink_bytes = sum(len(packed[k]) for k in packed if k[0] == 'blink_overlay')
    meta_bytes = len(entrance_flags) + len(overlay_entrance_regions)
    total = pal_bytes + ent_bytes + base_bytes + blink_bytes + meta_bytes

    print(f"\nGenerated: {OUTPUT_FILE}")
    print(f"Palette: {PALETTE_SIZE} colors = {pal_bytes} bytes")
    for i, idx in enumerate(ENTRANCE_FRAME_INDICES):
        mode = frame_info[idx]['mode']
        if mode == 'full':
            size = len(packed[('entrance_full', idx)])
        else:
            size = len(packed[('entrance_overlay', idx)])
        print(f"Entrance {i} ({mode}): {size} bytes")
    print(f"Base 4-bit: {base_bytes} bytes")
    print(f"Blink overlays: {blink_bytes} bytes")
    print(f"Metadata: {meta_bytes} bytes")
    print(f"Total image data: {total} bytes")

    # ========== Verification ==========
    # Reconstruct frames from indexed data and compare with originals
    print(f"\n--- Verification ---")

    # Build RGB palette for reconstruction
    palette_lut = np.array(quantized_palette_rgb, dtype=np.uint8)
    def indexed_to_rgb(indexed_arr):
        return palette_lut[indexed_arr]

    verify_ok = True
    verify_dir = os.path.join(SCRIPT_DIR, 'verify')
    os.makedirs(verify_dir, exist_ok=True)

    for frame_idx in ENTRANCE_FRAME_INDICES + BLINK_FRAME_INDICES:
        # Reconstruct frame
        if frame_idx in ENTRANCE_FRAME_INDICES:
            if frame_info[frame_idx]['mode'] == 'full':
                reconstructed = indexed_to_rgb(indexed_data[('entrance_full', frame_idx)])
            else:
                # Overlay on base
                reconstructed = indexed_to_rgb(indexed_data[('base', BASE_FRAME_INDEX)]).copy()
                x, y, w, h = frame_info[frame_idx]['region']
                overlay_rgb = indexed_to_rgb(indexed_data[('entrance_overlay', frame_idx)])
                reconstructed[y:y+h, x:x+w] = overlay_rgb
        else:
            # Blink: overlay on base
            reconstructed = indexed_to_rgb(indexed_data[('base', BASE_FRAME_INDEX)]).copy()
            overlay_rgb = indexed_to_rgb(indexed_data[('blink_overlay', frame_idx)])
            reconstructed[EYE_Y:EYE_Y+EYE_H, EYE_X:EYE_X+EYE_W] = overlay_rgb

        # Compare with original (after bg replacement)
        original = frames[frame_idx]
        diff = np.any(reconstructed != original, axis=2)
        diff_count = diff.sum()

        if diff_count > 0:
            print(f"  Frame {frame_idx}: {diff_count} pixels differ (quantization)")
            # Save diff image for inspection
            diff_img = np.zeros_like(original)
            diff_img[diff] = [255, 0, 0]  # red for differences
            Image.fromarray(diff_img).save(os.path.join(verify_dir, f'diff_frame{frame_idx}.png'))
            Image.fromarray(reconstructed).save(os.path.join(verify_dir, f'reconstructed_frame{frame_idx}.png'))
            Image.fromarray(original).save(os.path.join(verify_dir, f'original_frame{frame_idx}.png'))
        else:
            print(f"  Frame {frame_idx}: OK (exact match)")

    # Also verify base frame
    base_reconstructed = indexed_to_rgb(indexed_data[('base', BASE_FRAME_INDEX)])
    base_diff = np.any(base_reconstructed != frames[BASE_FRAME_INDEX], axis=2)
    base_diff_count = base_diff.sum()
    if base_diff_count > 0:
        print(f"  Base frame: {base_diff_count} pixels differ (quantization)")
        Image.fromarray(base_reconstructed).save(os.path.join(verify_dir, f'reconstructed_base.png'))
        Image.fromarray(frames[BASE_FRAME_INDEX]).save(os.path.join(verify_dir, f'original_base.png'))
    else:
        print(f"  Base frame: OK (exact match)")

    # Check specifically for white highlight preservation
    white_original = np.all(frames[BASE_FRAME_INDEX] == [255, 255, 255], axis=2)
    white_reconstructed = np.all(base_reconstructed == [255, 255, 255], axis=2)
    lost_white = white_original & ~white_reconstructed
    if lost_white.sum() > 0:
        print(f"  WARNING: {lost_white.sum()} white highlight pixels lost in base frame!")
        verify_ok = False
    else:
        gained = (~white_original & white_reconstructed).sum()
        print(f"  White highlights: preserved ({white_original.sum()} px original, {white_reconstructed.sum()} px reconstructed)")

    if verify_ok:
        print("Verification PASSED")
    else:
        print("Verification FAILED - check verify/ directory")


if __name__ == '__main__':
    main()
