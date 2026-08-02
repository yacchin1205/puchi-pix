#!/usr/bin/env python3
"""
Generate image_data.h for Puchi-Pix animation from a GIF.

Output format: Frame structure array with linked-list style next pointers.
Each frame is either a full frame or an overlay on a reference frame.
The .ino decides which frame to start at, where to loop, etc.

Run: python3 resources/generate_image_data.py <gif_path> <config> [gamma] [sat_power]

gamma (optional, default 1.0): applied to the output palette only.
  OLED panels drive pixels near-linearly, so sRGB artwork looks washed out;
  gamma 1.6-2.2 darkens midtones and restores perceived contrast/saturation.

sat_power (optional, default 1.0): HSV saturation curve S' = S^p, palette only.
  p < 1 boosts pale colors (e.g. skin tones that read as white on OLED) much
  more than already-saturated ones. 0.5-0.7 is a good range. Applied before gamma.

Config is a comma-separated list of frame specs:
  <gif_frame>:<next>[:<ref>[:<duration_ms>]]

  ref: reference frame index for overlay (omit or empty for full frame)
  duration_ms: display duration in ms (default: 150)

Example (blink loop):
  0::2000:1,1:2:0:150,2:3:0:150,3:0:0:150

  output frame 0 = gif frame 0, full, 2000ms, next=1
  output frame 1 = gif frame 1, overlay on 0, 150ms, next=2
  output frame 2 = gif frame 2, overlay on 0, 150ms, next=3
  output frame 3 = gif frame 1, overlay on 0, 150ms, next=0
"""
from PIL import Image
import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, '..', 'firmware', 'puchi_pix')

OVERLAY_THRESHOLD = 0.5
PALETTE_SIZE = 16


def rgb_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


# Pixel art with few unique colors: merge nearest color pairs instead of
# MEDIANCUT, which tends to drop small-but-salient colors (eye highlights,
# accent colors on clothing).
MERGE_QUANTIZE_MAX_COLORS = 48


def quantize_by_merging(img_arr, max_colors):
    import math
    u, c = np.unique(img_arr.reshape(-1, 3), axis=0, return_counts=True)
    colors = [tuple(int(v) for v in x) for x in u]
    weight = {col: int(n) for col, n in zip(colors, c)}
    protected = {(0, 0, 0), (255, 255, 255)} & set(colors)
    mapping = {col: col for col in colors}
    cur = list(colors)
    while len(cur) > max_colors:
        best = None
        best_score = float('inf')
        for a in range(len(cur)):
            for b in range(a + 1, len(cur)):
                if cur[a] in protected or cur[b] in protected:
                    continue
                d = sum((cur[a][k] - cur[b][k]) ** 2 for k in range(3))
                # sqrt(count) weighting: rare near-duplicates merge first,
                # but visually distinct rare colors survive
                score = d * math.sqrt(min(weight[cur[a]], weight[cur[b]]))
                if score < best_score:
                    best_score = score
                    best = (cur[a], cur[b])
        ca, cb = best
        keep, drop = (ca, cb) if weight[ca] >= weight[cb] else (cb, ca)
        for col in colors:
            if mapping[col] == drop:
                mapping[col] = keep
        weight[keep] += weight[drop]
        cur.remove(drop)
        print(f"  Merge: {drop} -> {keep}")

    palette_rgb = [list(col) for col in cur]
    pal_index = {col: i for i, col in enumerate(cur)}
    h, w = img_arr.shape[:2]
    indexed = np.zeros((h, w), dtype=np.uint8)
    flat = img_arr.reshape(-1, 3)
    idx_flat = np.array([pal_index[mapping[tuple(int(v) for v in px)]] for px in flat], dtype=np.uint8)
    indexed = idx_flat.reshape(h, w)
    while len(palette_rgb) < max_colors:
        palette_rgb.append([0, 0, 0])
    palette_rgb565 = [rgb_to_rgb565(r, g, b) for r, g, b in palette_rgb]
    return palette_rgb565, palette_rgb, indexed


def quantize_to_palette(img_arr, max_colors=PALETTE_SIZE):
    n_unique = len(np.unique(img_arr.reshape(-1, 3), axis=0))
    if n_unique <= MERGE_QUANTIZE_MAX_COLORS:
        print(f"  Quantize: merge method ({n_unique} unique colors)")
        return quantize_by_merging(img_arr, max_colors)

    img = Image.fromarray(img_arr)
    quantized = img.quantize(colors=max_colors, method=Image.MEDIANCUT)
    palette_data = quantized.getpalette()[:max_colors * 3]
    indexed = np.array(quantized)

    palette_rgb = []
    for i in range(0, len(palette_data), 3):
        palette_rgb.append([palette_data[i], palette_data[i + 1], palette_data[i + 2]])

    # Ensure important colors (black, white) are in the palette
    flat_orig = img_arr.reshape(-1, 3)
    important_colors = [[0, 0, 0], [255, 255, 255]]
    for ic in important_colors:
        has_in_orig = np.any(np.all(flat_orig == ic, axis=1))
        has_in_palette = any(p == ic for p in palette_rgb)
        if has_in_orig and not has_in_palette:
            flat_indexed = indexed.flatten()
            counts = np.bincount(flat_indexed, minlength=max_colors)
            least_used = int(np.argmin(counts))
            old_color = palette_rgb[least_used]
            palette_rgb[least_used] = ic
            orig_is_ic = np.all(img_arr == ic, axis=2)
            indexed[orig_is_ic] = least_used
            print(f"  Palette fix: replaced {old_color} (idx {least_used}, {counts[least_used]} uses) with {ic}")

    palette_rgb565 = []
    for r, g, b in palette_rgb:
        palette_rgb565.append(rgb_to_rgb565(r, g, b))

    return palette_rgb565, palette_rgb, indexed


def pack_4bit(data):
    flat = data.flatten().tolist()
    packed = []
    for i in range(0, len(flat), 2):
        hi = flat[i] & 0x0F
        lo = flat[i + 1] & 0x0F if i + 1 < len(flat) else 0
        packed.append((hi << 4) | lo)
    return packed


def write_c_array(f, type_str, name, data, per_line=16):
    f.write(f'static const {type_str} {name}[{len(data)}] PROGMEM = {{\n')
    fmt = '0x{:04X}' if '16' in type_str else '0x{:02X}'
    for i in range(0, len(data), per_line):
        line = ', '.join(fmt.format(data[j]) for j in range(i, min(i + per_line, len(data))))
        f.write(f'  {line},\n')
    f.seek(f.tell() - 2)
    f.write('\n};\n\n')


def extract_gif_frames(gif_path):
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


def replace_background(frames, base_idx):
    """Replace edge-connected background color with black."""
    from scipy import ndimage
    flat = frames[base_idx].reshape(-1, 3)
    unique, counts = np.unique(flat, axis=0, return_counts=True)
    bg_color = unique[np.argmax(counts)]
    total_replaced = 0
    for i in range(len(frames)):
        bg_mask = np.all(frames[i] == bg_color, axis=2)
        labeled, _ = ndimage.label(bg_mask)
        edge_labels = set()
        h, w = bg_mask.shape
        for edge in [labeled[0, :], labeled[h-1, :], labeled[:, 0], labeled[:, w-1]]:
            edge_labels.update(edge[edge > 0].tolist())
        exterior_mask = np.isin(labeled, list(edge_labels))
        frames[i][exterior_mask] = [0, 0, 0]
        total_replaced += exterior_mask.sum()
    print(f"Background {bg_color} -> black (flood fill, {total_replaced} px)")
    return frames


def compute_diff_region(base, frame, img_w, img_h):
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


def parse_config(config_str):
    """Parse frame config string.
    Format: gif_frame:next[:ref[:duration_ms]], ...
    Empty ref = full frame.  Default duration = 150ms.
    """
    frame_specs = []
    for spec in config_str.split(','):
        parts = spec.strip().split(':')
        gif_idx = int(parts[0])
        next_idx = int(parts[1])
        ref_idx = int(parts[2]) if len(parts) > 2 and parts[2] != '' else None
        duration = int(parts[3]) if len(parts) > 3 and parts[3] != '' else 150
        frame_specs.append({'gif': gif_idx, 'next': next_idx, 'ref': ref_idx, 'duration': duration})
    return frame_specs


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <gif_path> <config>")
        print(f"  config: comma-separated frame specs: gif_frame:next[:ref[:duration_ms]]")
        print(f"  Example: 0:1::2000,1:2:0:150,2:3:0:150,3:0:0:150")
        sys.exit(1)

    gif_path = sys.argv[1]
    config_str = sys.argv[2]
    gamma = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    sat_power = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    output_name = os.path.splitext(os.path.basename(gif_path))[0]
    output_file = os.path.join(OUTPUT_DIR, f'icon_{output_name}.h')

    if not os.path.exists(gif_path):
        print(f"Error: GIF not found: {gif_path}")
        sys.exit(1)

    print(f"Reading GIF: {gif_path}")
    gif_frames = extract_gif_frames(gif_path)
    print(f"Extracted {len(gif_frames)} GIF frames")

    frame_specs = parse_config(config_str)
    n_frames = len(frame_specs)
    print(f"Output frames: {n_frames}")

    # Determine base frame for background detection (first full frame)
    base_gif_idx = None
    for spec in frame_specs:
        if spec['ref'] is None:
            base_gif_idx = spec['gif']
            break
    if base_gif_idx is None:
        base_gif_idx = frame_specs[0]['gif']

    # Replace background
    gif_frames = replace_background(gif_frames, base_gif_idx)

    img_h, img_w = gif_frames[0].shape[:2]
    print(f"Image size: {img_w}x{img_h}")

    # Determine frame types and regions
    frame_info = []  # list of { 'type': 'full'|'overlay', 'region': (x,y,w,h)|None, ... }
    for i, spec in enumerate(frame_specs):
        src = gif_frames[spec['gif']]
        if spec['ref'] is not None:
            ref_src = gif_frames[frame_specs[spec['ref']]['gif']]
            region = compute_diff_region(ref_src, src, img_w, img_h)
            if region and (region[2] * region[3]) / (img_w * img_h) < OVERLAY_THRESHOLD:
                ftype = 'overlay'
                print(f"  Frame {i}: overlay on {spec['ref']} (region {region[2]}x{region[3]} at ({region[0]},{region[1]}))")
            else:
                ftype = 'full'
                region = None
                print(f"  Frame {i}: full (ref specified but diff too large)")
        else:
            ftype = 'full'
            region = None
            print(f"  Frame {i}: full")
        frame_info.append({'type': ftype, 'region': region, 'spec': spec})

    # Build combined image for palette quantization
    parts = []
    part_map = []

    for i, info in enumerate(frame_info):
        src = gif_frames[info['spec']['gif']]
        if info['type'] == 'full':
            parts.append(src)
            part_map.append(('full', i, img_h, img_w))
        else:
            x, y, w, h = info['region']
            region = src[y:y+h, x:x+w]
            padded = np.zeros((h, img_w, 3), dtype=region.dtype)
            padded[:, :w] = region
            parts.append(padded)
            part_map.append(('overlay', i, h, w))

    combined = np.vstack(parts)
    palette_565, palette_rgb, combined_indexed = quantize_to_palette(combined, PALETTE_SIZE)
    print(f"Palette: {len(palette_565)} colors")

    # Optional saturation boost + gamma correction, applied to the output palette
    # only (verification below still compares against the original colors)
    if sat_power != 1.0 or gamma != 1.0:
        import colorsys
        corrected = []
        for rgb in palette_rgb:
            r, g, b = [v / 255 for v in rgb]
            if sat_power != 1.0:
                h, s, v = colorsys.rgb_to_hsv(r, g, b)
                r, g, b = colorsys.hsv_to_rgb(h, s ** sat_power, v)
            corrected.append([round(255 * c ** gamma) for c in (r, g, b)])
        palette_565 = [rgb_to_rgb565(r, g, b) for r, g, b in corrected]
        print(f"Palette correction: sat_power={sat_power}, gamma={gamma}")

    # Extract indexed data
    indexed_data = {}
    y_offset = 0
    for kind, idx, height, width in part_map:
        if kind == 'full':
            indexed_data[idx] = combined_indexed[y_offset:y_offset+height]
        else:
            indexed_data[idx] = combined_indexed[y_offset:y_offset+height, :width]
        y_offset += height

    # Pack
    packed = {}
    for idx, data in indexed_data.items():
        packed[idx] = pack_4bit(data)

    # Dedupe identical frame data (e.g. the same gif frame appearing twice in a sequence)
    data_owner = {}   # output frame idx -> idx whose array holds the data
    seen = {}
    for i in range(n_frames):
        key = (bytes(packed[i]), frame_info[i]['type'], frame_info[i]['region'])
        if key in seen:
            data_owner[i] = seen[key]
            print(f"  Frame {i}: identical to frame {seen[key]}, sharing data array")
        else:
            seen[key] = i
            data_owner[i] = i

    # Generate header
    with open(output_file, 'w') as f:
        f.write(f'// Auto-generated image data: {output_name}\n')
        f.write(f'// Source: {os.path.basename(gif_path)}\n')
        f.write(f'// Config: {config_str}\n')
        f.write(f'// Run: python3 resources/generate_image_data.py "{gif_path}" "{config_str}" {gamma} {sat_power}\n\n')

        f.write(f'#define IMG_W {img_w}\n')
        f.write(f'#define IMG_H {img_h}\n')
        f.write(f'#define FRAME_COUNT {n_frames}\n')
        f.write(f'#define PALETTE_SIZE {PALETTE_SIZE}\n\n')
        f.write('#include "frame.h"\n\n')

        # Palette
        write_c_array(f, 'uint16_t', 'palette', palette_565, per_line=8)

        # Frame data arrays
        for i in range(n_frames):
            if data_owner[i] == i:
                write_c_array(f, 'uint8_t', f'frame_data_{i}', packed[i])

        f.write(f'static const Frame frames[FRAME_COUNT] PROGMEM = {{\n')
        for i, info in enumerate(frame_info):
            ftype = 0 if info['type'] == 'full' else 1
            next_idx = info['spec']['next']
            ref_idx = info['spec']['ref'] if info['spec']['ref'] is not None else 0
            if info['region']:
                rx, ry, rw, rh = info['region']
            else:
                rx, ry, rw, rh = 0, 0, 0, 0
            duration = info['spec']['duration']
            f.write(f'  {{ {ftype}, {next_idx}, {ref_idx}, {rx}, {ry}, {rw}, {rh}, {duration}, frame_data_{data_owner[i]} }},')
            f.write(f'  // f{i}: {"overlay on "+str(ref_idx) if ftype else "full"}')
            f.write(f' {duration}ms -> f{next_idx}\n')
        f.write('};\n')

    # Summary
    total = PALETTE_SIZE * 2 + sum(len(packed[i]) for i in range(n_frames) if data_owner[i] == i) + n_frames * 8
    print(f"\nGenerated: {output_file}")
    for i in range(n_frames):
        shared = '' if data_owner[i] == i else f" (shared with frame {data_owner[i]})"
        print(f"  Frame {i} ({frame_info[i]['type']}): {len(packed[i])} bytes{shared}")
    print(f"Total image data: {total} bytes")

    # Verification
    print(f"\n--- Verification ---")
    palette_lut = np.array(palette_rgb, dtype=np.uint8)
    def indexed_to_rgb(arr):
        return palette_lut[arr]

    verify_ok = True
    verify_dir = os.path.join(SCRIPT_DIR, 'verify')
    os.makedirs(verify_dir, exist_ok=True)

    for i, info in enumerate(frame_info):
        if info['type'] == 'full':
            reconstructed = indexed_to_rgb(indexed_data[i])
        else:
            ref_idx = info['spec']['ref']
            assert frame_info[ref_idx]['type'] == 'full', f"Frame {i} ref {ref_idx} is not full"
            reconstructed = indexed_to_rgb(indexed_data[ref_idx]).copy()
            x, y, w, h = info['region']
            reconstructed[y:y+h, x:x+w] = indexed_to_rgb(indexed_data[i])

        original = gif_frames[info['spec']['gif']]
        diff = np.any(reconstructed != original, axis=2)
        diff_count = diff.sum()

        if diff_count > 0:
            print(f"  Frame {i}: {diff_count} pixels differ (quantization)")
            Image.fromarray(reconstructed).save(os.path.join(verify_dir, f'reconstructed_{i}.png'))
            Image.fromarray(original).save(os.path.join(verify_dir, f'original_{i}.png'))
        else:
            print(f"  Frame {i}: OK (exact match)")

    # White highlight check on first full frame
    first_full = next(i for i, info in enumerate(frame_info) if info['type'] == 'full')
    orig = gif_frames[frame_info[first_full]['spec']['gif']]
    recon = indexed_to_rgb(indexed_data[first_full])
    white_orig = np.all(orig == [255, 255, 255], axis=2).sum()
    white_recon = np.all(recon == [255, 255, 255], axis=2).sum()
    lost = np.all(orig == [255, 255, 255], axis=2) & ~np.all(recon == [255, 255, 255], axis=2)
    if lost.sum() > 0:
        print(f"  WARNING: {lost.sum()} white highlight pixels lost!")
        verify_ok = False
    else:
        print(f"  White highlights: preserved ({white_orig} px)")

    print("Verification PASSED" if verify_ok else "Verification FAILED")


if __name__ == '__main__':
    main()
