---
name: generate-icon-data
description: Generate icon header for Puchi-Pix from a pixel art GIF animation. Use when the user provides a new GIF icon and wants to convert it to firmware image data.
argument-hint: [gif-path]
allowed-tools: Read, Bash, Glob, Grep, Write, Edit
---

# Generate Icon Data for Puchi-Pix

Convert a pixel art GIF animation into `icon_<name>.h` for the Puchi-Pix firmware.

## Prerequisites

- Python 3 with PIL/Pillow and NumPy
- ImageMagick (for frame extraction/visualization)

## Process

### Step 1: Analyze the GIF

Extract and display all frames from the GIF:

```bash
identify "$GIF_PATH"
for i in $(seq 0 N); do
  magick "$GIF_PATH[$i]" -coalesce "/tmp/frame${i}.png"
done
```

Show each frame to the user (use Read tool to display PNGs).

Then generate a **diff matrix** showing pixel differences between all frame pairs:

```python
from PIL import Image
import numpy as np

# For each pair (i, j), count pixels that differ
# Display as a matrix: "=" means identical, otherwise show pixel count
# This reveals which frames are duplicates and which are unique
```

Present the matrix to the user. This helps identify:
- Which frames are identical (duplicates to ignore)
- Which frames are unique
- Relative magnitude of changes between frames

### Step 2: Ask the user about frame roles

Ask the user two questions only:
1. **"Which frame is the base (idle) frame?"** - the normal resting state of the character
2. **"Which frames are the entrance animation?"** - typically the frames before the base frame

Do NOT assume what the remaining frames represent. The user will describe them.
The remaining frames (after entrance and base) are the **animation overlay frames** - they may be blink frames, expression changes, or anything else.

### Step 3: Build the frame config

Config format for `generate_image_data.py`:
```
gif_frame:next[:ref[:duration_ms]]
```

- `ref`: reference frame index for overlay (omit or empty for full frame)
- `duration_ms`: display duration in ms (default: 150)
- Entrance frames: full, 150ms each, chained sequentially
- Base frame: full, 2000ms (idle wait), next points to first animation frame
- Animation frames: overlay on base, 150ms each, last one loops back to base
- If gif frames are identical (diff=0), reuse the same gif frame index to avoid duplicate data

Example (entrance f0-f2, base f3, blink f4-f6):
```
0:1::150,1:2::150,2:3::150,3:4::2000,4:5:3:150,5:6:3:150,4:3:3:150
```

### Step 4: Run the script and verify

```bash
python3 resources/generate_image_data.py "$GIF_PATH" "$CONFIG"
```

Output filename is `icon_<name>.h` (derived from GIF filename). Ensure no spaces in the filename — rename if needed.

The script includes a built-in verification step that:
1. Reconstructs each frame from the generated indexed data (full frames + overlay compositing)
2. Compares reconstructed frames against the originals
3. Reports pixel differences (quantization errors are expected, structural errors are not)
4. Checks that white highlight pixels are preserved (not lost to background replacement)
5. Saves diff images to `resources/verify/` for visual inspection

Check the verification output:
- "Verification PASSED" means all frames are structurally correct
- If white highlights are lost, the background replacement (flood fill) may need adjustment
- Quantization differences (a few pixels per frame) are normal with 16-color palette
- Total bytes must fit in flash (32KB shared with firmware, ~22KB used by firmware)
- **Flash超過時**: diff matrixでベースとの差分が小さい登場フレームをoverlayに変更する。full frame (2048B) → overlay (~30-200B) で大幅に節約できる

### Step 5: Update firmware include

In `firmware/puchi_pix/puchi_pix.ino`, change the icon include at the top of the file:

```cpp
#include "icon_<name>.h"
```

This is the ONLY change needed in the .ino — the Frame struct-based animation system handles everything else automatically.

### Step 6: Build and test

Remind the user to:
1. Open `firmware/puchi_pix/puchi_pix.ino` in Arduino IDE
2. Build and upload
3. BOOT+RST required for upload (nBOOT_SEL must be 0)

## Data Format

- All frames share a single **16-color palette** (RGB565)
- Pixels are **4-bit packed** (2 pixels per byte, high nibble first)
- Each frame is a `Frame` struct (defined in `frame.h`) with type, next, ref, region, duration_ms, and data pointer
- Full frames: complete IMG_W x IMG_H pixel data
- Overlay frames: only the diff region, drawn on top of the reference frame
- Background color: auto-detected (most common color in base frame) and replaced with black
- Background replacement uses **flood fill from edges** to preserve interior pixels of the same color (e.g. white highlights in eyes)
- Each non-base frame is automatically stored as full frame or overlay based on diff size vs base (threshold: 50% of full frame)
