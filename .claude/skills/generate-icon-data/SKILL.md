---
name: generate-icon-data
description: Generate image_data.h for Puchi-Pix from a pixel art GIF animation. Use when the user provides a new GIF icon and wants to convert it to firmware image data.
argument-hint: [gif-path]
allowed-tools: Read, Bash, Glob, Grep, Write, Edit
---

# Generate Icon Data for Puchi-Pix

Convert a pixel art GIF animation into `image_data.h` for the Puchi-Pix firmware.

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

### Step 3: Auto-detect animation overlay region

Compare the base frame with each animation overlay frame to find the bounding box of all pixel differences:

```python
from PIL import Image
import numpy as np

# Compute union of all diffs between base and overlay frames
combined_diff = np.zeros((img_h, img_w), dtype=bool)
for overlay_frame in overlay_frames:
    diff = np.any(base != overlay_frame, axis=2)
    combined_diff |= diff

ys, xs = np.where(combined_diff)
# Add 1px margin, ensure even width for 4-bit packing
OVERLAY_X = max(0, xs.min() - 1)
OVERLAY_Y = max(0, ys.min() - 1)
OVERLAY_W = min(img_w, xs.max() + 2) - OVERLAY_X
OVERLAY_H = min(img_h, ys.max() + 2) - OVERLAY_Y
if OVERLAY_W % 2 != 0:
    OVERLAY_W += 1
```

Show the detected region to the user (scaled up with nearest-neighbor) for confirmation.

### Step 4: Update generate_image_data.py

Update `resources/generate_image_data.py` with:
- New overlay region coordinates
- New ENTRANCE_FRAMES count
- New frame assignments (number of overlay frames may differ)
- The script should handle variable numbers of overlay frames

The script must generate:
- Entrance frame data (4-bit packed, full frame)
- Base frame data (4-bit packed, full frame)
- Overlay frame data for each animation frame (4-bit packed, overlay region only)
- Shared 16-color palette (RGB565)

### Step 5: Run the script and verify

```bash
python3 resources/generate_image_data.py "$GIF_PATH"
```

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

### Step 6: Update firmware if needed

In `firmware/puchi_pix/puchi_pix.ino`, verify:
- `IMG_FRAMES` matches the overlay frame count
- Animation timing constants are appropriate
- The overlay drawing code handles the correct number of frames

### Step 7: Build and test

Remind the user to:
1. Open `firmware/puchi_pix/puchi_pix.ino` in Arduino IDE
2. Build and upload
3. BOOT+RST required for upload (nBOOT_SEL must be 0)

## Data Format

- All frames share a single **16-color palette** (RGB565)
- Pixels are **4-bit packed** (2 pixels per byte, high nibble first)
- Entrance frames: full 64x64
- Base frame: full 64x64
- Overlay frames: only the detected region, drawn on top of base frame
- Background color: auto-detected (most common color in base frame) and replaced with black
- Background replacement uses **flood fill from edges** to preserve interior pixels of the same color (e.g. white highlights in eyes)
- Each non-base frame is automatically stored as full frame or overlay based on diff size vs base (threshold: 50% of full frame)
