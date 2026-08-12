---
name: generate-icon-data
description: Generate icon header for Puchi-Pix from a pixel art GIF animation. Use when the user provides a new GIF icon and wants to convert it to firmware image data.
argument-hint: [gif-path]
allowed-tools: Read, Bash, Glob, Grep, Write, Edit
---

# Generate Icon Data for Puchi-Pix

Convert a pixel art GIF animation into `icon_<name>.h` for the STM32 firmware
(`firmware/puchi_pix`). Existing icon headers are self-documenting: the
`// Run:` line at the top reproduces the exact generator invocation — start
from the closest existing icon when in doubt.

## Prerequisites

- Python 3 with PIL/Pillow, NumPy, SciPy (background flood fill)
- GIF filename must have no spaces — copy/rename first (the output name
  `icon_<name>.h` derives from it)

## Process

### Step 1: Analyze the GIF

With PIL, report frame count, size, per-frame durations, identical-frame
pairs, and pixel-diff counts between frames. Render a contact sheet and LOOK
at it. Typical dotpict GIFs are symmetric: walk cycle (ping-pong pairs),
turn-in frames, main pose (often longer durations), turn-out = reversed
turn-in (identical frames → shared data arrays).

For reference selection, compute the bounding-box diff size between every
unique frame pair — also with vertical shifts of ±1..2 px (walk bounce is
often exactly 1 px) — and choose refs like a minimum spanning tree rooted at
the single full frame.

### Step 2: Decide frame roles with the user

The firmware plays up to five segments (state machine in `puchi_pix.ino`):
walk-in → intro (one-shot) → **main loop, repeating while handling is
detected** → outro (one-shot) → walk-out (reuses the walk frames), then
waits offscreen (dimmed, sleeps after 10 s) until handling is detected again.

Confirm with the user which GIF frames map to walk / intro / main / outro.
Intro and outro are optional. Omitting `--walk-seq` entirely produces a
plain-loop icon (`HAS_WALK 0`): frames follow their next chain and dim/sleep
run on the activity timers.

### Step 3: Build the config

```
gif_frame:next[:ref[:duration_ms[:dy[:split]]]]
```

- `ref`: overlay reference (may chain; firmware walks up to 8 hops including
  split helpers). Refs may point to any output frame, order-free.
- `dy`: vertical shift applied to ref content (1 px walk bounce).
- `split=1`: store the diff as two rects (best horizontal or vertical cut)
  instead of one bounding box; the second rect becomes a hidden helper frame
  (16 B table cost). Errors out if no cut beats the single rect.
- Walk frames' `next` is unused (the walk steps through `WALK_SEQ`); their
  GIF durations are also unused (fixed 150 ms per 4 px step).
- Reuse the same gif frame index for identical frames — data arrays dedupe
  automatically, so outro twins of intro frames cost only table entries.

Role flags: `--walk-seq i,i,...` (ping-pong indices), `--main-start` (its
next chain must cycle back to itself), `--intro-start` (chain must reach
main-start), `--outro-start` (chain ends when next == WALK_SEQ[0]).
All chains are validated; the generator fails fast on broken ones.

Palette correction for the OLED: gamma `2.2`, sat_power `0.7`
(user-approved; pastel art washes out otherwise).

### Step 4: Fit the flash budget

Total flash is 32 KB. Firmware code without icon data is ≈ 27.2 KB
(measure: build with `icon_original.h`, subtract its ~3.2 KB), leaving
≈ 5.5 KB for icon data. `arduino-cli` prints the sketch size; overflow shows
as a link error with the exact byte count.

Reduction order (lossless first):

1. MST-based refs incl. `dy` shifts
2. `--crop <x>:<w>` — crop all frames to the content columns (validated;
   64→48 px saves 512 B on the full frame)
3. `split=1` on large overlays (row+column cuts are both searched)
4. Frame dropping — lossy, ask the user first

### Step 5: Run and verify

```bash
python3 resources/generate_image_data.py <gif> "<config>" 2.2 0.7 \
  --crop 10:48 --walk-seq 0,1,2,3,2,1 --main-start 9 --intro-start 4 --outro-start 10
```

Check "Verification PASSED" (full reconstruction of every visible frame),
white-highlight preservation, and view `resources/verify/` reconstructed
vs original side by side — quantization to 16 colors should be barely
visible; structural errors are not acceptable.

### Step 6: Switch include, build, device-test

```cpp
#include "icon_<name>.h"   // in firmware/puchi_pix/puchi_pix.ino
```

```bash
arduino-cli compile --fqbn "STMicroelectronics:stm32:GenG0:pnum=GENERIC_G030F6PX,xserial=none" firmware/puchi_pix
```

Upload needs BOOT+RST (nBOOT_SEL=0, see README). Do not commit before the
user confirms on the device. Note `.gitignore` keeps `icon_*.h` local except
the tracked sample `icon_original.h` — the committed include should stay
`icon_original.h`; art icons are switched locally.

## Data Format

- Single 16-color RGB565 palette shared by all frames; 4-bit packed pixels
  (2 px/byte, high nibble first)
- `Frame` struct (`frame.h`): type (full/overlay), next, ref, region,
  refDy, duration_ms, data pointer; `FRAME_NONE` marks absent roles
- Split helper frames sit after the visible frames; the visible rect chains
  to the helper (refDy moves onto the last hop so the shift applies once)
- Background: most common color of the base frame, flood-filled from the
  edges to black (interior same-color pixels like eye highlights survive)
