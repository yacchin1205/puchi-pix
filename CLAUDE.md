# Puchi-Pix — Claude Code notes

- Project skills live in `.claude/skills/`. Check them before starting a
  repo-specific workflow, and keep them updated when the workflow changes.
  - `generate-icon-data`: convert a pixel art GIF into `icon_<name>.h`
    for the STM32 firmware (frame analysis, config, generation, verify).
- Icon headers are self-documenting: the `// Run:` line at the top of each
  `firmware/puchi_pix/icon_*.h` reproduces the exact generator invocation.
