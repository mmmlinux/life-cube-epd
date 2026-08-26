# life-cube-epd

Conway's Game of Life on a bouncing 3D cube for the LilyGo T5 4.7" e-paper panel (ESP32-WROVER, ED047TC1 960×540).

**Repo:** https://github.com/mmmlinux/life-cube-epd  
**Hardware:** LilyGo T5 4.7" e-paper devboard; connects at `/dev/cu.usbserial-*` (see `esptool.py chip_id` to identify the board)  
**Build:** `arduino-cli compile -u -p /dev/cu.usbserial-51850159751 --fqbn 'esp32:esp32:esp32:PSRAM=disabled,FlashSize=16M,PartitionScheme=huge_app,FlashFreq=80,CPUFreq=240,UploadSpeed=460800' --libraries libraries life_cube_epd`  
**License:** GPLv3 (the vendored LilyGo-EPD47 driver is GPLv3; see README Licensing section)

## Critical constraints

- **Memory:** 320 KB internal SRAM, PSRAM disabled. Three planes (fb, shown, debt) = 194.4 KB. Largest contiguous block after allocation ≈ 50 KB. No room for a fourth 64.8 KB buffer.
- **Speed vs contrast trade-off:** Contrast comes from *pass count*, not dwell. 1 pass = 40.6 fps (cells go grey), 2 passes = 20.1 fps (target), 4 passes ≈ 11 fps (darker blacks). Dwell below ~90 µs is free.
- **Panel is darken-only by default:** `epd_draw_frame_1bit()` ignores its `DrawMode_t` parameter and always emits `0b01` (darken) codes. Getting white back needs a custom LUT emitting `0b10` (lighten). `EpdFast` does this; the library does not.

## Key findings (all verified on hardware)

1. **Both drive directions fit in one waveform pass.** The 2-bit drive code is per-pixel, so `0b01` and `0b10` coexist in the same waveform. Nothing in LilyGo-EPD47 does this by default; it's used to cut the push time in half.

2. **White is asymmetric and not one level.** A pixel needs more drive to return to clean white than to go black. At low pass counts, just-vacated pixels are left short of white. Once the "what's shown" buffer records them as white they're never driven again — a grey trail. Fix: relight them one more time next frame (free, since that row is being driven anyway). BUT: white is not a single level on this panel; keep driving it whiter and it reaches a rail. The relit tracks would outrun the background, which reads as "the background went grey." Solution: the clear ends with extra lighten passes to saturate the background to the same rail.

3. **The I2S driver matters enormously.** LilyGo-EPD47 ships with an `esp_lcd` path (91 µs/row) and an untested register I2S path (43 µs/row) that upstream compiles out. Enabling it requires five IDF-5 renames; the trap is that `rtc_clk_apll_enable()` was reordered — IDF 5 needs `periph_rtc_apll_acquire()` *before* `rtc_clk_apll_coeff_set()`, and the wrong order hangs in `epd_base_init()` rather than failing to compile.

4. **The clear is a Bayer dissolve.** A hard black/white flash in the middle of an otherwise calm animation is startling. Making it slower doesn't mean raising the dwell (which just delays the same flash) — it means staggering *which* pixels are driven (8×8 ordered dither threshold), so pixels cross to black at different times and the flash becomes a wipe. This also lets each band of pixels be driven hard enough to go properly black.

## The stack

- **`life_cube_epd/life_cube_epd.ino`:** Main sketch. 1536 cells (6 cube faces, 16×16 each). Rotating on three incommensurate axes, bouncing, reseed on stagnation or button press. Runs at 20.1 fps.
  - Motion: same drift velocity always, walls suspend per-axis when off-panel (so a diagonal exit doesn't trap the cube on re-entry). Exit/wipe/enter sequence uses ordinary animation speed.
  - Button support: all three panel buttons (GPIO 34/35/39) are active-low with external pull-ups; probed at boot and dropped if they don't hold high. Presses are debounced 40 ms, edge-triggered.

- **`libraries/EpdFast/`:** Custom driver wrapping LilyGo-EPD47's low-level API. Main entry point: `epd_push_diff(y0, rows, cur, prev, relight, dwell)`. Carries both `0b01` and `0b10` drive codes per pixel in a single waveform pass.
  - `epd_wipe(hold, dark_settle, light_settle, dwell)`: Bayer dissolve to black and back, then settling passes. The dissolve is 64 dither levels × hold passes per level; `hold=2` is ~5.2 s.

- **`libraries/LilyGo-EPD47/`:** Vendored, patched (see file headers for `LOCAL PATCH` markers). Do not replace with a fresh clone — the patches will be lost.
  - Patched files: `src/ed047tc1.c`, `src/i2s_data_bus.c`, `src/rmt_pulse.c` (IDF-5 porting, register I2S enabling, APLL ordering).
  - Flat copy of the library; when building, Arduino IDE needs `--libraries libraries` to find it.

- **`bench/`:** Three test sketches, each verifiable on hardware:
  - `epd_smoke`: Bare driver test; confirms panel boots and `epd_clear` works.
  - `epd_bench`: Discriminator card. Two 7-column grids (darken onto white, lighten out of black), each column a different (passes, dwell) combo. Verifies contrast vs pass count, and that both directions can coexist. Timings are deterministic and reproducible to the millisecond.
  - `epd_soak`: 300-frame soak of a moving rotating square at 4 passes/frame. Band and row counts are reproducible. The soak's measured 103.4 ms push regressed from 99.1 ms after the relight changes; I investigated (hoisted the null-check out of the inner loop, made no difference) and reverted the hoist rather than commit a claim I couldn't verify.

## Measurement

Serial output at 115200 baud. On each reseed, a line reports:
```
gen=<generation> live=<cell_count> reason=<died-out|cycle-detected|max-generations|button> | <frames> frames, <fps> fps (render <ms> ms, push <ms> ms) | exit <s>s wipe <s>s enter <s>s
```

- `fps` is *only* on-panel frames, excluding transit (exit + wipe + enter). Transit frames are discarded from the average.
- `reason=button` means a user press ended the run.
- Exit/enter/wipe times come from `millis()`, so they include measurement imprecision and animation overhead. Expect ±50 ms.

## Working in this repo

- **Do not edit the vendored driver unless absolutely necessary.** If you must patch a file in `libraries/LilyGo-EPD47/src/`, add a `/* LOCAL PATCH: ... */` comment near the change and update the file's header notice (see `ed047tc1.c` for the format). The patches are load-bearing; a fresh clone will silently revert them.

- **The relight plane is optional.** If `s_debt` allocation fails, the sketch logs a warning and carries on without it. The trails will linger longer, but the animation is correct. This is by design — graceful degradation on smaller boards.

- **Frame rate varies by position.** A cube near a panel edge has its diff band clipped, so frames are cheaper. A 949-frame window measured 25.0 fps; a 3281-frame window measured 20.1 fps. Long windows are more representative.

- **The clear is slow on purpose.** The 5.2 s dissolve is not tunable down without losing the effect. If you want a faster reseed, increase `GENERATION_INTERVAL_MS` or lower `MAX_GENERATIONS_NO_LOOP`, not the clear.

- **Git:** All three original commits are on `main` and attributed to `mmmlinux`. The identity was rewritten after the initial port; if you see commit SHAs in old notes they no longer match.

## If something breaks

- **Panel stays grey or blank after reseeding:** The clear either failed or the panel is in an inconsistent state. Try a manual `epd_clear()` call or a power cycle.
- **Frame rate drops suddenly:** Check if `s_free_flight` got stuck in a transition; a timeout guard should catch it, but it's worth testing the exit/enter logic with a button press during different cube positions.
- **Trails appear again:** The relight plane likely failed to allocate. Check the boot log for `warning: no room for the relight plane`. The animation is still correct; the trails will go away on the next reseed.
- **Compiler errors on a fresh clone:** Confirm `--libraries libraries` is in the command. Without it, Arduino IDE won't find the vendored driver.

## References

- **ESP32 Arduino core 3.x / IDF-5 changes:** The APLL trap and register I2S path are both IDF-5 specifics. Older cores (pre-3.x) use IDF-4 and have a different API.
- **Waveform mechanics:** The LilyGo-EPD47 README mentions `calc_epd_input_1bpp()` and `epd_draw_frame_1bit()`. Read those if you need to understand the drive-code mechanics.
- **Panel specs:** ED047TC1 is 960×540, 4.7", 100 ppi. Ghosting is acceptable; perfect whiteness on budget hardware is not.
