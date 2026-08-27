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
  - Bouncing uses the cube's **actual** projected silhouette, not a bounding radius. `update_motion()` builds the frame's rotation matrix into `s_R`, `cube_extent()` takes the max over the 8 projected corners, and the four `s_wall_*` limits follow. `render_cube()` reads the same `s_R`, so the trig is paid once and the bounce and the render always agree. Costs **4.78 µs/frame**, measured — 0.01% of a ~50 ms frame.
  - `CUBE_RADIUS` (0.4804 × `PROJ_SCALE` = 149 px, i.e. √39/13, the worst case over all orientations) survives for `OFFSTAGE` — where worst case is correct, since a face-on exit could rotate corner-on and reappear after the wipe — and to seed `s_wall_*` before the first frame.
  - Button support: all three panel buttons (GPIO 34/35/39) are active-low with external pull-ups; probed at boot and dropped if they don't hold high. Presses are debounced 40 ms, edge-triggered. Buttons 1 and 2 end the run and reseed; button 3 (GPIO 39, `REFRESH_BUTTON`) runs an immediate de-ghost instead and leaves the run going.
  - The reseed cycle is a three-phase state machine (`RunPhase`: `RUN_ON_PANEL` / `RUN_LEAVING` / `RUN_ARRIVING`), driven entirely from `loop()`. There are no nested frame loops — `loop()` runs once per frame in every phase and `animate_frame()` is called from exactly one place. The phase decides only whether the walls are live and what happens when the cube reaches its mark; `update_motion()` reads `s_phase == RUN_ARRIVING` to know whether a suspended axis may reclaim its wall.
  - Because `loop()` always runs, buttons are polled there once a frame and a refresh needs no latch. The only windows where a press is missed are the clears, which block inside the library.
  - A run's figures are snapshotted into `s_run` when it ends and printed once the next cube has arrived, so the transit timings on the line belong to the transit that just happened. `s_run.pending` is false at boot, where there is an arrival but no run behind it to report.

- **`libraries/EpdFast/`:** Custom driver wrapping LilyGo-EPD47's low-level API. Main entry point: `epd_push_diff(y0, rows, cur, prev, relight, dwell)`. Carries both `0b01` and `0b10` drive codes per pixel in a single waveform pass.
  - `epd_wipe(hold, dark_settle, light_settle, dwell)`: Bayer dissolve to black and back, then settling passes. The dissolve is 64 dither levels × hold passes per level; `hold=2` is ~5.2 s.
  - `epd_deghost(cycles, dark_passes, light_passes, dwell)`: the same swing done as floods rather than a dissolve, so it reads as a flash. `(2, 4, 10)` is the button's de-ghost, measured at 541 ms.
  - `epd_white_dissolve(hold, light_settle, dwell)`: the lighten half of `epd_wipe()` on its own — Bayer dissolve towards white, then settling floods, no darken phase. This is the reseed clear at `WHITE_ONLY_REFRESH (1)`; `(1, 12)` measures 1.5 s. **A flood cannot be slowed** (passes past the white rail do nothing), so staggering which pixels are driven is the only control over how gradual it looks.
  - **`hold` wants to stay low here, unlike in `epd_wipe()`.** That one staggers a black crossing — a huge contrast change. This one staggers a tiny one, so what is mostly on show mid-dissolve is the 8×8 dither texture itself, and holding longer means more exposure to it. If the fade looks patterned, raise `light_settle`, not `hold`.

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

Pressing the refresh button logs its own line, wherever it happened:
```
refresh: <ms> ms at gen=<generation> live=<cell_count>
```

- `render` covers `update_motion()` **and** `render_cube()`. The trig moved into the former with the exact-bounds change, and timing only the latter would have shown that work as a saving rather than a cost.
- `render` cannot resolve small changes: it swings ~800 µs (3.1–3.9 ms observed) with live-cell count alone. Anything smaller needs a direct micro-benchmark — that is how the 4.78 µs figure above was taken.
- `fps` is *only* on-panel frames, excluding transit (exit + wipe + enter). The counters are zeroed on every `RUN_ARRIVING` → `RUN_ON_PANEL` transition, boot's arrival included, so the first reported window is on-panel too. (Before the state machine, boot's ~9 s of drifting on landed in the first window and made its `fps` read high.)
- `reason=button` means a button 1/2 press ended the run. Button 3 never ends a run; it logs a `refresh:` line instead.
- `exit` and `enter` are wall clock across the transit phases, measured from `s_phase_t0`, so a refresh taken mid-transit is included in them. Deliberate — the figure is what the transit actually took.
- `enter` is set by which axis has to come back into range, and since the exact-bounds change it varies a little within that. The cube covers `s_wall_lo + OFFSTAGE` to arrive, and `s_wall_lo` now depends on the silhouette at the moment the axis reclaims its wall — 107.8 px face-on to 153.4 px corner-on. So a horizontal arrival is **7.8–9.1 s** and a vertical one **16.6–19.4 s**. Measured 8.8 s and 9.0 s horizontal; 19.5 s vertical was recorded before the walls became dynamic. Side arrivals dominate because the horizontal drift is twice the vertical.
- `exit` has no such figure — it depends where the cube was when the run ended (9.6–27.2 s observed).
- Exit/enter/wipe times come from `millis()`, so they include measurement imprecision and animation overhead. Expect ±50 ms — the phase is only tested once a frame, which is where the ~60 ms above the arithmetic comes from.

## Working in this repo

- **Do not edit the vendored driver unless absolutely necessary.** If you must patch a file in `libraries/LilyGo-EPD47/src/`, add a `/* LOCAL PATCH: ... */` comment near the change and update the file's header notice (see `ed047tc1.c` for the format). The patches are load-bearing; a fresh clone will silently revert them.

- **The relight plane is optional.** If `s_debt` allocation fails, the sketch logs a warning and carries on without it. The trails will linger longer, but the animation is correct. This is by design — graceful degradation on smaller boards.

- **Frame rate varies by position.** A cube near a panel edge has its diff band clipped, so frames are cheaper. A 949-frame window measured 25.0 fps; a 3281-frame window measured 20.1 fps. Long windows are more representative.

- **`WHITE_ONLY_REFRESH` decides what the reseed clear is.** At `1` (current) the reseed is a white-only dissolve, 1.5 s instead of 5.2 s, and the full black-and-back wipe is used only at boot. At `0` the dissolve comes back for reseeds too. Boot always takes the full black-and-back wipe whatever the flag says: it has no idea what the previous sketch left on the panel.

- **With `WHITE_ONLY_REFRESH (1)`, nothing during a run swings the panel fully black,** so ghosting accumulates across reseeds with no bounded recovery. The refresh button is the counterweight and is the only full reset left. The two are a pair; changing one without thinking about the other is how the panel ends up grey.

- **The dissolve is slow on purpose.** Its 5.2 s is not tunable down without losing the effect - see finding 4. If you want a faster reseed, raise `GENERATION_INTERVAL_MS` or lower `MAX_GENERATIONS_NO_LOOP`, or set `WHITE_ONLY_REFRESH`; do not shorten the dissolve.

- **A refresh cannot be taken during a clear.** `epd_wipe()` and `epd_deghost()` block inside the library with no callback, so the boot wipe (5.2 s) and the reseed dissolve (1.5 s) are the two windows where a press is missed. Everything else - both transits included - polls every frame.

- **Git:** All three original commits are on `main` and attributed to `mmmlinux`. The identity was rewritten after the initial port; if you see commit SHAs in old notes they no longer match.

## If something breaks

- **Panel stays grey or blank after reseeding:** The clear either failed or the panel is in an inconsistent state. Try a manual `epd_clear()` call or a power cycle.
- **Frame rate drops suddenly:** Check whether `s_phase` is stuck in `RUN_LEAVING` or `RUN_ARRIVING`. `TRANSIT_TIMEOUT_MS` (90 s, measured from `s_phase_t0`) is the backstop and forces the phase on, but a cube that never reaches `cube_offstage()` would sit there until it fires.
- **Trails appear again:** The relight plane likely failed to allocate. Check the boot log for `warning: no room for the relight plane`. The animation is still correct; the trails will go away on the next reseed.
- **Compiler errors on a fresh clone:** Confirm `--libraries libraries` is in the command. Without it, Arduino IDE won't find the vendored driver.

## References

- **ESP32 Arduino core 3.x / IDF-5 changes:** The APLL trap and register I2S path are both IDF-5 specifics. Older cores (pre-3.x) use IDF-4 and have a different API.
- **Waveform mechanics:** The LilyGo-EPD47 README mentions `calc_epd_input_1bpp()` and `epd_draw_frame_1bit()`. Read those if you need to understand the drive-code mechanics.
- **Panel specs:** ED047TC1 is 960×540, 4.7", 100 ppi. Ghosting is acceptable; perfect whiteness on budget hardware is not.
