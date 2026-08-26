# Game of Life on a bouncing 3D cube — LilyGo T5 4.7" e-paper

Conway's Game of Life running on the *surface* of a 3D cube that tumbles on all
three axes and bounces around the screen, on the
[LilyGo T5 4.7"](https://github.com/Xinyuan-LilyGO/LilyGo-EPD47) (ESP32-WROVER
revision — ED047TC1, 960×540, ~235 DPI).

A port of [the ESP32-C5-LCD-1.47 version](../life-cube). The Life topology, the
tumble and the reseed logic are the same. Everything about how a frame reaches
the glass is different, because e-paper is not a framebuffer you can blit to
sixty times a second — except that, in the end, it sort of is.

**It runs at 20 fps.**

## What it does

- **Life on a closed surface.** The six 16×16 faces are one connected 1536-cell
  world, not six independent grids. Patterns crawl over face edges and around
  corners instead of dying against a frame — the cube has no boundary. The 24
  cells at the cube's corners correctly have 7 neighbours; every other cell has 8.
- **Tumbles and bounces.** Perspective-projected, spinning on three deliberately
  incommensurate axes so the orientation never repeats, drifting DVD-logo style
  and bouncing off the screen edges.
- **Runs forever unattended.** Reseeds when the population dies out, settles into
  a repeating cycle (rolling hash of the last 12 generations), or hits a
  generation cap.
- **Any of the three panel buttons ends the run**, on demand, running exactly
  the same exit / wipe / entrance sequence a natural reseed does.
- **Reseeds are staged as a scene change.** The spent cube drifts out of frame,
  the empty panel dissolves to black and back over five seconds, and the next
  cube drifts in from the opposite side. The wipe never happens over the top of
  a cube, so it reads as scenery rather than as a fault — and it doubles as the
  panel's particle reset.

## Getting a real frame rate out of this panel

This is most of the work, so it is worth writing down. Every number below was
measured on the board, not estimated.

### The two update paths LilyGo-EPD47 ships are both unusable

`epd_draw_grayscale_image()` runs 15 waveform frames, each spawning *and
deleting* two FreeRTOS tasks with a `vTaskDelay(5)` between them: ~500 ms.

`epd_draw_frame_1bit()` is a single pass and takes a `DrawMode_t`, which looks
like what you want — but its row expander `calc_epd_input_1bpp()` accepts that
argument and never reads it. `lut_1bpp` maps a set bit to the panel's `0b01`
"darken" code unconditionally. **It can only ever put ink down, never take it
away.** There is no white in it.

So this sketch drives the panel itself through the low-level row API in
`ed047tc1.h` (`epd_start_frame` / `epd_output_row` / `epd_skip` / …), with a
second expansion LUT that emits `0b10`. That is `libraries/EpdFast/`.

### Both directions ride in one pass

The panel takes two bits per pixel per pass — `00` leave alone, `01` darken,
`10` lighten — and those bits are **per pixel**. So a single pass can move some
pixels to black and others to white simultaneously.

Nothing in LilyGo-EPD47 does this. `reset_lut()` fills the entire lookup with
`0x55` (all-darken) or `0xAA` (all-lighten) and `update_LUT()` only ever clears
bits to `00`, so a greyscale frame is one direction or the other. The hardware
has no such restriction. Verified on the panel before anything was built on it —
it halves the cost of a differential update.

### Contrast comes from pass count, not drive time

The unintuitive result, and the one that decided the design. A test card of
patches at varying `(passes × dwell)` showed:

| recipe | same nominal drive? | result |
|---|---|---|
| 4 passes × 100 ticks | yes | **clearly blacker** |
| 1 pass × 400 ticks | yes | weaker |
| 1 pass × 8000 ticks | 20× the drive | still weaker than 4 short passes |

Per-row dwell below ~90 µs is *free* — the bus is the limit, not the waveform —
so passes are pinned at the minimum dwell and `PASSES` is the only quality knob.

### Enabling the register I2S path — the 2.8× win

`i2s_data_bus.c` contains two implementations behind `#define USER_I2S_REG`.
Upstream builds the `esp_lcd_panel_io` one, which costs **~91 µs per row** —
almost all of it software overhead, since the 248-byte DMA at 10 MHz is only
25 µs. The direct-register path (epdiy's original, APLL at 120 MHz with real
double buffering) was compiled out because it does not build on ESP32 Arduino
core 3.x.

Porting it is five mechanical IDF-5 renames, all in
`libraries/LilyGo-EPD47/src/`, each marked `LOCAL PATCH`:

| what | fix |
|---|---|
| `ESP_IDF_VERSION_MAJOR` tested before any IDF header defines it, so `rmt_pulse.c` took the legacy include branch and the IDF-5 code branch | include `<esp_idf_version.h>` first |
| `gpio_matrix_out()` | `esp_rom_gpio_connect_out_signal()` |
| `I2S1O_*_IDX`, `GPIO_PIN_MUX_REG` | include `<soc/gpio_sig_map.h>`, `<soc/gpio_periph.h>` |
| `start_pulse_pin` commented out | uncomment — the register path's ISR needs it |
| `rtc_clk_apll_enable(1,0,0,8,0)` | split into `periph_rtc_apll_acquire()` + `rtc_clk_apll_coeff_set(0,0,0,8)` |

That last one is the trap: IDF 5 requires the APLL to be **powered before** its
coefficients are written. Getting the order wrong doesn't fail to compile, it
hangs inside `epd_base_init()`.

Result: **43 µs per driven row, 24 µs per skipped row** — a full-screen pass
drops from 65 ms to 23 ms.

### Measured cost

| | esp_lcd path | register path |
|---|---|---|
| full-screen 1-bit pass | 65 ms | **23 ms** |
| per driven row | 91 µs | **43 µs** |
| clear the panel | 1899 ms (`epd_clear`) | **116 ms** (`epd_deghost`, 6 passes) |

The shipped clear is `epd_wipe()`, which is deliberately *slower* than it needs
to be — 5.2 s measured — for the reasons below.

Frame at the shipped settings: **~3.3 ms render + ~46.5 ms push = 20.1 fps**,
reported on every reseed line and confirmed on two independent multi-thousand
frame windows. Shorter windows read higher — a 949-frame sample came out at
25.0 fps — because a cube near a panel edge has its diff band clipped and its
frames are cheaper. Transit frames are excluded from the average: an off-panel
cube has an empty diff band and costs almost nothing to draw, and letting tens
of seconds of those in reported a frame rate the animation never ran at. `PASSES` is
the trade — 1 pass measured 40.6 fps (at a slightly larger cube) but the cells
go visibly grey; 4 would put it near 11 fps for blacker blacks. The relight
below costs about 6 ms a frame, not by driving more pixels but by holding a
vacated row in the band one frame longer than the image alone would.

## Design choices the panel forced

- **No greyscale, no dithering.** The panel does 16 levels, but a greyscale
  update is ~500 ms. Worse, ordered dither would re-quantize nearly every pixel
  as a face's brightness drifts, turning a cheap partial update into a
  full-screen one — the diff mask goes from "outline plus some cells" to "the
  whole cube". The design is 1-bit and the colour palette is gone.
- **Line weight instead of shading.** Face orientation is shown by the weight of
  its outline (1.5–4.5 px by Lambert term). It is the only shading channel a
  1-bit panel has that doesn't cost a full-screen redraw.
- **Cells drawn inset.** Each live cell is shrunk to 84% inside its quad, so the
  white gutter reads as the grid. On the colour version the face backdrop did
  that job.
- **Only changed rows are driven,** and the band copied into the "what the panel
  is showing" buffer is *exactly* the band that was driven. Copying in anything
  wider marks pixels as displayed that the panel was never told about, and those
  stay wrong permanently.

### White is not one level, and that is where the trails came from

At two passes the cube left a faint grey trail. The cause is asymmetry: a pixel
needs more drive to come back to clean white than it does to go black, so a
just-vacated pixel is left short — and once the "what the panel is showing"
buffer has recorded it as white, it is never driven again.

The fix costs nothing. Pixels vacated on one frame are handed back on the next
in a **relight** plane and driven towards white a second time. A frame's cost is
the number of rows driven, and a just-vacated row is being driven anyway, so the
extra pixels in the mask are free. (The plane is windowed to the cube's band, not
full-screen: a third 64,800-byte buffer does not fit — the largest contiguous
block of internal RAM left at that point is 47,092 bytes.)

That fixed the grey trail and immediately exposed the other half of the same
fact. Keep driving a white pixel whiter and it *keeps going*, up to a rail. The
relit track was now brighter than the paper around it — which does not look like
a bright track, it looks like the background has gone grey. So the clear ends
with far more lighten passes than darken ones: once the background is at the
rail, there is nothing brighter for a track to move to.

### The buttons need a boot-time probe

The three panel buttons are on GPIO 34, 35 and 39. On the ESP32 those pins are
input-only and have **no internal pull resistors at all** — `INPUT_PULLUP` on
them is silently a no-op. The board's external pull-ups are the only thing
holding them high, so a press reads low.

Which means a pin without a working pull-up floats, and a floating input would
fire the reseed at random. Each pin is probed for 120 ms at boot and only
trusted if it holds solidly high the whole time; anything else is dropped with a
log line. A button held down through boot gets dropped too — a fair trade for
never chasing phantom presses.

Presses are discarded rather than queued during a transit: `button_pressed()` is
only polled from `loop()`, which is not running during an exit or an entrance,
and the cycle a press would ask for is the one already playing.

### Nothing special happens to the cube during a transit

Exits and entrances use the ordinary drift speed, the ordinary spin rates, and
keep stepping Life the whole way. The only thing that changes is that the walls
stop catching the cube, so it carries on the heading it already had and leaves;
it returns on the opposite side unchanged, which reads as having gone round the
back of the panel.

That costs time — at 34 px/s across and 16 px/s down, clearing the panel takes
the better part of half a minute — and that is the point. An accelerated exit
reads as the cube being yanked off; a natural one reads as the drift continuing.
The simulation runs throughout, so it is not dead air.

The walls are released **per axis**, which matters on a diagonal exit: the cube
can come back on with one axis still mid-panel, and that axis has to keep
bouncing normally or it would wander straight off again while the other one is
arriving. An axis takes its wall back the moment it is in range.

One trap worth recording: `animate_frame()` clamps its timestep at 0.5 s, and
after a five-second wipe that clamp would have jumped the cube 160 px on its
first frame back. The frame clock is file-scope so an entrance can reset it.

### Why the clear is a dissolve

A hard black/white flash in the middle of an otherwise calm animation is
startling. Making it *slower* is not a matter of raising the dwell — contrast
here is pass count, so a longer dwell just makes the same flash arrive later.

Instead the clear dissolves: a pixel is driven once a step counter reaches its
slot in an 8×8 Bayer matrix, so pixels cross to black at staggered times and the
flash becomes a wipe. Each level is held for two passes, which stretches it to
5.2 s *and* gives each group enough drive to go properly black as it crosses.

The Bayer cell is 8 pixels wide — exactly one 16-bit group of drive codes — so a
row is one pattern repeated across the line buffer, which also sidesteps the
half-word swap the output stage does: both halves hold the same pattern.

The closing flood passes matter as much as the dissolve. The earliest-dithered
pixels collect dozens of lighten passes on the way through and the last ones get
two, so without enough settling the Bayer pattern prints faintly into the
background.

A 300-frame soak of a moving shape (at 4 passes, before the relight existed) left
no visible ghost trail, so the wipe is only needed at reseeds rather than on a
frame counter.

## Repository layout

```
life-cube-epd/                 <- repo root; run arduino-cli from here
  README.md
  life_cube_epd/
    life_cube_epd.ino          <- the sketch
  libraries/
    EpdFast/                   <- fast 1-bit differential updates + 1bpp rasterizer
    LilyGo-EPD47/              <- vendored panel driver, patched (see above)
  bench/
    epd_smoke/                 <- does the driver build and drive the panel at all
    epd_bench/                 <- test card: how much drive does a pixel need
    epd_soak/                  <- combined-pass proof + 300-frame ghost soak
```

The bench sketches were refactored onto the shared `EpdFast` driver after the
measurements above were taken, and re-run to confirm they still work; their
timings shift slightly with it (`epd_push_flood` does less CPU work per row than
the diff path, so at long dwells it feels the waveform timing rather than hiding
behind it).

`libraries/LilyGo-EPD47/` is a trimmed copy of
[Xinyuan-LilyGO/LilyGo-EPD47](https://github.com/Xinyuan-LilyGO/LilyGo-EPD47)
with `libjpeg/`, the touch driver and the unused fonts removed, plus the patches
above. It is vendored rather than installed because the patches are load-bearing.

## Licensing

**GPLv3**, and not by preference — by obligation.

`libraries/LilyGo-EPD47/` is vendored into this repo and is licensed **GPLv3**,
with no linking exception. LilyGo derived it from
[epdiy](https://github.com/vroland/epdiy), which is LGPLv3, taking the LGPL→GPL
upgrade that LGPLv3 permits. Because this repo redistributes that code — and
because `EpdFast` is written against the library's internals (`epd_start_frame`,
`epd_output_row`, its private row-buffer semantics) rather than at arm's length —
the combined work is GPLv3.

- Original code here (`life_cube_epd/`, `libraries/EpdFast/`, `bench/`):
  GPL-3.0-or-later.
- `libraries/LilyGo-EPD47/`: GPLv3, © LilyGO, patched locally (see above);
  its own LICENSE file is retained.
- `libraries/LilyGo-EPD47/src/zlib/`: zlib license, © Jean-loup Gailly and
  Mark Adler.

If you want a permissively-licensed version, the driver is the thing to
replace — nothing above `EpdFast` is encumbered by anything but the driver.

## Requirements

- **Hardware:** LilyGo T5 4.7" e-paper, **ESP32-WROVER** revision (ESP32-D0WD,
  16 MB flash). Not the newer T5-ePaper-S3 — the pin maps differ and the
  register I2S path is ESP32-specific.
- **`esp32:esp32` Arduino core 3.x** (built against 3.3.11).
- No wiring. No PSRAM needed: the two 1bpp framebuffers are 129,600 bytes
  together and live in internal RAM.

## Build and flash

From the repo root:

```sh
arduino-cli compile -u -p /dev/cu.usbserial-XXXXXXXX \
  --fqbn 'esp32:esp32:esp32:PSRAM=disabled,FlashSize=16M,PartitionScheme=huge_app,FlashFreq=80,CPUFreq=240,UploadSpeed=460800' \
  --libraries libraries life_cube_epd
```

`--libraries libraries` is required — it picks up the patched driver. `PSRAM` is
deliberately `disabled`: the board has it, but the `esp32wrover` FQBN forces
`-mfix-esp32-psram-cache-issue`, a global codegen penalty this chip revision
doesn't need, and nothing here uses PSRAM.

Upload at **460800**, not 921600 — the CP210x on this board fails to sync at the
higher rate.

## Notes

Serial is a real UART here, not USB CDC, so the
`Serial.setTxTimeoutMs(0)` workaround the C5 version needs does not apply. Log
lines are still kept out of the frame loop: a line at 115200 is milliseconds of
blocking, which is a visible hitch at 20 fps.
