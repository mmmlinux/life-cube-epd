// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * epd_bench - how much drive does a pixel on this panel actually need?
 *
 * The first run of this bench left two things unresolved: dwell (per-row gate
 * time) barely mattered while pass count mattered a lot. Two explanations fit:
 *
 *   (a) the panel responds to the number of waveform passes, and drive time per
 *       pass is nearly irrelevant, or
 *   (b) every row was already gated for the whole ~121 us bus-bound row period
 *       regardless of the dwell we asked for, so all the columns got identical
 *       drive and the sweep never varied anything.
 *
 * They point opposite ways: under (a) a faster bus buys a proportionally faster
 * animation, under (b) it buys almost nothing and the changed area has to shrink
 * instead. So this card puts equal-nominal-drive recipes side by side with
 * opposite structure, and prints what each one cost in wall time.
 *
 * LilyGo T5 4.7" (ESP32-WROVER, ED047TC1 960x540).
 */

#include <Arduino.h>
#include <esp_heap_caps.h>
#include "epd_fast.h"
#include "raster1.h"

// Heap, not .bss: two full-screen 1bpp buffers overflow the static DRAM segment
// even though there is plenty of internal RAM for them at runtime.
static uint8_t *s_fb;      // the image we want on the panel
static uint8_t *s_shown;   // what the panel is currently showing

// (passes, dwell ticks). Chosen so that neighbours isolate one variable:
// 0 vs 1 is the same nominal drive split two ways, 2/3 push dwell far past the
// bus floor, 4/5 scale passes, 6 is the best recipe from the previous card.
typedef struct { int passes; uint32_t dwell; } Recipe;
static const Recipe RECIPES[] = {
  { 4,   100 },
  { 1,   400 },
  { 1,  4000 },
  { 1,  8000 },
  { 8,   100 },
  { 16,  100 },
  { 4,  1000 },
};
#define NRECIPES ((int)(sizeof(RECIPES) / sizeof(RECIPES[0])))

#define PATCH_W  120
#define PATCH_GAP 16
#define ROW_A_Y   60
#define ROW_A_H  160
#define TALLY_Y  245
#define TALLY     10
#define TALLY_PITCH 14
#define BAND_Y   285
#define BAND_H   190
#define ROW_B_Y  300
#define ROW_B_H  160

static int patch_x(int col) {
  int total = NRECIPES * PATCH_W + (NRECIPES - 1) * PATCH_GAP;
  return (EPD_WIDTH - total) / 2 + col * (PATCH_W + PATCH_GAP);
}

// --------------------------------------------------------------------------
// Drive a rectangle to black or white and report what it cost.
// --------------------------------------------------------------------------
static uint32_t drive(int x, int y, int w, int h, EpdDir dir,
                      int passes, uint32_t dwell) {
  r1_rect(s_fb, x, y, w, h, dir == EPD_DARKEN);
  uint32_t t0 = micros();
  for (int p = 0; p < passes; p++)
    epd_push_diff(y, h, s_fb + (size_t)y * EPD_ROW_BYTES,
                  s_shown + (size_t)y * EPD_ROW_BYTES, NULL, dwell);
  uint32_t dt = micros() - t0;
  r1_rect(s_shown, x, y, w, h, dir == EPD_DARKEN);
  return dt;
}

// Anything that just has to be legible gets driven hard.
#define SOLID_PASSES 8
#define SOLID_DWELL  1000

// The library's epd_clear() is 32 full-screen pushes and takes ~1.9 s. Our own
// pusher can flush the panel to white with a handful of lighten passes: assume
// everything is black, so every pixel gets driven.
static void fast_clear(int cycles, int dark, int light, uint32_t dwell) {
  epd_deghost(cycles, dark, light, dwell);
  r1_clear(s_fb, false);
  r1_clear(s_shown, false);
}

static void discriminator_card(void) {
  uint32_t t0 = millis();
  fast_clear(1, 3, 7, 1000);
  Serial.printf("fast_clear %lu ms\n", (unsigned long)(millis() - t0));

  // Column tallies, so the user can index the patches without a font.
  for (int c = 0; c < NRECIPES; c++)
    for (int k = 0; k <= c; k++)
      drive(patch_x(c) + k * TALLY_PITCH, TALLY_Y, TALLY, TALLY,
            EPD_DARKEN, SOLID_PASSES, SOLID_DWELL);

  // Row A: darken onto white.
  Serial.println("\nrow A - darken onto white");
  for (int c = 0; c < NRECIPES; c++) {
    uint32_t dt = drive(patch_x(c), ROW_A_Y, PATCH_W, ROW_A_H,
                        EPD_DARKEN, RECIPES[c].passes, RECIPES[c].dwell);
    Serial.printf("  col %d: %2d pass x %5lu ticks  ->  %7.2f ms\n",
                  c + 1, RECIPES[c].passes, (unsigned long)RECIPES[c].dwell, dt / 1000.0);
  }

  // Row B: lighten out of a solid black band.
  drive(0, BAND_Y, EPD_WIDTH, BAND_H, EPD_DARKEN, SOLID_PASSES, SOLID_DWELL);
  Serial.println("\nrow B - lighten out of black");
  for (int c = 0; c < NRECIPES; c++) {
    uint32_t dt = drive(patch_x(c), ROW_B_Y, PATCH_W, ROW_B_H,
                        EPD_LIGHTEN, RECIPES[c].passes, RECIPES[c].dwell);
    Serial.printf("  col %d: %2d pass x %5lu ticks  ->  %7.2f ms\n",
                  c + 1, RECIPES[c].passes, (unsigned long)RECIPES[c].dwell, dt / 1000.0);
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\nepd_bench (discriminator card)");

  epd_base_init(EPD_WIDTH);   // not epd_init(): that also mallocs a 64 KB
                              // grayscale LUT and a ~30 KB queue we never use
  epd_fast_init();

  s_fb    = (uint8_t *)heap_caps_malloc(R1_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s_shown = (uint8_t *)heap_caps_malloc(R1_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!s_fb || !s_shown) { Serial.println("FATAL: framebuffer alloc failed"); while (1) delay(1000); }

  epd_poweron();
  discriminator_card();
  epd_poweroff();

  Serial.println("\ncolumns 1-7 (tally marks between the rows):");
  for (int c = 0; c < NRECIPES; c++)
    Serial.printf("  %d: %2d pass x %5lu ticks\n", c + 1, RECIPES[c].passes, (unsigned long)RECIPES[c].dwell);
  Serial.println("done");
}

void loop() { delay(1000); }
