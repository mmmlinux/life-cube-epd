// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * epd_fast.h - fast 1-bit differential updates for the ED047TC1 (960x540).
 *
 * LilyGo-EPD47 ships two update paths and neither is usable for animation:
 *
 *   - epd_draw_grayscale_image() runs 15 waveform frames, each spawning and
 *     deleting two FreeRTOS tasks with a vTaskDelay(5) between them. ~500 ms.
 *   - epd_draw_frame_1bit() is one pass and takes a DrawMode_t, but its
 *     expander calc_epd_input_1bpp() never reads that argument: lut_1bpp maps a
 *     set bit to the panel's 0b01 "darken" code unconditionally. It can only
 *     ever put ink down, never take it away.
 *
 * So we drive the panel ourselves through ed047tc1.h. The panel takes two bits
 * per pixel per pass - 0b00 leave alone, 0b01 darken, 0b10 lighten - and those
 * bits are per pixel, so one pass can move some pixels to black and others to
 * white at the same time. The library never does this (reset_lut fills the
 * whole lookup with 0x55 or 0xAA and update_LUT only ever clears bits to 0b00,
 * so a grayscale frame is all-darken or all-lighten), but the hardware has no
 * such restriction, and it halves the cost of a differential update.
 *
 * Measured on an ESP32-WROVER T5 4.7 with LilyGo-EPD47's USER_I2S_REG path
 * enabled: 43 us per driven row, 24 us per skipped row, so a full-screen pass
 * is ~23 ms. Contrast comes from the number of passes rather than from the
 * per-row dwell - four passes at the minimum dwell beat one pass at four times
 * the dwell, and dwell below ~90 us is free because the bus is the limit.
 *
 * Row buffer ownership depends on how the library was built, and the flag is
 * private to i2s_data_bus.c, so we probe for it instead of guessing:
 *
 *   USER_I2S_REG 1 (the direct-register path, what we build) really does have
 *   two line buffers. epd_output_row() starts DMA out of one and flips to the
 *   other, so the CPU can build the next row while the current one transmits.
 *
 *   USER_I2S_REG 0 (the esp_lcd path) has a single buffer and a no-op
 *   i2s_switch_buffer(), so writing the next row while the transfer is still in
 *   flight is a race - the library's own loops have it. There we must wait.
 *
 * epd_fast_init() flips the buffer and sees whether the pointer moved.
 */

#pragma once

#include <string.h>
#include <stdint.h>

#include "epd_driver.h"
#include "ed047tc1.h"
#include "i2s_data_bus.h"

// One row of the frame, 1 bit per pixel, LSB of each byte = leftmost pixel.
#define EPD_ROW_BYTES  (EPD_WIDTH / 8)   // 120
// One row as the panel wants it: 2-bit drive code per pixel.
#define EPD_LINE_BYTES (EPD_WIDTH / 4)   // 240

// Dwell is the per-row drive time in RMT ticks of 0.1 us. Anything below the
// ~90 us the bus already spends per row costs nothing, so there is no reason to
// ask for less than this.
#define EPD_DWELL_FREE (100)

typedef enum {
  EPD_DARKEN  = 0,   // set bit -> 0b01
  EPD_LIGHTEN = 1,   // set bit -> 0b10
} EpdDir;

// byte -> 8 pixels of drive code. Built once by epd_fast_init().
static uint16_t s_epd_lut[2][256];

// True when the library gave us two line buffers, so building the next row does
// not have to wait for the current row's DMA. See the header comment.
static bool s_epd_double_buffered;

static void epd_fast_init(void) {
  uint8_t *b0 = epd_get_current_buffer();
  epd_switch_buffer();
  s_epd_double_buffered = (epd_get_current_buffer() != b0);
  epd_switch_buffer();   // back to where we started

  for (int b = 0; b < 256; b++) {
    uint16_t d = 0;
    for (int i = 0; i < 8; i++) {
      if (b & (1 << i)) d |= (uint16_t)1 << (2 * i);
    }
    s_epd_lut[EPD_DARKEN][b]  = d;
    s_epd_lut[EPD_LIGHTEN][b] = (uint16_t)(d << 1);
  }
}

// Rows we are not driving. Mirrors skip_row() in epd_driver.c: the first skipped
// row still has to be clocked out (with a no-op line loaded) to flush the row
// before it, the second is a bare latch, and only from the third on can we use
// the cheap CKV-only skip.
static inline void IRAM_ATTR epd_skip_row_(int *skipping, uint32_t dwell) {
  if (*skipping == 0) {
    // Both buffers have to hold the no-op line: whichever one is in flight, the
    // next row out must not re-drive the row we just left.
    epd_switch_buffer();
    memset(epd_get_current_buffer(), 0, EPD_LINE_BYTES);
    epd_switch_buffer();
    memset(epd_get_current_buffer(), 0, EPD_LINE_BYTES);
    epd_output_row(dwell);
  } else if (*skipping < 2) {
    epd_output_row(10);
  } else {
    epd_skip();
  }
  (*skipping)++;
}

/*
 * One waveform pass over rows [y0, y0+rows), moving every pixel that differs
 * between `prev` (what the panel is showing) and `cur` (what we want) in
 * whichever direction it needs - both directions in the same pass.
 *
 * Both bitmaps are `rows` x EPD_ROW_BYTES, LSB of each byte the leftmost pixel,
 * a set bit meaning black. Rows with nothing to do are skipped, which is where
 * most of the saving comes from on a mostly-static screen.
 *
 * `relight` is optional (pass NULL). Set bits in it are driven towards white
 * again even though `prev` already agrees with `cur`. A pixel needs more drive
 * to come back to clean white than it does to go black, so at low pass counts a
 * pixel that has just been vacated is left slightly short - and once `prev` has
 * recorded it as white it would never be driven again. Handing those pixels
 * back for one more frame costs nothing, because cost here is per row driven
 * and those rows are being driven anyway.
 *
 * It carries its own row range rather than being indexed off y0, so the caller
 * can keep it in a window narrower than the panel - a full-screen third plane
 * does not fit in this chip's internal RAM alongside the two framebuffers.
 *
 * Returns the number of rows that actually carried ink.
 */
typedef struct {
  const uint8_t *bits;   // rows x EPD_ROW_BYTES
  int y0;                // panel row the first row of `bits` refers to
  int rows;
} EpdRelight;

static IRAM_ATTR int epd_push_diff(int y0, int rows, const uint8_t *cur,
                                   const uint8_t *prev, const EpdRelight *relight,
                                   uint32_t dwell) {
  const uint16_t *ld = s_epd_lut[EPD_DARKEN];
  const uint16_t *ll = s_epd_lut[EPD_LIGHTEN];
  int driven = 0;
  int skipping = 0;

  epd_start_frame();
  for (int y = 0; y < EPD_HEIGHT; y++) {
    if (y < y0 || y >= y0 + rows) {
      epd_skip_row_(&skipping, dwell);
      continue;
    }

    const size_t off = (size_t)(y - y0) * EPD_ROW_BYTES;
    const uint8_t *c = cur + off;
    const uint8_t *p = prev + off;
    const uint8_t *g = NULL;
    if (relight && y >= relight->y0 && y < relight->y0 + relight->rows) {
      g = relight->bits + (size_t)(y - relight->y0) * EPD_ROW_BYTES;
    }

    // Split the row's difference into the pixels that have to gain ink and the
    // pixels that have to lose it. Both masks require opposite values of `cur`,
    // so they are disjoint by construction and their drive codes can be OR'd
    // into one word without ever asserting both column drivers on one pixel.
    uint8_t dark[EPD_ROW_BYTES], light[EPD_ROW_BYTES];
    uint32_t any = 0;
    for (int i = 0; i < EPD_ROW_BYTES; i++) {
      uint8_t d = (uint8_t)(c[i] & ~p[i]);
      uint8_t l = (uint8_t)((p[i] | (g ? g[i] : 0)) & ~c[i]);
      dark[i] = d; light[i] = l;
      any |= (uint32_t)(d | l);
    }
    if (!any) { epd_skip_row_(&skipping, dwell); continue; }

    // With one shared buffer the in-flight transfer is still reading it.
    if (!s_epd_double_buffered) { while (i2s_is_busy()) { } }
    uint32_t *dst = (uint32_t *)epd_get_current_buffer();
    for (int j = 0; j < EPD_WIDTH / 16; j++) {
      const int i1 = j * 2, i2 = j * 2 + 1;
      const uint32_t w1 = (uint32_t)(ld[dark[i1]] | ll[light[i1]]);
      const uint32_t w2 = (uint32_t)(ld[dark[i2]] | ll[light[i2]]);
      dst[j] = (w1 << 16) | w2;
    }
    skipping = 0;
    driven++;
    epd_output_row(dwell);
  }
  // The row loop is pipelined, so the last row still needs clocking out.
  if (!skipping) epd_output_row(dwell);
  epd_end_frame();

  return driven;
}

/*
 * One waveform pass that drives every pixel in the band the same way, with no
 * source bitmap. Used for the de-ghosting flash, where the point is to swing
 * the whole panel rather than to reach a particular image.
 */
static IRAM_ATTR void epd_push_flood(int y0, int rows, EpdDir dir, uint32_t dwell) {
  const uint8_t code = (dir == EPD_DARKEN) ? 0x55 : 0xAA;
  int skipping = 0;

  epd_start_frame();
  for (int y = 0; y < EPD_HEIGHT; y++) {
    if (y < y0 || y >= y0 + rows) { epd_skip_row_(&skipping, dwell); continue; }
    if (!s_epd_double_buffered) { while (i2s_is_busy()) { } }
    memset(epd_get_current_buffer(), code, EPD_LINE_BYTES);
    skipping = 0;
    epd_output_row(dwell);
  }
  if (!skipping) epd_output_row(dwell);
  epd_end_frame();
}

/*
 * One pass that drives a dithered *fraction* of the panel, chosen by an ordered
 * threshold: a pixel is driven once `step` reaches its slot in an 8x8 Bayer
 * matrix. Walking step from 0 to 63 dissolves the whole panel one way.
 *
 * This exists because contrast on this panel comes from pass count, not from
 * per-row drive time - so a slow clear cannot be had by lengthening the dwell,
 * which only makes a hard flash arrive later. Staggering *which* pixels are
 * driven is what turns the flash into a wipe.
 *
 * The Bayer cell is 8 pixels wide, which is exactly one 16-bit group of drive
 * codes, so a row is a single pattern repeated across the line buffer. That
 * also sidesteps the half-word swap the output stage does: both halves of every
 * 32-bit word hold the same pattern, so swapping them changes nothing.
 */
static const uint8_t s_epd_bayer8[8][8] = {
  {  0, 32,  8, 40,  2, 34, 10, 42 },
  { 48, 16, 56, 24, 50, 18, 58, 26 },
  { 12, 44,  4, 36, 14, 46,  6, 38 },
  { 60, 28, 52, 20, 62, 30, 54, 22 },
  {  3, 35, 11, 43,  1, 33,  9, 41 },
  { 51, 19, 59, 27, 49, 17, 57, 25 },
  { 15, 47,  7, 39, 13, 45,  5, 37 },
  { 63, 31, 55, 23, 61, 29, 53, 21 },
};

static IRAM_ATTR void epd_push_dissolve(EpdDir dir, int step, uint32_t dwell) {
  const uint16_t code = (dir == EPD_DARKEN) ? 1 : 2;   // 0b01 / 0b10

  uint16_t pat[8];
  for (int r = 0; r < 8; r++) {
    uint16_t v = 0;
    for (int k = 0; k < 8; k++) {
      if (s_epd_bayer8[r][k] <= step) v |= (uint16_t)(code << (2 * k));
    }
    pat[r] = v;
  }

  epd_start_frame();
  for (int y = 0; y < EPD_HEIGHT; y++) {
    if (!s_epd_double_buffered) { while (i2s_is_busy()) { } }
    uint16_t *dst = (uint16_t *)epd_get_current_buffer();
    const uint16_t v = pat[y & 7];
    for (int j = 0; j < EPD_LINE_BYTES / 2; j++) dst[j] = v;
    epd_output_row(dwell);
  }
  epd_output_row(dwell);
  epd_end_frame();
}

/*
 * A deliberate, several-second clear: dissolve to black, settle, dissolve back
 * to white, settle again. Slower than a flash on purpose - a hard strobe in the
 * middle of an otherwise calm animation is startling, where a wipe that takes a
 * few seconds reads as something the sketch meant to do.
 *
 * `hold` is how many passes each of the 64 dither levels gets. It is the
 * duration control, and it does double duty: holding a level longer also means
 * each group of pixels is driven harder as it crosses, so the leading edge of
 * the dissolve is properly black rather than grey. One pass per level is about
 * 2.7 s end to end, two is about 5.4 s.
 *
 * The closing flood passes matter as much as the dissolve. Every pixel has to
 * end at the panel's white rail, not merely white: the earliest-dithered pixels
 * collect dozens of lighten passes on the way through and the last ones only
 * get `hold`, so without enough settling the Bayer pattern itself prints faintly
 * into the background.
 */
static void epd_wipe(int hold, int dark_settle, int light_settle, uint32_t dwell) {
  for (int s = 0; s < 64; s++)
    for (int h = 0; h < hold; h++) epd_push_dissolve(EPD_DARKEN, s, dwell);
  for (int p = 0; p < dark_settle;  p++) epd_push_flood(0, EPD_HEIGHT, EPD_DARKEN,  dwell);
  for (int s = 0; s < 64; s++)
    for (int h = 0; h < hold; h++) epd_push_dissolve(EPD_LIGHTEN, s, dwell);
  for (int p = 0; p < light_settle; p++) epd_push_flood(0, EPD_HEIGHT, EPD_LIGHTEN, dwell);
}

/*
 * Flush the panel to white, alternating polarity. Driving every pixel white
 * would get the screen white but leaves the particles part-packed, so old
 * images keep showing through; swinging fully black and back is what actually
 * resets them. Visible as a flash, which is why it belongs at a reseed.
 *
 * The two directions get separate pass counts on purpose. White is not a single
 * level on this panel - keep driving a white pixel whiter and it keeps going,
 * up to a rail. Anything that relights pixels during the animation will push
 * them past wherever a lazy clear left the background, and the result is a
 * track that is brighter than the paper around it, which reads as the
 * background having gone grey. Ending the clear with enough lighten passes to
 * reach the rail is what stops that: it leaves nothing brighter to move to.
 */
static void epd_deghost(int cycles, int dark_passes, int light_passes, uint32_t dwell) {
  for (int c = 0; c < cycles; c++) {
    for (int p = 0; p < dark_passes;  p++) epd_push_flood(0, EPD_HEIGHT, EPD_DARKEN,  dwell);
    for (int p = 0; p < light_passes; p++) epd_push_flood(0, EPD_HEIGHT, EPD_LIGHTEN, dwell);
  }
}
