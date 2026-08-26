// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * epd_soak - three questions the cube sketch cannot afford to get wrong.
 *
 *   1. Can one waveform pass carry both drive directions? epd_push_diff() emits
 *      0b01 for pixels gaining ink and 0b10 for pixels losing it in the same
 *      row. Nothing in LilyGo-EPD47 ever does this, so it needs proving on
 *      hardware before the frame budget is built on it.
 *   2. Does the frame cost match the model? ~43 us per driven row, ~24 us per
 *      skipped one, times the pass count.
 *   3. Does the untouched background survive a few hundred partial updates, or
 *      does ghosting turn it to mud?
 *
 * The moving shape is deliberately shaped like one face of the cube: a white
 * panel with a thick black border and a grid of black cells, rotating and
 * drifting, so the mix of darkening and lightening pixels per frame is
 * representative of the real thing.
 */

#include <Arduino.h>
#include <esp_heap_caps.h>
#include "epd_fast.h"
#include "raster1.h"

#define PASSES      4
#define SOAK_FRAMES 300
#define FACE        320       // side of the rotating panel
#define CELLS       8

static uint8_t *s_fb;      // what we want on the panel
static uint8_t *s_shown;   // what the panel is showing

static float s_ang = 0.0f;
static float s_cx = 480.0f, s_cy = 270.0f;
static float s_vx = 46.0f,  s_vy = 27.0f;
static uint32_t s_cellmask = 0x9E3779B9u;

// Rotate a face-local point (u,v) in [-1,1] about the shape centre.
static inline void xf(float u, float v, float c, float s, float *ox, float *oy) {
  const float h = FACE * 0.5f;
  *ox = s_cx + (u * c - v * s) * h;
  *oy = s_cy + (u * s + v * c) * h;
}

static void render(void) {
  r1_clear(s_fb, false);
  const float c = cosf(s_ang), s = sinf(s_ang);

  // Cells first, then the border over the top of them.
  for (int j = 0; j < CELLS; j++) {
    for (int i = 0; i < CELLS; i++) {
      if (!((s_cellmask >> ((j * CELLS + i) & 31)) & 1)) continue;
      const float u0 = -1.0f + 2.0f * i / CELLS, u1 = -1.0f + 2.0f * (i + 1) / CELLS;
      const float v0 = -1.0f + 2.0f * j / CELLS, v1 = -1.0f + 2.0f * (j + 1) / CELLS;
      float qx[4], qy[4];
      xf(u0, v0, c, s, &qx[0], &qy[0]);
      xf(u1, v0, c, s, &qx[1], &qy[1]);
      xf(u1, v1, c, s, &qx[2], &qy[2]);
      xf(u0, v1, c, s, &qx[3], &qy[3]);
      r1_fill_quad(s_fb, qx, qy, true);
    }
  }
  float bx[4], by[4];
  xf(-1, -1, c, s, &bx[0], &by[0]);
  xf( 1, -1, c, s, &bx[1], &by[1]);
  xf( 1,  1, c, s, &bx[2], &by[2]);
  xf(-1,  1, c, s, &bx[3], &by[3]);
  for (int i = 0; i < 4; i++) {
    const int j = (i + 1) % 4;
    r1_thick_line(s_fb, bx[i], by[i], bx[j], by[j], 4.0f, true);
  }
}

// Rows that differ between the two buffers, so we only drive what moved.
static bool diff_band(int *y0, int *rows) {
  int lo = -1, hi = -1;
  for (int y = 0; y < EPD_HEIGHT; y++) {
    const size_t off = (size_t)y * EPD_ROW_BYTES;
    if (memcmp(s_fb + off, s_shown + off, EPD_ROW_BYTES) != 0) {
      if (lo < 0) lo = y;
      hi = y;
    }
  }
  if (lo < 0) return false;
  *y0 = lo; *rows = hi - lo + 1;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\nepd_soak");

  epd_base_init(EPD_WIDTH);
  epd_fast_init();
  Serial.printf("double buffered: %d\n", (int)s_epd_double_buffered);

  s_fb    = (uint8_t *)heap_caps_malloc(R1_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s_shown = (uint8_t *)heap_caps_malloc(R1_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!s_fb || !s_shown) { Serial.println("FATAL: alloc failed"); while (1) delay(1000); }

  // Power stays on for the whole run: epd_poweron() has settling delays that
  // would dwarf a frame.
  epd_poweron();
  epd_deghost(2, 4, 6, EPD_DWELL_FREE);
  r1_clear(s_shown, false);

  uint32_t render_us = 0, push_us = 0, driven_total = 0;
  const uint32_t t_start = millis();

  for (int frame = 0; frame < SOAK_FRAMES; frame++) {
    uint32_t t0 = micros();
    s_ang += 0.035f;
    s_cx += s_vx * 0.09f;
    s_cy += s_vy * 0.09f;
    if (s_cx < 250 || s_cx > 710) s_vx = -s_vx;
    if (s_cy < 240 || s_cy > 300) s_vy = -s_vy;
    if ((frame % 12) == 0) s_cellmask = s_cellmask * 1664525u + 1013904223u;
    render();
    render_us += micros() - t0;

    int y0, rows;
    if (!diff_band(&y0, &rows)) continue;

    t0 = micros();
    int driven = 0;
    for (int p = 0; p < PASSES; p++)
      driven = epd_push_diff(y0, rows, s_fb + (size_t)y0 * EPD_ROW_BYTES,
                             s_shown + (size_t)y0 * EPD_ROW_BYTES, NULL, EPD_DWELL_FREE);
    push_us += micros() - t0;
    driven_total += driven;

    // Record exactly the band we drove, never more: anything copied in without
    // being driven would be recorded as shown while the panel still holds the
    // old pixels, and would stay wrong forever.
    memcpy(s_shown + (size_t)y0 * EPD_ROW_BYTES,
           s_fb + (size_t)y0 * EPD_ROW_BYTES,
           (size_t)rows * EPD_ROW_BYTES);

    if ((frame % 50) == 0)
      Serial.printf("frame %3d  band y=%3d rows=%3d driven=%3d\n", frame, y0, rows, driven);
  }

  const uint32_t total_ms = millis() - t_start;
  epd_poweroff();

  Serial.printf("\n%d frames in %lu ms -> %.1f fps\n",
                SOAK_FRAMES, (unsigned long)total_ms, SOAK_FRAMES * 1000.0 / total_ms);
  Serial.printf("  render %.2f ms/frame\n", render_us / 1000.0 / SOAK_FRAMES);
  Serial.printf("  push   %.2f ms/frame (%d passes, avg %lu driven rows)\n",
                push_us / 1000.0 / SOAK_FRAMES, PASSES,
                (unsigned long)(driven_total / SOAK_FRAMES));
  Serial.println("done - shape is left where it stopped, background should still be clean");
}

void loop() { delay(1000); }
