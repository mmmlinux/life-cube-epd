// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * raster1.h - 1bpp rasterizing into a full-screen ED047TC1 framebuffer.
 *
 * Bit 0 of each byte is the leftmost pixel, matching the panel's expansion LUT
 * in epd_fast.h; a set bit means black. Spans are filled a byte at a time
 * rather than a pixel at a time, because a frame of this cube is a few hundred
 * filled quads and the per-pixel version shows up in the frame budget.
 *
 * Polygon filling uses the half-open convention on both axes: a scanline
 * belongs to an edge if y is in [ymin, ymax), and a span covers [xl, xr). Quads
 * that share an edge therefore tile it exactly once, with no double-drawn seam
 * and no hairline gap - which matters here because live cells are drawn as
 * adjacent quads and any seam would read as a grid line.
 */

#pragma once

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "epd_fast.h"   // EPD_ROW_BYTES, EPD_WIDTH, EPD_HEIGHT

#define R1_FB_BYTES ((size_t)EPD_HEIGHT * EPD_ROW_BYTES)

static inline void r1_clear(uint8_t *fb, bool black) {
  memset(fb, black ? 0xFF : 0x00, R1_FB_BYTES);
}

static inline void r1_px(uint8_t *fb, int x, int y, bool black) {
  if ((unsigned)x >= EPD_WIDTH || (unsigned)y >= EPD_HEIGHT) return;
  uint8_t *b = fb + (size_t)y * EPD_ROW_BYTES + (x >> 3);
  const uint8_t m = (uint8_t)(1 << (x & 7));
  if (black) *b |= m; else *b &= (uint8_t)~m;
}

// Fill [xa, xb) on row y.
static void r1_span(uint8_t *fb, int y, int xa, int xb, bool black) {
  if ((unsigned)y >= EPD_HEIGHT) return;
  if (xa < 0) xa = 0;
  if (xb > EPD_WIDTH) xb = EPD_WIDTH;
  if (xa >= xb) return;

  uint8_t *row = fb + (size_t)y * EPD_ROW_BYTES;
  const int b0 = xa >> 3, b1 = (xb - 1) >> 3;
  const uint8_t m0 = (uint8_t)(0xFF << (xa & 7));
  const uint8_t m1 = (uint8_t)(0xFF >> (7 - ((xb - 1) & 7)));

  if (b0 == b1) {
    const uint8_t m = (uint8_t)(m0 & m1);
    if (black) row[b0] |= m; else row[b0] &= (uint8_t)~m;
    return;
  }
  if (black) row[b0] |= m0; else row[b0] &= (uint8_t)~m0;
  if (b1 > b0 + 1) memset(row + b0 + 1, black ? 0xFF : 0x00, (size_t)(b1 - b0 - 1));
  if (black) row[b1] |= m1; else row[b1] &= (uint8_t)~m1;
}

static void r1_rect(uint8_t *fb, int x, int y, int w, int h, bool black) {
  for (int j = 0; j < h; j++) r1_span(fb, y + j, x, x + w, black);
}

/*
 * Fill a convex polygon given as n points. For each scanline the polygon is
 * convex, so the covered range is just [min, max] of the edge crossings.
 */
static void r1_fill_convex(uint8_t *fb, const float *xs, const float *ys, int n, bool black) {
  float fy0 = ys[0], fy1 = ys[0];
  for (int i = 1; i < n; i++) {
    if (ys[i] < fy0) fy0 = ys[i];
    if (ys[i] > fy1) fy1 = ys[i];
  }
  int y0 = (int)ceilf(fy0 - 0.5f), y1 = (int)ceilf(fy1 - 0.5f);
  if (y0 < 0) y0 = 0;
  if (y1 > EPD_HEIGHT) y1 = EPD_HEIGHT;

  for (int y = y0; y < y1; y++) {
    const float sy = (float)y + 0.5f;
    float xl = 1e30f, xr = -1e30f;
    for (int i = 0; i < n; i++) {
      const int j = (i + 1) % n;
      const float ay = ys[i], by = ys[j];
      if (ay == by) continue;
      // Half-open in y, so a vertex shared by two edges is counted once.
      const float lo = ay < by ? ay : by, hi = ay < by ? by : ay;
      if (sy < lo || sy >= hi) continue;
      const float t = (sy - ay) / (by - ay);
      const float x = xs[i] + t * (xs[j] - xs[i]);
      if (x < xl) xl = x;
      if (x > xr) xr = x;
    }
    if (xr < xl) continue;
    const int xa = (int)ceilf(xl - 0.5f);
    const int xb = (int)ceilf(xr - 0.5f);
    r1_span(fb, y, xa, xb, black);
  }
}

static inline void r1_fill_quad(uint8_t *fb, const float *xs, const float *ys, bool black) {
  r1_fill_convex(fb, xs, ys, 4, black);
}

/*
 * A line of the given width, drawn as a quad so thickness is honest rather than
 * a stack of offset one-pixel lines. Line weight is how this design shows face
 * orientation, so it has to be even along the whole run.
 */
static void r1_thick_line(uint8_t *fb, float x0, float y0, float x1, float y1,
                          float width, bool black) {
  float dx = x1 - x0, dy = y1 - y0;
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 1e-4f) return;
  const float h = width * 0.5f;
  // Unit normal, scaled to half the width.
  const float nx = -dy / len * h, ny = dx / len * h;
  const float qx[4] = { x0 + nx, x1 + nx, x1 - nx, x0 - nx };
  const float qy[4] = { y0 + ny, y1 + ny, y1 - ny, y0 - ny };
  r1_fill_convex(fb, qx, qy, 4, black);
}
