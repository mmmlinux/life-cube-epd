// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Conway's Game of Life on a rotating, bouncing 3D cube - e-paper edition.
 * LilyGo T5 4.7" (ESP32-WROVER, ED047TC1 960x540, 16-level greyscale panel
 * driven as pure black and white).
 *
 * A port of the ESP32-C5-LCD-1.47 version of this sketch. The Life topology,
 * the tumble and the reseed logic are the same; everything about how it reaches
 * the glass is different, because e-paper is not a framebuffer you can just
 * blit to sixty times a second.
 *
 * What changed, and why:
 *
 *   - No colour, and no greyscale either. The panel does 16 levels, but a
 *     greyscale update is ~500 ms. Black and white differential updates run in
 *     tens of milliseconds, so the design is 1-bit and the palette is gone.
 *   - No dithering. Ordered dither would re-quantize nearly every pixel as a
 *     face's brightness drifts, which turns a cheap partial update into a
 *     full-screen one and looks worse besides. Face orientation is shown by the
 *     weight of its outline instead - lit faces get a heavier line.
 *   - Cells are drawn inset inside their quad, so the white gutter between them
 *     reads as the grid. On the colour version the face backdrop did that job.
 *   - Both drive directions ride in one waveform pass (see epd_fast.h), and
 *     only the rows that actually changed are driven.
 *
 * Measured on the hardware: 3.4 ms to render a frame and 46.6 ms to push it, so
 * 20 fps. Contrast on this panel comes from the number of waveform passes, not
 * from the per-row dwell, and PASSES is the whole trade: one pass measured
 * 40.6 fps but the cells go visibly grey, four would land near 11 fps for
 * blacker blacks.
 *
 * Build (from the repo root, which holds libraries/):
 *   arduino-cli compile -u -p /dev/cu.usbserial-XXXX \
 *     --fqbn esp32:esp32:esp32:PSRAM=disabled,FlashSize=16M,PartitionScheme=huge_app,UploadSpeed=460800 \
 *     --libraries libraries life_cube_epd
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <esp_heap_caps.h>
#include <esp_random.h>

#include "epd_fast.h"
#include "raster1.h"

// ---------------------------------------------------------------------------
// Update quality
//
// Passes per frame is the one knob that trades frame rate against how black the
// black is. The panel needs repeated waveform passes to move its particles all
// the way; more drive time per pass barely helps, which is why the dwell is
// pinned at the point below which the bus is the limit anyway. Measured at this
// cube size: 2 passes 20.0 fps. One pass measured 40.6 fps at a slightly larger
// cube, but visibly grey. Two is the point where the cells still read as black
// while moving.
// ---------------------------------------------------------------------------
#define PASSES        (2)
#define DWELL         (EPD_DWELL_FREE)

// A full de-ghosting flash, run at reseeds. Partial updates alone stay clean
// for hundreds of frames here, but a reseed is a free moment to reset the
// panel: the flash reads as a deliberate wipe rather than a glitch.
//
// Far more lighten passes than darken ones, because this is what sets the white
// the rest of the run is measured against. The relight in present() drives
// vacated pixels white repeatedly, and if the background were left short of the
// panel's white rail those tracks would end up brighter than the paper - which
// looks exactly like the background having gone grey.
// Run as a dissolve rather than a flash - see epd_wipe(). It takes a couple of
// seconds, which is the point: a strobe in the middle of a calm animation is
// startling, a slow wipe reads as deliberate.
#define WIPE_HOLD         (2)    // passes per dither level; 2 measures 5.2 s
#define WIPE_DARK_SETTLE  (4)
#define WIPE_LIGHT_SETTLE (10)

// Set to 1 to swap that dissolve for a white-only flood at reseeds: no darken
// phase at all, so ~0.2 s instead of ~5.2 s. The trade is ghosting - see
// epd_deghost(), which notes that lightening pixels without first swinging them
// black leaves the particles part-packed, so the spent cube is expected to show
// through. Boot keeps the full wipe either way: it has no idea what the
// previous sketch left on the panel, and that is the one clear that most needs
// a real reset.
//
// With this on, nothing during a run ever swings the panel fully black, so
// ghosting accumulates across reseeds with no bounded recovery. The refresh
// button (see REFRESH_BUTTON) is the counterweight: it becomes the only full
// reset left, for when the build-up starts to bother you.
#define WHITE_ONLY_REFRESH   (1)
#define WHITE_REFRESH_PASSES (12)   // full-screen lighten passes; measures 0.2 s

// ---------------------------------------------------------------------------
// Cube geometry / projection
//
// Model cube spans [-1,1]^3, camera at +Z distance CAM_D looking at the origin.
// Sweeping orientations on the host puts the worst-case projected corner at
// max |X/(CAM_D-Z)| = 0.4804, so the cube never reaches further than
// 0.4804 * PROJ_SCALE from its centre. That constant depends only on the unit
// cube and CAM_D, so it carries over from the colour version unchanged.
// ---------------------------------------------------------------------------
#define CAM_D       (4.0f)
#define PROJ_SCALE  (310.0f)
#define CUBE_RADIUS (0.4804f * PROJ_SCALE)   // 149 px

#define N           (16)                 // cells per face edge
#define FACE_CELLS  (N * N)
#define TOTAL_CELLS (6 * FACE_CELLS)     // 1536
#define MAX_NBR     (8)
#define VERTS       ((N + 1) * (N + 1))

// Cells are drawn shrunk towards their own centre; the gap that leaves is the
// grid. Too small and adjacent live cells merge into a blob, too large and the
// cube looks like a sparse dot matrix.
#define CELL_FILL   (0.84f)

// Outline weight, in pixels, for a face turned fully away from the light and
// fully towards it. This is the only shading channel a 1-bit panel has.
#define EDGE_MIN    (1.5f)
#define EDGE_MAX    (4.5f)

// ---------------------------------------------------------------------------
// Life on the cube surface
// ---------------------------------------------------------------------------
#define SEED_DENSITY_PCT        (32)
#define GENERATION_INTERVAL_MS  (260)
#define HASH_RING_LEN           (12)
#define MAX_GENERATIONS_NO_LOOP (900)

static uint8_t  s_cur[TOTAL_CELLS];
static uint8_t  s_next[TOTAL_CELLS];
static uint8_t  s_nbr_count[TOTAL_CELLS];
static int16_t  s_nbr[TOTAL_CELLS][MAX_NBR];

static uint32_t s_hash_ring[HASH_RING_LEN];
static uint8_t  s_hash_ring_pos = 0;
static uint32_t s_generation = 0;

// ---------------------------------------------------------------------------
// Motion
//
// Rates are per second and integrated against measured elapsed time, so the
// tumble looks the same whatever the frame rate settles at. The spin is slower
// than on the colour version: at ~15 fps a 0.85 rad/s roll advances 3 degrees
// between frames, which on e-paper reads as a stutter rather than a spin.
// ---------------------------------------------------------------------------
#define SPIN_X_RAD_S (0.52f)   // deliberately incommensurate, so the cube never
#define SPIN_Y_RAD_S (0.35f)   // settles into a repeating tumble
#define SPIN_Z_RAD_S (0.21f)
#define DRIFT_X_PX_S (34.0f)
#define DRIFT_Y_PX_S (16.0f)

static float s_pos_x, s_pos_y, s_vel_x, s_vel_y;
static float s_ang_x = 0.0f, s_ang_y = 0.0f, s_ang_z = 0.0f;

// Framebuffers live on the heap: two full-screen 1bpp buffers are 129,600 bytes
// together, which overflows the static DRAM segment even though there is plenty
// of internal RAM for them at runtime.
static uint8_t *s_fb;      // the frame we want on the panel
static uint8_t *s_shown;   // the frame the panel is actually holding
// Pixels vacated last frame, owed one more whitening. Only ever as tall as the
// band the cube occupies, because a third full-screen plane will not fit next
// to the two framebuffers: the largest contiguous block of internal RAM left at
// that point measures about 47 KB.
#define DEBT_ROWS (384)
static uint8_t *s_debt;
static int s_debt_y0 = 0, s_debt_rows = 0;

// Scratch: the projected vertex grid of the face being drawn.
static float s_vx[VERTS], s_vy[VERTS];

// Frame accounting, reported on the reseed line.
static uint32_t s_frames = 0, s_render_us = 0, s_push_us = 0;

// ---------------------------------------------------------------------------
// Cube surface topology
//
// Face f covers axis f/2 at sign (f&1 ? -1 : +1). Its two in-plane axes are
// u = (axis+1)%3 and v = (axis+2)%3, and cell (i,j) sits at
// u = -1 + (2i+1)/N, v = -1 + (2j+1)/N. Rendering uses the same convention, so
// the texture and the topology always agree.
//
// Neighbours come from probing one cell-width out in each of the 8 directions
// and folding the resulting 3D point back onto the cube with a cube-map lookup
// (largest-magnitude component picks the face). Points that walk off an edge
// land on the adjacent face at the correct distance from the shared edge; the
// 24 cells at the cube's 8 corners correctly end up with 7 neighbours each.
// ---------------------------------------------------------------------------
static inline int face_id(int axis, int sign)
{
    return axis * 2 + (sign < 0 ? 1 : 0);
}

static int cubemap_cell(const float q[3])
{
    int m = 0;
    float best = fabsf(q[0]);
    if (fabsf(q[1]) > best) { best = fabsf(q[1]); m = 1; }
    if (fabsf(q[2]) > best) { best = fabsf(q[2]); m = 2; }

    const int sm = (q[m] < 0.0f) ? -1 : 1;
    const int u2 = (m + 1) % 3;
    const int v2 = (m + 2) % 3;
    const float inv = 1.0f / fabsf(q[m]);

    int i = (int)floorf((q[u2] * inv + 1.0f) * 0.5f * N);
    int j = (int)floorf((q[v2] * inv + 1.0f) * 0.5f * N);
    if (i < 0) { i = 0; } else if (i >= N) { i = N - 1; }
    if (j < 0) { j = 0; } else if (j >= N) { j = N - 1; }

    return face_id(m, sm) * FACE_CELLS + j * N + i;
}

static bool add_neighbor(int a, int b)
{
    if (a == b) {
        return false;
    }
    for (int k = 0; k < s_nbr_count[a]; k++) {
        if (s_nbr[a][k] == b) {
            return false;
        }
    }
    if (s_nbr_count[a] >= MAX_NBR) {
        return false;
    }
    s_nbr[a][s_nbr_count[a]++] = (int16_t)b;
    return true;
}

static void build_topology(void)
{
    memset(s_nbr_count, 0, sizeof(s_nbr_count));

    for (int f = 0; f < 6; f++) {
        const int axis = f / 2;
        const int sign = (f & 1) ? -1 : 1;
        const int u = (axis + 1) % 3;
        const int v = (axis + 2) % 3;

        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                const int idx = f * FACE_CELLS + j * N + i;
                const float cu = -1.0f + (2.0f * i + 1.0f) / N;
                const float cv = -1.0f + (2.0f * j + 1.0f) / N;

                for (int dj = -1; dj <= 1; dj++) {
                    for (int di = -1; di <= 1; di++) {
                        if (di == 0 && dj == 0) {
                            continue;
                        }
                        float q[3];
                        q[axis] = (float)sign;
                        q[u] = cu + di * (2.0f / N);
                        q[v] = cv + dj * (2.0f / N);
                        add_neighbor(idx, cubemap_cell(q));
                    }
                }
            }
        }
    }

    // A cube corner is a genuine argmax tie for the diagonal probe, so the
    // fold's axis-order tie-break could in principle make adjacency one-way.
    // Force symmetry rather than trusting it, then report what we ended up
    // with - one-way adjacency doesn't crash, it just quietly makes Life
    // misbehave near the edges.
    int added = 0;
    for (int a = 0; a < TOTAL_CELLS; a++) {
        const int cnt = s_nbr_count[a];
        for (int k = 0; k < cnt; k++) {
            if (add_neighbor(s_nbr[a][k], a)) {
                added++;
            }
        }
    }

    int hist[MAX_NBR + 1];
    memset(hist, 0, sizeof(hist));
    for (int a = 0; a < TOTAL_CELLS; a++) {
        hist[s_nbr_count[a]]++;
    }

    Serial.printf("topology: %d cells, %d symmetry fixups\n", TOTAL_CELLS, added);
    for (int d = 0; d <= MAX_NBR; d++) {
        if (hist[d]) {
            Serial.printf("  degree %d: %d cells%s\n", d, hist[d],
                          (d == 7) ? "  (cube corners)" : "");
        }
    }
}

// ---------------------------------------------------------------------------
// Life rules
// ---------------------------------------------------------------------------
static void seed_grid(uint8_t density_percent)
{
    for (int i = 0; i < TOTAL_CELLS; i++) {
        s_cur[i] = ((esp_random() % 100) < density_percent) ? 1 : 0;
    }
}

static int step_life(void)
{
    int live_count = 0;

    for (int i = 0; i < TOTAL_CELLS; i++) {
        int neighbors = 0;
        const int cnt = s_nbr_count[i];
        for (int k = 0; k < cnt; k++) {
            neighbors += s_cur[s_nbr[i][k]];
        }

        uint8_t next_alive;
        if (s_cur[i]) {
            next_alive = (neighbors == 2 || neighbors == 3) ? 1 : 0;
        } else {
            next_alive = (neighbors == 3) ? 1 : 0;
        }
        s_next[i] = next_alive;
        live_count += next_alive;
    }

    memcpy(s_cur, s_next, TOTAL_CELLS);
    return live_count;
}

static int live_cells(void)
{
    int n = 0;
    for (int i = 0; i < TOTAL_CELLS; i++) {
        n += s_cur[i];
    }
    return n;
}

static uint32_t hash_grid(void)
{
    uint32_t hash = 2166136261u;
    for (int i = 0; i < TOTAL_CELLS; i++) {
        hash ^= s_cur[i];
        hash *= 16777619u;
    }
    return hash;
}

static bool hash_seen_recently(uint32_t hash)
{
    for (int i = 0; i < HASH_RING_LEN; i++) {
        if (s_hash_ring[i] == hash) {
            return true;
        }
    }
    return false;
}

static void push_hash(uint32_t hash)
{
    s_hash_ring[s_hash_ring_pos] = hash;
    s_hash_ring_pos = (s_hash_ring_pos + 1) % HASH_RING_LEN;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
static void render_cube(void)
{
    // R = Rz * Ry * Rx
    const float cx = cosf(s_ang_x), sx = sinf(s_ang_x);
    const float cy = cosf(s_ang_y), sy = sinf(s_ang_y);
    const float cz = cosf(s_ang_z), sz = sinf(s_ang_z);

    const float R[3][3] = {
        { cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx },
        { sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx },
        { -sy,     cy * sx,                cy * cx                },
    };

    // Light roughly over the viewer's shoulder.
    const float lx = 0.35f, ly = -0.45f, lz = 0.82f;

    r1_clear(s_fb, false);

    for (int f = 0; f < 6; f++) {
        const int axis = f / 2;
        const int sign = (f & 1) ? -1 : 1;
        const int u = (axis + 1) % 3;
        const int v = (axis + 2) % 3;

        // Face centre and outward normal coincide for a unit cube: both are
        // sign * (the axis'th column of R).
        const float nx = R[0][axis] * sign;
        const float ny = R[1][axis] * sign;
        const float nz = R[2][axis] * sign;

        // Exact perspective backface test: is the outward normal pointing away
        // from the ray that reaches this face from the camera at (0,0,CAM_D)?
        if (nx * nx + ny * ny + nz * (nz - CAM_D) >= 0.0f) {
            continue;
        }

        // Project this face's (N+1)x(N+1) vertex grid once, then let every cell
        // reuse the four corners it shares with its neighbours.
        for (int gj = 0; gj <= N; gj++) {
            const float gv = -1.0f + 2.0f * gj / N;
            for (int gi = 0; gi <= N; gi++) {
                const float gu = -1.0f + 2.0f * gi / N;

                float p[3];
                p[axis] = (float)sign;
                p[u] = gu;
                p[v] = gv;

                const float X = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2];
                const float Y = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2];
                const float Z = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2];

                const float w = PROJ_SCALE / (CAM_D - Z);
                const int idx = gj * (N + 1) + gi;
                s_vx[idx] = X * w + s_pos_x;
                s_vy[idx] = Y * w + s_pos_y;
            }
        }

        float lambert = nx * lx + ny * ly + nz * lz;
        if (lambert < 0.0f) {
            lambert = 0.0f;
        }

        // Live cells, each shrunk towards its own centre so the white gutter
        // between neighbours reads as the grid.
        const int base = f * FACE_CELLS;
        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                if (!s_cur[base + j * N + i]) {
                    continue;
                }
                const int a = j * (N + 1) + i;
                const int c[4] = { a, a + 1, a + N + 2, a + N + 1 };

                float qx[4], qy[4], mx = 0.0f, my = 0.0f;
                for (int k = 0; k < 4; k++) { mx += s_vx[c[k]]; my += s_vy[c[k]]; }
                mx *= 0.25f; my *= 0.25f;
                for (int k = 0; k < 4; k++) {
                    qx[k] = mx + (s_vx[c[k]] - mx) * CELL_FILL;
                    qy[k] = my + (s_vy[c[k]] - my) * CELL_FILL;
                }
                r1_fill_quad(s_fb, qx, qy, true);
            }
        }

        // Silhouette and crease outline. Its weight is the only cue this panel
        // has for which way a face is turned, so it carries the shading that
        // the colour version put into the face tint.
        const float width = EDGE_MIN + (EDGE_MAX - EDGE_MIN) * lambert;
        const int corner[4] = { 0, N, (N + 1) * (N + 1) - 1, N * (N + 1) };
        for (int k = 0; k < 4; k++) {
            const int a = corner[k], b = corner[(k + 1) % 4];
            r1_thick_line(s_fb, s_vx[a], s_vy[a], s_vx[b], s_vy[b], width, true);
        }
    }
}

// ---------------------------------------------------------------------------
// Presenting
//
// Only the rows that changed get driven, and the band recorded into s_shown is
// exactly the band that was driven. Copying in anything wider would mark pixels
// as displayed that the panel was never told about, and those pixels would then
// stay wrong for good.
//
// Coming back to clean white takes more drive than going black does, so at two
// passes the pixels the cube has just moved off are left a little short - which
// is exactly the faint trail that shows up behind it. Those pixels get one more
// whitening on the following frame, carried in s_debt. It is free: the cost of
// a frame is the number of rows driven, and a just-vacated row is being driven
// anyway.
// ---------------------------------------------------------------------------

// A row needs driving if the frame changed there, or if it still owes a pixel a
// second whitening. Counting the debt here is what stops the band dropping a
// vacated row before it has been paid off.
static bool row_needs_drive(int y)
{
    const size_t off = (size_t)y * EPD_ROW_BYTES;
    if (memcmp(s_fb + off, s_shown + off, EPD_ROW_BYTES) != 0) {
        return true;
    }
    if (y < s_debt_y0 || y >= s_debt_y0 + s_debt_rows) {
        return false;
    }
    const uint8_t *g = s_debt + (size_t)(y - s_debt_y0) * EPD_ROW_BYTES;
    for (size_t i = 0; i < EPD_ROW_BYTES; i++) {
        if (g[i] & ~s_fb[off + i]) {
            return true;
        }
    }
    return false;
}

static bool diff_band(int *y0, int *rows)
{
    int lo = -1, hi = -1;
    for (int y = 0; y < EPD_HEIGHT; y++) {
        if (row_needs_drive(y)) {
            if (lo < 0) {
                lo = y;
            }
            hi = y;
        }
    }
    if (lo < 0) {
        return false;
    }
    *y0 = lo;
    *rows = hi - lo + 1;
    return true;
}

static void present(void)
{
    int y0, rows;
    if (!diff_band(&y0, &rows)) {
        return;
    }

    const size_t off = (size_t)y0 * EPD_ROW_BYTES;
    const size_t n = (size_t)rows * EPD_ROW_BYTES;

    const EpdRelight rl = { s_debt, s_debt_y0, s_debt_rows };
    for (int p = 0; p < PASSES; p++) {
        epd_push_diff(y0, rows, s_fb + off, s_shown + off,
                      s_debt_rows ? &rl : NULL, DWELL);
    }

    // Whatever went white this frame is what we owe on the next one. Every row
    // carrying debt gets pulled back into the band by row_needs_drive(), so a
    // debt is always either paid or still recorded - never stranded. A band
    // taller than the plane can only happen if the cube grows, and then the
    // overflow simply goes unpaid rather than corrupting anything.
    if (s_debt) {
        const int drows = rows > DEBT_ROWS ? DEBT_ROWS : rows;
        for (int r = 0; r < drows; r++) {
            const size_t src = off + (size_t)r * EPD_ROW_BYTES;
            uint8_t *dst = s_debt + (size_t)r * EPD_ROW_BYTES;
            for (size_t i = 0; i < EPD_ROW_BYTES; i++) {
                dst[i] = (uint8_t)(s_shown[src + i] & ~s_fb[src + i]);
            }
        }
        s_debt_y0 = y0;
        s_debt_rows = drows;
    }
    memcpy(s_shown + off, s_fb + off, n);
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

// The walls are suspended per axis while the cube is off the panel. Doing it
// per axis matters on a diagonal exit: the cube can come back on with one axis
// still mid-panel, and that axis has to keep bouncing normally or it will just
// wander off again while the other one is arriving.
static bool s_free_x = false, s_free_y = false;
// While arriving, a suspended axis takes its wall back the moment it is in
// range. While leaving, it never does.
static bool s_arriving = false;

static const float BOUNCE_LO_X = CUBE_RADIUS + EDGE_MAX;
static const float BOUNCE_HI_X = EPD_WIDTH  - CUBE_RADIUS - EDGE_MAX;
static const float BOUNCE_LO_Y = CUBE_RADIUS + EDGE_MAX;
static const float BOUNCE_HI_Y = EPD_HEIGHT - CUBE_RADIUS - EDGE_MAX;

static void update_motion(float dt)
{
    s_ang_x += SPIN_X_RAD_S * dt;
    s_ang_y += SPIN_Y_RAD_S * dt;
    s_ang_z += SPIN_Z_RAD_S * dt;
    if (s_ang_x > (float)TWO_PI) s_ang_x -= (float)TWO_PI;
    if (s_ang_y > (float)TWO_PI) s_ang_y -= (float)TWO_PI;
    if (s_ang_z > (float)TWO_PI) s_ang_z -= (float)TWO_PI;

    s_pos_x += s_vel_x * dt;
    s_pos_y += s_vel_y * dt;

    // The cube is positioned by its centre and never projects further than
    // CUBE_RADIUS from it, so the bounce limits come straight from that.
    if (s_free_x) {
        if (s_arriving && s_pos_x >= BOUNCE_LO_X && s_pos_x <= BOUNCE_HI_X) s_free_x = false;
    } else {
        if (s_pos_x < BOUNCE_LO_X) { s_pos_x = BOUNCE_LO_X; s_vel_x = -s_vel_x; }
        if (s_pos_x > BOUNCE_HI_X) { s_pos_x = BOUNCE_HI_X; s_vel_x = -s_vel_x; }
    }
    if (s_free_y) {
        if (s_arriving && s_pos_y >= BOUNCE_LO_Y && s_pos_y <= BOUNCE_HI_Y) s_free_y = false;
    } else {
        if (s_pos_y < BOUNCE_LO_Y) { s_pos_y = BOUNCE_LO_Y; s_vel_y = -s_vel_y; }
        if (s_pos_y > BOUNCE_HI_Y) { s_pos_y = BOUNCE_HI_Y; s_vel_y = -s_vel_y; }
    }
}

// File-scope so a transit can reset it: after a five-second wipe the measured
// gap would otherwise clamp to the 0.5 s ceiling and jump the cube 160 px on
// its first frame back.
static uint32_t s_last_frame_us = 0;

static float animate_frame(void)
{
    const uint32_t start_us = micros();

    const uint32_t gap_us = start_us - s_last_frame_us;
    const bool first_frame = (s_last_frame_us == 0);
    s_last_frame_us = start_us;

    float dt = first_frame ? 0.0f : gap_us * 1e-6f;
    if (dt > 0.5f) {
        dt = 0.5f;   // a long stall - don't teleport the cube
    }

    update_motion(dt);

    uint32_t t = micros();
    render_cube();
    s_render_us += micros() - t;

    t = micros();
    present();
    s_push_us += micros() - t;
    s_frames++;

    return dt;
}

// ---------------------------------------------------------------------------
// Buttons
//
// The three panel buttons sit on GPIO 34, 35 and 39. On the ESP32 those pins
// are input-only and have no internal pull resistors at all, so the board's
// external pull-ups are the only thing holding them high and a press reads low.
// pinMode(..., INPUT_PULLUP) on these pins is silently a no-op.
//
// That also means a pin without a working pull-up floats, and a floating input
// would fire the reseed at random. Each pin is therefore probed at boot and
// only trusted if it sits solidly high for the whole probe; anything else is
// dropped, with a line saying so. A button held down during boot reads low and
// gets dropped too, which is a fair trade for not chasing phantom presses.
// ---------------------------------------------------------------------------
#define BUTTON_COUNT       (3)
#define BUTTON_DEBOUNCE_MS (40)
#define BUTTON_PROBE_MS    (120)

// Index into BUTTON_PINS of the button that runs an immediate de-ghost instead
// of ending the run. 2 is GPIO 39, which buttons_begin() logs as "button 3".
// The other two still force a reseed.
#define REFRESH_BUTTON     (2)

static const int BUTTON_PINS[BUTTON_COUNT] = { BUTTON_1, BUTTON_2, BUTTON_3 };
static bool      s_btn_ok[BUTTON_COUNT];
static uint8_t   s_btn_level[BUTTON_COUNT];
static uint32_t  s_btn_edge_ms[BUTTON_COUNT];

static void buttons_begin(void)
{
    bool stable[BUTTON_COUNT];
    for (int i = 0; i < BUTTON_COUNT; i++) {
        pinMode(BUTTON_PINS[i], INPUT);
        stable[i] = true;
    }

    const uint32_t until = millis() + BUTTON_PROBE_MS;
    while ((int32_t)(millis() - until) < 0) {
        for (int i = 0; i < BUTTON_COUNT; i++) {
            if (digitalRead(BUTTON_PINS[i]) != HIGH) {
                stable[i] = false;
            }
        }
        delay(2);
    }

    for (int i = 0; i < BUTTON_COUNT; i++) {
        s_btn_ok[i] = stable[i];
        s_btn_level[i] = HIGH;
        s_btn_edge_ms[i] = 0;
        Serial.printf("button %d (GPIO %2d): %s\n", i + 1, BUTTON_PINS[i],
                      stable[i] ? "ready" : "did not hold high - ignored");
    }
}

// Bitmask of the usable buttons that went down on this poll, bit i for
// BUTTON_PINS[i]. One bool could not tell the three apart, and they no longer
// all mean the same thing.
static uint32_t buttons_went_down(void)
{
    uint32_t hits = 0;
    const uint32_t now = millis();
    for (int i = 0; i < BUTTON_COUNT; i++) {
        if (!s_btn_ok[i]) {
            continue;
        }
        const uint8_t level = digitalRead(BUTTON_PINS[i]) ? HIGH : LOW;
        if (level == s_btn_level[i]) {
            continue;
        }
        if ((uint32_t)(now - s_btn_edge_ms[i]) < BUTTON_DEBOUNCE_MS) {
            continue;   // still bouncing
        }
        s_btn_edge_ms[i] = now;
        s_btn_level[i] = level;
        if (level == LOW) {
            hits |= 1u << i;
        }
    }
    return hits;
}

// ---------------------------------------------------------------------------
// Life's own clock
//
// Split out from loop() so that a transit can keep the simulation running
// without re-reading the verdict: once a run is over it stays over, and the
// caller that already acted on it owns the reseed.
// ---------------------------------------------------------------------------
static int  s_last_live = 0;
static const char *s_last_reason = "";

// Advances Life if its interval has elapsed. Returns true when the run is
// finished - died out, fell into a cycle, or hit the generation cap.
static bool life_tick(void)
{
    static uint32_t last_gen_ms = 0;
    const uint32_t now = millis();
    if ((uint32_t)(now - last_gen_ms) < GENERATION_INTERVAL_MS) {
        return false;
    }
    last_gen_ms = now;

    s_last_live = step_life();
    s_generation++;

    const uint32_t hash = hash_grid();
    const bool looping = hash_seen_recently(hash);
    push_hash(hash);

    const bool dead  = (s_last_live == 0);
    const bool stuck = (s_generation >= MAX_GENERATIONS_NO_LOOP);
    if (!dead && !looping && !stuck) {
        return false;
    }
    s_last_reason = dead ? "died-out" : looping ? "cycle-detected" : "max-generations";
    return true;
}

// ---------------------------------------------------------------------------
// Entrances and exits
//
// A reseed is a scene change, so it is staged as one: the spent cube drifts out
// of frame, the panel is wiped with nothing on it, and the next one drifts back
// in. Wiping an empty panel is the point - the wipe no longer happens over the
// top of a cube, so it reads as scenery rather than as a fault.
//
// Nothing about the motion changes for a transit. Same drift speed, same spin
// rates, and Life still stepping on its own clock the whole way. The only thing
// that changes is that the walls stop catching the cube, so it carries on the
// heading it already had and leaves. It comes back on from the opposite side,
// unchanged, which reads as the cube having gone round the back of the panel.
//
// The cost is time: at 34 px/s across and 16 px/s down, clearing the panel
// takes the better part of half a minute. That is the whole point of doing it
// this way rather than accelerating the cube off, and the simulation carries on
// running throughout, so it is not dead air.
// ---------------------------------------------------------------------------

// How far past the edge the centre has to be for the cube to be fully hidden.
#define OFFSTAGE (CUBE_RADIUS + EDGE_MAX + 4.0f)

// Only a backstop against a logic error; the drift always clears the panel.
#define TRANSIT_TIMEOUT_MS (90000)

static uint32_t s_wipe_ms = 0, s_exit_ms = 0, s_enter_ms = 0;

static bool cube_offstage(void)
{
    return s_pos_x < -OFFSTAGE || s_pos_x > EPD_WIDTH  + OFFSTAGE
        || s_pos_y < -OFFSTAGE || s_pos_y > EPD_HEIGHT + OFFSTAGE;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// Let go of the walls and carry on until the cube has left the panel.
static void exit_stage(void)
{
    s_arriving = false;
    s_free_x = s_free_y = true;

    const uint32_t t0 = millis();
    const uint32_t give_up = millis() + TRANSIT_TIMEOUT_MS;
    while (!cube_offstage() && (int32_t)(millis() - give_up) < 0) {
        life_tick();        // the simulation keeps running on its way out
        animate_frame();
    }
    s_exit_ms = millis() - t0;
}

// Put the cube back on the far side, on the same heading at the same speed.
// Whichever axis went off-panel wraps; an axis that is still mid-panel is pulled
// back inside its bounce range first - a silent nudge, since the cube is
// entirely off-screen at this point - so it does not arrive already clipping a
// wall.
static void wrap_to_far_side(void)
{
    if      (s_pos_x < -OFFSTAGE)              { s_pos_x = EPD_WIDTH + OFFSTAGE; s_free_x = true; }
    else if (s_pos_x > EPD_WIDTH + OFFSTAGE)   { s_pos_x = -OFFSTAGE;            s_free_x = true; }
    else    { s_pos_x = clampf(s_pos_x, BOUNCE_LO_X, BOUNCE_HI_X); s_free_x = false; }

    if      (s_pos_y < -OFFSTAGE)              { s_pos_y = EPD_HEIGHT + OFFSTAGE; s_free_y = true; }
    else if (s_pos_y > EPD_HEIGHT + OFFSTAGE)  { s_pos_y = -OFFSTAGE;             s_free_y = true; }
    else    { s_pos_y = clampf(s_pos_y, BOUNCE_LO_Y, BOUNCE_HI_Y); s_free_y = false; }
}

// Boot has no previous exit to come back from, so the first cube is simply
// placed off one of the side edges. Sides rather than top or bottom because the
// horizontal drift is twice the vertical one, and a first appearance that takes
// twenty seconds to finish arriving is a poor start.
static void place_offstage(void)
{
    s_vel_x = (esp_random() & 1) ? DRIFT_X_PX_S : -DRIFT_X_PX_S;
    s_vel_y = (esp_random() & 1) ? DRIFT_Y_PX_S : -DRIFT_Y_PX_S;

    s_pos_x = (s_vel_x > 0.0f) ? -OFFSTAGE : (EPD_WIDTH + OFFSTAGE);
    s_pos_y = BOUNCE_LO_Y + (BOUNCE_HI_Y - BOUNCE_LO_Y)
                          * ((esp_random() % 1000) / 1000.0f);
    s_free_x = true;    // drifting in
    s_free_y = false;   // already in range, so its wall is live
}

// Drift in from wherever the cube is sitting off-panel, running as normal.
// update_motion() hands each axis its wall back as that axis comes into range;
// the arrival is over once both have taken it.
static void enter_stage(void)
{
    s_arriving = true;
    s_last_frame_us = 0;   // the wipe just ate five seconds of wall clock

    const uint32_t t0 = millis();
    const uint32_t give_up = millis() + TRANSIT_TIMEOUT_MS;
    while ((s_free_x || s_free_y) && (int32_t)(millis() - give_up) < 0) {
        life_tick();        // already running as it drifts on
        animate_frame();
    }
    s_arriving = false;
    s_free_x = s_free_y = false;
    s_enter_ms = millis() - t0;
}

// `full` picks the black-and-back dissolve over the white-only flood. Reseeds
// follow WHITE_ONLY_REFRESH; boot always asks for the full one.
static void wipe_panel(bool full)
{
    const uint32_t t0 = millis();
    if (full) {
        epd_wipe(WIPE_HOLD, WIPE_DARK_SETTLE, WIPE_LIGHT_SETTLE, DWELL);
    } else {
        epd_deghost(1, 0, WHITE_REFRESH_PASSES, DWELL);   // one cycle, no darken passes
    }
    s_wipe_ms = millis() - t0;
    r1_clear(s_shown, false);
    s_debt_rows = 0;   // the wipe paid off everything outstanding
}

// ---------------------------------------------------------------------------
// On-demand de-ghost
//
// A refresh where the cube stands, without ending the run. The reseed clear is
// a dissolve because it is scenery; this one is a repair someone asked for, so
// it wants to be over quickly - flood passes rather than a dithered dissolve,
// swinging the whole panel black and back twice.
//
// Life, the cube's position and its spin all carry straight on; only the panel
// changes. The next frame then redraws from a white s_shown, so it is one
// full-band push rather than a narrow diff - a single expensive frame landing
// in the current fps window. One frame in thousands does not move the average,
// so the counters are deliberately left running rather than reset: throwing the
// window away would cost more than the frame does.
// ---------------------------------------------------------------------------
// 2 x (4 darken + 10 lighten) = 28 flood passes, which measures 541 ms.
#define REFRESH_CYCLES       (2)
#define REFRESH_DARK_PASSES  (4)
#define REFRESH_LIGHT_PASSES (10)

static void refresh_now(void)
{
    const uint32_t t0 = millis();
    epd_deghost(REFRESH_CYCLES, REFRESH_DARK_PASSES, REFRESH_LIGHT_PASSES, DWELL);

    // The panel holds nothing now, so the next diff has to redraw the lot: say
    // so, and drop the debt the flood just paid off.
    r1_clear(s_shown, false);
    s_debt_rows = 0;

    // The flash ate a chunk of wall clock. Without this the next frame would
    // integrate that gap as elapsed time and jump the cube several pixels.
    s_last_frame_us = 0;

    Serial.printf("refresh: %lu ms at gen=%lu live=%d\n",
                  (unsigned long)(millis() - t0),
                  (unsigned long)s_generation, live_cells());
}

static void reset_life(void)
{
    seed_grid(SEED_DENSITY_PCT);
    s_generation = 0;
    memset(s_hash_ring, 0, sizeof(s_hash_ring));
    s_hash_ring_pos = 0;
}

void setup(void)
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\nLilyGo T5 4.7 - Game of Life on a bouncing 3D cube (e-paper)");

    // epd_base_init() rather than epd_init(): the latter also allocates a 64 KB
    // greyscale conversion LUT and a ~30 KB row queue, both of which belong to
    // the slow greyscale path we never use.
    epd_base_init(EPD_WIDTH);
    epd_fast_init();

    s_fb    = (uint8_t *)heap_caps_malloc(R1_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_shown = (uint8_t *)heap_caps_malloc(R1_FB_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_fb || !s_shown) {
        Serial.println("FATAL: framebuffer allocation failed");
        while (true) {
            delay(1000);
        }
    }
    r1_clear(s_fb, false);
    r1_clear(s_shown, false);

    // The trail cleanup is an improvement, not a requirement: if the debt plane
    // will not fit, say so and carry on without it rather than refusing to run.
    s_debt = (uint8_t *)heap_caps_malloc((size_t)DEBT_ROWS * EPD_ROW_BYTES,
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_debt) {
        memset(s_debt, 0, (size_t)DEBT_ROWS * EPD_ROW_BYTES);
    } else {
        Serial.println("warning: no room for the relight plane - trails will linger");
    }

    Serial.printf("panel %dx%d, %u bytes per frame, %u bytes free after 3 planes\n",
                  EPD_WIDTH, EPD_HEIGHT, (unsigned)R1_FB_BYTES,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    buttons_begin();
    build_topology();

    // Power stays on for the whole run. epd_poweron() carries settling delays
    // that would dwarf a 60 ms frame if it were called per update.
    epd_poweron();

    // Same staging as a reseed, minus the exit: wipe whatever the last sketch
    // left on the panel, then let the first cube drift on from off-frame.
    wipe_panel(true);   // whatever the last sketch left needs a real reset
    reset_life();
    place_offstage();
    enter_stage();
}

void loop(void)
{
    // Life still advances on its own clock whatever the buttons do. They only
    // decide whether the panel gets a clean-up, or whether this run is over.
    // refresh_now() zeroes s_last_frame_us and nothing between here and
    // animate_frame() reads it, so the flash stays out of the cube's motion.
    const uint32_t hits = buttons_went_down();
    if (hits & (1u << REFRESH_BUTTON)) {
        refresh_now();
    }
    const bool forced = (hits & ~(1u << REFRESH_BUTTON)) != 0;
    const bool finished = life_tick();

    if (forced || finished) {
        if (forced) {
            // The button takes the label whatever Life was doing: the reseed
            // happened because someone asked for it.
            s_last_live = live_cells();
            s_last_reason = "button";
        }

        // Snapshot the run's animation figures before the transit dilutes them.
        // An off-panel cube costs almost nothing to draw - the diff band is
        // empty, so present() returns immediately - and a transit is tens of
        // seconds of those frames. Averaging them in reports a frame rate the
        // animation never actually ran at.
        const uint32_t gen = s_generation, frames = s_frames;
        const uint32_t render_us = s_render_us, push_us = s_push_us;
        const int live = s_last_live;
        const char *reason = s_last_reason;

        exit_stage();       // carries on exactly as it was until it is off-panel
        wipe_panel(!WHITE_ONLY_REFRESH);
        reset_life();
        wrap_to_far_side();
        enter_stage();      // and drifts back on, already running

        // Logged after the sequence rather than before it, so the transit
        // timings belong to the transit that just happened instead of the one
        // before. Logged outside the frame loop either way: a line at 115200
        // baud is milliseconds of blocking, a visible hitch at 20 fps.
        Serial.printf("gen=%lu live=%d reason=%s | %lu frames, %.1f fps "
                      "(render %.1f ms, push %.1f ms) | exit %.1fs wipe %.1fs enter %.1fs\n",
                      (unsigned long)gen, live, reason, (unsigned long)frames,
                      frames ? 1e6f * frames / (float)(render_us + push_us) : 0.0f,
                      frames ? render_us / 1000.0f / frames : 0.0f,
                      frames ? push_us / 1000.0f / frames : 0.0f,
                      s_exit_ms / 1000.0f, s_wipe_ms / 1000.0f, s_enter_ms / 1000.0f);

        // Discard the transit's frames; the next window starts on-panel.
        s_frames = 0; s_render_us = 0; s_push_us = 0;
    }

    animate_frame();
}
