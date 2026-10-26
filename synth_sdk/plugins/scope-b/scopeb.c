/* "scope-b" - a compact oscilloscope for the OP-1 FX slot.
 *
 * It is a pass-through effect: the audio is not changed, it is only watched.  The synth (or drum / master)
 * signal is captured in a small ring buffer and drawn on the FX page.
 *
 *   blue  : retrig type   auto / ascend / descend / instant
 *   green : vertical range in dB below full scale (0 .. 48 dB), i.e. the display gain
 *   white : horizontal length in Hz (20 Hz .. 4 kHz): one period of this frequency fills the screen
 *   red   : hold.  Turn right (or press the red encoder) to freeze the trace, left / press again to run
 *
 * Retrig: "auto" starts on the most recent zero crossing (either slope), "ascend" on a rising crossing,
 * "descend" on a falling crossing, "instant" shows the latest samples without triggering.  A small
 * hysteresis keeps noise around zero from retriggering the trace.
 *
 * The screen: a small settings strip at the top, a 4 x 4 grid and the waveform in the main area.
 * Everything is drawn in the GUI thread; the audio interrupt only writes the ring buffer and a tiny
 * trigger ring (no per-sample division, no float, no libc).
 *
 * Copyright (C) 2026 nvmsocool. GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */
#include "op1_plugin.h"

#define SCOPE_RING   8192u                  /* captured samples per FX position (power of two) */
#define SCOPE_MASK   (SCOPE_RING - 1u)
#define SCOPE_TRIG   512u                   /* recent zero crossings per FX position (power of two) */
#define SCOPE_TMASK  (SCOPE_TRIG - 1u)
#define SCOPE_MAXWIN 2304                   /* 44100 / 20 Hz, rounded up */
#define SCOPE_HYST   64                     /* trigger deadband, ~ -54 dBFS */

/* screen geometry (320 x 160, the visible area ends at about y 140) */
#define SC_X0        6
#define SC_X1        314
#define SC_Y0        22
#define SC_Y1        134
#define SC_CX        ((SC_X0 + SC_X1) / 2)
#define SC_CY        ((SC_Y0 + SC_Y1) / 2)
#define SC_W         (SC_X1 - SC_X0)
#define SC_H         (SC_Y1 - SC_Y0)
#define SC_HALF_H    (SC_H / 2)
#define SC_COLS      (SC_W + 1)

typedef struct {
    int16_t ring[SCOPE_RING];
    volatile uint32_t w;                    /* next ring slot */
    volatile uint32_t total;                /* samples written since init (audio clock) */
    volatile int16_t last;                  /* last sample, for crossing detection */
    volatile uint32_t trig[SCOPE_TRIG];     /* absolute sample index of a crossing */
    volatile uint8_t trig_dir[SCOPE_TRIG];  /* 1 = rising, 0 = falling */
    volatile uint32_t tw;                   /* next trigger slot */
    volatile uint32_t tcount;               /* valid trigger entries, <= SCOPE_TRIG */
    uint8_t low_seen;                       /* been below -HYST since the last rising trigger */
    uint8_t high_seen;                      /* been above +HYST since the last falling trigger */
} scope_t;

static scope_t st[3];

/* display snapshot, per FX position (GUI thread) */
static int16_t disp[3][SCOPE_MAXWIN];
static uint16_t disp_len[3];
static uint8_t disp_valid[3];
static uint8_t disp_mode[3], disp_db[3];
static uint16_t disp_hz[3];
static uint16_t disp_gain[3];               /* Q8 gain */

/* dB below full scale -> Q8 linear gain: 256 = 0 dB, 64304 = 48 dB.  Max product with int16 still fits
 * in int32, so the GUI can apply it without 64-bit maths. */
static const uint16_t gain_q8[49] = {
    256,   287,   322,   362,   406,   455,   511,   573,   643,   722,
    810,   908,  1019,  1144,  1283,  1440,  1615,  1812,  2033,  2282,
   2560,  2872,  3223,  3616,  4057,  4552,  5108,  5731,  6430,  7215,
   8095,  9083, 10192, 11435, 12830, 14396, 16153, 18123, 20335, 22816,
  25600, 28724, 32228, 36161, 40573, 45524, 51079, 57311, 64304
};

/* 20 Hz .. 4 kHz, one period fills the screen.  Higher = shorter window (more zoom). */
static const uint16_t hz_tab[] = {
     20,  25,  30,  40,  50,  60,  80, 100, 120, 150, 200, 250,
    300, 400, 500, 600, 800, 1000, 1200, 1500, 2000, 2500, 3000, 4000
};
#define HZ_N ((int)(sizeof hz_tab / sizeof hz_tab[0]))

static void scope_init(int pos)
{
    scope_t *s = &st[pos % 3];
    for (uint32_t i = 0; i < SCOPE_RING; i++) s->ring[i] = 0;
    s->w = 0; s->total = 0; s->last = 0;
    s->tw = 0; s->tcount = 0;
    s->low_seen = 0; s->high_seen = 0;
    disp_valid[pos % 3] = 0;
    disp_len[pos % 3] = 0;
}

static void scope_render(int pos, int16_t *buf, const int16_t *p)
{
    scope_t *s = &st[pos % 3];
    uint32_t w = s->w, total = s->total, tw = s->tw, tc = s->tcount;
    int16_t prev = s->last;
    uint8_t low = s->low_seen, high = s->high_seen;

    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        int16_t x = buf[n];
        s->ring[w] = x;

        /* zero-crossing trigger with a small deadband: a rising edge only counts after the signal has
         * been below -HYST, a falling edge only after it has been above +HYST. */
        if (prev < 0 && x >= 0 && low) {
            s->trig[tw] = total;
            s->trig_dir[tw] = 1;
            tw = (tw + 1) & SCOPE_TMASK;
            if (tc < SCOPE_TRIG) tc++;
            low = 0;
        } else if (prev >= 0 && x < 0 && high) {
            s->trig[tw] = total;
            s->trig_dir[tw] = 0;
            tw = (tw + 1) & SCOPE_TMASK;
            if (tc < SCOPE_TRIG) tc++;
            high = 0;
        }
        if (x < -SCOPE_HYST) low = 1;
        if (x >  SCOPE_HYST) high = 1;

        prev = x;
        w = (w + 1) & SCOPE_MASK;
        total++;
    }
    s->w = w; s->total = total; s->tw = tw; s->tcount = tc; s->last = prev;
    s->low_seen = low; s->high_seen = high;
    (void)p;
}

const op1_fx_t FX_scope_b = { scope_init, scope_render,
                              { OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_LATCH }, 0 };

/* ======================= screen (GUI thread) ======================= */
#include "op1_synth.h"

#ifdef OP1_HOST
/* The audio harness links the plugin without screen_harness.c, so it has no text stubs.  These weak
 * definitions keep `harness/audio.py` working; screen_harness.c's strong ones are used for previews. */
__attribute__((weak)) void *host_text_ctor(void *obj, int32_t x_q10, int32_t y, int32_t size, const uint8_t *rgba)
{ (void)x_q10; (void)y; (void)size; (void)rgba; return obj; }
__attribute__((weak)) void host_text_set(void *t, const char *s) { (void)t; (void)s; }
__attribute__((weak)) void host_text_draw(void *t, void *r, const int32_t *m9) { (void)t; (void)r; (void)m9; }
#endif

static const uint8_t c_blue[4]  = { 0x4D, 0x9E, 0xFF, 0xFF };
static const uint8_t c_green[4] = { 0x00, 0xED, 0x95, 0xFF };
static const uint8_t c_white[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
static const uint8_t c_red[4]   = { 0xFF, 0x3A, 0x5D, 0xFF };
static const uint8_t c_grey[4]  = { 0x6F, 0x71, 0x73, 0xFF };
static const uint8_t c_dim[4]   = { 0x3A, 0x3B, 0x3D, 0xFF };

enum { TX_MODE, TX_RANGE, TX_HZ, TX_HOLD, TX_N };
static uint32_t tx[TX_N][OP1_TEXT_SIZE / 4];
static char shown[TX_N][16];
static uint8_t tx_made;

static const char *const mode_name[4] = { "auto", "asc", "desc", "inst" };

static void ln(void *r, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t w, const uint8_t *c)
{
    if (x0 == x1 && y0 == y1) return;                   /* the OP-1 drops zero-length lines */
    op1_pt a = { x0 << 10, y0 << 10 }, b = { x1 << 10, y1 << 10 };   /* px -> Q10 */
    op1_raster_line(r, &a, &b, w, c);
}

static char *put(char *p, const char *s) { while (*s) *p++ = *s++; *p = 0; return p; }

static char *put_num(char *p, int32_t v)                /* unsigned decimal */
{
    char t[12]; int n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) *p++ = t[--n];
    *p = 0;
    return p;
}

static void set_text(void *r, int i, const char *s)
{
    static const int32_t ident[9] = { 1024, 0, 0, 0, 1024, 0, 0, 0, 1024 };
    if (!s || !s[0]) return;
    int same = 1;
    for (int j = 0; j < 15; j++) {
        if (shown[i][j] != s[j]) { same = 0; break; }
        if (!s[j]) break;
    }
    if (!same) {
        op1_text_set(tx[i], s);
        int j = 0;
        for (; j < 15 && s[j]; j++) shown[i][j] = s[j];
        shown[i][j] = 0;
    }
    op1_text_draw(tx[i], r, ident);
}

static void make_text(void)
{
    if (tx_made) return;
    /* settings strip: one value per knob, in knob colour order */
    op1_text_ctor(tx[TX_MODE],  OP1_Q10(8),   2, 11, c_blue);
    op1_text_ctor(tx[TX_RANGE], OP1_Q10(88),  2, 11, c_green);
    op1_text_ctor(tx[TX_HZ],    OP1_Q10(168), 2, 11, c_white);
    op1_text_ctor(tx[TX_HOLD],  OP1_Q10(248), 2, 11, c_red);
    tx_made = 1;
}

/* newest trigger of the requested kind that is at least `win` samples old, or 0xFFFFFFFF */
static uint32_t trigger_start(const scope_t *s, uint32_t total, uint32_t win, int mode)
{
    uint32_t tw = s->tw, tc = s->tcount;
    if (tc > SCOPE_TRIG) tc = SCOPE_TRIG;
    for (uint32_t i = 1; i <= tc; i++) {
        uint32_t j = (tw - i) & SCOPE_TMASK;
        uint32_t idx = s->trig[j];
        int32_t age = (int32_t)(total - idx);           /* signed: a stale total must not wrap */
        if (age < (int32_t)win) continue;               /* too recent: the trace would run off the end */
        if (age >= (int32_t)SCOPE_RING) continue;       /* too old: the ring has overwritten it */
        if (mode == 0) return idx;                      /* auto: either slope */
        if (mode == 1 && s->trig_dir[j]) return idx;    /* ascend */
        if (mode == 2 && !s->trig_dir[j]) return idx;   /* descend */
    }
    return 0xFFFFFFFFu;
}

/* copy `win` samples starting at absolute index `start` into the display buffer */
static void snapshot(int pos, const scope_t *s, uint32_t start, uint32_t win, uint32_t total)
{
    int16_t *d = disp[pos];
    for (uint32_t i = 0; i < win; i++) {
        uint32_t idx = start + i;
        int16_t v = 0;
        if ((int32_t)(total - idx) > 0) v = s->ring[idx & SCOPE_MASK];   /* past the write head: silence */
        d[i] = v;
    }
}

static void grid(void *r)
{
    for (int i = 0; i <= 4; i++) {
        int32_t x = SC_X0 + SC_W * i / 4;
        int32_t y = SC_Y0 + SC_H * i / 4;
        ln(r, x, SC_Y0, x, SC_Y1, 0x400, c_dim);        /* vertical */
        ln(r, SC_X0, y, SC_X1, y, 0x400, c_dim);        /* horizontal */
    }
    /* centre axes a little brighter */
    ln(r, SC_CX, SC_Y0, SC_CX, SC_Y1, 0x500, c_grey);
    ln(r, SC_X0, SC_CY, SC_X1, SC_CY, 0x500, c_grey);
    /* frame */
    ln(r, SC_X0, SC_Y0, SC_X1, SC_Y0, 0x600, c_grey);
    ln(r, SC_X0, SC_Y1, SC_X1, SC_Y1, 0x600, c_grey);
    ln(r, SC_X0, SC_Y0, SC_X0, SC_Y1, 0x600, c_grey);
    ln(r, SC_X1, SC_Y0, SC_X1, SC_Y1, 0x600, c_grey);
}

/* one vertical min/max bar per screen column, plus a centre trace between columns */
static void wave(void *r, int pos)
{
    int32_t win = disp_len[pos];
    if (!disp_valid[pos] || win < 2) return;
    int32_t gain = disp_gain[pos];
    int32_t prev_x = 0, prev_y = 0;

    for (int32_t c = 0; c < SC_COLS; c++) {
        int32_t i0 = (int32_t)((uint32_t)c * (uint32_t)win / (uint32_t)SC_COLS);
        int32_t i1 = (int32_t)((uint32_t)(c + 1) * (uint32_t)win / (uint32_t)SC_COLS);
        if (i1 <= i0) i1 = i0 + 1;
        if (i1 > win) i1 = win;

        int32_t mn = 32767, mx = -32768;
        for (int32_t i = i0; i < i1; i++) {
            int32_t v = ((int32_t)disp[pos][i] * gain) >> 8;
            if (v > 32767) v = 32767; else if (v < -32767) v = -32767;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        int32_t x = SC_X0 + c;
        int32_t yt = SC_CY - ((mx * SC_HALF_H) >> 15);
        int32_t yb = SC_CY - ((mn * SC_HALF_H) >> 15);
        if (yb <= yt) { yt--; yb++; }                   /* a flat trace still shows a dot */
        ln(r, x, yt, x, yb, 0x500, c_green);

        int32_t mid = (mx + mn) / 2;
        int32_t ym = SC_CY - ((mid * SC_HALF_H) >> 15);
        if (c) ln(r, prev_x, prev_y, x, ym, 0x800, c_green);
        prev_x = x; prev_y = ym;
    }
}

static void scope_screen_init(void *scr) { (void)scr; }

static void scope_screen_update(void *scr, int dt)
{
    make_text();
    int pos = op1_fx_screen_pos() % 3;
    scope_t *s = &st[pos];
    int16_t k[4];
    op1_fx_knobs_live(k);

    int mode = k[0] >> 13;                              /* 0..3 */
    if (mode > 3) mode = 3;
    int db = (k[1] * 49) >> 15;                         /* 0..48 dB below full scale */
    if (db > 48) db = 48;
    int hi = (k[2] * HZ_N) >> 15;
    if (hi >= HZ_N) hi = HZ_N - 1;
    int hz = hz_tab[hi];
    int win = OP1_SAMPLE_RATE / hz;
    if (win < 2) win = 2;
    if (win > SCOPE_MAXWIN) win = SCOPE_MAXWIN;

    int held = op1_fx_page[pos][3] || k[3] >= 16384;

    if (!held || !disp_valid[pos]) {
        uint32_t total = s->total;
        uint32_t start = 0xFFFFFFFFu;
        if (mode != 3) start = trigger_start(s, total, (uint32_t)win, mode);
        if (start == 0xFFFFFFFFu) start = total > (uint32_t)win ? total - (uint32_t)win : 0;
        snapshot(pos, s, start, (uint32_t)win, total);
        disp_len[pos] = (uint16_t)win;
        disp_mode[pos] = (uint8_t)mode;
        disp_db[pos] = (uint8_t)db;
        disp_hz[pos] = (uint16_t)hz;
        disp_gain[pos] = gain_q8[db];
        disp_valid[pos] = 1;
    }

    int dmode = disp_mode[pos];
    int ddb = disp_db[pos];
    int dhz = disp_hz[pos];
    int dheld = held;

    op1_screen_draw(scr, dt);
    void *r = op1_raster();
    grid(r);
    wave(r, pos);

    /* settings strip (frozen values while hold is on) */
    char v[16];
    char *q = put(v, "trig ");
    put(q, mode_name[dmode]);
    set_text(r, TX_MODE, v);
    q = put(v, "rng ");
    if (ddb) q = put(q, "-");
    q = put_num(q, (int32_t)ddb);
    put(q, "db");
    set_text(r, TX_RANGE, v);
    q = put(v, "hz ");
    put_num(q, (int32_t)dhz);
    set_text(r, TX_HZ, v);
    set_text(r, TX_HOLD, dheld ? "hold on" : "hold off");
}

const op1_screen_t SCREEN_scope_b = { scope_screen_init, scope_screen_update };
