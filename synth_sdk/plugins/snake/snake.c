/* "snake" - the game Snake as a custom FX.
 *
 * The screen is the game. The keys steer the snake while the synth plays as usual (the FX does not swallow them):
 * a key you press turns the snake by 90 degrees depending on the key you pressed before it: a higher key turns it
 * clockwise, a lower key counter-clockwise. The same key twice in a row does nothing. Two quick keys that add up to a
 * 180 over the previous leg are ignored until the snake has moved a little (TURN_GAP), so a same-frame double tap
 * cannot double it back onto its own tail, while a 180 timed after that short leg still works. The first
 * key starts the game, and after a game over a key held for 1.5 s starts a new one.
 *
 * Sound: the game over plays a low tone, and the whole FX bus goes through a delay and then a reverb (both after the
 * synth, which stays dry otherwise). Knobs:
 *   blue  : reverb time (decay)
 *   green : reverb mix
 *   white : delay time (30 ms .. 1.1 s)
 *   red   : delay mix (the orange knob)
 * The screen shows the four knob levels as 1 px lines just above the wall: a dot in the middle of the knob's quarter
 * at 0, growing to both sides until it fills that quarter at the top.
 *
 * The snake is one line. The food is a 6x6 square. Eating it: the food disappears, the snake grows a little longer,
 * +1 point, the snake gets a little faster, a new food appears somewhere else.
 * Hitting yourself or a wall ends the game: the screen shows the score.
 *
 * Custom FX (docs/51): the game runs in the screen update (GUI thread, dt in ms) and only while its screen is shown.
 * Keys are observed non-destructively in the KeyHandler queue (s_key_queue, 0x01DD5044, u16 code | pressed << 8):
 * the game never consumes them, so the synth still gets every note. Positions are Q10 pixels of the 320 x 160
 * screen. The snake is a polyline of corner points (axis-aligned segments); the head moves in steps of at most 2 px
 * so nothing is skipped at high speed. The audio side plays the game-over tone in the FX bus and runs the delay and
 * reverb there. Delay lines live in this FX position's scratch buffer; the game itself is shared by the visible FX page.
 * Copyright (C) 2026 nick (nvmsocool). GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */
#include "op1_plugin.h"
#include "seq/seq_text.h"                 /* stroke-font text (as arc / palme / mixtur) */

#define PX(v)        ((int32_t)((v) * 1024))

/* ---- game constants ---- */
#define MAXV         96                   /* corner points of the snake (a turn with no room left is ignored) */
#define WALL_L       PX(4.5)              /* the head's centre must stay inside (the wall is drawn at 3 / 317, 3 / 137) */
#define WALL_R       PX(315.5)
#define WALL_T       PX(4.5)
#define WALL_B       PX(135.5)
#define BODY_W       PX(3)                /* line width */
#define START_LEN    PX(40)
#define GROW         PX(14)               /* longer per food */
#define SPEED0       50                   /* px / s */
#define SPEED_UP     1                    /* faster per food */
#define SPEED_MAX    260
#define NECK         PX(8)                /* body closer than this along the line to the head cannot be hit */
#define HIT_R        PX(2.5)              /* head to body distance that counts as a hit */
#define FOOD_HALF    PX(3)                /* 6 x 6 square */
#define STEP_MAX     PX(2)
#define TURN_GAP     PX(4)                /* a leg must be this long before a turn can double back onto it */
#define RESTART_MS   1500                 /* hold a key this long on the end screen */

/* ---- audio constants ---- */
#define NPOS         3                    /* synth / drum / master */

enum { S_READY, S_PLAY, S_OVER };
enum { D_UP, D_LEFT, D_DOWN, D_RIGHT };   /* clockwise is (d + 3) & 3, counter-clockwise (d + 1) & 3 */
static const int8_t DX[4] = { 0, -1, 0, 1 }, DY[4] = { -1, 0, 1, 0 };

/* key codes of the 24 keyboard keys in the KeyHandler queue (seqdev/ms_fw.c kb_keys) */
static const uint8_t KEY_CODE[24] = {
    39, 13, 40, 14, 41, 15, 42, 28, 32, 29, 22, 30, 31, 23, 48, 24, 49, 25, 50, 51, 26, 52, 27, 53
};

/* ---- game state ---- */
typedef struct {
    volatile int state;               /* read by the audio render */
    volatile int init;                /* 0 = the screen must reset the game (audio init asked for it) */
    op1_pt   p[MAXV];                 /* p[0] tail end ... p[n-1] head */
    int      n;
    int      dir;
    int32_t  len, total;              /* wanted / actual length of the line, Q10 px */
    int32_t  speed;                   /* px / s */
    op1_pt   food;                    /* centre */
    int      score;
    uint32_t rng;
    uint32_t tick;                    /* ms since the screen was first shown: seeds the random numbers */
    int      prev_key;                /* the key before (0..23) */
    int      prev_dir;                /* the direction of the leg before the current one, -1 = none */
    int      have_prev;
    int      hold_ms;                 /* how long a key has been held on the end screen */
    int      hold_armed;              /* a key was pressed after the game over */
} game_t;
static game_t g;

/* keys currently down (bit 0..23), written by the screen thread */
static volatile uint32_t sn_keys;

static uint32_t rnd(void) { g.rng ^= g.rng << 13; g.rng ^= g.rng >> 17; g.rng ^= g.rng << 5; return g.rng; }
static int32_t iabs(int32_t a) { return a < 0 ? -a : a; }

/* game -> audio: a game over happened (the audio render starts the low tone and clears it) */
static volatile uint8_t over_evt[NPOS];

/* distance (squared, Q6 px) from point (x, y) to the axis-aligned segment a-b, and where on the segment the closest
 * point is (|closest - b|, Q10 px) */
static int32_t seg_dist2(op1_pt a, op1_pt b, int32_t x, int32_t y, int32_t *from_b)
{
    int32_t cx, cy;
    if (a.y == b.y) {                                   /* horizontal (also a point) */
        int32_t lo = a.x < b.x ? a.x : b.x, hi = a.x < b.x ? b.x : a.x;
        cx = x < lo ? lo : (x > hi ? hi : x); cy = a.y;
    } else {
        int32_t lo = a.y < b.y ? a.y : b.y, hi = a.y < b.y ? b.y : a.y;
        cx = a.x; cy = y < lo ? lo : (y > hi ? hi : y);
    }
    *from_b = iabs(cx - b.x) + iabs(cy - b.y);
    int32_t dx = (x - cx) >> 4, dy = (y - cy) >> 4;     /* Q6: squares stay below 2^31 */
    return dx * dx + dy * dy;
}

static int hit_self(void)
{
    op1_pt h = g.p[g.n - 1];
    int32_t acc = 0;                                    /* line length from the head to the near end of the segment */
    int32_t r6 = HIT_R >> 4;
    for (int i = g.n - 2; i >= 0; i--) {
        int32_t fb, d2 = seg_dist2(g.p[i], g.p[i + 1], h.x, h.y, &fb);
        if (d2 < r6 * r6 && acc + fb >= NECK) return 1;
        acc += iabs(g.p[i + 1].x - g.p[i].x) + iabs(g.p[i + 1].y - g.p[i].y);
    }
    return 0;
}

static int hit_wall(void)
{
    op1_pt h = g.p[g.n - 1];
    return h.x < WALL_L || h.x > WALL_R || h.y < WALL_T || h.y > WALL_B;
}

static int food_hit(void)
{
    op1_pt h = g.p[g.n - 1];
    int32_t r = FOOD_HALF + BODY_W / 2;
    return iabs(h.x - g.food.x) <= r && iabs(h.y - g.food.y) <= r;
}

static void spawn_food(void)
{
    for (int tries = 0; tries < 40; tries++) {
        op1_pt f = { PX(14) + (int32_t)(rnd() % 293u) * 1024, PX(14) + (int32_t)(rnd() % 113u) * 1024 };
        int ok = 1;
        for (int i = 0; i + 1 < g.n && ok; i++) {
            int32_t fb;
            if (seg_dist2(g.p[i], g.p[i + 1], f.x, f.y, &fb) < (PX(10) >> 4) * (PX(10) >> 4)) ok = 0;
        }
        g.food = f;
        if (ok) return;
    }
}

static void game_reset(void)
{
    g.p[0].x = PX(30); g.p[0].y = PX(80);
    g.p[1].x = PX(30) + START_LEN; g.p[1].y = PX(80);
    g.n = 2; g.dir = D_RIGHT;
    g.len = g.total = START_LEN;
    g.speed = SPEED0; g.score = 0;
    g.prev_dir = -1;
    g.have_prev = 0;
    g.hold_ms = 0; g.hold_armed = 0;
    spawn_food();
    g.state = S_READY;
    g.init = 1;
}

/* drop the tail until the line is as long as wanted */
static void trim_tail(void)
{
    int32_t excess = g.total - g.len;
    while (excess > 0 && g.n >= 2) {
        int32_t l0 = iabs(g.p[1].x - g.p[0].x) + iabs(g.p[1].y - g.p[0].y);
        if (l0 <= excess && g.n > 2) {
            for (int i = 1; i < g.n; i++) g.p[i - 1] = g.p[i];
            g.n--; excess -= l0; g.total -= l0;
        } else {
            if (excess > l0) excess = l0;
            if (g.p[1].x != g.p[0].x) g.p[0].x += (g.p[1].x > g.p[0].x ? excess : -excess);
            else                      g.p[0].y += (g.p[1].y > g.p[0].y ? excess : -excess);
            g.total -= excess; excess = 0;
        }
    }
}

/* turn into direction d (not the opposite one, not the same): a new corner at the head. `prev_dir` is the direction of
 * the leg before the current one; turning opposite to it is a 180 over two turns, allowed only once the current leg is
 * at least TURN_GAP long (otherwise the snake doubles back onto its own tail). Returns 1 = turned, 0 = not a turn,
 * -1 = a 180 blocked by the cooldown (on_key keeps the old key as the reference then). */
static int turn(int d)
{
    if (d == g.dir || d == ((g.dir + 2) & 3) || g.n >= MAXV) return 0;
    if (g.prev_dir >= 0 && d == ((g.prev_dir + 2) & 3)) {
        int32_t leg = iabs(g.p[g.n - 1].x - g.p[g.n - 2].x) +
                      iabs(g.p[g.n - 1].y - g.p[g.n - 2].y);
        if (leg < TURN_GAP) return -1;
    }
    g.p[g.n] = g.p[g.n - 1]; g.n++;
    g.prev_dir = g.dir;
    g.dir = d;
    return 1;
}

static void game_step(int dt, int pos)
{
    int32_t dist = g.speed * dt * 1024 / 1000;           /* Q10 px this frame */
    while (dist > 0 && g.state == S_PLAY) {
        int32_t s = dist < STEP_MAX ? dist : STEP_MAX;
        op1_pt *h = &g.p[g.n - 1];
        h->x += DX[g.dir] * s; h->y += DY[g.dir] * s;
        g.total += s; dist -= s;
        trim_tail();
        if (hit_wall() || hit_self()) {                  /* a held key starts the next game */
            g.state = S_OVER; g.have_prev = 0; g.hold_ms = 0; g.hold_armed = 0;
            over_evt[pos] = 1;
            return;
        }
        if (food_hit()) {
            g.score++;
            g.len += GROW;
            g.speed += SPEED_UP; if (g.speed > SPEED_MAX) g.speed = SPEED_MAX;
            spawn_food();
        }
    }
}

/* ---- keys: observe the KeyHandler queue without consuming it ---- */
#define KEYQ_BASE 0x01DD5044              /* u16 code | pressed << 8, 256 entries */
#define KEYQ_RD   0x01DD5244              /* read index (advanced by the stock KeyHandler) */

static int key_of_code[64];               /* queue code -> keyboard key 0..23, -1 for other keys */
static volatile uint32_t keyq_last;
static volatile int keyq_init;

/* a key was pressed (screen thread): start / restart, or turn by 90 degrees */
static void on_key(int key)
{
    if (g.state == S_OVER) {                                 /* the restart waits for RESTART_MS of key hold */
        g.prev_key = key; g.have_prev = 1; g.hold_armed = 1;
        return;
    }
    if (g.state != S_PLAY) {                                 /* first key: start playing */
        g.rng ^= g.tick * 2654435761u; if (!g.rng) g.rng = 1;
        g.state = S_PLAY;
        g.prev_key = key; g.have_prev = 1;
        return;
    }
    if (!g.have_prev) { g.prev_key = key; g.have_prev = 1; return; }
    if (key == g.prev_key) return;                           /* the same key */
    int r = turn(key > g.prev_key ? ((g.dir + 3) & 3) : ((g.dir + 1) & 3));
    if (r != -1) g.prev_key = key;                           /* a blocked 180 does not move the reference key */
}

static void key_map_init(void)
{
    for (int i = 0; i < 64; i++) key_of_code[i] = -1;
    for (int i = 0; i < 24; i++) key_of_code[KEY_CODE[i]] = i;
}

#ifndef OP1_HOST
/* take every event the KeyHandler has already consumed since the last look; the read index itself is not moved, so
 * the synth still plays the keys. A large gap means the ring wrapped: skip it and forget the held keys. */
static void keys_poll(int reset)
{
    if (!keyq_init) key_map_init();
    uint32_t rd = *(volatile uint32_t *)KEYQ_RD;
    if (reset || !keyq_init) { keyq_last = rd; keyq_init = 1; sn_keys = 0; return; }
    if ((uint32_t)((rd - keyq_last) & 0xFF) > 128) { keyq_last = rd; sn_keys = 0; return; }
    while (keyq_last != rd) {
        uint16_t ev = ((volatile uint16_t *)KEYQ_BASE)[keyq_last];
        keyq_last = (keyq_last + 1) & 0xFF;
        int code = ev & 0xFF, down = (ev >> 8) & 1;
        int k = code < 64 ? key_of_code[code] : -1;
        if (k < 0) continue;
        if (down) { sn_keys |= 1u << k; on_key(k); }
        else      sn_keys &= ~(1u << k);
    }
}
#else
static void keys_poll(int reset) { (void)reset; }
/* host test hook: feed a game key (0..23) directly, as if the key had been pressed/released */
void host_sn_key(int key, int down)
{
    if (key < 0 || key > 23) return;
    if (down) { sn_keys |= 1u << key; on_key(key); }
    else      sn_keys &= ~(1u << key);
}
int  host_sn_state(void) { return g.state; }
int  host_sn_dir(void) { return g.dir; }
int  host_sn_score(void) { return g.score; }
int  host_sn_head_x(void) { return g.p[g.n - 1].x; }
int  host_sn_head_y(void) { return g.p[g.n - 1].y; }
int  host_sn_len(void) { return g.n; }
void host_sn_place_food(int x, int y) { g.food.x = x; g.food.y = y; }
#endif

/* ---- audio: game-over tone -> delay -> reverb, in this FX position's bus ----
 *
 * Reverb: the "reverb" FX (fixed-point port of the Mutable Instruments Elements reverb, (c) 2014 Emilie Gillet,
 * MIT), with its diffusion and tone fixed at the middle of their range. Only the two parameters this effect exposes
 * stay on knobs: blue = time (decay), green = mix. Memory: RV_SIZE int16 at the start of the scratch buffer.
 *
 * Delay: own, white = time (30 ms .. 1.1 s on a square curve), red = mix, 0.37 feedback with a little damping in the
 * loop and a glided read pointer (changing the time bends the pitch instead of clicking). Memory: DL_SIZE int16
 * right after the reverb's lines; the line is cleared a block at a time from the render (init runs on the audio
 * thread, where 96 KB of writes are too much at once). */
#define RV_SIZE      32768                /* int16, 64 KB */
#define RV_MASK      (RV_SIZE - 1)
#define RV_KNEE      24576
/* ap1..ap4, dap1a, dap1b, del1, dap2a, dap2b, del2 (Elements lengths * 44100 / 32000) */
static const int16_t rv_len[10] = { 207, 295, 440, 726, 3007, 3707, 6203, 3480, 3028, 8698 };

typedef struct {
    int16_t *buf;
    int32_t wp, base[10];
    int32_t lp1, lp2;
    int32_t lfo_iir[2], lfo_y0[2], lfo_y1[2], lfo[2];    /* stmlib CosineOscillator (approximate), Q29; lfo Q15 0..1 */
} reverb_t;
static reverb_t rv[NPOS];

static inline int32_t c17(int32_t x) { return x < -65535 ? -65535 : (x > 65535 ? 65535 : x); }
/* |b| <= 32767: fits 32 bit. Truncates towards zero: >> alone biases the loop into a DC limit cycle and
 * rounding into a low-level one, magnitude truncation lets the tail die out */
static inline int32_t m15(int32_t a, int32_t b) { int32_t x = c17(a) * b; return x >= 0 ? x >> 15 : -(-x >> 15); }
static inline int16_t sat(int32_t x) { return (int16_t)(x < -32768 ? -32768 : (x > 32767 ? 32767 : x)); }
#define RVR(i, off) r->buf[(r->wp + r->base[i] + (off)) & RV_MASK]
#define TAIL(i) RVR(i, rv_len[i] - 1)

/* Interpolate(line, offset + amp * lfo) */
static inline int32_t interp(reverb_t *r, int i, int32_t off, int32_t amp, int32_t lfo)
{
    int32_t pos = (off << 15) + amp * lfo;                /* Q15 samples */
    int32_t k = pos >> 15, f = pos & 32767;
    int32_t a = RVR(i, k), b = RVR(i, k + 1);
    return a + m15(b - a, f);
}

/* allpass stage: Read(TAIL, g) then WriteAllPass(line, -g) */
static inline int32_t allpass(reverb_t *r, int i, int32_t acc, int32_t g)
{
    int32_t prev = TAIL(i);
    acc += m15(prev, g);
    RVR(i, 0) = sat(acc);
    return m15(acc, -g) + prev;
}

static void reverb_init(int pos)
{
    reverb_t *r = &rv[pos];
    r->buf = (int16_t *)op1_fx_scratch(pos);
    if (!r->buf) return;
    for (int i = 0; i < RV_SIZE; i++) r->buf[i] = 0;
    for (int i = 0, b = 0; i < 10; i++) { r->base[i] = b; b += rv_len[i] + 1; }
    r->wp = 0; r->lp1 = r->lp2 = 0;
    for (int k = 0; k < 2; k++) {                          /* 0.5 Hz, 0.3 Hz, as clouds_dsp.c */
        int64_t f = ((int64_t)(k ? 3 : 5) * 32 << 29) / (10LL * 44100);
        int64_t fr = (1LL << 27) - f;
        r->lfo_iir[k] = (int32_t)((16 * fr * ((1LL << 29) - 2 * fr)) >> 29);
        r->lfo_y1[k] = r->lfo_iir[k] / 4;
        r->lfo_y0[k] = 1 << 28;
        r->lfo[k] = (r->lfo_y1[k] + (1 << 28)) >> 14;
    }
}

static void reverb_render(int pos, int16_t *buf, const int16_t *p)
{
    reverb_t *r = &rv[pos];
    if (!r->buf) return;
    int32_t krt = 11469 + (op1_fx_param_q15(p, 0) * 20644 >> 15);      /* blue: time, 0.35 .. 0.98 */
    int32_t amount = op1_fx_param_q15(p, 1);                          /* green: mix */
    const int32_t kap = 20480;                                        /* diffusion: middle (0.625) */
    const int32_t klp = 20479;                                        /* tone: middle (0.625) */
    const int32_t gain = 6554;                        /* input 0.2 as Elements: headroom in the 16-bit lines */

    for (int t = 0; t < OP1_FX_BLOCK; t++) {
        if (--r->wp < 0) r->wp += RV_SIZE;
        if ((r->wp & 31) == 0)                                         /* FxEngine: LFOs every 32 samples */
            for (int k = 0; k < 2; k++) {
                int32_t y = r->lfo_y0[k];
                r->lfo_y0[k] = (int32_t)(((int64_t)r->lfo_iir[k] * y) >> 29) - r->lfo_y1[k];
                r->lfo_y1[k] = y;
                r->lfo[k] = (y + (1 << 28)) >> 14;
            }
        int32_t in = buf[t], acc, apout, wet;
        /* smear AP1 inside the loop */
        RVR(0, 138) = sat(interp(r, 0, 14, 110, r->lfo[0]));
        acc = m15(in, gain);
        acc = allpass(r, 0, acc, kap);
        acc = allpass(r, 1, acc, kap);
        acc = allpass(r, 2, acc, kap);
        acc = allpass(r, 3, acc, kap);
        apout = acc;
        /* loop half 1 */
        acc = apout + m15(interp(r, 9, 8550, 138, r->lfo[1]), krt);
        r->lp1 += m15(acc - r->lp1, klp); acc = r->lp1;
        acc = allpass(r, 4, acc, -kap);
        acc = allpass(r, 5, acc, kap);
        RVR(6, 0) = sat(acc);
        wet = acc;                                                     /* Write(del1, 2.0): *2, halved below */
        /* loop half 2 */
        acc = apout + m15(TAIL(6), krt);
        r->lp2 += m15(acc - r->lp2, klp); acc = r->lp2;
        acc = allpass(r, 7, acc, kap);
        acc = allpass(r, 8, acc, -kap);
        RVR(9, 0) = sat(acc);
        wet = (wet + acc) * 5;                                         /* mono: 2 L + 2 R, makeup for the 0.2 input */
        int32_t o = in + m15(in, -amount) + (((wet >> 3) * amount) >> 12);   /* dry .. wet */
        int32_t ao = o < 0 ? -o : o;                                   /* soft knee above -2.5 dBFS */
        if (ao > RV_KNEE) {
            int32_t d = ao - RV_KNEE;
            ao = d < 16384 ? RV_KNEE + (d >> 2) : 28672 + ((d - 16384) >> 4);
            o = sat(o < 0 ? -ao : ao);
        }
        buf[t] = (int16_t)o;
    }
}

#define DL_SIZE      49152                /* int16, 96 KB (right after the reverb's RV_SIZE; not a power of two) */
#define DL_MIN       1323                 /* 30 ms */
#define DL_MAX       48000                /* ~1.09 s */
#define DL_FB        12000                /* 0.37 feedback */
#define DL_LP        6000                 /* one-pole damping in the feedback loop */
#define DL_GLIDE     10                   /* time glide per sample (Q8), ~30 ms to settle */
#define DL_CLR       4096                 /* samples of the line cleared per block at init */

typedef struct {
    int16_t *buf;
    int32_t wp;
    int32_t len, len_t;                   /* read distance, Q8 samples (smoothed) */
    int32_t lp;
    int32_t clr;                          /* samples still to clear (see delay_init) */
} delay_t;
static delay_t dl[NPOS];

static void delay_init(int pos)
{
    delay_t *d = &dl[pos];
    int16_t *mem = (int16_t *)op1_fx_scratch(pos);
    d->buf = mem ? mem + RV_SIZE : 0;
    d->wp = 0; d->lp = 0;
    d->len = d->len_t = DL_MIN << 8;
    d->clr = d->buf ? DL_SIZE : 0;
}

static void delay_render(int pos, int16_t *buf, const int16_t *p)
{
    delay_t *d = &dl[pos];
    if (!d->buf) return;
    if (d->clr > 0) {                                 /* clear a slice of the line, then pass dry */
        int32_t n = d->clr < DL_CLR ? d->clr : DL_CLR;
        for (int32_t i = 0; i < n; i++) d->buf[d->clr - 1 - i] = 0;
        d->clr -= n;
        return;
    }
    int32_t q = op1_fx_param_q15(p, 2);               /* white: time */
    d->len_t = (DL_MIN + (int32_t)(((int64_t)(DL_MAX - DL_MIN) * q * q) >> 30)) << 8;
    int32_t amount = op1_fx_param_q15(p, 3);          /* red: mix */

    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        d->len += (d->len_t - d->len) >> DL_GLIDE;
        int32_t k = d->len >> 8, f = d->len & 255;
        int32_t ia = d->wp - k, ib = ia - 1;              /* wrap by hand: DL_SIZE is not a power of two */
        if (ia < 0) ia += DL_SIZE;
        if (ib < 0) ib += DL_SIZE;
        int32_t a = d->buf[ia], b = d->buf[ib];
        int32_t out = a + (((b - a) * f) >> 8);
        d->lp += ((out - d->lp) * DL_LP) >> 15;
        int32_t in = buf[n];
        d->buf[d->wp] = sat(in + ((d->lp * DL_FB) >> 15));
        if (++d->wp >= DL_SIZE) d->wp = 0;
        buf[n] = sat(in + ((out * amount) >> 15));
    }
}

/* the game-over tone: a fixed level (the knobs now belong to the two effects) */
#define TONE_GAIN 20000

typedef struct { uint32_t phase, inc; int32_t env, dec; } sn_voice_t;
static sn_voice_t au[NPOS];

static inline int32_t sn_sine(uint32_t ph) { return OP1_SINE_TABLE[((int32_t)ph >> 19) + 4096]; }

static void tone_start(sn_voice_t *v, int32_t inc, int32_t env, int32_t dec)
{
    v->phase = 0; v->inc = (uint32_t)inc; v->env = env; v->dec = dec < 1 ? 1 : dec;
}

static void tone_render(int pos, int16_t *buf)
{
    sn_voice_t *v = &au[pos];

    /* started once, then it decays on its own */
    if (over_evt[pos]) { over_evt[pos] = 0; tone_start(v, 10713070, 30000, 2); }   /* A2, long low tone */
    if (v->env <= 0) return;

    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        if (v->env <= 0) break;                       /* the tone has ended */
        int32_t e = (v->env * TONE_GAIN) >> 15;
        int32_t s = (sn_sine(v->phase) * e) >> 15;
        if (s) buf[n] = op1_sat16(buf[n] + s);
        v->phase += v->inc;
        v->env -= v->dec;
    }
}

static void snake_init(int pos)
{
    pos %= NPOS;
    au[pos].env = 0;
    over_evt[pos] = 0;
    reverb_init(pos);
    delay_init(pos);
    /* the game is reset by the screen thread (the audio thread must not write the game state) */
    g.init = 0;
    g.state = S_READY;
    keyq_init = 0;
    key_map_init();
}

static void snake_render(int pos, int16_t *buf, const int16_t *p)
{
    pos %= NPOS;
    tone_render(pos, buf);
    delay_render(pos, buf, p);
    reverb_render(pos, buf, p);
}

const op1_fx_t FX_snake = { snake_init, snake_render,
                            { OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE }, 0 };

/* ---- screen ---- */
static const uint8_t C_SNAKE[4] = { 0x00, 0xED, 0x95, 0xFF }, C_DEAD[4] = { 0x6F, 0x71, 0x73, 0xFF },
                     C_ORANGE[4] = { 0xFF, 0x3A, 0x5D, 0xFF }, C_WHITE[4] = { 0xFF, 0xFF, 0xFF, 0xFF },
                     C_BLUE[4] = { 0x4D, 0x9E, 0xFF, 0xFF };
/* the four knobs in their official colours: blue, green, white, red (the orange knob) */
static const uint8_t *const C_KNOB[4] = { C_BLUE, C_SNAKE, C_WHITE, C_ORANGE };
/* stroke-font text (seq/seq_text.h): the stock font's uppercase letters come out as icons, and the prompts and score
 * are in capitals. Scale 2 = 8 x 12 px per glyph, centred on the screen. */
#define TXT_S     OP1_Q10(2)             /* px per grid unit */
#define TXT_W     0x600                   /* stroke width */
#define HOLD_BAR_W PX(64)                 /* restart-hold progress bar */
static void text_center(void *r, int32_t y, const char *s, const uint8_t *c)
{
    int32_t w = sq_width(TXT_S, s) - 2 * TXT_S;          /* the last glyph's 2-unit gap is not ink */
    sq_text(r, PX(160) - w / 2, y, TXT_S, s, TXT_W, c);
}

static void ln(void *r, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t w, const uint8_t *c)
{
    sq_line(r, x0, y0, x1, y1, w, c);
}

static volatile int screen_pos = -1;

static void sn_screen_update(void *scr, int dt)
{
    int pos = op1_fx_screen_pos() % NPOS;
    int reset = 0;
    if (pos != screen_pos) { screen_pos = pos; reset = 1; }
    if (dt > 250) reset = 1;                             /* the screen was away: the keys pressed meanwhile are stale */
    if (dt < 0) dt = 0; else if (dt > 100) dt = 100;
    if (!g.init) {
        if (!g.rng) g.rng = 0x2545F491u;
        screen_pos = pos;
        game_reset();
        reset = 1;
    }
    keys_poll(reset);
    g.tick += (uint32_t)dt;

    if (g.state == S_PLAY) game_step(dt, pos);
    if (g.state == S_OVER) {                             /* restart only after a key has been held for 1.5 s */
        int held = sn_keys != 0;
        if (g.hold_armed && held) {
            g.hold_ms += dt;
            if (g.hold_ms >= RESTART_MS) {
                int key = g.prev_key;
                game_reset();
                g.rng ^= g.tick * 2654435761u; if (!g.rng) g.rng = 1;
                g.state = S_PLAY;
                g.prev_key = key; g.have_prev = 1;
            }
        } else {
            g.hold_ms = 0;
        }
    }

    op1_screen_draw(scr, dt);
    void *r = op1_raster();

    /* knob levels across the top: a 1 px line per knob over its quarter of the width, a dot in the middle at 0
     * growing out to both sides until it fills the quarter at 32 767 (the line sits just above the wall) */
    int16_t k[4];
    op1_fx_knobs_live(k);
    for (int i = 0; i < 4; i++) {
        int32_t cx = PX(i * 80 + 40);
        int32_t half = 512 + ((int32_t)k[i] * (PX(40) - 512) >> 15);   /* 0.5 .. 40 px */
        ln(r, cx - half, PX(2), cx + half, PX(2), PX(1), C_KNOB[i]);
    }

    /* the food: a 6 px wide line 6 px long = a 6 x 6 square */
    ln(r, g.food.x - FOOD_HALF, g.food.y, g.food.x + FOOD_HALF, g.food.y, 2 * FOOD_HALF, C_ORANGE);
    /* the snake: one line; each segment reaches half a width past its ends so the corners are filled */
    const uint8_t *col = g.state == S_OVER ? C_DEAD : C_SNAKE;
    int32_t e = BODY_W / 2;
    for (int i = 0; i + 1 < g.n; i++) {
        op1_pt a = g.p[i], b = g.p[i + 1];
        if (a.y == b.y) { if (a.x <= b.x) { a.x -= e; b.x += e; } else { a.x += e; b.x -= e; } }
        else            { if (a.y <= b.y) { a.y -= e; b.y += e; } else { a.y += e; b.y -= e; } }
        ln(r, a.x, a.y, b.x, b.y, BODY_W, col);
    }

    if (g.state == S_READY) {
        text_center(r, PX(108), "PRESS A KEY", C_WHITE);
    } else if (g.state == S_OVER) {
        char b[20];
        sq_num(sq_cat(b, "SCORE "), (uint32_t)g.score, 1);
        text_center(r, PX(56), "GAME OVER", C_ORANGE);
        text_center(r, PX(76), b, C_WHITE);
        text_center(r, PX(100), "HOLD A KEY", C_DEAD);
        int32_t bx = PX(160) - HOLD_BAR_W / 2, by = PX(118);   /* fills while the key is held */
        ln(r, bx, by, bx + HOLD_BAR_W, by, PX(2), C_DEAD);
        if (g.hold_ms > 0)
            ln(r, bx, by, bx + HOLD_BAR_W * g.hold_ms / RESTART_MS, by, PX(2), C_WHITE);
    }
}

static void sn_screen_init(void *scr) { (void)scr; }
const op1_screen_t SCREEN_snake = { sn_screen_init, sn_screen_update };
