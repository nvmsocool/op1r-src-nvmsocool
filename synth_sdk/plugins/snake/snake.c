/* "snake" - the game Snake as a custom FX.
 *
 * The screen is the game. The keys steer the snake while the synth plays as usual (the FX does not swallow them):
 * a key you press turns the snake by 90 degrees depending on the key you pressed before it: a higher key turns it
 * clockwise, a lower key counter-clockwise. The same key twice in a row does nothing. There is no cooldown. The first
 * key starts the game, and after a game over a key held for 1.5 s starts a new one (that also empties the loop).
 *
 * The sound is the score: every food the snake eats puts one random note into a 16-step loop that plays at the OP-1
 * tempo (it starts empty). Each note is a short sine blip of an A minor pentatonic over two octaves; the loop follows
 * the metronome's beat clock, one step per 1/16 note. Eating also plays the new note at once, and the game over plays a
 * low tone. Blue knob = game sound level (the dry synth is untouched).
 *
 * The snake is one line. The food is a 6x6 square. Eating it: the food disappears, the snake grows a little longer,
 * +1 point, the snake gets a little faster, a random note joins the loop, a new food appears somewhere else.
 * Hitting yourself or a wall ends the game: the loop keeps playing, the screen shows the score.
 *
 * Custom FX (docs/51): the game runs in the screen update (GUI thread, dt in ms) and only while its screen is shown.
 * Keys are observed non-destructively in the KeyHandler queue (s_key_queue, 0x01DD5044, u16 code | pressed << 8):
 * the game never consumes them, so the synth still gets every note. Positions are Q10 pixels of the 320 x 160
 * screen. The snake is a polyline of corner points (axis-aligned segments); the head moves in steps of at most 2 px
 * so nothing is skipped at high speed. The audio side only reads the loop and drives a few sine voices in the FX
 * bus. State is per FX position where it matters (the voices); the game itself is shared by the visible FX page.
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
#define RESTART_MS   1500                 /* hold a key this long on the end screen */

/* ---- loop constants ---- */
#define LOOP_STEPS   16                   /* one bar of 1/16 notes */
#define NPOS         3                    /* synth / drum / master */

enum { S_READY, S_PLAY, S_OVER };
enum { D_UP, D_LEFT, D_DOWN, D_RIGHT };   /* clockwise is (d + 3) & 3, counter-clockwise (d + 1) & 3 */
static const int8_t DX[4] = { 0, -1, 0, 1 }, DY[4] = { -1, 0, 1, 0 };

/* the minor pentatonic over two octaves: A3, C4, D4, E4, G4, A4, C5, D5, E5, G5 */
/* phase increment per sample for the same ten pitches (2^32 = one cycle) */
static const uint32_t PENT_INC[10] = {
    21426141u, 25480119u, 28600467u, 32102938u, 38177043u,
    42852281u, 50960238u, 57200933u, 64205877u, 76354085u
};

/* the loop: written by the game (GUI), read by the audio render */
static volatile int8_t loop_note[LOOP_STEPS];      /* 0 = empty, else 1 + index into PENT_SEMI */

/* key codes of the 24 keyboard keys in the KeyHandler queue (seqdev/ms_fw.c kb_keys) */
static const uint8_t KEY_CODE[24] = {
    39, 13, 40, 14, 41, 15, 42, 28, 32, 29, 22, 30, 31, 23, 48, 24, 49, 25, 50, 51, 26, 52, 27, 53
};

/* ---- game state ---- */
typedef struct {
    volatile int state;               /* read by the audio render */
    volatile int init;                /* 0 = the screen must reset the game (audio init asked for it) */
    volatile uint32_t gen;            /* bumped on every reset: the audio render restarts its loop at step 0 */
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
    int      have_prev;
    int      hold_ms;                 /* how long a key has been held on the end screen */
    int      hold_armed;              /* a key was pressed after the game over */
} game_t;
static game_t g;

/* keys currently down (bit 0..23), written by the screen thread */
static volatile uint32_t sn_keys;

static uint32_t rnd(void) { g.rng ^= g.rng << 13; g.rng ^= g.rng >> 17; g.rng ^= g.rng << 5; return g.rng; }
static int32_t iabs(int32_t a) { return a < 0 ? -a : a; }

/* ---- note events (game -> audio): 1..10 = pentatonic degree, 0x20 = game over ---- */
#define EVQ_N 16
static volatile uint8_t evq[NPOS][EVQ_N];
static volatile uint8_t evq_w[NPOS], evq_r[NPOS];

static void ev_push(int pos, int ev)
{
    pos %= NPOS;
    uint8_t w = evq_w[pos], n = (uint8_t)((w + 1) % EVQ_N);
    if (n != evq_r[pos]) { evq[pos][w] = (uint8_t)ev; evq_w[pos] = n; }
}

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
    g.have_prev = 0;
    g.hold_ms = 0; g.hold_armed = 0;
    spawn_food();
    g.state = S_READY;
    g.init = 1;
    g.gen++;
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

/* turn into direction d (not the opposite one, not the same): a new corner at the head */
static int turn(int d)
{
    if (d == g.dir || d == ((g.dir + 2) & 3) || g.n >= MAXV) return 0;
    g.p[g.n] = g.p[g.n - 1]; g.n++;
    g.dir = d;
    return 1;
}

/* a random new note in the loop, and the same note at once (the food's reward) */
static void add_note(int pos)
{
    int s = (int)(rnd() % LOOP_STEPS);
    int d = (int)(rnd() % 10u);
    loop_note[s] = (int8_t)(1 + d);
    ev_push(pos, 1 + d);
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
            ev_push(pos, 0x20);
            return;
        }
        if (food_hit()) {
            g.score++;
            g.len += GROW;
            g.speed += SPEED_UP; if (g.speed > SPEED_MAX) g.speed = SPEED_MAX;
            add_note(pos);
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
    if (g.state != S_PLAY) {                                 /* first key: start playing with an empty loop */
        g.rng ^= g.tick * 2654435761u; if (!g.rng) g.rng = 1;
        g.state = S_PLAY;
        g.prev_key = key; g.have_prev = 1;
        return;
    }
    if (!g.have_prev) { g.prev_key = key; g.have_prev = 1; return; }
    if (key == g.prev_key) return;                           /* the same key */
    turn(key > g.prev_key ? ((g.dir + 3) & 3) : ((g.dir + 1) & 3));
    g.prev_key = key;
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
void host_sn_keymap(void) { key_map_init(); }
int  host_sn_state(void) { return g.state; }
int  host_sn_score(void) { return g.score; }
int  host_sn_head_x(void) { return g.p[g.n - 1].x; }
int  host_sn_head_y(void) { return g.p[g.n - 1].y; }
int  host_sn_len(void) { return g.n; }
uint32_t host_sn_gen(void) { return g.gen; }
int  host_sn_hold(void) { return g.hold_ms; }
void host_sn_place_food(int x, int y) { g.food.x = x; g.food.y = y; }
int  host_sn_notes(void) { int n = 0; for (int i = 0; i < LOOP_STEPS; i++) if (loop_note[i]) n++; return n; }
int  host_sn_loop(int i) { return i >= 0 && i < LOOP_STEPS ? loop_note[i] : 0; }
#endif

/* ---- audio: a few sine voices over the FX bus ---- */
#define SN_VOICES 6
typedef struct { uint32_t phase, inc; int32_t env, dec; } sn_voice_t;
typedef struct {
    sn_voice_t v[SN_VOICES];
    int32_t step_left;                /* samples to the next 1/16 note */
    int32_t step;                     /* 0..15 */
    uint32_t gen;                     /* game generation the loop follows */
} sn_audio_t;
static sn_audio_t au[NPOS];

static inline int32_t sn_sine(uint32_t ph) { return OP1_SINE_TABLE[((int32_t)ph >> 19) + 4096]; }

static void voice_start(sn_audio_t *a, int32_t inc, int32_t env, int32_t dec)
{
    int best = 0;
    for (int i = 1; i < SN_VOICES; i++) if (a->v[i].env < a->v[best].env) best = i;
    sn_voice_t *v = &a->v[best];
    v->phase = 0; v->inc = (uint32_t)inc; v->env = env; v->dec = dec < 1 ? 1 : dec;
}

/* samples per 1/16 note from the OP-1 beat clock (120 BPM when there is no clock yet) */
static int32_t step_samples(void)
{
    op1_beat_t b;
    op1_beat_clock(&b);
    int32_t spb = b.len > (256 << 14) ? (b.len >> 14) : 22050;   /* samples per beat */
    int32_t s = spb >> 2;
    return s < 256 ? 256 : (s > 44100 ? 44100 : s);
}

static void audio_render(int pos, int16_t *buf, int32_t gain)
{
    sn_audio_t *a = &au[pos];

    /* one-shot notes from the game (food / game over) */
    uint8_t r = evq_r[pos];
    while (r != evq_w[pos]) {
        int ev = evq[pos][r];
        r = (uint8_t)((r + 1) % EVQ_N);
        if (ev >= 1 && ev <= 10) voice_start(a, PENT_INC[ev - 1], 22000, 4);
        else if (ev == 0x20)    voice_start(a, 10713070, 30000, 2);  /* A2, long low tone */
    }
    evq_r[pos] = r;

    /* the loop runs while the game plays or is over; a new game restarts it at step 0 */
    int st = g.state;
    if (g.gen != a->gen) { a->gen = g.gen; a->step = 0; a->step_left = 0; }
    if (st == S_PLAY || st == S_OVER) {
        int32_t len = step_samples();
        if (a->step_left <= 0) {
            int8_t d = loop_note[a->step];
            int32_t gate = len * 3 / 5;                        /* ~60 % of the step */
            if (d >= 1 && d <= 10) voice_start(a, PENT_INC[d - 1], 24000, 24000 / gate);
            a->step = (a->step + 1) & (LOOP_STEPS - 1);
            a->step_left += len;
        }
        a->step_left -= OP1_FX_BLOCK;
    } else {
        a->step = 0; a->step_left = 0;
    }

    for (int n = 0; n < OP1_FX_BLOCK; n++) {
        int32_t s = 0;
        for (int i = 0; i < SN_VOICES; i++) {
            sn_voice_t *v = &a->v[i];
            if (v->env <= 0) continue;
            int32_t e = (v->env * gain) >> 15;                  /* fold the level in per voice: no 32-bit overflow */
            s += (sn_sine(v->phase) * e) >> 15;
            v->phase += v->inc;
            v->env -= v->dec;
        }
        if (s) buf[n] = op1_sat16(buf[n] + s);
    }
}

static void snake_init(int pos)
{
    pos %= NPOS;
    sn_audio_t *a = &au[pos];
    for (int i = 0; i < SN_VOICES; i++) a->v[i].env = 0;
    a->step = 0; a->step_left = 0; a->gen = g.gen;
    evq_w[pos] = evq_r[pos] = 0;
    /* the game is reset by the screen thread (the audio thread must not write the game state) */
    g.init = 0;
    g.state = S_READY;
    keyq_init = 0;
    key_map_init();
}

static void snake_render(int pos, int16_t *buf, const int16_t *p)
{
    pos %= NPOS;
    int32_t vol = op1_fx_param_q15(p, 0);                /* blue: game sound level */
    audio_render(pos, buf, vol);
}

const op1_fx_t FX_snake = { snake_init, snake_render,
                            { OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE }, 0 };

#ifdef OP1_HOST
int host_sn_astep(void) { return au[0].step; }
int host_sn_aleft(void) { return au[0].step_left; }
int host_sn_aenv(void) { int e = 0; for (int i = 0; i < SN_VOICES; i++) if (au[0].v[i].env > e) e = au[0].v[i].env; return e; }
#endif

/* ---- screen ---- */
static const uint8_t C_SNAKE[4] = { 0x00, 0xED, 0x95, 0xFF }, C_DEAD[4] = { 0x6F, 0x71, 0x73, 0xFF },
                     C_FOOD[4] = { 0xFF, 0x3A, 0x5D, 0xFF }, C_WHITE[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
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
                for (int i = 0; i < LOOP_STEPS; i++) loop_note[i] = 0;
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

    /* the food: a 6 px wide line 6 px long = a 6 x 6 square */
    ln(r, g.food.x - FOOD_HALF, g.food.y, g.food.x + FOOD_HALF, g.food.y, 2 * FOOD_HALF, C_FOOD);
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
        text_center(r, PX(56), "GAME OVER", C_FOOD);
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
