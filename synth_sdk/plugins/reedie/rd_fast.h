/* reedie: a faster rd_render() for reedie.c (rd_render_fast), the same samples bit for bit.
 * Included by reedie.c after rd_note_off(); the original rd_render() stays in reedie.c as the reference
 * (build with -DRD_REFERENCE to use it; harness/reedie_fast_test.c compares the two).
 *
 * What changed, none of it audible:
 *  - The Blackfin has no divide instruction: each 64-bit `/` and `*` is a libgcc call (__divdi3 about a thousand
 *    cycles; three of them and two __muldi3 in every block, four more 32-bit __divsi3). Here the 64-bit quotients
 *    are exact long divisions (rd_divlo) and the products are split into 16-bit halves (same truncation, same floor
 *    for negative numbers). What depends on the pitch alone (2^47 / inc, the DC-blocker pole, the loss pole) is kept
 *    per voice with the inputs it was made from (rd_pc) and reused while they are the same; what depends on the four
 *    knob words alone (bore blend, colour, reed rest and drive, start ramp, gain) is kept once for the words last
 *    seen (rd_kc); the start-up steps (fill ramp, chiff: three divides) are made once per bore length.
 *  - The sample loop (rd_loop) reads one ring sample a tap (the neighbour sample of a tap is the previous sample's
 *    value), keeps the start-up products (fill, attack blow, reed ramp, chiff) as running sums, and is run in
 *    stretches of a block split where a start-up term starts or ends (the fill at age = di, the reed ramp at ramplen,
 *    the chiff for seeddur, the attack blow at max(1500, ramplen + seeddur)), each compiled without the tests that
 *    are constant there.
 *  - On the Blackfin the loops are written in assembly (rd_asm.S, made by gen_asm.py from rd_loop, bit for bit): see
 *    rd_asm_run below. The host builds and -DRD_ASM_C use the C loop, which stays the reference.
 * Ranges: pitch increments any (clamped as in reedie.c), knob words any int16 (clamped), velocity as
 * op1_voice_vel_gain.
 * Written by Hundekraft from nvmsocool's reedie.c (Copyright (C) 2026 nvmsocool).
 * Copyright (C) 2026 Hundekraft. GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */

/* floor((r 2^32 + lo) / 2^(32 - n) / d) mod 2^32 for r < d < 2^30: restoring division, the top n bits of lo shifted in,
 * one quotient bit a step */
static uint32_t rd_divlo(uint32_t r, uint32_t lo, int n, uint32_t d)
{
    uint32_t q = 0;
    for (int i = 0; i < n; i++) {
        r = (r << 1) | (lo >> 31); lo <<= 1; q <<= 1;
        if (r >= d) { r -= d; q |= 1; }
    }
    return q;
}

/* what the four knob words alone decide, for the last words seen (all voices of a slot read the same ones) */
typedef struct {
    int16_t k[4]; uint8_t ok;
    int32_t breath, buzz, c, c1, ex_hi6, kcol, rq8, rest, amp, u0, ramplen, rstep, gain, tilt;
} rd_kc_t;
static rd_kc_t rd_kc;

/* what the pitch (and for aloss the loss corner) decides, per voice, for the last values seen */
typedef struct {
    int32_t inc, rq8, Dt0, alpha, aloss;     /* inc 0: nothing kept */
    int32_t dint, fstep, seeddur, sstep;     /* dint 0 (the zeroed .bss; a real one is >= 4): nothing kept */
} rd_pc_t;
static rd_pc_t rd_pc[OP1_VOICES];

static void rd_kc_make(rd_kc_t *q, const int16_t *params)
{
    int32_t kb = rd_clamp(params[0], 0, 32767);
    int32_t kz = rd_clamp(params[1], 0, 32767);
    int32_t kt = rd_clamp(params[2], 0, 32767) - 16384;
    int32_t kp = rd_clamp(params[3], 0, 32767);
    q->breath = (kb >> 1) + ((kb * kb) >> 16);
    q->buzz   = (kz >> 1) + ((kz * kz) >> 16);
    int32_t c = 0;
    if (kp > 8192) c = kp >= 22937 ? 32767 : (kp - 8192) * 32767 / 14745;
    int32_t ex_lo = kp < 8192 ? 8192 - kp : 0;
    int32_t ex_hi = kp > 22937 ? kp - 22937 : 0;
    int32_t kcol = 16384 - ((ex_lo * 8000) >> 13) + ((ex_hi * 9000) >> 13);
    q->kcol = rd_clamp(kcol, 4096, 28000);
    q->rq8 = rd_clamp(768 - ((ex_lo * 256) >> 13), 320, 1400);
    q->c = c; q->c1 = 32768 - c;
    q->ex_hi6 = (ex_hi * 6000) >> 13;
    int32_t rest = 9830 + (kt >> 4);
    int32_t amp = rd_clamp(16384 - (kt >> 4), 14000, 19000);
    int32_t rest2 = (rest * rest) >> 15;
    int32_t rest3 = (rest2 * rest) >> 15;
    q->rest = rest; q->amp = amp;
    q->u0 = ((rest - rest3) * amp) >> 13;
    q->ramplen = 882 + ((q->breath >> 5) << 1);
    q->rstep = (32767 << 15) / q->ramplen;
    q->gain = 44000 - ((11232 * c) >> 15);
    q->tilt = q->kcol - 16384;
    for (int i = 0; i < 4; i++) q->k[i] = params[i];
    q->ok = 1;
}

static void rd_pc_make(rd_pc_t *p, int32_t inc, int32_t rq8)
{
    uint32_t ih = (uint32_t)inc >> 16, il = (uint32_t)inc & 0xffff;
    /* (int32_t)(2^47 / inc): the low word of the quotient of 2^15 2^32 by inc (r = 2^15 mod inc < inc) */
    p->Dt0 = (int32_t)rd_divlo(32768u % (uint32_t)inc, 0, 32, (uint32_t)inc);
    /* (inc * 51472) >> 32 */
    uint32_t a = ih * 51472u + ((il * 51472u) >> 16);
    p->alpha = rd_clamp((int32_t)(a >> 16), 48, 12000);
    /* inc * rq8 / 5340160, the product (< 2^40) as X 2^16 + Y; r = product >> 18 < 5340160, 18 bits to shift in */
    uint32_t X = ih * (uint32_t)rq8, Y = il * (uint32_t)rq8;
    uint32_t r = (X + (Y >> 16)) >> 2, low = ((X << 16) + Y) & 0x3ffffu;
    p->aloss = rd_clamp((int32_t)rd_divlo(r, low << 14, 18, 5340160u), 120, 20000);
    p->inc = inc; p->rq8 = rq8;
}

/* one block's constants and the voice state, so the loop below is one function that is inlined three or four times */
typedef struct {
    int32_t di, fr, di2, fr2, c, c1, alpha, aloss, rest, amp, u0, ramplen, rstep, fstep, seeddur, sstep;
    int32_t buzz, breath, vgain, tilt, gain, ex_hi6;
    int32_t b1, b2, chat, op, lp, dc, rel, age;
    uint32_t rng, wp;
} rd_ctx_t;

/* samples n0..n1 of the block, upstream's loop with the tests that are constant in the stretch compiled out:
 *   F   the fill ramp is running (age < di);  Y  the attack blow, the reed's opening ramp or the chiff may be running;
 *   REL the key is up (the release fade; upstream's `!v->gate` branch).
 * Per sample the second tap's neighbour sample is the first tap's value of the sample before (the ring is only written
 * at wp, which neither neighbour is), so each tap reads one ring sample, not two. */
static inline __attribute__((always_inline))
void rd_loop(rd_ctx_t *s, int16_t *ring, int16_t *out, int n0, int n1, const int F, const int Y, const int REL)
{
    const int32_t g = 32760, relstep = (32767 << 15) / 4410, astep = (8192 << 15) / 1500, alp = 14238;
    const int32_t di = s->di, fr = s->fr, di2 = s->di2, fr2 = s->fr2, c = s->c, c1 = s->c1, alpha = s->alpha, aloss = s->aloss;
    const int32_t rest = s->rest, amp = s->amp, u0 = s->u0, ramplen = s->ramplen, rstep = s->rstep;
    const int32_t fstep = s->fstep, seeddur = s->seeddur, sstep = s->sstep;
    const int32_t buzz = s->buzz, breath = s->breath, vgain = s->vgain, tilt = s->tilt, gain = s->gain, ex_hi6 = s->ex_hi6;
    int32_t b1 = s->b1, b2 = s->b2, chat = s->chat, op = s->op, lp = s->lp, dc = s->dc, rel = s->rel, age = s->age;
    uint32_t rng = s->rng, wp = s->wp;
    int32_t pa = ring[((int32_t)wp - di - 1) & RMASK], pb = ring[((int32_t)wp - di2 - 1) & RMASK];
    /* the start-up products as running sums (exact: each is a product of age that steps by one a sample; unsigned, so
     * the sums may run past their range where they are no longer read) */
    uint32_t fq = 0, aq = 0, rq = 0, sq = 0;
    if (F) fq = (uint32_t)age * (uint32_t)fstep;
    if (Y) {
        aq = (uint32_t)(1500 - age) * (uint32_t)astep;
        rq = (uint32_t)age * (uint32_t)rstep;
        sq = (uint32_t)(seeddur - (age - ramplen)) * (uint32_t)sstep;
    }

    for (int n = n0; n < n1; n++) {
        int32_t a = ring[((int32_t)wp - di) & RMASK];
        int32_t a0 = a;
        a += ((pa - a) * fr) >> 15;
        pa = a0;
        int32_t b = ring[((int32_t)wp - di2) & RMASK];
        int32_t b0 = b;
        b += ((pb - b) * fr2) >> 15;
        pb = b0;
        int32_t r = ((c1 * a) - (c * b)) >> 15;

        if (F && age < di) {
            int32_t f = (int32_t)fq;                  /* age * fstep */
            if (f < 32767) r = (r * f) >> 15;
        }
        if (F) fq += (uint32_t)fstep;

        lp += ((r - lp) * aloss) >> 15;
        dc += ((lp - dc) * alpha) >> 15;
        r = (((lp - dc) * g) >> 15);

        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        int32_t nz = (int16_t)(rng >> 16);

        int32_t an = nz < 0 ? -nz : nz;
        chat += ((an - chat) * 220) >> 15;
        int32_t wob = ((chat - 16384) * buzz) >> 15;
        wob = rd_clamp(wob, -8192, 8192);

        int32_t ampe = amp;
        if (Y) {
            int32_t ak = 1500 - age;
            if (ak > 0) ampe = amp + ((amp * ((int32_t)aq >> 15)) >> 15);      /* aq = ak * astep */
            aq -= (uint32_t)astep;
        }
        int32_t x = rest - r + (wob >> 3);
        x = rd_clamp(x, -32767, 32767);
        int32_t x2 = (x * x) >> 15;
        int32_t x3 = (x2 * x) >> 15;
        int32_t u = ((x - x3) * ampe) >> 13;
        u -= u0;
        int32_t reed = 32767;
        if (REL) {
            reed = rel >= 4410 ? 0 : 32767 - ((rel * relstep) >> 15);
            rel++;
        } else if (Y) reed = age >= ramplen ? 32767 : ((int32_t)rq >> 15);  /* rq = age * rstep */
        if (Y) rq += (uint32_t)rstep;
        u = (u * reed) >> 15;

        b1 += ((nz - b1) * 934) >> 15;
        int32_t hp = nz - b1;
        b2 += ((hp - b2) * alp) >> 15;
        int32_t bamp = 32767 - age * 3;
        if (bamp < 11468) bamp = 11468;
        int32_t airq = ((((b2 * bamp) >> 15) * breath) >> 14);
        airq = (airq * vgain) >> 15;
        airq = rd_clamp(airq, -12000, 12000);

        int32_t rasp = (((nz * chat) >> 15) * buzz) >> 15;

        int32_t seed = 0;
        if (Y) {
            int32_t sk = age - ramplen;
            if (sk >= 0 && sk < seeddur)
                seed = (nz * ((int32_t)sq >> 15)) >> 16;                      /* sq = (seeddur - sk) * sstep */
            sq -= (uint32_t)sstep;
        }

        int32_t in = u + (airq >> 1) + (wob >> 2) + rasp + seed + (nz >> 10);
        ring[wp] = op1_sat16(in);
        wp = (wp + 1) & RMASK;

        age++;
        op += ((r - op) * 6400) >> 15;
        int32_t col = r + ((tilt * (r - op)) >> 15);
        int32_t y = (col * gain) >> 15;
        y += airq + (airq >> 2);
        y += (ex_hi6 * b2) >> 15;
        if (y > 28000) y = 28000 + ((y - 28000) >> 3);
        else if (y < -28000) y = -28000 + ((y + 28000) >> 3);
        if (y > 32767) y = 32767;
        else if (y < -32767) y = -32767;
        out[n] = (int16_t)y;
    }

    if (age > 64000) age = 64000;                 /* upstream stops counting there: every age test is settled by then */
    s->b1 = b1; s->b2 = b2; s->chat = chat; s->op = op; s->lp = lp; s->dc = dc; s->rel = rel; s->age = age;
    s->rng = rng; s->wp = wp;
}

/* The sample loop in Blackfin assembly (rd_asm.S from gen_asm.py, bit for bit rd_loop: the settled stretch, the start-up
 * stretches split where a term starts or ends, the release once the start-up is over); the host builds and RD_ASM_C
 * use the C. rd_ak_t: what the asm reads, packed for its 16-bit multiplies (offsets pinned here and in gen_asm.py). */
#if defined(__bfin__) && !defined(RD_ASM_C)
#define RD_ASM 1
#endif
#if defined(RD_ASM) || defined(RD_ASM_REF)
typedef struct {
    uint32_t frc, fr, one, c, ld, al, g, ch, b1k, bz, wp, wn; int32_t rest, p32767, m32767; uint32_t ua;
    uint32_t k8192; int32_t k14238, ex, k11468, breath, vgain, p12000, m12000, p28000, m28000, k6400, tilt, gain, k3;
    uint32_t rng, lpdc;
    uint32_t b1ch; int32_t b2, op, br, zero; uint32_t fq, fstep, aq, astep, rq, rstep, sq, sstep;
    int16_t *ring, *pa, *pb, *w, *out;
} rd_ak_t;                                   /* rd_asm.S reaches the words in three windows of 16: k, k + 64, k + 128 */
#define RD_OFF(f, o) typedef char rd_off_##f[__builtin_offsetof(rd_ak_t, f) == (o) ? 1 : -1];
RD_OFF(fr, 4) RD_OFF(ua, 60) RD_OFF(k8192, 64) RD_OFF(lpdc, 124) RD_OFF(b1ch, 128) RD_OFF(br, 140) RD_OFF(zero, 144)
RD_OFF(sstep, 176) RD_OFF(out, 196)
#undef RD_OFF
enum { RK_SET, RK_FR, RK_R, RK_FS, RK_S, RK_A, RK_REL, RK_REL0, RK_FREL, RK_FSREL, RK_AREL, RK_SREL, RK_SREL0 };
void rd_k_set(rd_ak_t *k, int count); void rd_k_fr(rd_ak_t *k, int count); void rd_k_r(rd_ak_t *k, int count);
void rd_k_fs(rd_ak_t *k, int count); void rd_k_s(rd_ak_t *k, int count); void rd_k_a(rd_ak_t *k, int count);
void rd_k_rel(rd_ak_t *k, int count); void rd_k_rel0(rd_ak_t *k, int count); void rd_k_frel(rd_ak_t *k, int count);
void rd_k_fsrel(rd_ak_t *k, int count); void rd_k_arel(rd_ak_t *k, int count); void rd_k_srel(rd_ak_t *k, int count);
void rd_k_srel0(rd_ak_t *k, int count);

#define RD_H(h, l) (((uint32_t)(uint16_t)(h) << 16) | (uint16_t)(l))
static inline int rd_i16(int32_t x) { return x >= -32768 && x <= 32767; }

/* samples n0..n1 through the asm loop v, which the caller has chosen so that its terms are the ones rd_loop would run
 * there (RK_FR / RK_FS: age < di; RK_FR, RK_R: age < ramplen; RK_FS, RK_S: the chiff; RK_FR..RK_A: age < t1, the
 * attack blow from aq's sign; RK_SET, RK_REL, RK_REL0: age >= t1; RK_REL: rel < 4410, RK_REL0: rel >= 4410; released
 * in the start-up, rel < 4410: RK_FREL age < di, ramplen; RK_FSREL age < di in the chiff; RK_SREL the chiff; RK_AREL the rest;
 * RK_SREL0: rel >= 4410 in the chiff, the only start-up term left once age >= 4410).
 * 0 (nothing done) when the state is outside what the asm's 16-bit products assume (never for the engine's own:
 * lp, dc and b1 stay int16, chat 0..32767) */
static int rd_asm_run(rd_ctx_t *s, int16_t *ring, int16_t *out, int n0, int n1, int v)
{
    if (n1 <= n0 || !rd_i16(s->lp) || !rd_i16(s->dc) || !rd_i16(s->b1) || s->chat < 0 || s->chat > 32767) return 0;
    rd_ak_t k;
    const int32_t wp = (int32_t)s->wp, age = s->age, cnt = n1 - n0;
    k.ring = ring; k.pa = &ring[(wp - s->di - 1) & RMASK]; k.pb = &ring[(wp - s->di2 - 1) & RMASK]; k.w = &ring[wp];
    k.out = out + n0;
    k.frc = RD_H(32767 - s->fr2, 32767 - s->fr); k.fr = RD_H(s->fr2, s->fr); k.one = 0x00010001u;
    k.c = RD_H(s->c, 32767 - s->c); k.ld = RD_H(32768 - s->alpha, 32768 - s->aloss); k.al = RD_H(s->alpha, s->aloss);
    k.g = 32760; k.ch = RD_H(220, 32548); k.b1k = RD_H(934, 31834); k.bz = RD_H(16384, s->buzz);
    k.wp = RD_H(8192, 0); k.wn = RD_H(-8192, 0);
    k.rest = s->rest; k.p32767 = 32767; k.m32767 = -32767; k.ua = RD_H(s->u0, s->amp); k.k8192 = 8192;
    k.k14238 = 14238; k.ex = s->ex_hi6; k.k11468 = 11468; k.breath = s->breath; k.vgain = s->vgain;
    k.p12000 = 12000; k.m12000 = -12000; k.p28000 = 28000; k.m28000 = -28000; k.k6400 = 6400; k.tilt = s->tilt;
    k.gain = s->gain; k.k3 = 3; k.zero = 0;
    k.rng = s->rng; k.lpdc = RD_H(s->dc, s->lp); k.b1ch = RD_H(s->b1, s->chat); k.b2 = s->b2; k.op = s->op;
    k.br = 32767 - 3 * (age - 1);
    k.fstep = (uint32_t)s->fstep; k.fq = (uint32_t)age * k.fstep;                                  /* as rd_loop's sums */
    k.astep = (uint32_t)((8192 << 15) / 1500); k.aq = (uint32_t)(1500 - age) * k.astep;
    k.sstep = (uint32_t)s->sstep; k.sq = (uint32_t)(s->seeddur - (age - s->ramplen)) * k.sstep;
    if (v == RK_REL || (v >= RK_FREL && v != RK_SREL0)) { k.rstep = (uint32_t)((32767 << 15) / 4410); k.rq = (uint32_t)s->rel * k.rstep; }
    else { k.rstep = (uint32_t)s->rstep; k.rq = (uint32_t)age * k.rstep; }
    switch (v) {
    case RK_SET: rd_k_set(&k, cnt); break;
    case RK_FR: rd_k_fr(&k, cnt); break;
    case RK_R: rd_k_r(&k, cnt); break;
    case RK_FS: rd_k_fs(&k, cnt); break;
    case RK_S: rd_k_s(&k, cnt); break;
    case RK_A: rd_k_a(&k, cnt); break;
    case RK_REL: rd_k_rel(&k, cnt); break;
    case RK_REL0: rd_k_rel0(&k, cnt); break;
    case RK_FREL: rd_k_frel(&k, cnt); break;
    case RK_FSREL: rd_k_fsrel(&k, cnt); break;
    case RK_AREL: rd_k_arel(&k, cnt); break;
    case RK_SREL: rd_k_srel(&k, cnt); break;
    default: rd_k_srel0(&k, cnt); break;
    }
    if (v >= RK_REL) s->rel += cnt;
    s->rng = k.rng; s->lp = (int16_t)k.lpdc; s->dc = (int16_t)(k.lpdc >> 16); s->chat = (int16_t)k.b1ch;
    s->b1 = (int16_t)(k.b1ch >> 16); s->b2 = k.b2; s->op = k.op;
    s->wp = (uint32_t)(k.w - ring);
    s->age = age + cnt;
    if (s->age > 64000) s->age = 64000;
    return 1;
}
#endif

static void rd_render_fast(void *self, int16_t *out, const int16_t *params)
{
    reedie_voice_t *v = (reedie_voice_t *)self;
    int32_t vi = op1_voice_index(self);
    int16_t *ring;
    if (vi < 0 || vi >= OP1_VOICES) vi = 0;
    ring = rd_ring[vi];

    /* ---- knobs, once per change of the four words ---- */
    rd_kc_t *kc = &rd_kc;
    if (!kc->ok || kc->k[0] != params[0] || kc->k[1] != params[1] || kc->k[2] != params[2] || kc->k[3] != params[3])
        rd_kc_make(kc, params);
    const int32_t breath = kc->breath, buzz = kc->buzz, c = kc->c, c1 = kc->c1, rq8 = kc->rq8;
    const int32_t vgain = op1_voice_vel_gain(self);

    /* ---- pitch -> bore length, as upstream with the products split in halves ---- */
    int32_t inc = rd_clamp(OP1_PITCH_INC[0], 1, INCMAX);
    rd_pc_t *pc = &rd_pc[vi];
    if (pc->inc != inc || pc->rq8 != rq8) rd_pc_make(pc, inc, rq8);
    int32_t Dt = pc->Dt0;
    int32_t comp = 2200 + ((300 * c) >> 15);
    comp -= 3355;
    if (rq8 < 768) comp -= ((768 - rq8) * 7) >> 1;
    if (Dt >= 0 && vgain >= 0 && vgain < 65536) {
        uint32_t hi = (uint32_t)Dt >> 16, lo = (uint32_t)Dt & 0xffff;      /* (Dt vgain) >> 23 */
        Dt -= (int32_t)((hi * (uint32_t)vgain + ((lo * (uint32_t)vgain) >> 16)) >> 7);
    } else Dt -= (int32_t)(((int64_t)Dt * vgain) >> 23);
    if (Dt >= 0 && comp > -65536 && comp < 65536) {                        /* (Dt comp) >> 15, floor for comp < 0 */
        uint32_t hi = (uint32_t)Dt >> 16, lo = (uint32_t)Dt & 0xffff;
        uint32_t m = (uint32_t)(comp < 0 ? -comp : comp);
        if (comp >= 0) Dt += (int32_t)((hi * m << 1) + ((lo * m) >> 15));
        else           Dt -= (int32_t)((hi * m << 1) + ((lo * m + 32767u) >> 15));
    } else Dt += (int32_t)(((int64_t)Dt * comp) >> 15);
    Dt += 32768;
    Dt = rd_clamp(Dt, DMIN_Q16, DMAX_Q16);
    int32_t D = v->D_q16 ? v->D_q16 + ((Dt - v->D_q16) >> 1) : Dt;
    D = rd_clamp(D, DMIN_Q16, DMAX_Q16);

    const int32_t gate = v->gate;
    rd_ctx_t s;
    s.di = D >> 16; s.fr = (D >> 1) & 32767;
    const int32_t D2 = D << 1;
    s.di2 = D2 >> 16; s.fr2 = (D2 >> 1) & 32767;
    s.c = c; s.c1 = c1; s.alpha = pc->alpha; s.aloss = pc->aloss; s.rest = kc->rest; s.amp = kc->amp; s.u0 = kc->u0;
    s.ramplen = kc->ramplen; s.rstep = kc->rstep; s.buzz = buzz; s.breath = breath; s.vgain = vgain;
    s.tilt = kc->tilt; s.gain = kc->gain; s.ex_hi6 = kc->ex_hi6;
    s.b1 = v->b1; s.b2 = v->b2; s.chat = v->chat; s.op = v->op;
    s.rng = v->rng; s.wp = v->wp; s.age = v->age; s.lp = v->lp; s.dc = v->dc; s.rel = v->rel;
    s.fstep = s.sstep = s.seeddur = 0;

    if (pc->dint != s.di) {                                   /* the start-up steps, once per bore length */
        pc->dint = s.di;
        pc->fstep = s.di > 0 ? 32767 / s.di : 32767;
        int32_t sd0 = s.di * 8;
        pc->seeddur = sd0 < 2048 ? sd0 : 2048;
        if (pc->seeddur < 8) pc->seeddur = 8;
        pc->sstep = (32767 << 15) / pc->seeddur;
    }
    s.fstep = pc->fstep; s.seeddur = pc->seeddur; s.sstep = pc->sstep;

    const int32_t t1 = s.ramplen + s.seeddur > 1500 ? s.ramplen + s.seeddur : 1500;   /* the start-up is over */
    if (!gate) {
#ifdef RD_ASM
        {                                        /* split where a term starts or ends, as the held notes below */
            int n = 0;
            while (n < OP1_BLOCK) {
                int32_t age = s.age, m = OP1_BLOCK - n, v, sd = s.ramplen + s.seeddur;
                if (s.rel < 4410) { if (4410 - s.rel < m) m = 4410 - s.rel; }
                if (age >= t1) v = s.rel < 4410 ? RK_REL : RK_REL0;
                else {
                    int32_t e = t1;
                    const int F = age < s.di;
                    if (F && s.di < e) e = s.di;
                    if (age < s.ramplen) { if (s.ramplen < e) e = s.ramplen; }
                    else if (age < sd && sd < e) e = sd;
                    if (e - age < m) m = e - age;
                    const int S = age >= s.ramplen && age < sd;
                    v = s.rel >= 4410 ? (S && !F ? RK_SREL0 : -1) : S ? (F ? RK_FSREL : RK_SREL) : F ? RK_FREL : RK_AREL;
                }
                if (v < 0 || !rd_asm_run(&s, ring, out, n, n + m, v)) rd_loop(&s, ring, out, n, n + m, 1, 1, 1);
                n += m;
            }
        }
#else
        rd_loop(&s, ring, out, 0, OP1_BLOCK, 1, 1, 1);               /* release: everything as upstream wrote it */
#endif
    } else {
        /* held: split the block where a start-up term starts or ends (the fill at age = di; the reed ramp at ramplen,
         * then the chiff for seeddur; the attack blow at T1), so each stretch runs a loop without the tests that are
         * then constant (the C keeps the attack and chiff tests until T1) */
        int n = 0;
        while (n < OP1_BLOCK) {
            int32_t age = s.age, m = OP1_BLOCK - n;
            if (age >= t1) {
#ifdef RD_ASM
                if (!rd_asm_run(&s, ring, out, n, OP1_BLOCK, RK_SET))
#endif
                rd_loop(&s, ring, out, n, OP1_BLOCK, 0, 0, 0);
                break;
            }
            const int F = age < s.di;
            int32_t e = t1, sd = s.ramplen + s.seeddur;             /* the next age where a term starts or ends */
            if (F && s.di < e) e = s.di;
            if (age < s.ramplen) { if (s.ramplen < e) e = s.ramplen; }
            else if (age < sd && sd < e) e = sd;
            if (e - age < m) m = e - age;
#ifdef RD_ASM
            int v = age < s.ramplen ? (F ? RK_FR : RK_R) : age < sd ? (F ? RK_FS : RK_S) : (F ? -1 : RK_A);
            if (v < 0 || !rd_asm_run(&s, ring, out, n, n + m, v))
#endif
            {
                if (F) rd_loop(&s, ring, out, n, n + m, 1, 1, 0);
                else rd_loop(&s, ring, out, n, n + m, 0, 1, 0);
            }
            n += m;
        }
    }

    v->wp = (uint16_t)s.wp;
    v->age = (uint16_t)s.age;
    v->D_q16 = D;
    v->lp = s.lp;
    v->dc = s.dc;
    v->b1 = (int16_t)s.b1;
    v->b2 = (int16_t)s.b2;
    v->chat = (int16_t)s.chat;
    v->op = (int16_t)s.op;
    v->rel = (uint16_t)(s.rel > 65535 ? 65535 : s.rel);
    v->rng = s.rng;
}
