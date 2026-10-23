/* Reedy synth engine: sine-to-square interpolation with breathy noise and timbral vibrato.
 *   blue : interpolation amount (sine=0% to reedy=100%)
 *   green: white noise mix level after attack (0% = no noise, 100% = full noise)
 *   white: interpolation modulation depth (0% = no modulation, 100% = full range)
 *   red  : interpolation modulation speed (0 Hz to 30 Hz)
 * Copyright (C) 2026 nick (nvmsocool). GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */
#include "op1_plugin.h"
#include "reedy_warp.h"

/* Interpolation modulation frequency range: red (orange) knob maps to 0..30 Hz on a linear curve.
 * The LFO is advanced once per SAMPLE, so these are per-sample phase increments:
 * inc = f * 2^32 / 44100 (1 Hz = 97392). */
#define MOD_MIN_INC   0          /*  0   Hz */
#define MOD_MAX_INC   2921760    /* 30   Hz */

/* Interpolation modulation depth: white knob maps to 0..100% of the interpolation range.
 * At white=100%, the modulation sweeps the full 0..100% interpolation range. */
#define MOD_MAX_AMT   32767

/* Interpolation smoothing: smooth the blue knob tracking to avoid clicks. */
#define INTERP_SMOOTH 6          /* 6th-order IIR: alpha = 1/64 */

/* Output low-pass filter: gentle roll-off to tame high-frequency artifacts from the warp. */
#define LP_COEFF      16384      /* 0.5 in Q15: lp += ((sample - lp) * 16384) >> 15 */

/* Voice state (the firmware gives us 0x24 bytes: vtable + 32 bytes). */
typedef struct {
    uint32_t vtable;          /* firmware-owned */
    uint32_t ph;              /* oscillator phase (2^32 = one cycle) */
    uint32_t mod_phase;       /* interpolation modulation LFO phase (2^32 = one cycle) */
    int32_t  lp;              /* output low-pass state */
    int32_t  noise_state;     /* noise one-pole low-pass state (Q15) */
    int32_t  noise_state2;    /* noise second pole state (Q15) */
    int32_t  interp_amount;   /* current interpolation amount (Q15, smoothed) */
    int32_t  prev_block_env;  /* stock envelope value of the previous block (Q15) */
    int16_t  held;            /* 1 from note_on until note_off */
    int16_t  noise_level;     /* current noise level (Q15, 0..32767) */
} reedy_voice_t;
typedef char reedy_voice_fits[sizeof(reedy_voice_t) <= OP1_VOICE_SIZE ? 1 : -1];

/* Interpolation modulation centering: given blue value (Q15) and half-depth (Q15),
 * return the center value. If blue +/- half_depth would exceed [0, 32767], shift center
 * toward 0.5 (16384) so the range fits. */
static inline int32_t mod_center_for(int32_t blue, int32_t half_depth)
{
    int32_t lo = blue - half_depth;
    int32_t hi = blue + half_depth;
    if (lo < 0) {
        /* Shift center up so lo = 0 */
        return half_depth;
    }
    if (hi > 32767) {
        /* Shift center down so hi = 32767 */
        return 32767 - half_depth;
    }
    return blue;
}

/* xorshift32 PRNG: generates pseudo-random noise.
 * Uses a static shared state (like clarinet and arc engines). */
static uint32_t shared_rng = 0x12345678u;

static inline uint32_t xorshift32(void)
{
    shared_rng ^= shared_rng << 13;
    shared_rng ^= shared_rng >> 17;
    shared_rng ^= shared_rng << 5;
    return shared_rng;
}

static void reedy_note_on(void *self, const int16_t *params)
{
    reedy_voice_t *v = (reedy_voice_t *)self;

    /* Initialize oscillator phase. */
    v->ph = 0;

    /* Initialize interpolation modulation LFO. */
    v->mod_phase = 0;

    /* Initialize output low-pass. */
    v->lp = 0;

    /* Initialize noise filter states. */
    v->noise_state = 0;
    v->noise_state2 = 0;

    /* Initialize interpolation amount (starts at 0, smoothed to blue target). */
    v->interp_amount = 0;

    /* Initialize previous block envelope. */
    v->prev_block_env = 0;

    /* Initialize noise level (starts at 100%, fades to green target). */
    v->noise_level = 32767;

    /* Initialize held flag. */
    v->held = 1;
}

static void reedy_note_off(void *self)
{
    ((reedy_voice_t *)self)->held = 0;
}

static void reedy_render(void *self, int16_t *out, const int16_t *params)
{
    reedy_voice_t *v = (reedy_voice_t *)self;
    const int32_t *inc = OP1_PITCH_INC;

    uint32_t ph = v->ph;
    uint32_t mod_phase = v->mod_phase;
    int32_t lp = v->lp;
    int32_t noise_state = v->noise_state;
    int32_t noise_state2 = v->noise_state2;
    int32_t interp_amount = v->interp_amount;
    int32_t noise_level = v->noise_level;

    int32_t vg = op1_voice_vel_gain(self);             /* note velocity */

    /* ---- per-block control values ---- */

    /* Stock envelope: ramp linearly from previous block to this block. */
    int32_t env_end = OP1_VOICE_ENV[OP1_BLOCK - 1];
    int32_t env_q = v->prev_block_env << 7;
    int32_t env_step = env_end - v->prev_block_env;

    /* Interpolation modulation speed (orange knob): linear mapping 0..30 Hz. */
    int32_t mod_phase_inc = MOD_MIN_INC + ((MOD_MAX_INC * params[3]) >> 15);

    /* Interpolation modulation depth (white knob): 0..100% of interpolation range. */
    int32_t white_sq = (params[2] * params[2]) >> 15;  /* squared for fine control at low end */
    int32_t mod_depth = (white_sq * MOD_MAX_AMT) >> 15;

    /* Half-depth for centering calculation. */
    int32_t half_depth = mod_depth >> 1;

    /* Blue knob target (interpolation amount). */
    int32_t blue_target = params[0];

    /* Compute modulation center with clamping. */
    int32_t mod_c = mod_center_for(blue_target, half_depth);

    /* Green knob: noise target level. */
    int32_t noise_target = (params[1] * 32767) >> 15;

    /* Smooth blue knob tracking. */
    int32_t target_diff = blue_target - interp_amount;
    interp_amount += (target_diff + (target_diff > 0 ? (INTERP_SMOOTH - 1) : 0)) >> INTERP_SMOOTH;

    for (int n = 0; n < OP1_BLOCK; n++) {
        /* Smoothed envelope ramp (Q15, 0..32767). */
        env_q += env_step;
        int32_t env_val = env_q >> 7;

        /* Noise fade: from 100% at note onset toward green target during attack.
         * The envelope drives the fade: env_val goes from 0 to peak during attack. */
        int32_t noise_fade = op1_mul16(32767 - noise_target, env_val);
        noise_level = noise_target + noise_fade;
        noise_level = noise_level > 32767 ? 32767 : (noise_level < 0 ? 0 : noise_level);

        /* Interpolation modulation: sine LFO modulating the interpolation amount. */
        int32_t mod_val = (OP1_SINE_TABLE[mod_phase >> 19] * mod_depth) >> 15;
        mod_phase += mod_phase_inc;

        /* Apply modulation to interpolation amount, centered on mod_c. */
        int32_t interp = mod_c + mod_val;
        interp = interp > 32767 ? 32767 : (interp < 0 ? 0 : interp);

        /* Phase warp: 4-point bilinear in (oscillator phase row, table column). */
        int32_t s = ph >> 26;
        int32_t row_frac_q15 = (ph & 0x3FFFFFF) >> 11;

        /* Map interpolation amount to column position (0..63). */
        int32_t col_pos = interp * 64;
        int32_t col = col_pos >> 15;
        int32_t col_frac = col_pos & 0x7FFF;

        /* Clamp column to valid range. */
        if (col > 63) col = 63;

        /* Bilinear warp lookup. */
        int32_t w00 = warp_table[s][col];
        int32_t w10 = s < 63 ? warp_table[s + 1][col] : 32768;
        int32_t w01 = warp_table[s][col + 1];
        int32_t w11 = s < 63 ? warp_table[s + 1][col + 1] : 32768;
        int32_t top = w00 + op1_mul16(w01 - w00, col_frac);
        int32_t bottom = w10 + op1_mul16(w11 - w10, col_frac);
        int32_t ph_f_warped = top + op1_mul16(bottom - top, row_frac_q15);

        /* Convert warped phase to sine table index. */
        int32_t sine_idx = (ph_f_warped * 8191) >> 15;
        sine_idx = sine_idx > 8191 ? 8191 : (sine_idx < 0 ? 0 : sine_idx);

        /* Lookup sine value. */
        int32_t sample = OP1_SINE_TABLE[sine_idx];

        /* Output low-pass filter. */
        lp += ((sample - lp) * LP_COEFF) >> 15;

        /* Noise generation: xorshift32 + two-pole low-pass for breathy character. */
        uint32_t rng_out = xorshift32();
        int32_t raw_noise = ((int32_t)(rng_out >> 17)) - 16384;  /* range: -16384 .. +16383 */

        /* Two-pole low-pass cascade for breathy noise character.
         * Pole 1: alpha = 0.75 (cutoff ~11 kHz)
         * Pole 2: alpha = 0.875 (cutoff ~22 kHz)
         * Combined rolloff gives a gentle high-frequency emphasis. */
        int32_t pole1 = raw_noise;
        pole1 += ((pole1 - noise_state) >> 2);  /* alpha = 0.75 */
        noise_state = pole1;

        int32_t pole2 = pole1;
        pole2 += ((pole2 - noise_state2) >> 3);  /* alpha = 0.875 */
        noise_state2 = pole2;

        /* Mix noise at the current noise level. */
        int32_t noise_mix = op1_mul16(pole2, noise_level);

        /* Combine signal and noise. */
        int32_t mixed = lp + noise_mix;

        /* Velocity and saturation. */
        out[n] = op1_sat16((mixed * vg) >> 16);
    }

    v->ph = ph;
    v->mod_phase = mod_phase;
    v->lp = lp;
    v->noise_state = noise_state;
    v->noise_state2 = noise_state2;
    v->interp_amount = interp_amount;
    v->prev_block_env = env_end;
    v->noise_level = noise_level;
}

const op1_engine_t ENGINE_reedy = { reedy_note_on, reedy_render, reedy_note_off, { OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE } };
