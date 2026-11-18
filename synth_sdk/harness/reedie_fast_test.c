/* Host test: plugins/reedie/rd_fast.h (rd_render_fast) against reedie.c's rd_render, bit for bit. Two voices (0 the
 * original, 1 the fast one) get the same knobs, pitch increment, velocity and note on / off, block after block; every
 * output sample and the final voice state must be equal. Knob ends x increments over the whole range (clamps, a settled
 * note, the release), random knobs / velocities / increments, knob moves and pitch glides during a note.
 *   cc -O1 -g -Wall -Wextra -DOP1_HOST -fsanitize=address,undefined -fno-sanitize-recover=undefined -I.. \
 *      reedie_fast_test.c -lm -o /tmp/reedie_fast_test && /tmp/reedie_fast_test
 * Copyright (C) 2026 Hundekraft. GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */
#include "../op1_plugin.h"
#include <stdio.h>
#include <string.h>

int16_t host_sine[8192]; int32_t host_pitch_inc[128]; uint8_t host_voice_mem[6 * 0x24]; int16_t host_voice_vel[6];
int16_t host_voice_env[128]; int16_t host_accel[3]; int16_t host_knobs[8][8]; int host_slot; const uint8_t *host_sample_buffer[8];
int16_t host_fx_knobs[3][8]; int16_t host_fx_meter[3]; void *host_fx_scratch[3];

#define RD_REFERENCE 1                       /* reedie.c keeps rd_render as the engine; rd_fast.h is included below */
#include "../plugins/reedie/reedie.c"
#include "../plugins/reedie/rd_fast.h"

static int exact_run(const int16_t *k, int32_t inc, int vel, int hold, int total, int knob_at, int glide)
{
    int16_t kk[8]; memcpy(kk, k, sizeof kk);
    static int16_t oa[OP1_BLOCK], ob[OP1_BLOCK];
    memset(host_voice_mem, 0, sizeof host_voice_mem); memset(rd_ring, 0, sizeof rd_ring);
    memset(rd_pc, 0, sizeof rd_pc); memset(&rd_kc, 0, sizeof rd_kc);
    host_voice_vel[0] = host_voice_vel[1] = (int16_t)(vel << 8);
    void *va = host_voice_mem, *vb = host_voice_mem + 0x24;
    rd_note_on(va, kk); rd_note_on(vb, kk);
    ((reedie_voice_t *)vb)->rng = ((reedie_voice_t *)va)->rng;          /* note_on seeds from the address */
    int bad = 0;
    for (int c = 0; c < total; c++) {
        if (c == hold) { rd_note_off(va); rd_note_off(vb); }
        if (knob_at && c == knob_at) kk[3] = (int16_t)(kk[3] ^ 9000);
        if (glide && c % 7 == 0) inc += glide;
        host_pitch_inc[0] = inc;
        rd_render(va, oa, kk); rd_render_fast(vb, ob, kk);
        if (memcmp(oa, ob, sizeof oa)) bad++;
    }
    if (memcmp((uint8_t *)va + 4, (uint8_t *)vb + 4, 0x20)) bad++;
    return bad;
}

int main(void)
{
    static const int16_t ext[] = { 0, 1, 8191, 8192, 16384, 22937, 22938, 32766, 32767, -5 };
    static const int32_t incs[] = { 1, 2, 3, 1000, 65535, 65536, 65537, 400000, 3000000, 6300000, 24000000, 100000000,
                                    536870911, 536870912, 600000000, 0x7fffffff, 0, -4 };
    int runs = 0; long bad = 0; unsigned q = 0;
    for (int a = 0; a < 10; a++) for (int b = 0; b < 10; b++) for (int c = 0; c < 10; c++) for (int d = 0; d < 10; d++) {
        if ((a * 7 + b * 3 + c + d) % 5 && !(a == 0 && b == 0) && !(d == 5 && c == 4)) continue;   /* a spread of the 10^4 combinations */
        int16_t k[8] = { ext[a], ext[b], ext[c], ext[d], 0x5300, 0, 0, 0 };
        int32_t inc = incs[q++ % (sizeof incs / sizeof *incs)];
        bad += exact_run(k, inc, (int)(q % 127) + 1, 70 + (int)(q % 3) * 10, 100, 0, 0); runs++;
    }
    uint32_t seed = 5;
    for (int i = 0; i < 300; i++) {
        int16_t k[8] = { 0, 0, 0, 0, 0x5300, 0, 0, 0 };
        for (int j = 0; j < 4; j++) { seed = seed * 1664525u + 1013904223u; k[j] = (int16_t)((seed >> 8) & 32767); }
        seed = seed * 1664525u + 1013904223u; int32_t inc = (int32_t)(1000000u + (seed >> 4) % 90000000u);
        seed = seed * 1664525u + 1013904223u; int vel = 1 + (int)((seed >> 8) % 127);
        bad += exact_run(k, inc, vel, 50 + (int)(seed % 50), 110, (i & 1) ? 62 : 0, (i & 2) ? (int)(seed % 20000) - 10000 : 0); runs++;
    }
    printf("rd_render_fast vs rd_render: %d runs, %ld differing blocks (bit-exact: 0)\n", runs, bad);
    return bad != 0;
}
