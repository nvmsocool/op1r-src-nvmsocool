#!/usr/bin/env python3
"""Generate the three-point exponential phase warp table for the reedy synth engine.

The warp function (from user's spec):
  For each third of the phase cycle:
    ph_f_sym = (ph_f - anchor) / half_width    # map to [-1, 1]
    abs_value = |ph_f_sym|
    x' = (exp(timbre * abs_value) - 1) / (exp(timbre) - 1)
    ph_f_warped = anchor + sign(ph_f_sym) * x' * half_width

At timbre=0: ph_f_warped = ph_f (identity, pure sine)
At timbre=30: phase squishes toward three anchors (0%, 50%, 100%)

Output: C array of Q15 values in [0, 32767] representing [0, 1].
"""
import math

NUM_SAMPLES = 64  # 6-bit phase resolution
MAX_TIMBRE = 30
NUM_COLUMNS = 64

def warp(ph_f, timbre):
    """Warp normalized phase ph_f in [0,1] toward three anchors by timbre."""
    if abs(timbre) < 0.01:
        return ph_f

    # Determine which third we're in
    if ph_f < 0.25:
        anchor = 0.0
        half_width = 0.25
    elif ph_f < 0.75:
        anchor = 0.5
        half_width = 0.25
    else:
        anchor = 1.0
        half_width = 0.25

    # Map to [-1, 1] relative to anchor
    ph_f_sym = (ph_f - anchor) / half_width

    # Apply exponential squish
    abs_value = abs(ph_f_sym)
    denom = math.exp(-timbre) - 1.0
    if abs(denom) < 1e-10:
        return ph_f
    numerator = math.exp(-timbre * abs_value) - 1.0
    x_warped = numerator / denom
    if ph_f_sym < 0:
        x_warped = -x_warped

    # Map back to [0, 1]
    ph_f_warped = anchor + x_warped * half_width
    return max(0.0, min(1.0, ph_f_warped))

def to_q15(v):
    """Convert float in [0, 1] to Q15 int."""
    v = max(0.0, min(1.0, v))
    return int(v * 32767)

lines = []
lines.append("/* Three-point exponential phase warp table: warp_table[s][t] = Q15 warped phase in [0,1] */")
lines.append("/* s = 0..63 (phase index), t = 0..64 (timbre 0..100 mapped from column) */")
lines.append("static const int16_t warp_table[64][65] = {")

for s in range(NUM_SAMPLES):
    ph_f = s / NUM_SAMPLES  # 0 to 1
    lines.append("    {  // s=%d, ph_f=%.4f" % (s, ph_f))

    vals = []
    for t in range(NUM_COLUMNS + 1):
        timbre = t * MAX_TIMBRE / NUM_COLUMNS
        ph_f_warped = warp(ph_f, timbre)
        vals.append("%6d" % to_q15(ph_f_warped))
        if len(vals) == 8:
            lines.append("        " + ", ".join(vals) + ",")
            vals = []
    if vals:
        lines.append("        " + ", ".join(vals) + ",")
    lines.append("    },")

lines.append("};")

print("\n".join(lines))
