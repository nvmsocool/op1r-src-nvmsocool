#!/usr/bin/env python3
"""Write rd_asm.S, reedie's Blackfin sample loops (rd_fast.h rd_loop, bit for bit), from the linear loop bodies below.

    python3 synth_sdk/plugins/reedie/gen_asm.py          (rewrites rd_asm.S next to this file)

Each body is written one instruction a line in program order; pack() then moves every load back and every store
forward into the free load/store slots of a neighbouring 32-bit instruction (Blackfin multi-issue: one 32-bit ALU /
MAC op || two 16-bit loads/stores, at most one store and one P-register access a packet), never past an instruction
that reads or writes the register, the same I-register stream or the same k word. the hundekraft kennel port harness/rd_asm_cmp.py checks the
result against the C on bcore (that harness is not part of this repo).
Copyright (C) 2026 Hundekraft. GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE).
"""
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent

# rd_ak_t (rd_fast.h pins the offsets)
FIELDS = ['FRC', 'FR', 'ONE', 'C', 'LD', 'AL', 'G', 'CH', 'B1', 'BZ', 'WP', 'WN', 'REST', 'P32767', 'M32767', 'UA',
          '8192', '14238', 'EX', '11468', 'BREATH', 'VGAIN', 'P12000', 'M12000', 'P28000', 'M28000', '6400', 'TILT',
          'GAIN', '3', 'RNG', 'LPDC', 'B1CH', 'B2', 'OP', 'BR', 'ZERO', 'FQ', 'FSTEP', 'AQ', 'ASTEP', 'RQ', 'RSTEP',
          'SQ', 'SSTEP', 'RING', 'PA', 'PB', 'W', 'OUT']
BASES = ['P1', 'P3', 'P4']   # k, k + 64, k + 128: a parallel load reaches [Preg + 0..60]


def kref(m):
    """[P1 + K_X] in a body -> the base register and offset that reach it in a 16-bit load or store"""
    i = FIELDS.index(m.group(1))
    return f'[{BASES[i // 16]} + {(i % 16) * 4}]'


NOTES = {'FRC': 'H 32767 - fr2, L 32767 - fr', 'FR': 'H fr2, L fr', 'ONE': '0x00010001', 'C': 'H c, L 32767 - c',
         'LD': 'H 32768 - alpha, L 32768 - aloss', 'AL': 'H alpha, L aloss', 'G': 'L 32760', 'CH': 'H 220, L 32548',
         'B1': 'H 934, L 31834', 'BZ': 'H 16384, L buzz', 'WP': 'H 8192', 'WN': 'H -8192', 'UA': 'H u0, L amp',
         '8192': 'L 8192', 'EX': 'ex_hi6', 'LPDC': 'H dc, L lp', 'B1CH': 'H b1, L chat',
         'BR': '32767 - 3 (age - 1): decremented before use', 'FQ': 'age fstep (fill)',
         'AQ': '(1500 - age) astep (attack blow)', 'RQ': 'age rstep (reed ramp) or rel relstep (release)',
         'SQ': '(seeddur - (age - ramplen)) sstep (chiff)'}

# the loops: (name, F fill, A attack blow, R reed ramp, S chiff, L release fade, Z released past the fade); see body().
# No F without R or S: age < di <= 2000 is below ramplen + seeddur (seeddur is 2048 once di >= 256). Released past the
# fade (rel >= 4410) inside the start-up means age >= 4410 > di, ramplen, 1500: only the chiff can be live (srel0).
VARIANTS = [('set', 0, 0, 0, 0, 0, 0), ('fr', 1, 1, 1, 0, 0, 0), ('r', 0, 1, 1, 0, 0, 0), ('fs', 1, 1, 0, 1, 0, 0),
            ('s', 0, 1, 0, 1, 0, 0), ('a', 0, 1, 0, 0, 0, 0), ('rel', 0, 0, 0, 0, 1, 0),
            ('rel0', 0, 0, 0, 0, 0, 1),
            ('frel', 1, 1, 0, 0, 1, 0), ('fsrel', 1, 1, 0, 1, 1, 0), ('arel', 0, 1, 0, 0, 1, 0), ('srel', 0, 1, 0, 1, 1, 0),
            ('srel0', 0, 0, 0, 1, 0, 1)]

SETTLED = """
/* noise: rng ^= rng << 13; ^= >> 17; ^= << 5; nz = rng.H */
R1 = R7 << 13;
R7 = R7 ^ R1;
R1 = R7 >> 17;
R7 = R7 ^ R1;
R1 = R7 << 5;
R7 = R7 ^ R1;
/* chat = (chat 32548 + |nz| 220) >> 15; b1 = (b1 31834 + nz 934) >> 15 */
R2 = [P1 + K_CH];
R3 = [P1 + K_B1];
A0 = R7.H * R2.H;
A0 = ABS A0;
R5.L = (A0 += R5.L * R2.L) (T);
A1 = R5.H * R3.L;
R5.H = (A1 += R7.H * R3.H) (T);
/* the two taps: R0 = (pb : pa), R1 = (b0 : a0) -> R0 = (b : a) */
R0.L = W[I0++];
R1.L = W[I0];
R0.H = W[I1++];
R1.H = W[I1];
R2 = [P1 + K_FRC];
R3 = [P1 + K_FR];
R4 = [P1 + K_ONE];
A1 = R1.H * R2.H, A0 = R1.L * R2.L;
A1 += R0.H * R3.H, A0 += R0.L * R3.L;
R0.H = (A1 += R1.H * R4.H), R0.L = (A0 += R1.L * R4.L) (T);
/* r = (c1 a - c b) >> 15 */
R2 = [P1 + K_C];
A0 = R0.L * R2.L;
A0 += R0.L * R4.L;
R1.L = (A0 -= R0.H * R2.H) (T);
/* lp, dc */
R3 = [P1 + K_LD];
R2 = [P1 + K_AL];
A1 = R6.H * R3.H, A0 = R6.L * R3.L;
R6.L = (A0 += R1.L * R2.L) (T);
R6.H = (A1 += R6.L * R2.H) (T);
/* r = ((lp - dc) 32760) >> 15, 32 bits: R0 */
R3 = [P1 + K_G];
A0 = R6.L * R3.L (IS);
R0 = (A0 -= R6.H * R3.L) (IS);
R0 = R0 >>> 15;
/* op += ((r - op) 6400) >> 15; col = r + ((tilt (r - op)) >> 15); Y = (col gain) >> 15: R2 */
R1 = [P1 + K_OP];
R2 = R0 - R1 (NS);
R3 = [P1 + K_6400];
R2 *= R3;
R2 = R2 >>> 15;
R1 = R1 + R2 (NS);
[P1 + K_OP] = R1;
R2 = R0 - R1 (NS);
R3 = [P1 + K_TILT];
R2 *= R3;
R2 = R2 >>> 15;
R2 = R2 + R0 (NS);
R3 = [P1 + K_GAIN];
R2 *= R3;
R2 = R2 >>> 15;
/* wob = clamp(((chat - 16384) buzz) >> 15, +-8192), in R1.H */
R3 = [P1 + K_BZ];
A1 = R5.L * R3.L;
R1.H = (A1 -= R3.H * R3.L) (T);
R3 = [P1 + K_WN];
R4 = [P1 + K_WP];
R1 = MAX(R1, R3) (V);
R1 = MIN(R1, R4) (V);
/* x = clamp(rest - r + (wob >> 3), +-32767): R3 */
R3 = [P1 + K_REST];
R3 = R3 - R0 (NS);
R4 = R1 >>> 19;
R3 = R3 + R4 (NS);
R4 = [P1 + K_P32767];
R0 = [P1 + K_M32767];
R3 = MIN(R3, R4);
R3 = MAX(R3, R0);
/* in = (wob >> 2) + (nz >> 10): R1 */
R1 = R1 >>> 18;
R4 = R7 >>> 26;
R1 = R1 + R4 (NS);
/* x2 = x x >> 15, x3 = x2 x >> 15, dx = x - x3: R0.L */
R0.L = R3.L * R3.L (T);
R0.H = R0.L * R3.L (T);
R0.L = R3.L - R0.H (NS);
/* u = (dx amp >> 13) - u0 = (dx amp - u0 8192) >> 13; u = (u 32767) >> 15; in += u */
R4 = [P1 + K_UA];
R3 = [P1 + K_8192];
A0 = R0.L * R4.L (IS);
R0 = (A0 -= R4.H * R3.L) (IS);
R0 = R0 >>> 13;
R3 = [P1 + K_P32767];
R0 *= R3;
R0 = R0 >>> 15;
R1 = R1 + R0 (NS);
/* rasp = (((nz chat) >> 15) buzz) >> 15; in += rasp */
R3 = [P1 + K_BZ];
R0.L = R7.H * R5.L (T);
R0.L = R0.L * R3.L (T);
R0 = R0.L (X);
R1 = R1 + R0 (NS);
/* b2 += ((nz - b1 - b2) 14238) >> 15: R4 */
R0 = R7 >>> 16;
R3 = R5 >>> 16;
R0 = R0 - R3 (NS);
R4 = [P1 + K_B2];
R0 = R0 - R4 (NS);
R3 = [P1 + K_14238];
R0 *= R3;
R0 = R0 >>> 15;
R4 = R4 + R0 (NS);
[P1 + K_B2] = R4;
/* Y += (ex_hi6 b2) >> 15 */
R0 = [P1 + K_EX];
R0 *= R4;
R0 = R0 >>> 15;
R2 = R2 + R0 (NS);
/* airq = clamp((((((b2 bamp) >> 15) breath) >> 14) vgain) >> 15, +-12000): R0 */
R0 = [P1 + K_BR];
R3 = [P1 + K_3];
R0 = R0 - R3 (NS);
[P1 + K_BR] = R0;
R3 = [P1 + K_11468];
R0 = MAX(R0, R3);
R0 *= R4;
R0 = R0 >>> 15;
R3 = [P1 + K_BREATH];
R0 *= R3;
R0 = R0 >>> 14;
R3 = [P1 + K_VGAIN];
R0 *= R3;
R0 = R0 >>> 15;
R3 = [P1 + K_P12000];
R4 = [P1 + K_M12000];
R0 = MIN(R0, R3);
R0 = MAX(R0, R4);
/* ring[wp] = sat16(in + (airq >> 1)) */
R3 = R0 >>> 1;
R1 = R1 + R3 (NS);
R1 = R1 << 16 (S);
W[I2++] = R1.H;
/* y = Y + airq + (airq >> 2), rounded above 28000, clamped to +-32767 */
R2 = R2 + R0 (NS);
R0 = R0 >>> 2;
R2 = R2 + R0 (NS);
R3 = [P1 + K_M28000];
R4 = [P1 + K_P28000];
R0 = MAX(R2, R3);
R0 = MIN(R0, R4);
R2 = R2 - R0 (NS);
R2 = R2 >>> 3;
R2 = R2 + R0 (NS);
R3 = [P1 + K_M32767];
R4 = [P1 + K_P32767];
R2 = MAX(R2, R3);
R2 = MIN(R2, R4);
W[P0++] = R2;
"""

FILL = """
/* fill: r = (r fq) >> 15 (fq = age fstep < 32767 while age < di); fq += fstep */
R3 = [P1 + K_FQ];
R1.L = R1.L * R3.L (T);
R2 = [P1 + K_FSTEP];
R3 = R3 + R2 (NS);
[P1 + K_FQ] = R3;
"""
U_AMP = """R4 = [P1 + K_UA];
R3 = [P1 + K_8192];
A0 = R0.L * R4.L (IS);
R0 = (A0 -= R4.H * R3.L) (IS);
"""
U_ATTACK = """/* attack blow: amp + ((amp max(aq >> 15, 0)) >> 15) (aq = (1500 - age) astep, >= -6.3e8 in the start-up); aq -= astep */
R3 = [P1 + K_AQ];
R4 = [P1 + K_ASTEP];
R4 = R3 - R4 (NS);
[P1 + K_AQ] = R4;
R3 = R3 >>> 15;
R4 = [P1 + K_ZERO];
R3 = MAX(R3, R4);
R4 = [P1 + K_UA];
R3.L = R4.L * R3.L (T);
R3.L = R3.L + R4.L (NS);
A0 = R0.L * R3.L (IS);
R3 = [P1 + K_8192];
R0 = (A0 -= R4.H * R3.L) (IS);
"""
REED_ON = """R3 = [P1 + K_P32767];
R0 *= R3;
"""
REED_RAMP = """/* reed ramp: reed = rq >> 15 (rq = age rstep, age < ramplen) */
R3 = [P1 + K_RQ];
R4 = [P1 + K_RSTEP];
R4 = R3 + R4 (NS);
[P1 + K_RQ] = R4;
R3 = R3 >>> 15;
R0 *= R3;
"""
REED_REL = """/* release: reed = 32767 - (rq >> 15) (rq = rel relstep, rel < 4410) */
R3 = [P1 + K_RQ];
R4 = [P1 + K_RSTEP];
R4 = R3 + R4 (NS);
[P1 + K_RQ] = R4;
R3 = R3 >>> 15;
R4 = [P1 + K_P32767];
R3 = R4 - R3 (NS);
R0 *= R3;
"""
CHIFF = """/* chiff: in += (nz (sq >> 15)) >> 16 (sq = (seeddur - sk) sstep, 0 <= sk < seeddur); sq -= sstep */
R3 = [P1 + K_SQ];
R4 = [P1 + K_SSTEP];
R4 = R3 - R4 (NS);
[P1 + K_SQ] = R4;
R3 = R3 >>> 15;
R3.H = R7.H * R3.L (T);
R3 = R3 >>> 17;
R1 = R1 + R3 (NS);
"""


def body(F, A, R, S, L, Z):
    """the settled loop with the start-up and release terms of rd_loop for one stretch: F the fill (age < di), A the
    attack blow (age < t1: ampe), R the reed ramp (age < ramplen), S the chiff (ramplen <= age < ramplen + seeddur),
    L the release fade (rel < 4410), Z released past it (reed 0: u is 0, the reed is skipped)"""
    b = SETTLED
    def sub(old, new):
        nonlocal b
        assert b.count(old) == 1, old
        b = b.replace(old, new)
    if F:
        sub("R1.L = (A0 -= R0.H * R2.H) (T);\n", "R1.L = (A0 -= R0.H * R2.H) (T);\n" + FILL.strip() + "\n")
    if A:
        sub(U_AMP, U_ATTACK)
    if R:
        sub(REED_ON, REED_RAMP)
    if L:
        sub(REED_ON, REED_REL)
    if S:
        sub("R0 = R0.L (X);\nR1 = R1 + R0 (NS);\n", "R0 = R0.L (X);\nR1 = R1 + R0 (NS);\n" + CHIFF.strip() + "\n")
    if Z:
        a = b.index('/* x = clamp')
        e = b.index('/* rasp')
        keep = "R1 = R1 >>> 18;\nR4 = R7 >>> 26;\nR1 = R1 + R4 (NS);\n"
        b = b[:a] + "/* released past the fade: u = 0 */\n" + keep + b[e:]
    return b


REG = re.compile(r'\b(R[0-7]|A[01]|P[0-5]|I[0-3])\b')
LOAD = re.compile(r'^(R[0-7])(?:\.[LH])? = (?:W)?\[(P[0-5]|I[0-3])([^\]]*)\]')
STORE = re.compile(r'^(?:W)?\[(P[0-5]|I[0-3])([^\]]*)\] = (R[0-7])(?:\.[LH])?$')


class Mem:
    def __init__(self, text, dst, base, off, store):
        self.text, self.reg, self.base, self.store = text, dst, base, store
        self.key = base + off.replace(' ', '') if base.startswith('P') and '++' not in off else base


class Pkt:
    def __init__(self, op=None, comment=None):
        self.op, self.mem, self.comment = op, [], comment
        if op:
            lhs = [a.split('=')[0] for a in op.split(', ')]
            self.writes = {r for s in lhs for r in REG.findall(s)} | set(re.findall(r'\((A[01]) [-+]?=', op))
            if '*=' in op:
                self.writes.add(op.split()[0])
            self.reads = set(REG.findall(op))
            self.dsp32 = not (' ^ ' in op or '*=' in op or '(X)' in op)
        else:
            self.writes, self.reads, self.dsp32 = set(), set(), False

    def touches(self, reg):
        return reg in self.reads or reg in self.writes or any(m.reg == reg for m in self.mem)

    def regs_written(self):
        return self.writes | {m.reg for m in self.mem if not m.store} | {m.base for m in self.mem if '++' in m.text}

    def room(self, m):
        if not self.dsp32 or len(self.mem) >= 2:
            return False
        if m.base.startswith('P') and any(x.base.startswith('P') for x in self.mem):
            return False
        if m.store and any(x.store for x in self.mem):
            return False
        if m.base[0] == 'I' and any(x.base == m.base for x in self.mem):
            return False
        if not m.store and m.reg in self.writes:
            return False
        if m.store and m.reg in self.regs_written():
            return False
        return all(x.reg != m.reg for x in self.mem)

    def text(self):
        if not self.mem:
            return self.op + ';'
        return ' || '.join([self.op or 'MNOP'] + [m.text for m in self.mem] + (['NOP'] if len(self.mem) == 1 else [])) + ';'


def blocks(a, m):
    """True if moving memory op m across packet a would change what it does or what a sees"""
    if any(x.key == m.key and (x.store or m.store) for x in a.mem):
        return True
    if any(x.base == m.base and m.base[0] == 'I' for x in a.mem) or (m.base[0] == 'I' and m.base in a.reads):
        return True
    if any('++' in x.text and x.base == m.base for x in a.mem) and '++' in m.text:
        return True
    if m.store:
        return m.reg in a.regs_written()
    return a.touches(m.reg)


def pack(body):
    pk = []
    for raw in body.strip().split('\n'):
        s = raw.strip()
        if s.startswith('/*'):
            pk.append(Pkt(comment=s))
            continue
        s = re.sub(r'\[P1 \+ K_(\w+)\]', kref, s.rstrip(';'))
        lm, sm = LOAD.match(s), STORE.match(s)
        if lm or sm:
            m = Mem(s, lm.group(1), lm.group(2), lm.group(3), False) if lm else Mem(s, sm.group(3), sm.group(1), sm.group(2), True)
            p = Pkt()
            p.mem.append(m)
            pk.append(p)
        else:
            pk.append(Pkt(s))
    ops = [p for p in pk if p.comment is None]
    # loads back
    i = 0
    while i < len(ops):
        p = ops[i]
        if p.op is None and len(p.mem) == 1 and not p.mem[0].store:
            m = p.mem[0]
            j = i - 1
            while j >= 0:
                a = ops[j]
                if a.room(m) and not any(x.reg == m.reg for x in a.mem):
                    a.mem.append(m)
                    p.mem = []
                    break
                if blocks(a, m):
                    break
                j -= 1
        i += 1
    ops = [p for p in ops if p.op or p.mem]
    # stores forward
    i = len(ops) - 1
    while i >= 0:
        p = ops[i]
        if p.op is None and len(p.mem) == 1 and p.mem[0].store:
            m = p.mem[0]
            j = i + 1
            while j < len(ops):
                a = ops[j]
                if a.room(m):
                    a.mem.append(m)
                    p.mem = []
                    break
                if blocks(a, m):
                    break
                j += 1
        i -= 1
    keep = set(map(id, (p for p in ops if p.op or p.mem)))
    return [p for p in pk if p.comment is not None or id(p) in keep]


def emit_loop(name, body):
    pk = pack(body)
    ops = [p for p in pk if p.comment is None]
    lines = [f'.L{name}_b:']
    for p in pk:
        if p.comment:
            lines.append('\t' + p.comment)
            continue
        if p is ops[-1]:
            lines.append(f'.L{name}_e:')
        lines.append('\t' + p.text())
    return lines, len(ops)


PRO = """[--SP] = (R7:4, P5:3);
P1 = R0;
P2 = R1;
R2 = 64;
R2 = R0 + R2;
P3 = R2;
R2 = 128;
R2 = R0 + R2;
P4 = R2;
R0 = [P1 + K_RING];
B0 = R0;
B1 = R0;
B2 = R0;
R1 = 8192;
L0 = R1;
L1 = R1;
L2 = R1;
R0 = [P1 + K_PA];
I0 = R0;
R0 = [P1 + K_PB];
I1 = R0;
R0 = [P1 + K_W];
I2 = R0;
P0 = [P1 + K_OUT];
R7 = [P1 + K_RNG];
R6 = [P1 + K_LPDC];
R5 = [P1 + K_B1CH];
LSETUP (.Ls_b, .Ls_e) LC0 = P2;"""
EPI = """[P1 + K_RNG] = R7;
[P1 + K_LPDC] = R6;
[P1 + K_B1CH] = R5;
R0 = I2;
[P1 + K_W] = R0;
R0 = 0;
L0 = R0;
L1 = R0;
L2 = R0;
(R7:4, P5:3) = [SP++];
RTS;"""


def main():
    out = [f'/* rd_asm.S: reedie\'s sample loops (rd_fast.h rd_loop: settled, start-up, release) in Blackfin assembly, bit for bit.',
           ' * Generated by gen_asm.py (edit that, not this file).',
           ' * void rd_k_<variant>(rd_ak_t *k, int count): R0 = k (rd_fast.h rd_ak_t: ring pointers, packed constants, state),',
           ' * R1 = count >= 1. The C (rd_loop) is the spec and the host path; the hundekraft kennel port harness/rd_asm_cmp.py compares the two on bcore.',
           ' *',
           ' * Ranges the 16-bit multiplies rest on (rd_fast.h rd_asm_run checks the incoming state, the rest is by construction):',
           ' *  - the two taps: a = a0 + ((pa - a0) fr >> 15) = (a0 (32767 - fr) + pa fr + a0) >> 15, a blend of two int16, so int16;',
           ' *  - r = (c1 a - c b) >> 15 = (a (32767 - c) + a - c b) >> 15, within int16 for int16 a, b;',
           ' *  - lp, dc: lp + ((r - lp) k >> 15) = (lp (32768 - k) + r k) >> 15 with 0 < k < 32768: stays between lp and r;',
           ' *  - chat: (chat 32548 + |nz| 220) >> 15 in 0..32767 (|nz| up to 32768: the product\'s absolute value in A0);',
           ' *  - b1: (b1 31834 + nz 934) >> 15, int16; x clamped to +-32767, x2, x3 and x - x3 int16; u0 < 32768 (rest <= 10853);',
           ' *  - r after the DC blocker, b2, op, u, col, airq and the output are 17 bits or more: 32-bit ops (*= for products).',
           ' * Fraction-mode products are doubled and a (T) half extraction takes bits 31..16: floor(sum / 32768). No operand',
           ' * pair is -32768 x -32768 (every constant is >= 0, x is clamped to +-32767, chat >= 0).',
           ' * Copyright (C) 2026 Hundekraft. Derived from reedie.c, Copyright (C) 2026 nvmsocool.',
           ' * GPL-3.0-or-later with the OP-1 firmware permission (see LICENSE and NOTICE). */',
           '#ifdef __bfin__                 /* the host builds use the C */', '', '/* rd_ak_t offsets (rd_fast.h pins them) */']
    for i, f in enumerate(FIELDS):
        out.append(f'#define K_{f:<8}{i * 4}' + (f'      /* {NOTES[f]} */' if f in NOTES else ''))
    out += ['', '/* Each loop: P1 = k, P3 = k + 64, P4 = k + 128, P0 = out, I0 = pa (then a0), I1 = pb (then b0), I2 = write, all',
            ' * three on the ring (circular, 8192 bytes). Kept: R7 = rng, R6 = (dc : lp), R5 = (b1 : chat); b2, op, the breath',
            ' * ramp and the start-up sums in k. */', '.text']
    counts = []
    for name, *flags in VARIANTS:
        loop, n = emit_loop(name, body(*flags))
        counts.append((name, n))
        fn = f'_rd_k_{name}'
        out += ['\t.balign 32', f'\t.global {fn}', f'\t.type {fn}, STT_FUNC',
                f'/* rd_k_{name}(k, count): F {flags[0]} A {flags[1]} R {flags[2]} S {flags[3]} L {flags[4]} Z {flags[5]} (body()); '
                f'{n} packets a sample */', f'{fn}:']
        out += ['\t' + x for x in PRO.replace('.Ls_', f'.L{name}_').split('\n')] + loop + ['\t' + x for x in EPI.split('\n')]
        out += [f'\t.size {fn}, .-{fn}', '']
    out += ['#endif', '']
    (HERE / 'rd_asm.S').write_text('\n'.join(out))
    print('rd_asm.S: ' + ', '.join(f'{a} {n}' for a, n in counts) + ' packets a sample')


if __name__ == '__main__':
    main()
