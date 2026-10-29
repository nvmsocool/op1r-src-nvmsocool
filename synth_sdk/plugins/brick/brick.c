/* "brick" - a synth engine that is just a brick.
 *
 * It makes no sound. The keys trigger silent voices, the green / white / red knobs edit
 * parameters that nothing reads, and the blue knob spins the brick. That is the whole engine.
 *
 * Copyright (C) 2026 the op1r contributors. GPL-3.0-or-later with the OP-1 firmware permission
 * (see LICENSE and NOTICE). */
#include "op1_plugin.h"

static void br_note_on(void *self, const int16_t *params)
{
    (void)self;
    (void)params;
}

static void br_render(void *self, int16_t *out, const int16_t *params)
{
    (void)self;
    (void)params;
    for (int n = 0; n < OP1_BLOCK; n++)
        out[n] = 0;
}

const op1_engine_t ENGINE_brick = {
    br_note_on, br_render, 0,
    { OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE, OP1_PRESS_NONE }
};

/* ======================= screen (GUI thread) =======================
 * The brick is a 3D box seen in orthographic projection. brick.svg holds six face groups in
 * local 2D coordinates; every frame each visible face gets an affine transform that is the
 * projection of its 3D rectangle. The blue knob turns the box around its vertical axis.
 *
 * Model axes: x right, y up, z towards the viewer. The camera looks down -z with a fixed 22
 * degree tilt, so the top face is always visible and the four sides take turns at the front. */
#pragma GCC push_options
#pragma GCC optimize("Os")

#define BW 168                      /* brick length */
#define BH 68                       /* brick height */
#define BD 48                       /* brick depth */
#define MHW (BW / 2)
#define MHH (BH / 2)
#define MHD (BD / 2)

#define M(v) ((int32_t)(v) * 1024)  /* model units -> Q10 */
#define CX_Q10 (172 * 1024)         /* screen centre, Q10 */
#define CY_Q10 (80 * 1024)
#define SCALE_Q10 1075              /* 1.05: 1024 * 1.05 */
#define PITCH_C 949                 /* cos 22 deg, Q10 */
#define PITCH_S 384                 /* sin 22 deg, Q10 */
#define BASE_YAW (-455)             /* -20 deg in 1/8192 turn */
#define SHADOW_DY (8 * 1024)        /* drop shadow: the footprint shifted down, Q10 px */

/* Each face group's local SVG units are model millimetres: one SVG unit along +x is the world
 * direction u, one along +y (down) is v. Axes are unit vectors, the projection scales them. */
#define U 1024                          /* 1.0 in Q10 */

typedef struct {
    int32_t px, py, pz;                 /* local (0,0) corner, Q10 model space */
    int32_t ux, uy, uz;                 /* local x axis, unit vector in Q10 */
    int32_t vx, vy, vz;                 /* local y axis, unit vector in Q10 */
    void *node;
} brick_face_t;

static brick_face_t faces[6] = {
    /* front: origin top-left seen from +z; x = +x, y (down) = -y */
    { M(-MHW), M(MHH), M(MHD),     U, 0, 0,    0, -U, 0,   0 },
    /* back: origin top-left seen from -z; x = -x, y (down) = -y */
    { M(MHW),  M(MHH), M(-MHD),    -U, 0, 0,   0, -U, 0,   0 },
    /* right: origin top-left seen from +x; x = -z, y (down) = -y */
    { M(MHW),  M(MHH), M(MHD),     0, 0, -U,   0, -U, 0,   0 },
    /* left: origin top-left seen from -x; x = +z, y (down) = -y */
    { M(-MHW), M(MHH), M(-MHD),    0, 0, U,    0, -U, 0,   0 },
    /* top: origin back-left seen from +y; x = +x, y (down) = +z (towards the viewer) */
    { M(-MHW), M(MHH), M(-MHD),    U, 0, 0,    0, 0, U,    0 },
    /* bottom: hidden in this view; x = +x, y (down) = -z */
    { M(-MHW), M(-MHH), M(MHD),    U, 0, 0,    0, 0, -U,   0 }
};

static void *shadow_node;               /* the projected footprint, drawn under the brick */

/* orthographic projection of a Q10 model vector: R = Rx(22 deg) * Ry(yaw), then scale */
static void br_proj(int32_t vx, int32_t vy, int32_t vz, int32_t sy, int32_t cy,
                    int32_t *x, int32_t *y)
{
    int32_t x1 = (cy * vx + sy * vz) >> 10;
    int32_t z1 = (-sy * vx + cy * vz) >> 10;
    int32_t y2 = (PITCH_C * vy - PITCH_S * z1) >> 10;
    *x = (x1 * SCALE_Q10) >> 10;
    *y = -((y2 * SCALE_Q10) >> 10);
}

static void br_face_matrix(const brick_face_t *f, int32_t sy, int32_t cy, int32_t m[9])
{
    int32_t ux, uy, vx, vy, px, py;
    br_proj(f->ux, f->uy, f->uz, sy, cy, &ux, &uy);
    br_proj(f->vx, f->vy, f->vz, sy, cy, &vx, &vy);
    br_proj(f->px, f->py, f->pz, sy, cy, &px, &py);
    m[0] = ux; m[1] = uy; m[2] = 0;
    m[3] = vx; m[4] = vy; m[5] = 0;
    m[6] = px + CX_Q10; m[7] = py + CY_Q10; m[8] = 1024;
}

static void br_pose(int32_t knob)
{
    /* centre = the front/right/top view; the knob turns the brick like a turntable (clockwise
     * knob = clockwise from above) and sweeps half a turn each way. */
    int32_t spin = (16384 - knob) >> 2;
    int32_t a = (BASE_YAW + spin) & 8191;
    int32_t sy = (int32_t)OP1_SINE_TABLE[a] >> 5;          /* Q10 */
    int32_t cy = (int32_t)OP1_SINE_TABLE[(a + 2048) & 8191] >> 5;
    int32_t m[9];

    for (int i = 0; i < 6; i++) {
        int vis;
        switch (i) {
        case 0: vis = cy > 0; break;                        /* front */
        case 1: vis = cy < 0; break;                        /* back */
        case 2: vis = sy < 0; break;                        /* right */
        case 3: vis = sy > 0; break;                        /* left */
        case 4: vis = 1; break;                             /* top */
        default: vis = 0; break;                            /* bottom */
        }
        if (!faces[i].node) continue;
        op1_node_set_visible(faces[i].node, vis);
        if (!vis) continue;

        br_face_matrix(&faces[i], sy, cy, m);
        op1_node_set_transform(faces[i].node, m);
    }

    /* shadow: the bottom face's footprint, nudged down the screen so a sliver shows */
    if (shadow_node) {
        br_face_matrix(&faces[5], sy, cy, m);
        m[7] += SHADOW_DY;
        op1_node_set_transform(shadow_node, m);
    }
}

static void br_screen_init(void *scr)
{
    static const char *const ids[6] = { "front", "back", "right", "left", "top", "bottom" };
    for (int i = 0; i < 6; i++)
        faces[i].node = op1_find_node(scr, ids[i]);
    shadow_node = op1_find_node(scr, "shadow");
    br_pose(16384);                                         /* default blue knob: base view */
}

static void br_screen_update(void *scr, int dt)
{
    int16_t k[4];
    op1_synth_knobs_live(OP1_SCREEN_SLOT & 7, k);
    br_pose(k[0]);
    op1_screen_draw(scr, dt);
}

#pragma GCC pop_options
const op1_screen_t SCREEN_brick = { br_screen_init, br_screen_update };
