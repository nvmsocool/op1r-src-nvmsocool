# brick

A synth engine that is a brick.

It makes no sound. The keys trigger silent voices and the screen shows a brick. The blue knob turns
the brick; the other three knobs edit parameters that nothing reads. That is the entire feature set.

| knob | parameter |
|---|---|
| blue | spin the brick (±180°) |
| green | nothing |
| white | nothing |
| red | nothing |

The screen is a 3D brick in orthographic projection. `brick.svg` holds six face groups in local 2D
coordinates and `brick.c` gives each visible face a per-frame affine transform, so the box turns
around its vertical axis as the blue knob moves. The camera has a fixed 22° tilt, so the top face is
always visible; front/back and left/right swap at the silhouette edges. The bottom face is never
visible in this view. All of the projection runs in Q10 integer math against the firmware sine
table—no floats, no 64-bit multiply.

The spin range is ±180° around a base yaw of −20°, so the knob centre is the classic three-quarter
view of a brick, not a flat-on front face. Turning the knob clockwise turns the brick clockwise seen
from above, like a turntable. The drawing has three holes in the top, pitting on the sides and a drop
shadow made from the projected footprint.

## Files and building

```sh
cd synth_sdk
python3 make_plugin.py plugins/brick     # -> build/plugins/brick.op1p
python3 harness/render.py brick          # -> display/brick_preview.png
```

`harness/audio.py brick` also runs, but exits with `FAIL: output silent`. That is the point.
