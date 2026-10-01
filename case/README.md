# 3D-printed case — cold shoe mount

A two-part case that drops the Magic Pocket Control device onto a camera cold shoe.

- `cold_shoe_case.scad` — the parametric OpenSCAD model for both parts.

## Build

Render with [OpenSCAD](https://openscad.org/):

```sh
openscad -o cold_shoe_case.svg case/cold_shoe_case.scad   # preview
```

or generate STLs directly per part by setting `part` in the file to `1` or `2`.

## Parts

Set `part` at the top of the SCAD to export each:

| `part` | Part | Notes |
| ------ | ---- | ----- |
| 1 | Cold shoe mount | 25 x 30 mm plate, 3.5 mm capture lip, M3 tap recess |
| 2 | Pocket | friction-fit sleeve the device drops into |
| 3 | Assembled | preview only, not for printing |

Print each with its flat face down — no supports needed.

## Assembly

1. Tap the recess in the mount with an M3 tap.
2. Seat the mount in the camera cold shoe (lip end first).
3. Place the pocket on the mount and screw in one M3 SHCS from above.
4. Drop the device into the pocket.

## Tuning the fit

FDM printers usually run slightly large, so `fit = -0.2` gives a tight
friction fit (push in by hand, pull out with a firm tug). If your printer
runs small, try `fit = 0` (line-to-line) or `fit = 0.2` (loose drop-in).

Wall layout (top view, screen up): front = 3 buttons, back = USB-C,
left = microSD slot, right = plain. Flip the shoe lip with `lip_side`.
