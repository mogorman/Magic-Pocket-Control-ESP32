/*
  Cold-shoe mount for a 43.3 x 43.3 x 22.5 mm device
  (rounded corners; straight edge between them is 38.99 mm)

  Two printed parts, selected with `part`:
    1 = shoe_mount()    the cold shoe mount: 25 x 30 plate, 3.5 mm capture
                        lip, M3 tap recess in the middle
    2 = pocket()        the pocket the device drops into; grips the device
                        by friction fit
    3 = both, assembled

  How it holds:
    - the device drops into the pocket from the top (screen facing up) and
      is gripped by the walls (friction fit, set with `fit`)
    - the pocket is screwed to the mount with one M3 SHCS: its head sits
      recessed in the pocket floor (below the device) and its tip threads
      into a blind tap recess in the mount's top face

  Assembly order:
    1. tap the recess in the mount with an M3 tap
    2. seat the mount in the camera cold shoe (lip end first)
    3. place the pocket on the mount and screw in the M3 SHCS from above
       (the head recesses into the floor, flush below where the device sits)
    4. drop the device into the pocket

  Tuning the fit: FDM printers usually run slightly large, so the default
    fit = -0.2 (pocket 0.1 mm smaller per side than the device) typically
    lands as a tight friction fit you can push in by hand and pull out
    with a firm tug. If your printer runs small, try 0 (line-to-line) or
    0.2 (loose drop-in).

  Printing: each part prints with its flat face down, no supports needed.

  Wall layout (top view, screen facing up):
    front (-y) : 3 buttons      back (+y) : microSD slot
    left  (-x) : plain          right (+x): USB-C
  The shoe lip is on the USB side by default; flip it with lip_side.
*/

$fn = 40;

// ================= Part selector =================
part = 2;   // 1 = cold shoe mount, 2 = pocket, 3 = both

// ================= Device =================
obj_w     = 46.6;    // width
obj_d     = 46.6;    // depth
obj_h     = 22.5;    // height
obj_flat  = 38.99;   // straight edge between the rounded corners
obj_r     = (obj_w - obj_flat) / 2;   // device corner radius (2.155)

// ================= Pocket (part 2) =================
// Friction fit: 0 = line-to-line, negative = interference (grip),
// positive = clearance (loose). See the note above.
fit     = -0.2;
wall_t  = 3;         // wall thickness
floor_t = 5;         // pocket floor thickness (thick enough for a blind head recess)
wall_lip = 1;        // wall height above the device top (grip lip)

inner_w = obj_w + 2 * fit;
inner_d = obj_d + 2 * fit;
outer_w = inner_w + 2 * wall_t;
outer_d = inner_d + 2 * wall_t;
inner_r = obj_r + fit;
outer_r = inner_r + wall_t;
wall_h  = obj_h + wall_lip;   // wall height above the floor's top face

// --- access cutouts -----------------------------------------------------
// each cutout: [width, height, height of its centre from the device top,
//               offset of its centre along the wall from the wall's centre]
usb_cut = [16, 10, 8.6, 0];    // USB-C, right wall (+x)
usb_r   = 2;                   // USB-C window corner radius (rounded rectangle)
sd_cut  = [19, 6, 8.3, 0];     // microSD slot, back wall (+y) - widened for easy removal
sd_r    = 2.5;                 // microSD window corner radius (rounded rectangle)

// microSD window is open at the top: it covers the slot and runs up past the
// wall's top edge so the card can be pushed in from above.
sd_win_top = floor_t + wall_h + 1;                              // 1 mm above the wall top
sd_win_bot = floor_t + (obj_h - sd_cut[2]) - sd_cut[1] / 2 - 4; // slot bottom, minus 4 mm
sd_win_h   = sd_win_top - sd_win_bot +3;
sd_win_c   = (sd_win_top + sd_win_bot) / 2;

// one large rounded-rect window covering the whole button row, front wall (-y):
//   width  = 2 pitches (10 mm) + 6 mm = 26 mm
//   height = twice the old single holes (6 mm -> 12 mm), centred on the buttons
btn_win_w  = 29;
btn_win_h  = 28;
btn_win_r  = 3;               // corner radius
btn_z      = 9.6;             // window centre = button centre, measured down from the device top
btn_pitch  = 10;              // button pitch (useful when tuning the window width)

// --- joint to the cold shoe mount ---------------------------------------
// One M3 SHCS joins the pocket to the mount: head recessed in the pocket
// floor (below the device), tip threaded into a blind tap recess in the mount.
tap_d        = 2.6;         // mount tap recess: M3 (3.0 for a self-tapper)
tap_dep      = 2.0;         // tap-recess depth into the mount plate
screw_hole_d = 3.3;         // shank clearance hole through the pocket floor
csbore_d     = 5.8;         // head-recess (counterbore) diameter
screw_head_h = 2.5;         // M3 SHCS head height
screw_recess = 1.0;         // head top sits this far BELOW the floor's top face
csbore_t     = screw_head_h + screw_recess;  // head-recess depth (blind, not through)

// ================= Cold shoe mount (part 1) =================
base_w   = 25;
base_d   = 30;
base_t   = 3;
lip_h    = 3.5;    // capture-lip height (camera-shoe standard)
lip_len  = 2;      // lip depth along the plate
base_r   = 1;      // plate corner round
lip_side = 1;      // 1 = lip on the +y (USB) side, -1 = on the -y (button) side

// ================= Helpers =================

// 2D rounded rectangle, w x d, corner radius r, centred on the origin
module rounded_rect(w, d, r) {
  union() {
    square([w - 2 * r, d], true);
    square([w, d - 2 * r], true);
    for (sx = [-1, 1])
      for (sy = [-1, 1])
        translate([sx * (w / 2 - r), sy * (d / 2 - r)])
          circle(r);
  }
}

// ================= Part 1: cold shoe mount =================
module shoe_mount() {
  difference() {
    union() {
      // main plate
      linear_extrude(base_t)
        rounded_rect(base_w, base_d, base_r);
      // capture lip: 0.5 mm proud band on one end of the plate
      translate([0, lip_side * (base_d / 2 - lip_len / 2), lip_h / 2])
        cube([base_w, lip_len, lip_h], true);
    }
    // blind M3 tap recess in the top face: the screw tip seats flush in this
    translate([0, 0, base_t - tap_dep])
      cylinder(h = tap_dep, d = tap_d);
  }
}

// ================= Part 2: pocket =================
module pocket() {
  difference() {
    union() {
      // floor
      linear_extrude(floor_t)
        rounded_rect(outer_w, outer_d, outer_r);
      // walls, rising from the floor's top face
      translate([0, 0, floor_t])
        linear_extrude(wall_h)
          rounded_rect(outer_w, outer_d, outer_r);
    }

    // the pocket cavity the device drops into, open at the top
    translate([0, 0, floor_t])
      linear_extrude(wall_h)
        rounded_rect(inner_w, inner_d, inner_r);

    // shank clearance hole through the floor, plus a blind head recess at the
    // top face (the screw head sits recessed in it, below the device)
    translate([-5.91-0.125, 0, 0])
    cylinder(h = floor_t, d = screw_hole_d);
    translate([-5.91-0.125, 0, floor_t - csbore_t])
      cylinder(h = csbore_t, d = csbore_d);


   // shank clearance hole through the floor, plus a blind head recess at the
    // top face (the screw head sits recessed in it, below the device)
    translate([5.91+0.125+0.05, 0, 0])
    cylinder(h = floor_t, d = screw_hole_d);
    translate([5.91+0.125+0.05, 0, floor_t - csbore_t])
      cylinder(h = csbore_t, d = csbore_d);

    // microSD slot cutout, back wall (+y): rounded rectangle, open at the top
    #translate([sd_cut[3], inner_d / 2 - 1, sd_win_c])
      rotate([-90, 0, 0])
        linear_extrude(wall_t + 2)
          rounded_rect(sd_cut[0], sd_win_h, sd_r);

    // USB-C cutout, right wall (+x): rounded rectangle
    translate([4+ inner_d / 2, usb_cut[3], floor_t + (obj_h - usb_cut[2])])
      rotate([90, 0, 0])
        rotate([0, -90, 0])
          linear_extrude(wall_t + 2)
            rounded_rect(usb_cut[0], usb_cut[1], usb_r);

    // button window, front wall (-y): one large rounded rectangle
    translate([0, 1 - inner_d / 2, floor_t + (obj_h - btn_z)])
      rotate([90, 0, 0])
        linear_extrude(wall_t + 2)
          rounded_rect(btn_win_w, btn_win_h, btn_win_r);
  }
}

// ================= Build =================
if (part == 1)
  shoe_mount();
else if (part == 2)
  pocket();
else
  union() {
    shoe_mount();
    translate([0, 0, base_t])
      pocket();
  }
