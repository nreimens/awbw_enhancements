# Laser Hockey Trainer: Project Context

A Raspberry Pi stickhandling trainer. A stepper-driven pan/tilt head aims a laser dot at
the floor. A camera tracks a puck or ball, and when the player moves it onto the dot, the
dot jumps to a new spot. The goal is an **open-source design** that should eventually run
on an **embedded platform**, not a Pi.

Owner: Nate, an engineer in medical device development who is comfortable with
electronics, 3D printing, Python and CAD. Keep explanations technical and to the point.

## Current status

- **Mechanical:** the first print-ready revision is done. It's a parametric OpenSCAD model
  with 6 printed parts (see below), and a software sweep found no collisions between
  moving parts. The parts are being printed and fit-tested now.
- **Software:** not started yet. That's the next big chunk of work.

## Hardware on hand (prototype)

| Item | Notes |
|---|---|
| Raspberry Pi 4B | Main computer for the prototype |
| Adafruit DC & Stepper Motor HAT (PCA9685 + 2× TB6612) | I2C at 0x60. Stepper 1 = M1/M2, stepper 2 = M3/M4. Datasheet PDF is in the repo. |
| 2× 28BYJ-48 (5 V, unipolar) | Pan + tilt. About 2048 full steps/rev through the 64:1 gearbox, ~0.18°/step, with gear backlash of roughly 1–2°. Red center tap goes to the GND terminal between each coil pair. Datasheet: `stepd-01-data-sheet-1143075.pdf` |
| HiLetgo 6 mm 650 nm 5 mW laser diode (5 V, built-in resistor) | **Class 3R.** Aim only downward, clamp tilt in software, and consider PWM-dimming it. Switch it through an NPN or MOSFET, not straight from GPIO. |
| Arducam OV9281 1 MP global-shutter USB (UVC) camera, M12 lens | Monochrome, up to 120 fps. 38×38 mm board with M2 holes on a 34 mm square. Datasheet PDF is in the repo. |

**Monochrome camera implication:** the OV9281 sees no color, so color-blob tracking won't
work. Track brightness and contrast instead: a white or very bright ball on a darker
floor, a reflective sticker on the puck, or background subtraction. The laser dot shows
up as a small saturated spot. A 650 nm bandpass or IR-cut filter question is open.

## Mechanical design (`hockey_trainer_pantilt.scad`)

The whole layout is about 100 × 70 mm and ~11 cm tall:
- **Tray (base):** holds the Pi 4B and the HAT stack. The Pi sits on 12 mm bosses.
  The floor has a 1/4"-20 tripod nut trap. Port cutouts: USB-C, HDMI and audio on −Y;
  the USB/Ethernet end is open; there's a microSD slot and a wire notch by the HAT
  terminal blocks.
- **Deck (lid):** carries the pan motor (shaft up, in a locating collar on two posts)
  and the camera hinge ears. A wire slot under the pan motor lets leads drop into the base.
- **Pan yoke:** has a double-D bore on the pan shaft with an M3 grub screw. The tilt
  motor bolts to the outside of its upright, and the tilt axis is 52 mm above the deck.
- **Laser holder:** double-D hub on the tilt shaft with an M3 grub screw and a 6 mm
  bore for the laser. The laser sits close to the pan axis.
- **Camera plate:** H-frame with M2 standoffs, hinged on one M3 bolt, adjustable 0–85°
  downward.
- **Pi spacers:** 4× 11 mm long, 6 mm OD, with M2.5 clearance holes, fitted between
  the Pi and the HAT.

Export a part: `openscad -D 'part="tray"' -o stl/tray.stl hockey_trainer_pantilt.scad`
The `part` values are `assembly | tray | deck | pan_yoke | laser_holder | camera_plate | pi_spacers`.
Preview the assembly at given angles with `-D pan_angle=.. -D tilt_angle=.. -D cam_angle=..`.

### Measured / tuned values (don't revert)
- `stack_h = 25.4`: measured with calipers, from the bottom of the Pi PCB to the top of the HAT terminal blocks.
- `standoff_h = 12`: raised from 8 because parts under the Pi hit the tripod nut boss.
- `m25_pilot = 2.5`: raised from 2.2 because the printed holes were too tight for M2.5.
- `m2_pilot = 1.7`: the camera board uses M2 screws, not M2.5.
- Pilot holes are self-tapping sizes for PLA/PETG. Printed holes come out ~0.2–0.3 mm undersize.

### Design conventions
- Every fastener goes into a printed pilot hole. Heat-set inserts are an optional upgrade.
- Parts must print without supports. Each part's print orientation is set in the
  `if (part == ...)` block at the bottom of the file.
- After any geometry change, re-export the affected STLs, check that they're watertight
  (`trimesh`), and re-run the collision sweep (intersect the moving parts with the
  static ones across pan/tilt/camera angles; an empty result means clear). Render a PNG
  with `xvfb-run openscad -o x.png --camera=...` and actually look at it.

## Hardware list

| Qty | Part | Where |
|---|---|---|
| 2 | M3×8–10 | Pan motor tabs → deck posts |
| 2 | M3×5–6 | Tilt motor tabs → yoke upright (the wall is 4 mm) |
| 2 | M3×6 grub | Pan yoke hub and laser holder hub |
| 4 | M3×10 | Deck → tray corners |
| 1 | M3×16 + nyloc + 2 washers | Camera hinge |
| 4 | M2×6 | Camera board |
| 4 | M2.5×20 | HAT → spacer → Pi → tray boss |
| 1 | 1/4"-20 hex nut | Tripod mount (optional) |

## Decisions made so far (with reasons)

- **Steppers (28BYJ-48) for the prototype.** Nate already had them. They're light,
  their gear friction holds position with power off (so call `release()` when idle),
  and they don't jitter. Backlash is handled by:
  1. using the **camera-observed** dot position for hit detection, not the commanded one;
  2. approaching every target from the same direction (overshoot, then come back);
  3. letting gravity preload the tilt axis.
- **Servos are likely for the released open-source / embedded design.** They're
  absolute, need no homing, have fewer parts, and on a microcontroller their hardware
  timers remove the Linux PWM jitter problem. Keep motor control behind an
  **`aim(x, y)` abstraction** so the actuators can be swapped.
- **Tracking approach:** track the puck or ball and test whether it's within N px of the
  dot. Don't rely on "dot disappeared" detection, since the dot lands on top of the puck.
- **Calibration:** the camera closes the loop. At startup, sweep until the dot is seen,
  then build a step↔floor (pixel) mapping, e.g. a homography or grid fit. No endstops.
- **Embedded candidates later:** OpenMV Cam (a single board doing vision + PWM) is the
  cleanest fit. ESP32-S3 + OV2640/5640 is cheaper but means more custom vision code.
  A Pi Zero 2 W is a middle ground.

## Suggested next steps (software)

1. Bring-up scripts: jog each stepper with `adafruit-circuitpython-motorkit`
   (`kit.stepper1` / `kit.stepper2`, `INTERLEAVE` or `DOUBLE` style), confirm direction
   and coil order, and toggle the laser via GPIO.
2. Camera capture with OpenCV (`cv2.VideoCapture`, MJPG, 1280×800 or 640×400 at high
   fps). Lock the exposure so the laser and the ball are both detectable.
3. Laser dot detection: threshold and find the brightest small blob, ideally comparing
   laser-on and laser-off frames.
4. Calibration routine: step through a grid of pan/tilt positions, record the dot pixel
   positions, and fit an inverse mapping from pixel to steps.
5. Puck/ball tracker: a bright blob or background-subtraction based tracker that rejects
   the laser.
6. Game loop: pick a random reachable target, `aim()` at it, wait for the puck within
   the radius (with dwell time), score it, play a sound, repeat. Track reaction times.
7. Safety: software tilt limits so the beam never rises above a set angle, a laser
   timeout, and the laser off whenever the motors are released or the program exits.

Suggested layout: `trainer/` package with `motion.py` (aim abstraction + stepper
backend), `vision.py`, `calibrate.py`, `game.py`, and `config.yaml` for pins, limits
and calibration data.

## Files

- `hockey_trainer_pantilt.scad`: parametric source of truth for every printed part
- `stl/`: exported printable parts
- `README.md`: print settings, hardware, assembly order, wiring
- `renders/`: assembly preview PNGs
- Datasheets: Adafruit Motor HAT, 28BYJ-48, OV9281 camera
