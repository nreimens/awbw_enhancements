# Laser Hockey Trainer: software

Pan/tilt laser + camera puck tracking. Project context and decisions are in `CLAUDE.md`.

```
trainer/
  config.py     typed config.yaml loader (unknown keys are errors)
  motion.py     Aim interface; StepperAim = 2x Axis (limits, backlash take-up) over MotorKit or fake drivers
  laser.py      GpioLaser / FakeLaser: off by default, watchdog timeout, off at exit
  vision.py     Camera (V4L2, MJPG, manual exposure, auto-find by name), dot + puck detection
  calibrate.py  grid sweep -> polynomial pixel<->(pan,tilt) fit, saved as calibration.json
  game.py       Targeter.aim_at_pixel(x, y) + Game loop (camera-observed dot, dwell, reaction time)
  sim.py        synthetic world so everything above runs in CI with no hardware
  hwcheck.py    bring-up checks to run on the Pi
tests/          pytest, runs on any machine
```

## On the Pi

```bash
sudo raspi-config nonint do_i2c 0 && sudo reboot          # enable I2C once
sudo apt install -y i2c-tools python3-opencv python3-venv
git clone <repo> && cd awbw_enhancements/hockey_trainer
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -e '.[pi]'
```

### Bring-up order (each step is safe to run on its own)

```bash
python -m trainer.hwcheck scan        # no motion: libs, I2C 0x60 HAT, finds the OV9281 node
python -m trainer.hwcheck camera      # fps, exposure stats, saves hwcheck_out/camera_*.png
python -m trainer.hwcheck stepper pan # jogs +10 deg and back; asks if direction was right
python -m trainer.hwcheck stepper tilt
python -m trainer.hwcheck laser       # asks you to type YES first
python -m trainer.hwcheck dot         # laser on/off differencing: dot pixel + jitter
python -m trainer.hwcheck all         # all of the above
```

Output is `PASS/WARN/FAIL/SKIP` lines plus a summary, and a non-zero exit code on any FAIL; paste it
back into the session. Failures print the config key to change (e.g. `motors.pan.invert`).

Add `--fake` to run the identical flow against the simulator.

### Power-up pose

There are no endstops. **Start every session with the head pointing straight down and centered**:
that pose is step-count zero. Tilt is limited to `[5, 45]` degrees from vertical by default
(`motors.tilt.limit_deg`); the lower bound is above 0 because pan cannot be observed at the exact
nadir. Moves finish in the + direction, so backlash overshoot goes toward the floor, never above the
upper tilt limit.

### Then

```bash
python -m trainer.calibrate           # writes calibration.json (laser confirmation prompt)
python -m trainer.game --targets 10
```

## Dev

```bash
pip install -e '.[vision,dev]' && pytest
```
