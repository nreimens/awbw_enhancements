import math
import random

import numpy as np
import pytest

from trainer.calibrate import Calibration, Sample, collect_samples, fit_calibration, grid, run_calibration
from trainer.config import AxisConfig, Config, load_config
from trainer.game import Game, Targeter
from trainer.laser import FakeLaser
from trainer.motion import Axis, FakeStepper
from trainer.sim import SimCamera, SimWorld
from trainer.vision import PuckTracker, detect_laser_dot, frame_stats


def no_sleep(_):
    pass


# ------------------------------------------------------------------ config

def test_default_config_file_loads_and_matches_defaults():
    cfg = load_config("config.yaml")
    assert cfg.motors.pan.limit_deg == (-45.0, 45.0)
    assert cfg.motors.tilt.limit_deg == (5.0, 45.0)
    assert cfg.motors.tilt.channel == 2
    assert cfg == Config()


def test_partial_override_and_typo_guard(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("motors:\n  tilt:\n    limit_deg: [0, 30]\ngame:\n  hit_radius_px: 12\n")
    cfg = load_config(p)
    assert cfg.motors.tilt.limit_deg == (0, 30)
    assert cfg.motors.tilt.channel == 2          # untouched default survives
    assert cfg.game.hit_radius_px == 12
    p.write_text("laser:\n  gpio_bcn: 4\n")
    with pytest.raises(ValueError):
        load_config(p)


# ------------------------------------------------------------------ motion

def make_axis(**kw):
    cfg = AxisConfig(steps_per_rev=2048, backlash_deg=kw.pop("backlash_deg", 1.5),
                     limit_deg=kw.pop("limit_deg", (-45.0, 45.0)), **kw)
    drv = FakeStepper()
    return Axis(drv, cfg, "t"), drv


def test_axis_steps_and_clamp():
    ax, drv = make_axis()
    assert ax.move_to(90) == pytest.approx(45, abs=0.2)          # clamped to limit
    assert drv.position == ax.pos_steps == round(45 * 2048 / 360)
    ax.move_to(0)
    assert ax.pos_steps == 0


def test_negative_moves_overshoot_then_approach_from_positive_side():
    ax, drv = make_axis()
    ax.move_to(20)
    drv.log.clear()
    ax.move_to(10)
    over = ax.backlash_steps
    assert over > 0
    assert drv.log[0] < 0 and drv.log[1] == over           # go past, come back up
    assert drv.position == ax.pos_steps


def test_invert_flips_driver_direction_not_logical_angle():
    ax, drv = make_axis(invert=True)
    ax.move_to(10)
    assert ax.angle_deg > 0 and drv.position < 0


def test_tilt_never_exceeds_upper_limit_even_while_overshooting():
    ax, drv = make_axis(limit_deg=(5.0, 45.0))
    peak = 0
    for target in (45, 6, 80, 0, 30, 7, 44.9):
        ax.move_to(target)
        peak = max(peak, max(np.cumsum([0] + drv.log)))     # logical == driver here (no invert)
    assert peak <= round(45 * 2048 / 360)


def test_release_is_forwarded():
    ax, drv = make_axis()
    ax.release()
    assert drv.released


# ------------------------------------------------------------------ laser

def test_fake_laser_watchdog_turns_off():
    import time
    l = FakeLaser(max_on_s=0.05)
    l.on()
    assert l.is_on
    time.sleep(0.2)
    assert not l.is_on


# ------------------------------------------------------------------ vision

def blank(h=400, w=640, v=30):
    return np.full((h, w), v, np.uint8)


def test_dot_detection_subpixel_and_requires_diff():
    off = blank()
    on = off.copy()
    yy, xx = np.mgrid[0:400, 0:640]
    on = np.clip(on + 200 * np.exp(-((xx - 123.4) ** 2 + (yy - 87.6) ** 2) / (2 * 1.2 ** 2)), 0, 255).astype(np.uint8)
    det = detect_laser_dot(on, off, Config().vision)
    assert det is not None
    assert abs(det.x - 123.4) < 0.5 and abs(det.y - 87.6) < 0.5
    assert detect_laser_dot(off, off, Config().vision) is None


def test_dot_detection_ignores_static_bright_blob():
    off = blank()
    off[100:140, 100:140] = 250                               # bright puck, present in both
    assert detect_laser_dot(off.copy(), off, Config().vision) is None


def test_puck_tracker_rejects_dot_sized_spot_and_finds_puck():
    f = blank()
    f[200:203, 300:303] = 255                                 # 9 px: dot-sized
    tr = PuckTracker(Config().vision)
    assert tr.update(f) is None
    yy, xx = np.mgrid[0:400, 0:640]
    f[(xx - 400) ** 2 + (yy - 150) ** 2 <= 144] = 240
    d = tr.update(f)
    assert d and abs(d.x - 400) < 1 and abs(d.y - 150) < 1


def test_frame_stats():
    s = frame_stats(blank(v=255))
    assert s["pct_saturated"] == 100


# ------------------------------------------------------------------ calibration (closed loop on the sim)

@pytest.fixture
def world():
    return SimWorld(Config())


def calibrate(world, cfg=None):
    cfg = cfg or Config()
    return run_calibration(world.aim, world.laser, SimCamera(world), cfg, world.width, world.height,
                           sleep=no_sleep, log=lambda s: None)


def test_calibration_recovers_geometry(world):
    cal = calibrate(world)
    assert cal.n_samples >= 25
    assert cal.rms_px < 2.5, cal.rms_px
    # Round-trip against ground truth at off-grid angles.
    for pan, tilt in [(-20.0, 15.0), (5.0, 33.0), (31.0, 20.0)]:
        gx, gy = world.dot_pixel(pan, tilt)
        px, py = cal.angles_to_pixel(pan, tilt)
        assert math.hypot(px - gx, py - gy) < 2.5
        bp, bt = cal.pixel_to_angles(gx, gy)
        assert abs(bp - pan) < 1.0 and abs(bt - tilt) < 1.0
    assert not world.laser.is_on


def test_calibration_save_load_roundtrip(world, tmp_path):
    cal = calibrate(world)
    cal.save(tmp_path / "c.json")
    cal2 = Calibration.load(tmp_path / "c.json")
    assert cal2.pixel_to_angles(300, 150) == pytest.approx(cal.pixel_to_angles(300, 150))


def test_fit_needs_enough_samples():
    with pytest.raises(ValueError):
        fit_calibration([Sample(0, 0, 1, 1)] * 3, 640, 400)


def test_collect_skips_missing_dots_and_leaves_laser_off(world):
    # Laser permanently "off" in the camera's eyes -> no samples, no crash.
    cam = SimCamera(world)
    world.laser.on = lambda brightness=None: None
    s = collect_samples(world.aim, world.laser, cam, Config().vision, grid(-10, 10, 2), grid(5, 10, 2),
                        sleep=no_sleep)
    assert s == [] and not world.laser.is_on


# ------------------------------------------------------------------ game (end to end on the sim)

def test_game_scores_when_puck_reaches_dot_and_times_out_otherwise(world):
    cfg = Config()
    cfg.game.target_timeout_s = 1.0
    cfg.game.dwell_s = 0.0
    cal = calibrate(world)
    clock = {"t": 0.0}

    def tick():
        clock["t"] += 0.01
        return clock["t"]

    mode = {"hit": True}

    state = {"pos": None, "n": 0}

    def puck(w):
        # The puck arrives only after the dot has been located (2 reads after each re-aim),
        # as in real play; before that, and when mode["hit"] is False, it is far away.
        if w.aim.position != state["pos"]:
            state["pos"], state["n"] = w.aim.position, 0
        state["n"] += 1
        if mode["hit"] and state["n"] > 2:
            return w.dot_pixel()
        return (20.0, 380.0)

    world.puck_fn = puck
    game = Game(Targeter(world.aim, cal), world.laser, SimCamera(world), PuckTracker(cfg.vision), cfg,
                clock=tick, sleep=no_sleep, rng=random.Random(1))
    r1 = game.play_target()
    assert r1.hit and r1.reaction_s is not None
    mode["hit"] = False
    r2 = game.play_target()
    assert not r2.hit
    game.laser.off()
    assert len(game.results) == 2


def test_pick_target_stays_in_image(world):
    cal = calibrate(world)
    t = Targeter(world.aim, cal)
    rng = random.Random(3)
    prev = None
    for _ in range(30):
        x, y = t.pick_target(rng, 40, 100, prev)
        assert 40 <= x <= 600 and 40 <= y <= 360
        prev = (x, y)


def test_aim_at_pixel_lands_on_target_in_sim(world):
    cal = calibrate(world)
    t = Targeter(world.aim, cal)
    rng, prev = random.Random(7), None
    for _ in range(25):
        x, y = t.pick_target(rng, 40, 100, prev)
        t.aim_at_pixel(x, y)
        assert math.dist(world.dot_pixel(), (x, y)) < 6.0   # 28BYJ-48 step is ~0.18 deg
        prev = (x, y)
