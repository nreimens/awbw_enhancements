"""Hardware bring-up checks to run ON THE PI.

    python -m trainer.hwcheck scan            # find I2C HAT, cameras, libraries (safe, no motion)
    python -m trainer.hwcheck camera          # open camera, report fps/exposure, save frames
    python -m trainer.hwcheck stepper pan     # jog an axis +/- and confirm direction
    python -m trainer.hwcheck laser           # blink laser (asks for confirmation)
    python -m trainer.hwcheck dot             # laser on/off differencing, prints dot pixel + jitter
    python -m trainer.hwcheck all             # everything, in a safe order, with a summary

Add --fake to run the same checks against the simulator (no hardware needed).
Exit code is non-zero if any check FAILs.
"""
from __future__ import annotations

import argparse
import importlib
import os
import platform
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Sequence

from .config import Config, load_config

PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"
HAT_ADDR = 0x60


@dataclass
class Result:
    name: str
    status: str
    detail: str = ""


class Reporter:
    def __init__(self, out: Callable[[str], None] = print):
        self.results: List[Result] = []
        self.out = out

    def add(self, name: str, status: str, detail: str = "") -> Result:
        r = Result(name, status, detail)
        self.results.append(r)
        self.out(f"[{status:4}] {name}" + (f": {detail}" if detail else ""))
        return r

    def summary(self) -> int:
        self.out("\n=== summary ===")
        for r in self.results:
            self.out(f"{r.status:4}  {r.name}")
        failed = [r for r in self.results if r.status == FAIL]
        self.out(f"{len(failed)} failed, {sum(r.status == WARN for r in self.results)} warnings")
        return 1 if failed else 0


# ---------------------------------------------------------------- scan

def i2c_scan(bus: int = 1) -> List[int]:
    """Addresses that ACK. Uses smbus2 if present, else `i2cdetect -y`."""
    try:
        from smbus2 import SMBus
    except ImportError:
        out = subprocess.run(["i2cdetect", "-y", str(bus)], capture_output=True, text=True, check=True).stdout
        found = []
        for line in out.splitlines()[1:]:
            row, _, cells = line.partition(":")
            for i, c in enumerate(cells.split()):
                if c not in ("--", "UU"):
                    found.append(int(row, 16) + i)
        return found
    found = []
    with SMBus(bus) as b:
        for addr in range(0x03, 0x78):
            try:
                b.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
    return found


def check_scan(rep: Reporter, cfg: Config) -> None:
    model = "unknown"
    try:
        model = Path("/proc/device-tree/model").read_text().strip("\x00\n ")
    except OSError:
        pass
    rep.add("platform", PASS, f"{platform.platform()} | {model} | python {platform.python_version()}")
    if "Raspberry Pi" not in model:
        rep.add("running on a Pi", WARN, "device-tree model is not a Raspberry Pi")

    for mod, needed in (("numpy", True), ("cv2", True), ("yaml", True),
                        ("adafruit_motorkit", True), ("gpiozero", True), ("smbus2", False)):
        try:
            m = importlib.import_module(mod)
            rep.add(f"import {mod}", PASS, str(getattr(m, "__version__", "")))
        except ImportError as exc:
            rep.add(f"import {mod}", FAIL if needed else WARN, str(exc))

    if not Path("/dev/i2c-1").exists():
        rep.add("I2C bus /dev/i2c-1", FAIL, "missing: enable I2C (sudo raspi-config nonint do_i2c 0) and reboot")
    else:
        try:
            found = i2c_scan()
            rep.add("I2C devices", PASS if found else WARN, ", ".join(f"0x{a:02x}" for a in found) or "none")
            if HAT_ADDR in found:
                rep.add(f"Motor HAT (PCA9685) at 0x{HAT_ADDR:02x}", PASS)
            else:
                rep.add(f"Motor HAT (PCA9685) at 0x{HAT_ADDR:02x}", FAIL,
                        "not found: check HAT seating, 5 V motor supply is not required for detection")
        except Exception as exc:
            rep.add("I2C scan", FAIL, f"{exc} (install i2c-tools or smbus2; user needs i2c group)")

    check_video_nodes(rep, cfg)


def check_video_nodes(rep: Reporter, cfg: Config) -> None:
    try:
        import cv2
        from .vision import list_video_devices
    except ImportError as exc:
        rep.add("video devices", SKIP, f"{exc}")
        return
    devs = list_video_devices()
    if not devs:
        rep.add("video devices", FAIL, "no /dev/video* nodes: is the OV9281 plugged into a USB port?")
        return
    hit = None
    for path, name in devs:
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        ok = cap.isOpened() and cap.read()[0]
        cap.release()
        match = cfg.camera.name_hint.lower() in name.lower()
        rep.add(f"{path} '{name}'", PASS if ok else WARN,
                ("captures frames" if ok else "no frames (likely a metadata node)") + (" <- matches hint" if match else ""))
        if ok and match and hit is None:
            hit = path
    if hit:
        rep.add("camera located", PASS, f"{hit} (set camera.device in config.yaml to pin it)")
    else:
        rep.add("camera located", FAIL, f"no capture node matches name_hint {cfg.camera.name_hint!r}")


# ---------------------------------------------------------------- camera / dot

def check_camera(rep: Reporter, rig, out_dir: Path, frames: int = 5) -> None:
    import cv2
    from .vision import frame_stats, measure_fps
    out_dir.mkdir(parents=True, exist_ok=True)
    cam = rig.camera
    a = cam.actual()
    rep.add("camera settings", PASS, f"{a['device']} {a['width']}x{a['height']} {a['fourcc']} "
                                    f"reported {a['fps']:.0f} fps exposure {a['exposure']}")
    first = cam.read_fresh()
    if first is None:
        rep.add("camera frame", FAIL, "read() returned nothing")
        return
    st = frame_stats(first)
    rep.add("camera frame", PASS, f"mean {st['mean']:.0f} max {st['max']} "
                                 f"saturated {st['pct_saturated']:.2f}% black {st['pct_black']:.1f}%")
    if st["pct_saturated"] > 5:
        rep.add("exposure", WARN, "over 5% saturated: lower camera.exposure")
    elif st["max"] < 100:
        rep.add("exposure", WARN, "dark image: raise camera.exposure or add light")
    for i in range(frames):
        cv2.imwrite(str(out_dir / f"camera_{i}.png"), cam.read())
    fps = measure_fps(cam, 60)
    rep.add("measured fps", PASS if fps >= 30 else WARN, f"{fps:.0f} fps (USB bandwidth / exposure may cap it)")
    rep.add("saved frames", PASS, str(out_dir / "camera_*.png"))


def check_dot(rep: Reporter, rig, cfg: Config, out_dir: Path, trials: int = 10) -> None:
    import cv2
    from .calibrate import locate_dot
    out_dir.mkdir(parents=True, exist_ok=True)
    xs, ys, misses = [], [], 0
    last_on = None
    try:
        for _ in range(trials):
            rig.laser.off()
            time.sleep(0.05)
            off = rig.camera.read_fresh()
            rig.laser.on()
            time.sleep(0.05)
            on = rig.camera.read_fresh()
            last_on = on
            from .vision import detect_laser_dot
            det = detect_laser_dot(on, off, cfg.vision)
            if det is None:
                misses += 1
            else:
                xs.append(det.x)
                ys.append(det.y)
    finally:
        rig.laser.off()
    if not xs:
        rep.add("laser dot detection", FAIL,
                f"0/{trials} detections: check aim, exposure (dot must not be washed out), "
                f"vision.dot_diff_threshold={cfg.vision.dot_diff_threshold}")
        return
    sx = statistics.pstdev(xs) if len(xs) > 1 else 0.0
    sy = statistics.pstdev(ys) if len(ys) > 1 else 0.0
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    ok = misses == 0 and max(sx, sy) < 1.5
    rep.add("laser dot detection", PASS if ok else WARN,
            f"{len(xs)}/{trials} seen at ({mx:.1f}, {my:.1f}) px, jitter sd ({sx:.2f}, {sy:.2f}) px")
    if last_on is not None:
        vis = cv2.cvtColor(last_on, cv2.COLOR_GRAY2BGR)
        cv2.circle(vis, (int(mx), int(my)), 12, (0, 255, 0), 1)
        cv2.imwrite(str(out_dir / "dot.png"), vis)
        rep.add("saved annotated frame", PASS, str(out_dir / "dot.png"))


# ---------------------------------------------------------------- stepper / laser

def check_stepper(rep: Reporter, rig, axis: str, deg: float, ask: Callable[[str], str], assume_yes: bool) -> None:
    ax = rig.aim.pan_axis if axis == "pan" else rig.aim.tilt_axis
    lo, hi = ax.lo, ax.hi
    amp = min(abs(deg), hi if hi > 0 else abs(lo))
    if amp <= 0:
        rep.add(f"{axis} jog", FAIL, "limits leave no room to move in + direction from zero")
        return
    start = time.perf_counter()
    try:
        # Order matters: up first (+), then back to 0 (takes up backlash), so the beam never
        # goes beyond the configured + limit and tilt never rises past tilt.limit_deg[1].
        for target in (amp, 0.0):
            ax.move_to(target)
            time.sleep(0.3)
        elapsed = time.perf_counter() - start
        steps = ax.pos_steps
        rep.add(f"{axis} jog +{amp:g} deg then back", PASS,
                f"{elapsed:.1f}s, final step count {steps}, steps/deg {ax.steps_per_deg:.2f}")
        if not assume_yes:
            ans = ask(f"Did the {axis} axis move smoothly, in the + direction first "
                      f"({'counter-clockwise from above' if axis == 'pan' else 'beam rising away from straight down'})? "
                      "[y]es / [n]o (wrong direction) / [s]tuck-or-jitter: ").strip().lower()
            if ans.startswith("y"):
                rep.add(f"{axis} direction/motion", PASS)
            elif ans.startswith("n"):
                rep.add(f"{axis} direction/motion", FAIL, f"set motors.{axis}.invert: true in config.yaml")
            else:
                rep.add(f"{axis} direction/motion", FAIL,
                        "check coil wiring (red center tap to GND, coil order), 5 V supply, step_delay_s")
    finally:
        ax.release()


def check_laser(rep: Reporter, rig, seconds: float, ask: Callable[[str], str], assume_yes: bool) -> None:
    try:
        for _ in range(3):
            rig.laser.on()
            time.sleep(seconds)
            rig.laser.off()
            time.sleep(seconds)
        rep.add("laser blink x3", PASS, f"{seconds:g}s on/off")
        if not assume_yes:
            ans = ask("Did you see the laser blink three times? [y/n]: ").strip().lower()
            rep.add("laser visual confirm", PASS if ans.startswith("y") else FAIL,
                    "" if ans.startswith("y") else "check GPIO pin (laser.gpio_bcm), transistor wiring, 5 V")
    finally:
        rig.laser.off()


# ---------------------------------------------------------------- CLI

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="trainer.hwcheck", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="path to config.yaml (default ./config.yaml)")
    ap.add_argument("--fake", action="store_true", help="run against the simulator instead of hardware")
    ap.add_argument("--out", default="hwcheck_out", help="directory for saved frames")
    ap.add_argument("--yes", action="store_true",
                    help="skip motion/visual prompts (the laser confirmation is skipped only with --yes-laser)")
    ap.add_argument("--yes-laser", action="store_true", help="skip the laser safety confirmation")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    sub.add_parser("camera")
    p = sub.add_parser("stepper")
    p.add_argument("axis", choices=["pan", "tilt"])
    p.add_argument("--deg", type=float, default=10.0)
    p = sub.add_parser("laser")
    p.add_argument("--seconds", type=float, default=0.5)
    sub.add_parser("dot")
    sub.add_parser("all")
    return ap


def run(args, ask: Callable[[str], str] = input, out: Callable[[str], None] = print) -> int:
    cfg = load_config(args.config)
    rep = Reporter(out)
    out_dir = Path(args.out)
    laser_yes = args.yes_laser
    if args.cmd == "scan":
        if args.fake:
            rep.add("scan", SKIP, "nothing to scan in --fake mode")
        else:
            check_scan(rep, cfg)
        return rep.summary()

    from .rig import confirm_laser, open_rig
    with open_rig(cfg, fake=args.fake) as rig:
        if args.cmd in ("camera", "all"):
            if args.cmd == "all" and not args.fake:
                check_scan(rep, cfg)
            check_camera(rep, rig, out_dir)
        if args.cmd in ("stepper", "all"):
            axes = [args.axis] if args.cmd == "stepper" else ["pan", "tilt"]
            deg = getattr(args, "deg", 10.0)
            for ax in axes:
                check_stepper(rep, rig, ax, deg, ask, args.yes)
        if args.cmd in ("laser", "dot", "all"):
            if laser_yes or confirm_laser(ask):
                if args.cmd in ("laser", "all"):
                    check_laser(rep, rig, getattr(args, "seconds", 0.5), ask, args.yes)
                if args.cmd in ("dot", "all"):
                    check_dot(rep, rig, cfg, out_dir)
            else:
                rep.add("laser/dot checks", SKIP, "laser not confirmed")
    return rep.summary()


def main(argv: Optional[Sequence[str]] = None) -> int:
    return run(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
