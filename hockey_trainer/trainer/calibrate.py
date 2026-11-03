"""Closed-loop calibration: map camera pixels <-> (pan, tilt) degrees.

No endstops and no geometry model: sweep a grid of angles, find the laser dot in the
camera at each, then least-squares fit a polynomial in each direction. The camera is the
truth, so mount tolerances, camera hinge angle and lens distortion are absorbed.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .config import Config, VisionConfig, load_config
from .laser import Laser
from .motion import Aim
from .vision import FrameSource, detect_laser_dot

PAN_SCALE = TILT_SCALE = 50.0  # normalise degrees for conditioning


@dataclass
class Sample:
    pan: float
    tilt: float
    x: float
    y: float


def _features(u: np.ndarray, v: np.ndarray, degree: int) -> np.ndarray:
    cols = [u ** i * v ** j for d in range(degree + 1) for i in range(d, -1, -1) for j in [d - i]]
    return np.stack(cols, axis=-1)


def _n_terms(degree: int) -> int:
    return (degree + 1) * (degree + 2) // 2


class Calibration:
    """Bidirectional pixel <-> angle mapping fitted from samples."""

    def __init__(self, degree: int, width: int, height: int,
                 px2ang: np.ndarray, ang2px: np.ndarray, rms_px: float, rms_deg: float, n_samples: int):
        self.degree, self.width, self.height = degree, width, height
        self.px2ang, self.ang2px = np.asarray(px2ang), np.asarray(ang2px)
        self.rms_px, self.rms_deg, self.n_samples = rms_px, rms_deg, n_samples

    # -- normalisation
    def _npx(self, x, y):
        s = max(self.width, self.height) / 2.0
        return (np.asarray(x, float) - self.width / 2.0) / s, (np.asarray(y, float) - self.height / 2.0) / s

    def _px_from_n(self, u, v):
        s = max(self.width, self.height) / 2.0
        return u * s + self.width / 2.0, v * s + self.height / 2.0

    # -- mapping
    def pixel_to_angles(self, x: float, y: float, iterations: int = 8) -> Tuple[float, float]:
        """Angles that put the dot at pixel (x, y).

        The direct pixel->angle polynomial is a poor fit near the nadir (pan is
        unobservable where the dot barely moves), so it only seeds a Gauss-Newton
        refinement on the smooth, well-conditioned angles->pixel map.
        """
        u, v = self._npx(x, y)
        seed = _features(np.atleast_1d(u), np.atleast_1d(v), self.degree) @ self.px2ang
        a = np.array([seed[0, 0] * PAN_SCALE, seed[0, 1] * TILT_SCALE])
        target = np.array([float(x), float(y)])
        eps = 0.05
        for _ in range(iterations):
            f0 = np.array(self.angles_to_pixel(*a))
            err = target - f0
            if np.hypot(*err) < 0.05:
                break
            jac = np.empty((2, 2))
            for k in range(2):
                da = a.copy()
                da[k] += eps
                jac[:, k] = (np.array(self.angles_to_pixel(*da)) - f0) / eps
            try:
                step = np.linalg.solve(jac, err)
            except np.linalg.LinAlgError:
                break
            a = a + np.clip(step, -10.0, 10.0)
        return float(a[0]), float(a[1])

    def angles_to_pixel(self, pan: float, tilt: float) -> Tuple[float, float]:
        a, b = np.atleast_1d(pan / PAN_SCALE), np.atleast_1d(tilt / TILT_SCALE)
        out = _features(a, b, self.degree) @ self.ang2px
        x, y = self._px_from_n(out[0, 0], out[0, 1])
        return float(x), float(y)

    # -- persistence
    def to_dict(self) -> dict:
        return {"degree": self.degree, "width": self.width, "height": self.height,
                "px2ang": self.px2ang.tolist(), "ang2px": self.ang2px.tolist(),
                "rms_px": self.rms_px, "rms_deg": self.rms_deg, "n_samples": self.n_samples}

    @classmethod
    def from_dict(cls, d: dict) -> "Calibration":
        return cls(d["degree"], d["width"], d["height"], np.array(d["px2ang"]), np.array(d["ang2px"]),
                   d["rms_px"], d["rms_deg"], d["n_samples"])

    def save(self, path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path) -> "Calibration":
        return cls.from_dict(json.loads(Path(path).read_text()))


def fit_calibration(samples: Sequence[Sample], width: int, height: int,
                    degree: Optional[int] = None) -> Calibration:
    """Fit both mappings. Degree auto-selects from sample count (>= 2x the terms needed)."""
    n = len(samples)
    if degree is None:
        degree = next((d for d in (3, 2, 1) if n >= 2 * _n_terms(d)), None)
        if degree is None:
            raise ValueError(f"need at least {2 * _n_terms(1)} samples, got {n}")
    elif n < _n_terms(degree):
        raise ValueError(f"degree {degree} needs >= {_n_terms(degree)} samples, got {n}")

    pan = np.array([s.pan for s in samples]) / PAN_SCALE
    tilt = np.array([s.tilt for s in samples]) / TILT_SCALE
    scale = max(width, height) / 2.0
    u = (np.array([s.x for s in samples]) - width / 2.0) / scale
    v = (np.array([s.y for s in samples]) - height / 2.0) / scale

    px2ang, *_ = np.linalg.lstsq(_features(u, v, degree), np.stack([pan, tilt], 1), rcond=None)
    ang2px, *_ = np.linalg.lstsq(_features(pan, tilt, degree), np.stack([u, v], 1), rcond=None)

    pred_uv = _features(pan, tilt, degree) @ ang2px
    rms_px = float(np.sqrt(np.mean(np.sum(((pred_uv - np.stack([u, v], 1)) * scale) ** 2, axis=1))))
    pred_ang = _features(u, v, degree) @ px2ang
    rms_deg = float(np.sqrt(np.mean(np.sum(((pred_ang - np.stack([pan, tilt], 1)) * PAN_SCALE) ** 2, axis=1))))
    return Calibration(degree, width, height, px2ang, ang2px, rms_px, rms_deg, n)


def locate_dot(laser: Laser, cam: FrameSource, vcfg: VisionConfig, flush: Optional[int] = None,
               sleep: Callable[[float], None] = time.sleep, light_s: float = 0.05):
    """Blink the laser and return the dot Detection (or None)."""
    laser.off()
    sleep(light_s)
    off = cam.read_fresh(flush)
    laser.on()
    sleep(light_s)
    on = cam.read_fresh(flush)
    if on is None or off is None:
        return None
    return detect_laser_dot(on, off, vcfg)


def grid(lo: float, hi: float, n: int) -> List[float]:
    return [float(v) for v in np.linspace(lo, hi, max(1, n))]


def collect_samples(aim: Aim, laser: Laser, cam: FrameSource, vcfg: VisionConfig,
                    pan_values: Iterable[float], tilt_values: Iterable[float],
                    settle_s: float = 0.4, sleep: Callable[[float], None] = time.sleep,
                    log: Callable[[str], None] = lambda s: None) -> List[Sample]:
    """Visit the grid, return samples where the dot was seen. Laser is always left off."""
    samples: List[Sample] = []
    pans, tilts = list(pan_values), list(tilt_values)
    try:
        for ti, tilt in enumerate(tilts):
            row = pans if ti % 2 == 0 else pans[::-1]  # serpentine = short moves
            for pan in row:
                p, t = aim.aim(pan, tilt)
                sleep(settle_s)
                det = locate_dot(laser, cam, vcfg, sleep=sleep)
                laser.off()
                if det is None:
                    log(f"pan {p:6.1f} tilt {t:5.1f}: dot not seen")
                    continue
                log(f"pan {p:6.1f} tilt {t:5.1f}: dot at ({det.x:.1f}, {det.y:.1f})")
                samples.append(Sample(p, t, det.x, det.y))
    finally:
        laser.off()
    return samples


def run_calibration(aim: Aim, laser: Laser, cam: FrameSource, cfg: Config, width: int, height: int,
                    sleep: Callable[[float], None] = time.sleep,
                    log: Callable[[str], None] = print) -> Calibration:
    (pan_lo, pan_hi), (tilt_lo, tilt_hi) = aim.limits
    samples = collect_samples(aim, laser, cam, cfg.vision,
                              grid(pan_lo, pan_hi, cfg.calibration.pan_points),
                              grid(tilt_lo, tilt_hi, cfg.calibration.tilt_points),
                              cfg.calibration.settle_s, sleep, log)
    cal = fit_calibration(samples, width, height)
    log(f"fit degree {cal.degree} from {cal.n_samples} samples: "
        f"rms {cal.rms_px:.2f} px / {cal.rms_deg:.3f} deg")
    return cal


def main(argv: Optional[Sequence[str]] = None) -> int:  # pragma: no cover - needs hardware
    ap = argparse.ArgumentParser(description="Run closed-loop pan/tilt <-> pixel calibration.")
    ap.add_argument("--config", default=None)
    ap.add_argument("--yes-laser", action="store_true", help="skip the laser safety confirmation")
    args = ap.parse_args(argv)
    from .rig import open_rig, confirm_laser
    cfg = load_config(args.config)
    if not args.yes_laser and not confirm_laser():
        return 1
    with open_rig(cfg) as rig:
        a = rig.camera.actual()
        cal = run_calibration(rig.aim, rig.laser, rig.camera, cfg, a["width"], a["height"])
        cal.save(cfg.calibration.file)
        print(f"saved {cfg.calibration.file}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
