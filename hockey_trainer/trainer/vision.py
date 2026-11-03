"""Camera access and detection for the monochrome OV9281.

Detection is brightness based (no color): the laser dot is found by differencing a
laser-on frame against a laser-off frame; the puck/ball is a bright blob (or a
background-subtraction foreground blob) that is larger than the dot.
"""
from __future__ import annotations

import glob
import os
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Protocol, Tuple

import cv2
import numpy as np

from .config import CameraConfig, VisionConfig


@dataclass
class Detection:
    x: float
    y: float
    area: float
    peak: float = 0.0


class FrameSource(Protocol):
    def read(self) -> Optional[np.ndarray]: ...
    def read_fresh(self, flush: Optional[int] = None) -> Optional[np.ndarray]: ...
    def close(self) -> None: ...


def to_gray(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame


# ---------------------------------------------------------------- camera discovery

def list_video_devices() -> List[Tuple[str, str]]:
    """[(device path, V4L2 name)] from sysfs, sorted by index."""
    out = []
    for sys_dir in sorted(glob.glob("/sys/class/video4linux/video*"),
                          key=lambda p: int(p.rsplit("video", 1)[1])):
        try:
            with open(os.path.join(sys_dir, "name")) as fh:
                name = fh.read().strip()
        except OSError:
            name = "?"
        out.append(("/dev/" + os.path.basename(sys_dir), name))
    return out


def find_camera_device(name_hint: str = "") -> Optional[str]:
    """First video node whose name contains `name_hint` (case-insensitive) and can capture.

    A UVC camera usually registers two nodes (capture + metadata); we return the first
    node that actually delivers a frame. With no match on the hint, returns None.
    """
    for path, name in list_video_devices():
        if name_hint and name_hint.lower() not in name.lower():
            continue
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        try:
            if cap.isOpened():
                ok, _ = cap.read()
                if ok:
                    return path
        finally:
            cap.release()
    return None


class Camera:
    """cv2.VideoCapture wrapper: MJPG, fixed size/fps, manual exposure."""

    def __init__(self, cfg: CameraConfig):
        self.cfg = cfg
        dev = cfg.device
        if dev == "auto":
            dev = find_camera_device(cfg.name_hint)
            if dev is None:
                raise RuntimeError(f"no camera matching {cfg.name_hint!r} found "
                                   f"(devices: {list_video_devices()})")
        elif isinstance(dev, str) and dev.isdigit():
            dev = int(dev)
        self.device = dev
        self._cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera {dev}")
        if cfg.fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cfg.fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        self._cap.set(cv2.CAP_PROP_FPS, cfg.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cfg.exposure is not None:
            # V4L2: 1 = manual mode, 3 = aperture priority (auto)
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self._cap.set(cv2.CAP_PROP_EXPOSURE, cfg.exposure)

    def actual(self) -> dict:
        c = self._cap
        code = int(c.get(cv2.CAP_PROP_FOURCC))
        return {
            "device": self.device,
            "width": int(c.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(c.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": c.get(cv2.CAP_PROP_FPS),
            "fourcc": "".join(chr((code >> 8 * i) & 0xFF) for i in range(4)),
            "exposure": c.get(cv2.CAP_PROP_EXPOSURE),
        }

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._cap.read()
        return to_gray(frame) if ok else None

    def read_fresh(self, flush: Optional[int] = None) -> Optional[np.ndarray]:
        """Drop `flush` buffered (stale) frames, then return the next one."""
        for _ in range(self.cfg.flush_frames if flush is None else flush):
            self._cap.grab()
        return self.read()

    def close(self) -> None:
        self._cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ---------------------------------------------------------------- detection

def detect_laser_dot(on: np.ndarray, off: np.ndarray, cfg: VisionConfig) -> Optional[Detection]:
    """Locate the laser dot as the brightest compact blob in (on - off)."""
    diff = cv2.subtract(to_gray(on), to_gray(off))  # saturating: noise below zero -> 0
    diff = cv2.GaussianBlur(diff, (3, 3), 0)
    _, mask = cv2.threshold(diff, cfg.dot_diff_threshold, 255, cv2.THRESH_BINARY)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best, best_score = None, 0.0
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if not (cfg.dot_min_area <= area <= cfg.dot_max_area):
            continue
        ys, xs = np.nonzero(labels == i)
        w = diff[ys, xs].astype(np.float64)
        score = float(w.sum())
        if score > best_score:
            best_score = score
            best = Detection(float((xs * w).sum() / w.sum()), float((ys * w).sum() / w.sum()),
                             float(area), float(w.max()))
    return best


def _largest_blob(mask: np.ndarray, min_area: float, max_area: float) -> Optional[Detection]:
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = None
    for i in range(1, n):
        area = float(stats[i, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area and (best is None or area > best.area):
            best = Detection(float(cents[i][0]), float(cents[i][1]), area)
    return best


class PuckTracker:
    """Finds the puck/ball as the largest in-range bright (or moving) blob.

    The laser dot is rejected by size (puck_min_area > dot_max_area), not by position:
    the dot lands on top of the puck during a hit, so a position mask would hide it.
    """

    def __init__(self, cfg: VisionConfig):
        self.cfg = cfg
        self._bg = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=25,
                                                      detectShadows=False) if cfg.puck_mode == "bgsub" else None

    def update(self, frame: np.ndarray) -> Optional[Detection]:
        gray = to_gray(frame)
        if self._bg is not None:
            mask = self._bg.apply(gray)
        else:
            _, mask = cv2.threshold(gray, self.cfg.puck_threshold, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return _largest_blob(mask, self.cfg.puck_min_area, self.cfg.puck_max_area)


def frame_stats(gray: np.ndarray) -> dict:
    """Quick exposure diagnostics: mean/min/max and % saturated / near-black pixels."""
    g = to_gray(gray)
    return {
        "mean": float(g.mean()),
        "min": int(g.min()),
        "max": int(g.max()),
        "pct_saturated": float((g >= 250).mean() * 100),
        "pct_black": float((g <= 5).mean() * 100),
    }


def measure_fps(src: FrameSource, n: int = 60, clock: Callable[[], float] = time.perf_counter) -> float:
    src.read()
    t0 = clock()
    got = 0
    for _ in range(n):
        if src.read() is not None:
            got += 1
    dt = clock() - t0
    return got / dt if dt > 0 else 0.0
