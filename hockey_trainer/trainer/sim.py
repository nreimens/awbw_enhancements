"""Software-in-the-loop simulator: fake aim + fake laser + synthetic camera.

Geometry: laser at height `h` above the floor, camera straight down at height `cam_h`.
Floor point for (pan, tilt) = h*tan(tilt)*(sin(pan), cos(pan)); pixels are linear in floor
position. Real rigs are messier (camera hinge, lens), which the polynomial fit absorbs.
"""
from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import numpy as np

from .config import Config
from .laser import FakeLaser
from .motion import StepperAim, make_stepper_aim


class SimWorld:
    def __init__(self, cfg: Config, width: int = 640, height: int = 400, laser_h: float = 0.11,
                 cam_h: float = 1.0, focal_px: float = 500.0, noise: float = 2.0, seed: int = 0):
        self.cfg, self.width, self.height = cfg, width, height
        self.laser_h, self.cam_h, self.f, self.noise = laser_h, cam_h, focal_px, noise
        self.rng = np.random.default_rng(seed)
        self.aim: StepperAim = make_stepper_aim(cfg.motors, fake=True)
        # laser stands in for the GPIO output; keep it permissive for tests
        self.laser = FakeLaser()
        self.puck_px: Optional[Tuple[float, float]] = None
        self.puck_fn: Optional[Callable[["SimWorld"], Optional[Tuple[float, float]]]] = None
        self.reads = 0

    # ground-truth geometry, scaled so the +-limits land inside the image
    def dot_pixel(self, pan_deg: Optional[float] = None, tilt_deg: Optional[float] = None) -> Tuple[float, float]:
        if pan_deg is None:
            pan_deg, tilt_deg = self.aim.position
        r = self.laser_h * math.tan(math.radians(tilt_deg)) * 3.0  # 3x: floor is bigger than the head
        x = self.f * r * math.sin(math.radians(pan_deg)) / self.cam_h * 1.6
        y = self.f * r * math.cos(math.radians(pan_deg)) / self.cam_h * 1.6
        return self.width / 2.0 + x, self.height * 0.15 + y

    def render(self) -> np.ndarray:
        img = np.clip(self.rng.normal(30, self.noise, (self.height, self.width)), 0, 255)
        yy, xx = np.mgrid[0:self.height, 0:self.width]
        if self.puck_fn is not None:
            self.puck_px = self.puck_fn(self)
        if self.puck_px is not None:
            px, py = self.puck_px
            img[(xx - px) ** 2 + (yy - py) ** 2 <= 12 ** 2] = 230  # ~450 px^2 disc
        if self.laser.is_on:
            dx, dy = self.dot_pixel()
            img += 220 * np.exp(-((xx - dx) ** 2 + (yy - dy) ** 2) / (2 * 1.2 ** 2))
        return np.clip(img, 0, 255).astype(np.uint8)


class SimCamera:
    def __init__(self, world: SimWorld):
        self.world = world

    def read(self) -> np.ndarray:
        self.world.reads += 1
        return self.world.render()

    def read_fresh(self, flush: Optional[int] = None) -> np.ndarray:
        return self.read()

    def actual(self) -> dict:
        return {"device": "sim", "width": self.world.width, "height": self.world.height,
                "fps": 0.0, "fourcc": "SIM", "exposure": 0.0}

    def close(self) -> None:
        pass
