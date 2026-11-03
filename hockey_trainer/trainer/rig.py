"""Assemble aim + laser + camera and guarantee a safe shutdown."""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Iterator

from .config import Config
from .laser import Laser
from .motion import Aim


@dataclass
class Rig:
    aim: Aim
    laser: Laser
    camera: Any

    def shutdown(self) -> None:
        """Laser first, then de-energize motors, then the camera."""
        for step in (self.laser.close, self.aim.release, self.camera.close):
            try:
                step()
            except Exception:
                pass


def confirm_laser(input_fn=input) -> bool:
    print("LASER SAFETY: Class 3R diode (650 nm, 5 mW). The beam must point at the floor, away from\n"
          "anyone's eyes, with no reflective surfaces in its path. Tilt is clamped in software but\n"
          "verify the head pose before powering up (pointing straight down = zero).")
    return input_fn("Type YES to enable the laser: ").strip() == "YES"


@contextmanager
def open_rig(cfg: Config, fake: bool = False) -> Iterator[Rig]:
    if fake:
        from .sim import SimCamera, SimWorld
        world = SimWorld(cfg)
        rig = Rig(world.aim, world.laser, SimCamera(world))
        rig.world = world  # type: ignore[attr-defined]
    else:  # pragma: no cover - needs Pi hardware
        from .laser import GpioLaser
        from .motion import make_stepper_aim
        from .vision import Camera
        laser = GpioLaser(cfg.laser)
        try:
            aim = make_stepper_aim(cfg.motors)
            cam = Camera(cfg.camera)
        except Exception:
            laser.close()
            raise
        rig = Rig(aim, laser, cam)
    try:
        yield rig
    finally:
        rig.shutdown()
