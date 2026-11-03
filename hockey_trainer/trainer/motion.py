"""Motion layer.

`Aim` is the actuator abstraction (angles in degrees). Steppers implement it now; a
servo backend can implement the same interface later without touching vision/game code.

Conventions
-----------
pan  : 0 = centered, + = positive logical direction.
tilt : 0 = beam straight down, + = beam rising away from vertical.
Every move ends in the + direction (overshoot below, then come back up) so gear
backlash is always taken up the same way. For tilt this also means any overshoot is
toward the floor, never above the commanded/clamped angle.
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Callable, List, Optional, Protocol, Tuple

from .config import AxisConfig, MotorConfig


class StepperDriver(Protocol):
    def step(self, n: int) -> None:
        """Move |n| full steps; n > 0 forward, n < 0 backward."""

    def release(self) -> None:
        """De-energize the coils."""


class FakeStepper:
    """In-memory driver. `position` is net steps; `log` keeps every call."""

    def __init__(self) -> None:
        self.position = 0
        self.log: List[int] = []
        self.released = False

    def step(self, n: int) -> None:
        self.released = False
        self.position += n
        self.log.append(n)

    def release(self) -> None:
        self.released = True


_kit_cache = {}


def get_motorkit(address: int = 0x60):
    """Shared MotorKit instance (both steppers live on one HAT)."""
    if address not in _kit_cache:
        try:
            from adafruit_motorkit import MotorKit
        except ImportError as exc:  # pragma: no cover - needs Pi libs
            raise RuntimeError(
                "adafruit-circuitpython-motorkit not installed (pip install adafruit-circuitpython-motorkit)"
            ) from exc
        _kit_cache[address] = MotorKit(address=address)
    return _kit_cache[address]


class MotorKitStepper:
    """Adafruit Motor HAT stepper channel (1 = M1/M2, 2 = M3/M4)."""

    def __init__(self, channel: int, style: str = "DOUBLE", delay_s: float = 0.004,
                 kit=None, sleep: Callable[[float], None] = time.sleep):
        if channel not in (1, 2):
            raise ValueError("stepper channel must be 1 or 2")
        from adafruit_motor import stepper as _st  # pragma: no cover - needs Pi libs
        kit = kit or get_motorkit()
        self._stepper = kit.stepper1 if channel == 1 else kit.stepper2
        self._fwd, self._back = _st.FORWARD, _st.BACKWARD
        self._style = getattr(_st, style.upper())
        self.delay_s = delay_s
        self._sleep = sleep

    def step(self, n: int) -> None:  # pragma: no cover - needs Pi hardware
        direction = self._fwd if n > 0 else self._back
        for _ in range(abs(n)):
            self._stepper.onestep(direction=direction, style=self._style)
            self._sleep(self.delay_s)

    def release(self) -> None:  # pragma: no cover - needs Pi hardware
        self._stepper.release()


class Axis:
    """One stepper axis in degrees, with limits and backlash take-up."""

    def __init__(self, driver: StepperDriver, cfg: AxisConfig, name: str = "axis"):
        self.driver = driver
        self.name = name
        self.steps_per_deg = cfg.steps_per_rev / 360.0
        self.lo, self.hi = float(cfg.limit_deg[0]), float(cfg.limit_deg[1])
        self.backlash_steps = int(round(cfg.backlash_deg * self.steps_per_deg))
        self._sign = -1 if cfg.invert else 1
        self.pos_steps = 0  # logical steps from the power-up pose

    @property
    def angle_deg(self) -> float:
        return self.pos_steps / self.steps_per_deg

    def clamp(self, deg: float) -> float:
        return max(self.lo, min(self.hi, deg))

    def move_to(self, deg: float) -> float:
        """Move to `deg` (clamped). Returns the angle actually commanded."""
        target = int(round(self.clamp(deg) * self.steps_per_deg))
        delta = target - self.pos_steps
        if delta < 0:
            over = self.backlash_steps
            self.driver.step(self._sign * (delta - over))
            if over:
                self.driver.step(self._sign * over)
        elif delta > 0:
            self.driver.step(self._sign * delta)
        self.pos_steps = target
        return self.angle_deg

    def release(self) -> None:
        self.driver.release()


class Aim(ABC):
    """Actuator interface: point the beam at (pan, tilt) degrees."""

    @abstractmethod
    def aim(self, pan_deg: float, tilt_deg: float) -> Tuple[float, float]:
        """Move; returns the (pan, tilt) actually commanded after clamping."""

    @abstractmethod
    def release(self) -> None: ...

    @property
    @abstractmethod
    def position(self) -> Tuple[float, float]: ...

    @property
    @abstractmethod
    def limits(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """((pan_lo, pan_hi), (tilt_lo, tilt_hi))."""


class StepperAim(Aim):
    def __init__(self, pan: Axis, tilt: Axis):
        self.pan_axis, self.tilt_axis = pan, tilt

    def aim(self, pan_deg: float, tilt_deg: float) -> Tuple[float, float]:
        return self.pan_axis.move_to(pan_deg), self.tilt_axis.move_to(tilt_deg)

    def release(self) -> None:
        self.pan_axis.release()
        self.tilt_axis.release()

    @property
    def position(self) -> Tuple[float, float]:
        return self.pan_axis.angle_deg, self.tilt_axis.angle_deg

    @property
    def limits(self):
        return (self.pan_axis.lo, self.pan_axis.hi), (self.tilt_axis.lo, self.tilt_axis.hi)


def make_stepper_aim(cfg: MotorConfig, fake: bool = False) -> StepperAim:
    if fake:
        pan_drv, tilt_drv = FakeStepper(), FakeStepper()
    else:  # pragma: no cover - needs Pi hardware
        pan_drv = MotorKitStepper(cfg.pan.channel, cfg.style, cfg.step_delay_s)
        tilt_drv = MotorKitStepper(cfg.tilt.channel, cfg.style, cfg.step_delay_s)
    return StepperAim(Axis(pan_drv, cfg.pan, "pan"), Axis(tilt_drv, cfg.tilt, "tilt"))
