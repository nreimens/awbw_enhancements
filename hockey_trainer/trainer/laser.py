"""Laser output with hard safety behavior: off by default, watchdog timeout, off at exit."""
from __future__ import annotations

import atexit
import threading
from abc import ABC, abstractmethod
from typing import Callable, List, Optional, Tuple

from .config import LaserConfig


class Laser(ABC):
    @abstractmethod
    def on(self, brightness: Optional[float] = None) -> None: ...

    @abstractmethod
    def off(self) -> None: ...

    @property
    @abstractmethod
    def is_on(self) -> bool: ...

    def close(self) -> None:
        self.off()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class _Watchdog:
    """Forces `off` after max_on_s unless re-armed. Daemon timer, cancelled on off."""

    def __init__(self, max_on_s: float, off: Callable[[], None]):
        self.max_on_s = max_on_s
        self._off = off
        self._timer: Optional[threading.Timer] = None

    def arm(self) -> None:
        self.cancel()
        if self.max_on_s and self.max_on_s > 0:
            self._timer = threading.Timer(self.max_on_s, self._off)
            self._timer.daemon = True
            self._timer.start()

    def cancel(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None


class FakeLaser(Laser):
    """Records transitions; used by tests, the simulator and `--fake` hardware checks."""

    def __init__(self, max_on_s: float = 0.0):
        self._on = False
        self.brightness = 0.0
        self.history: List[Tuple[str, float]] = []
        self._watchdog = _Watchdog(max_on_s, self.off)

    def on(self, brightness: Optional[float] = None) -> None:
        self._on = True
        self.brightness = 1.0 if brightness is None else float(brightness)
        self.history.append(("on", self.brightness))
        self._watchdog.arm()

    def off(self) -> None:
        self._watchdog.cancel()
        self._on = False
        self.history.append(("off", 0.0))

    @property
    def is_on(self) -> bool:
        return self._on


class GpioLaser(Laser):
    """Laser switched through an NPN/MOSFET on a BCM GPIO via gpiozero."""

    def __init__(self, cfg: LaserConfig):
        try:
            from gpiozero import DigitalOutputDevice, PWMOutputDevice
        except ImportError as exc:  # pragma: no cover - needs Pi libs
            raise RuntimeError("gpiozero not installed (pip install gpiozero lgpio)") from exc
        self.cfg = cfg
        cls = PWMOutputDevice if cfg.pwm else DigitalOutputDevice
        self._dev = cls(cfg.gpio_bcm, active_high=cfg.active_high, initial_value=False)
        self._on = False
        self._watchdog = _Watchdog(cfg.max_on_s, self.off)
        atexit.register(self.close)

    def on(self, brightness: Optional[float] = None) -> None:
        level = self.cfg.brightness if brightness is None else brightness
        level = max(0.0, min(1.0, float(level)))
        if self.cfg.pwm:
            self._dev.value = level
        else:
            self._dev.on()
        self._on = level > 0
        self._watchdog.arm()

    def off(self) -> None:
        self._watchdog.cancel()
        try:
            self._dev.off()
        finally:
            self._on = False

    @property
    def is_on(self) -> bool:
        return self._on

    def close(self) -> None:
        self.off()
        try:
            self._dev.close()
        except Exception:
            pass
