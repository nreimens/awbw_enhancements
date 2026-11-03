"""Typed configuration loaded from config.yaml (all fields have defaults)."""
from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Optional, Tuple, Union

import yaml


@dataclass
class AxisConfig:
    channel: int = 1
    steps_per_rev: int = 2048
    invert: bool = False
    backlash_deg: float = 1.5
    limit_deg: Tuple[float, float] = (-45.0, 45.0)


@dataclass
class MotorConfig:
    style: str = "DOUBLE"
    step_delay_s: float = 0.004
    pan: AxisConfig = field(default_factory=lambda: AxisConfig(channel=1, limit_deg=(-45.0, 45.0)))
    tilt: AxisConfig = field(default_factory=lambda: AxisConfig(channel=2, limit_deg=(5.0, 45.0)))


@dataclass
class LaserConfig:
    gpio_bcm: int = 17
    active_high: bool = True
    pwm: bool = False
    brightness: float = 1.0
    max_on_s: float = 10.0


@dataclass
class CameraConfig:
    device: Union[str, int] = "auto"
    name_hint: str = "OV9281"
    width: int = 640
    height: int = 400
    fps: int = 120
    fourcc: str = "MJPG"
    exposure: Optional[float] = 40
    flush_frames: int = 3


@dataclass
class VisionConfig:
    dot_diff_threshold: int = 40
    dot_min_area: int = 1
    dot_max_area: int = 150
    puck_mode: str = "bright"
    puck_threshold: int = 200
    puck_min_area: int = 40
    puck_max_area: int = 5000


@dataclass
class CalibrationConfig:
    file: str = "calibration.json"
    pan_points: int = 7
    tilt_points: int = 5
    settle_s: float = 0.4


@dataclass
class GameConfig:
    hit_radius_px: float = 30
    dwell_s: float = 0.15
    target_timeout_s: float = 15.0
    min_jump_px: float = 100
    edge_margin_px: float = 40
    settle_s: float = 0.4


@dataclass
class Config:
    motors: MotorConfig = field(default_factory=MotorConfig)
    laser: LaserConfig = field(default_factory=LaserConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    game: GameConfig = field(default_factory=GameConfig)


def _build(cls, data: dict):
    """Recursively build a dataclass from a dict; unknown keys are an error (typo guard)."""
    known = {f.name: f for f in fields(cls)}
    unknown = set(data) - set(known)
    if unknown:
        raise ValueError(f"unknown config key(s) for {cls.__name__}: {sorted(unknown)}")
    kwargs: dict[str, Any] = {}
    default = cls()
    for name, value in data.items():
        current = getattr(default, name)
        if is_dataclass(current) and isinstance(value, dict):
            kwargs[name] = _build(type(current), {**_asdict_shallow(current), **value})
        elif isinstance(current, tuple) and isinstance(value, (list, tuple)):
            kwargs[name] = tuple(value)
        else:
            kwargs[name] = value
    return cls(**{**_asdict_shallow(default), **kwargs})


def _asdict_shallow(obj) -> dict:
    return {f.name: getattr(obj, f.name) for f in fields(obj)}


def load_config(path: Union[str, Path, None] = None) -> Config:
    """Load config.yaml; with no path, look for ./config.yaml, else use defaults."""
    if path is None:
        candidate = Path("config.yaml")
        if not candidate.exists():
            return Config()
        path = candidate
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return _build(Config, data)
