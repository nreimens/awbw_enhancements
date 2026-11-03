"""Game loop: aim at a random reachable spot; score when the puck sits on the dot."""
from __future__ import annotations

import argparse
import math
import random
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

from .calibrate import Calibration, locate_dot
from .config import Config, load_config
from .laser import Laser
from .motion import Aim
from .vision import FrameSource, PuckTracker


@dataclass
class TargetResult:
    target_px: Tuple[float, float]
    hit: bool
    reaction_s: Optional[float]


class Targeter:
    """aim(x, y) in camera pixels, via the calibration."""

    def __init__(self, aim: Aim, cal: Calibration):
        self.aim, self.cal = aim, cal

    def aim_at_pixel(self, x: float, y: float) -> Tuple[float, float]:
        pan, tilt = self.cal.pixel_to_angles(x, y)
        return self.aim.aim(pan, tilt)

    def pick_target(self, rng: random.Random, margin: float, min_jump: float,
                    prev: Optional[Tuple[float, float]], tries: int = 200) -> Tuple[float, float]:
        (plo, phi), (tlo, thi) = self.aim.limits
        best = None
        for _ in range(tries):
            px, py = self.cal.angles_to_pixel(rng.uniform(plo, phi), rng.uniform(tlo, thi))
            if not (margin <= px <= self.cal.width - margin and margin <= py <= self.cal.height - margin):
                continue
            best = (px, py)
            if prev is None or math.dist(prev, best) >= min_jump:
                return best
        if best is None:
            raise RuntimeError("no reachable target inside the image; recalibrate or widen limits")
        return best


class Game:
    def __init__(self, targeter: Targeter, laser: Laser, cam: FrameSource, tracker: PuckTracker, cfg: Config,
                 clock: Callable[[], float] = time.monotonic, sleep: Callable[[float], None] = time.sleep,
                 rng: Optional[random.Random] = None,
                 on_hit: Callable[[TargetResult], None] = lambda r: None):
        self.t, self.laser, self.cam, self.tracker, self.cfg = targeter, laser, cam, tracker, cfg
        self.clock, self.sleep, self.rng, self.on_hit = clock, sleep, rng or random.Random(), on_hit
        self.results: List[TargetResult] = []
        self._prev: Optional[Tuple[float, float]] = None

    def play_target(self) -> TargetResult:
        g = self.cfg.game
        target = self.t.pick_target(self.rng, g.edge_margin_px, g.min_jump_px, self._prev)
        self._prev = target
        self.t.aim_at_pixel(*target)
        self.sleep(g.settle_s)
        # Hit test uses the camera-observed dot (not the commanded one), per the backlash decision.
        seen = locate_dot(self.laser, self.cam, self.cfg.vision, sleep=self.sleep)
        dot = (seen.x, seen.y) if seen else target
        self.laser.on()
        start = self.clock()
        on_since: Optional[float] = None
        result = TargetResult(dot, False, None)
        while self.clock() - start < g.target_timeout_s:
            frame = self.cam.read()
            now = self.clock()
            puck = self.tracker.update(frame) if frame is not None else None
            if puck is not None and math.dist((puck.x, puck.y), dot) <= g.hit_radius_px:
                on_since = now if on_since is None else on_since
                if now - on_since >= g.dwell_s:
                    result = TargetResult(dot, True, on_since - start)
                    break
            else:
                on_since = None
        self.results.append(result)
        if result.hit:
            self.on_hit(result)
        return result

    def run(self, n_targets: int) -> List[TargetResult]:
        try:
            for _ in range(n_targets):
                self.play_target()
        finally:
            self.laser.off()
        return self.results


def main(argv: Optional[Sequence[str]] = None) -> int:  # pragma: no cover - needs hardware
    ap = argparse.ArgumentParser(description="Play the stickhandling game.")
    ap.add_argument("--config", default=None)
    ap.add_argument("--targets", type=int, default=10)
    ap.add_argument("--yes-laser", action="store_true")
    args = ap.parse_args(argv)
    from .rig import confirm_laser, open_rig
    cfg = load_config(args.config)
    if not args.yes_laser and not confirm_laser():
        return 1
    cal = Calibration.load(cfg.calibration.file)
    with open_rig(cfg) as rig:
        game = Game(Targeter(rig.aim, cal), rig.laser, rig.camera, PuckTracker(cfg.vision), cfg,
                    on_hit=lambda r: print(f"hit in {r.reaction_s:.2f}s"))
        res = game.run(args.targets)
    hits = [r for r in res if r.hit]
    print(f"{len(hits)}/{len(res)} hit; mean reaction "
          f"{sum(r.reaction_s for r in hits) / len(hits):.2f}s" if hits else f"0/{len(res)} hit")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
