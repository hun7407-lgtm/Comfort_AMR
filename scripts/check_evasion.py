#!/usr/bin/env python3
"""Exercise the yield logic offline, without ROS or Isaac Sim.

    python3 scripts/check_evasion.py
    python3 scripts/check_evasion.py --config configs/factory_u.yaml

Why this exists
---------------
`scripts/check_nodes.py` catches a bad import. It cannot catch a bad
name INSIDE a function, and that is the mistake that keeps costing
whole waves: one `self.cp` in a module-level function, and the comfort
gate died the first time a walker triggered a sidestep. From the
outside it looked like the robot yielded politely and then stood
still -- nine trials recorded as Comfort Space deadlocks, which is a
result this study publishes, so the fault was not even obviously a
fault.

These paths are pure Python by design (that is why they live in
comfort_core), so they can be run in a second here instead of
twenty minutes there.

It drives a few synthetic encounters and asserts only that the logic
RUNS and returns something of the right shape. It is not a test of
whether the robot yields well -- that is what the stress waves measure.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config  # noqa: E402
from comfort_core.evasion import (ComfortParams, EvasionController,  # noqa: E402
                                  EvasionParams, Person, choose_comfort_spot,
                                  choose_target)
from comfort_core.shapes import ComfortShape  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/factory_u.yaml")
    args = parser.parse_args()

    cfg = load_config(common_path=str(PROJECT_ROOT / args.config), scenario="b3")
    p = EvasionParams.from_config(cfg)
    cp = ComfortParams.from_config(cfg)

    # A walker coming the other way down each aisle, and the robot
    # somewhere along it. Both aisles, both directions, and a case with
    # somebody standing beside the lane -- which is the one that found
    # the robot driving through a bystander on its way to a waiting
    # point.
    cases = [
        ("north, head-on", (6.0, 4.21, math.pi), Person("c", 2.0, 4.21, speed=1.0, heading=math.pi), []),
        ("north, from behind", (2.0, 4.21, 0.0), Person("c", 6.0, 4.21, speed=1.0, heading=math.pi), []),
        ("south, head-on", (-6.0, -4.21, 0.0), Person("c", -2.0, -4.21, speed=1.0, heading=0.0), []),
        ("bystander beside the lane", (0.0, 4.21, math.pi),
         Person("c", -4.0, 4.21, speed=1.0, heading=0.0),
         [ComfortShape("w", -1.0, 3.01, 0.6, 0.6, math.pi / 2)]),
    ]

    failures = []
    for name, pose, walker, others in cases:
        try:
            spot = choose_comfort_spot(cfg, walker, 1.2, 1.2, pose, others, p, cp)
            target = choose_target(cfg, walker, (pose[0], pose[1]),
                                   [(o.x, o.y) for o in others], p)
            controller = EvasionController(cfg, log=lambda _m: None)
            raw = {walker.key: ComfortShape(walker.key, walker.x, walker.y,
                                            1.2, 1.2, walker.heading)}
            for shape in others:
                raw[shape.key] = shape
            people = [walker] + [Person(o.key, o.x, o.y, speed=0.0, heading=0.0)
                                 for o in others]
            for tick in range(20):
                controller.step(tick * 0.1, pose, people, raw)
        except Exception as exc:  # noqa: BLE001 - this is the point
            failures.append(f"{name}: {type(exc).__name__}: {exc}")
            continue
        where = "no spot" if spot is None else f"({spot.x:+.2f}, {spot.y:+.2f})"
        print(f"  {name:28s} spot {where:>18s}  target "
              f"{'none' if target is None else 'found'}")

    if failures:
        print("[check_evasion] " + "\n[check_evasion] ".join(failures),
              file=sys.stderr)
        return 1
    print(f"[check_evasion] {len(cases)} encounter(s) ran without raising")
    return 0


if __name__ == "__main__":
    sys.exit(main())
