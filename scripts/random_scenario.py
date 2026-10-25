#!/usr/bin/env python3
"""Write a randomised variant of the U bay, for stress running.

    python3 scripts/random_scenario.py --seed 7 --out configs/stress

Plain Python 3. No ROS, no omni.

What varies
-----------
    start / goal      any two different empty cart slots
    the swap pair     which two people change places, and when
    an extra walker   sometimes a third person crosses an aisle

What does not vary is the bay: same machines, same aisles, same cart
blocks, same twelve people. A stress run is meant to find out where the
driving breaks, so everything that is not the driving is held still.

Every variant is checked before it is written -- the layout validator,
and the state estimator agreeing with every declared state -- and a
seed that cannot produce a valid scene is reported rather than written.
The seed is in the file name and in the file, so any run can be
reproduced exactly.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config, resolve_path  # noqa: E402
from comfort_core.factory_layout import build_layout, validate_factory  # noqa: E402
from comfort_core.human_state import SensorStateProvider  # noqa: E402
from comfort_core.layout import human_positions, human_states, work_spots  # noqa: E402

BASE = "configs/factory_u.yaml"


def _slots(cfg) -> List[Tuple[str, float, float]]:
    """Empty cart slots, which is where a trial starts and ends."""
    layout = build_layout(cfg)
    return [(s.name, s.centre[0], s.centre[1]) for s in layout.cart_slots_free]


def _belt_posts(cfg, north: bool) -> List[Tuple[float, float]]:
    """Working positions on one leg of the U."""
    return [(x, y) for x, y, _f in work_spots(cfg)
            if (y > 0) == north and abs(y) < 4.0]


def _idle_spots(cfg, north: bool) -> List[Tuple[float, float]]:
    """Somewhere on the storage side to stand that is not a work spot.

    The gaps between cart blocks, which is the only floor over there
    that is neither a slot nor racking.
    """
    layout = build_layout(cfg)
    side = 1.0 if north else -1.0
    blocks = sorted((b for b in layout.cart_zones if (b.y_min > 0) == north),
                    key=lambda b: b.x_min)
    posts = work_spots(cfg)
    out = []
    for left, right in zip(blocks, blocks[1:]):
        middle = (left.x_max + right.x_min) / 2.0
        y = side * 5.60
        # The gap between two blocks is also where a picker works, and
        # standing in a working position IS `station_engaged`. Shift
        # along the gap until the spot is clearly nobody's post.
        for dx in (0.0, -0.6, 0.6, -0.9, 0.9):
            x = middle + dx
            if not (left.x_max + 0.3 <= x <= right.x_min - 0.3):
                continue
            if all(math.hypot(x - px, y - py) > 0.80 for px, py, _f in posts):
                out.append((round(x, 2), y))
                break
    return out


def _aisle_y(cfg) -> Tuple[float, float]:
    bands = {b.name: b for b in build_layout(cfg).bands}
    return (bands["aisle_north"].centre, bands["aisle_south"].centre)


def _round_the_u(cfg, start: Tuple[float, float], end: Tuple[float, float],
                 east: bool) -> List[List[float]]:
    """A walk from one leg of the U to the other, the long way round.

    Nobody walks across a conveyor, so changing sides means going round
    one end of it: into the nearest aisle, along it, across the end,
    and back along the other.
    """
    north_y, south_y = _aisle_y(cfg)
    crossings = [float(x) for x in cfg["factory"]["cross_aisles"]["at"]]
    cross_x = max(crossings) if east else min(crossings)
    # Just inside the crossing, so a person is not walking its edge.
    cross_x -= math.copysign(0.3, cross_x)

    from_north = start[1] > 0
    first = north_y if from_north else south_y
    last = south_y if from_north else north_y
    return [[start[0], first], [cross_x, first], [cross_x, last],
            [end[0], last], [end[0], end[1]]]


def build(seed: int) -> Dict[str, Any]:
    """One variant, as the dictionary that gets written out."""
    rng = random.Random(seed)
    cfg = load_config(common_path=str(PROJECT_ROOT / BASE))

    slots = _slots(cfg)
    # Far enough apart to be a journey. Seed 4 drew two slots 1.4 m
    # apart, which the robot reached in four seconds: nothing about the
    # driving was tested, and the trial still counted as a trial.
    pairs = [(a, b) for a in slots for b in slots
             if math.hypot(a[1] - b[1], a[2] - b[2]) >= 8.0]
    if not pairs:
        raise ValueError("no two slots are far enough apart")
    start, goal = rng.choice(pairs)
    north_goal = goal[2] > 0

    # The swap: somebody idle on one storage side changes places with
    # somebody working on the opposite leg.
    idle_north = rng.random() < 0.5
    idle_spot = rng.choice(_idle_spots(cfg, idle_north))
    belt_post = rng.choice(_belt_posts(cfg, not idle_north))
    east = rng.random() < 0.5
    start_at = round(rng.uniform(3.0, 15.0), 1)

    humans = {
        "w11": {
            "position": [idle_spot[0], idle_spot[1]],
            "state": "stationary",
            "facing_deg": -90.0 if idle_north else 90.0,
            "walk": {"start_at": start_at,
                     "route": _round_the_u(cfg, idle_spot, belt_post, east)},
        },
        "w7": {
            "position": [belt_post[0], belt_post[1]],
            "state": "station_engaged",
            "facing_deg": 90.0 if idle_north else -90.0,
            "walk": {"start_at": start_at,
                     "route": _round_the_u(cfg, belt_post, idle_spot,
                                           not east)},
        },
    }

    out: Dict[str, Any] = {
        "base": BASE,
        "seed": seed,
        "navigation": {
            "start": [round(start[1], 3), round(start[2], 3)],
            "goal": [round(goal[1], 3), round(goal[2], 3)],
            "goal_yaw_deg": -90.0 if north_goal else 90.0,
        },
        "amr": {"spawn": {"yaw_deg": -90.0 if start[2] > 0 else 90.0}},
        "humans": humans,
        "recording": {"version": "stress"},
    }
    return out


def check(variant: Dict[str, Any]) -> List[str]:
    """Every reason this variant is not usable. Empty means it is."""
    import copy
    import yaml

    tmp = resolve_path("results/stress/.check.yaml")
    tmp.parent.mkdir(parents=True, exist_ok=True)
    tmp.write_text(yaml.safe_dump(variant, sort_keys=False), encoding="utf-8")
    try:
        cfg = load_config(common_path=str(tmp))
    except Exception as exc:  # noqa: BLE001
        return [f"will not load: {exc}"]

    problems = list(validate_factory(cfg, build_layout(cfg)))

    # Every declared state has to be the state the map-based estimator
    # reads, or the trial is scoring the estimator against a scene that
    # contradicts itself before anybody moves.
    truth = human_states(cfg)
    pos = {k: (x, y) for k, x, y in human_positions(cfg)}
    est = SensorStateProvider(cfg)
    read = est.update(pos, {k: 0.0 for k in pos}, 0.0)
    for key, state in sorted(truth.items()):
        if read.get(key) != state:
            problems.append(f"humans.{key} is declared {state} but the map "
                            f"reads {read.get(key)}")

    # Nobody standing where the robot starts or parks.
    for label in ("start", "goal"):
        px, py = cfg["navigation"][label]
        for key, (hx, hy) in pos.items():
            if math.hypot(hx - px, hy - py) < 1.2:
                problems.append(f"humans.{key} is {math.hypot(hx-px, hy-py):.2f} m "
                                f"from navigation.{label}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--out", default="configs/stress")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    import yaml

    variant = build(args.seed)
    problems = check(variant)
    if problems:
        print(f"[seed {args.seed}] unusable:\n  - " + "\n  - ".join(problems),
              file=sys.stderr)
        return 1

    out_dir = resolve_path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"stress_{args.seed:04d}.yaml"
    header = (f"# Randomised stress variant, seed {args.seed}.\n"
              f"# Written by scripts/random_scenario.py; edit the seed, not this.\n")
    path.write_text(header + yaml.safe_dump(variant, sort_keys=False),
                    encoding="utf-8")
    if not args.quiet:
        nav = variant["navigation"]
        print(f"{path}  start {nav['start']} -> goal {nav['goal']}  "
              f"swap at t={variant['humans']['w11']['walk']['start_at']} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
