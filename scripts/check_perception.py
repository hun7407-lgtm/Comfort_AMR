#!/usr/bin/env python3
"""Drive the perception path offline, without Isaac Sim.

    python3 scripts/check_perception.py
    python3 scripts/check_perception.py --config configs/factory_u_lidar.yaml

Why this exists
---------------
A Comfort Space policy that produces nothing looks exactly like B1
from the outside: the robot drives straight past everybody and every
number in the trial summary is ordinary except a yield time of zero.

That happened. Detections are named `p1`, the shape builder iterated
the config's `w1`, and twelve B2 and B3 trials silently ran as B1
before anyone noticed. Half an hour of simulation each.

The whole chain below is pure Python -- that is why it lives in
comfort_core and in a node that can be imported -- so it can be
exercised here in a second instead of there in a day. It asserts the
chain CONNECTS, not that it detects well: how well is what
`scripts/people_tracker.py --evaluate` measures against a running
simulation.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from comfort_core import load_config  # noqa: E402
from comfort_core.comfort_space import make_policy, robot_grow  # noqa: E402
from comfort_core.comfort_space import shapes as build_shapes  # noqa: E402
from comfort_core.human_state import SensorStateProvider  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/factory_u_lidar.yaml")
    parser.add_argument("--scenario", default="b3")
    args = parser.parse_args()

    cfg = load_config(scenario=args.scenario,
                      common_path=str(PROJECT_ROOT / args.config))
    policy = make_policy(cfg)
    provider = SensorStateProvider(cfg)
    grow = robot_grow(cfg)

    # Three people the sensor has found, named the way a detector names
    # them -- which is NOT the way the config does.
    seen = {"p1": (2.50, 3.05), "p2": (4.00, 3.05), "p3": (0.00, 4.21)}
    speeds = {"p1": 0.0, "p2": 0.0, "p3": 1.10}
    headings = {"p1": 0.0, "p2": 0.0, "p3": 3.14}

    failures = []
    states = {}
    for tick in range(20):                       # 2 s at 10 Hz
        states = provider.update(seen, speeds, tick * 0.1, headings)
    print(f"  states     : {states}")
    if set(states) != set(seen):
        failures.append(f"states are named {sorted(states)}, "
                        f"expected {sorted(seen)}")

    axes = policy.axes(states, speeds)
    out = build_shapes(cfg, axes, seen, headings, grow)
    print("  shapes     : " + ", ".join(
        f"{s.key} at ({s.x:+.2f},{s.y:+.2f}) {s.along:.2f} x {s.across:.2f} m"
        for s in out))

    if policy.gives_space() and len(out) != len(seen):
        failures.append(f"{len(seen)} people were detected and "
                        f"{len(out)} Comfort Space(s) came back; this "
                        f"scenario would run as B1")
    if not policy.gives_space() and out:
        failures.append("the policy says no Comfort Space and shapes "
                        "came back anyway")

    # A person the sensor has lost must stop having a Comfort Space,
    # or the robot keeps yielding to somebody who is not there.
    gone = provider.update({"p1": seen["p1"]}, {"p1": 0.0}, 5.0)
    print(f"  after loss : {sorted(gone)}")
    if set(gone) != {"p1"}:
        failures.append(f"people the sensor stopped seeing are still in "
                        f"the state: {sorted(gone)}")

    if failures:
        print("[check_perception] " + "\n[check_perception] ".join(failures),
              file=sys.stderr)
        return 1
    print(f"[check_perception] detections -> states -> Comfort Spaces, "
          f"{len(out)} shape(s) for {len(seen)} detected people")
    return 0


if __name__ == "__main__":
    sys.exit(main())
