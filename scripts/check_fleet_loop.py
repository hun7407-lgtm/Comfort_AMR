#!/usr/bin/env python3
"""Is every fleet place in a config clear of the mission's ends?

    python3 scripts/check_fleet_loop.py --config configs/fleet_loop.yaml

The scenario generator keeps fleet jobs `scenario_pool.mission_standoff`
away from the measured robot's start and goal; an overlay that rewrites
the job lists by hand bypasses that, so this recomputes it.
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from comfort_core import load_config  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--scenario", default=None)
    args = ap.parse_args()
    cfg = load_config(scenario=args.scenario, common_path=args.config)
    standoff = float(cfg.get("scenario_pool", {}).get("mission_standoff", 1.8))
    ends = [tuple(float(v) for v in cfg["navigation"]["start"][:2]),
            tuple(float(v) for v in cfg["navigation"]["goal"][:2])]
    places = cfg["fleet"]["places"]
    bad = 0
    for name, spec in sorted(cfg["fleet"]["amrs"].items()):
        if not spec.get("enabled", True):
            continue
        for place in [spec["start"]] + list(spec.get("jobs", [])):
            x, y = (float(v) for v in places[place])
            d = min(math.dist((x, y), e) for e in ends)
            ok = d >= standoff
            bad += not ok
            print(f"  {'ok  ' if ok else 'FAIL'} {name} {place:<8s} ({x:+.2f}, {y:+.2f}) "
                  f"{d:.2f} m from the nearest mission end (need {standoff:.2f})")
    print(f"{'all clear' if not bad else str(bad) + ' place(s) too close'}")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
