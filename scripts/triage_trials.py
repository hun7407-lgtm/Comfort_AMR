#!/usr/bin/env python3
"""Sort recorded trials into "fine" and "something is wrong here".

    python3 scripts/triage_trials.py results/stress/wave_*/trials.csv

Plain Python 3. Reads what comfort_core.metrics writes.

The point of a stress run is to find driving faults, and a stress run
of forty trials is too many to read by hand. This says which ones are
worth opening, in severity order, and what to open them for.

What counts as a fault
----------------------
Not every trial that fails to reach the goal is a fault. A robot that
declines to squeeze past somebody is doing its job, and a deadlock that
comes from a Comfort Space is the thing the study is about -- B2 is
SUPPOSED to deadlock. So those are reported and not flagged.

These are faults:

    contact       the robot drove into a person. Nothing excuses this.
    diverged      the simulation blew up; the trial is not evidence of
                  anything and something physical is wrong
    stuck         no progress, and not because of anybody's Comfort
                  Space: the safety layer latched, the planner gave up,
                  or the robot parked itself somewhere it could not
                  leave
    no-plan       the planner could not find a route at all
    crawl         it arrived, but spent most of the trial not moving
                  with nobody to wait for
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.metrics import read_rows  # noqa: E402

# Worst first: this is the order they get fixed in.
SEVERITY = ["contact", "diverged", "stuck", "no-plan", "crawl"]


def _f(row: Dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "") or 0.0)
    except ValueError:
        return 0.0


def classify(row: Dict[str, str]) -> Optional[str]:
    """The fault in this trial, or None if there is nothing to fix."""
    if _f(row, "contact_robot_seconds") > 0:
        return "contact"
    if row.get("valid", "True") != "True":
        return "diverged"
    if row.get("planning_success") != "True":
        return "no-plan"

    cause = (row.get("deadlock_cause") or "").lower()
    # ONE cause is the behaviour under study: the yield policy declining
    # to intrude, which is what B2 is supposed to do. Everything else
    # that leaves the robot standing is a fault in the driving.
    #
    # "no progress ... while yielding" is NOT that. It means the robot
    # stopped to give way and then never started again, which is the
    # fault this stress run exists to find.
    if "yielding to a comfort space" in cause:
        return None
    if row.get("result") != "SUCCESS":
        return "stuck"

    # Arrived -- but how much of the trial was spent standing still for
    # no stated reason? Waiting for people and yielding are reasons.
    travel = _f(row, "travel_time")
    if travel > 0 and row.get("goal_reached") == "True":
        moving = _f(row, "travel_distance") / max(travel, 1e-6)
        excused = _f(row, "waited_for_humans") + _f(row, "yield_seconds")
        if moving < 0.15 and (travel - excused) > 0.6 * travel:
            return "crawl"
    return None


def detail(row: Dict[str, str], fault: str) -> str:
    if fault == "contact":
        return (f"drove into {row.get('min_clearance_who', '?')}, "
                f"{_f(row, 'contact_robot_seconds'):.1f} s, "
                f"closest {row.get('min_clearance')} m")
    if fault == "diverged":
        return row.get("invalid_reason", "")[:70]
    if fault == "stuck":
        cause = row.get("deadlock_cause") or row.get("result", "")
        return (f"{cause[:60]}  (safety {_f(row, 'safety_seconds'):.0f} s "
                f"[{row.get('safety_polygon', '-')}])")
    if fault == "no-plan":
        return f"nav2 {row.get('result')}, no route from the planner"
    return (f"arrived in {_f(row, 'travel_time'):.0f} s but moved only "
            f"{_f(row, 'travel_distance'):.1f} m")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+")
    parser.add_argument("--faults-only", action="store_true")
    args = parser.parse_args()

    rows: List[Dict[str, str]] = []
    for name in args.files:
        path = Path(name)
        if path.is_file():
            rows.extend(read_rows(str(path)))
        else:
            print(f"[WARN] no file at {path}", file=sys.stderr)
    if not rows:
        print("[ERROR] no rows read", file=sys.stderr)
        return 1

    faults: Dict[str, List[Dict[str, str]]] = {}
    clean = 0
    for row in rows:
        fault = classify(row)
        if fault is None:
            clean += 1
        else:
            faults.setdefault(fault, []).append(row)

    print(f"\n=== {len(rows)} trials: {clean} without a fault, "
          f"{len(rows) - clean} to look at ===\n")

    for fault in SEVERITY:
        group = faults.get(fault, [])
        if not group:
            continue
        print(f"  {fault.upper()}  x{len(group)}")
        for row in group:
            config = Path(row.get("config", "")).stem
            print(f"    {config:<16s} {row.get('scenario', ''):<3s} "
                  f"{detail(row, fault)}")
        print()

    if not args.faults_only:
        # What the clean trials did, because "no fault" is not the same
        # as "went well" -- a bay where every robot declines to move is
        # free of faults too.
        arrived = sum(1 for r in rows if r.get("goal_reached") == "True")
        comfort = sum(1 for r in rows if r.get("deadlock") == "True"
                      and "comfort" in (r.get("deadlock_cause") or "").lower())
        print(f"  of the {len(rows)}: {arrived} reached the goal, "
              f"{comfort} stopped on a Comfort Space (not a fault)")
    return 2 if faults else 0


if __name__ == "__main__":
    sys.exit(main())
