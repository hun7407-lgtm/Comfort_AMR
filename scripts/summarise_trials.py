#!/usr/bin/env python3
"""Group recorded trials and print what they say.

    python3 scripts/summarise_trials.py results/batches/<stamp>/trials.csv
    python3 scripts/summarise_trials.py results/trials/*/trials.csv
    python3 scripts/summarise_trials.py <file> --csv summary.csv

Runs in plain Python 3. No ROS, no omni, no pandas.

Reads the rows `comfort_core.metrics` writes and reports one line per
(layout, scenario): how many trials, how many got there, how many
deadlocked, how long it took, how close it came to people and how far
into their Comfort Space it went.

Three rules about the numbers
-----------------------------
1. Discarded trials (a diverged simulation) are counted and then left
   out of every statistic. They are reported separately so a run of
   twenty that quietly became a run of twelve is visible.
2. Times are medians, not means. One deadlock that sits at the time
   limit drags a mean of five trials by twenty seconds and hides what
   the other four did.
3. The contact figure is `contact_robot_seconds`, the robot driving
   into somebody. People walk into a stopped robot in this study by
   design -- they do not avoid it -- and counting that against a policy
   would be scoring it for something it cannot control. See
   docs/RESEARCH_DESIGN.md section 1.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.metrics import read_rows  # noqa: E402

# Scenarios come out in the order the study compares them, not
# alphabetically by accident.
ORDER = {"b1": 0, "b2": 1, "b3": 2}


def _float(value: str) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out


def _median(values: Sequence[Optional[float]]) -> Optional[float]:
    kept = [v for v in values if v is not None]
    return statistics.median(kept) if kept else None


def _fmt(value: Optional[float], places: int = 2, dash: str = "   -") -> str:
    return dash if value is None else f"{value:.{places}f}"


def summarise(rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """One summary per (config, scenario), in comparison order."""
    groups: Dict[tuple, List[Dict[str, str]]] = {}
    for row in rows:
        key = (row.get("config", ""), row.get("scenario", ""))
        groups.setdefault(key, []).append(row)

    out = []
    for (config, scenario), group in groups.items():
        discarded = [r for r in group if r.get("valid", "True") != "True"]
        kept = [r for r in group if r.get("valid", "True") == "True"]
        n = len(kept)
        success = [r for r in kept if r.get("result") == "SUCCESS"]
        deadlock = [r for r in kept if r.get("deadlock") == "True"]
        timeout = [r for r in kept if r.get("result") == "TIMEOUT"]
        # Time and distance are only meaningful for trials that arrived:
        # a deadlock's "travel time" is however long it took to notice.
        out.append({
            "config": Path(config).stem or config,
            "scenario": scenario,
            "n": n,
            "discarded": len(discarded),
            "success": len(success),
            "deadlock": len(deadlock),
            "timeout": len(timeout),
            "time": _median([_float(r.get("travel_time", "")) for r in success]),
            "distance": _median([_float(r.get("travel_distance", ""))
                                 for r in success]),
            "yield": _median([_float(r.get("yield_seconds", "")) for r in kept]),
            "clearance": _median([_float(r.get("min_clearance", ""))
                                  for r in kept]),
            "intrusion": _median([_float(r.get("worst_intrusion", ""))
                                  for r in kept]),
            "intruded": _median([_float(r.get("people_intruded", ""))
                                 for r in kept]),
            "contact": sum(1 for r in kept
                           if (_float(r.get("contact_robot_seconds", "")) or 0) > 0),
            "safety": _median([_float(r.get("safety_seconds", "")) for r in kept]),
            "state": _median([_float(r.get("state_agreement", "")) for r in kept]),
        })
    out.sort(key=lambda s: (s["config"], ORDER.get(s["scenario"], 99),
                            s["scenario"]))
    return out


def report(summaries: List[Dict[str, Any]], rows: List[Dict[str, str]]) -> str:
    lines = []
    lines.append("")
    lines.append(f"=== {len(rows)} trials recorded ===")
    lines.append("")
    lines.append("  layout        scn    n  arrived  deadlock  timeout  "
                 "median s   median m   yield s  closest  intrusion  "
                 "people in  ROBOT HIT  safety s  state")
    lines.append("  " + "-" * 155)
    for s in summaries:
        lines.append(
            f"  {s['config']:<13s} {s['scenario']:<4s} {s['n']:>3d}  "
            f"{s['success']:>6d}  {s['deadlock']:>8d}  {s['timeout']:>7d}  "
            f"{_fmt(s['time'], 1):>8s}  {_fmt(s['distance'], 1):>9s}  "
            f"{_fmt(s['yield'], 1):>8s}  {_fmt(s['clearance'], 3):>7s}  "
            f"{_fmt(s['intrusion'], 2):>9s}  {_fmt(s['intruded'], 0):>9s}  "
            f"{s['contact']:>9d}  {_fmt(s['safety'], 1):>8s}  "
            f"{_fmt(s['state'], 3):>5s}")
        if s["discarded"]:
            lines.append(f"  {'':13s} {'':4s}      "
                         f"({s['discarded']} discarded, left out of the row)")
    lines.append("")
    lines.append("  arrived   reached the goal          deadlock  gave up, a route existed")
    lines.append("  closest   metres, footprint to person; negative is contact")
    lines.append("  intrusion metres into the reference Comfort Space, deepest in the trial")
    lines.append("  ROBOT HIT trials where the robot drove into somebody. B1 must be 0.")
    lines.append("  state     map-based state estimate vs the truth, 1.0 is perfect")

    worst = [s for s in summaries if s["contact"]]
    if worst:
        lines.append("")
        for s in worst:
            lines.append(f"  [!] {s['config']}/{s['scenario']}: the robot drove "
                         f"into somebody in {s['contact']} of {s['n']} trials")
    thin = [s for s in summaries if 0 < s["n"] < 3]
    if thin:
        lines.append("")
        lines.append("  [!] fewer than 3 valid trials in a cell; nothing here is "
                     "a median worth quoting yet")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="one or more trials.csv")
    parser.add_argument("--csv", default=None,
                        help="also write the summary itself as a csv")
    args = parser.parse_args()

    rows: List[Dict[str, str]] = []
    for name in args.files:
        path = Path(name)
        if not path.is_file():
            print(f"[WARN] no file at {path}", file=sys.stderr)
            continue
        rows.extend(read_rows(str(path)))
    if not rows:
        print("[ERROR] no rows read", file=sys.stderr)
        return 1

    summaries = summarise(rows)
    print(report(summaries, rows))

    if args.csv:
        out = Path(args.csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
        print(f"\n  summary written to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
