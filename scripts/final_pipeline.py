#!/usr/bin/env python3
"""Run the final Stage 1-3 experiments end to end, unattended and resumable.

    nohup python3 scripts/final_pipeline.py > results/batches/261125_final_pipeline.log 2>&1 &

Plain Python 3. Every phase is idempotent (final_run.py skips finished
trials), so the same command resumes after any interruption. Decisions
between phases are the pre-registered rules of docs/FINAL_261125_PLAN.md,
computed by the report scripts, never chosen by hand:

    1. Stage 1 coarse     B1 + B2 0.80..1.60, seeds 1-100     final_run.py
    2. Stage 1 report     transition / fine interval          final_stage1_report.py
    3. Stage 1 fine       0.05 steps inside the interval       final_plan.py stage1-fine
                          (transition at the smallest size -> 0.60..0.75; none -> no fine search)
    4. Stage 1 report     R_nav_upper (final)
    5. Stage 2 proxy      R_lower_proxy(H,d)                   final_stage2_proxy.py
    6. Stage 3 configs    B1 / B2 / B3-O MID (+ LOWER, UPPER)  final_plan.py stage3
    7. Stage 3 main       seeds 101-200 x 3                     final_run.py
    8. Stage 3 sensitivity seeds 101-200 x 2                    final_run.py

Reduced run (user decision 2026-11-25): --stage1-seeds 1-30 --stage3-seeds 101-130.
Manifests keep all 100 seeds, so the remaining seeds can be added later by
running again with the full ranges.

A STOP file in the pipeline folder (results/batches/261125_final_pipeline.STOP)
stops it after the current trial phase call returns.
"""

from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
B = PROJECT_ROOT / "results" / "batches"
S1 = B / "261125_final_stage1"
S3 = B / "261125_final_stage3"
STOP = B / "261125_final_pipeline.STOP"


def say(msg):
    print(f"{datetime.now():%m-%d %H:%M:%S}  [pipeline] {msg}", flush=True)


def run(*cmd):
    say(" ".join(cmd))
    r = subprocess.run([sys.executable, *cmd], cwd=PROJECT_ROOT)
    if r.returncode != 0:
        raise SystemExit(f"[pipeline] failed: {' '.join(cmd)} (exit {r.returncode})")


def trials(phase_dir: Path, seeds: str):
    run("scripts/final_run.py", "--dir", str(phase_dir.relative_to(PROJECT_ROOT)), "--seeds", seeds)
    if STOP.exists():
        raise SystemExit("[pipeline] STOP file found")


def fine_sizes(dec):
    t, interval = dec["transition_size_m"], dec["fine_interval"]
    if t is None:
        return []
    lo = interval[0] if interval and interval[0] is not None else round(t - 0.25, 2)
    out, x = [], round(lo + 0.05, 2)
    while x < t - 1e-9:
        out.append(round(x, 2))
        x = round(x + 0.05, 2)
    return out


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage1-seeds", default="1-100")
    ap.add_argument("--stage3-seeds", default="101-200")
    a = ap.parse_args()
    say(f"seeds: Stage 1 {a.stage1_seeds}, Stage 3 {a.stage3_seeds}")
    trials(S1 / "coarse", a.stage1_seeds)
    run("scripts/final_stage1_report.py", "--partial")
    dec = json.loads((S1 / "analysis" / "stage1_upper_bound.json").read_text())["decision"]
    sizes = fine_sizes(dec)
    say(f"coarse decision {dec}; fine sizes {sizes}")
    if sizes:
        run("scripts/final_plan.py", "stage1-fine", "--sizes", ",".join(f"{s:.2f}" for s in sizes))
        trials(S1 / "fine", a.stage1_seeds)
    run("scripts/final_stage1_report.py")
    run("scripts/final_stage2_proxy.py")
    run("scripts/final_plan.py", "stage3")
    trials(S3 / "main", a.stage3_seeds)
    trials(S3 / "sensitivity", a.stage3_seeds)
    say("all phases finished")
    return 0


if __name__ == "__main__":
    sys.exit(main())
