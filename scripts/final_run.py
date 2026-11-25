#!/usr/bin/env python3
"""Run a final-experiment manifest, one fresh simulation per trial, resumable.

    python3 scripts/final_run.py --dir results/batches/261125_final_stage1/coarse
    python3 scripts/final_run.py --dir <batch> --limit 1500 --retries 2 --no-record
    python3 scripts/final_run.py --dir <batch> --seeds 1-30     # only these seeds of the manifest

Plain Python 3 (calls ./scripts/run_scenario.sh). Reads <dir>/manifest.tsv
(trial_id, seed, condition, config) and runs every trial that has no
<dir>/trials/<trial_id>/DONE yet, in manifest order. Stop it any time
(Ctrl+C / kill) and start it again: finished trials are skipped, an
unfinished one is moved aside (attempt folder kept) and run again.

Per trial, in <dir>/trials/<trial_id>/:
    trials.csv, fleet_trials.csv, <stamp>_*.json   nav2_goal / fleet_goals rows (AMR 1, a1, a2)
    trial_log.jsonl                                scripts/trial_logger.py (people, Comfort Spaces,
                                                   poses, plans, safety states) on the sim clock
    <stamp>_camera.mp4, <stamp>_frames.jsonl       top-view recording of THIS run + pose log
    console.log                                    run_scenario.sh output
    DONE                                           {"result", "attempts", "wall_s", ...}

A run is repeated (up to --retries more times) only when it never became a
trial: Isaac Sim / Nav2 / fleet Nav2 did not come up, no AMR 1 row was
written, or the row is INVALID because the simulation stopped (no odometry).
Every other outcome -- SUCCESS, a deadlock, a protective stop, a timeout --
is a result and is kept as it is. Failed attempts stay in
trials/<trial_id>/attempt_<k>/ for the record.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
INFRA = ("did not come up", "did not activate")


def read_manifest(d: Path):
    rows = []
    for line in (d / "manifest.tsv").read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        tid, seed, cond, cfg = line.split("\t")
        rows.append((tid, int(seed), cond, cfg))
    return rows


def amr1_row(t: Path):
    f = t / "trials.csv"
    if not f.exists():
        return None
    rows = list(csv.DictReader(f.open()))
    return rows[-1] if rows else None


def infra_failure(t: Path, row) -> str:
    log = (t / "console.log").read_text(errors="replace") if (t / "console.log").exists() else ""
    for k in INFRA:
        if k in log:
            return k
    if row is None:
        return "no AMR 1 row"
    if row.get("result") == "INVALID" and "no odometry" in (row.get("invalid_reason") or ""):
        return "simulation stopped (no odometry)"
    return ""


def say(d: Path, msg: str) -> None:
    line = f"{datetime.now():%m-%d %H:%M:%S}  {msg}"
    print(line, flush=True)
    with (d / "progress.log").open("a") as fh:
        fh.write(line + "\n")


def run_one(d: Path, tid: str, cond: str, cfg: str, limit: int, record: bool, fleet_tail: int) -> Path:
    t = d / "trials" / tid
    t.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, TRIAL_LOG=str(t / "trial_log.jsonl"), TRIAL_VIDEO_DIR=str(t))
    cmd = ["timeout", "-k", "30", str(limit), "./scripts/run_scenario.sh", cond, "--config", cfg,
           "--headless", "--results", str(t), "--fleet-tail", str(fleet_tail)]
    if record:
        cmd.append("--record")
    with (t / "console.log").open("w") as out:
        subprocess.run(cmd, cwd=PROJECT_ROOT, env=env, stdin=subprocess.DEVNULL, stdout=out,
                       stderr=subprocess.STDOUT)
    return t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", required=True)
    ap.add_argument("--limit", type=int, default=1500, help="wall-clock seconds per trial")
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--fleet-tail", type=int, default=150)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--max", type=int, default=0, help="stop after this many trials (0 = all)")
    ap.add_argument("--seeds", default="", help="only manifest rows with these seeds, e.g. 1-30 (default all)")
    args = ap.parse_args()
    d = Path(args.dir)
    if not d.is_absolute():
        d = PROJECT_ROOT / d
    rows = read_manifest(d)
    if args.seeds:
        a, b = (int(x) for x in args.seeds.split("-"))
        rows = [r for r in rows if a <= int(r[1]) <= b]
    todo = [r for r in rows if not (d / "trials" / r[0] / "DONE").exists()]
    commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT,
                            capture_output=True, text=True).stdout.strip()
    say(d, f"start: {len(rows)} in manifest, {len(rows) - len(todo)} done, {len(todo)} to run (commit {commit})")
    n = 0
    for tid, seed, cond, cfg in todo:
        if args.max and n >= args.max:
            break
        t = d / "trials" / tid
        if t.exists():                       # interrupted earlier: keep it aside
            k = 1
            while (d / "trials" / f"{tid}.interrupted_{k}").exists():
                k += 1
            t.rename(d / "trials" / f"{tid}.interrupted_{k}")
        attempts = []
        t0 = time.time()
        for a in range(args.retries + 1):
            ta = time.time()
            run_one(d, tid, cond, cfg, args.limit, not args.no_record, args.fleet_tail)
            row = amr1_row(t)
            why = infra_failure(t, row)
            attempts.append({"attempt": a + 1, "wall_s": round(time.time() - ta), "infra_failure": why or None})
            if not why:
                break
            keep = t / f"attempt_{a + 1}"
            keep.mkdir()
            for f in list(t.iterdir()):
                if not f.name.startswith("attempt_"):
                    shutil.move(str(f), str(keep / f.name))
            say(d, f"  {tid}: attempt {a + 1} not a trial ({why}); retrying")
        row = amr1_row(t)
        fleet = list(csv.DictReader((t / "fleet_trials.csv").open())) if (t / "fleet_trials.csv").exists() else []
        done = {"trial_id": tid, "seed": seed, "condition": cond, "config": cfg, "commit": commit,
                "result": (row or {}).get("result"), "infra_failure": attempts[-1]["infra_failure"],
                "fleet": {f["robot"]: f["result"] for f in fleet}, "attempts": attempts,
                "wall_s": round(time.time() - t0), "finished": datetime.now().isoformat(timespec="seconds")}
        (t / "DONE").write_text(json.dumps(done, indent=1))
        n += 1
        left = len(todo) - n
        say(d, f"{tid}: AMR1 {done['result']}  fleet {done['fleet']}  {done['wall_s']} s  "
               f"[{len(rows) - left}/{len(rows)}]")
    say(d, "stopped" if args.max and n >= args.max and n < len(todo) else "finished")
    return 0


if __name__ == "__main__":
    sys.exit(main())
