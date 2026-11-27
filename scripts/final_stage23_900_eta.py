#!/usr/bin/env python3
"""Measured progress + ETA (Asia/Seoul) for the stage23_900 campaign.

    python3 scripts/final_stage23_900_eta.py            # write ETA_REPORT.md + progress_<ts>.json once
    python3 scripts/final_stage23_900_eta.py --watch    # rewrite at every 30 completed MAIN trials
                                                        # (and when a batch finishes); exits when all done

Only measured wall-clock time is used: every trial's DONE `wall_s` (Isaac Sim start,
ROS 2 bring-up, the run, teardown and data writing, all retries of that trial) and
the gap between one trial's end and the next one's start (runner overhead). One Isaac
Sim worker on one GPU: trials run one after another (no parallelism is assumed).

Per stage, from all of its launches so far (pilot + main, every outcome incl. timeouts):
    optimistic   = P25 wall + median gap        per remaining trial
    central      = mean wall + median gap
    conservative = P90 wall + P90 gap, + 5 % retry / infra margin
Once >= 30 MAIN trials of a stage exist, central also reports the live throughput
(trials/hour over the main run's elapsed wall time). Remaining work is executed in the
campaign order below; a fixed allowance covers freeze / commit / analysis steps.
No sample -> that stage's ETA is NOT computed (stated as such).
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
KST = timezone(timedelta(hours=9))
PREFIX = "results/batches/261127_stage23_900"
STATUS = PROJECT_ROOT / f"{PREFIX}_status"
# campaign order: (batch suffix, stage, is_main, nominal count if no manifest yet)
ORDER = [("s2pilot", 2, False, 60), ("s3pilot", 3, False, 15), ("s2main", 2, True, 750),
         ("s3main", 3, True, 150), ("s3dup", 3, False, 6)]
ALLOW_MIN = {"freeze_and_commit": 15, "stage2_analysis": 20, "stage3_analysis": 20, "final_report": 30}
MAIN_TARGET = {2: 750, 3: 150}


def kst(ts: float) -> str:
    return datetime.fromtimestamp(ts, KST).strftime("%Y-%m-%d %H:%M")


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def manifest_ids(d: Path):
    mf = d / "manifest.tsv"
    if not mf.exists():
        return None
    return [ln.split("\t")[0] for ln in mf.read_text().splitlines() if ln and not ln.startswith("#")]


def batch_state(suffix):
    d = PROJECT_ROOT / f"{PREFIX}_{suffix}"
    ids = manifest_ids(d)
    done = []
    for t in sorted((d / "trials").glob("*/DONE")) if d.exists() else []:
        try:
            x = json.loads(t.read_text())
        except Exception:      # noqa: BLE001
            continue
        fin = datetime.fromisoformat(x["finished"]).replace(tzinfo=KST).timestamp()
        done.append({"id": x["trial_id"], "wall": float(x.get("wall_s") or 0.0), "end": fin,
                     "attempts": len(x.get("attempts") or []), "result": x.get("result"),
                     "invalid": bool(x.get("infra_failure")) or x.get("result") in (None, "INVALID", "NO_ROW"),
                     "timeout": x.get("result") == "TIMEOUT"})
    done.sort(key=lambda r: r["end"])
    return {"dir": d, "ids": ids, "done": done}


def gaps(done):
    out = []
    for a, b in zip(done, done[1:]):
        g = (b["end"] - b["wall"]) - a["end"]
        if 0 <= g < 600:          # a longer gap is a pause between runs, not runner overhead
            out.append(g)
    return out


def compute():
    now = time.time()
    B = {s: batch_state(s) for s, *_ in ORDER}
    per_stage = {2: [], 3: []}
    gap_s = {2: [], 3: []}
    for s, st, _m, _n in ORDER:
        per_stage[st] += [r["wall"] for r in B[s]["done"]]
        gap_s[st] += gaps(B[s]["done"])
    stats = {}
    for st in (2, 3):
        w, g = per_stage[st], gap_s[st]
        stats[st] = None if not w else {
            "n_samples": len(w), "mean_s": round(statistics.mean(w), 1), "median_s": round(statistics.median(w), 1),
            "p25_s": round(pct(w, 0.25), 1), "p90_s": round(pct(w, 0.90), 1), "min_s": min(w), "max_s": max(w),
            "gap_median_s": round(statistics.median(g), 1) if g else 5.0,
            "gap_p90_s": round(pct(g, 0.90), 1) if g else 10.0, "gap_measured": bool(g)}
    main_live = {}
    for s, st, m, _n in ORDER:
        if m and len(B[s]["done"]) >= 30:
            d = B[s]["done"]
            el = d[-1]["end"] - (d[0]["end"] - d[0]["wall"])
            main_live[st] = round(len(d) / (el / 3600.0), 2)
    # remaining work, in campaign order
    scen = {}
    for name in ("optimistic", "central", "conservative"):
        t = now
        ends = {}
        ok = True
        for s, st, m, nominal in ORDER:
            ids = B[s]["ids"]
            total = len(ids) if ids is not None else nominal
            rem = max(0, total - len(B[s]["done"]))
            S = stats[st]
            if rem and S is None:
                ok = False
                ends[s] = None
                continue
            if rem:
                per = {"optimistic": S["p25_s"] + S["gap_median_s"],
                       "central": S["mean_s"] + S["gap_median_s"],
                       "conservative": (S["p90_s"] + S["gap_p90_s"]) * 1.05}[name]
                if name == "central" and m and st in main_live:
                    per = 3600.0 / main_live[st]
                t += rem * per
            if s == "s3pilot":
                t += ALLOW_MIN["freeze_and_commit"] * 60
            if s == "s2main":
                t += ALLOW_MIN["stage2_analysis"] * 60 if rem else 0
            if s == "s3main":
                t += ALLOW_MIN["stage3_analysis"] * 60 if rem else 0
            ends[s] = t if ok else None
        final = (t + ALLOW_MIN["final_report"] * 60) if ok else None
        scen[name] = {"stage2_main_end": ends.get("s2main"), "stage3_main_end": ends.get("s3main"),
                      "all_runs_end": ends.get("s3dup"), "final_analysis_end": final}
    prog = {}
    for st in (2, 3):
        s = "s2main" if st == 2 else "s3main"
        d = B[s]["done"]
        valid = [r for r in d if not r["invalid"]]
        prog[st] = {"main_target": MAIN_TARGET[st], "main_done": len(d), "main_valid": len(valid),
                    "main_invalid": len(d) - len(valid), "main_retried": sum(r["attempts"] > 1 for r in d),
                    "main_timeouts": sum(r["timeout"] for r in d),
                    "main_remaining": MAIN_TARGET[st] - len(valid),
                    "main_started_kst": kst(d[0]["end"] - d[0]["wall"]) if d else None,
                    "live_trials_per_hour": main_live.get(st)}
    batches = {s: {"manifest": len(B[s]["ids"]) if B[s]["ids"] is not None else None, "done": len(B[s]["done"]),
                   "invalid": sum(r["invalid"] for r in B[s]["done"])} for s, *_ in ORDER}
    return now, stats, prog, scen, batches


def hw():
    def run(c):
        try:
            return subprocess.run(c, capture_output=True, text=True, timeout=10).stdout.strip()
        except Exception:      # noqa: BLE001
            return None
    return {"gpu": run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"]),
            "cpu": (run(["bash", "-c", "lscpu | grep 'Model name' | sed 's/.*: *//'"]) or ""),
            "workers": 1, "mode": "Isaac Sim 5.1 + ROS 2 Jazzy + Nav2, one trial at a time (run_scenario.sh per trial)"}


def write(now, stats, prog, scen, batches, note=""):
    STATUS.mkdir(parents=True, exist_ok=True)
    tsn = datetime.fromtimestamp(now, KST)
    total_main = sum(p["main_target"] for p in prog.values())
    valid_main = sum(p["main_valid"] for p in prog.values())
    js = {"generated_kst": tsn.strftime("%Y-%m-%d %H:%M:%S"), "hardware": hw(), "note": note,
          "progress_percent": round(100.0 * valid_main / total_main, 1), "main_valid": valid_main,
          "main_target": total_main, "stages": {str(k): v for k, v in prog.items()},
          "wall_time_stats": {str(k): v for k, v in stats.items()}, "batches": batches,
          "eta_kst": {n: {k: (kst(v) if v else None) for k, v in d.items()} for n, d in scen.items()},
          "eta_remaining_h": {n: {k: (round((v - now) / 3600, 2) if v else None) for k, v in d.items()}
                              for n, d in scen.items()},
          "method": __doc__.split("Only measured")[1].strip()}
    (STATUS / f"progress_{tsn.strftime('%Y%m%d_%H%M%S')}.json").write_text(json.dumps(js, indent=1))
    L = [f"# ETA_REPORT — stage23_900 (generated {js['generated_kst']} KST)", "",
         "**Estimate from measured wall-clock time, not a guarantee.** One Isaac Sim worker, trials in sequence.",
         f"Hardware: {js['hardware']['gpu']}; {js['hardware']['cpu']}.", ""]
    if note:
        L += [note, ""]
    L += [f"**MAIN progress {js['progress_percent']} %** ({valid_main}/{total_main} valid)", "",
          "| stage | target | done | valid | invalid | retried | timeouts | remaining | main started | live trials/h |",
          "|---|---:|---:|---:|---:|---:|---:|---:|---|---:|"]
    for st, p in prog.items():
        L.append(f"| Stage {st} | {p['main_target']} | {p['main_done']} | {p['main_valid']} | {p['main_invalid']} | "
                 f"{p['main_retried']} | {p['main_timeouts']} | {p['main_remaining']} | {p['main_started_kst'] or '—'} | "
                 f"{p['live_trials_per_hour'] or '—'} |")
    L += ["", "Measured wall time per launch (all launches so far, pilot + main, every outcome):", "",
          "| stage | samples | mean s | median s | P25 s | P90 s | min–max s | runner gap median / P90 s | trials/h (mean) |",
          "|---|---:|---:|---:|---:|---:|---|---|---:|"]
    for st, S in stats.items():
        if S is None:
            L.append(f"| Stage {st} | 0 | — | — | — | — | — | — | — (no measured sample: ETA not computed) |")
        else:
            L.append(f"| Stage {st} | {S['n_samples']} | {S['mean_s']} | {S['median_s']} | {S['p25_s']} | {S['p90_s']} | "
                     f"{S['min_s']:.0f}–{S['max_s']:.0f} | {S['gap_median_s']} / {S['gap_p90_s']}"
                     f"{'' if S['gap_measured'] else ' (assumed)'} | "
                     f"{round(3600 / (S['mean_s'] + S['gap_median_s']), 1)} |")
    L += ["", "| scenario | Stage 2 MAIN end | Stage 3 MAIN end | all runs (incl. duplicate control) | final analysis |",
          "|---|---|---|---|---|"]
    for n, d in js["eta_kst"].items():
        h = js["eta_remaining_h"][n]
        L.append(f"| {n} | {d['stage2_main_end'] or '—'} KST | {d['stage3_main_end'] or '—'} KST | "
                 f"{d['all_runs_end'] or '—'} KST | **{d['final_analysis_end'] or '—'} KST** "
                 f"({h['final_analysis_end']} h) |")
    L += ["", "Batches:", "", "| batch | manifest | done | invalid |", "|---|---:|---:|---:|"]
    for s, b in batches.items():
        L.append(f"| {s} | {b['manifest'] or '—'} | {b['done']} | {b['invalid']} |")
    L += ["", "Method:", "", "```", js["method"], "```", ""]
    (STATUS / "ETA_REPORT.md").write_text("\n".join(L) + "\n")
    return js


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--note", default="")
    a = ap.parse_args()
    js = write(*compute(), note=a.note)
    print(json.dumps({k: js[k] for k in ("generated_kst", "progress_percent", "eta_kst")}, indent=1))
    if not a.watch:
        return 0
    last = None
    while True:
        now, stats, prog, scen, batches = compute()
        n = sum(p["main_done"] for p in prog.values())
        key = (n // 30, tuple((b["done"] == b["manifest"]) for b in batches.values()))
        if key != last:
            write(now, stats, prog, scen, batches, note=a.note)
            last = key
        if all(b["manifest"] and b["done"] >= b["manifest"] for b in batches.values()):
            write(now, stats, prog, scen, batches, note=a.note)
            return 0
        time.sleep(60)


if __name__ == "__main__":
    sys.exit(main())
