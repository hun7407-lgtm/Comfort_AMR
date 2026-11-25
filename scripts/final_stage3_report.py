#!/usr/bin/env python3
"""Stage 3 (final, rev. 2026-11-25): B1 / B2 / B3-O on holdout seeds 101-200.

    python3 scripts/final_stage3_report.py [--partial]

Plain Python 3 + numpy + scipy + matplotlib. Reads
results/batches/261125_final_stage3/{main,sensitivity}/trials/*/ and
stage3_selection.json; writes results/batches/261125_final_stage3/analysis/
(per-trial cache, stage3_results.json, STAGE3_REPORT.md, figures/).

System success = all three AMRs reached their mission (SUCCESS / NORMAL_YIELD /
TEMPORARY_HOLD). Paired tests on the same seeds: exact McNemar for binary
outcomes, Wilcoxon signed-rank for continuous ones (B2 vs B3-O, B1 vs B3-O).
"""

from __future__ import annotations

import argparse
import json
import statistics as st
import sys
from collections import Counter
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.final_metrics import COMFORT_CLASSES, SUCCESS_CLASSES, analyse_trial, wilson  # noqa: E402

S3 = PROJECT_ROOT / "results" / "batches" / "261125_final_stage3"
OUT = S3 / "analysis"
AMRS = ("AMR1", "a1", "a2")


def label(cond):
    if cond == "B1":
        return "B1"
    if cond.startswith("B2"):
        return "B2"
    return {"B3O_MID": "B3-O", "B3O_LOWER": "B3-O LOWER", "B3O_UPPER": "B3-O UPPER"}.get(cond, cond)


def load(dirs):
    cache = OUT / "trials"
    cache.mkdir(parents=True, exist_ok=True)
    out = []
    for d in dirs:
        for t in sorted((d / "trials").glob("*/DONE")):
            c = cache / f"{t.parent.name}.json"
            if c.exists() and c.stat().st_mtime >= t.stat().st_mtime:
                r = json.loads(c.read_text())
            else:
                r = analyse_trial(t.parent)
                r["phase"] = d.name
                c.write_text(json.dumps(r))
            if not r.get("infra_failure"):
                out.append(r)
    return out


def rate(k, n):
    lo, hi = wilson(k, n)
    return {"count": k, "n": n, "rate": round(k / n, 4) if n else None, "ci95": [round(lo, 4), round(hi, 4)]}


def fmt(r):
    return "–" if not r["n"] else f"{r['count']}/{r['n']} ({100 * r['rate']:.0f}%, {100 * r['ci95'][0]:.0f}–{100 * r['ci95'][1]:.0f})"


def med(xs):
    xs = [x for x in xs if x is not None]
    return round(st.median(xs), 3) if xs else None


def per_trial(r):
    a = r["amrs"]
    inter = r["interactions"]
    return {
        "system": r["system_success"], "comfort": r["comfort_deadlock"], "fail": not r["system_success"],
        "robot_robot": any(x["class"] == "ROBOT_ROBOT_BLOCK" for x in a.values()),
        "protective_stop": any((x.get("protective_stop_s") or 0) > 0 for x in a.values()),
        "contact": any((x.get("contact_s") or 0) > 0 for x in a.values()),
        "amr1_travel": a["AMR1"]["travel_time"] if a["AMR1"]["class"] in SUCCESS_CLASSES else None,
        "amr1_path": a["AMR1"]["path_length"] if a["AMR1"]["class"] in SUCCESS_CLASSES else None,
        "yield_s": a["AMR1"].get("gate_yield_s"),
        "hold_s": sum((x.get("hold_s") or 0) for x in a.values()),
        "replans": sum((x.get("replans") or 0) for x in a.values()),
        "min_human": min([x["min_human_m"] for x in a.values() if x["min_human_m"] is not None] or [None]),
        "penetration_max": max([e["penetration"] for e in inter] or [0.0]),
        "intrusion_s": sum(e["intrusion_s"] for e in inter),
        "overlap_m2": r["spatial"].get("overlap_max_m2"),
        "corridor_m2": r["spatial"].get("corridor_intrusion_mean_m2"),
    }


def mcnemar(a, b):
    from scipy.stats import binomtest
    n01 = sum(1 for x, y in zip(a, b) if not x and y)
    n10 = sum(1 for x, y in zip(a, b) if x and not y)
    p = binomtest(n01, n01 + n10, 0.5).pvalue if n01 + n10 else 1.0
    return {"only_second": n01, "only_first": n10, "p_exact": round(p, 4)}


def wilcoxon(a, b):
    from scipy.stats import wilcoxon as w
    pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    if len(pairs) < 6 or all(abs(x - y) < 1e-9 for x, y in pairs):
        return {"n": len(pairs), "p": None}
    s = w([x for x, _ in pairs], [y for _, y in pairs])
    return {"n": len(pairs), "median_diff": round(st.median([y - x for x, y in pairs]), 3), "p": round(s.pvalue, 4)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--partial", action="store_true")
    args = ap.parse_args()
    recs = load([d for d in (S3 / "main", S3 / "sensitivity") if (d / "trials").exists()])
    by = {}
    for r in recs:
        by.setdefault(label(r["condition"]), {})[r["seed"]] = r
    rows = {}
    for pol, seeds in by.items():
        P = [per_trial(r) for r in seeds.values()]
        n = len(P)
        rows[pol] = {
            "n": n, "system_success": rate(sum(p["system"] for p in P), n),
            "per_amr_success": {a: rate(sum(r["amrs"].get(a, {}).get("class") in SUCCESS_CLASSES
                                            for r in seeds.values()), n) for a in AMRS},
            "comfort_deadlock": rate(sum(p["comfort"] for p in P), n),
            "all_failure": rate(sum(p["fail"] for p in P), n),
            "robot_robot_deadlock": rate(sum(p["robot_robot"] for p in P), n),
            "protective_stop_trials": rate(sum(p["protective_stop"] for p in P), n),
            "contact_trials": rate(sum(p["contact"] for p in P), n),
            **{f"{k}_median": med([p[k] for p in P]) for k in
               ("amr1_travel", "amr1_path", "yield_s", "hold_s", "replans", "min_human", "penetration_max",
                "intrusion_s", "overlap_m2", "corridor_m2")},
            "classes": dict(Counter(r["amrs"][a]["class"] for r in seeds.values() for a in AMRS if a in r["amrs"])),
        }
    tests = {}
    for x, y in (("B2", "B3-O"), ("B1", "B3-O"), ("B1", "B2")):
        if x in by and y in by:
            common = sorted(set(by[x]) & set(by[y]))
            px = [per_trial(by[x][s]) for s in common]
            py = [per_trial(by[y][s]) for s in common]
            tests[f"{x} vs {y}"] = {"paired_seeds": len(common),
                                    "system_success": mcnemar([p["system"] for p in px], [p["system"] for p in py]),
                                    "comfort_deadlock": mcnemar([p["comfort"] for p in px], [p["comfort"] for p in py]),
                                    **{k: wilcoxon([p[k] for p in px], [p[k] for p in py])
                                       for k in ("amr1_travel", "hold_s", "yield_s", "min_human", "overlap_m2",
                                                 "penetration_max")}}
    sel = json.loads((S3 / "stage3_selection.json").read_text()) if (S3 / "stage3_selection.json").exists() else {}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stage3_results.json").write_text(json.dumps({"partial": args.partial, "selection": sel, "policies": rows,
                                                         "paired_tests": tests}, indent=1))
    order = [p for p in ("B1", "B2", "B3-O", "B3-O LOWER", "B3-O UPPER") if p in rows]
    sd = sorted({r["seed"] for r in recs})
    L = [f"# Stage 3 (final) — B1 / B2 / B3-O on holdout seeds {sd[0]}–{sd[-1]} ({len(sd)} seeds)" if sd else
         "# Stage 3 (final)", "",
         "> Same seed = same people, missions, timing, navigation and safety layer; only the Comfort Space differs. "
         "B3-O sizes from R_nav_upper (Stage 1) and the Simulation-derived Provisional Lower Bound (Stage 2 proxy, "
         "not a human study). Rates: count / n (Wilson 95 % CI)." + (" **PARTIAL.**" if args.partial else ""), "",
         f"R_nav_upper = {sel.get('R_nav_upper_m')} m; B2 static = {sel.get('B2_static_m')} m.", "",
         "| Policy | System Success | Comfort Deadlock | All Failure | Travel Time (AMR 1, median s) | "
         "Yield Time (AMR 1 gate, median s) | Min Human Distance (median m) |", "|---|---:|---:|---:|---:|---:|---:|"]
    for p in order:
        r = rows[p]
        L.append(f"| {p} | {fmt(r['system_success'])} | {fmt(r['comfort_deadlock'])} | {fmt(r['all_failure'])} | "
                 f"{r['amr1_travel_median']} | {r['yield_s_median']} | {r['min_human_median']} |")
    L += ["", "## More metrics (medians per trial)", "",
          "| Policy | AMR1 / a1 / a2 success | robot-robot deadlock | hold s (3 AMRs) | replans | path m (AMR 1) | "
          "penetration max m | intrusion s | overlap m² | corridor m² | protective stop trials | contact trials |",
          "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|"]
    for p in order:
        r = rows[p]
        L.append(f"| {p} | " + " / ".join(str(r["per_amr_success"][a]["count"]) for a in AMRS) + f" of {r['n']} | "
                 f"{fmt(r['robot_robot_deadlock'])} | {r['hold_s_median']} | {r['replans_median']} | "
                 f"{r['amr1_path_median']} | {r['penetration_max_median']} | {r['intrusion_s_median']} | "
                 f"{r['overlap_m2_median']} | {r['corridor_m2_median']} | {fmt(r['protective_stop_trials'])} | "
                 f"{fmt(r['contact_trials'])} |")
    L += ["", "## Paired tests (same seeds)", ""]
    for k, v in tests.items():
        L.append(f"- **{k}** (n = {v['paired_seeds']}): system success McNemar only-{k.split(' vs ')[0]} "
                 f"{v['system_success']['only_first']} / only-{k.split(' vs ')[1]} {v['system_success']['only_second']}, "
                 f"p = {v['system_success']['p_exact']}; travel {v['amr1_travel']}; hold {v['hold_s']}; "
                 f"min human {v['min_human']}")
    L += ["", "## Failure classes (per AMR)", ""]
    for p in order:
        L.append(f"- {p}: " + ", ".join(f"{c} {k}" for c, k in sorted(rows[p]["classes"].items())))
    (OUT / "STAGE3_REPORT.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[:12 + len(order)]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
