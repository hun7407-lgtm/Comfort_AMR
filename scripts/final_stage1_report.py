#!/usr/bin/env python3
"""Stage 1 (final, rev. 2026-11-25): tables, figures and R_nav_upper.

    python3 scripts/final_stage1_report.py                  # coarse + fine, whatever is DONE
    python3 scripts/final_stage1_report.py --partial         # allowed while trials still run

Plain Python 3 + numpy + matplotlib. Reads
results/batches/261125_final_stage1/{coarse,fine}/trials/*/ and writes
results/batches/261125_final_stage1/analysis/ (per-trial JSON cache,
stage1_upper_bound.json, STAGE1_REPORT.md, tables, figures).

Decision rule (docs/FINAL_261125_PLAN.md §4, fixed before the data):
    transition  = first size whose comfort-induced deadlock rate > 5 % (5/100; 1/30 at 30 seeds)
    R_nav_upper = largest size with comfort-induced deadlock rate <= 5 % AND
                  system success not more than 5 %p below the same seeds' B1
Rates always as count / denominator with a Wilson 95 % CI.
"""

from __future__ import annotations

import argparse
import json
import statistics as st
import sys
from collections import Counter, defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.final_metrics import COMFORT_CLASSES, SUCCESS_CLASSES, analyse_trial, wilson  # noqa: E402

BATCH = PROJECT_ROOT / "results" / "batches" / "261125_final_stage1"
OUT = BATCH / "analysis"
THRESH = 0.05
AMRS = ("AMR1", "a1", "a2")


def size_of(cond: str):
    return None if cond == "B1" else int(cond.split("_r")[1]) / 100.0


def analyse_all(dirs):
    cache = OUT / "trials"
    cache.mkdir(parents=True, exist_ok=True)
    recs = []
    for d in dirs:
        for t in sorted((d / "trials").glob("*/DONE")):
            tdir = t.parent
            c = cache / f"{tdir.name}.json"
            if c.exists() and c.stat().st_mtime >= t.stat().st_mtime:
                r = json.loads(c.read_text())
            else:
                r = analyse_trial(tdir)
                r["phase"] = d.name
                r["trial_dir"] = str(tdir.relative_to(PROJECT_ROOT))
                c.write_text(json.dumps(r))
            recs.append(r)
    return recs


def rate(k, n):
    lo, hi = wilson(k, n)
    return {"count": k, "n": n, "rate": round(k / n, 4) if n else None, "ci95": [round(lo, 4), round(hi, 4)]}


def fmt(r):
    if not r["n"]:
        return "–"
    return f"{r['count']}/{r['n']} ({100 * r['rate']:.0f}%, {100 * r['ci95'][0]:.0f}–{100 * r['ci95'][1]:.0f})"


def med(xs):
    xs = [x for x in xs if x is not None]
    return round(st.median(xs), 3) if xs else None


def mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 3) if xs else None


def summarise(recs):
    by = defaultdict(list)
    for r in recs:
        if r.get("infra_failure"):
            continue
        by[r["condition"]].append(r)
    b1 = {r["seed"]: r for r in by.get("B1", [])}
    rows = {}
    for cond, rs in sorted(by.items(), key=lambda kv: (-1 if kv[0] == "B1" else size_of(kv[0]))):
        n = len(rs)
        paired = [r for r in rs if r["seed"] in b1]
        b1_ok = [r for r in paired if b1[r["seed"]]["system_success"]]
        row = {
            "condition": cond, "size_m": size_of(cond), "n": n,
            "system_success": rate(sum(r["system_success"] for r in rs), n),
            "comfort_deadlock": rate(sum(r["comfort_deadlock"] for r in rs), n),
            "comfort_deadlock_paired_b1_success": rate(sum(r["comfort_deadlock"] for r in b1_ok), len(b1_ok)),
            "all_cause_failure": rate(sum(not r["system_success"] for r in rs), n),
            "b1_system_success_same_seeds": rate(sum(b1[r["seed"]]["system_success"] for r in paired), len(paired)),
            "per_amr_success": {a: rate(sum(r["amrs"].get(a, {}).get("class") in SUCCESS_CLASSES for r in rs), n)
                                for a in AMRS},
            "classes": dict(Counter(r["amrs"][a]["class"] for r in rs for a in AMRS if a in r["amrs"])),
            "amr1_travel_time_success_median_s": med([r["amrs"]["AMR1"]["travel_time"] for r in rs
                                                      if r["amrs"]["AMR1"]["class"] in SUCCESS_CLASSES]),
            "amr1_path_length_success_median_m": med([r["amrs"]["AMR1"]["path_length"] for r in rs
                                                      if r["amrs"]["AMR1"]["class"] in SUCCESS_CLASSES]),
            "hold_s_per_trial_median": med([sum((r["amrs"][a].get("hold_s") or 0) for a in r["amrs"]) for r in rs]),
            "amr1_gate_yield_s_median": med([r["amrs"]["AMR1"].get("gate_yield_s") for r in rs]),
            "min_human_amr_m_median": med([min([r["amrs"][a]["min_human_m"] for a in r["amrs"]
                                                if r["amrs"][a]["min_human_m"] is not None] or [None]) for r in rs]),
            "overlap_max_m2_median": med([r["spatial"]["overlap_max_m2"] for r in rs]),
            "overlap_s_median": med([r["spatial"]["overlap_s"] for r in rs]),
            "corridor_intrusion_mean_m2_median": med([r["spatial"]["corridor_intrusion_mean_m2"] for r in rs]),
            "residual_width_min_m_median": med([min([r["amrs"][a]["residual_width_min_m"] for a in r["amrs"]
                                                     if r["amrs"][a]["residual_width_min_m"] is not None] or [None])
                                                for r in rs]),
            "replans_per_trial_mean": mean([sum(r["amrs"][a].get("replans") or 0 for a in r["amrs"]) for r in rs]),
            "protective_stop_trials": rate(sum(any(r["amrs"][a]["class"] == "PROTECTIVE_STOP" for a in r["amrs"])
                                               for r in rs), n),
        }
        rows[cond] = row
    return rows


def decide(rows):
    sizes = sorted((r for r in rows.values() if r["size_m"] is not None), key=lambda r: r["size_m"])
    transition = next((r["size_m"] for r in sizes if r["comfort_deadlock"]["rate"] > THRESH), None)
    ok = [r for r in sizes if r["comfort_deadlock"]["rate"] <= THRESH and r["system_success"]["n"]
          and r["b1_system_success_same_seeds"]["n"]
          and r["system_success"]["rate"] >= r["b1_system_success_same_seeds"]["rate"] - 0.05]
    upper = max((r["size_m"] for r in ok), default=None)
    below = [r["size_m"] for r in sizes if transition is not None and r["size_m"] < transition]
    return {"transition_size_m": transition, "R_nav_upper_m": upper,
            "fine_interval": [max(below) if below else None, transition] if transition else None,
            "rule": "transition = first size with comfort-induced deadlock rate > 5 % (5/100; 1/30 at 30 seeds); R_nav_upper = largest size "
                    "with comfort-induced deadlock rate <= 5 % and system success >= same-seed B1 - 5 %p",
            "sizes_tested": [r["size_m"] for r in sizes]}


def paired_transitions(recs):
    seq = defaultdict(dict)
    for r in recs:
        if not r.get("infra_failure"):
            seq[r["seed"]][r["condition"]] = r
    return seq


def figures(rows, recs, out: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out.mkdir(parents=True, exist_ok=True)
    sizes = sorted((r for r in rows.values() if r["size_m"] is not None), key=lambda r: r["size_m"])
    if not sizes:
        return []
    x = [r["size_m"] for r in sizes]
    b1 = rows.get("B1")
    made = []

    def rate_plot(key, title, fname, ylabel):
        fig, ax = plt.subplots(figsize=(6.4, 4.0), dpi=150)
        y = [100 * r[key]["rate"] for r in sizes]
        lo = [100 * (r[key]["rate"] - r[key]["ci95"][0]) for r in sizes]
        hi = [100 * (r[key]["ci95"][1] - r[key]["rate"]) for r in sizes]
        ax.errorbar(x, y, yerr=[lo, hi], marker="o", color="#1f5fa8", capsize=4, lw=2, label="B2 Static")
        if b1 and key in b1 and b1[key]["n"]:
            ax.axhline(100 * b1[key]["rate"], color="#888", ls="--", lw=1.5, label="B1 (no Comfort Space)")
        if key == "comfort_deadlock":
            ax.axhline(100 * THRESH, color="#c0392b", ls=":", lw=1.2, label="5 % rule")
        for xi, r in zip(x, sizes):
            ax.annotate(f"{r[key]['count']}/{r[key]['n']}", (xi, 100 * r[key]["rate"]), textcoords="offset points",
                        xytext=(0, 8), ha="center", fontsize=8, color="#333")
        ax.set_xlabel("Static Comfort Space radius r (m)")
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=11)
        ax.set_ylim(-3, 105)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out / fname)
        plt.close(fig)
        made.append(fname)

    rate_plot("system_success", "Stage 1: system mission success (3 AMRs)", "S1_size_vs_system_success.png", "%")
    rate_plot("comfort_deadlock", "Stage 1: comfort-induced deadlock", "S1_size_vs_comfort_deadlock.png", "% of trials")

    def box(field_fn, title, fname, ylabel):
        fig, ax = plt.subplots(figsize=(6.4, 4.0), dpi=150)
        data, labels = [], []
        for cond in (["B1"] if "B1" in rows else []) + [r["condition"] for r in sizes]:
            vals = [v for v in (field_fn(r) for r in recs if r["condition"] == cond and not r.get("infra_failure"))
                    if v is not None]
            data.append(vals or [float("nan")])
            labels.append("B1" if cond == "B1" else f"{size_of(cond):.2f}")
        ax.boxplot(data, labels=labels, showfliers=True, flierprops={"markersize": 3})
        ax.set_xlabel("condition (B1, or Static radius r in m)")
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=11)
        ax.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        fig.savefig(out / fname)
        plt.close(fig)
        made.append(fname)

    box(lambda r: r["amrs"]["AMR1"]["travel_time"] if r["amrs"]["AMR1"]["class"] in SUCCESS_CLASSES else None,
        "AMR 1 travel time (successful trials)", "S1_size_vs_travel_time.png", "s")
    box(lambda r: sum((a.get("hold_s") or 0) for a in r["amrs"].values()),
        "Standing (yield/hold) time, sum of 3 AMRs", "S1_size_vs_hold_time.png", "s")
    box(lambda r: r["spatial"]["overlap_max_m2"], "Max Comfort Space overlap on the floor",
        "S1_size_vs_overlap.png", "m² (≥ 2 spaces)")
    box(lambda r: min([a["min_human_m"] for a in r["amrs"].values() if a["min_human_m"] is not None] or [None]),
        "Min human–AMR clearance (centre → footprint)", "S1_size_vs_clearance.png", "m")
    box(lambda r: min([a["residual_width_min_m"] for a in r["amrs"].values()
                       if a["residual_width_min_m"] is not None] or [None]),
        "Narrowest residual free width ahead of an AMR", "S1_size_vs_residual_width.png", "m")
    return made


def write_report(rows, dec, recs, made, partial):
    L = ["# Stage 1 (final) — Navigation Upper Bound, B2 Static size sweep", "",
         "> Cross scenario (8 people, AMR 1/2/3), frozen navigation (buffer 0.10 m), every AMR on Nav2 only. "
         f"Seeds {min(r['seed'] for r in recs)}–{max(r['seed'] for r in recs)} "
         f"({len({r['seed'] for r in recs})} seeds; reduced from the planned 100 by user decision 2026-11-25), "
         "same seed = same people / missions / timing; only the Comfort Space differs. "
         "Rates are count / n with a Wilson 95 % CI." + (" **PARTIAL: trials still running.**" if partial else ""),
         "", "## Summary", "",
         "| condition | n | system success | comfort-induced deadlock | same, B1-success seeds | all-cause failure | "
         "B1 same seeds | AMR1 / a1 / a2 success | AMR1 travel s (median, success) | hold s / trial (median) | "
         "min human–AMR m (median) | overlap m² (median max) | residual width m (median min) |",
         "|" + "---|" * 13]
    for r in rows.values():
        L.append(f"| {r['condition']} | {r['n']} | {fmt(r['system_success'])} | {fmt(r['comfort_deadlock'])} | "
                 f"{fmt(r['comfort_deadlock_paired_b1_success'])} | {fmt(r['all_cause_failure'])} | "
                 f"{fmt(r['b1_system_success_same_seeds'])} | "
                 + " / ".join(f"{r['per_amr_success'][a]['count']}" for a in AMRS) + f" of {r['n']} | "
                 f"{r['amr1_travel_time_success_median_s']} | {r['hold_s_per_trial_median']} | "
                 f"{r['min_human_amr_m_median']} | {r['overlap_max_m2_median']} | {r['residual_width_min_m_median']} |")
    L += ["", "## Failure classes (per AMR, all trials)", "",
          "| condition | " + " | ".join(sorted({c for r in rows.values() for c in r["classes"]})) + " |",
          "|---|" + "---|" * len({c for r in rows.values() for c in r["classes"]})]
    allc = sorted({c for r in rows.values() for c in r["classes"]})
    for r in rows.values():
        L.append(f"| {r['condition']} | " + " | ".join(str(r["classes"].get(c, 0)) for c in allc) + " |")
    L += ["", "## Decision (pre-registered rule)", "", f"- rule: {dec['rule']}",
          f"- transition size: {dec['transition_size_m']}", f"- fine-search interval: {dec['fine_interval']}",
          f"- **R_nav_upper = {dec['R_nav_upper_m']} m**" + (" (provisional, data incomplete)" if partial else ""),
          "- operational upper bound of THIS scenario and navigation stack; not a mathematical maximum and not a "
          "comfort distance.", "", "## Figures", ""] + [f"![{m}](figures/{m})" for m in made]
    (OUT / "STAGE1_REPORT.md").write_text("\n".join(L) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--partial", action="store_true")
    args = ap.parse_args()
    dirs = [d for d in (BATCH / "coarse", BATCH / "fine") if (d / "trials").exists()]
    recs = analyse_all(dirs)
    rows = summarise(recs)
    dec = decide(rows)
    made = figures(rows, recs, OUT / "figures")
    seq = paired_transitions(recs)
    out = {"source": "results/batches/261125_final_stage1", "partial": args.partial, "decision": dec,
           "conditions": rows,
           "paired_seed_classes": {s: {c: [r["amrs"][a]["class"] for a in AMRS if a in r["amrs"]]
                                       for c, r in v.items()} for s, v in sorted(seq.items())}}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stage1_upper_bound.json").write_text(json.dumps(out, indent=1))
    write_report(rows, dec, recs, made, args.partial)
    print(json.dumps(dec, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
