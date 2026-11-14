#!/usr/bin/env python3
"""Stage 3 report: B3-O Initial vs B3-O Optimized (+ Static baseline).

    python3 scripts/stage3_report.py --dir results/batches/261215_stage3_validation
    python3 scripts/stage3_report.py --dir ... --overlay-seeds 5 12 25

Reads <dir>/trials.csv, fleet_trials.csv and the per-trial JSON (which,
for trials run after 2026-11-12, holds the driven trajectory and the
first Nav2 plan) and writes

    <dir>/REPORT.md, tables.md, comparison.json
    <dir>/figures/V1_summary.png            success / comfort deadlock / travel time /
                                            distance / clearance / intrusion per condition
    <dir>/figures/V2_paired_deltas.png      per-seed Optimized - Initial, time and distance
    <dir>/figures/V3_mechanisms.png         how each condition's failures happened
    <dir>/figures/V4_trajectories_s<seed>.png   driven routes and first plans overlaid,
                                            one file per requested seed (default: the
                                            seeds where the two B3-O outcomes differ,
                                            plus the three longest Initial detours)

Conditions are recognised by scenario name: static_baseline, b3o_initial,
b3o_optimized. Rows with other names are ignored and counted. Nothing is
inferred for a missing trial.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from comfort_core import load_config                      # noqa: E402
from comfort_core.layout import human_positions           # noqa: E402
from report_pilot import f, mean, truthy                  # noqa: E402
from stage1_report import (MECHS, MECH_COLOUR, MECH_LABEL, _sd, _stamp_s,  # noqa: E402
                           mechanism_of)

CONDS = ["static_baseline", "b3o_initial", "b3o_optimized"]
LABEL = {"static_baseline": "Static baseline", "b3o_initial": "B3-O Initial",
         "b3o_optimized": "B3-O Optimized"}
COLOUR = {"static_baseline": "#d55e00", "b3o_initial": "#56b4e9", "b3o_optimized": "#0072b2"}


def read(path: Path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as h:
        return list(csv.DictReader(h))


def metrics(rs, frs_by_stamp):
    v = [r for r in rs if truthy(r.get("valid", "True"))]
    s = [r for r in v if r["result"] == "SUCCESS"]
    mech = Counter(r["_mech"] for r in rs if r["_mech"])
    gaps = []
    for r in v:
        t0 = _stamp_s(r["stamp"])
        g = [f(fr.get("min_gap_amr1_m")) for (st, cfg), frs in frs_by_stamp.items()
             if cfg == r["config"] and abs(_stamp_s(st) - t0) < 240 for fr in frs]
        g = [x for x in g if not math.isnan(x)]
        if g:
            gaps.append(min(g))
    return {
        "n": len(rs), "n_valid": len(v), "n_success": len(s),
        "success_rate": len(s) / len(v) if v else math.nan,
        "comfort_deadlock_n": mech.get("comfort_gate_hold", 0),
        "other_fail_n": sum(1 for r in v if r["result"] != "SUCCESS") - mech.get("comfort_gate_hold", 0),
        "mechanisms": dict(mech),
        "travel_time_mean": mean([f(r["travel_time"]) for r in s]),
        "travel_time_sd": _sd([f(r["travel_time"]) for r in s]),
        "travel_distance_mean": mean([f(r["travel_distance"]) for r in s]),
        "travel_distance_sd": _sd([f(r["travel_distance"]) for r in s]),
        "planned_path_mean": mean([f(r.get("planned_path_length")) for r in s]),
        "detour_mean": mean([f(r["travel_distance"]) - f(r.get("planned_path_length"))
                             for r in s if r.get("planned_path_length")]),
        "yield_mean": mean([f(r["yield_seconds"]) for r in v]),
        "waited_mean": mean([f(r["waited_for_humans"]) for r in v]),
        "min_clearance_mean": mean([f(r["min_clearance"]) for r in v]),
        "min_clearance_min": min([f(r["min_clearance"]) for r in v], default=math.nan),
        "contact_trials": sum(1 for r in v if f(r["contact_seconds"]) > 0),
        "contact_robot_trials": sum(1 for r in v if f(r["contact_robot_seconds"]) > 0),
        "intrusion_mean": mean([f(r["worst_intrusion"]) for r in v]),
        "intrusion_trials": sum(1 for r in v if f(r["worst_intrusion"]) > 0),
        "comfort_seconds_mean": mean([f(r["comfort_seconds_total"]) for r in v]),
        "robot_gap_min": min(gaps) if gaps else math.nan,
        "robot_gap_mean": mean(gaps),
        "size_m": next((r.get("comfort_size_m") for r in rs if r.get("comfort_size_m")), ""),
    }


def fmt(v, nd=2):
    if isinstance(v, float):
        return "-" if math.isnan(v) else f"{v:.{nd}f}"
    return "-" if v in (None, "") else str(v)


def pct(v):
    return "-" if isinstance(v, float) and math.isnan(v) else f"{100 * v:.0f}%"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--overlay-seeds", nargs="*", type=int, default=None)
    ap.add_argument("--max-overlays", type=int, default=6)
    args = ap.parse_args()
    d = Path(args.dir)
    folder = d.name
    rows = read(d / "trials.csv")
    fleet = read(d / "fleet_trials.csv")
    by: dict = defaultdict(list)
    other = 0
    for r in rows:
        if r["scenario"] not in CONDS:
            other += 1
            continue
        r["_mech"] = mechanism_of(r)
        r["_seed"] = int(r["seed"]) if r.get("seed") else None
        by[r["scenario"]].append(r)
    conds = [c for c in CONDS if by.get(c)]
    if not conds:
        print(f"[stage3_report] no Stage 3 rows in {d} (scenario names {CONDS})")
        return 1
    frs_by_stamp = defaultdict(list)
    for fr in fleet:
        frs_by_stamp[(fr["stamp"], fr["config"])].append(fr)
    M = {c: metrics(by[c], frs_by_stamp) for c in conds}
    figs = d / "figures"
    figs.mkdir(exist_ok=True)

    # paired per seed: optimized - initial
    paired = []
    if "b3o_initial" in by and "b3o_optimized" in by:
        ini = {r["_seed"]: r for r in by["b3o_initial"]}
        opt = {r["_seed"]: r for r in by["b3o_optimized"]}
        for seed in sorted(set(ini) & set(opt)):
            a, b = ini[seed], opt[seed]
            both_ok = a["result"] == "SUCCESS" and b["result"] == "SUCCESS"
            paired.append({"seed": seed, "initial": a["result"], "optimized": b["result"],
                           "initial_mech": a["_mech"], "optimized_mech": b["_mech"],
                           "dt": f(b["travel_time"]) - f(a["travel_time"]) if both_ok else math.nan,
                           "dd": f(b["travel_distance"]) - f(a["travel_distance"]) if both_ok else math.nan,
                           "dclr": f(b["min_clearance"]) - f(a["min_clearance"]),
                           "dintr": f(b["worst_intrusion"]) - f(a["worst_intrusion"])})

    # V1 summary
    panels = [("success_rate", "success rate", pct), ("comfort_deadlock_n", "comfort gate holds (n)", str),
              ("travel_time_mean", "travel time (s, successes)", lambda v: fmt(v, 1)),
              ("travel_distance_mean", "travel distance (m, successes)", fmt),
              ("min_clearance_mean", "min human clearance (m)", fmt),
              ("intrusion_mean", "worst intrusion into ref. space (m)", fmt)]
    fig, axes = plt.subplots(2, 3, figsize=(11, 6.2))
    for ax, (key, title, ff) in zip(axes.flat, panels):
        xs = range(len(conds))
        ys = [M[c][key] for c in conds]
        bars = ax.bar(list(xs), [0 if (isinstance(y, float) and math.isnan(y)) else y for y in ys], 0.6,
                      color=[COLOUR[c] for c in conds], edgecolor="white")
        sd_key = key.replace("_mean", "_sd")
        if sd_key in M[conds[0]]:
            ax.errorbar(list(xs), ys, yerr=[M[c][sd_key] if not math.isnan(M[c][sd_key]) else 0 for c in conds],
                        fmt="none", ecolor="#333333", capsize=3, lw=1)
        for b, y in zip(bars, ys):
            ax.text(b.get_x() + b.get_width() / 2, b.get_height(), ff(y), ha="center", va="bottom",
                    fontsize=8)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([f"{LABEL[c]}\n(n={M[c]['n_valid']})" for c in conds], fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.grid(axis="y", alpha=0.25)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    fig.suptitle(f"{folder}\nV1  Stage 3 validation summary (AMR 1)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(figs / "V1_summary.png", dpi=150)
    plt.close(fig)

    # V2 paired deltas
    if paired:
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for ax, key, lab in ((axes[0], "dt", "travel time Optimized - Initial (s)"),
                             (axes[1], "dd", "travel distance Optimized - Initial (m)")):
            seeds = [p["seed"] for p in paired]
            vals = [p[key] for p in paired]
            cols = ["#0072b2" if not math.isnan(v) and v <= 0 else "#d55e00" for v in vals]
            ax.bar(range(len(seeds)), [0 if math.isnan(v) else v for v in vals], color=cols, edgecolor="white")
            for i, (p, v) in enumerate(zip(paired, vals)):
                if math.isnan(v):
                    ax.text(i, 0, "x", ha="center", va="center", fontsize=8, color="#666666")
            ax.axhline(0, color="#333333", lw=0.8)
            ok = [v for v in vals if not math.isnan(v)]
            ax.set_title(f"{lab}\nmean {mean(ok):+.2f} over {len(ok)} seeds both reached the goal "
                         f"(x: not both)", fontsize=9)
            ax.set_xticks(range(len(seeds)))
            ax.set_xticklabels(seeds, fontsize=7)
            ax.set_xlabel("seed")
            ax.grid(axis="y", alpha=0.25)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
        fig.suptitle(f"{folder}\nV2  per-seed paired difference (blue: Optimized faster/shorter)", fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, 0.9))
        fig.savefig(figs / "V2_paired_deltas.png", dpi=150)
        plt.close(fig)

    # V3 mechanisms
    fig, ax = plt.subplots(figsize=(7, 4))
    bottom = [0.0] * len(conds)
    for m in MECHS:
        vals = [M[c]["mechanisms"].get(m, 0) for c in conds]
        if not any(vals):
            continue
        ax.bar(range(len(conds)), vals, 0.6, bottom=bottom, color=MECH_COLOUR[m], label=MECH_LABEL[m],
               edgecolor="white")
        for i, (b, v) in enumerate(zip(bottom, vals)):
            if v:
                ax.text(i, b + v / 2, str(v), ha="center", va="center", fontsize=8)
        bottom = [b + v for b, v in zip(bottom, vals)]
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels([f"{LABEL[c]}\n(n={M[c]['n']})" for c in conds])
    ax.set_ylabel("AMR 1 trials not reaching the goal")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    fig.suptitle(f"{folder}\nV3  how AMR 1 stopped", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(figs / "V3_mechanisms.png", dpi=150)
    plt.close(fig)

    # V4 trajectory overlays
    jsons = {}
    for p in d.glob("*.json"):
        try:
            j = json.loads(p.read_text())
        except Exception:  # noqa: BLE001
            continue
        row = j.get("row", {})
        if row.get("scenario") in CONDS and row.get("seed"):
            jsons[(row["scenario"], int(row["seed"]))] = j
    if args.overlay_seeds is None:
        differ = [p["seed"] for p in paired if p["initial"] != p["optimized"]]
        longest = sorted((p for p in paired if not math.isnan(p["dd"])), key=lambda p: p["dd"])
        picks = differ + [p["seed"] for p in longest[:3]] + [p["seed"] for p in longest[-3:]]
        seen = []
        for s in picks:
            if s not in seen:
                seen.append(s)
        overlay_seeds = seen[:args.max_overlays]
    else:
        overlay_seeds = args.overlay_seeds
    drawn = []
    for seed in overlay_seeds:
        have = [(c, jsons[(c, seed)]) for c in conds if (c, seed) in jsons and jsons[(c, seed)].get("trajectory")]
        if not have:
            continue
        cfg = load_config(common_path=f"configs/campaigns/cross/campaign_h8_a3_cross_s{seed}.yaml")
        fig, ax = plt.subplots(figsize=(9, 7))
        for key, hx, hy in human_positions(cfg):
            ax.add_patch(plt.Circle((hx, hy), 0.25, color="#999999", alpha=0.6))
            ax.text(hx, hy + 0.35, key, ha="center", fontsize=7, color="#555555")
        for c, j in have:
            tr = j["trajectory"]
            ax.plot([s[1] for s in tr], [s[2] for s in tr], color=COLOUR[c], lw=2,
                    label=f"{LABEL[c]} driven: {j['row']['result']}, {f(j['row']['travel_time']):.0f} s, "
                          f"{f(j['row']['travel_distance']):.1f} m")
            pp = j.get("planned_path") or []
            if pp:
                ax.plot([q[0] for q in pp], [q[1] for q in pp], color=COLOUR[c], lw=1, ls="--", alpha=0.7,
                        label=f"{LABEL[c]} first plan {f(j['row'].get('planned_path_length')):.1f} m")
            ax.plot(tr[-1][1], tr[-1][2], marker="x", color=COLOUR[c], ms=9, mew=2)
        nav = cfg["navigation"]
        ax.plot(*nav["start"], marker="o", color="#222222", ms=7)
        ax.plot(*nav["goal"], marker="*", color="#222222", ms=12)
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(fontsize=7, loc="best")
        fig.suptitle(f"{folder}\nV4  seed {seed}: driven routes (solid) and first Nav2 plans (dashed); "
                     f"x marks where each trial ended", fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, 0.93))
        fig.savefig(figs / f"V4_trajectories_s{seed}.png", dpi=150)
        plt.close(fig)
        drawn.append(seed)

    # tables + report
    cols = ["condition", "size m", "n", "valid", "success", "comfort DL", "other fail", "travel s",
            "dist m", "plan m", "detour m", "yield s", "wait s", "min clr m", "clr min", "contact",
            "robot in", "intr m", "intr n", "in CS s", "r-r gap m"]
    t = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for c in conds:
        m = M[c]
        t.append("| " + " | ".join([
            LABEL[c], fmt(m["size_m"]), str(m["n"]), str(m["n_valid"]),
            f"{m['n_success']}/{m['n_valid']} ({pct(m['success_rate'])})", str(m["comfort_deadlock_n"]),
            str(m["other_fail_n"]), f"{fmt(m['travel_time_mean'], 1)}±{fmt(m['travel_time_sd'], 1)}",
            f"{fmt(m['travel_distance_mean'])}±{fmt(m['travel_distance_sd'])}", fmt(m["planned_path_mean"]),
            fmt(m["detour_mean"]), fmt(m["yield_mean"], 1), fmt(m["waited_mean"], 1),
            fmt(m["min_clearance_mean"]), fmt(m["min_clearance_min"]), str(m["contact_trials"]),
            str(m["contact_robot_trials"]), fmt(m["intrusion_mean"]), str(m["intrusion_trials"]),
            fmt(m["comfort_seconds_mean"], 1), fmt(m["robot_gap_min"])]) + " |")
    table = "\n".join(t)
    mech_t = ["| condition | " + " | ".join(MECH_LABEL[m] for m in MECHS) + " |", "|" + "---|" * (len(MECHS) + 1)]
    for c in conds:
        mech_t.append(f"| {LABEL[c]} | " + " | ".join(str(M[c]["mechanisms"].get(m, 0)) for m in MECHS) + " |")
    pair_t = []
    if paired:
        pair_t = ["| seed | Initial | Optimized | Δ time s | Δ dist m | Δ min clr m | Δ intrusion m |",
                  "|---|---|---|---|---|---|---|"]
        for p in paired:
            pair_t.append(f"| {p['seed']} | {p['initial']}{' (' + p['initial_mech'] + ')' if p['initial_mech'] else ''} | "
                          f"{p['optimized']}{' (' + p['optimized_mech'] + ')' if p['optimized_mech'] else ''} | "
                          f"{fmt(p['dt'], 1)} | {fmt(p['dd'])} | {fmt(p['dclr'])} | {fmt(p['dintr'])} |")
    (d / "tables.md").write_text("\n".join(
        [f"# {folder}: Stage 3 tables", "", "## AMR 1 by condition", "", table, "",
         "## Failure mechanism", "", "\n".join(mech_t), ""]
        + (["## Paired per seed (Optimized − Initial)", "", "\n".join(pair_t), ""] if pair_t else [])),
        encoding="utf-8")
    (d / "comparison.json").write_text(json.dumps({"conditions": M, "paired": paired}, indent=2, default=str),
                                       encoding="utf-8")
    ok_pairs = [p for p in paired if not math.isnan(p["dt"])]
    r = [f"# {folder}: Stage 3 — B3-O Initial vs Optimized" + (" (+ Static baseline)" if "static_baseline" in conds else ""),
         "", f"{sum(M[c]['n'] for c in conds)} AMR 1 trials ({other} rows with other scenario names ignored). "
         "Same campaign config per seed; only the Comfort Space overlay differs.", "",
         "## Headline", "", table, ""]
    if ok_pairs:
        r += [f"- Over the {len(ok_pairs)} seeds where both B3-O conditions reached the goal: travel time "
              f"{mean([p['dt'] for p in ok_pairs]):+.1f} s, distance {mean([p['dd'] for p in ok_pairs]):+.2f} m "
              f"(Optimized − Initial, mean).",
              f"- Outcomes differ on seeds: {[p['seed'] for p in paired if p['initial'] != p['optimized']] or 'none'}.",
              f"- Comfort gate holds: Initial {M['b3o_initial']['comfort_deadlock_n']}, Optimized "
              f"{M['b3o_optimized']['comfort_deadlock_n']}.", ""]
    r += ["## What to check against the research logic", "",
          "- Success maintained or improved; comfort-induced deadlocks not increased (table above, V3).",
          "- Travel time and distance reduced; detour (driven − first plan) reduced (V1, V2, 'detour m').",
          "- Human comfort requirement: the Optimized geometry is the Stage 2 result; intrusion here is "
          "measured against the fixed reference geometry, identical for every condition, so it is "
          "comparable across conditions but is not itself a comfort judgement.", "",
          "## Figures", "", "| | file |", "|---|---|",
          "| V1 | figures/V1_summary.png |", "| V2 | figures/V2_paired_deltas.png |",
          "| V3 | figures/V3_mechanisms.png |"]
    r += [f"| V4 | figures/V4_trajectories_s{s}.png |" for s in drawn]
    r += ["", "Tables: tables.md. Data: comparison.json.", ""]
    (d / "REPORT.md").write_text("\n".join(r), encoding="utf-8")
    print(f"[stage3_report] {len(conds)} conditions, overlays for seeds {drawn} -> {d / 'REPORT.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
