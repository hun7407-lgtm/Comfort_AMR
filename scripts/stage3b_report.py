#!/usr/bin/env python3
"""Stage 3 main comparison: B2 Static Calibrated vs B3-O Dynamic Selection.

    python3 scripts/stage3b_report.py --dir results/batches/<stage3b batch>

Plain Python 3 (+ scipy for the tests when present). No ROS.

Paired by seed: a seed is one scenario (people, walks, state changes,
missions, start times, map, Nav2 all identical), run once under each
policy. Seeds are blocks, not a factor.

Per AMR 1 trial the failure class is decided by comfort_core/failure_class.py,
with the selection facts for the person the robot stopped at looked up
from the condition overlays: the spot table of configs/stage3/b3o_selection.yaml
(R_nav_safe per spot and state), the frozen Stage 2 requirements
(R_comfort_min per context) and B2's calibrated radius. Classes are never
merged; UNCLASSIFIED_STALL is reported as such.

Outputs in the batch folder: REPORT.md, tables.md, comparison.json, figures/
    M1_outcomes.png       success and failure class per policy, stacked
    M2_paired_outcome.png 2x2 same-seed outcome (B2 fail -> B3-O success ...)
    M3_paired_time.png    per-seed travel-time difference where both succeeded

Statistics: exact McNemar on the discordant pairs (success; comfort-induced
stop = classes PASSAGE_INFEASIBLE + STATIC_OVER_RESERVATION + SELECTION_FAILURE),
paired Wilcoxon signed-rank on continuous outcomes over pairs where both
values exist. Reported for all seeds and separately for seeds 1-33 (used in
the preliminary study) and 34-100 (never seen before).

Display names: AMR 1, AMR 2, AMR 3 (comfort_core/labels.py).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from math import comb
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

from comfort_core.failure_class import CLASSES, classify  # noqa: E402
from comfort_core.labels import robot_label  # noqa: E402

B2, B3O = "b2_static_calibrated", "b3o_selection"
LABEL = {B2: "B2 Static Calibrated", B3O: "B3-O Dynamic Selection"}
COLOUR = {B2: "#d55e00", B3O: "#0072b2"}
CLASS_COLOUR = {"PASSAGE_INFEASIBLE": "#999999", "STATIC_OVER_RESERVATION": "#e69f00",
                "SELECTION_FAILURE": "#b2182b", "ROBOT_ROBOT": "#cc79a7",
                "PROTECTIVE_STOP": "#009e73",
                "POST_EVADE_STALL": "#ef8a62", "NAV2_FAILURE": "#56b4e9",
                "UNCLASSIFIED_STALL": "#444444", "INVALID": "#dddddd"}
COMFORT_STOP = {"PASSAGE_INFEASIBLE", "STATIC_OVER_RESERVATION", "SELECTION_FAILURE"}
HUMAN_RADIUS = 0.25


def f(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return math.nan
    return x


def read(p: Path):
    return list(csv.DictReader(p.open(newline=""))) if p.exists() else []


def selection_facts(row, spots, reqs, r_b2):
    """Add selection_* columns for the verdict person (see failure_class)."""
    p = row.get("verdict_person") or ""
    px, py, state = f(row.get("verdict_person_x")), f(row.get("verdict_person_y")), \
        row.get("verdict_person_state") or ""
    row["r_b2_m"] = r_b2
    if not p or math.isnan(px):
        return row
    best = min(spots, key=lambda s: math.hypot(s["x"] - px, s["y"] - py), default=None)
    if not best or math.hypot(best["x"] - px, best["y"] - py) > 0.6:
        return row
    entry = best.get("states", {}).get(state)
    if not entry:
        return row
    r_nav = entry.get("r_nav_safe_m")
    r_min = (reqs.get(entry["context"]) or {}).get("r_comfort_min_m")
    row[f"selection_r_nav_safe_{p}"] = r_nav
    row[f"selection_r_comfort_min_{p}"] = r_min
    if r_nav is not None and r_min is not None:
        row[f"selection_status_{p}"] = "PASSAGE_INFEASIBLE" if r_nav < r_min else "FEASIBLE"
    return row


def mcnemar_exact(b: int, c: int) -> float:
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)


def wilcoxon(xs):
    xs = [x for x in xs if not math.isnan(x)]
    if len(xs) < 6:
        return None
    try:
        from scipy.stats import wilcoxon as w
        return float(w(xs).pvalue)
    except Exception:
        return None


def summarise(rows):
    v = [r for r in rows if r["_class"] != "INVALID"]
    s = [r for r in v if r["_class"] == "SUCCESS"]

    def mean(key, rs, add=0.0):
        xs = [f(r.get(key)) + add for r in rs if not math.isnan(f(r.get(key)))]
        return (sum(xs) / len(xs)) if xs else math.nan
    return {"n": len(rows), "n_valid": len(v), "success": len(s),
            "classes": dict(Counter(r["_class"] for r in rows)),
            "travel_time": mean("travel_time", s), "travel_distance": mean("travel_distance", s),
            "plan_length": mean("planned_path_length", s),
            "detour": (mean("travel_distance", s) - mean("planned_path_length", s)),
            "yield_s": mean("yield_seconds", v), "wait_s": mean("waited_for_humans", v),
            "min_dist_centre": mean("min_clearance", v, HUMAN_RADIUS),
            "contact_s": mean("contact_seconds", v),
            "contacts": sum(1 for r in v if f(r.get("contact_seconds")) > 0)}


def paired(by_seed, seeds):
    pairs = [(by_seed[s][B2], by_seed[s][B3O]) for s in seeds
             if B2 in by_seed[s] and B3O in by_seed[s]
             and by_seed[s][B2]["_class"] != "INVALID" and by_seed[s][B3O]["_class"] != "INVALID"]
    ok = lambda r: r["_class"] == "SUCCESS"
    cs = lambda r: r["_class"] in COMFORT_STOP
    t = Counter((ok(a), ok(b)) for a, b in pairs)
    u = Counter((cs(a), cs(b)) for a, b in pairs)
    dt = [f(b["travel_time"]) - f(a["travel_time"]) for a, b in pairs if ok(a) and ok(b)]
    dd = [f(b["min_clearance"]) - f(a["min_clearance"]) for a, b in pairs]
    return {"pairs": len(pairs),
            "success": {"both": t[(True, True)], "B2_only": t[(True, False)],
                        "B3O_only": t[(False, True)], "neither": t[(False, False)],
                        "mcnemar_p": mcnemar_exact(t[(True, False)], t[(False, True)])},
            "comfort_stop": {"both": u[(True, True)], "B2_only": u[(True, False)],
                             "B3O_only": u[(False, True)], "neither": u[(False, False)],
                             "mcnemar_p": mcnemar_exact(u[(True, False)], u[(False, True)])},
            "travel_time_delta_mean": (sum(dt) / len(dt)) if dt else math.nan,
            "travel_time_wilcoxon_p": wilcoxon(dt), "travel_time_pairs": len(dt),
            "min_clearance_delta_mean": (sum(x for x in dd if not math.isnan(x)) /
                                         max(1, sum(1 for x in dd if not math.isnan(x)))),
            "min_clearance_wilcoxon_p": wilcoxon(dd), "_dt": dt}


def fmt(v, nd=2):
    return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--selection", default="configs/stage3/b3o_selection.yaml")
    ap.add_argument("--b2", default="configs/stage3/b2_static_calibrated.yaml")
    ap.add_argument("--requirements", default="configs/frozen/comfort_requirements.yaml")
    args = ap.parse_args()
    d = Path(args.dir)
    rows = [r for r in read(d / "trials.csv") if r.get("scenario") in (B2, B3O)]
    if not rows:
        print(f"[stage3b_report] no {B2}/{B3O} rows in {d}")
        return 1
    sel = yaml.safe_load(Path(args.selection).read_text())["comfort_space"]["selection"]
    spots = sel.get("spots") or []
    reqs = (yaml.safe_load(Path(args.requirements).read_text()) or {}).get("contexts") or {}
    r_b2 = (yaml.safe_load(Path(args.b2).read_text()) or {})["comfort_space"].get("static_radius")
    by_seed = defaultdict(dict)
    for r in rows:
        selection_facts(r, spots, reqs, r_b2)
        r["_class"] = classify(r)
        by_seed[int(r["seed"])][r["scenario"]] = r
    seeds = sorted(by_seed)
    M = {c: summarise([r for r in rows if r["scenario"] == c]) for c in (B2, B3O)}
    P = {"all": paired(by_seed, seeds),
         "seeds 1-33 (seen in the preliminary study)": paired(by_seed, [s for s in seeds if s <= 33]),
         "seeds 34-100 (new)": paired(by_seed, [s for s in seeds if s > 33])}
    fleet = defaultdict(lambda: defaultdict(list))
    for fr in read(d / "fleet_trials.csv"):
        if fr.get("scenario") in (B2, B3O):
            fleet[fr["scenario"]][fr["robot"]].append(fr)

    figs = d / "figures"
    figs.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 4))
    for i, c in enumerate((B2, B3O)):
        bottom = 0
        for k in ("SUCCESS",) + tuple(x for x in CLASSES if x != "SUCCESS"):
            n = M[c]["classes"].get(k, 0)
            if n:
                ax.bar(i, n, bottom=bottom, color=CLASS_COLOUR.get(k, COLOUR[c] if k == "SUCCESS" else "#888"),
                       edgecolor="white", width=0.55, label=k if i == 0 or k not in M[B2]["classes"] else None)
                ax.text(i, bottom + n / 2, f"{k.lower().replace('_', ' ')} {n}", ha="center", va="center", fontsize=7)
                bottom += n
    ax.set_xticks([0, 1])
    ax.set_xticklabels([LABEL[B2], LABEL[B3O]])
    ax.set_ylabel("AMR 1 trials")
    ax.set_title("M1  AMR 1 outcome and failure class per policy", fontsize=10)
    fig.tight_layout()
    fig.savefig(figs / "M1_outcomes.png", dpi=150)
    plt.close(fig)

    s = P["all"]["success"]
    fig, ax = plt.subplots(figsize=(4.6, 4))
    grid = [[s["both"], s["B2_only"]], [s["B3O_only"], s["neither"]]]
    ax.imshow([[1, 0], [0, 1]], cmap="Greys", alpha=0.15)
    for (i, j), lab in zip([(0, 0), (0, 1), (1, 0), (1, 1)],
                           ["both succeed", "B2 only", "B3-O only", "neither"]):
        ax.text(j, i, f"{lab}\n{grid[i][j]}", ha="center", va="center", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f"M2  same seed, AMR 1 success (exact McNemar p = {fmt(s['mcnemar_p'], 3)})", fontsize=9)
    fig.tight_layout()
    fig.savefig(figs / "M2_paired_outcome.png", dpi=150)
    plt.close(fig)

    dt = P["all"]["_dt"]
    if dt:
        fig, ax = plt.subplots(figsize=(6, 3.4))
        ax.hist(dt, bins=20, color=COLOUR[B3O], edgecolor="white")
        ax.axvline(0, color="#333", lw=1)
        ax.set_xlabel("travel time, B3-O minus B2 (s), seeds where both succeeded")
        ax.set_ylabel("seeds")
        ax.set_title("M3  paired travel-time difference", fontsize=10)
        fig.tight_layout()
        fig.savefig(figs / "M3_paired_time.png", dpi=150)
        plt.close(fig)

    keys = [("n", "trials", 0), ("n_valid", "valid", 0), ("success", "success", 0),
            ("travel_time", "travel time s (successes)", 1), ("travel_distance", "travel distance m", 2),
            ("plan_length", "first plan m", 2), ("detour", "detour m", 2), ("yield_s", "yield s", 1),
            ("wait_s", "waiting for walkers s", 1), ("min_dist_centre", "min distance person centre - AMR 1 m", 2),
            ("contacts", "trials with contact", 0)]
    t = ["| | " + " | ".join(LABEL[c] for c in (B2, B3O)) + " |", "|---|---|---|"]
    for k, lab, nd in keys:
        t.append(f"| {lab} | " + " | ".join(fmt(M[c][k], nd) if isinstance(M[c][k], float) else str(M[c][k])
                                           for c in (B2, B3O)) + " |")
    cls = ["| failure class | " + " | ".join(LABEL[c] for c in (B2, B3O)) + " |", "|---|---|---|"]
    for k in CLASSES:
        cls.append(f"| {k} | " + " | ".join(str(M[c]["classes"].get(k, 0)) for c in (B2, B3O)) + " |")
    pt = ["| seeds | pairs | success both / B2 only / B3-O only / neither | McNemar p | comfort stop B2 only / B3-O only | McNemar p | Δ travel time s (B3-O - B2) | Wilcoxon p |",
          "|---|---|---|---|---|---|---|---|"]
    for name, p in P.items():
        pt.append(f"| {name} | {p['pairs']} | {p['success']['both']} / {p['success']['B2_only']} / "
                  f"{p['success']['B3O_only']} / {p['success']['neither']} | {fmt(p['success']['mcnemar_p'], 3)} | "
                  f"{p['comfort_stop']['B2_only']} / {p['comfort_stop']['B3O_only']} | "
                  f"{fmt(p['comfort_stop']['mcnemar_p'], 3)} | {fmt(p['travel_time_delta_mean'], 1)} "
                  f"(n {p['travel_time_pairs']}) | {fmt(p['travel_time_wilcoxon_p'], 3)} |")
    fl = ["| policy | robot | trials | success | min gap to AMR 1 m |", "|---|---|---|---|---|"]
    for c in (B2, B3O):
        for robot, frs in sorted(fleet[c].items()):
            gaps = [f(x.get("min_gap_amr1_m")) for x in frs if not math.isnan(f(x.get("min_gap_amr1_m")))]
            fl.append(f"| {LABEL[c]} | {robot_label(robot)} | {len(frs)} | "
                      f"{sum(1 for x in frs if x.get('result') == 'SUCCESS')} | {fmt(min(gaps) if gaps else math.nan)} |")
    rep = [f"# {d.name}: Stage 3 main comparison", "",
           f"{len(rows)} AMR 1 trials over {len(seeds)} seeds; each seed is one scenario run under "
           "both policies (same people, walks, state changes, missions, start times, map, Nav2, "
           "safety layer).", "",
           f"B2 Static Calibrated radius: {r_b2} m (max over contexts of R_comfort_min, frozen). "
           "B3-O Dynamic Selection: per spot and state R* within [R_comfort_min, R_nav_safe], "
           "PASSAGE_INFEASIBLE otherwise (Comfort Space kept at R_comfort_min).", "",
           "## AMR 1 by policy", ""] + t + ["", "## Failure classes (comfort_core/failure_class.py)", ""] + cls + [
           "", "## Paired by seed", ""] + pt + ["", "## AMR 2 / AMR 3", ""] + fl + [
           "", "Figures: `figures/M1_outcomes.png`, `figures/M2_paired_outcome.png`, `figures/M3_paired_time.png`.",
           "", "> R_nav_safe is a conservative bound of this navigation stack (Stage 1). R_comfort_min "
           "comes from the Stage 2 human study. Neither is an ISO or validated comfort distance."]
    (d / "REPORT.md").write_text("\n".join(rep) + "\n", encoding="utf-8")
    (d / "comparison.json").write_text(json.dumps(
        {"summary": M, "paired": {k: {kk: vv for kk, vv in v.items() if kk != "_dt"} for k, v in P.items()},
         "per_seed": {s: {c: by_seed[s][c]["_class"] for c in by_seed[s]} for s in seeds}},
        indent=2, default=str), encoding="utf-8")
    print(f"[stage3b_report] {len(rows)} trials, {len(seeds)} seeds -> {d / 'REPORT.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
