#!/usr/bin/env python3
"""Stage 1 report: Comfort Space size vs navigation feasibility.

    python3 scripts/stage1_report.py --dir results/batches/261112_stage1_size_sweep

Reads <dir>/trials.csv (AMR1) and <dir>/fleet_trials.csv (a1, a2) and
writes

    <dir>/REPORT.md               thresholds, per-size tables, per-AMR tables
    <dir>/tables.md               every number behind the figures
    <dir>/thresholds.json         R_deadlock_onset / R_nav_max per policy
    <dir>/figures/P1_success_vs_size.png
    <dir>/figures/P2_comfort_deadlock_vs_size.png     with the thresholds drawn
    <dir>/figures/P3_travel_time_vs_size.png
    <dir>/figures/P4_travel_distance_vs_size.png
    <dir>/figures/P5_min_clearance_vs_size.png
    <dir>/figures/P6_size_x_seed_matrix.png
    <dir>/figures/P7_threshold_geometry.png            how much of the 1.5 m aisle each size takes
    <dir>/figures/P8_failure_mechanism.png
    <dir>/figures/P9_fleet_vs_size.png                 a1 / a2 outcomes

Threshold definition (per policy)
---------------------------------
Only `comfort_gate_hold` counts: the yield gate holding the robot out of
a Comfort Space when the no-progress rule fired. That is the one
mechanism the SIZE is responsible for. Post-evade stalls, Nav2 aborts,
plain stalls and invalid trials are reported beside it, never added in.

    R_deadlock_onset  smallest size with >= --onset-count comfort deadlocks
    R_nav_max         largest size below the onset with none

If every size has at least one, R_nav_max is "below the sweep"; if none
has any, R_deadlock_onset is "above the sweep". The count table is
printed in full so a non-monotonic pattern is visible rather than
summarised away.

Plain Python 3 + matplotlib. Nothing here changes a result; a missing
trial is a missing row, and the tables say n for every cell.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from comfort_core import load_config                       # noqa: E402
from comfort_core.comfort_space import make_policy, robot_grow  # noqa: E402
from comfort_core.size_sweep import (DEFAULT_SIZES, KIND_LABEL, KIND_POLICY,  # noqa: E402
                                     parse_condition)
from report_pilot import f, geometry, mean, truthy  # noqa: E402

KINDS = ["static", "b3o"]
# Okabe-Ito vermilion / blue: distinguishable under every common colour
# vision deficiency, and the two series are always labelled as well.
COLOUR = {"static": "#d55e00", "b3o": "#0072b2"}
MARKER = {"static": "s", "b3o": "o"}

MECHS = ["comfort_gate_hold", "post_evade_stall", "nav2_abort", "stall",
         "planning_failure", "timeout", "invalid"]
MECH_LABEL = {"comfort_gate_hold": "Comfort gate hold", "post_evade_stall": "Post-evade stall",
              "nav2_abort": "Nav2 abort", "stall": "Stall (other)",
              "planning_failure": "Planning failure", "timeout": "Timeout",
              "invalid": "Invalid (physics / measurement)"}
# One hue family per mechanism, status-like: the comfort one is the one
# the sweep is about and gets the strongest colour.
MECH_COLOUR = {"comfort_gate_hold": "#b2182b", "post_evade_stall": "#ef8a62",
               "nav2_abort": "#67a9cf", "stall": "#999999",
               "planning_failure": "#5e3c99", "timeout": "#cccccc", "invalid": "#f0f0f0"}
OUTCOME_COLOUR = {"SUCCESS": "#1a9850", **MECH_COLOUR}

RESULTS = ("SUCCESS", "DEADLOCK", "TIMEOUT", "INVALID", "PLANNING_FAILURE")


# ---------------------------------------------------------------- data
def read(path: Path) -> List[Dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as h:
        return list(csv.DictReader(h))


def mechanism_of(r: Dict[str, str]) -> str:
    """The recorded mechanism, with a fallback for rows written before the column."""
    if r.get("result") == "SUCCESS":
        return ""
    m = r.get("deadlock_mechanism") or ""
    if m:
        return m
    if r.get("result") == "INVALID":
        return "invalid"
    if r.get("result") == "TIMEOUT":
        return "timeout"
    if r.get("result") == "PLANNING_FAILURE":
        return "planning_failure"
    if (r.get("deadlock_cause") or "").startswith("yielding to a Comfort Space"):
        return "comfort_gate_hold"
    return "stall"


def load(d: Path):
    rows = read(d / "trials.csv")
    fleet = read(d / "fleet_trials.csv")
    cells: Dict[str, Dict[float, List[Dict[str, str]]]] = {k: defaultdict(list) for k in KINDS}
    skipped = 0
    for r in rows:
        parsed = parse_condition(r["scenario"])
        if not parsed:
            skipped += 1
            continue
        kind, size = parsed
        r["_mech"] = mechanism_of(r)
        r["_seed"] = int(r["seed"]) if r.get("seed") else None
        cells[kind][size].append(r)
    fcells: Dict[str, Dict[float, Dict[str, List[Dict[str, str]]]]] = {
        k: defaultdict(lambda: defaultdict(list)) for k in KINDS}
    for r in fleet:
        parsed = parse_condition(r["scenario"])
        if not parsed:
            continue
        kind, size = parsed
        fcells[kind][size][r["robot"]].append(r)
    return cells, fcells, skipped, rows


def sizes_of(cells) -> List[float]:
    return sorted({s for k in KINDS for s in cells[k]})


# ---------------------------------------------------------------- metrics
def valid(rs):
    return [r for r in rs if truthy(r.get("valid", "True"))]


def succ(rs):
    return [r for r in rs if r["result"] == "SUCCESS"]


def rate(rs, pred) -> float:
    return sum(1 for r in rs if pred(r)) / len(rs) if rs else math.nan


def se(p: float, n: int) -> float:
    return math.sqrt(p * (1 - p) / n) if n and not math.isnan(p) else math.nan


def fleet_min_gap(fcells_size: Dict[str, List[Dict[str, str]]], stamp_match) -> float:
    """AMR1's closest centre-to-centre approach to any fleet vehicle in a trial."""
    gaps = []
    for robot, frs in fcells_size.items():
        for fr in frs:
            if stamp_match(fr):
                gaps.append(f(fr.get("min_gap_amr1_m")))
    gaps = [g for g in gaps if not math.isnan(g)]
    return min(gaps) if gaps else math.nan


def cell_metrics(rs: List[Dict[str, str]], frs: Dict[str, List[Dict[str, str]]]) -> Dict[str, Any]:
    v = valid(rs)
    s = succ(v)
    mech = Counter(r["_mech"] for r in rs if r["_mech"])
    out: Dict[str, Any] = {
        "n": len(rs), "n_valid": len(v), "n_success": len(s),
        "success_rate": rate(v, lambda r: r["result"] == "SUCCESS"),
        "comfort_deadlock_n": mech.get("comfort_gate_hold", 0),
        "comfort_deadlock_rate": rate(v, lambda r: r["_mech"] == "comfort_gate_hold"),
        "total_deadlock_n": sum(1 for r in v if r["result"] != "SUCCESS"),
        "total_deadlock_rate": rate(v, lambda r: r["result"] != "SUCCESS"),
        "mechanisms": dict(mech),
        # cost, over SUCCESSFUL trials only (a deadlocked trial's travel
        # time is the time to the deadlock, not a travel time)
        "travel_time_mean": mean([f(r["travel_time"]) for r in s]),
        "travel_time_sd": _sd([f(r["travel_time"]) for r in s]),
        "travel_distance_mean": mean([f(r["travel_distance"]) for r in s]),
        "travel_distance_sd": _sd([f(r["travel_distance"]) for r in s]),
        "planned_path_mean": mean([f(r.get("planned_path_length")) for r in s]),
        "yield_mean": mean([f(r["yield_seconds"]) for r in v]),
        "waited_mean": mean([f(r["waited_for_humans"]) for r in v]),
        "evade_mean": mean([f(r["evade_seconds"]) for r in v]),
        # people, over ALL valid trials (a deadlocked robot still has a
        # closest approach)
        "min_clearance_mean": mean([f(r["min_clearance"]) for r in v]),
        "min_clearance_min": min([f(r["min_clearance"]) for r in v], default=math.nan),
        "contact_trials": sum(1 for r in v if f(r["contact_seconds"]) > 0),
        "contact_robot_trials": sum(1 for r in v if f(r["contact_robot_seconds"]) > 0),
        "intrusion_mean": mean([f(r["worst_intrusion"]) for r in v]),
        "intrusion_trials": sum(1 for r in v if f(r["worst_intrusion"]) > 0),
        "comfort_seconds_mean": mean([f(r["comfort_seconds_total"]) for r in v]),
        "backoffs": sum(int(f(r["backoffs"], 0)) for r in v),
        "recoveries": sum(int(f(r["off_lane_recoveries"], 0)) for r in v),
        "safety_stop_mean": mean([f(r["safety_stop_only_seconds"]) for r in v]),
    }
    gaps = []
    for r in v:
        # Same trial = same config, fleet rows stamped within a few
        # minutes of the AMR1 row (they are written at the end of the
        # same run, the fleet's a little later).
        t0 = _stamp_s(r["stamp"])
        g = fleet_min_gap(frs, lambda fr, t0=t0, cfg=r["config"]:
                          fr["config"] == cfg and abs(_stamp_s(fr["stamp"]) - t0) < 240)
        if not math.isnan(g):
            gaps.append(g)
    out["robot_gap_min"] = min(gaps) if gaps else math.nan
    out["robot_gap_mean"] = mean(gaps)
    return out


def _stamp_s(stamp: str) -> float:
    from datetime import datetime
    try:
        return datetime.fromisoformat(stamp).timestamp()
    except ValueError:
        return math.nan


def _sd(xs: List[float]) -> float:
    xs = [x for x in xs if not math.isnan(x)]
    if len(xs) < 2:
        return math.nan
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def fleet_metrics(frs: List[Dict[str, str]]) -> Dict[str, Any]:
    return {
        "n": len(frs),
        "success_rate": rate(frs, lambda r: r["result"] == "SUCCESS"),
        "deadlock_n": sum(1 for r in frs if r["result"] == "DEADLOCK"),
        "aborted_n": sum(1 for r in frs if r["result"] == "ABORTED"),
        "legs_done_rate": (sum(f(r["legs_done"], 0) for r in frs)
                           / max(sum(f(r["legs_total"], 0) for r in frs), 1)) if frs else math.nan,
        "distance_mean": mean([f(r["travel_distance_m"]) for r in frs]),
        "min_clearance_mean": mean([f(r["min_clearance_m"]) for r in frs]),
        "min_clearance_min": min([f(r["min_clearance_m"]) for r in frs], default=math.nan),
        "contact_trials": sum(1 for r in frs if f(r["contact_s"]) > 0),
        "intrusion_mean": mean([f(r["worst_intrusion_m"]) for r in frs]),
        "gap_amr1_min": min([f(r["min_gap_amr1_m"]) for r in frs], default=math.nan),
        "gap_fleet_min": min([f(r["min_gap_fleet_m"]) for r in frs], default=math.nan),
        "safety_stop_mean": mean([f(r["safety_stop_s"]) for r in frs]),
    }


# ---------------------------------------------------------------- thresholds
def thresholds(sizes: List[float], counts: Dict[float, int], ns: Dict[float, int],
               onset_count: int) -> Dict[str, Any]:
    present = [s for s in sizes if ns.get(s, 0) > 0]
    onset = next((s for s in present if counts.get(s, 0) >= onset_count), None)
    if onset is None:
        nav_max = present[-1] if present else None
        note = (f"no comfort-induced deadlock at any swept size; R_nav_max is at "
                f"least {nav_max:.2f} m (the largest size swept), the onset lies above it")
    else:
        below = [s for s in present if s < onset and counts.get(s, 0) < onset_count]
        nav_max = below[-1] if below else None
        if nav_max is None:
            note = (f"comfort-induced deadlock already at the smallest swept size "
                    f"{onset:.2f} m; R_nav_max lies below the sweep")
        else:
            gap = onset - nav_max
            note = (f"onset between {nav_max:.2f} and {onset:.2f} m; refine with "
                    f"sizes in ({nav_max:.2f}, {onset:.2f}) at 0.05-0.10 m steps")
    later_zero = [s for s in present if onset is not None and s > onset
                  and counts.get(s, 0) < onset_count]
    return {"R_deadlock_onset_m": onset, "R_nav_max_m": nav_max,
            "onset_count_rule": onset_count, "note": note,
            "non_monotonic_sizes": later_zero,
            "comfort_deadlocks_by_size": {f"{s:.2f}": counts.get(s, 0) for s in present},
            "trials_by_size": {f"{s:.2f}": ns.get(s, 0) for s in present}}


def transition_thresholds(sizes: List[float], cells_k: Dict[float, List[Dict[str, str]]],
                          onset_count: int) -> Dict[str, Any]:
    """Thresholds from SIZE-INDUCED gate holds only (same-seed transitions).

    A seed that already ends in a comfort gate hold at the smallest swept
    size cannot be stopped by the size: that scenario is impassable for the
    policy as such (the person stands where no size fits, or the gate holds
    on a crossing at a bad moment). Those seeds are listed and excluded.
    For the remaining seeds every gate hold IS a transition success -> hold
    caused by growing the space, and those are counted per size with the
    same onset rule as thresholds(). Seeds that hold at one size and pass
    again at a larger one are listed as non-monotonic, not hidden.
    """
    by_seed: Dict[int, Dict[float, str]] = defaultdict(dict)
    for s in sizes:
        for r in valid(cells_k.get(s, [])):
            if r["_seed"] is not None:
                by_seed[r["_seed"]][s] = r["_mech"]
    present = sorted({s for o in by_seed.values() for s in o})
    if not present:
        return {"R_deadlock_onset_m": None, "R_nav_max_m": None, "excluded_seeds": [],
                "size_induced_by_size": {}, "seeds_by_size": {}, "non_monotonic_seeds": {},
                "onset_count_rule": onset_count, "note": "no valid rows"}
    smallest = present[0]
    intrinsic = sorted(sd for sd, o in by_seed.items() if o.get(smallest) == "comfort_gate_hold")
    counts: Dict[float, int] = {}
    ns: Dict[float, int] = {}
    nonmono: Dict[int, List[float]] = {}
    for sd, o in by_seed.items():
        if sd in intrinsic:
            continue
        holds = [s for s in present if o.get(s) == "comfort_gate_hold"]
        for s in present:
            if s in o:
                ns[s] = ns.get(s, 0) + 1
                if o[s] == "comfort_gate_hold":
                    counts[s] = counts.get(s, 0) + 1
        later_ok = [s for s in present if holds and s > holds[0] and o.get(s) == ""]
        if later_ok:
            nonmono[sd] = later_ok
    base = thresholds(present, counts, ns, onset_count)
    return {"R_deadlock_onset_m": base["R_deadlock_onset_m"], "R_nav_max_m": base["R_nav_max_m"],
            "onset_count_rule": onset_count, "note": base["note"],
            "excluded_seeds": intrinsic,
            "size_induced_by_size": {f"{s:.2f}": counts.get(s, 0) for s in present},
            "seeds_by_size": {f"{s:.2f}": ns.get(s, 0) for s in present},
            "non_monotonic_seeds": {str(sd): [f"{s:.2f}" for s in v] for sd, v in sorted(nonmono.items())}}


# ---------------------------------------------------------------- figures
def _axes_sizes(ax, sizes):
    if sizes:
        pad = max(0.12, 0.25 * (max(sizes) - min(sizes)) / max(len(sizes) - 1, 1))
        ax.set_xlim(min(sizes) - pad, max(sizes) + pad)
    ax.set_xticks(sizes)
    ax.set_xticklabels([f"{s:.2f}" for s in sizes])
    ax.set_xlabel("Comfort Space size (m)  [static radius / B3-O moving front @ 1.0 m/s]")
    ax.grid(alpha=0.25)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)


def _title(fig, title, folder):
    fig.suptitle(f"{folder}\n{title}", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))


def plot_rate(sizes, M, key, ylabel, title, out, folder, thr=None, se_key_n="n_valid"):
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for k in KINDS:
        xs = [s for s in sizes if s in M[k]]
        ys = [M[k][s][key] for s in xs]
        err = [se(M[k][s][key], M[k][s][se_key_n]) for s in xs]
        ax.errorbar(xs, ys, yerr=err, color=COLOUR[k], marker=MARKER[k], lw=2, ms=7,
                    capsize=3, label=f"{KIND_LABEL[k]} (n per size "
                    f"{min((M[k][s][se_key_n] for s in xs), default=0)}-"
                    f"{max((M[k][s][se_key_n] for s in xs), default=0)})")
        for x, y, s in zip(xs, ys, xs):
            ax.annotate(f"{M[k][s][key.replace('_rate', '_n')] if key.replace('_rate', '_n') in M[k][s] else y:.0f}"
                        if key != "success_rate" else f"{M[k][s]['n_success']}/{M[k][s]['n_valid']}",
                        (x, y), textcoords="offset points", xytext=(0, 8 if k == "b3o" else -14),
                        ha="center", fontsize=7, color="#333333")
    if thr:
        # Each policy's two lines at its own height, so Static and B3-O
        # thresholds at the same size do not write over each other.
        for i, k in enumerate(KINDS):
            t = thr[k]
            y_lab = 0.88 - 0.12 * i
            if t["R_deadlock_onset_m"] is not None:
                ax.axvline(t["R_deadlock_onset_m"], color=COLOUR[k], ls="--", lw=1)
                ax.text(t["R_deadlock_onset_m"] + 0.01, y_lab, f"{KIND_LABEL[k]} onset",
                        color=COLOUR[k], fontsize=8, va="center", ha="left")
            if t["R_nav_max_m"] is not None:
                ax.axvline(t["R_nav_max_m"], color=COLOUR[k], ls=":", lw=1)
                ax.text(t["R_nav_max_m"] - 0.01, y_lab, f"{KIND_LABEL[k]} R_nav_max",
                        color=COLOUR[k], fontsize=8, va="center", ha="right")
    ax.set_ylim(-0.03, 1.05)
    ax.set_ylabel(ylabel)
    _axes_sizes(ax, sizes)
    ax.legend(fontsize=8, loc="best")
    _title(fig, title, folder)
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_measure(sizes, M, key, sd_key, ylabel, title, out, folder, extra=None):
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for k in KINDS:
        xs = [s for s in sizes if s in M[k] and not math.isnan(M[k][s][key])]
        ys = [M[k][s][key] for s in xs]
        err = [M[k][s][sd_key] if sd_key and not math.isnan(M[k][s][sd_key]) else 0 for s in xs]
        ax.errorbar(xs, ys, yerr=err, color=COLOUR[k], marker=MARKER[k], lw=2, ms=7, capsize=3,
                    label=f"{KIND_LABEL[k]}" + (" (mean ± sd, successful trials)" if sd_key else ""))
        if extra:
            ex = [M[k][s][extra] for s in xs]
            ax.plot(xs, ex, color=COLOUR[k], marker=MARKER[k], mfc="white", lw=1, ls="--", ms=6,
                    label=f"{KIND_LABEL[k]} {extra.replace('_', ' ')}")
    ax.set_ylabel(ylabel)
    _axes_sizes(ax, sizes)
    ax.legend(fontsize=8)
    _title(fig, title, folder)
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_matrix(sizes, cells, out, folder):
    seeds = sorted({r["_seed"] for k in KINDS for s in cells[k] for r in cells[k][s]
                    if r["_seed"] is not None})
    fig, axes = plt.subplots(1, 2, figsize=(max(8, 0.28 * len(seeds) + 3), 4.6), sharey=True)
    for ax, k in zip(axes, KINDS):
        for i, s in enumerate(sizes):
            by_seed = {r["_seed"]: r for r in cells[k].get(s, [])}
            for j, seed in enumerate(seeds):
                r = by_seed.get(seed)
                if r is None:
                    colour, txt = "white", ""
                elif r["result"] == "SUCCESS":
                    colour, txt = OUTCOME_COLOUR["SUCCESS"], ""
                else:
                    colour = OUTCOME_COLOUR.get(r["_mech"], "#999999")
                    txt = {"comfort_gate_hold": "C", "post_evade_stall": "E", "nav2_abort": "A",
                           "stall": "S", "invalid": "x", "timeout": "T",
                           "planning_failure": "P"}.get(r["_mech"], "?")
                ax.add_patch(Rectangle((j, i), 1, 1, facecolor=colour, edgecolor="white", lw=1))
                if txt:
                    ax.text(j + 0.5, i + 0.5, txt, ha="center", va="center", fontsize=7,
                            color="white" if txt in ("C", "P") else "#222222")
        ax.set_xlim(0, len(seeds))
        ax.set_ylim(0, len(sizes))
        ax.set_xticks([j + 0.5 for j in range(len(seeds))])
        ax.set_xticklabels(seeds, fontsize=7)
        ax.set_yticks([i + 0.5 for i in range(len(sizes))])
        ax.set_yticklabels([f"{s:.2f} m" for s in sizes])
        ax.set_xlabel("seed")
        ax.set_title(KIND_LABEL[k], fontsize=10, color=COLOUR[k])
        ax.tick_params(length=0)
        for spine in ax.spines.values():
            spine.set_visible(False)
    axes[0].set_ylabel("Comfort Space size")
    handles = [Patch(facecolor=OUTCOME_COLOUR["SUCCESS"], label="reached the goal")]
    handles += [Patch(facecolor=MECH_COLOUR[m], label=f"{MECH_LABEL[m]} ({t})")
                for m, t in (("comfort_gate_hold", "C"), ("post_evade_stall", "E"),
                             ("nav2_abort", "A"), ("stall", "S"), ("invalid", "x"))]
    handles.append(Patch(facecolor="white", edgecolor="#cccccc", label="not run"))
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=8, frameon=False,
               bbox_to_anchor=(0.5, -0.01))
    fig.suptitle(f"{folder}\nP6  AMR1 outcome per size x seed", fontsize=10)
    fig.tight_layout(rect=(0, 0.08, 1, 0.92))
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_mechanisms(sizes, M, out, folder):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), sharey=True)
    for ax, k in zip(axes, KINDS):
        xs = list(range(len(sizes)))
        bottom = [0.0] * len(sizes)
        for m in MECHS:
            vals = [M[k][s]["mechanisms"].get(m, 0) if s in M[k] else 0 for s in sizes]
            if not any(vals):
                continue
            ax.bar(xs, vals, 0.62, bottom=bottom, color=MECH_COLOUR[m], label=MECH_LABEL[m],
                   edgecolor="white", lw=1)
            for x, b, v in zip(xs, bottom, vals):
                if v:
                    ax.text(x, b + v / 2, str(v), ha="center", va="center", fontsize=7,
                            color="white" if m in ("comfort_gate_hold", "planning_failure") else "#222222")
            bottom = [b + v for b, v in zip(bottom, vals)]
        ax.set_xticks(xs)
        ax.set_xticklabels([f"{s:.2f}\n(n={M[k][s]['n'] if s in M[k] else 0})" for s in sizes], fontsize=8)
        ax.set_title(f"{KIND_LABEL[k]}", fontsize=10, color=COLOUR[k])
        ax.set_xlabel("Comfort Space size (m)")
        ax.grid(axis="y", alpha=0.25)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
    axes[0].set_ylabel("AMR1 trials not reaching the goal")
    h, l = axes[0].get_legend_handles_labels()
    h2, l2 = axes[1].get_legend_handles_labels()
    seen = dict(zip(l + l2, h + h2))
    fig.legend(seen.values(), seen.keys(), loc="lower center", ncol=4, fontsize=8, frameon=False)
    fig.suptitle(f"{folder}\nP8  how AMR1 stopped, by size (only 'Comfort gate hold' is a size effect)",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0.1, 1, 0.92))
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_geometry(sizes, out, folder, campaign: str):
    sizes = sorted(set(sizes) | {s for s in DEFAULT_SIZES
                                 if (PROJECT_ROOT / "configs/stage1" / f"b3o_r{s:.2f}.yaml").is_file()})
    """P7: how much of the 1.5 m aisle each size leaves for the robot.

    Three people the cross scenario actually puts beside the north
    aisle, each drawn at every size for both policies:

      belt worker, engaged, facing the belt (back to the aisle)
      the same post with a STANDING person facing the belt (the N3
          condition: E1 has walked to N3's post)
      ... plus a stationary person on the storage side facing the aisle

    Reach into the aisle = extent toward the aisle - the person's
    offset from the aisle edge. Free channel = aisle - reaches. The
    robot needs its footprint width to pass without entering any space;
    the gate enforces exactly the configured extent to the nearest part
    of the footprint, so this is the geometric statement of the gate
    hold. The costmap buffer only biases the planner, it is not drawn.
    """
    g = geometry()
    aisle, robot_w = g["aisle"], g["robot_w"]
    worker_off, stander_off = g["worker_offset"], g["stander_offset"]
    cases = [
        ("engaged belt worker\n(back to the aisle)", "station_engaged", "rear", False),
        ("standing person at the belt post\n(N3 condition, back to the aisle)", "stationary", "rear", False),
        ("standing at the belt post\n+ stander on the storage side", "stationary", "rear", True),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.4), sharey=True)
    for ax, (title, state, side, two) in zip(axes, cases):
        for kk, k in enumerate(KINDS):
            frees = []
            for s in sizes:
                cond = PROJECT_ROOT / "configs/stage1" / f"{k}_r{s:.2f}.yaml"
                cfg = load_config(scenario=str(cond), common_path=campaign)
                pol = make_policy(cfg)
                e1 = pol.extents_for("_", state)
                reach1 = max(0.0, getattr(e1, side) - worker_off)
                reach2 = 0.0
                if two:
                    e2 = pol.extents_for("_", "stationary")
                    reach2 = max(0.0, e2.front - stander_off)
                frees.append(max(aisle - reach1 - reach2, 0.0))
            xs = [i + (kk - 0.5) * 0.36 for i in range(len(sizes))]
            bars = ax.bar(xs, frees, 0.34, color=COLOUR[k], label=KIND_LABEL[k], edgecolor="white")
            for b, fr in zip(bars, frees):
                ax.text(b.get_x() + b.get_width() / 2, fr + 0.02, f"{fr:.2f}", ha="center",
                        fontsize=7, color="#333333")
        ax.axhline(robot_w, color="#222222", lw=1.2, ls="--")
        ax.text(len(sizes) - 0.5, robot_w + 0.02, f"robot width {robot_w:.2f} m", ha="right",
                fontsize=8, color="#222222")
        ax.axhline(aisle, color="#999999", lw=0.8)
        ax.text(-0.45, aisle + 0.02, f"aisle {aisle:.1f} m", fontsize=8, color="#666666")
        ax.set_xticks(range(len(sizes)))
        ax.set_xticklabels([f"{s:.2f}" for s in sizes])
        ax.set_xlabel("Comfort Space size (m)")
        ax.set_title(title, fontsize=9)
        ax.set_ylim(0, aisle + 0.25)
        ax.grid(axis="y", alpha=0.25)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
    axes[0].set_ylabel("free channel left in the aisle (m)")
    axes[0].legend(fontsize=8, loc="upper right")
    fig.suptitle(f"{folder}\nP7  threshold geometry: aisle width left outside every Comfort Space "
                 f"(worker {worker_off:.2f} m from the aisle edge, stander {stander_off:.2f} m). "
                 f"Robot width is the physical lower bound; the planner also keeps the "
                 f"{g['buffer']:.2f} m cost buffer and wall inflation.", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_fleet(sizes, FM, out, folder):
    robots = sorted({r for k in KINDS for s in FM[k] for r in FM[k][s]})
    if not robots:
        return False
    fig, axes = plt.subplots(1, len(robots), figsize=(5.2 * len(robots), 4.0), sharey=True)
    axes = list(axes) if len(robots) > 1 else [axes]
    for ax, robot in zip(axes, robots):
        for k in KINDS:
            xs = [s for s in sizes if s in FM[k] and robot in FM[k][s]]
            ys = [FM[k][s][robot]["success_rate"] for s in xs]
            ns = [FM[k][s][robot]["n"] for s in xs]
            ax.errorbar(xs, ys, yerr=[se(y, n) for y, n in zip(ys, ns)], color=COLOUR[k],
                        marker=MARKER[k], lw=2, ms=7, capsize=3, label=KIND_LABEL[k])
        ax.set_title(f"{robot}  (fleet vehicle, no yield gate)", fontsize=10)
        ax.set_ylim(-0.03, 1.05)
        _axes_sizes(ax, sizes)
        ax.set_xlabel("Comfort Space size (m)")
    axes[0].set_ylabel("success rate (all legs reached)")
    axes[0].legend(fontsize=8)
    fig.suptitle(f"{folder}\nP9  fleet vehicles vs Comfort Space size", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return True


# ---------------------------------------------------------------- tables
def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        return "-" if math.isnan(v) else f"{v:.{nd}f}"
    return str(v)


def pct(v):
    return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{100 * v:.0f}%"


def table_sizes(sizes, M, k) -> str:
    cols = ["size (m)", "n", "valid", "success", "comfort DL", "total DL", "travel s", "dist m",
            "plan m", "yield s", "wait s", "evade s", "min clr m", "clr min", "contact", "robot in",
            "intr m", "intr n", "in CS s", "r-r gap m", "stop s"]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for s in sizes:
        if s not in M[k]:
            continue
        m = M[k][s]
        lines.append("| " + " | ".join([
            f"{s:.2f}", str(m["n"]), str(m["n_valid"]),
            f"{m['n_success']}/{m['n_valid']} ({pct(m['success_rate'])})",
            f"{m['comfort_deadlock_n']} ({pct(m['comfort_deadlock_rate'])})",
            f"{m['total_deadlock_n']} ({pct(m['total_deadlock_rate'])})",
            f"{fmt(m['travel_time_mean'], 1)}±{fmt(m['travel_time_sd'], 1)}",
            f"{fmt(m['travel_distance_mean'])}±{fmt(m['travel_distance_sd'])}",
            fmt(m["planned_path_mean"]), fmt(m["yield_mean"], 1), fmt(m["waited_mean"], 1),
            fmt(m["evade_mean"], 1), fmt(m["min_clearance_mean"]), fmt(m["min_clearance_min"]),
            str(m["contact_trials"]), str(m["contact_robot_trials"]), fmt(m["intrusion_mean"]),
            str(m["intrusion_trials"]), fmt(m["comfort_seconds_mean"], 1), fmt(m["robot_gap_min"]),
            fmt(m["safety_stop_mean"], 1)]) + " |")
    return "\n".join(lines)


def table_mech(sizes, M, k) -> str:
    cols = ["size (m)"] + [MECH_LABEL[m] for m in MECHS]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for s in sizes:
        if s not in M[k]:
            continue
        lines.append("| " + " | ".join([f"{s:.2f}"] + [str(M[k][s]["mechanisms"].get(m, 0))
                                                        for m in MECHS]) + " |")
    return "\n".join(lines)


def table_fleet(sizes, FM, k, robot) -> str:
    cols = ["size (m)", "n", "success", "deadlock", "aborted", "legs done", "dist m", "min clr m",
            "clr min", "contact", "intr m", "gap AMR1 m", "gap fleet m", "stop s"]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for s in sizes:
        m = FM[k].get(s, {}).get(robot)
        if not m:
            continue
        lines.append("| " + " | ".join([
            f"{s:.2f}", str(m["n"]), pct(m["success_rate"]), str(m["deadlock_n"]),
            str(m["aborted_n"]), pct(m["legs_done_rate"]), fmt(m["distance_mean"]),
            fmt(m["min_clearance_mean"]), fmt(m["min_clearance_min"]), str(m["contact_trials"]),
            fmt(m["intrusion_mean"]), fmt(m["gap_amr1_min"]), fmt(m["gap_fleet_min"]),
            fmt(m["safety_stop_mean"], 1)]) + " |")
    return "\n".join(lines)


def table_geometry(sizes, campaign) -> str:
    cols = ["condition", "size", "scale", "eng F/S/R", "stat F/S/R", "mov F/S/R @1.0 m/s"]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for s in sizes:
        for k in KINDS:
            cond = PROJECT_ROOT / "configs/stage1" / f"{k}_r{s:.2f}.yaml"
            if not cond.is_file():
                continue
            from comfort_core.comfort_space import geometry_record
            cfg = load_config(scenario=str(cond), common_path=campaign)
            g = geometry_record(cfg)
            lines.append("| " + " | ".join([
                cond.stem, fmt(float(g["comfort_size_m"])), fmt(float(g["comfort_scale"]), 4),
                f"{g['cs_engaged_front_m']:.2f}/{g['cs_engaged_side_m']:.2f}/{g['cs_engaged_rear_m']:.2f}",
                f"{g['cs_stationary_front_m']:.2f}/{g['cs_stationary_side_m']:.2f}/{g['cs_stationary_rear_m']:.2f}",
                f"{g['cs_moving_front_m']:.2f}/{g['cs_moving_side_m']:.2f}/{g['cs_moving_rear_m']:.2f}",
            ]) + " |")
    return "\n".join(lines)


def listing(cells, k, s, mech) -> str:
    rs = [r for r in cells[k].get(s, []) if r["_mech"] == mech]
    return ", ".join(f"s{r['_seed']}" for r in sorted(rs, key=lambda r: r["_seed"] or 0)) or "-"


# ---------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--onset-count", type=int, default=1,
                    help="comfort deadlocks at a size that count as the onset (default 1)")
    ap.add_argument("--campaign", default="configs/campaigns/cross/campaign_h8_a3_cross_s1.yaml",
                    help="config the geometry figure/table is evaluated on")
    args = ap.parse_args()
    d = Path(args.dir)
    folder = d.name
    cells, fcells, skipped, rows = load(d)
    if not rows:
        print(f"[stage1_report] no trials.csv rows in {d}")
        return 1
    sizes = sizes_of(cells)
    if not sizes:
        print("[stage1_report] no size-condition rows (scenario names like static_r1.00)")
        return 1
    figs = d / "figures"
    figs.mkdir(exist_ok=True)

    M = {k: {s: cell_metrics(cells[k][s], fcells[k].get(s, {})) for s in cells[k]} for k in KINDS}
    FM = {k: {s: {robot: fleet_metrics(frs) for robot, frs in fcells[k][s].items()}
              for s in fcells[k]} for k in KINDS}
    # Two readings of the same counts. `thr_any` is the literal rule (any
    # gate hold at a size); `thr` excludes seeds that already hold at the
    # smallest size and is the one the report and P2 lead with.
    thr_any = {k: thresholds(sizes, {s: M[k][s]["comfort_deadlock_n"] for s in M[k]},
                             {s: M[k][s]["n_valid"] for s in M[k]}, args.onset_count) for k in KINDS}
    thr = {k: transition_thresholds(sizes, cells[k], args.onset_count) for k in KINDS}
    (d / "thresholds.json").write_text(json.dumps(
        {KIND_LABEL[k]: {"size_induced": thr[k], "any_gate_hold": thr_any[k]} for k in KINDS},
        indent=2), encoding="utf-8")

    # figures
    plot_rate(sizes, M, "success_rate", "AMR1 success rate (valid trials)",
              "P1  Comfort Space size vs navigation success", figs / "P1_success_vs_size.png", folder)
    plot_rate(sizes, M, "comfort_deadlock_rate", "comfort-induced deadlock rate (gate hold)",
              "P2  Comfort Space size vs comfort-induced deadlock, with thresholds",
              figs / "P2_comfort_deadlock_vs_size.png", folder, thr=thr)
    plot_measure(sizes, M, "travel_time_mean", "travel_time_sd", "AMR1 travel time (s)",
                 "P3  Comfort Space size vs travel time (successful trials)",
                 figs / "P3_travel_time_vs_size.png", folder)
    plot_measure(sizes, M, "travel_distance_mean", "travel_distance_sd", "AMR1 travel distance (m)",
                 "P4  Comfort Space size vs travel distance (successful trials; dashed: first plan)",
                 figs / "P4_travel_distance_vs_size.png", folder, extra="planned_path_mean")
    plot_measure(sizes, M, "min_clearance_mean", None, "closest footprint-to-person distance (m)",
                 "P5  Comfort Space size vs minimum human clearance (mean; dashed: worst trial)",
                 figs / "P5_min_clearance_vs_size.png", folder, extra="min_clearance_min")
    plot_matrix(sizes, cells, figs / "P6_size_x_seed_matrix.png", folder)
    plot_geometry(sizes, figs / "P7_threshold_geometry.png", folder, args.campaign)
    plot_mechanisms(sizes, M, figs / "P8_failure_mechanism.png", folder)
    has_fleet = plot_fleet(sizes, FM, figs / "P9_fleet_vs_size.png", folder)

    # tables.md
    t = [f"# {folder}: Stage 1 tables", "",
         f"{len(rows)} AMR1 rows ({skipped} not size conditions, ignored); sizes "
         f"{', '.join(f'{s:.2f}' for s in sizes)} m.", "",
         "Travel time/distance: mean ± sd over SUCCESSFUL trials. Clearance, intrusion, "
         "contact, waits: over all valid trials. 'comfort DL' = Comfort gate hold only. "
         "'robot in' = trials where the robot closed the last centimetre of a contact. "
         "'r-r gap' = AMR1's closest centre-to-centre approach to a1/a2 (min over trials). "
         "Intrusion is measured against the fixed reference geometry "
         f"(evaluation.comfort_reference = {rows[0].get('comfort_reference', '?')}), "
         "identical for every size.", ""]
    for k in KINDS:
        t += [f"## {KIND_LABEL[k]} — AMR1 by size", "", table_sizes(sizes, M, k), "",
              f"### {KIND_LABEL[k]} — failure mechanism by size", "", table_mech(sizes, M, k), ""]
        for m in ("comfort_gate_hold", "post_evade_stall", "nav2_abort", "stall", "invalid"):
            items = [f"{s:.2f}: {listing(cells, k, s, m)}" for s in sizes if s in M[k]
                     and M[k][s]["mechanisms"].get(m)]
            if items:
                t.append(f"- {MECH_LABEL[m]} seeds — " + "; ".join(items))
        t.append("")
        if has_fleet:
            for robot in sorted({r for s in FM[k] for r in FM[k][s]}):
                t += [f"### {KIND_LABEL[k]} — {robot} by size", "", table_fleet(sizes, FM, k, robot), ""]
    t += ["## Applied Comfort Space extents (m)", "", table_geometry(sizes, args.campaign), ""]
    (d / "tables.md").write_text("\n".join(t), encoding="utf-8")

    # REPORT.md
    n_total = sum(M[k][s]["n"] for k in KINDS for s in M[k])
    r = [f"# {folder}: Stage 1 — Comfort Space size vs navigation feasibility", "",
         f"Cross scenario, {n_total} AMR1 trials over {len(sizes)} sizes x {len(KINDS)} policies; "
         f"seeds per cell: " + ", ".join(
             f"{KIND_LABEL[k]} {min(M[k][s]['n'] for s in M[k])}-{max(M[k][s]['n'] for s in M[k])}"
             for k in KINDS if M[k]) + ".", "",
         "Same campaign config for every condition of a seed: people, walks, state transitions, "
         "AMR missions and start times, map and Nav2 parameters are identical; only the "
         "`comfort_space` overlay differs (`configs/stage1/`). Physical safety layer unchanged "
         "(scripts/check_stage1.py group 4).", "",
         "## Thresholds", "",
         "Definition: only `comfort_gate_hold` (the yield gate holding the robot out of a "
         "Comfort Space when the 12 s no-progress rule fired) is a Comfort-Space-induced "
         "deadlock. R_deadlock_onset = smallest size with >= "
         f"{args.onset_count} such deadlock(s); R_nav_max = largest size below it with none.", "",
         "**Size-induced (primary).** Seeds that already end in a gate hold at the smallest "
         f"swept size ({sizes[0]:.2f} m) are impassable for the policy regardless of size "
         "(scenario-intrinsic) and are excluded; for the other seeds every gate hold is a "
         "success -> hold transition caused by growing the space.", ""]
    r += ["| policy | R_nav_max | R_deadlock_onset | size-induced gate holds by size (n) | excluded seeds (hold at smallest size) | non-monotonic seeds | note |",
          "|---|---|---|---|---|---|---|"]
    for k in KINDS:
        tk = thr[k]
        by = ", ".join(f"{s}: {c}/{tk['seeds_by_size'][s]}" for s, c in tk["size_induced_by_size"].items())
        nm = "; ".join(f"seed {sd} passes again at {', '.join(v)}" for sd, v in tk["non_monotonic_seeds"].items()) or "-"
        r.append(f"| {KIND_LABEL[k]} | "
                 f"{fmt(tk['R_nav_max_m']) + ' m' if tk['R_nav_max_m'] is not None else 'below the sweep'} | "
                 f"{fmt(tk['R_deadlock_onset_m']) + ' m' if tk['R_deadlock_onset_m'] is not None else 'above the sweep'} | "
                 f"{by} | {', '.join(map(str, tk['excluded_seeds'])) or '-'} | {nm} | {tk['note']} |")
    r += ["", "**Any gate hold (literal rule, for reference).** Every comfort gate hold counts, "
          "including scenario-intrinsic ones; with a non-zero floor at the smallest size this "
          "reading collapses to the smallest size and says nothing about the size effect.", "",
          "| policy | R_nav_max | R_deadlock_onset | comfort deadlocks by size (n) | note |",
          "|---|---|---|---|---|"]
    for k in KINDS:
        tk = thr_any[k]
        by = ", ".join(f"{s}: {c}/{tk['trials_by_size'][s]}" for s, c in tk["comfort_deadlocks_by_size"].items())
        r.append(f"| {KIND_LABEL[k]} | "
                 f"{fmt(tk['R_nav_max_m']) + ' m' if tk['R_nav_max_m'] is not None else 'below the sweep'} | "
                 f"{fmt(tk['R_deadlock_onset_m']) + ' m' if tk['R_deadlock_onset_m'] is not None else 'above the sweep'} | "
                 f"{by} | {tk['note']}"
                 + (f"; zero again at {tk['non_monotonic_sizes']} (non-monotonic)" if tk["non_monotonic_sizes"] else "")
                 + " |")
    r += ["", "## Summary by size", ""]
    cols = ["size (m)"] + [f"{KIND_LABEL[k]} success" for k in KINDS] + \
           [f"{KIND_LABEL[k]} comfort DL" for k in KINDS] + [f"{KIND_LABEL[k]} other fail" for k in KINDS] + \
           [f"{KIND_LABEL[k]} travel s" for k in KINDS] + [f"{KIND_LABEL[k]} min clr m" for k in KINDS]
    r += ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for s in sizes:
        cells_ = []
        for group_ in (lambda m: f"{m['n_success']}/{m['n_valid']}",
                       lambda m: str(m["comfort_deadlock_n"]),
                       lambda m: str(m["total_deadlock_n"] - m["comfort_deadlock_n"]),
                       lambda m: fmt(m["travel_time_mean"], 1),
                       lambda m: fmt(m["min_clearance_mean"])):
            for k in KINDS:
                cells_.append(group_(M[k][s]) if s in M[k] else "-")
        r.append(f"| {s:.2f} | " + " | ".join(cells_) + " |")
    r += ["", "Full per-size, per-mechanism and per-AMR (a1, a2) tables: `tables.md`. "
          "Thresholds as data: `thresholds.json`.", "",
          "## Figures", "",
          "| | file | shows |", "|---|---|---|",
          "| P1 | figures/P1_success_vs_size.png | success rate vs size, both policies, ±1 SE |",
          "| P2 | figures/P2_comfort_deadlock_vs_size.png | comfort-induced deadlock rate vs size; onset (dashed) and R_nav_max (dotted) |",
          "| P3 | figures/P3_travel_time_vs_size.png | travel time of successful trials |",
          "| P4 | figures/P4_travel_distance_vs_size.png | distance driven vs first Nav2 plan |",
          "| P5 | figures/P5_min_clearance_vs_size.png | closest approach to a person |",
          "| P6 | figures/P6_size_x_seed_matrix.png | outcome per size x seed, mechanism lettered |",
          "| P7 | figures/P7_threshold_geometry.png | aisle width left outside the Comfort Spaces, per size |",
          "| P8 | figures/P8_failure_mechanism.png | how AMR1 stopped, per size |"]
    if has_fleet:
        r.append("| P9 | figures/P9_fleet_vs_size.png | a1 / a2 success vs size |")
    r += ["", "## Reading notes", "",
          "- A trial that did not run has no row; n is printed for every cell.",
          "- Travel time of a trial that did not reach the goal is time-to-stop and is excluded "
          "from the travel-time means.",
          "- Post-evade stalls and Nav2 aborts are reported beside the comfort deadlocks and never "
          "added into the threshold; they are faults of the evasion module / planner, seen under "
          "every policy in the earlier campaigns.",
          "- Static and B3-O share the size axis by the moving-front extent at 1.0 m/s (1.20 m "
          "unscaled); B3-O's other extents are smaller at every size (see tables.md, applied extents).",
          "- None of these sizes is an experimentally validated comfort distance; Stage 2 measures "
          "that. The physical safety layer is identical in every trial.", ""]
    (d / "REPORT.md").write_text("\n".join(r), encoding="utf-8")

    print(f"[stage1_report] {n_total} trials; wrote {d / 'REPORT.md'}, tables.md, thresholds.json, "
          f"figures/P1-P{'9' if has_fleet else '8'}")
    for k in KINDS:
        tk, ta = thr[k], thr_any[k]
        print(f"  {KIND_LABEL[k]:7s} size-induced: R_nav_max={fmt(tk['R_nav_max_m'])} "
              f"onset={fmt(tk['R_deadlock_onset_m'])} by size {tk['size_induced_by_size']} "
              f"excluded seeds {tk['excluded_seeds']}")
        print(f"  {'':7s} any gate hold: R_nav_max={fmt(ta['R_nav_max_m'])} "
              f"onset={fmt(ta['R_deadlock_onset_m'])} by size {ta['comfort_deadlocks_by_size']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
