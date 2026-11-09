#!/usr/bin/env python3
"""Tables and figures for a policy x AMR-count ladder (261105_MAIN_pilot_policy_x_amrs_seeds1-8 and alike).

    python3 scripts/report_pilot.py --dir results/batches/261105_MAIN_pilot_policy_x_amrs_seeds1-8

Reads <dir>/trials.csv (AMR1), <dir>/fleet_trials.csv (the other AMRs)
and <dir>/logs/*.log (per-person closest approach, which the csv does
not carry), writes <dir>/tables.md and <dir>/figures/P1..P6.png.

What each figure is for:

  P1  AMR1 outcome by policy, one panel per AMR count. The deadlock claim.
  P2  Time cost of comfort: travel time minus B1's on the same seed,
      successes only, so a deadlock never enters a mean as a number.
  P3  Fraction of trials arrived by t. Shows the deadlocks as a curve
      that stops rising instead of inventing a timeout value for them.
  P4  Closest approach to belt workers and to storage-side people, by
      policy. The comfort claim, measured independently of any policy's
      own Comfort Space.
  P5  The fleet: per-vehicle outcome and legs/min by policy and AMR count.
  P6  Geometry: how far each policy's Comfort Space reaches into the
      1.5 m aisle from where the people stand, against the width the
      robot needs. The prediction behind P1.

Rows written before scripts/nav2_goal.py had GOAL_POSE_SLACK_M are
corrected the same way scripts/report_campaign.py does, and listed.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

POLICIES = ["b1", "b2", "b3", "b3d"]
POLICY_LABEL = {"b1": "B1 none", "b2": "B2 static", "b3": "B3 state", "b3d": "B3-O directional"}
POLICY_COLOUR = {"b1": "#8a8a8a", "b2": "#d9534f", "b3": "#5b9bd5", "b3d": "#2e8b57"}

GOAL_TOLERANCE_M = 0.25
GOAL_POSE_SLACK_M = 0.05


def classify(config: str) -> Dict[str, Any]:
    name = Path(config).stem
    out = {"seed": None, "humans": None, "amrs": None, "config": name}
    m = re.search(r"h(\d+)_a(\d+)_\w+?_s(\d+)$", name)
    if m:
        out.update(humans=int(m.group(1)), amrs=int(m.group(2)), seed=int(m.group(3)))
        return out
    m = re.match(r"campaign_h(\d+)_s(\d+)_a(\d+)$", name)
    if m:
        out.update(humans=int(m.group(1)), seed=int(m.group(2)), amrs=int(m.group(3)))
    return out


def read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as h:
        return list(csv.DictReader(h))


def f(v: Any, default: float = math.nan) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def truthy(v: Any) -> bool:
    return str(v).strip().lower() in ("true", "1", "yes")


def correct_goal_rows(rows: List[Dict[str, str]]) -> List[str]:
    fixed = []
    for r in rows:
        if (r.get("result") == "DEADLOCK" and not truthy(r.get("deadlock"))
                and not truthy(r.get("goal_reached"))
                and f(r.get("final_distance_to_goal")) <= GOAL_TOLERANCE_M + GOAL_POSE_SLACK_M):
            r["result"] = "SUCCESS"
            r["goal_reached"] = "True"
            fixed.append(f"{Path(r.get('config', '')).stem} {r.get('scenario')} "
                         f"({f(r.get('final_distance_to_goal')):.3f} m)")
    return fixed


CLOSEST_RE = re.compile(r"closest to (\w+)\s*:\s*([+-]?\d+\.\d+) m")


def closest_from_log(folder: Path, row: Dict[str, str]) -> Dict[str, float]:
    """{person: closest approach m} parsed from the trial's log."""
    stem = Path(row["config"]).stem
    hits = sorted(folder.glob(f"logs/*_{stem}_{row['scenario']}.log"))
    if not hits:
        return {}
    out: Dict[str, float] = {}
    for line in hits[-1].read_text(errors="replace").splitlines():
        m = CLOSEST_RE.search(line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def rate(rows, pred) -> float:
    return (sum(1 for r in rows if pred(r)) / len(rows)) if rows else math.nan


def mean(xs: List[float]) -> float:
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else math.nan


def median(xs: List[float]) -> float:
    xs = sorted(x for x in xs if not math.isnan(x))
    if not xs:
        return math.nan
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def grouped_bars(ax, series: Dict[str, Dict[str, float]], ylabel: str, title: str,
                 ylim=None, fmt="{:.2f}") -> None:
    names = list(series)
    width = 0.8 / max(len(names), 1)
    for i, s in enumerate(names):
        xs = [j + i * width for j in range(len(POLICIES))]
        ys = [series[s].get(p, math.nan) for p in POLICIES]
        alpha = 0.35 + 0.65 * (i + 1) / len(names)
        bars = ax.bar(xs, ys, width, label=s, color=[POLICY_COLOUR[p] for p in POLICIES],
                      alpha=alpha, edgecolor="black", linewidth=0.5)
        for b, y in zip(bars, ys):
            if not math.isnan(y):
                ax.text(b.get_x() + b.get_width() / 2, b.get_height(), fmt.format(y),
                        ha="center", va="bottom", fontsize=7)
    ax.set_xticks([j + width * (len(names) - 1) / 2 for j in range(len(POLICIES))])
    ax.set_xticklabels([POLICY_LABEL[p] for p in POLICIES], fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    if ylim:
        ax.set_ylim(*ylim)
    if len(names) > 1:
        ax.legend(fontsize=7, title="AMRs", title_fontsize=7)
    ax.grid(axis="y", alpha=0.3)


def _find(node: Any, pred) -> Any:
    """First value anywhere in a nested yaml structure that satisfies pred."""
    if pred(node):
        return node
    if isinstance(node, dict):
        for v in node.values():
            hit = _find(v, pred)
            if hit is not None:
                return hit
    elif isinstance(node, list):
        for v in node:
            hit = _find(v, pred)
            if hit is not None:
                return hit
    return None


def geometry() -> Dict[str, Any]:
    """Aisle and Comfort Space numbers from the configs, for P6."""
    common = yaml.safe_load((PROJECT_ROOT / "configs/common.yaml").read_text())
    factory = yaml.safe_load((PROJECT_ROOT / "configs/factory_u.yaml").read_text())
    cs = common["comfort_space"]
    d = cs["directional"]
    lanes = _find(factory, lambda n: isinstance(n, dict) and "north_y" in n)
    aisle_zone = _find(factory, lambda n: isinstance(n, dict) and n.get("name") == "aisle_north")
    aisle = float(aisle_zone["depth"]) if aisle_zone else 1.5
    lane_y = float(lanes["north_y"]) if lanes else 4.215
    inner_edge = lane_y - aisle / 2.0
    outer_edge = lane_y + aisle / 2.0
    worker_y = 3.01          # belt-side operators, scenario generator
    stander_y = 5.6          # storage-side people at the slot row
    return {
        "aisle": aisle,
        "robot_w": float(common["amr"]["footprint_width"]),
        "inflation": 0.34,
        "buffer": float(cs["costmap"].get("buffer_width", 0.0)),
        "worker_offset": inner_edge - worker_y,
        "stander_offset": stander_y - outer_edge,
        "reach": {
            # (belt worker facing away from the aisle, storage-side person facing it)
            "b2": (cs["static_radius"], cs["static_radius"]),
            "b3": (cs["state_radius"]["station_engaged"], cs["state_radius"]["stationary"]),
            "b3d": (d["station_engaged"]["rear_m"], d["stationary"]["front_m"]),
            "b3d (before 2026-11-05)": (0.95, 0.80),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", required=True)
    args = ap.parse_args()
    folder = Path(args.dir)
    figs = folder / "figures"
    figs.mkdir(parents=True, exist_ok=True)

    trials = read_csv(folder / "trials.csv")
    corrected = correct_goal_rows(trials)
    fleet = read_csv(folder / "fleet_trials.csv")
    for r in trials:
        r.update(classify(r["config"]))
        r["closest"] = closest_from_log(folder, r)
    for r in fleet:
        r.update(classify(r["config"]))
    counts = sorted({r["amrs"] for r in trials if r["amrs"]})
    lines: List[str] = [f"# {folder.name}: policy x AMR count\n",
                        f"{len(trials)} AMR1 rows, {len(fleet)} fleet vehicle rows, "
                        f"AMR counts {counts}\n"]
    if corrected:
        lines.append(f"{len(corrected)} AMR1 row(s) relabelled DEADLOCK -> SUCCESS (Nav2 SUCCEEDED, "
                     f"no deadlock detected, base within {GOAL_POSE_SLACK_M} m of the "
                     f"{GOAL_TOLERANCE_M} m tolerance; GOAL_POSE_SLACK_M in scripts/nav2_goal.py): "
                     + ", ".join(corrected) + "\n")

    def rows(amrs: int, policy: str) -> List[Dict[str, str]]:
        return [r for r in trials if r["amrs"] == amrs and r["scenario"] == policy]

    # ---- P1: AMR1 outcome ---------------------------------------------------
    lines.append("\n## AMR1 outcome by policy and AMR count\n")
    lines.append("| AMRs | policy | n | SUCCESS | DEADLOCK | other | deadlock causes |")
    lines.append("|---|---|---|---|---|---|---|")
    succ: Dict[str, Dict[str, float]] = {}
    dead: Dict[str, Dict[str, float]] = {}
    for a in counts:
        key = f"{a}"
        succ[key], dead[key] = {}, {}
        for p in POLICIES:
            rs = rows(a, p)
            if not rs:
                continue
            succ[key][p] = rate(rs, lambda r: r["result"] == "SUCCESS")
            dead[key][p] = rate(rs, lambda r: r["result"] == "DEADLOCK")
            causes = defaultdict(int)
            for r in rs:
                if r["result"] == "DEADLOCK":
                    c = r.get("deadlock_cause", "") or "(no cause recorded)"
                    causes[c.split(",")[0]] += 1
            lines.append(f"| {a} | {POLICY_LABEL[p]} | {len(rs)} | {succ[key][p]:.0%} | {dead[key][p]:.0%} | "
                         f"{1 - succ[key][p] - dead[key][p]:.0%} | "
                         + "; ".join(f"{k} x{v}" for k, v in causes.items()) + " |")
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    grouped_bars(axes[0], succ, "share of trials", "AMR1: SUCCESS rate", (0, 1.18), "{:.0%}")
    grouped_bars(axes[1], dead, "share of trials", "AMR1: DEADLOCK rate", (0, 1.18), "{:.0%}")
    fig.suptitle("AMR1 outcome by policy, one bar group per AMR count (8 people)", fontsize=11)
    fig.tight_layout()
    fig.savefig(figs / "P1_amr1_outcome.png", dpi=130)
    plt.close(fig)

    # ---- P2: time cost of comfort, paired by seed ---------------------------
    lines.append("\n## Travel time (successes only) and its cost against B1 on the same seed\n")
    lines.append("| AMRs | policy | successes | travel time s (mean) | paired seeds | time - B1 s (mean) | (median) | (max) |")
    lines.append("|---|---|---|---|---|---|---|---|")
    deltas: Dict[int, Dict[str, List[float]]] = {}
    for a in counts:
        deltas[a] = {}
        base = {r["seed"]: f(r["travel_time"]) for r in rows(a, "b1") if r["result"] == "SUCCESS"}
        for p in POLICIES:
            rs = [r for r in rows(a, p) if r["result"] == "SUCCESS"]
            tt = [f(r["travel_time"]) for r in rs]
            d = [f(r["travel_time"]) - base[r["seed"]] for r in rs if r["seed"] in base]
            deltas[a][p] = d
            if rs:
                lines.append(f"| {a} | {POLICY_LABEL[p]} | {len(rs)} | {mean(tt):.1f} | {len(d)} | "
                             f"{mean(d):+.1f} | {median(d):+.1f} | {max(d) if d else math.nan:+.1f} |")
    fig, axes = plt.subplots(1, len(counts), figsize=(4.2 * len(counts), 4.2), squeeze=False)
    for ax, a in zip(axes[0], counts):
        for i, p in enumerate(POLICIES[1:], start=1):
            d = deltas[a].get(p, [])
            ax.scatter([i + (k - len(d) / 2) * 0.05 for k in range(len(d))], d,
                       color=POLICY_COLOUR[p], s=26, zorder=3)
            if d:
                ax.hlines(mean(d), i - 0.3, i + 0.3, color=POLICY_COLOUR[p], linewidth=2)
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels([POLICY_LABEL[p] for p in POLICIES[1:]], fontsize=8)
        ax.set_ylabel("travel time - B1 on the same seed, s")
        ax.set_title(f"{a} AMR{'s' if a > 1 else ''}: time cost of comfort (successes)", fontsize=10)
        ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figs / "P2_time_cost_paired.png", dpi=130)
    plt.close(fig)

    # ---- P3: arrival curves -------------------------------------------------
    fig, axes = plt.subplots(1, len(counts), figsize=(4.4 * len(counts), 4.0), squeeze=False)
    for ax, a in zip(axes[0], counts):
        tmax = 0.0
        for p in POLICIES:
            rs = rows(a, p)
            if not rs:
                continue
            times = sorted(f(r["travel_time"]) for r in rs if r["result"] == "SUCCESS")
            tmax = max([tmax] + times)
            xs = [0.0] + times + [1e9]
            ys = [0.0] + [(i + 1) / len(rs) for i in range(len(times))] + [len(times) / len(rs)]
            ax.step(xs, ys, where="post", color=POLICY_COLOUR[p], label=f"{POLICY_LABEL[p]} (n={len(rs)})", linewidth=2)
        ax.set_xlim(0, max(tmax * 1.25, 10))
        ax.set_ylim(0, 1.05)
        ax.set_xlabel("simulated s since start")
        ax.set_ylabel("share of trials arrived")
        ax.set_title(f"{a} AMR{'s' if a > 1 else ''}: AMR1 arrival curve", fontsize=10)
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(figs / "P3_arrival_curves.png", dpi=130)
    plt.close(fig)

    # ---- P4: closest approach by whom ---------------------------------------
    lines.append("\n## AMR1: closest approach to people, by whom (m, mean over trials of the per-trial minimum)\n")
    lines.append("| AMRs | policy | n | belt workers (N/S) | storage-side people (E) | any person | contact: robot drove in |")
    lines.append("|---|---|---|---|---|---|---|")
    near_w: Dict[str, Dict[str, float]] = {}
    near_e: Dict[str, Dict[str, float]] = {}
    for a in counts:
        key = f"{a}"
        near_w[key], near_e[key] = {}, {}
        for p in POLICIES:
            rs = rows(a, p)
            if not rs:
                continue
            w = [min([v for k, v in r["closest"].items() if k[0] in "NS"] or [math.nan]) for r in rs]
            e = [min([v for k, v in r["closest"].items() if k[0] == "E"] or [math.nan]) for r in rs]
            near_w[key][p] = mean(w)
            near_e[key][p] = mean(e)
            lines.append(f"| {a} | {POLICY_LABEL[p]} | {len(rs)} | {near_w[key][p]:.2f} | {near_e[key][p]:.2f} | "
                         f"{mean([f(r.get('min_clearance')) for r in rs]):.2f} | "
                         f"{rate(rs, lambda r: f(r.get('contact_robot_seconds'), 0) > 0):.0%} |")
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    grouped_bars(axes[0], near_w, "m", "AMR1: closest approach to a belt worker (N/S)")
    grouped_bars(axes[1], near_e, "m", "AMR1: closest approach to a storage-side person (E)")
    fig.suptitle("Comfort measured without any policy's own Comfort Space: how close the robot came", fontsize=11)
    fig.tight_layout()
    fig.savefig(figs / "P4_closest_by_whom.png", dpi=130)
    plt.close(fig)

    # ---- P5: the fleet ------------------------------------------------------
    fcounts = sorted({r["amrs"] for r in fleet if r["amrs"]})
    if fcounts:
        lines.append("\n## Fleet vehicles by policy and AMR count\n")
        lines.append("| AMRs | policy | vehicle rows | SUCCESS | DEADLOCK | ABORTED | legs/min (mean) | min clearance m (mean) | robot drove in | nearest other robot m (min) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        fs: Dict[str, Dict[str, float]] = {}
        fd: Dict[str, Dict[str, float]] = {}
        fl: Dict[str, Dict[str, float]] = {}
        for a in fcounts:
            key = f"{a}"
            fs[key], fd[key], fl[key] = {}, {}, {}
            for p in POLICIES:
                rs = [r for r in fleet if r["amrs"] == a and r["scenario"] == p]
                if not rs:
                    continue
                fs[key][p] = rate(rs, lambda r: r["result"] == "SUCCESS")
                fd[key][p] = rate(rs, lambda r: r["result"] == "DEADLOCK")
                lpm = [f(r["legs_done"]) / (f(r["observed_s"]) / 60.0) for r in rs if f(r["observed_s"]) > 0]
                fl[key][p] = mean(lpm)
                gaps = [min(f(r["min_gap_amr1_m"], 99), f(r["min_gap_fleet_m"], 99)) for r in rs]
                lines.append(
                    f"| {a} | {POLICY_LABEL[p]} | {len(rs)} | {fs[key][p]:.0%} | {fd[key][p]:.0%} | "
                    f"{rate(rs, lambda r: r['result'] == 'ABORTED'):.0%} | {fl[key][p]:.2f} | "
                    f"{mean([f(r['min_clearance_m']) for r in rs]):.2f} | "
                    f"{rate(rs, lambda r: f(r['contact_robot_s'], 0) > 0):.0%} | {min(gaps) if gaps else math.nan:.2f} |")
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
        grouped_bars(axes[0], fs, "share of vehicle rows", "Fleet: SUCCESS rate", (0, 1.18), "{:.0%}")
        grouped_bars(axes[1], fd, "share of vehicle rows", "Fleet: DEADLOCK rate", (0, 1.18), "{:.0%}")
        grouped_bars(axes[2], fl, "legs / min", "Fleet: throughput while observed")
        fig.suptitle("The other AMRs by policy and fleet size (8 people)", fontsize=11)
        fig.tight_layout()
        fig.savefig(figs / "P5_fleet.png", dpi=130)
        plt.close(fig)

    # ---- P6: geometry -------------------------------------------------------
    g = geometry()
    lines.append("\n## Geometry: Comfort Space reach into the aisle (from the configs)\n")
    lines.append(f"aisle {g['aisle']:.2f} m, robot {g['robot_w']:.2f} m wide, inflation {g['inflation']:.2f} m, "
                 f"buffer band outside the Comfort Space {g['buffer']:.2f} m; belt worker centre "
                 f"{g['worker_offset']:.3f} m outside the aisle edge, storage-side stander "
                 f"{g['stander_offset']:.3f} m outside the far edge.\n")
    lines.append("| policy | belt worker: reach into aisle m | + buffer | storage-side stander: reach m | + buffer |")
    lines.append("|---|---|---|---|---|")
    fig, ax = plt.subplots(figsize=(9, 4.4))
    names = list(g["reach"])
    for i, name in enumerate(names):
        rw, rs_ = g["reach"][name]
        reach_w = max(0.0, rw - g["worker_offset"])
        reach_s = max(0.0, rs_ - g["stander_offset"])
        lines.append(f"| {name} | {reach_w:.2f} | {reach_w + g['buffer']:.2f} | {reach_s:.2f} | {reach_s + g['buffer']:.2f} |")
        colour = POLICY_COLOUR.get(name.split()[0], "#444444")
        for j, (reach, lab) in enumerate(((reach_w, "belt worker"), (reach_s, "storage-side stander"))):
            x = i + (j - 0.5) * 0.36
            ax.bar(x, reach, 0.32, color=colour, alpha=1.0 if j == 0 else 0.55, edgecolor="black", linewidth=0.5,
                   hatch="" if j == 0 else "//")
            ax.bar(x, g["buffer"], 0.32, bottom=reach, color=colour, alpha=0.25, edgecolor="black",
                   linewidth=0.5, linestyle=":")
    free_body = g["aisle"] - g["robot_w"]
    free_cost = free_body - g["inflation"]
    ax.axhline(free_body, color="black", linewidth=1.2)
    ax.text(len(names) - 0.5, free_body + 0.01, f"aisle - robot width = {free_body:.2f} m", ha="right", fontsize=8)
    ax.axhline(free_cost, color="black", linewidth=1.2, linestyle="--")
    ax.text(len(names) - 0.5, free_cost + 0.01, f"... - far-wall inflation = {free_cost:.2f} m", ha="right", fontsize=8)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels([POLICY_LABEL.get(n, n) for n in names], fontsize=8)
    ax.set_ylabel("reach into the aisle, m")
    ax.set_title("Comfort Space reach into the 1.5 m aisle (solid: belt worker, hatched: storage-side stander; "
                 "faint cap: soft buffer band)", fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figs / "P6_geometry.png", dpi=130)
    plt.close(fig)

    (folder / "tables.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\nfigures in {figs}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
