#!/usr/bin/env python3
"""Tables and figures for a campaign folder.

    python3 scripts/report_campaign.py --dir results/campaign_261105

Reads trials.csv (AMR1, one row per trial), fleet_trials.csv (one row
per fleet vehicle per trial) and the trial JSON details, classifies
every row by what the config name says -- seed, people, AMRs, fleet
pattern, keep-right rule -- and writes <dir>/figures/*.png and
<dir>/tables.md. The narrative (what each hypothesis expected and
whether it held) is written by hand around these numbers; this script
only counts and draws.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from comfort_core.palette import MISSION_COLOUR, fleet_colour  # noqa: E402

POLICIES = ["b1", "b2", "b3", "b3d"]
POLICY_LABEL = {"b1": "B1 none", "b2": "B2 static", "b3": "B3 state", "b3d": "B3-O directional"}
POLICY_COLOUR = {"b1": "#8a8a8a", "b2": "#d9534f", "b3": "#5b9bd5", "b3d": "#2e8b57"}


def classify(config: str) -> Dict[str, Any]:
    """What a config path says about the trial it ran."""
    name = Path(config).stem
    out = {"seed": None, "humans": None, "amrs": None, "pattern": "roundtrip",
           "rule": "off", "kind": "other", "config": name}
    m = re.search(r"h(\d+)_a(\d+)_\w+?_s(\d+)(?:_(loop|ring))?$", name)
    if m:
        out.update(humans=int(m.group(1)), amrs=int(m.group(2)), seed=int(m.group(3)),
                   pattern=m.group(4) or "roundtrip", kind="paper")
        return out
    m = re.match(r"campaign_h(\d+)_s(\d+)_(a4|loop|ring)(?:_(norule|rule))?$", name)
    if m:
        out.update(humans=int(m.group(1)), seed=int(m.group(2)), amrs=4,
                   pattern={"a4": "roundtrip"}.get(m.group(3), m.group(3)),
                   rule="on" if m.group(4) == "rule" else "off", kind="campaign")
    return out


def read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as h:
        return list(csv.DictReader(h))


# Trials recorded before scripts/nav2_goal.py gained GOAL_POSE_SLACK_M:
# Nav2 reported SUCCEEDED, no deadlock was detected, and the base was
# measured a few millimetres outside goal_tolerance, so the row says
# DEADLOCK. The raw csv is left as written; the correction is applied
# here, counted, and listed in tables.md so it is on the record.
GOAL_TOLERANCE_M = 0.25
GOAL_POSE_SLACK_M = 0.05


def correct_goal_rows(rows: List[Dict[str, str]]) -> List[str]:
    fixed = []
    for r in rows:
        if (r.get("result") == "DEADLOCK"
                and r.get("deadlock", "").lower() not in ("true", "1")
                and r.get("goal_reached", "").lower() not in ("true", "1")
                and f(r.get("final_distance_to_goal")) <= GOAL_TOLERANCE_M + GOAL_POSE_SLACK_M):
            r["result"] = "SUCCESS"
            r["goal_reached"] = "True"
            fixed.append(f"{Path(r.get('config', '')).stem} {r.get('scenario')} "
                         f"({f(r.get('final_distance_to_goal')):.3f} m)")
    return fixed


def f(v: Any, default: float = math.nan) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def detail_for(folder: Path, row: Dict[str, str]) -> Optional[Dict[str, Any]]:
    """The trial's JSON detail, matched by stamp and scenario."""
    stamp = row["stamp"].replace(":", "").replace("-", "")
    for p in folder.glob(f"{stamp}_{row['scenario']}_*.json"):
        try:
            return json.loads(p.read_text())
        except Exception:
            return None
    return None


def rate(rows, pred) -> float:
    return (sum(1 for r in rows if pred(r)) / len(rows)) if rows else math.nan


def bar_by_policy(ax, groups: Dict[str, Dict[str, float]], ylabel: str, title: str,
                  ylim=None, fmt="{:.2f}") -> None:
    """groups: series -> {policy: value}; grouped bars."""
    series = list(groups)
    width = 0.8 / max(len(series), 1)
    for i, s in enumerate(series):
        xs = [j + i * width for j in range(len(POLICIES))]
        ys = [groups[s].get(p, math.nan) for p in POLICIES]
        bars = ax.bar(xs, ys, width, label=s,
                      color=[POLICY_COLOUR[p] for p in POLICIES],
                      alpha=1.0 if i == len(series) - 1 else 0.45,
                      edgecolor="black" if i == len(series) - 1 else "none")
        for b, y in zip(bars, ys):
            if not math.isnan(y):
                ax.text(b.get_x() + b.get_width() / 2, b.get_height(), fmt.format(y),
                        ha="center", va="bottom", fontsize=8)
    ax.set_xticks([j + width * (len(series) - 1) / 2 for j in range(len(POLICIES))])
    ax.set_xticklabels([POLICY_LABEL[p] for p in POLICIES], fontsize=9)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=10)
    if ylim:
        ax.set_ylim(*ylim)
    if len(series) > 1:
        ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)


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
        r["detail"] = detail_for(folder, r)
    for r in fleet:
        r.update(classify(r["config"]))
    lines: List[str] = [f"# {folder.name}: counts and figures\n",
                        f"{len(trials)} AMR1 rows, {len(fleet)} fleet rows\n"]
    if corrected:
        lines.append(f"{len(corrected)} AMR1 row(s) relabelled DEADLOCK -> SUCCESS: Nav2 "
                     f"reported SUCCEEDED, no deadlock was detected, and the base was within "
                     f"{GOAL_POSE_SLACK_M} m of the {GOAL_TOLERANCE_M} m tolerance "
                     f"(see GOAL_POSE_SLACK_M in scripts/nav2_goal.py): "
                     + ", ".join(corrected) + "\n")

    # ------------------------------------------------------------------
    # H1/H2 + H4 (AMR1 side): outcome, intrusion, clearance by policy,
    # one AMR vs four (roundtrip, people present)
    # ------------------------------------------------------------------
    def amr1_rows(amrs, pattern="roundtrip", humans=8, rule=None):
        return [r for r in trials if r["amrs"] == amrs and r["pattern"] == pattern
                and r["humans"] == humans and (rule is None or r["rule"] == rule)]

    series = {"1 AMR": amr1_rows(1), "4 AMRs (roundtrip)": amr1_rows(4)}
    lines.append("\n## AMR1 by policy (people present)\n")
    lines.append("| setting | policy | n | SUCCESS | DEADLOCK | contact: robot drove in | worst intrusion m (mean) | comfort s (mean) | min clearance m (mean) | slowdown s (mean) |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    succ, dead, intr, clr = {}, {}, {}, {}
    for s, rows in series.items():
        succ[s], dead[s], intr[s], clr[s] = {}, {}, {}, {}
        for p in POLICIES:
            rs = [r for r in rows if r["scenario"] == p]
            if not rs:
                continue
            succ[s][p] = rate(rs, lambda r: r["result"] == "SUCCESS")
            dead[s][p] = rate(rs, lambda r: r["result"] == "DEADLOCK")
            vals = [f(r["worst_intrusion"]) for r in rs]
            intr[s][p] = sum(vals) / len(vals)
            cl = [f(r.get("min_clearance", "nan")) for r in rs]
            cl = [v for v in cl if not math.isnan(v)]
            clr[s][p] = sum(cl) / len(cl) if cl else math.nan
            lines.append(
                f"| {s} | {POLICY_LABEL[p]} | {len(rs)} | {succ[s][p]:.0%} | {dead[s][p]:.0%} | "
                f"{rate(rs, lambda r: f(r.get('contact_robot_seconds', 0)) > 0):.0%} | "
                f"{intr[s][p]:.2f} | {sum(f(r.get('comfort_seconds_total', 0), 0) for r in rs) / len(rs):.1f} | "
                f"{clr[s][p]:.2f} | {sum(f(r.get('safety_slowdown_seconds', 0), 0) for r in rs) / len(rs):.1f} |")
    bar_by_policy(axes[0, 0], succ, "share of trials", "AMR1: SUCCESS rate", (0, 1.15), "{:.0%}")
    bar_by_policy(axes[0, 1], dead, "share of trials", "AMR1: DEADLOCK rate", (0, 1.15), "{:.0%}")
    bar_by_policy(axes[1, 0], intr, "m", "AMR1: worst intrusion into the reference Comfort Space (mean)")
    bar_by_policy(axes[1, 1], clr, "m", "AMR1: closest approach to any person (mean)")
    fig.suptitle("AMR1 by policy — one AMR vs four (roundtrip fleet), 8 people", fontsize=12)
    fig.tight_layout()
    fig.savefig(figs / "F1_amr1_by_policy.png", dpi=130)
    plt.close(fig)

    # ------------------------------------------------------------------
    # H2: where the worst intrusion was, seen from the person (bearing),
    # and against which state -- from the JSON details (AMR1) and the
    # fleet column
    # ------------------------------------------------------------------
    bearing = {p: defaultdict(int) for p in POLICIES}
    by_state = {p: defaultdict(list) for p in POLICIES}
    for r in trials:
        d = r.get("detail")
        if not d or r["humans"] != 8:
            continue
        worst = d.get("comfort_worst", {})
        for who, val in worst.items():
            if isinstance(val, (list, tuple)) and len(val) >= 4 and f(val[0]) > 0:
                bearing[r["scenario"]][str(val[3])] += 1
            if isinstance(val, (list, tuple)):
                state = "walker (E)" if who.startswith("E") else "belt worker (N/S)"
                by_state[r["scenario"]][state].append(f(val[0]))
    for r in fleet:
        if r["humans"] == 8 and f(r.get("worst_intrusion_m")) > 0 and r.get("worst_intrusion_bearing"):
            bearing[r["scenario"]][r["worst_intrusion_bearing"]] += 1
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    sectors = ["front", "side", "rear"]
    width = 0.2
    for i, p in enumerate(POLICIES):
        tot = sum(bearing[p].values()) or 1
        axes[0].bar([j + i * width for j in range(3)], [bearing[p][s] / tot for s in sectors], width,
                    label=POLICY_LABEL[p], color=POLICY_COLOUR[p])
    axes[0].set_xticks([j + 1.5 * width for j in range(3)])
    axes[0].set_xticklabels(sectors)
    axes[0].set_ylabel("share of intrusions (depth > 0)")
    axes[0].set_title("Where the deepest intrusion happened, seen from the person", fontsize=10)
    axes[0].legend(fontsize=8)
    axes[0].grid(axis="y", alpha=0.3)
    lines.append("\n## Bearing of the deepest intrusion (AMR1 + fleet, 8 people)\n")
    lines.append("| policy | front | side | rear |")
    lines.append("|---|---|---|---|")
    for p in POLICIES:
        lines.append(f"| {POLICY_LABEL[p]} | " + " | ".join(str(bearing[p][s]) for s in sectors) + " |")
    for i, p in enumerate(POLICIES):
        states = ["belt worker (N/S)", "walker (E)"]
        means = [sum(by_state[p][s]) / len(by_state[p][s]) if by_state[p][s] else math.nan for s in states]
        axes[1].bar([j + i * width for j in range(2)], means, width, label=POLICY_LABEL[p], color=POLICY_COLOUR[p])
    axes[1].set_xticks([j + 1.5 * width for j in range(2)])
    axes[1].set_xticklabels(["belt workers (N/S)", "storage-side walkers (E)"])
    axes[1].set_ylabel("m (mean over people and trials)")
    axes[1].set_title("AMR1: worst intrusion depth by whom", fontsize=10)
    axes[1].grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figs / "F2_intrusion_bearing_state.png", dpi=130)
    plt.close(fig)

    # ------------------------------------------------------------------
    # H4: the fleet with people -- per-vehicle outcome and throughput by
    # policy, roundtrip pattern
    # ------------------------------------------------------------------
    def fleet_rows(pattern, humans, rule=None):
        return [r for r in fleet if r["pattern"] == pattern and r["humans"] == humans
                and (rule is None or r["rule"] == rule)]

    lines.append("\n## Fleet vehicles by policy (4 AMRs, roundtrip, 8 people)\n")
    lines.append("| policy | vehicle rows | SUCCESS | DEADLOCK | ABORTED | legs/min (mean) | worst intrusion m (mean) | min clearance m (mean) | robot drove in | nearest other robot m (min) |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    fsucc, fdead, legs_min, fintr = {"fleet": {}}, {"fleet": {}}, {"fleet": {}}, {"fleet": {}}
    for p in POLICIES:
        rs = [r for r in fleet_rows("roundtrip", 8) if r["scenario"] == p]
        if not rs:
            continue
        fsucc["fleet"][p] = rate(rs, lambda r: r["result"] == "SUCCESS")
        fdead["fleet"][p] = rate(rs, lambda r: r["result"] == "DEADLOCK")
        lpm = [f(r["legs_done"]) / (f(r["observed_s"]) / 60.0) for r in rs if f(r["observed_s"]) > 0]
        legs_min["fleet"][p] = sum(lpm) / len(lpm) if lpm else math.nan
        fintr["fleet"][p] = sum(f(r["worst_intrusion_m"], 0) for r in rs) / len(rs)
        gaps = [min(f(r["min_gap_amr1_m"], 99), f(r["min_gap_fleet_m"], 99)) for r in rs]
        cl = [f(r["min_clearance_m"]) for r in rs if r["min_clearance_m"]]
        lines.append(
            f"| {POLICY_LABEL[p]} | {len(rs)} | {fsucc['fleet'][p]:.0%} | {fdead['fleet'][p]:.0%} | "
            f"{rate(rs, lambda r: r['result'] == 'ABORTED'):.0%} | {legs_min['fleet'][p]:.2f} | "
            f"{fintr['fleet'][p]:.2f} | {sum(cl) / len(cl) if cl else math.nan:.2f} | "
            f"{rate(rs, lambda r: f(r['contact_robot_s'], 0) > 0):.0%} | {min(gaps) if gaps else math.nan:.2f} |")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    bar_by_policy(axes[0, 0], fsucc, "share of vehicle rows", "Fleet: SUCCESS rate", (0, 1.15), "{:.0%}")
    bar_by_policy(axes[0, 1], fdead, "share of vehicle rows", "Fleet: DEADLOCK rate", (0, 1.15), "{:.0%}")
    bar_by_policy(axes[1, 0], legs_min, "legs / min", "Fleet: throughput while observed")
    bar_by_policy(axes[1, 1], fintr, "m", "Fleet: worst intrusion into the reference Comfort Space (mean)")
    fig.suptitle("The other three AMRs by policy — roundtrip pattern, 8 people", fontsize=12)
    fig.tight_layout()
    fig.savefig(figs / "F3_fleet_by_policy.png", dpi=130)
    plt.close(fig)

    # ------------------------------------------------------------------
    # H3/H5: traffic -- loop vs ring, rule off vs on, with and without
    # people: per-vehicle deadlock rate and nearest-robot distance
    # ------------------------------------------------------------------
    lines.append("\n## Traffic: pattern x people x keep-right\n")
    lines.append("| pattern | people | rule | policy | vehicle rows | DEADLOCK | SUCCESS | legs/min | nearest robot m (median) | AMR1 DEADLOCK |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    cells = []
    for pattern in ("loop", "ring"):
        for humans in (0, 8):
            for rule in ("off", "on"):
                rs = fleet_rows(pattern, humans, rule)
                if not rs:
                    continue
                for p in sorted({r["scenario"] for r in rs}):
                    rp = [r for r in rs if r["scenario"] == p]
                    gaps = sorted(min(f(r["min_gap_amr1_m"], 99), f(r["min_gap_fleet_m"], 99)) for r in rp)
                    med = gaps[len(gaps) // 2] if gaps else math.nan
                    lpm = [f(r["legs_done"]) / (f(r["observed_s"]) / 60.0) for r in rp if f(r["observed_s"]) > 0]
                    t1 = [r for r in trials if r["pattern"] == pattern and r["humans"] == humans
                          and r["rule"] == rule and r["scenario"] == p]
                    cells.append((pattern, humans, rule, p, len(rp),
                                  rate(rp, lambda r: r["result"] == "DEADLOCK"),
                                  rate(rp, lambda r: r["result"] == "SUCCESS"),
                                  sum(lpm) / len(lpm) if lpm else math.nan, med,
                                  rate(t1, lambda r: r["result"] == "DEADLOCK")))
                    lines.append("| " + " | ".join([pattern, str(humans), rule, POLICY_LABEL.get(p, p), str(len(rp)),
                                                    f"{cells[-1][5]:.0%}", f"{cells[-1][6]:.0%}", f"{cells[-1][7]:.2f}",
                                                    f"{med:.2f}", f"{cells[-1][9]:.0%}"]) + " |")
    if cells:
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
        labels = [f"{c[0]}\n{c[1]} ppl, rule {c[2]}\n{POLICY_LABEL.get(c[3], c[3])}" for c in cells]
        xs = range(len(cells))
        axes[0].bar(xs, [c[5] for c in cells], color=["#d9534f" if c[2] == "off" else "#5b9bd5" for c in cells])
        axes[0].set_xticks(list(xs)); axes[0].set_xticklabels(labels, fontsize=7)
        axes[0].set_ylim(0, 1.1); axes[0].set_ylabel("fleet DEADLOCK rate"); axes[0].grid(axis="y", alpha=0.3)
        axes[0].set_title("Robot traffic: deadlocks by pattern and keep-right", fontsize=10)
        axes[1].bar(xs, [c[8] for c in cells], color=["#d9534f" if c[2] == "off" else "#5b9bd5" for c in cells])
        axes[1].set_xticks(list(xs)); axes[1].set_xticklabels(labels, fontsize=7)
        axes[1].set_ylabel("m"); axes[1].grid(axis="y", alpha=0.3)
        axes[1].set_title("Nearest another robot came (median over vehicles)", fontsize=10)
        fig.tight_layout()
        fig.savefig(figs / "F4_traffic.png", dpi=130)
        plt.close(fig)

    # ------------------------------------------------------------------
    # distributions: min clearance and worst intrusion per trial, AMR1,
    # one AMR, all seeds -- the raw points behind F1
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    for i, p in enumerate(POLICIES):
        rs = [r for r in amr1_rows(1) if r["scenario"] == p]
        ys = [f(r["worst_intrusion"]) for r in rs]
        axes[0].scatter([i + (k - len(ys) / 2) * 0.04 for k in range(len(ys))], ys, color=POLICY_COLOUR[p], s=28)
        cl = [f(r.get("min_clearance", "nan")) for r in rs]
        axes[1].scatter([i + (k - len(cl) / 2) * 0.04 for k in range(len(cl))], cl, color=POLICY_COLOUR[p], s=28)
    for ax, title, yl in ((axes[0], "AMR1: worst intrusion per trial (1 AMR)", "m"),
                          (axes[1], "AMR1: closest approach per trial (1 AMR)", "m")):
        ax.set_xticks(range(4)); ax.set_xticklabels([POLICY_LABEL[p] for p in POLICIES], fontsize=9)
        ax.set_title(title, fontsize=10); ax.set_ylabel(yl); ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figs / "F5_amr1_points.png", dpi=130)
    plt.close(fig)

    (folder / "tables.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\nfigures in {figs}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
