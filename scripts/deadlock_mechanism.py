#!/usr/bin/env python3
"""Sort every AMR1 trial that did not reach its goal by HOW it stopped.

    python3 scripts/deadlock_mechanism.py --dir results/batches/261107_MAIN_cross_seeds1-33_merged

Reads <dir>/trials.csv and the trial logs' 2 s trace lines. One row per
failed trial with:

  mechanism   gate hold         YIELDING flag on the last trace lines: the
                                yield gate was holding the robot out of a
                                Comfort Space when the 12 s rule fired
              post-evade stall  the robot had stepped aside (evade > 0),
                                the evade had ended, a plan existed, and the
                                base produced no motion for 12 s
              nav2 abort        Nav2 gave the goal up (ABORTED), usually
                                after a person walked through the robot
              stall             no evade, no flag, no abort
              lost              watchdog / no result block
  where       refuge            y > aisle north edge: inside a pull-off slot
              belt edge         y < aisle edge + 0.15: pinned on the belt side
              crossing          |x| > 9.3: in a crossing
              aisle             anywhere else

Writes <dir>/deadlock_mechanism.md and figures/P12_deadlock_mechanism.png.
Plain Python 3 + matplotlib.
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from report_pilot import classify  # noqa: E402

POLICIES = ["none", "static", "state_adaptive", "state_directional"]
LABEL = {"none": "B1", "static": "B2", "state_adaptive": "B3", "state_directional": "B3-O"}
FILE = {"none": "b1", "static": "b2", "state_adaptive": "b3", "state_directional": "b3d"}
MECHS = ["gate hold", "post-evade stall", "nav2 abort", "stall", "lost"]
MCOL = {"gate hold": "#d9534f", "post-evade stall": "#f0ad4e", "nav2 abort": "#5bc0de",
        "stall": "#999999", "lost": "#dddddd"}
TRACE = re.compile(r"^\s+t=\s*([\d.]+)s\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)\s+dist=\s*([\d.]+)m\s+v=\s*([\d.]+)m/s\s+stuck=\s*([\d.]+)s(.*)$")
AISLE_N_EDGE, AISLE_S_EDGE = 4.965, 3.465   # north aisle, m (factory_u.yaml)


def where(x: float, y: float) -> str:
    if abs(x) > 9.3:
        return "crossing"
    ay = abs(y)
    if ay > AISLE_N_EDGE:
        return "refuge"
    if ay < AISLE_S_EDGE + 0.15:
        return "belt edge"
    return "aisle"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    args = ap.parse_args()
    d = Path(args.dir)
    rows = list(csv.DictReader((d / "trials.csv").open()))
    out_rows = []
    for r in rows:
        if r["result"] == "SUCCESS":
            continue
        info = classify(r["config"])
        seed = int(r.get("seed") or info["seed"])
        stem = Path(r["config"]).stem
        logs = [l for l in sorted(d.glob(f"logs/*_{stem}_{FILE[r['policy']]}.log")) if "attempt" not in l.name]
        trace, nav2_status, has_result = [], "", False
        if logs:
            for line in logs[-1].read_text(errors="replace").splitlines():
                m = TRACE.match(line)
                if m:
                    trace.append((float(m.group(1)), float(m.group(2)), float(m.group(3)),
                                  float(m.group(5)), m.group(7).strip()))
                elif "nav2 status" in line:
                    nav2_status = line.split(":", 1)[1].strip()
                elif line.startswith("=== Result"):
                    has_result = True
        evade = float(r["evade_seconds"] or 0)
        last = trace[-3:]
        flags = " ".join(t[4] for t in last)
        if not has_result or r["result"] == "TIMEOUT":
            mech = "lost"
        elif "YIELDING" in flags:
            mech = "gate hold"
        elif nav2_status.startswith("ABORTED"):
            mech = "nav2 abort"
        elif evade > 0:
            mech = "post-evade stall"
        else:
            mech = "stall"
        x, y = (trace[-1][1], trace[-1][2]) if trace else (float("nan"), float("nan"))
        out_rows.append(dict(seed=seed, north=(r.get("north_exchange") or "?").split("->")[0],
                             policy=r["policy"], result=r["result"], mech=mech,
                             where=where(x, y) if trace else "?", x=x, y=y,
                             t=float(r["travel_time"] or 0), yield_s=float(r["yield_seconds"] or 0),
                             evade_s=evade, who=r["min_clearance_who"], clr=float(r["min_clearance"] or "nan"),
                             contact_p=float(r["contact_person_seconds"] or 0),
                             cause=r["deadlock_cause"]))
    n_by_pol = Counter(r["policy"] for r in rows)
    counts = defaultdict(Counter)
    for o in out_rows:
        counts[o["policy"]][o["mech"]] += 1
    wheres = defaultdict(Counter)
    for o in out_rows:
        wheres[o["policy"]][o["where"]] += 1

    # ---- P12
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
    xs = range(len(POLICIES))
    bottom = [0.0] * len(POLICIES)
    for m in MECHS:
        vals = [counts[p][m] for p in POLICIES]
        axes[0].bar(list(xs), vals, 0.6, bottom=bottom, color=MCOL[m], label=m, edgecolor="k", lw=0.4)
        bottom = [b + v for b, v in zip(bottom, vals)]
    axes[0].set_xticks(list(xs))
    axes[0].set_xticklabels([f"{LABEL[p]}\n(n={n_by_pol[p]})" for p in POLICIES])
    axes[0].set_ylabel("AMR1 trials not reaching the goal")
    axes[0].set_title("how the robot stopped", fontsize=10)
    axes[0].legend(fontsize=8)
    axes[0].grid(axis="y", alpha=0.25)
    WH = ["aisle", "refuge", "belt edge", "crossing", "?"]
    WCOL = {"aisle": "#777777", "refuge": "#8e44ad", "belt edge": "#e67e22", "crossing": "#2980b9", "?": "#dddddd"}
    bottom = [0.0] * len(POLICIES)
    for w in WH:
        vals = [wheres[p][w] for p in POLICIES]
        if not any(vals):
            continue
        axes[1].bar(list(xs), vals, 0.6, bottom=bottom, color=WCOL[w], label=w, edgecolor="k", lw=0.4)
        bottom = [b + v for b, v in zip(bottom, vals)]
    axes[1].set_xticks(list(xs))
    axes[1].set_xticklabels([LABEL[p] for p in POLICIES])
    axes[1].set_title("where it stopped", fontsize=10)
    axes[1].legend(fontsize=8)
    axes[1].grid(axis="y", alpha=0.25)
    fig.suptitle(f"{d.name}: AMR1 failures by mechanism and place (gate hold = Comfort Space held it; "
                 "post-evade stall = stepped aside, then did not move again)", fontsize=10)
    fig.tight_layout()
    (d / "figures").mkdir(exist_ok=True)
    fig.savefig(d / "figures" / "P12_deadlock_mechanism.png", dpi=130)
    plt.close(fig)

    # ---- table
    out = [f"# {d.name}: how each failed AMR1 trial stopped\n",
           "| policy | n | gate hold | post-evade stall | nav2 abort | stall | lost |", "|---|---|---|---|---|---|---|"]
    for p in POLICIES:
        out.append(f"| {LABEL[p]} | {n_by_pol[p]} | " + " | ".join(str(counts[p][m]) for m in MECHS) + " |")
    out.append("\n| seed | north | policy | result | mechanism | where | final pos | travel s | yield s | evade s | closest (who) | person walked in s | runner's cause |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for o in sorted(out_rows, key=lambda o: (o["seed"], POLICIES.index(o["policy"]))):
        out.append(f"| {o['seed']} | {o['north']} | {LABEL[o['policy']]} | {o['result']} | {o['mech']} | {o['where']} | "
                   f"({o['x']:.2f}, {o['y']:.2f}) | {o['t']:.1f} | {o['yield_s']:.1f} | {o['evade_s']:.1f} | "
                   f"{o['clr']:.2f} ({o['who']}) | {o['contact_p']:.1f} | {o['cause'][:70]} |")
    (d / "deadlock_mechanism.md").write_text("\n".join(out) + "\n")
    print("\n".join(out[:7]))
    print("wrote", d / "figures" / "P12_deadlock_mechanism.png", d / "deadlock_mechanism.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
