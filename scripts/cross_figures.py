#!/usr/bin/env python3
"""Interaction figures for the 261105_cross_seeds1-8 campaign (three AMRs that meet).

Reads the trial logs (AMR1 progress lines every 2 s, fleet position lines
every 2 s with the gap to AMR1) and the two csv files, and writes

  P9_convoy_gap.png     AMR1 <-> a1 gap over time, one panel per seed,
                        one line per policy: when did AMR1 catch the
                        leader, how long did it sit behind it
  P10_interactions.png  per policy: AMR1 time within 2 m of a1, AMR1 time
                        stopped while a1 was within 2 m, a2 closest
                        approach to a person, contacts by who closed in
  P11_by_condition.png  outcome by which north belt worker walks out (N1/N2/N3)
  cross_tables.md       the numbers behind all three

Usage: python3 scripts/cross_figures.py --dir results/batches/261105_cross_seeds1-8
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from report_pilot import classify, correct_goal_rows, read_csv  # noqa: E402

POLICIES = ["none", "static", "state_adaptive", "state_directional"]
LABEL = {"none": "B1", "static": "B2", "state_adaptive": "B3", "state_directional": "B3-O"}
FILE = {"none": "b1", "static": "b2", "state_adaptive": "b3", "state_directional": "b3d"}
COLOUR = {"none": "#7f7f7f", "static": "#d62728", "state_adaptive": "#1f77b4",
          "state_directional": "#2ca02c"}

AMR1_RE = re.compile(r"t=\s*([\d.]+)s\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)\s+dist=\s*([\d.]+)m\s+v=\s*([\d.]+)m/s")
FLEET_RE = re.compile(r"\[fleet\] (a\d) t=\s*([\d.]+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)\s+v=([\d.]+)m/s\s+gap_amr1=([\d.inf]+)m")
NEAR_M = 2.0


def parse_log(log: Path, trial_start: float):
    amr1, fleet = [], defaultdict(list)
    for line in log.read_text(errors="replace").splitlines():
        m = FLEET_RE.search(line)
        if m:
            gap = float(m.group(6)) if m.group(6) != "inf" else math.inf
            fleet[m.group(1)].append((float(m.group(2)) - trial_start, float(m.group(3)),
                                      float(m.group(4)), float(m.group(5)), gap))
            continue
        if "[fleet]" in line:
            continue
        m = AMR1_RE.search(line)
        if m:
            amr1.append(tuple(float(m.group(i)) for i in range(1, 6)))
    return amr1, fleet


def amr1_speed_at(amr1, t):
    best = None
    for row in amr1:
        if best is None or abs(row[0] - t) < abs(best[0] - t):
            best = row
    return best[4] if best and abs(best[0] - t) <= 1.5 else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--trial-start", type=float, default=45.0,
                    help="navigation.trial_start_sim_time (fleet lines are on the sim clock)")
    args = ap.parse_args()
    d = Path(args.dir)
    figs = d / "figures"
    figs.mkdir(exist_ok=True)

    rows = read_csv(d / "trials.csv")
    correct_goal_rows(rows)
    fleet_rows = read_csv(d / "fleet_trials.csv")
    for r in rows:
        info = classify(r["config"])
        r["_seed"] = int(r.get("seed") or info["seed"])
    seeds = sorted({r["_seed"] for r in rows})

    # per (policy, seed): convoy series and interaction numbers
    series = {}
    kpi = defaultdict(dict)
    for r in rows:
        pol, seed = r["policy"], r["_seed"]
        stem = Path(r["config"]).stem
        logs = [l for l in sorted(d.glob(f"logs/*_{stem}_{FILE[pol]}.log")) if "attempt" not in l.name]
        if not logs:
            continue
        amr1, fleet = parse_log(logs[-1], args.trial_start)
        a1 = fleet.get("a1", [])
        series[(pol, seed)] = a1
        near_s = sum(2.0 for p in a1 if p[4] <= NEAR_M)
        stopped_near_s = 0.0
        for p in a1:
            if p[4] <= NEAR_M:
                v = amr1_speed_at(amr1, p[0])
                if v is not None and v < 0.05:
                    stopped_near_s += 2.0
        min_gap = min((p[4] for p in a1), default=math.nan)
        kpi[(pol, seed)].update(result=r["result"], travel=float(r["travel_time"] or "nan"),
                                near_s=near_s, stopped_near_s=stopped_near_s, min_gap=min_gap,
                                amr1_clr=float(r["min_clearance"] or "nan"), amr1_who=r["min_clearance_who"],
                                amr1_contact=float(r["contact_seconds"] or 0),
                                amr1_contact_robot=float(r["contact_robot_seconds"] or 0),
                                yield_s=float(r["yield_seconds"] or 0))
    for fr in fleet_rows:
        info = classify(fr["config"])
        key = (fr["policy"], int(info["seed"]))
        kpi[key][f"{fr['robot']}_result"] = fr["result"]
        kpi[key][f"{fr['robot']}_clr"] = float(fr["min_clearance_m"] or "nan")
        kpi[key][f"{fr['robot']}_who"] = fr["min_clearance_with"]
        kpi[key][f"{fr['robot']}_contact"] = float(fr["contact_s"] or 0)
        kpi[key][f"{fr['robot']}_contact_robot"] = float(fr["contact_robot_s"] or 0)
        kpi[key][f"{fr['robot']}_legs"] = f"{fr['legs_done']}/{fr['legs_total']}"
        # the dispatcher's own minimum (every tick) beats the 2 s samples
        if fr["robot"] == "a1" and fr.get("min_gap_amr1_m"):
            kpi[key]["min_gap"] = float(fr["min_gap_amr1_m"])

    # ---- P9 convoy gap
    n = len(seeds)
    cols = 4 if n > 4 else n
    rws = math.ceil(n / cols)
    fig, axes = plt.subplots(rws, cols, figsize=(4.2 * cols, 3.0 * rws), sharex=True, sharey=True)
    axes = list(axes.flat) if n > 1 else [axes]
    for ax, seed in zip(axes, seeds):
        for pol in POLICIES:
            s = series.get((pol, seed))
            if not s:
                continue
            ax.plot([p[0] for p in s], [min(p[4], 15) for p in s], color=COLOUR[pol], lw=1.6,
                    label=LABEL[pol])
        ax.axhline(NEAR_M, color="k", lw=0.8, ls=":")
        ax.set_title(f"seed {seed}", fontsize=10)
        ax.set_ylim(0, 15)
        ax.grid(alpha=0.25)
    for ax in axes[len(seeds):]:
        ax.axis("off")
    axes[0].legend(fontsize=8, loc="upper right")
    fig.supxlabel("s since the trial start (a1 sets off at 1-1.3 s, 4.6 m ahead of AMR1)")
    fig.supylabel("AMR1 - a1 centre gap, m (clipped at 15)")
    fig.suptitle("Convoy: how close AMR1 got to the leader a1, and for how long (dotted = 2 m)")
    fig.tight_layout()
    fig.savefig(figs / "P9_convoy_gap.png", dpi=130)
    plt.close(fig)

    # ---- P10 interaction KPIs by policy
    def mean(vals):
        vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
        return sum(vals) / len(vals) if vals else math.nan

    agg = {}
    for pol in POLICIES:
        ks = [k for k in kpi if k[0] == pol and "result" in kpi[k]]
        agg[pol] = dict(
            n=len(ks),
            near_s=mean([kpi[k]["near_s"] for k in ks]),
            stopped_near_s=mean([kpi[k]["stopped_near_s"] for k in ks]),
            min_gap=mean([kpi[k]["min_gap"] for k in ks]),
            amr1_clr=mean([kpi[k]["amr1_clr"] for k in ks]),
            a2_clr=mean([kpi[k].get("a2_clr", math.nan) for k in ks]),
            a1_clr=mean([kpi[k].get("a1_clr", math.nan) for k in ks]),
            amr1_contact=mean([kpi[k]["amr1_contact"] for k in ks]),
            amr1_contact_robot=mean([kpi[k]["amr1_contact_robot"] for k in ks]),
            a2_contact=mean([kpi[k].get("a2_contact", 0.0) for k in ks]),
            a2_contact_robot=mean([kpi[k].get("a2_contact_robot", 0.0) for k in ks]),
            success=mean([1.0 if kpi[k]["result"] == "SUCCESS" else 0.0 for k in ks]),
            a1_success=mean([1.0 if kpi[k].get("a1_result") == "SUCCESS" else 0.0 for k in ks]),
            a2_success=mean([1.0 if kpi[k].get("a2_result") == "SUCCESS" else 0.0 for k in ks]),
        )
    fig, axes = plt.subplots(1, 4, figsize=(17, 4))
    x = range(len(POLICIES))
    labels = [LABEL[p] for p in POLICIES]
    cs = [COLOUR[p] for p in POLICIES]
    axes[0].bar(x, [agg[p]["near_s"] for p in POLICIES], color=cs, alpha=0.5, label="within 2 m of a1")
    axes[0].bar(x, [agg[p]["stopped_near_s"] for p in POLICIES], color=cs, label="... and stopped")
    axes[0].set_title("AMR1 behind the leader a1 (s, mean)")
    axes[0].legend(fontsize=8)
    axes[1].bar(x, [agg[p]["min_gap"] for p in POLICIES], color=cs)
    axes[1].set_title("closest AMR1 - a1 gap (m, mean)")
    w = 0.27
    axes[2].bar([i - w for i in x], [agg[p]["amr1_clr"] for p in POLICIES], w, color=cs, label="AMR1")
    axes[2].bar(list(x), [agg[p]["a1_clr"] for p in POLICIES], w, color=cs, alpha=0.6, label="a1")
    axes[2].bar([i + w for i in x], [agg[p]["a2_clr"] for p in POLICIES], w, color=cs, alpha=0.3, label="a2")
    axes[2].axhline(0, color="k", lw=0.8)
    axes[2].set_title("closest approach to a person (m, mean; <0 = touching)")
    axes[2].legend(fontsize=8)
    axes[3].bar([i - w / 2 for i in x], [agg[p]["amr1_contact"] - agg[p]["amr1_contact_robot"] for p in POLICIES], w,
                color=cs, alpha=0.4, label="AMR1: person walked in")
    axes[3].bar([i - w / 2 for i in x], [agg[p]["amr1_contact_robot"] for p in POLICIES], w,
                bottom=[agg[p]["amr1_contact"] - agg[p]["amr1_contact_robot"] for p in POLICIES],
                color=cs, label="AMR1: robot drove in")
    axes[3].bar([i + w / 2 for i in x], [agg[p]["a2_contact"] - agg[p]["a2_contact_robot"] for p in POLICIES], w,
                color=cs, alpha=0.4, hatch="//", label="a2: person walked in")
    axes[3].bar([i + w / 2 for i in x], [agg[p]["a2_contact_robot"] for p in POLICIES], w,
                bottom=[agg[p]["a2_contact"] - agg[p]["a2_contact_robot"] for p in POLICIES],
                color=cs, hatch="//", label="a2: robot drove in")
    axes[3].set_title("contact time with a person (s, mean), by who closed in")
    axes[3].legend(fontsize=7)
    for ax in axes:
        ax.set_xticks(list(x))
        ax.set_xticklabels(labels)
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle(f"{d.name}\nthe three encounters, by policy (8 people, AMR1 + a1 + a2)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(figs / "P10_interactions.png", dpi=130)
    plt.close(fig)

    # ---- P11 outcome by condition (who walks out to meet AMR1)
    # The seed only redraws the exchange: which belt worker on each side
    # swaps with the storage-side person, and when. The north draw decides
    # what AMR1 meets -- N1 is behind its start (nothing), N2 crosses its
    # path (a yield), N3 crosses its path AND leaves E1 standing at a
    # station downstream of the robot (a Comfort Space in the aisle ahead).
    north = {}
    for r in rows:
        north[r["_seed"]] = (r.get("north_exchange") or "?").split("->")[0]
    cats = [c for c in ("N1", "N2", "N3") if c in north.values()]
    cond = {}
    for c in cats:
        cs = [sd for sd in seeds if north.get(sd) == c]
        for p in POLICIES:
            ks = [kpi.get((p, sd)) for sd in cs]
            ks = [k for k in ks if k and "result" in k]
            b1 = {sd: kpi.get(("none", sd)) for sd in cs}
            dts = []
            for sd in cs:
                k, b = kpi.get((p, sd)), b1.get(sd)
                if k and b and k.get("result") == "SUCCESS" and b.get("result") == "SUCCESS":
                    dts.append(k["travel"] - b["travel"])
            cond[(c, p)] = dict(
                n=len(ks), seeds=cs,
                success=sum(k["result"] == "SUCCESS" for k in ks) / len(ks) if ks else math.nan,
                deadlock=sum(k["result"] == "DEADLOCK" for k in ks),
                dt=mean(dts), yield_s=mean([k["yield_s"] for k in ks]),
                amr1_clr=mean([k["amr1_clr"] for k in ks]),
                a2_success=mean([1.0 if k.get("a2_result") == "SUCCESS" else 0.0 for k in ks if "a2_result" in k]))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    w = 0.2
    xs = range(len(cats))
    for i, p in enumerate(POLICIES):
        off = (i - 1.5) * w
        axes[0].bar([x + off for x in xs], [cond[(c, p)]["success"] for c in cats], w, color=COLOUR[p], label=LABEL[p])
        axes[1].bar([x + off for x in xs], [cond[(c, p)]["dt"] if p != "none" else 0 for c in cats], w, color=COLOUR[p])
        axes[2].bar([x + off for x in xs], [cond[(c, p)]["yield_s"] for c in cats], w, color=COLOUR[p])
    for ax, title, yl in zip(axes, ("AMR1 success rate", "travel time - B1 on the same seed (s, mean of successes)",
                                    "AMR1 yield time (s, mean)"),
                             ("share of trials", "s", "s")):
        ax.set_xticks(list(xs))
        ax.set_xticklabels([f"{c} walks out\n(n={cond[(c, POLICIES[0])]['n']} seeds)" for c in cats])
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(yl)
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylim(0, 1.15)
    axes[0].legend(fontsize=8, ncol=2, loc="lower left")
    fig.suptitle(f"{d.name}\noutcome by which north belt worker swaps with E1 "
                 "(N1 = behind AMR1's start, N2 = crosses its path, N3 = crosses it and E1 then stands at N3's station ahead of it)",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(figs / "P11_by_condition.png", dpi=130)
    plt.close(fig)

    # ---- tables
    out = [f"# {d.name}: interaction KPIs\n"]
    out.append("## By policy (means over seeds)\n")
    out.append("| policy | n | AMR1 success | a1 success | a2 success | AMR1 within 2 m of a1 s | ... stopped s | min AMR1-a1 gap m | AMR1 closest person m | a1 closest m | a2 closest m | AMR1 contact s (robot drove in) | a2 contact s (robot drove in) |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for p in POLICIES:
        a = agg[p]
        out.append(f"| {LABEL[p]} | {a['n']} | {a['success']:.0%} | {a['a1_success']:.0%} | {a['a2_success']:.0%} | "
                   f"{a['near_s']:.1f} | {a['stopped_near_s']:.1f} | {a['min_gap']:.2f} | {a['amr1_clr']:.2f} | "
                   f"{a['a1_clr']:.2f} | {a['a2_clr']:.2f} | {a['amr1_contact']:.1f} ({a['amr1_contact_robot']:.1f}) | "
                   f"{a['a2_contact']:.1f} ({a['a2_contact_robot']:.1f}) |")
    out.append("\n## By condition: which north belt worker walks out to swap with E1\n")
    out.append("| north partner | seeds | policy | n | AMR1 success | deadlocks | time - B1 s (mean, successes) | yield s (mean) | AMR1 closest person m | a2 success |")
    out.append("|---|---|---|---|---|---|---|---|---|---|")
    for c in cats:
        for p in POLICIES:
            a = cond[(c, p)]
            out.append(f"| {c} | {','.join(str(x) for x in a['seeds'])} | {LABEL[p]} | {a['n']} | {a['success']:.0%} | {a['deadlock']} | "
                       f"{a['dt']:+.1f} | {a['yield_s']:.1f} | {a['amr1_clr']:.2f} | {a['a2_success']:.0%} |")
    out.append("\n## Per seed\n")
    out.append("| seed | policy | AMR1 | travel s | yield s | AMR1 closest (who) | within 2 m of a1 s | stopped s | min gap m | a1 | a1 closest (who) | a2 | a2 closest (who) | a2 contact s |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for seed in seeds:
        for p in POLICIES:
            k = kpi.get((p, seed))
            if not k or "result" not in k:
                out.append(f"| {seed} | {LABEL[p]} | missing | | | | | | | | | | | |")
                continue
            out.append(f"| {seed} | {LABEL[p]} | {k['result']} | {k['travel']:.1f} | {k['yield_s']:.1f} | "
                       f"{k['amr1_clr']:.2f} ({k['amr1_who']}) | {k['near_s']:.0f} | {k['stopped_near_s']:.0f} | "
                       f"{k['min_gap']:.2f} | {k.get('a1_result', '')} {k.get('a1_legs', '')} | "
                       f"{k.get('a1_clr', float('nan')):.2f} ({k.get('a1_who', '')}) | "
                       f"{k.get('a2_result', '')} {k.get('a2_legs', '')} | "
                       f"{k.get('a2_clr', float('nan')):.2f} ({k.get('a2_who', '')}) | {k.get('a2_contact', 0):.1f} |")
    (d / "cross_tables.md").write_text("\n".join(out) + "\n")
    print("wrote", figs / "P9_convoy_gap.png", figs / "P10_interactions.png", figs / "P11_by_condition.png", d / "cross_tables.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
