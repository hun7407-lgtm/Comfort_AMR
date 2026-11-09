#!/usr/bin/env python3
"""Two extra figures for a policy x AMR-count pilot (see report_pilot.py).

P7  seed x policy outcome matrix, one panel per AMR count. Shows whether
    the same seeds fail under every AMR count (then the failures come from
    the people, not from the other robots).
P8  AMR1 traces for one seed under every policy, read from the 2 s
    progress lines the trial runner prints ("t= .. pos=(x, y)"), drawn
    over the aisles and crossings with the walkers' scripted routes.

Usage:
  python3 scripts/pilot_extra_figures.py --dir results/batches/261105_MAIN_pilot_policy_x_amrs_seeds1-8 --trace-seed 7
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from report_pilot import classify, correct_goal_rows, read_csv  # noqa: E402

POLICIES = ["none", "static", "state_adaptive", "state_directional"]

POLICY_LABEL = {"none": "B1", "static": "B2", "state_adaptive": "B3",
                "state_directional": "B3-O"}
POLICY_FILE = {"none": "b1", "static": "b2", "state_adaptive": "b3",
               "state_directional": "b3d"}
COLOUR = {"none": "#7f7f7f", "static": "#d62728", "state_adaptive": "#1f77b4",
          "state_directional": "#2ca02c"}
OUTCOME_COLOUR = {"SUCCESS": "#3a9d5d", "DEADLOCK": "#d9534f", "INVALID": "#f0ad4e",
                  "missing": "#dddddd"}

TRACE_RE = re.compile(r"t=\s*([\d.]+)s\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)")
FLEET_TRACE_RE = re.compile(r"\[fleet\] (a\d) t=\s*([\d.]+)\s+pos=\(\s*([-\d.]+),\s*([-\d.]+)\)")
FLEET_COLOUR = {"a1": "#ff7f0e", "a2": "#9467bd", "a3": "#8c564b"}


def seed_matrix(rows, out: Path, title: str) -> None:
    counts = sorted({int(r["amr_count"]) for r in rows})
    seeds = sorted({int(r["seed"]) for r in rows})
    fig, axes = plt.subplots(1, len(counts), figsize=(max(4.2, 0.42 * len(seeds) + 1.0) * len(counts), 3.2), sharey=True)
    if len(counts) == 1:
        axes = [axes]
    for ax, n in zip(axes, counts):
        grid = {}
        for r in rows:
            if int(r["amr_count"]) != n:
                continue
            grid[(r["policy"], int(r["seed"]))] = r["result"]
        for yi, pol in enumerate(POLICIES):
            for xi, s in enumerate(seeds):
                res = grid.get((pol, s), "missing")
                ax.add_patch(plt.Rectangle((xi, yi), 1, 1, facecolor=OUTCOME_COLOUR.get(res, "#999"),
                                           edgecolor="white", linewidth=1.5))
                ax.text(xi + 0.5, yi + 0.5, {"SUCCESS": "S", "DEADLOCK": "D", "INVALID": "I",
                                             "missing": "-"}.get(res, "?"),
                        ha="center", va="center", fontsize=9, color="white", fontweight="bold")
        ax.set_xlim(0, len(seeds))
        ax.set_ylim(len(POLICIES), 0)
        ax.set_xticks([i + 0.5 for i in range(len(seeds))])
        ax.set_xticklabels([str(s) for s in seeds])
        ax.set_yticks([i + 0.5 for i in range(len(POLICIES))])
        ax.set_yticklabels([POLICY_LABEL[p] for p in POLICIES])
        ax.set_xlabel("seed (same people and walks for every policy)")
        ax.set_title(f"{n} AMR{'s' if n > 1 else ''}")
        ax.tick_params(length=0)
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=OUTCOME_COLOUR[k]) for k in
               ("SUCCESS", "DEADLOCK", "INVALID", "missing")]
    fig.legend(handles, ["SUCCESS", "DEADLOCK", "INVALID (robot thrown off the map)",
                         "missing (watchdog)"], loc="lower center", ncol=4, frameon=False,
               bbox_to_anchor=(0.5, -0.04))
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def read_trace(log: Path):
    pts = []
    for line in log.read_text(errors="replace").splitlines():
        if "[fleet]" in line:
            continue
        m = TRACE_RE.search(line)
        if m:
            pts.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return pts


def read_fleet_traces(log: Path, trial_start: float):
    """Fleet position lines are on the simulation clock; shift them to
    seconds since the trial start like AMR1's."""
    out = {}
    for line in log.read_text(errors="replace").splitlines():
        m = FLEET_TRACE_RE.search(line)
        if m:
            out.setdefault(m.group(1), []).append(
                (float(m.group(2)) - trial_start, float(m.group(3)), float(m.group(4))))
    return out


def draw_lanes(ax, factory: dict) -> None:
    f = factory["factory"]
    half_w = f["width"] / 2.0
    half_l = f["column_pitch"] * f["bays"] / 2.0
    y = -half_w
    for row in f["rows"]:
        if row["kind"] == "aisle":
            ax.add_patch(plt.Rectangle((-half_l, y), 2 * half_l, row["depth"],
                                       facecolor="#f2f2f2", edgecolor="#bbbbbb", zorder=0))
        y += row["depth"]
    cw = f["cross_aisles"]["width"]
    aisles = [r for r in f["rows"]]
    # the crossings join the two aisles
    y0 = -half_w + aisles[0]["depth"]
    y1 = half_w - aisles[-1]["depth"]
    for cx in f["cross_aisles"]["at"]:
        ax.add_patch(plt.Rectangle((cx - cw / 2, y0), cw, y1 - y0,
                                   facecolor="#f2f2f2", edgecolor="#bbbbbb", zorder=0))


def trace_figure(result_dir: Path, scenario_yaml: Path, factory_yaml: Path, seed: int,
                 stem: str, out: Path, label: str = "1 AMR",
                 xlim=(-12.5, 3.0)) -> None:
    sys.path.insert(0, str(ROOT))
    from comfort_core import load_config  # noqa: E402
    sc = load_config("b3", common_path=str(scenario_yaml))
    factory = yaml.safe_load(factory_yaml.read_text())
    humans = sc["humans"]
    trial_start = float(sc["navigation"].get("trial_start_sim_time", 0.0))
    places = {k: tuple(v) for k, v in sc.get("fleet", {}).get("places", {}).items()}
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), sharex=True, sharey=True)
    for ax, pol in zip(axes.flat, POLICIES):
        draw_lanes(ax, factory)
        # people
        for name, h in humans.items():
            if not isinstance(h, dict) or "position" not in h or h.get("enabled") is False:
                continue
            px, py = h["position"]
            ax.plot(px, py, "o", color="#333333", ms=6, zorder=3)
            ax.annotate(name, (px, py), textcoords="offset points", xytext=(4, 4), fontsize=8)
            walk = h.get("walk")
            if walk:
                route = [h["position"]] + [list(p) for p in walk["route"]]
                ax.plot([p[0] for p in route], [p[1] for p in route], "--", color="#e377c2",
                        lw=1.2, zorder=2)
                ax.annotate(f"{name} walks t={walk['start_at']:.1f}s",
                            (route[-1][0], route[-1][1]), textcoords="offset points",
                            xytext=(4, -10), fontsize=7, color="#b03a8c")
        nav = sc["navigation"]
        ax.plot(*nav["start"], "s", color="k", ms=7, zorder=3)
        ax.plot(*nav["goal"], "*", color="k", ms=11, zorder=3)
        logs = sorted(result_dir.glob(f"logs/*_{stem}_{POLICY_FILE[pol]}.log"))
        logs = [l for l in logs if "attempt" not in l.name]
        res = "no log"
        if logs:
            pts = read_trace(logs[-1])
            if pts:
                ax.plot([p[1] for p in pts], [p[2] for p in pts], "-", color=COLOUR[pol], lw=2, zorder=4)
                ax.scatter([p[1] for p in pts], [p[2] for p in pts], c=[p[0] for p in pts],
                           cmap="viridis", s=14, zorder=5)
                for p in pts[::5]:
                    ax.annotate(f"{p[0]:.0f}", (p[1], p[2]), textcoords="offset points",
                                xytext=(3, 3), fontsize=6, color=COLOUR[pol])
            for veh, fpts in read_fleet_traces(logs[-1], trial_start).items():
                ax.plot([p[1] for p in fpts], [p[2] for p in fpts], "-", color=FLEET_COLOUR.get(veh, "k"),
                        lw=1.5, alpha=0.8, zorder=3)
                for p in fpts[::5]:
                    ax.annotate(f"{veh} {p[0]:.0f}", (p[1], p[2]), textcoords="offset points",
                                xytext=(3, -8), fontsize=6, color=FLEET_COLOUR.get(veh, "k"))
            for veh, spec in sc.get("fleet", {}).get("amrs", {}).items():
                if spec.get("enabled") and str(spec.get("start")) in places:
                    ax.plot(*places[str(spec["start"])], "D", color=FLEET_COLOUR.get(veh, "k"), ms=6, zorder=3)
            txt = logs[-1].read_text(errors="replace")
            m = re.search(r"result\s*:\s*(\w+)", txt)
            t = re.search(r"travel time\s*:\s*([\d.]+)", txt)
            c = re.search(r"deadlock detected\s*:\s*True\s*\((.*?)\)", txt)
            res = (m.group(1) if m else "?") + (f", {t.group(1)} s" if t else "")
            if c:
                res += f"\n{c.group(1)}"
        ax.set_title(f"{POLICY_LABEL[pol]}: {res}", fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(-6.5, 6.5)
        ax.grid(alpha=0.2)
    fig.suptitle(f"seed {seed}, {label}: AMR1 trace every 2 s (numbers = s since start); "
                 f"thin = fleet vehicles (diamond = their start); dashed = scripted walks")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--trace-seed", type=int, default=None)
    ap.add_argument("--trace-stem", default=None,
                    help="config stem of the 1-AMR trial, default paper_h8_a1_scr_s%%04d")
    ap.add_argument("--factory", default="configs/factory_u.yaml")
    ap.add_argument("--scenario-yaml", default=None,
                    help="config of the traced trial (default configs/scenarios/<stem>.yaml)")
    ap.add_argument("--trace-label", default="1 AMR")
    ap.add_argument("--xlim", default="-12.5,3.0")
    args = ap.parse_args()
    d = Path(args.dir)
    figs = d / "figures"
    figs.mkdir(exist_ok=True)
    rows = read_csv(d / "trials.csv")
    correct_goal_rows(rows)          # relabels in place, see report_pilot.py
    for r in rows:
        info = classify(r["config"])
        r.setdefault("amr_count", "")
        if not r["amr_count"]:
            r["amr_count"] = str(info["amrs"])
        if not r.get("seed"):
            r["seed"] = str(info["seed"])
    seed_matrix(rows, figs / "P7_seed_matrix.png",
                f"{d.name}: AMR1 outcome per seed, policy and AMR count")
    print("wrote", figs / "P7_seed_matrix.png")
    if args.trace_seed is not None:
        stem = args.trace_stem or f"paper_h8_a1_scr_s{args.trace_seed:04d}"
        yml = Path(args.scenario_yaml) if args.scenario_yaml else ROOT / "configs" / "scenarios" / f"{stem}.yaml"
        xlim = tuple(float(v) for v in args.xlim.split(","))
        trace_figure(d, yml, ROOT / args.factory, args.trace_seed, stem,
                     figs / f"P8_trace_seed{args.trace_seed}.png", label=args.trace_label, xlim=xlim)
        print("wrote", figs / f"P8_trace_seed{args.trace_seed}.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
