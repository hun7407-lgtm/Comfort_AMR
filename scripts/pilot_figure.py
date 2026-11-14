#!/usr/bin/env python3
"""The preliminary study as ONE composite figure (docs/STAGES.md, revised design).

    python3 scripts/pilot_figure.py --dir results/batches/261112_stage1_size_sweep

Left: Comfort Space size vs AMR 1 comfort-gate-hold rate, Static and B3-O
(counts on the points). Right: AMR 1 outcome per size x seed, both policies.
No thresholds are drawn: the preliminary study shows the trend, it does
not define a boundary (several interactions act at once in this scene).
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

from comfort_core.size_sweep import parse_condition  # noqa: E402

COLOUR = {"static": "#d55e00", "b3o": "#0072b2"}
LABEL = {"static": "Static", "b3o": "B3-O"}
# outcome codes for the matrix
CODES = {"SUCCESS": 0, "comfort_gate_hold": 1, "other": 2, "invalid": 3}
CMAP = ListedColormap(["#cfe3f1", "#b2182b", "#f4a582", "#eeeeee"])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    d = Path(args.dir)
    rows = list(csv.DictReader((d / "trials.csv").open(newline="")))
    cells = defaultdict(dict)
    for r in rows:
        p = parse_condition(r["scenario"])
        if not p:
            continue
        kind, size = p
        res = r["result"]
        mech = r.get("deadlock_mechanism") or ""
        code = ("SUCCESS" if res == "SUCCESS" else "invalid" if res == "INVALID" or mech == "invalid"
                else "comfort_gate_hold" if mech == "comfort_gate_hold" else "other")
        cells[(kind, size)][int(r["seed"])] = code
    sizes = sorted({s for _k, s in cells})
    seeds = sorted({sd for v in cells.values() for sd in v})

    fig = plt.figure(figsize=(12, 4.6))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.1, 1.0, 1.0], wspace=0.35)
    ax = fig.add_subplot(gs[0])
    for k in ("static", "b3o"):
        xs, ys, ns = [], [], []
        for s in sizes:
            v = [c for c in cells[(k, s)].values() if c != "invalid"]
            if v:
                xs.append(s)
                ys.append(sum(c == "comfort_gate_hold" for c in v) / len(v))
                ns.append(f"{sum(c == 'comfort_gate_hold' for c in v)}/{len(v)}")
        ax.plot(xs, ys, marker="s" if k == "static" else "o", color=COLOUR[k], lw=2, label=LABEL[k])
        for x, y, n in zip(xs, ys, ns):
            ax.annotate(n, (x, y), textcoords="offset points", xytext=(0, 8 if k == "static" else -13),
                        ha="center", fontsize=7, color="#333")
    ax.set_xticks(sizes)
    ax.set_xlabel("Comfort Space size (m)\n[Static radius / B3-O moving front @ 1.0 m/s]")
    ax.set_ylabel("AMR 1 comfort gate hold rate")
    ax.set_ylim(-0.09, 1.05)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("(a) size vs comfort-induced deadlock", fontsize=10)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)

    for j, k in enumerate(("static", "b3o")):
        ax = fig.add_subplot(gs[1 + j])
        grid = [[CODES[cells[(k, s)].get(sd, "invalid")] for s in sizes] for sd in seeds]
        ax.imshow(grid, aspect="auto", cmap=CMAP, vmin=0, vmax=3, interpolation="nearest")
        ax.set_xticks(range(len(sizes)))
        ax.set_xticklabels([f"{s:.1f}" for s in sizes])
        ax.set_yticks(range(0, len(seeds), 4))
        ax.set_yticklabels([seeds[i] for i in range(0, len(seeds), 4)], fontsize=7)
        ax.set_xlabel("size (m)")
        if j == 0:
            ax.set_ylabel("seed (same scenario across sizes)")
        ax.set_title(f"({'b' if j == 0 else 'c'}) {LABEL[k]}: AMR 1 outcome per size x seed", fontsize=10)
    handles = [plt.Rectangle((0, 0), 1, 1, color=CMAP(i)) for i in range(4)]
    fig.legend(handles, ["success", "comfort gate hold", "other failure (evade stall, Nav2 abort, stall)",
                         "invalid"], loc="lower center", ncol=4, fontsize=8, frameon=False)
    fig.suptitle("Preliminary study: complex cross scenario (8 people, 3 AMRs, 33 seeds)", fontsize=11)
    fig.subplots_adjust(bottom=0.24, top=0.86)
    out = Path(args.out) if args.out else d / "figures" / "PRELIM_composite.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160)
    print(f"[pilot_figure] wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
