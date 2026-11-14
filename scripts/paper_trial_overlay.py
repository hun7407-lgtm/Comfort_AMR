#!/usr/bin/env python3
"""Top-view overlay of RECORDED Stage 1 corridor trials, drawn from their JSON.

    python3 scripts/paper_trial_overlay.py --trial <trial.json> --out a.png
    python3 scripts/paper_trial_overlay.py --trial A.json --trial B.json --trial C.json \\
        --panel-titles "buffer 0" "buffer 0.10" "buffer 0.35" --out cmp.png --ylim -1.3 1.3

Plain Python 3 + matplotlib. Nothing is simulated or recomputed from a
model: every line comes from the trial's own record or its config.

    walls            corridor.width of the trial's environment config
    inflation band   walls grown by the inflation radius (Nav2 inflation layer)
    Comfort Space    the person's space, person centre -> AMR surface (solid)
    gate boundary    where the gate keeps the AMR CENTRE out: Comfort Space
                     grown by the AMR half width + yield margin (dashed)
    buffer region    Comfort Space grown by half width + costmap buffer_width
                     of THIS trial's condition (dotted band, soft cost)
    first plan       Nav2's first global plan, as recorded (planned_path)
    trajectory       AMR 1's recorded path (trajectory)
    final footprint  AMR 1's footprint at its final pose, with the verdict

--clean draws walls, person, Comfort Space and AMR only (no plan, no
labels). Same --xlim/--ylim and size for every panel of a comparison.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.comfort_space import robot_grow  # noqa: E402
from comfort_core.config import load_config  # noqa: E402
from comfort_core.labels import robot_label  # noqa: E402
from comfort_core.layout import corridor_bounds  # noqa: E402
from comfort_core.nav2_tuning import inflation_radius  # noqa: E402

C_COMFORT = "#E69F00"
C_PLAN = "#0072B2"
C_TRAJ = "#222222"
C_WALL = "#7f7f7f"
C_FAIL = "#D55E00"
C_OK = "#009E73"


def outline(shape, grow=0.0, n=180):
    s = shape.grown(grow) if grow else shape
    cx, cy = s.centre
    c, si = math.cos(s.heading), math.sin(s.heading)
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        u, v = s.along * math.cos(t), s.across * math.sin(t)
        pts.append((cx + u * c - v * si, cy + u * si + v * c))
    return pts


def footprint(cfg, pose):
    x, y, yaw = pose
    hl, hw = float(cfg["amr"]["footprint_length"]) / 2, float(cfg["amr"]["footprint_width"]) / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + u * c - v * s, y + u * s + v * c) for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def condition_for(trial_json: Path, scen: str):
    """The condition overlay this trial ran with: the manifest of its batch
    folder names it (configs/stage1a/conditions*/<scenario>.yaml)."""
    man = trial_json.parent / "manifest.txt"
    if man.exists():
        for line in man.read_text().splitlines():
            parts = line.split()
            if parts and not line.startswith("#") and Path(parts[0]).stem == scen:
                return parts[0]
    return None


def draw(ax, trial_json: Path, args, title=None, cond=None):
    d = json.loads(trial_json.read_text())
    row = d["row"]
    cond = cond or condition_for(trial_json, row["scenario"])
    cfg = load_config(scenario=cond, common_path=row["config"]) if cond else load_config(common_path=row["config"])
    b = corridor_bounds(cfg)
    infl = float(inflation_radius(cfg))
    grow = robot_grow(cfg)
    margin = float(cfg["comfort_space"]["yield"]["margin"])
    buf = float((cfg["comfort_space"].get("costmap") or {}).get("buffer_width", 0.0) or 0.0)
    shapes = fz.comfort_shapes(cfg, 0.0) if row["scenario"] != "b1" else []
    hx, hy = cfg["humans"]["a"]["position"]

    xl, yl = args.xlim, args.ylim or (b.y_min - 0.25, b.y_max + 0.25)
    ax.add_patch(Rectangle((xl[0] - 1, b.y_max), xl[1] - xl[0] + 2, 1.0, color=C_WALL, zorder=1))
    ax.add_patch(Rectangle((xl[0] - 1, b.y_min - 1.0), xl[1] - xl[0] + 2, 1.0, color=C_WALL, zorder=1))
    if not args.clean:
        for y0 in (b.y_max - infl, b.y_min):
            ax.add_patch(Rectangle((xl[0] - 1, y0), xl[1] - xl[0] + 2, infl, color=C_WALL, alpha=0.18,
                                   lw=0, zorder=1))
    for s in shapes:
        if not args.clean:
            if buf > 0:
                ax.add_patch(Polygon(outline(s, grow + buf), closed=True, fc=C_COMFORT, alpha=0.12,
                                     ec=C_COMFORT, ls=":", lw=1.4, zorder=2))
            ax.add_patch(Polygon(outline(s, grow + margin), closed=True, fill=False, ec=C_FAIL,
                                 ls="--", lw=1.4, zorder=3))
        ax.add_patch(Polygon(outline(s), closed=True, fc=C_COMFORT, alpha=0.55, ec=C_COMFORT, lw=1.5, zorder=3))
    ax.add_patch(plt.Circle((hx, hy), float(cfg["humans"]["radius"]), color="#444444", zorder=4))
    traj = d.get("trajectory") or []
    if not args.clean:
        plan = d.get("planned_path") or []
        if plan:
            ax.plot([p[0] for p in plan], [p[1] for p in plan], color=C_PLAN, lw=2.0, zorder=5)
    if traj and not args.clean:
        ax.plot([p[1] for p in traj], [p[2] for p in traj], color=C_TRAJ, lw=1.4, ls="-", zorder=6)
    pose = d.get("final_pose") or (traj[-1][1:4] if traj else None)
    ok = row["result"] == "SUCCESS"
    if pose:
        ax.add_patch(Polygon(footprint(cfg, pose), closed=True, fc=(C_OK if ok else C_FAIL), alpha=0.85,
                             ec="black", lw=1.0, zorder=7))
    ax.set_xlim(*xl)
    ax.set_ylim(*yl)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    if not args.clean:
        e = fz.extent_of(cfg, "static" if row["policy"] == "static" else "b3o",
                         float(row["scenario"].split("_r")[1]), "a") if "_r" in row["scenario"] else None
        mech = row.get("deadlock_mechanism") or ""
        cause = row.get("deadlock_cause") or ""
        if mech == "stall" and "ProtectiveStop" in cause:
            mech = "ProtectiveStop (stall)"
        verdict = "SUCCESS" if ok else (mech or row["result"])
        label = title or ""
        ax.set_title(f"{label}\n{robot_label('AMR1')}: {verdict}   W {float(cfg['corridor']['width']):.1f} m   "
                     f"e {e:.2f} m   buffer {buf:.2f} m" if e is not None else
                     f"{label}\n{robot_label('AMR1')}: {verdict}   W {float(cfg['corridor']['width']):.1f} m",
                     fontsize=args.fs)
    return {"trial": str(trial_json), "result": row["result"], "mechanism": row.get("deadlock_mechanism"),
            "cause": row.get("deadlock_cause"), "buffer": buf, "env": row["config"],
            "scenario": row["scenario"], "condition": cond}


def legend(fig, clean, fs=9):
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    h = [Patch(fc=C_COMFORT, alpha=0.55, label="Comfort Space (person centre -> AMR surface)"),
         Patch(fc=C_WALL, label="wall"), Patch(fc=C_OK, label="AMR 1 footprint (final, success)"),
         Patch(fc=C_FAIL, label="AMR 1 footprint (final, failure)")]
    if not clean:
        h += [Line2D([], [], color=C_FAIL, ls="--", label="gate boundary for the AMR centre (+ half width + margin)"),
              Patch(fc=C_COMFORT, alpha=0.15, ec=C_COMFORT, ls=":", label="costmap buffer region (+ half width + buffer)"),
              Patch(fc=C_WALL, alpha=0.2, label="inflation band"),
              Line2D([], [], color=C_PLAN, lw=2, label="first global plan (recorded)"),
              Line2D([], [], color=C_TRAJ, lw=1.4, label="AMR 1 trajectory (recorded)")]
    fig.legend(handles=h, loc="lower center", ncol=3, fontsize=fs, frameon=False)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trial", action="append", required=True)
    ap.add_argument("--condition", action="append", default=None,
                    help="condition overlay per --trial, in order (default: the batch manifest)")
    ap.add_argument("--panel-titles", nargs="*", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--xlim", type=float, nargs=2, default=(-5.6, 5.6))
    ap.add_argument("--ylim", type=float, nargs=2, default=None)
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--json", default=None, help="write what was drawn (provenance) here")
    args = ap.parse_args()
    n = len(args.trial)
    args.fs = 18 if n == 1 else (14 if n == 2 else 11)   # one strip leaves room for larger text
    fig, axes = plt.subplots(n, 1, figsize=(19.2, 10.8), squeeze=False)
    info = []
    for i, (ax, t) in enumerate(zip(axes[:, 0], args.trial)):
        title = args.panel_titles[i] if args.panel_titles and i < len(args.panel_titles) else None
        cond = args.condition[i] if args.condition and i < len(args.condition) else None
        info.append(draw(ax, Path(t), args, title, cond))
    if args.title:
        fig.suptitle(args.title, fontsize=args.fs + 2)
    legend(fig, args.clean, 14 if n == 1 else 9)
    fig.tight_layout(rect=(0, 0.09, 1, 0.96 if args.title else 1))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=100)
    plt.close(fig)
    if args.json:
        Path(args.json).write_text(json.dumps(info, indent=2))
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
