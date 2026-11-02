#!/usr/bin/env python3
"""Draw a scenario looking down on the bay, with everything labelled.

    python3 scripts/preview_figure.py --config configs/scenarios/paper_h8_a1_scr_s0001.yaml
    python3 scripts/preview_figure.py --seed 1 --humans 8 --amrs 2

Plain Python 3 plus matplotlib. No ROS, no omni, no simulator: it reads
the scenario and draws it, which takes a second and answers the
question the preview exists for -- who is where, who walks where, and
where the robot is going.

It is the companion to the floor decals
(isaac_sim/environments/preview.py). Those let you match what you see
in the viewport to a person; this is the plan the viewport is showing.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from comfort_core.comfort_space import make_policy, robot_grow  # noqa: E402
from comfort_core.factory_layout import build_layout  # noqa: E402
from comfort_core.layout import human_positions, human_states  # noqa: E402
from comfort_core.scenario import BASE_CONFIG, DENSITY  # noqa: E402

STATE_COLOUR = {
    "station_engaged": "#1f6fd0",
    "stationary": "#e0a80f",
    "moving": "#d03a2a",
}
BELT_WALK = "#e2600f"
EDGE_WALK = "#15a76f"


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None,
                   help="a generated scenario config; overrides --seed")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--humans", type=int, default=8, choices=sorted(DENSITY))
    p.add_argument("--amrs", type=int, default=1, choices=(1, 2, 3))
    p.add_argument("--provider", default="scripted",
                   choices=("scripted", "sensor"))
    p.add_argument("--policy", default="b3",
                   choices=("b1", "b2", "b3r", "b3"),
                   help="whose Comfort Spaces to draw")
    p.add_argument("--out", default=None)
    return p.parse_args()


def _bay(ax, cfg) -> None:
    layout = build_layout(cfg)
    for band in layout.bands:
        ax.axhspan(band.y_min, band.y_max,
                   color="#f3f3f0" if band.kind == "aisle" else "#e4e4de",
                   zorder=0)
    for station in layout.stations:
        f = station.footprint
        ax.add_patch(mpatches.Rectangle(
            (f.x_min, f.y_min), f.x_max - f.x_min, f.y_max - f.y_min,
            facecolor="#b9bcc2", edgecolor="#8d9096", lw=0.5, zorder=1))
    for slot in layout.cart_slots_taken:
        ax.add_patch(mpatches.Rectangle(
            (slot.x_min, slot.y_min), slot.x_max - slot.x_min,
            slot.y_max - slot.y_min, facecolor="#cfc2a8",
            edgecolor="#9b8f78", lw=0.4, zorder=1))
    for slot in layout.cart_slots_free:
        ax.add_patch(mpatches.Rectangle(
            (slot.x_min, slot.y_min), slot.x_max - slot.x_min,
            slot.y_max - slot.y_min, facecolor="none",
            edgecolor="#9aa0a6", lw=0.5, ls=":", zorder=1))
    for cx, cy in layout.columns:
        half = float(cfg["factory"].get("column_size", 0.3)) / 2.0
        ax.add_patch(mpatches.Rectangle((cx - half, cy - half), 2 * half,
                                        2 * half, facecolor="#7c8086",
                                        zorder=2))
    for cross in layout.crosses:
        ax.axvspan(cross.x_min, cross.x_max, color="#f3f3f0", zorder=0)


def _people(ax, cfg, policy) -> None:
    states = human_states(cfg)
    humans = cfg["humans"]
    grow = robot_grow(cfg)
    for key, x, y in human_positions(cfg):
        state = states.get(key, "stationary")
        colour = STATE_COLOUR.get(state, "#666666")
        radius = policy.radius_for(key, state, 0.0)
        if radius > 0.0:
            ax.add_patch(plt.Circle((x, y), radius, facecolor=colour,
                                    alpha=0.13, edgecolor=colour, lw=0.8,
                                    ls="--", zorder=3))
            ax.add_patch(plt.Circle((x, y), radius + grow, facecolor="none",
                                    edgecolor=colour, lw=0.5, alpha=0.5,
                                    zorder=3))
        ax.add_patch(plt.Circle((x, y), float(humans["radius"]),
                                facecolor=colour, edgecolor="white", lw=0.8,
                                zorder=5))
        ax.annotate(key, (x, y), textcoords="offset points", xytext=(0, 9),
                    ha="center", fontsize=8, fontweight="bold",
                    color="#1a1a1a", zorder=6)

        walk = (humans.get(key) or {}).get("walk") or {}
        route = walk.get("route") or []
        if not route:
            continue
        half_edge = str(walk.get("ends_as")) == "station_engaged"
        rgb = EDGE_WALK if half_edge else BELT_WALK
        xs = [x] + [float(p[0]) for p in route]
        ys = [y] + [float(p[1]) for p in route]
        ax.plot(xs, ys, color=rgb, lw=1.6, zorder=4,
                solid_capstyle="round")
        ax.annotate("", xy=(xs[-1], ys[-1]), xytext=(xs[-2], ys[-2]),
                    arrowprops=dict(arrowstyle="-|>", color=rgb, lw=1.6),
                    zorder=4)
        ax.annotate(f"{key}  t+{walk.get('start_at', 0):.1f}s  "
                    f"{walk.get('speed', 0):.2f}m/s",
                    (xs[len(xs) // 2], ys[len(ys) // 2]),
                    textcoords="offset points", xytext=(0, 4), ha="center",
                    fontsize=6.5, color=rgb, zorder=6)


def _mission(ax, cfg) -> None:
    sx, sy = cfg["navigation"]["start"][:2]
    gx, gy = cfg["navigation"]["goal"][:2]
    for (x, y), label, colour in (((sx, sy), "AMR1 START", "#1052c8"),
                                  ((gx, gy), "AMR1 GOAL", "#e2600f")):
        ax.add_patch(plt.Circle((x, y), 0.45, facecolor="none",
                                edgecolor=colour, lw=2.0, zorder=6))
        ax.annotate(label, (x, y), textcoords="offset points",
                    xytext=(0, -17), ha="center", fontsize=7.5,
                    fontweight="bold", color=colour, zorder=7)

    places = cfg.get("fleet", {}).get("places", {})
    for name, item in sorted(cfg.get("fleet", {}).get("amrs", {}).items()):
        if not item.get("enabled", False):
            continue
        start = places.get(str(item.get("start", "")))
        if start is None:
            continue
        ax.add_patch(mpatches.Rectangle(
            (start[0] - 0.35, start[1] - 0.24), 0.70, 0.48,
            facecolor="#8b5cf6", alpha=0.7, edgecolor="#5b21b6", lw=0.8,
            zorder=6))
        ax.annotate(f"{name.upper()} t+{item.get('start_at', 0):.1f}s",
                    (start[0], start[1]), textcoords="offset points",
                    xytext=(0, 11), ha="center", fontsize=7,
                    fontweight="bold", color="#5b21b6", zorder=7)
        legs = [places[j] for j in item.get("jobs", []) if j in places]
        if legs:
            xs = [start[0]] + [p[0] for p in legs]
            ys = [start[1]] + [p[1] for p in legs]
            ax.plot(xs, ys, color="#8b5cf6", lw=1.0, ls="--", alpha=0.7,
                    zorder=4)


def main() -> int:
    args = parse_args()
    if args.config:
        path = args.config
    else:
        from comfort_core.scenario import ExperimentFactors, generate
        import yaml
        resolved = generate(ExperimentFactors(
            seed=args.seed, human_count=args.humans, amr_count=args.amrs,
            policy=args.policy, provider=args.provider))
        scratch = resolve_path("results/scenarios/.figure.yaml")
        scratch.parent.mkdir(parents=True, exist_ok=True)
        scratch.write_text(yaml.safe_dump(resolved.to_config(),
                                          sort_keys=False), encoding="utf-8")
        path = str(scratch)

    cfg = load_config(scenario=args.policy, common_path=path)
    meta = cfg.get("scenario_meta", {})
    policy = make_policy(cfg)

    bounds = build_layout(cfg)
    length = bounds.length
    width = bounds.width

    fig, ax = plt.subplots(figsize=(14.0, 14.0 * width / length))
    _bay(ax, cfg)
    _people(ax, cfg, policy)
    _mission(ax, cfg)

    ax.set_xlim(-length / 2 - 0.3, length / 2 + 0.3)
    ax.set_ylim(-width / 2 - 0.3, width / 2 + 0.3)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    title = meta.get("scenario_id", Path(path).stem)
    ax.set_title(
        f"{title}   |   policy {args.policy.upper()} "
        f"({cfg['comfort_space']['policy']}, "
        f"{cfg['comfort_space'].get('shape', 'circle')})   |   "
        f"{meta.get('human_count', '?')} humans, "
        f"{meta.get('amr_count', '?')} AMR(s), "
        f"mission {meta.get('mission', {}).get('distance', '?')} m",
        fontsize=10)
    handles = [
        mpatches.Patch(color=STATE_COLOUR["station_engaged"],
                       label="station_engaged"),
        mpatches.Patch(color=STATE_COLOUR["stationary"], label="stationary"),
        mpatches.Patch(color=STATE_COLOUR["moving"], label="moving"),
        plt.Line2D([], [], color=BELT_WALK, lw=1.6,
                   label="belt worker -> storage"),
        plt.Line2D([], [], color=EDGE_WALK, lw=1.6,
                   label="storage worker -> belt"),
    ]
    ax.legend(handles=handles, loc="upper center", ncol=5, fontsize=7.5,
              framealpha=0.9, bbox_to_anchor=(0.5, -0.08))
    fig.tight_layout()

    out = Path(args.out) if args.out else resolve_path(
        f"results/scenarios/{title}_{args.policy}.png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches="tight")
    print(f"  written: {out}")
    # The scratch config only existed to be loaded; a stale one left
    # lying about is a config somebody could run by mistake.
    if not args.config:
        resolve_path("results/scenarios/.figure.yaml").unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
