#!/usr/bin/env python3
"""The scenario, drawn over a plan view of the scene: who goes where.

    python3 scripts/scenario_figure.py --config configs/fleet_loop.yaml --scenario b3d \
        --background results/.../scene_fleet_loop_00_plan_ortho.png --out figure.png

Over the orthographic plan render from isaac_sim/scripts/scene_shots.py
(30 m across 1920 px, centred on the origin, +x right, +y up -- see
ORTHO_* there and here), this draws:

    * each AMR's route along the lane network, in its own colour
      (comfort_core/palette.py), with arrowheads for direction, a ring
      at its start and numbered discs at its destinations
    * each person, coloured by state, labelled with their role, and the
      walk they take if they take one, dashed, with when they set off

Routes are the roadmap's -- out to the lane, round, in -- not Nav2's
plan: this is the scenario as designed, which is what the figure is
for. With no --background it draws on a plain floor, which is enough
to check the drawing itself.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config  # noqa: E402
from comfort_core.palette import MISSION_COLOUR, fleet_colour  # noqa: E402
from comfort_core.labels import robot_label  # noqa: E402
from comfort_core.roadmap import Roadmap  # noqa: E402

Point = Tuple[float, float]

# Must match isaac_sim/scripts/scene_shots.py ORTHO_*.
ORTHO_WIDTH_M = 30.0
ORTHO_CENTRE = (0.0, 0.0)
IMG_W, IMG_H = 1920, 1080


def bgr(rgb) -> Tuple[int, int, int]:
    r, g, b = (int(round(c * 255)) for c in rgb)
    return (b, g, r)


class Plan:
    def __init__(self, image: np.ndarray):
        self.img = image
        h, w = image.shape[:2]
        self.scale = w / ORTHO_WIDTH_M
        self.cx, self.cy = w / 2.0, h / 2.0

    def px(self, x: float, y: float) -> Tuple[int, int]:
        return (int(round(self.cx + (x - ORTHO_CENTRE[0]) * self.scale)),
                int(round(self.cy - (y - ORTHO_CENTRE[1]) * self.scale)))

    @staticmethod
    def offset(pts: List[Point], d: float) -> List[Point]:
        """Shift a polyline sideways by d metres (drawing only).

        Four routes share the same lane centrelines; drawn on top of
        each other only the last one shows. Each AMR gets its own small
        offset, to its right in its direction of travel, which is also
        roughly where keep-right would put it.
        """
        if abs(d) < 1e-9 or len(pts) < 2:
            return list(pts)
        out = []
        for i, p in enumerate(pts):
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, len(pts) - 1)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(dx, dy) or 1.0
            # Right-hand normal of the travel direction.
            out.append((p[0] + dy / n * d, p[1] - dx / n * d))
        return out

    def polyline(self, pts: List[Point], colour, thick: int, dashed: bool = False,
                 arrows_every_m: float = 3.0, head_m: float = 0.16) -> None:
        for a, b in zip(pts, pts[1:]):
            pa, pb = self.px(*a), self.px(*b)
            if dashed:
                seg = math.dist(pa, pb)
                n = max(int(seg / 14), 1)
                for i in range(0, n, 2):
                    t0, t1 = i / n, min((i + 1) / n, 1.0)
                    q0 = (int(pa[0] + (pb[0] - pa[0]) * t0), int(pa[1] + (pb[1] - pa[1]) * t0))
                    q1 = (int(pa[0] + (pb[0] - pa[0]) * t1), int(pa[1] + (pb[1] - pa[1]) * t1))
                    cv2.line(self.img, q0, q1, colour, thick, cv2.LINE_AA)
            else:
                cv2.line(self.img, pa, pb, colour, thick, cv2.LINE_AA)
            # Arrowheads along the segment, so direction reads anywhere.
            length = math.dist(a, b)
            if length < 0.8:
                continue
            count = max(int(length // arrows_every_m), 1)
            for k in range(count):
                f = (k + 0.5) / count
                mid = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
                ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
                tip = (mid[0] + ux * head_m, mid[1] + uy * head_m)
                tail = (mid[0] - ux * head_m, mid[1] - uy * head_m)
                cv2.arrowedLine(self.img, self.px(*tail), self.px(*tip), colour,
                                thick, cv2.LINE_AA, tipLength=1.0)

    def ring(self, p: Point, r_m: float, colour, thick: int) -> None:
        cv2.circle(self.img, self.px(*p), int(r_m * self.scale), colour, thick, cv2.LINE_AA)

    def disc(self, p: Point, r_m: float, colour) -> None:
        cv2.circle(self.img, self.px(*p), int(r_m * self.scale), colour, -1, cv2.LINE_AA)

    def label(self, p: Point, text: str, colour, dy_px: int = -14, scale: float = 0.55,
              anchor: str = "center") -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, scale, 2)
        x, y = self.px(*p)
        x0 = x - tw // 2 if anchor == "center" else x
        y0 = y + dy_px
        cv2.rectangle(self.img, (x0 - 4, y0 - th - 4), (x0 + tw + 4, y0 + 4), (255, 255, 255), -1)
        cv2.rectangle(self.img, (x0 - 4, y0 - th - 4), (x0 + tw + 4, y0 + 4), colour, 1)
        cv2.putText(self.img, text, (x0, y0), font, scale, colour, 2, cv2.LINE_AA)


STATE_BGR = {"station_engaged": bgr((0.10, 0.45, 0.95)),
             "stationary": bgr((0.95, 0.75, 0.10)),
             "moving": bgr((0.90, 0.20, 0.20))}
STATE_KO = {"station_engaged": "engaged", "stationary": "standing", "moving": "walking"}


def mission_route(cfg, roadmap: Roadmap) -> List[Point]:
    start = tuple(float(v) for v in cfg["navigation"]["start"][:2])
    goal = tuple(float(v) for v in cfg["navigation"]["goal"][:2])
    roadmap.places["__goal__"] = goal
    return [start] + roadmap.route(start, "__goal__")


def fleet_routes(cfg, roadmap: Roadmap) -> Dict[str, List[Tuple[str, List[Point]]]]:
    """name -> [(job, polyline from the previous point to it), ...]."""
    out = {}
    fleet = cfg.get("fleet", {})
    if not fleet.get("enabled", False):
        return out
    for name, spec in sorted(fleet.get("amrs", {}).items()):
        if not spec.get("enabled", True):
            continue
        lanes = cfg["fleet"]["lanes"]
        mid = (float(lanes["north_y"]) + float(lanes["south_y"])) / 2.0
        roadmap.places.setdefault("via:east", (float(lanes["east_x"]), mid))
        roadmap.places.setdefault("via:west", (float(lanes["west_x"]), mid))
        here = roadmap.places[str(spec["start"])]
        legs = []
        pending: List[Point] = []
        for job in spec.get("jobs", []):
            pts = [here] + roadmap.route(here, str(job))
            here = roadmap.places[str(job)]
            if str(job).startswith("via:"):
                pending += pts[1:]      # ride along into the next real leg
                continue
            legs.append((str(job), ([pending[0]] if pending else []) + pending[1:] + pts
                         if pending else pts))
            pending = []
        out[name] = legs
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", required=True)
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--background", default=None, help="the 00_plan_ortho render")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default=None)
    args = ap.parse_args()
    cfg = load_config(scenario=args.scenario, common_path=args.config)

    if args.background:
        img = cv2.imread(args.background)
        if img is None:
            print(f"[ERROR] cannot read {args.background}", file=sys.stderr)
            return 1
        img = cv2.resize(img, (IMG_W, IMG_H)) if img.shape[1] != IMG_W else img
        # Mute the render so the drawing reads on top of it.
        img = cv2.addWeighted(img, 0.72, np.full_like(img, 255), 0.28, 0)
    else:
        img = np.full((IMG_H, IMG_W, 3), (236, 240, 236), dtype=np.uint8)
    plan = Plan(img)
    roadmap = Roadmap(cfg["fleet"])

    # -- the measured robot's mission -------------------------------
    mission = mission_route(cfg, roadmap)
    col = bgr(MISSION_COLOUR)
    plan.polyline(plan.offset(mission, 0.0), col, 4)
    plan.ring(mission[0], 0.45, col, 4)
    plan.disc(mission[-1], 0.40, col)
    plan.label(mission[0], f"{robot_label('AMR1')} START", col, dy_px=-40)
    plan.label(mission[-1], f"{robot_label('AMR1')} GOAL", col, dy_px=46)

    # -- the fleet --------------------------------------------------
    lateral = {"a1": 0.22, "a2": 0.44, "a3": 0.66}
    for index, (name, legs) in enumerate(fleet_routes(cfg, roadmap).items()):
        col = bgr(fleet_colour(name))
        tag = robot_label(name)
        start = legs[0][1][0] if legs else None
        # Labels staggered per vehicle so neighbours in one block do
        # not print on top of each other.
        row = 44 + 24 * index
        for order, (job, pts) in enumerate(legs, start=1):
            plan.polyline(plan.offset(pts, lateral.get(name, 0.2)), col, 3)
            end = pts[-1]
            if math.isclose(end[0], start[0]) and math.isclose(end[1], start[1]):
                continue  # the return leg ends at the start ring, labelled below
            plan.disc(end, 0.26, col)
            plan.label(end, f"{tag}-{order} {job}", col, dy_px=row if end[1] < 0 else -row,
                       scale=0.5)
        if start is not None:
            plan.ring(start, 0.45, col, 4)
            start_at = cfg["fleet"]["amrs"][name].get("start_at", 0.0)
            back = f" / {tag}-{len(legs)} back" if legs and math.isclose(
                legs[-1][1][-1][0], start[0]) and math.isclose(legs[-1][1][-1][1], start[1]) else ""
            plan.label(start, f"{tag} START t+{float(start_at):.1f}s{back}", col,
                       dy_px=row if start[1] < 0 else -row, scale=0.5)

    # -- the people -------------------------------------------------
    humans = cfg.get("humans", {})
    for key, spec in sorted(humans.items()):
        if not isinstance(spec, dict) or "position" not in spec or not spec.get("enabled", True):
            continue
        x, y = (float(v) for v in spec["position"])
        state = str(spec.get("state", "stationary"))
        col = STATE_BGR.get(state, (60, 60, 60))
        plan.disc((x, y), 0.25, col)
        cv2.circle(img, plan.px(x, y), int(0.25 * plan.scale), (255, 255, 255), 2, cv2.LINE_AA)
        facing = spec.get("facing_deg")
        if facing is not None:
            a = math.radians(float(facing))
            plan.polyline([(x, y), (x + 0.6 * math.cos(a), y + 0.6 * math.sin(a))], col, 2,
                          arrows_every_m=99.0)
            cv2.arrowedLine(img, plan.px(x, y),
                            plan.px(x + 0.6 * math.cos(a), y + 0.6 * math.sin(a)),
                            col, 2, cv2.LINE_AA, tipLength=0.5)
        walk = spec.get("walk") or {}
        route = walk.get("route") or spec.get("route") or []
        text = f"{key} {STATE_KO.get(state, state)}"
        if route:
            pts = [(x, y)] + [(float(p[0]), float(p[1])) for p in route]
            plan.polyline(pts, col, 2, dashed=True, arrows_every_m=2.0)
            text += f"  walks t+{float(walk.get('start_at', 0.0)):.1f}s"
        # People at the belts are labelled towards the belt (away from
        # the lane and the AMR labels); the two at the storage edge
        # towards the wall.
        toward_belt = abs(y) < 4.0
        if toward_belt:
            up = y < 0
            plan.label((x, y), text, col, dy_px=-28 if up else 36, scale=0.5)
        else:
            # The storage-edge people sit among the AMR labels; theirs
            # goes beside them instead of above or below.
            plan.label((x + 0.45, y), text, col, dy_px=6, scale=0.5, anchor="left")

    # -- legend and title -------------------------------------------
    # Inside the belt's U, the one piece of floor nothing of the
    # scenario crosses.
    lx, ly = plan.px(-6.3, 1.25)
    y0 = ly + 22
    items = [(f"{robot_label('AMR1')} mission (ring start, disc goal)", bgr(MISSION_COLOUR))]
    for name in sorted(cfg.get("fleet", {}).get("amrs", {})):
        if cfg["fleet"]["amrs"][name].get("enabled", True) and cfg["fleet"].get("enabled"):
            items.append((f"{robot_label(name)} job list", bgr(fleet_colour(name))))
    items += [(f"person: {v}", STATE_BGR[k]) for k, v in STATE_KO.items()]
    items.append(("dashed: a person's walk, with when it starts", (90, 90, 90)))
    box_h = 22 * len(items) + 16
    cv2.rectangle(img, (lx, y0 - 22), (lx + 544, y0 - 22 + box_h), (255, 255, 255), -1)
    cv2.rectangle(img, (lx, y0 - 22), (lx + 544, y0 - 22 + box_h), (120, 120, 120), 1)
    for i, (text, colour) in enumerate(items):
        yy = y0 + i * 22
        cv2.line(img, (lx + 12, yy - 6), (lx + 42, yy - 6), colour, 4)
        cv2.putText(img, text, (lx + 52, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    title = args.title or (f"{Path(args.config).stem}  |  "
                           f"{cfg.get('scenario_meta', {}).get('scenario_id', '')}  |  "
                           f"policy {cfg['comfort_space'].get('policy', '')}")
    cv2.putText(img, title, (IMG_W - 12 - cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0], 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 2, cv2.LINE_AA)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
