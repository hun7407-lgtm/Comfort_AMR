"""Corridor geometry: bounds, clearances and the pinch point.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

Every dimension comes from configs/common.yaml. Nothing here hardcodes a
number, because the whole point of the study is to sweep corridor width,
station spacing and human position and see when the passage closes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple


@dataclass(frozen=True)
class Bounds:
    """Axis-aligned rectangle of free floor between the walls."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.x_min + margin <= x <= self.x_max - margin
                and self.y_min + margin <= y <= self.y_max - margin)


def corridor_bounds(cfg: Dict[str, Any]) -> Bounds:
    """Inner free space of the corridor, wall faces excluded."""
    c = cfg["corridor"]
    cx, cy = c["center"]
    half_len = float(c["length"]) / 2.0
    half_wid = float(c["width"]) / 2.0
    return Bounds(cx - half_len, cx + half_len, cy - half_wid, cy + half_wid)


def placed_entries(section: Dict[str, Any]) -> List[str]:
    """Keys of a config section that describe something with a position.

    A section like `humans` or `stations` mixes per-item entries with
    shared settings, and both are dictionaries. Selecting on the presence
    of a `position` key is what tells `humans.a` apart from
    `humans.character`. Sorted, so scene building and mask painting always
    walk them in the same order.
    """
    return sorted(k for k, v in section.items()
                  if isinstance(v, dict) and "position" in v
                  and v.get("enabled", True))


def human_positions(cfg: Dict[str, Any]) -> List[Tuple[str, float, float]]:
    """Every human as (key, x, y), in a stable order."""
    humans = cfg["humans"]
    out = []
    for key in placed_entries(humans):
        x, y = humans[key]["position"]
        out.append((key, float(x), float(y)))
    return out


def human_states(cfg: Dict[str, Any]) -> Dict[str, str]:
    """Ground-truth state of every human, keyed the same way."""
    humans = cfg["humans"]
    return {k: str(humans[k]["state"]) for k in placed_entries(humans)}


@dataclass(frozen=True)
class PassageReport:
    """Whether the AMR can get from start to goal, and how tight it is."""
    passable: bool
    clearance: float          # widest free strip at the tightest cross-section
    required: float           # what the AMR needs
    tightest_x: float         # where that cross-section is


def robot_clearance_radius(cfg: Dict[str, Any]) -> float:
    """Half-width the AMR needs around its centre line.

    The inscribed radius plus the configured margin, not the
    circumscribed radius. A differential base can rotate to line up with
    a gap, so its width rather than its diagonal is what has to fit.
    """
    half_width = float(cfg["amr"]["footprint_width"]) / 2.0
    margin = float(cfg["navigation"].get("passage_margin", 0.10))
    return half_width + margin


def required_width(cfg: Dict[str, Any]) -> float:
    """Lateral space the AMR needs to fit through, including margin."""
    return 2.0 * robot_clearance_radius(cfg)


def free_intervals_at(cfg: Dict[str, Any], x: float,
                      radii: Dict[str, float] | None = None,
                      shapes: List = None) -> List[Tuple[float, float]]:
    """Unblocked spans of y at a given x, as (low, high) pairs.

    A person only narrows the corridor near their own x, so each Comfort
    Space contributes a band of y and nothing outside it.

    `shapes` takes precedence when given, which is how a walking
    person's ellipse and a live position reach this calculation. Without
    it the configured positions and circular radii are used.
    """
    bounds = corridor_bounds(cfg)
    default_radius = float(cfg["humans"]["radius"])

    blocked: List[Tuple[float, float]] = []
    if shapes is not None:
        for shape in shapes:
            span = shape.y_span_at(x)
            if span is not None:
                blocked.append(span)
    else:
        for key, hx, hy in human_positions(cfg):
            r = float((radii or {}).get(key, default_radius))
            dx = abs(x - hx)
            if dx >= r:
                continue
            half = math.sqrt(r * r - dx * dx)
            blocked.append((hy - half, hy + half))

    blocked.sort()
    free: List[Tuple[float, float]] = []
    cursor = bounds.y_min
    for lo, hi in blocked:
        if lo > cursor:
            free.append((cursor, lo))
        cursor = max(cursor, hi)
    if cursor < bounds.y_max:
        free.append((cursor, bounds.y_max))
    return free


def passage_report(cfg: Dict[str, Any],
                   radii: Dict[str, float] | None = None,
                   samples: int = 400,
                   shapes: List = None) -> PassageReport:
    """Can the AMR get past the humans, and where is it tightest?

    Why this is not a single number
    -------------------------------
    An earlier version of this module summed every human's blocked band
    onto one y axis, as if all the humans stood at the same x. That is
    wrong for the staggered layout this study uses. With Station A at
    x = -2 and Station B at x = +2, the two Comfort Spaces may not
    overlap along x at all, and the AMR simply weaves: it passes A on
    one side and B on the other. The one-axis model called that blocked
    when it is not.

    So the corridor is scanned along x and the widest free strip is
    measured at each cross-section. The passage is blocked when some
    cross-section leaves nothing wide enough, because the AMR cannot get
    past that x at any y.

    This assumes the free strip is one connected corridor along x, which
    holds for circular obstacles between two straight walls. It would
    need a real connectivity search for a more complex room.
    """
    bounds = corridor_bounds(cfg)
    required = required_width(cfg)

    tightest = float("inf")
    tightest_x = bounds.x_min
    step = (bounds.x_max - bounds.x_min) / max(samples - 1, 1)

    for i in range(samples):
        x = bounds.x_min + i * step
        intervals = free_intervals_at(cfg, x, radii, shapes)
        widest = max((hi - lo for lo, hi in intervals), default=0.0)
        if widest < tightest:
            tightest = widest
            tightest_x = x

    return PassageReport(
        passable=tightest >= required,
        clearance=tightest,
        required=required,
        tightest_x=tightest_x,
    )


def validate(cfg: Dict[str, Any]) -> List[str]:
    """Check the configured layout for mistakes. Returns problem strings.

    Cheap to run and worth running every time the scene is built: a typo
    that puts a human inside a wall, or a goal outside the corridor,
    produces a trial that fails for the wrong reason.
    """
    problems: List[str] = []
    if not cfg["corridor"].get("enabled", True):
        return problems

    bounds = corridor_bounds(cfg)
    human_r = float(cfg["humans"]["radius"])

    for name in ("start", "goal"):
        x, y = cfg["navigation"][name]
        if not bounds.contains(float(x), float(y)):
            problems.append(
                f"navigation.{name} ({x}, {y}) is outside the corridor "
                f"x[{bounds.x_min}, {bounds.x_max}] y[{bounds.y_min}, {bounds.y_max}]")

    for key, x, y in human_positions(cfg):
        if not bounds.contains(x, y, margin=human_r):
            problems.append(
                f"humans.{key} at ({x}, {y}) with radius {human_r} "
                f"overlaps a corridor wall")

    sx, sy = (float(v) for v in cfg["navigation"]["start"])
    gx, gy = (float(v) for v in cfg["navigation"]["goal"])
    for key, x, y in human_positions(cfg):
        for label, px, py in (("start", sx, sy), ("goal", gx, gy)):
            if math.hypot(px - x, py - y) < human_r + 0.5:
                problems.append(
                    f"humans.{key} at ({x}, {y}) is on top of "
                    f"navigation.{label} ({px}, {py})")

    stations = cfg["stations"]
    sxe, sye, _ = (float(v) for v in stations["size"])
    for key in placed_entries(stations):
        x, y = (float(v) for v in stations[key]["position"])
        if abs(y) + sye / 2.0 > abs(bounds.y_max) + 1e-9:
            problems.append(
                f"stations.{key} at ({x}, {y}) with depth {sye} "
                f"sticks through the corridor wall")

    return problems
