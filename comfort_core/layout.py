"""Corridor geometry: bounds, clearances and the pinch point.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

Every dimension comes from configs/common.yaml. Nothing here hardcodes a
number, because the whole point of the study is to sweep corridor width,
station spacing and human position and see when the passage closes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


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


def scene_bounds(cfg: Dict[str, Any]) -> Bounds:
    """The free-space rectangle of whichever scene is configured.

    The corridor is one long box; the factory bay is a square with
    machines in it. Everything that used to ask for the corridor asks
    for this instead, so the same yielding and tracking code works in
    both without knowing which it is in.
    """
    if "factory" in cfg:
        from comfort_core.factory_layout import build_layout
        layout = build_layout(cfg)
        return Bounds(layout.x_min, layout.x_max, layout.y_min, layout.y_max)
    return corridor_bounds(cfg)


def obstacle_boxes(cfg: Dict[str, Any]) -> List[Tuple[float, float, float, float]]:
    """Axis-aligned (xmin, xmax, ymin, ymax) of everything static.

    The robot may not wait inside one of these, and a scan return from
    one of them is not a person. Corridor stations or factory machines,
    depending on the scene.
    """
    if "factory" in cfg:
        from comfort_core.factory_layout import build_layout
        layout = build_layout(cfg)
        boxes = [(s.footprint.x_min, s.footprint.x_max,
                  s.footprint.y_min, s.footprint.y_max)
                 for s in layout.stations]
        # Columns and parked carts belong here too. A person detector
        # that only knows about machines calls a building column a
        # person: it is a narrow column reaching well past head height,
        # which is exactly the shape the detector looks for.
        half = float(cfg["factory"].get("column_size", 0.3)) / 2.0
        boxes += [(cx - half, cx + half, cy - half, cy + half)
                  for cx, cy in layout.columns]
        boxes += [(s.x_min, s.x_max, s.y_min, s.y_max)
                  for s in layout.cart_slots_taken]
        return boxes
    st = cfg["stations"]
    sx, sy = (float(v) for v in st["size"][:2])
    out = []
    for key in placed_entries(st):
        x, y = (float(v) for v in st[key]["position"])
        out.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
    return out


def drivable_area(cfg: Dict[str, Any]) -> Optional[List[Tuple[float, float, float, float]]]:
    """Rectangles the robot may occupy, or None for "anywhere inside".

    The corridor is one open box, so there is nothing to say. The
    factory bay is mostly work zones, and a robot that waits in one is
    standing where someone works.
    """
    if "factory" not in cfg:
        return None
    from comfort_core.factory_layout import build_layout, drivable_rects
    return [(r.x_min, r.x_max, r.y_min, r.y_max)
            for r in drivable_rects(build_layout(cfg))]


def plannable_area(cfg: Dict[str, Any]) -> Optional[List[Tuple[float, float, float, float]]]:
    """Rectangles the PLANNER can plan from -- the occupancy map's own.

    Not the same thing as `drivable_area`, and the difference cost a
    trial forty seconds of fighting itself.

    `drivable_area` is about where the robot may legitimately come to
    rest: it leaves out the work zones, because a robot waiting in one
    is standing where somebody works. The occupancy map is drawn from
    `navigable_rects` instead -- the robot may be SENT into a cart
    block, it just may not hide in one -- and it is painted with a
    skirt of `lane_margin` around every lane, because there is no wall
    at the edge of painted floor and treating one as a wall strands the
    robot the first time it steps over the line.

    So Nav2 will quite legitimately drive to a pose that
    `drivable_area` calls off-limits. The gate, reading that pose as
    "off the painted floor", creeped the robot back; Nav2 steered out
    again; and the two fought for forty seconds -- measured in the U
    bay, oscillating between y 4.37 and 5.24 at x 2.2 with the goal
    6.9 m west, until Nav2 gave up on the goal.

    The recovery exists for a robot standing where the planner CANNOT
    plan. That boundary is this one.
    """
    if "factory" not in cfg:
        return None
    from comfort_core.factory_layout import build_layout, navigable_rects
    margin = float(cfg.get("factory", {}).get("map", {})
                   .get("lane_margin", 0.0))
    return [(r.x_min - margin, r.x_max + margin,
             r.y_min - margin, r.y_max + margin)
            for r in navigable_rects(build_layout(cfg))]


def refuge_centres(cfg: Dict[str, Any]) -> List[Tuple[float, float]]:
    """Middle of every refuge, as somewhere worth trying to wait.

    Empty for the corridor, which has none: it is wide enough to stand
    aside in the lane the robot is already in.
    """
    if "factory" not in cfg:
        return []
    from comfort_core.factory_layout import build_layout
    return [r.centre for r in build_layout(cfg).refuges]


def work_spots(cfg: Dict[str, Any],
               stand_gap: float = 0.30) -> List[Tuple[float, float, float]]:
    """Where a person stands when they are working at a machine.

    This is the prior an AMR carries in its map: not where people are,
    but where the floor says somebody would be standing if they were
    working. It is the only way a LiDAR can tell a worker from a person
    who has simply stopped -- range data cannot see what someone is
    attending to, but the map knows there is a bench here and that
    anyone standing still in front of it is using it.

    Factory bays have the spots directly: the layout puts a worker in
    front of each machine. The corridor has stations without declared
    worker spots, so the spot is taken to be in front of the station,
    `stand_gap` clear of its face, toward the middle of the corridor --
    which is where configs/common.yaml's engaged person stands.

    Each spot is (x, y, facing): where to stand, and which way a person
    using that machine faces, in radians. The facing matters as much as
    the position -- somebody standing at a bench with their back to it
    is not working at it.
    """
    if "factory" in cfg:
        from comfort_core.factory_layout import build_layout
        return [(wx, wy, facing) for station in build_layout(cfg).stations
                for wx, wy, facing in station.workers]
    stations = cfg.get("stations", {})
    if not stations or "size" not in stations:
        return []
    _sx, sy, _sz = (float(v) for v in stations["size"])
    out: List[Tuple[float, float, float]] = []
    for key in placed_entries(stations):
        x, y = (float(v) for v in stations[key]["position"])
        side = math.copysign(1.0, y)
        out.append((x, y - side * (sy / 2.0 + stand_gap),
                    math.pi / 2.0 if side > 0 else -math.pi / 2.0))
    return out


def inside_drivable(rects, points, margin: float = 0.0) -> bool:
    """True if every point is inside at least one rectangle.

    Adjacent rectangles count as one area, which is what lets the robot
    sit half in a refuge and half in the aisle.
    """
    if rects is None:
        return True
    return all(any(x0 - margin <= px <= x1 + margin
                   and y0 - margin <= py <= y1 + margin
                   for x0, x1, y0, y1 in rects)
               for px, py in points)


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
