"""Navigation feasibility bound of a Comfort Space: R_path^infl, R_path^lethal.

Pure Python + numpy/scipy. No ROS, no omni.

The question this answers, before any simulation is run: in this
environment, with this person where they are and in the state they are
in, what is the largest Comfort Space the AMR can still drive past?

What "can drive past" means here is read off the code that decides it at
run time, not assumed:

  * The Comfort Space itself is a SOFT cost for Nav2 (KeepoutFilter mask,
    mask_value < 100, scripts/comfort_publisher.py), so a planned path
    always exists through it and cannot be the criterion.
  * What stops the robot is scripts/comfort_gate.py: it refuses any
    command whose predicted robot CENTRE lies inside the shape grown by
    the AMR's inscribed radius (comfort_space.robot_grow) plus the gate
    margin (comfort_space.yield.margin). That is the hard exclusion, and
    it is what is rasterised here.
  * Walls and static boxes: the robot centre is lethal within the
    inscribed radius and carries cost out to the inflation radius
    (comfort_core.nav2_tuning.inflation_radius). The two bounds differ
    only in which of the two is taken as the wall clearance:

        R_path^infl    centre kept inflation_radius from static obstacles
                       (conservative; R_nav_safe)
        R_path^lethal  centre kept the inscribed radius from them
                       (optimistic upper bound)

  * The person's body is a LiDAR obstacle as well: humans.radius plus the
    same clearance.

Nothing is counted twice: robot_grow and the gate margin act on the
Comfort Space only, inflation/inscribed radius on static obstacles and
the body only; passage_margin enters only through inflation_radius.
(comfort_core.layout.passage_report, by contrast, grows the shapes by the
inscribed radius AND asks for footprint + 2 * passage_margin of free
strip, counting the robot's half width twice. It stays as the scene
preview; it is not used for these predictions.)

"Effective extent" e is how far the person's Comfort Space (not grown)
reaches from the person's centre toward the robot's passage: the
quantity the Stage 1 x axis is in, whatever policy produced the shape.

> These are bounds of a model of the navigation stack, checked against
> the simulation in Stage 1 (R_exec). Not comfort distances.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy import ndimage

from comfort_core.comfort_space import (HeadingResolver, make_policy, robot_grow,
                                        shapes as build_shapes)
from comfort_core.human_state import make_state_provider
from comfort_core.layout import corridor_bounds, obstacle_boxes
from comfort_core.nav2_tuning import half_extents, inflation_radius

BOUNDS = ("infl", "lethal")

# The execution model (added 2026-11-14 after the Stage 1 smoke runs).
# Two terms the path bounds above leave out, both read from the stack:
#
#   * the planner keeps the robot centre out of the Comfort Space grown by
#     the inscribed radius AND the costmap buffer band
#     (comfort_space.costmap.buffer_width, soft cost) whenever a cost-free
#     channel exists -- so it plans its passage at e + grow + buffer from
#     the person, not at e + grow + gate margin;
#   * the collision monitor's protective field reaches sideways
#     footprint half-width + scanner tolerance (scripts/generate_nav2_params.py
#     `sy`, 0.34 m), the same as the inflation radius, so a robot driven
#     along the inflation edge has the wall on the edge of its field and
#     stops (seen in the smoke runs as ProtectiveStop next to the wall).
#
# The wall clearance the robot actually needs while passing is therefore
# not a constant of the code; it is `wall_clearance_m` of an execution
# model, fitted on the Stage 1A-1 calibration widths only
# (scripts/stage1a_calibrate.py) and frozen before the hold-outs run.


# ---------------------------------------------------------------- shapes
def comfort_shapes(cfg: Dict[str, Any], grow: float = 0.0) -> List[Any]:
    """Every person's Comfort Space at rest, as the gate builds it."""
    policy = make_policy(cfg)
    states = make_state_provider(cfg).states()
    if not states:
        return []
    return build_shapes(cfg, policy.axes(states),
                        headings=HeadingResolver(cfg, policy).resolve(states),
                        grow=grow)


def _inside(shape, X: np.ndarray, Y: np.ndarray, margin: float) -> np.ndarray:
    """Vectorised ComfortShape.contains."""
    a, b = shape.along + margin, shape.across + margin
    if a <= 0.0 or b <= 0.0:
        return np.zeros_like(X, dtype=bool)
    c, s = math.cos(shape.heading), math.sin(shape.heading)
    cx, cy = shape.centre
    u = (X - cx) * c + (Y - cy) * s
    v = -(X - cx) * s + (Y - cy) * c
    return (u / a) ** 2 + (v / b) ** 2 <= 1.0


def reach(shape, direction: Tuple[float, float]) -> float:
    """How far the shape reaches from the PERSON along a unit direction."""
    dx, dy = direction
    n = math.hypot(dx, dy)
    dx, dy = dx / n, dy / n
    # Support function of the ellipse, shifted from its centre to the person.
    c, s = math.cos(shape.heading), math.sin(shape.heading)
    u, v = dx * c + dy * s, -dx * s + dy * c
    support = math.hypot(shape.along * u, shape.across * v)
    cx, cy = shape.centre
    return support + (cx - shape.x) * dx + (cy - shape.y) * dy


def passage_direction(cfg: Dict[str, Any], key: str) -> Tuple[float, float]:
    """Unit vector from the person toward the robot's passage.

    The perpendicular from the person to the straight start-goal line:
    in the Stage 1 corridor that is straight across the aisle.
    """
    person = cfg["humans"][key]["position"]
    sx, sy = cfg["navigation"]["start"]
    gx, gy = cfg["navigation"]["goal"]
    lx, ly = gx - sx, gy - sy
    L = math.hypot(lx, ly)
    lx, ly = lx / L, ly / L
    px, py = float(person[0]) - sx, float(person[1]) - sy
    t = px * lx + py * ly
    fx, fy = sx + t * lx, sy + t * ly          # foot of the perpendicular
    dx, dy = fx - float(person[0]), fy - float(person[1])
    d = math.hypot(dx, dy)
    if d < 1e-9:
        raise ValueError(f"humans.{key} stands on the start-goal line; "
                         "no passage side is defined")
    return dx / d, dy / d


def effective_extent(cfg: Dict[str, Any], key: Optional[str] = None) -> Dict[str, float]:
    """e per person: reach of the (un-grown) Comfort Space toward the passage."""
    out = {}
    for shape in comfort_shapes(cfg, grow=0.0):
        if key is not None and shape.key != key:
            continue
        out[shape.key] = reach(shape, passage_direction(cfg, shape.key))
    return out


# ---------------------------------------------------------------- grid
@dataclass
class Grid:
    x0: float
    y0: float
    res: float
    nx: int
    ny: int

    def mesh(self) -> Tuple[np.ndarray, np.ndarray]:
        xs = self.x0 + (np.arange(self.nx) + 0.5) * self.res
        ys = self.y0 + (np.arange(self.ny) + 0.5) * self.res
        return np.meshgrid(xs, ys)

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return int((y - self.y0) / self.res), int((x - self.x0) / self.res)


def _static_occupancy(cfg: Dict[str, Any], res: float) -> Tuple[Grid, np.ndarray]:
    """Static obstacles: everything outside the free area, plus static boxes.

    Corridor: the box between the walls. Factory bay: the planner's own area
    (comfort_core.layout.plannable_area, the rectangles the occupancy map is
    drawn from), minus machines, columns and taken cart slots
    (layout.obstacle_boxes). Painted floor has no wall, but the map treats
    its edge as one, so the same clearance applies -- conservative where
    no physical wall stands. One cell of obstacle all round."""
    if "factory" in cfg:
        from comfort_core.layout import plannable_area, scene_bounds
        b = scene_bounds(cfg)
        pad = 2 * res
        grid = Grid(b.x_min - pad, b.y_min - pad, res,
                    int(math.ceil((b.x_max - b.x_min + 2 * pad) / res)),
                    int(math.ceil((b.y_max - b.y_min + 2 * pad) / res)))
        X, Y = grid.mesh()
        free = np.zeros_like(X, dtype=bool)
        for x0, x1, y0, y1 in plannable_area(cfg) or []:
            free |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        occ = ~free
        for x0, x1, y0, y1 in obstacle_boxes(cfg):
            occ |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        # A LOCAL passage: the bay is a loop, and a path round the far side
        # would call any Comfort Space passable. `_feasibility_window`
        # [x_min, x_max] confines the search to the stretch being passed.
        win = cfg.get("_feasibility_window")
        if win:
            occ |= (X < float(win[0])) | (X > float(win[1]))
        return grid, occ
    b = corridor_bounds(cfg)
    pad = 2 * res
    grid = Grid(b.x_min - pad, b.y_min - pad, res,
                int(math.ceil((b.x_max - b.x_min + 2 * pad) / res)),
                int(math.ceil((b.y_max - b.y_min + 2 * pad) / res)))
    X, Y = grid.mesh()
    occ = ~((X >= b.x_min) & (X <= b.x_max) & (Y >= b.y_min) & (Y <= b.y_max))
    for x0, x1, y0, y1 in obstacle_boxes(cfg):
        occ |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    return grid, occ


def wall_clearance(cfg: Dict[str, Any], bound: str) -> float:
    """Clearance of the robot centre from static obstacles, per bound."""
    if bound == "infl":
        return float(inflation_radius(cfg))
    if bound == "lethal":
        return float(half_extents(cfg)[1])
    raise ValueError(f"bound must be one of {BOUNDS}")


def comfort_buffer(cfg: Dict[str, Any]) -> float:
    return float(cfg["comfort_space"].get("costmap", {}).get("buffer_width", 0.0) or 0.0)


def free_space(cfg: Dict[str, Any], bound: str, res: float = 0.01,
               model: Optional[Dict[str, float]] = None) -> Tuple[Grid, np.ndarray]:
    """Where the robot CENTRE may be: True = free.

    bound 'infl' / 'lethal': the gate's exclusion and the static clearance.
    bound 'exec': the execution model -- Comfort Space grown by the inscribed
    radius plus the costmap buffer, walls and body kept at
    model['wall_clearance_m'].
    """
    grid, occ = _static_occupancy(cfg, res)
    if bound == "exec":
        if not model or model.get("wall_clearance_m") is None:
            raise ValueError("bound 'exec' needs a fitted model with wall_clearance_m")
        clear = float(model["wall_clearance_m"])
        grow = robot_grow(cfg) + comfort_buffer(cfg)
        margin = 0.0
    else:
        clear = wall_clearance(cfg, bound)
        grow = robot_grow(cfg)
        margin = float(cfg["comfort_space"].get("yield", {}).get("margin", 0.05))
    dist = ndimage.distance_transform_edt(~occ) * res
    free = dist > clear
    X, Y = grid.mesh()
    body = float(cfg["humans"].get("radius", 0.25)) + clear
    for shape in comfort_shapes(cfg, grow=grow):
        free &= (X - shape.x) ** 2 + (Y - shape.y) ** 2 > body ** 2
        free &= ~_inside(shape, X, Y, margin)
    return grid, free


def path_exists(cfg: Dict[str, Any], bound: str, res: float = 0.01,
                model: Optional[Dict[str, float]] = None) -> bool:
    grid, free = free_space(cfg, bound, res, model)
    labels, _ = ndimage.label(free)
    s = grid.cell(*cfg["navigation"]["start"])
    g = grid.cell(*cfg["navigation"]["goal"])
    try:
        ls, lg = labels[s], labels[g]
    except IndexError:
        return False
    return bool(ls) and ls == lg


# ---------------------------------------------------------------- search
def with_size(cfg: Dict[str, Any], kind: str, value: float) -> Dict[str, Any]:
    """The same scene with the Comfort Space set to `value`.

    kind 'static': static_radius; kind 'b3o': aisle_scaling target_size_m
    (whole-geometry scale, floor 0), exactly as comfort_core.size_sweep
    writes the condition overlays.
    """
    c = copy.deepcopy(cfg)
    cs = c["comfort_space"]
    if kind == "static":
        cs["policy"] = "static"
        cs["static_radius"] = float(value)
    elif kind == "b3o":
        cs["policy"] = "state_directional"
        sc = dict(cs.get("aisle_scaling") or {})
        sc.update({"enabled": True, "target_size_m": float(value), "floor_m": 0.0,
                   "scale": 1.0})
        cs["aisle_scaling"] = sc
    else:
        raise ValueError(f"kind must be static or b3o, not {kind!r}")
    return c


def extent_of(cfg: Dict[str, Any], kind: str, value: float, key: str) -> float:
    return effective_extent(with_size(cfg, kind, value), key)[key]


def value_for_extent(cfg: Dict[str, Any], kind: str, e: float, key: str) -> float:
    """The policy parameter that gives effective extent e (e is linear in it
    above the min_radius floor, so one ratio is exact)."""
    if kind == "static":
        return float(e)
    ref = 1.20
    k = extent_of(cfg, kind, ref, key) / ref
    return float(e) / k


def r_path(cfg: Dict[str, Any], kind: str, key: str, bound: str,
           lo: float = 0.05, hi: float = 4.0, tol: float = 0.002,
           res: float = 0.01, model: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    """Largest parameter (and its effective extent) with a feasible path.

    Bisection on the policy parameter; feasibility is monotone in it
    because every extent grows with it.
    """
    if not path_exists(with_size(cfg, kind, lo), bound, res, model):
        return {"bound": bound, "value": None, "e": None,
                "note": f"infeasible already at {lo} (environment blocks the passage)"}
    if path_exists(with_size(cfg, kind, hi), bound, res, model):
        return {"bound": bound, "value": hi, "e": extent_of(cfg, kind, hi, key),
                "note": f"feasible up to the search limit {hi}"}
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if path_exists(with_size(cfg, kind, mid), bound, res, model):
            lo = mid
        else:
            hi = mid
    return {"bound": bound, "value": lo, "e": extent_of(cfg, kind, lo, key), "note": ""}


def predict(cfg: Dict[str, Any], kind: str, key: str, res: float = 0.01,
            model: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    """R_path^infl and R_path^lethal for one person (and R_exec_model when a
    fitted execution model is given), plus the closed form when the scene
    is a straight corridor with the person beside a wall."""
    out = {b: r_path(cfg, kind, key, b, res=res) for b in BOUNDS}
    if model:
        out["exec"] = r_path(cfg, kind, key, "exec", res=res, model=model)
    out["closed_form"] = corridor_closed_form(cfg, key, model)
    return out


def corridor_closed_form(cfg: Dict[str, Any], key: str,
                         model: Optional[Dict[str, float]] = None) -> Optional[Dict[str, float]]:
    """e_max = (distance from the person to the far wall) - wall clearance
    - (inscribed radius + gate margin). None outside the plain corridor."""
    if "factory" in cfg:
        return None
    b = corridor_bounds(cfg)
    x, y = (float(v) for v in cfg["humans"][key]["position"])
    dx, dy = passage_direction(cfg, key)
    if abs(dx) > 1e-6:
        return None
    far = (y - b.y_min) if dy < 0 else (b.y_max - y)
    grow = robot_grow(cfg) + float(cfg["comfort_space"].get("yield", {}).get("margin", 0.05))
    out = {bd: far - wall_clearance(cfg, bd) - grow for bd in BOUNDS}
    if model and model.get("wall_clearance_m") is not None:
        out["exec"] = (far - float(model["wall_clearance_m"])
                       - robot_grow(cfg) - comfort_buffer(cfg))
    return out
