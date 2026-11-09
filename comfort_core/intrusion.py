"""How far the AMR got into someone's Comfort Space.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

comfort_core/clearance.py measures the physical gap. This measures the
social one: whether any part of the robot's footprint was inside a
person's Comfort Space, and how deep.

The Comfort Space radius is defined from the person's centre to the
nearest part of the AMR, so the shape used here is the raw one, not the
copy enlarged by the robot's half width that the planner works with.

The footprint is sampled along its outline. For each sample the
ellipse's scale factor k is computed: k = 1 on the boundary, k < 1
inside. The smallest k over the outline is the worst point, and the
depth reported is how far that point would have to move straight away
from the person to reach the boundary.

> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Tuple

from comfort_core.shapes import ComfortShape

Point = Tuple[float, float]


def footprint_outline(pose: Tuple[float, float, float], half_length: float,
                      half_width: float, per_edge: int = 6) -> List[Point]:
    """Points spaced along the footprint rectangle's four edges."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(half_length, half_width), (half_length, -half_width),
               (-half_length, -half_width), (-half_length, half_width)]
    out = []
    for i in range(4):
        (u0, v0), (u1, v1) = corners[i], corners[(i + 1) % 4]
        for j in range(per_edge):
            f = j / per_edge
            u, v = u0 + (u1 - u0) * f, v0 + (v1 - v0) * f
            out.append((x + u * c - v * s, y + u * s + v * c))
    return out


def ellipse_scale(shape: ComfortShape, px: float, py: float) -> float:
    """k for one point: 1 on the boundary, below 1 inside.

    Measured from the ELLIPSE's centre, which is not the person's
    position once the shape is offset along their heading. Using the
    person's position here would read an asymmetric space as if it were
    centred on them, reporting an intrusion behind a standing worker as
    shallower than it is and one in front as deeper.
    """
    c, s = math.cos(shape.heading), math.sin(shape.heading)
    cx, cy = shape.centre
    dx, dy = px - cx, py - cy
    u = dx * c + dy * s
    v = -dx * s + dy * c
    return math.hypot(u / max(shape.along, 1e-9), v / max(shape.across, 1e-9))


def intrusion(shape: ComfortShape, outline: Iterable[Point]) -> Tuple[float, float]:
    """(k_min, depth in metres) of a footprint outline against one shape.

    k_min >= 1 means the footprint stayed outside; depth is then 0.
    """
    best_k, depth = math.inf, 0.0
    for px, py in outline:
        k = ellipse_scale(shape, px, py)
        if k < best_k:
            best_k = k
            if k < 1.0:
                cx, cy = shape.centre
                d = math.hypot(px - cx, py - cy)
                depth = min(shape.along, shape.across) if k < 1e-6 else d * (1.0 / k - 1.0)
            else:
                depth = 0.0
    return best_k, depth


def bearing_sector(shape, x: float, y: float) -> str:
    """Where the robot is, seen from the person: front, side or rear.

    By the shape's heading -- the direction the policy sized it by,
    the person's body or their velocity -- so "front" means the side
    the Comfort Space is largest on. What the directional policy
    claims is that intrusions can be allowed behind a person that are
    not allowed in front; this is the column that can show it.
    """
    import math as _m
    a = _m.atan2(y - shape.y, x - shape.x) - shape.heading
    a = abs(_m.atan2(_m.sin(a), _m.cos(a)))
    return "front" if a < _m.pi / 4 else ("rear" if a > 3 * _m.pi / 4 else "side")
