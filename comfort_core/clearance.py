"""Physical clearance between the AMR and a person.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

The Comfort Space says how close the robot *should* get. This says how
close it actually got, which is the number that shows whether a walker
went through the robot. Both shapes are the ones physics uses: the AMR's
footprint rectangle, centred on base_footprint, and the person's upright
cylinder of `humans.radius`.

A clearance of zero or less means the two bodies touched.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple


def footprint_half_extents(cfg: Dict[str, Any]) -> Tuple[float, float]:
    """(half length, half width) of the AMR footprint, as Nav2 is given it."""
    amr = cfg["amr"]
    return float(amr["footprint_length"]) / 2.0, float(amr["footprint_width"]) / 2.0


def rectangle_clearance(pose: Tuple[float, float, float],
                        half_length: float, half_width: float,
                        px: float, py: float, radius: float) -> float:
    """Gap between a yawed rectangle and a circle, in metres.

    Negative when they overlap. Computed exactly: the point is moved into
    the rectangle's frame, where the nearest point on the rectangle is a
    clamp on each axis.
    """
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    u = dx * c + dy * s
    v = -dx * s + dy * c
    qu = abs(u) - half_length
    qv = abs(v) - half_width
    if qu <= 0.0 and qv <= 0.0:
        # Centre of the person is inside the footprint.
        return max(qu, qv) - radius
    outside = math.hypot(max(qu, 0.0), max(qv, 0.0))
    return outside - radius


def clearance(cfg: Dict[str, Any], pose: Tuple[float, float, float],
              px: float, py: float) -> float:
    """Gap between the AMR at `pose` and a person standing at (px, py)."""
    half_length, half_width = footprint_half_extents(cfg)
    return rectangle_clearance(pose, half_length, half_width, px, py,
                               float(cfg["humans"]["radius"]))
