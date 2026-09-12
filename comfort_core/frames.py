"""Coordinate frame conversions.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

Why this exists
---------------
Isaac Sim's IsaacComputeOdometry reports the chassis pose relative to
wherever the chassis was when the simulation started. That is correct
ROS behaviour: the odom frame's origin is the robot's starting pose.

But every quantity in this experiment is defined in world coordinates:
the corridor, the stations, the humans, the Comfort Space circles, the
start and the goal. Reading /odom and comparing it against a world-frame
goal silently compares two different frames.

Observed in STEP 5 before this module existed: the robot spawned at
world (-4, 0), /odom reported (0, 0), and the controller drove to odom
(4, 0), which is world (0, 0). It reported SUCCESS having covered half
the intended distance.

Everything downstream should work in world coordinates and convert once,
here, at the boundary.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Pose2D = Tuple[float, float, float]  # x, y, yaw


def wrap_to_pi(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(base: Pose2D, local: Pose2D) -> Pose2D:
    """Express `local`, given in `base`'s frame, in `base`'s parent frame."""
    bx, by, byaw = base
    lx, ly, lyaw = local
    c, s = math.cos(byaw), math.sin(byaw)
    return (
        bx + c * lx - s * ly,
        by + s * lx + c * ly,
        wrap_to_pi(byaw + lyaw),
    )


def invert(pose: Pose2D) -> Pose2D:
    """The transform that undoes `pose`."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (-(c * x + s * y), -(-s * x + c * y), wrap_to_pi(-yaw))


def odom_origin_in_world(cfg) -> Pose2D:
    """Where the odom frame's origin sits in world coordinates.

    It is the robot's spawn pose, because that is where Isaac Sim zeroes
    the odometry. Both come from the same config keys, so the two cannot
    drift apart.
    """
    start: Sequence[float] = cfg["navigation"]["start"]
    yaw_deg = float(cfg["amr"]["spawn"]["yaw_deg"])
    return (float(start[0]), float(start[1]), math.radians(yaw_deg))


def odom_to_world(cfg, odom_pose: Pose2D) -> Pose2D:
    """Convert a pose read from /odom into world coordinates."""
    return compose(odom_origin_in_world(cfg), odom_pose)


def world_to_odom(cfg, world_pose: Pose2D) -> Pose2D:
    """Convert a world-frame pose into the odom frame."""
    return compose(invert(odom_origin_in_world(cfg)), world_pose)
