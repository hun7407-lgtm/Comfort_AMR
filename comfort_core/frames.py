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
from typing import Any, Dict, Sequence, Tuple

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


# ---------------------------------------------------------------------------
# One tf tree, several robots.
#
# The fleet AMRs share the global /tf with AMR1 and keep
# apart by frame prefix: a1/base_link next to base_link. Topics are split
# by ROS namespace instead: /a1/odom next to /odom. AMR1
# has no namespace, so every name below comes back unchanged for it, and
# its stack is the one every earlier result was measured on.
# ---------------------------------------------------------------------------

def prefixed(namespace: str, name: str) -> str:
    """`a1/odom` for a fleet AMR, `odom` for AMR1."""
    ns = str(namespace or "").strip("/")
    return f"{ns}/{name}" if ns else name


def namespaced_topic(namespace: str, topic: str) -> str:
    """`/a1/scan` for a fleet AMR, the topic as configured otherwise.

    Absolute on purpose. A relative name in a Nav2 parameter file is
    resolved against whichever node reads it, and the costmaps are
    nodes one namespace deeper than the server that owns them, so
    `scan` there means `/a1/local_costmap/scan`.
    """
    ns = str(namespace or "").strip("/")
    return f"/{ns}/{topic.lstrip('/')}" if ns else topic


def fleet_spawn(cfg: Dict[str, Any], name: str) -> Pose2D:
    """Where fleet AMR `name` spawns, and so where its odom frame sits.

    The position is its `start` place. The heading faces the nearer
    lane when the start is a cart slot -- the convention the measured
    robot spawns with, so the vehicle drives out of the slot forwards
    and its protective field points at open floor -- and otherwise
    points at the first job. isaac_sim/utils/fleet.py spawns it with
    the same call, so the static map -> a1/odom transform cannot
    disagree with the pose physics starts from.
    """
    fleet = cfg["fleet"]
    spec = fleet["amrs"][name]
    places = fleet["places"]
    x, y = (float(v) for v in places[str(spec["start"])])
    lanes = fleet.get("lanes", {})
    north = float(lanes.get("north_y", 0.0))
    south = float(lanes.get("south_y", 0.0))
    if lanes and abs(y - north) > 0.3 and abs(y - south) > 0.3:
        return (x, y, -math.pi / 2.0 if y > (north + south) / 2.0 else math.pi / 2.0)
    jobs = [str(j) for j in spec.get("jobs", [])]
    yaw = 0.0
    if jobs:
        tx, ty = (float(v) for v in places[jobs[0]])
        if (tx, ty) != (x, y):
            yaw = math.atan2(ty - y, tx - x)
    return (x, y, yaw)
