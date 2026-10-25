"""Build the shared corridor scene in Isaac Sim.

Isaac Sim only: imports omni and pxr. Never import this from a ROS 2 node.

Everything here is driven by configs/common.yaml. B2 and B3 call the same
function with the same config, so the physical environment is identical
by construction rather than by discipline.

Geometry only. Comfort Space visualisation lives in
isaac_sim/utils/comfort_viz.py (STEP 7) so that changing how a Comfort
Space is drawn can never change where a wall is.
"""

from __future__ import annotations

from typing import Any, Dict, List

import numpy as np
from isaacsim.core.api.objects import FixedCuboid, FixedCylinder
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from comfort_core.layout import corridor_bounds, human_positions, placed_entries

ENV_ROOT = "/Environment/Corridor"

WALL_COLOR = np.array([0.55, 0.55, 0.58])
STATION_COLOR = np.array([0.20, 0.35, 0.60])
HUMAN_COLOR = np.array([0.85, 0.55, 0.25])


def _wall(prim_path: str, center, size) -> FixedCuboid:
    """A static box with collision. Walls and stations are both this."""
    return FixedCuboid(
        prim_path=prim_path,
        position=np.array([float(center[0]), float(center[1]), float(center[2])]),
        scale=np.array([float(size[0]), float(size[1]), float(size[2])]),
        color=WALL_COLOR,
    )


def build_walls(cfg: Dict[str, Any]) -> List[FixedCuboid]:
    """Two side walls, optionally capped at both ends.

    Wall centres sit half a thickness outside the free-space bounds, so
    the corridor's usable width is exactly `corridor.width`.
    """
    c = cfg["corridor"]
    bounds = corridor_bounds(cfg)
    h = float(c["wall_height"])
    t = float(c["wall_thickness"])
    z = h / 2.0

    length = bounds.x_max - bounds.x_min
    width = bounds.y_max - bounds.y_min
    cx = (bounds.x_min + bounds.x_max) / 2.0
    cy = (bounds.y_min + bounds.y_max) / 2.0

    walls = []
    # Side walls run the full length, including the end-wall footprint,
    # so the corners meet without a gap.
    outer_len = length + (2.0 * t if c.get("end_walls", True) else 0.0)
    for name, sign in (("wall_y_pos", +1.0), ("wall_y_neg", -1.0)):
        walls.append(_wall(
            f"{ENV_ROOT}/{name}",
            (cx, cy + sign * (width / 2.0 + t / 2.0), z),
            (outer_len, t, h),
        ))

    if c.get("end_walls", True):
        for name, sign in (("wall_x_pos", +1.0), ("wall_x_neg", -1.0)):
            walls.append(_wall(
                f"{ENV_ROOT}/{name}",
                (cx + sign * (length / 2.0 + t / 2.0), cy, z),
                (t, width, h),
            ))

    return walls


def build_stations(cfg: Dict[str, Any]) -> List[FixedCuboid]:
    """Workstations against the corridor walls."""
    stations = cfg["stations"]
    sx, sy, sz = (float(v) for v in stations["size"])

    out = []
    for key in placed_entries(stations):
        x, y = (float(v) for v in stations[key]["position"])
        out.append(FixedCuboid(
            prim_path=f"{ENV_ROOT}/station_{key}",
            position=np.array([x, y, sz / 2.0]),
            scale=np.array([sx, sy, sz]),
            color=STATION_COLOR,
        ))
    return out


def build_humans(cfg: Dict[str, Any]) -> List[FixedCylinder]:
    """Humans as upright cylinders.

    Static colliders on purpose. At this stage a human is a fixed
    obstacle with a position and a state; walking humans and articulated
    characters are out of scope. What the experiment needs is that the
    AMR cannot drive through a person.
    """
    humans = cfg["humans"]
    radius = float(humans["radius"])
    height = float(humans["height"])

    out = []
    movers = []
    for key, x, y in human_positions(cfg):
        cylinder = FixedCylinder(
            prim_path=f"{ENV_ROOT}/human_{key}",
            position=np.array([x, y, height / 2.0]),
            radius=radius,
            height=height,
            color=HUMAN_COLOR,
        )
        # Anybody who moves at any point in the trial. `state: moving`
        # is somebody already walking when it starts; `walk` is a
        # worker who leaves her machine part way through.
        #
        # A static collider cannot be moved. The pose set on it goes
        # into the physics view and never reaches USD, so /tf keeps
        # publishing the spawn point: the character walked across the
        # aisle, the collider stayed at the machine, and every
        # clearance the trial measured was to where she used to be.
        # Nothing reported an error -- reading the pose back returned
        # the value just written.
        if (str(humans[key].get("state", "")) == "moving"
                or humans[key].get("walk")):
            _make_kinematic(cylinder.prim)
            movers.append(key)
        out.append(cylinder)
    if movers:
        print(f"  kinematic collider(s) for {', '.join(sorted(movers))} "
              f"(these follow a walking character)", flush=True)
    return out


def _make_kinematic(prim) -> None:
    """Let a collider be moved every frame without upsetting the solver.

    A walking person's collider has to follow them. Teleporting a static
    collider each step makes PhysX rebuild its broadphase entry from
    scratch and gives contacts no history; a kinematic rigid body is the
    supported way to move geometry under script control, and it still
    pushes nothing around, which is what a person should do to an AMR
    that is supposed to yield rather than be shoved.
    """
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr().Set(True)
    rb.CreateRigidBodyEnabledAttr().Set(True)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim)


def build_corridor(cfg: Dict[str, Any]) -> Dict[str, int]:
    """Build walls, stations and humans. Returns how many of each."""
    if not cfg["corridor"].get("enabled", True):
        return {"walls": 0, "stations": 0, "humans": 0}

    return {
        "walls": len(build_walls(cfg)),
        "stations": len(build_stations(cfg)),
        "humans": len(build_humans(cfg)),
    }
