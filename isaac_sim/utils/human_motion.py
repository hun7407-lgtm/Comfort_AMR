"""Keep a walking human's collider on top of their animated character.

Isaac Sim only: imports omni and pxr.

omni.anim.people drives the character's root transform, and the
character asset carries no physics. The collider that the LiDAR, the
Comfort Space and the yield policy all reason about is a separate
cylinder, so something has to move it in step with the animation. That
is all this does: read where the character is, put the collider there.

The collider is the authoritative position, not the mesh. It is what
gets published on /tf, what the geometry checks size, and what the AMR
can actually run into.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

import carb
import numpy as np
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import get_current_stage

from pxr import Usd

from comfort_core.comfort_space import make_policy, robot_grow, shapes as build_shapes
from comfort_core.human_state import make_state_provider
from comfort_core.layout import human_positions
from isaac_sim.utils import comfort_viz


class WalkerSync:
    """Copies each walking character's pose onto its collider each step."""

    def __init__(self, cfg: Dict[str, Any]):
        self.pairs: List[Tuple[str, str, str]] = []
        stage = get_current_stage()
        humans = cfg["humans"]

        for key, _x, _y in human_positions(cfg):
            if str(humans[key].get("state", "")) != "moving":
                continue
            root = f"/World/Characters/human_{key}"
            collider = f"/Environment/Corridor/human_{key}"
            if not (stage.GetPrimAtPath(root).IsValid()
                    and stage.GetPrimAtPath(collider).IsValid()):
                continue
            skel = None
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.GetTypeName() == "SkelRoot":
                    skel = str(prim.GetPrimPath())
                    break
            self.pairs.append((key, skel or root, collider))

        self.height = float(humans["height"])
        self.cfg = cfg
        self.policy = make_policy(cfg)
        self.declared = make_state_provider(cfg).states()
        self.buffer_width = float(
            cfg["comfort_space"].get("costmap", {}).get("buffer_width", 0.0))
        self.grow = robot_grow(cfg)
        self._prev_pos: Dict[str, Tuple[float, float]] = {}
        self._speeds: Dict[str, float] = {}
        self._headings: Dict[str, float] = {}
        # Physics owns a kinematic body's transform and writes it back
        # every step, so setting xformOp:translate on the prim did
        # nothing: the collider sat at its spawn point while the
        # character walked away, and the Comfort Space stayed behind
        # with it. Moving it through the rigid body wrapper sets the
        # kinematic target, which is the supported way.
        self._bodies: Dict[str, SingleRigidPrim] = {}
        self._ticks = 0

    def __bool__(self) -> bool:
        return bool(self.pairs)

    def describe(self) -> List[str]:
        return [f"collider for human {key} follows the walking character"
                for key, _c, _p in self.pairs]

    def _track(self, key: str, x: float, y: float, dt: float) -> None:
        """Estimate speed and heading from successive positions."""
        px, py = self._prev_pos.get(key, (x, y))
        self._prev_pos[key] = (x, y)
        if dt <= 0.0:
            return
        vx, vy = (x - px) / dt, (y - py) / dt
        speed = math.hypot(vx, vy)
        self._speeds[key] = speed
        if speed > 1e-3:
            self._headings[key] = math.atan2(vy, vx)

    def _redraw(self) -> None:
        """Resize and move the drawn Comfort Spaces to match the walkers.

        The picture has to track the same speed-dependent size the
        costmap mask does, or the viewport shows a circle where the
        planner sees a stretched ellipse.
        """
        axes = self.policy.axes(self.declared, self._speeds)
        positions = {k: v for k, v in self._prev_pos.items()}
        shapes = build_shapes(self.cfg, axes, positions, self._headings,
                              self.grow)
        comfort_viz.update(get_current_stage(), shapes, self.buffer_width)

    def update(self, dt: float = 1.0 / 60.0) -> None:
        """Move each collider to its character's current position."""
        if not self.pairs:
            return
        import omni.anim.graph.core as ag

        stage = get_current_stage()
        self._ticks += 1
        report = self._ticks % 300 == 1
        for _key, character_path, collider_path in self.pairs:
            # Read the pose from the animation runtime, not from USD.
            # omni.anim.people moves a character through the animation
            # graph; the prim's xformOp:translate keeps its spawn value
            # forever. Reading USD left the collider parked at the start
            # of the route while the character walked the corridor, and
            # the Comfort Space stayed parked with it.
            character = ag.get_character(character_path)
            if character is None:
                if report:
                    print(f"  [walker] {_key}: animation runtime has no "
                          f"character at {character_path}", flush=True)
                continue

            pos = carb.Float3(0.0, 0.0, 0.0)
            rot = carb.Float4(0.0, 0.0, 0.0, 0.0)
            character.get_world_transform(pos, rot)
            x, y = float(pos[0]), float(pos[1])

            self._track(_key, x, y, dt)

            body = self._bodies.get(_key)
            if body is None:
                body = SingleRigidPrim(prim_path=collider_path)
                try:
                    body.initialize()
                except Exception as exc:
                    # The physics view is not ready on the first step.
                    if report:
                        print(f"  [walker] {_key}: rigid body not ready ({exc})",
                              flush=True)
                    continue
                self._bodies[_key] = body

            # The cylinder is centred on its own height; the character's
            # transform sits at their feet.
            body.set_world_pose(
                position=np.array([x, y, self.height / 2.0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
            if report:
                got = body.get_world_pose()[0]
                print(f"  [walker] {_key}: ({x:+.2f},{y:+.2f}) "
                      f"{self._speeds.get(_key, 0.0):.2f} m/s", flush=True)

        self._redraw()
