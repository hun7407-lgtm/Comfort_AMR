"""A parts cart dragged behind the AMR on a drawbar.

Isaac Sim only: imports omni and pxr.

The real platform tows rather than lifts. The Safeway E-MOTION manual
calls the fitting on its back a 견인장치 -- "a hook for pulling wheeled
loads, or for coupling to another robot" -- so a cart here is not
carried on top of the robot, it trails behind it on a bar and cuts the
corner on every turn.

Kinematic, not jointed
----------------------
The cart follows a tractrix: each step its axle is pulled straight
toward the hitch until the bar is taut again. It is not attached with
a physics joint, deliberately.

A joint would be more faithful and would also make every trial depend
on solver behaviour at the hitch -- exactly the kind of thing that
makes two runs of the same scenario differ, which this study cannot
afford. The cart still has a collider and still has visible geometry,
so the LiDAR sees it, the costmap marks it and anything that runs into
it stops. What it does not do is push the robot around.

What this does NOT solve
------------------------
The robot's own LiDAR can see the cart it is towing. Nothing here
hides it. See `docs/` and the note in configs: a tug that marks its own
trailer as an obstacle will refuse to reverse, and the fix belongs in
the perception configuration, not here.
"""

from __future__ import annotations

import math
from typing import Any, Dict

import numpy as np
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import create_prim, is_prim_path_valid

from isaac_sim.environments import machines


def _yaw_of(quat) -> float:
    """Yaw from a (w, x, y, z) quaternion."""
    w, x, y, z = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat(yaw: float):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


class TowedCart:
    """One cart, kept a drawbar's length behind the robot."""

    def __init__(self, cfg: Dict[str, Any]):
        self.enabled = False
        spec = cfg.get("amr", {}).get("tow", {})
        if not spec.get("enabled", False):
            return
        self.chassis = str(cfg["amr"]["chassis_path"])
        if not is_prim_path_valid(self.chassis):
            return

        size = spec.get("cart_size", [1.10, 0.65, 1.00])
        self.length, self.depth, self.height = (float(v) for v in size)
        # Hitch to cart axle. Shorter cuts the corner harder; see the
        # turning study in docs -- below about 0.25 m the rig stops
        # fitting a 1.5 m aisle at any turn radius.
        self.bar = float(spec.get("drawbar", 0.35))
        # Where the hook is, measured back from the robot's own centre.
        self.hook = float(spec.get("hook_offset", 0.45))
        # A real drawbar has a stop in it. Without one the cart folds
        # right back alongside the robot and keeps going -- measured at
        # 162 degrees in a hard turn, which puts a cart where the
        # LiDAR is looking and is not a thing a hitch can do.
        self.max_fold = math.radians(float(spec.get("max_fold_deg", 85.0)))

        self.prim_path = str(spec.get("prim_path", "/World/Cart"))
        robot = SingleXFormPrim(self.chassis)
        pos, quat = robot.get_world_pose()
        yaw = _yaw_of(quat)
        hx = float(pos[0]) - math.cos(yaw) * self.hook
        hy = float(pos[1]) - math.sin(yaw) * self.hook
        # Straight behind, to start.
        self.axle = np.array([hx - math.cos(yaw) * self.bar,
                              hy - math.sin(yaw) * self.bar])
        self.yaw = yaw

        create_prim(self.prim_path, "Xform",
                    position=[float(self.axle[0]), float(self.axle[1]), 0.0],
                    orientation=_quat(yaw - math.pi / 2.0))
        machines.build(cfg, str(spec.get("kind", "cart")), self.prim_path,
                       (self.length, self.depth, self.height))
        self.body = SingleXFormPrim(self.prim_path)
        self.robot = robot
        self._ticks = 0
        self.enabled = True

    def update(self, dt: float = 1.0 / 60.0) -> None:
        """Pull the cart up to the hitch, then put it there."""
        if not self.enabled:
            return
        pos, quat = self.robot.get_world_pose()
        yaw = _yaw_of(quat)
        hx = float(pos[0]) - math.cos(yaw) * self.hook
        hy = float(pos[1]) - math.sin(yaw) * self.hook

        dx, dy = hx - self.axle[0], hy - self.axle[1]
        d = math.hypot(dx, dy)
        if d > 1e-6:
            # The bar is rigid: whatever slack has opened up is taken
            # out along the bar, which is what makes the cart cut the
            # corner instead of tracking the robot's path.
            self.axle = self.axle + (d - self.bar) * np.array([dx, dy]) / d
            self.yaw = math.atan2(hy - self.axle[1], hx - self.axle[0])

        # The stop. Past the limit the cart is pushed round by the
        # robot rather than pulled by it, so the bar ends up at the
        # limit and the axle hangs off the hitch there.
        fold = math.atan2(math.sin(self.yaw - yaw), math.cos(self.yaw - yaw))
        if abs(fold) > self.max_fold:
            self.yaw = yaw + math.copysign(self.max_fold, fold)
            self.axle = np.array([hx - math.cos(self.yaw) * self.bar,
                                  hy - math.sin(self.yaw) * self.bar])

        # Say where it is now and then. A cart that has quietly stayed
        # at the spawn point looks identical to one that is following
        # perfectly, until the video is watched.
        self._ticks += 1
        if self._ticks % 300 == 1:
            lag = math.degrees(math.atan2(math.sin(yaw - self.yaw),
                                          math.cos(yaw - self.yaw)))
            print(f"  [cart] robot ({pos[0]:+.2f}, {pos[1]:+.2f}) "
                  f"cart ({self.axle[0]:+.2f}, {self.axle[1]:+.2f}) "
                  f"bar {math.hypot(hx - self.axle[0], hy - self.axle[1]):.2f} m "
                  f"angle {lag:+.0f} deg", flush=True)

        # The body hangs behind its axle, like a hand truck.
        cx = self.axle[0] - math.cos(self.yaw) * self.length / 2.0
        cy = self.axle[1] - math.sin(self.yaw) * self.length / 2.0
        self.body.set_world_pose(
            position=np.array([cx, cy, 0.0]),
            orientation=_quat(self.yaw - math.pi / 2.0))

    def pose(self):
        """Where the cart is, for anything that has to reason about it."""
        return (float(self.axle[0]), float(self.axle[1]), float(self.yaw))
