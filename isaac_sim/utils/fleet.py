"""The other AMRs in the bay, driving to places on a fixed lane network.

Isaac Sim only: imports omni and pxr.

Not a planner
-------------
These do not plan a path across the floor. They route on a roadmap --
the two aisle centrelines and the two crossings, with a short spur out
to each place -- exactly the way a real fleet controller runs virtual
paths. Letting them find their own way would have them cutting across
pull-off refuges and parked carts, and would also make the traffic the
robot under test meets different in every run.

The robot under test is NOT one of these. It keeps Nav2 and the
Comfort Space costmap, because that is the thing being measured. The
occupancy map is already restricted to the same lanes, so in practice
both travel the same corridors.

What they give way to
---------------------
People: the scenario's own Comfort Space policy, with the scenario's
own radii. A fleet that drove through people would make the bay look
like nothing a plant would run, and would leave the robot under test
as the only machine in the building that respects anybody.

Each other, and the robot under test: plain collision avoidance. No
Comfort Space between machines -- a robot is not owed personal space.
Whoever is in front wins; the one behind waits.

They are kinematic. They carry visible geometry so the LiDAR sees
them, but contact does not shove them about, which would make two runs
of the same scenario differ.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import create_prim, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage
from pxr import Usd

from comfort_core import resolve_path
from comfort_core.comfort_space import make_policy
from comfort_core.layout import placed_entries

HUMAN_ROOT = "/Environment/Corridor"
Point = Tuple[float, float]


def _quat(yaw: float):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


class Roadmap:
    """The lanes, and how to get from anywhere on them to a place.

    The network is a rectangle: the two aisle centrelines joined by the
    two crossings. Anything off it -- a delivery point in a refuge, a
    cart block -- hangs off a spur at right angles, so a trip is always
    "back to the lane, round, then in".
    """

    def __init__(self, spec: Dict[str, Any]):
        lanes = spec.get("lanes", {})
        self.north = float(lanes["north_y"])
        self.south = float(lanes["south_y"])
        self.west = float(lanes["west_x"])
        self.east = float(lanes["east_x"])
        # The loop, clockwise from the north-west corner. Distance
        # round it is the only coordinate the router needs.
        self.corners: List[Point] = [
            (self.west, self.north), (self.east, self.north),
            (self.east, self.south), (self.west, self.south)]
        self.edges = [math.dist(self.corners[i],
                                self.corners[(i + 1) % 4]) for i in range(4)]
        self.perimeter = sum(self.edges)

        self.places: Dict[str, Point] = {
            name: (float(p[0]), float(p[1]))
            for name, p in spec.get("places", {}).items()}

    def access(self, place: Point) -> Point:
        """Where the spur to a place leaves the lane."""
        x, y = place
        if y > self.north:
            return (x, self.north)
        if y < self.south:
            return (x, self.south)
        return place

    def _distance_round(self, point: Point) -> float:
        """How far round the loop a point on it lies."""
        best, best_s = float("inf"), 0.0
        s = 0.0
        for i in range(4):
            a, b = self.corners[i], self.corners[(i + 1) % 4]
            seg = self.edges[i]
            # Project onto this edge.
            dx, dy = b[0] - a[0], b[1] - a[1]
            t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (seg * seg)
            t = max(0.0, min(1.0, t))
            px, py = a[0] + dx * t, a[1] + dy * t
            d = math.hypot(point[0] - px, point[1] - py)
            if d < best:
                best, best_s = d, s + seg * t
            s += seg
        return best_s

    def route(self, start: Point, target_name: str) -> List[Point]:
        """Waypoints from where the AMR is to the named place.

        Off the spur, round the loop the short way, then in. The only
        thing to work out is which corners lie between the two access
        points in the chosen direction.
        """
        place = self.places[target_name]
        here = self.access(start)
        there = self.access(place)

        out: List[Point] = []
        if here != start:
            out.append(here)

        s0 = self._distance_round(here)
        s1 = self._distance_round(there)
        forward = (s1 - s0) % self.perimeter
        backward = self.perimeter - forward
        if forward <= backward:
            ahead = lambda c: (self._distance_round(c) - s0) % self.perimeter
            span = forward
        else:
            ahead = lambda c: (s0 - self._distance_round(c)) % self.perimeter
            span = backward
        between = [c for c in self.corners if 1e-6 < ahead(c) < span - 1e-6]
        out += sorted(between, key=ahead)

        out.append(there)
        if there != place:
            out.append(place)
        return out


def _comfort(cfg: Dict[str, Any]):
    """The scenario's own policy, and who it applies to."""
    spec = cfg.get("comfort_space", {})
    if not spec.get("enabled", True) or not spec.get("policy"):
        return None, []
    try:
        policy = make_policy(cfg)
    except Exception:  # noqa: BLE001 - a scenario without a policy is fine
        return None, []
    humans = cfg.get("humans", {})
    return policy, [(k, str(humans[k].get("state", "stationary")))
                    for k in placed_entries(humans)]


def _pose_of(path: str) -> Optional[Point]:
    if not is_prim_path_valid(path):
        return None
    pos, _q = SingleXFormPrim(path).get_world_pose()
    return float(pos[0]), float(pos[1])


class FleetAMR:
    """One AMR working its way down a list of places."""

    def __init__(self, cfg: Dict[str, Any], name: str, spec: Dict[str, Any],
                 shared: Dict[str, Any], roadmap: Roadmap, root: str):
        self.name = name
        self.roadmap = roadmap
        self.speed = float(spec.get("speed", shared.get("speed", 0.35)))
        self.turn = math.radians(float(
            spec.get("turn_speed_deg", shared.get("turn_speed_deg", 60.0))))
        self.pause = float(spec.get("pause", shared.get("pause", 2.0)))
        self.look_ahead = float(shared.get("look_ahead", 0.9))
        self.clearance = float(shared.get("robot_clearance", 1.2))
        self.length = float(cfg["amr"]["footprint_length"])
        self.width = float(cfg["amr"]["footprint_width"])

        self.jobs: List[str] = [str(j) for j in spec.get("jobs", [])]
        if not self.jobs:
            raise ValueError(f"fleet.{name} has no jobs")
        self.job = 0
        self.x, self.y = roadmap.places[str(spec["start"])]
        self.yaw = 0.0
        self.legs: List[Point] = []
        self.holder = ""
        self.waiting = 0.0
        self.why = ""        # why it is not moving, for the report

        self.policy, self.people = _comfort(cfg)
        self.prim_path = f"{root}/{name}"
        create_prim(self.prim_path, "Xform", position=[self.x, self.y, 0.0],
                    orientation=_quat(self.yaw))
        _reference_amr(cfg, self.prim_path)
        self.body = SingleXFormPrim(self.prim_path)
        self._plan()

    # -- routing ----------------------------------------------------
    def _plan(self) -> None:
        self.legs = self.roadmap.route((self.x, self.y), self.jobs[self.job])

    def _next_job(self) -> None:
        self.job = (self.job + 1) % len(self.jobs)
        self.waiting = self.pause
        self._plan()

    # -- giving way -------------------------------------------------
    def _nose(self) -> Point:
        """The point ahead that has to be clear to go on.

        Never further than the leg the vehicle is actually on. Looking
        past the next turn means braking for somebody who is not on the
        route at all: a3 came out of a slot 0.71 m short of the lane,
        its 0.90 m look-ahead reached across the lane to a worker on
        the far side, and it stopped there -- in the middle of the
        aisle, for the rest of the trial, with the robot under test
        behind it. What it was about to do was turn and drive away from
        her.
        """
        reach = self.look_ahead
        if self.legs:
            tx, ty = self.legs[0]
            reach = min(reach, math.hypot(tx - self.x, ty - self.y) + 0.10)
        reach = max(reach, self.length / 2.0)
        return (self.x + math.cos(self.yaw) * reach,
                self.y + math.sin(self.yaw) * reach)

    def _person_in_the_way(self) -> bool:
        """Sets self.holder to whoever it is, for the report.

        "held by person" on its own is not a diagnosis: three AMRs
        standing still for a whole trial look the same whether one
        walker is in the way of all of them or the rule is wrong. The
        name and the distance say which.
        """
        self.holder = ""
        if self.policy is None:
            return False
        nx, ny = self._nose()
        for key, state in self.people:
            pose = _pose_of(f"{HUMAN_ROOT}/human_{key}")
            if pose is None:
                continue
            gap = math.hypot(pose[0] - nx, pose[1] - ny)
            need = self.policy.radius_for(key, state, 0.0) + self.width / 2.0
            if gap < need:
                self.holder = f"{key} at {gap:.2f} m, needs {need:.2f}"
                return True
        return False

    def _machine_in_the_way(self, others: List[Point]) -> bool:
        """Plain clearance, no Comfort Space: a robot is not owed room."""
        nx, ny = self._nose()
        for ox, oy in others:
            if math.hypot(ox - nx, oy - ny) < self.clearance:
                # Only what is actually in front, or two AMRs passing
                # in opposite lanes would stop each other.
                if ((ox - self.x) * math.cos(self.yaw)
                        + (oy - self.y) * math.sin(self.yaw)) > 0.0:
                    return True
        return False

    # -- driving ----------------------------------------------------
    def update(self, dt: float, others: List[Point]) -> None:
        if self.waiting > 0.0:
            self.waiting = max(0.0, self.waiting - dt)
            self.why = "pausing"
            return
        if not self.legs:
            self._next_job()
            return
        # Held or not, the vehicle may still turn on the spot. Turning
        # does not take it closer to anybody -- it is a 0.70 x 0.48 m
        # body rotating about its own centre -- and refusing to turn
        # makes the hold permanent in exactly the case where turning is
        # the way out of it.
        #
        # Measured: a3 came out of a slot pointing south, its nose
        # landed 1.14 m from w5, which is that person's Comfort Space
        # to the centimetre, and it was held. The next thing it wanted
        # to do was turn west, away from w5, which would have cleared
        # the hold at once. Instead it stood in the middle of the north
        # aisle for the whole trial, and the robot under test ran into
        # the back of it and gave up.
        held = ""
        if self._person_in_the_way():
            held = "person"
        elif self._machine_in_the_way(others):
            held = "machine"
        self.why = held

        tx, ty = self.legs[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.10:
            if held:
                return
            self.legs.pop(0)
            if not self.legs:
                self._next_job()
            return
        want = math.atan2(dy, dx)
        err = math.atan2(math.sin(want - self.yaw), math.cos(want - self.yaw))
        if abs(err) > math.radians(5.0):
            self.yaw += math.copysign(min(self.turn * dt, abs(err)), err)
        elif not held:
            self.yaw = want
            step = min(self.speed * dt, dist)
            self.x += math.cos(self.yaw) * step
            self.y += math.sin(self.yaw) * step
        self.body.set_world_pose(position=np.array([self.x, self.y, 0.0]),
                                 orientation=_quat(self.yaw))

    @property
    def position(self) -> Point:
        return (self.x, self.y)


def _reference_amr(cfg: Dict[str, Any], prim_path: str) -> None:
    """The same machine the study is about, minus two things.

    Its ActionGraph subscribes to /cmd_vel, so every fleet AMR would
    drive itself from the topic meant for the robot under test. And its
    bodies are dynamic, so teleporting the Xform they hang under fights
    the solver; made kinematic they simply go where they are put.
    """
    from pxr import PhysxSchema, UsdPhysics

    model = f"{prim_path}/model"
    create_prim(model, "Xform",
                usd_path=str(resolve_path(cfg["amr"]["usd_path"])))
    stage = get_current_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(model)):
        if prim.GetName() == "AMR_Controller":
            prim.SetActive(False)
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            api = UsdPhysics.RigidBodyAPI(prim)
            attr = api.GetKinematicEnabledAttr() or api.CreateKinematicEnabledAttr()
            attr.Set(True)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)


class Fleet:
    """Every AMR in the bay that is not the one under test."""

    def __init__(self, cfg: Dict[str, Any]):
        self.members: List[FleetAMR] = []
        self._ticks = 0
        self.chassis = str(cfg.get("amr", {}).get("chassis_path", ""))
        spec = cfg.get("fleet", {})
        if not spec.get("enabled", False):
            return
        roadmap = Roadmap(spec)
        root = "/World/Fleet"
        create_prim(root, "Xform")
        for name, item in sorted(spec.get("amrs", {}).items()):
            if not item.get("enabled", True):
                continue
            self.members.append(
                FleetAMR(cfg, name, item, spec, roadmap, root))

    def update(self, dt: float) -> None:
        if not self.members:
            return
        self._ticks += 1
        if self._ticks % 300 == 1:
            # An AMR that has quietly stopped looks the same as one
            # that is between jobs. Say which, and why.
            for amr in self.members:
                target = amr.jobs[amr.job]
                print(f"  [fleet] {amr.name} ({amr.x:+6.2f},{amr.y:+6.2f}) "
                      f"-> {target:9s} {len(amr.legs)} leg(s) left"
                      f"{'  held by ' + amr.why if amr.why else '  moving'}"
                      f"{' (' + amr.holder + ')' if amr.holder else ''}",
                      flush=True)
        # The robot under test counts as traffic to everyone else.
        measured = _pose_of(self.chassis) if self.chassis else None
        for amr in self.members:
            others = [m.position for m in self.members if m is not amr]
            if measured is not None:
                others.append(measured)
            amr.update(dt, others)

    def __len__(self) -> int:
        return len(self.members)
