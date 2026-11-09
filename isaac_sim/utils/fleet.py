"""The other AMRs in the bay, driving to places on a fixed lane network.

Isaac Sim only: imports omni and pxr.

Not a planner
-------------
These do not plan a path across the floor. They route on a roadmap --
the two aisle centrelines and the two crossings, with a short spur out
to each place -- exactly the way a real fleet controller runs virtual
paths. Letting them find their own way would have them cutting across
pull-off refuges and parked carts, and would also make the traffic the
AMR1 meets different in every run.

AMR1 is NOT one of these. It keeps Nav2 and the
Comfort Space costmap, because that is the thing being measured. The
occupancy map is already restricted to the same lanes, so in practice
both travel the same corridors.

What they give way to
---------------------
People: the scenario's own Comfort Space policy, with the scenario's
own radii. A fleet that drove through people would make the bay look
like nothing a plant would run, and would leave AMR1
as the only machine in the building that respects anybody.

Each other, and AMR1: plain collision avoidance. No
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
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import create_prim, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage
from pxr import Usd, UsdGeom

from comfort_core import resolve_path
from comfort_core.comfort_space import make_policy
from comfort_core.frames import fleet_spawn
from comfort_core.palette import fleet_colour
from comfort_core.roadmap import Roadmap
from comfort_core.layout import placed_entries
from isaac_sim.utils import robot_graph

HUMAN_ROOT = "/Environment/Corridor"
Point = Tuple[float, float]


def _quat(yaw: float):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def _stripes(prim_path: str, name: str) -> None:
    """A coloured stripe down each flank, so a recording says which AMR.

    The three vehicles are the same model; the costmap video labels
    their job lists in comfort_core/palette.py's colours, and this puts
    the same colour on the body. On the flanks, below the LiDAR's
    plane and inside the footprint, so no sensor -- its own or
    another's -- sees anything the bare body would not. Drawing only.

    Placed from the body mesh's bounds IN base_link's frame, measured
    here rather than copied from the asset file: the first attempt
    used the file's world-frame numbers and the stripes floated a hand
    above the deck.
    """
    stage = get_current_stage()
    base = stage.GetPrimAtPath(f"{prim_path}/model/AMR/base_link")
    body = stage.GetPrimAtPath(f"{prim_path}/model/AMR/base_link/base_link_visual")
    if not base.IsValid() or not body.IsValid():
        return
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeRelativeBound(body, base).ComputeAlignedRange()
    lo, hi = box.GetMin(), box.GetMax()
    length = (hi[0] - lo[0]) * 0.6
    half_w = (hi[1] - lo[1]) / 2.0
    cy = (hi[1] + lo[1]) / 2.0
    # The body's bound includes the sensor mast on the deck, so the
    # stripe is kept to the lower part of it, where the side panel is.
    cz = lo[2] + (hi[2] - lo[2]) * 0.38
    height = (hi[2] - lo[2]) * 0.24
    colour = np.array(fleet_colour(name))
    for side, sy in (("l", 1.0), ("r", -1.0)):
        VisualCuboid(
            prim_path=f"{prim_path}/model/AMR/base_link/stripe_{side}",
            translation=np.array([(hi[0] + lo[0]) / 2.0, cy + sy * (half_w + 0.004), cz]),
            scale=np.array([length, 0.006, height]),
            color=colour,
        )


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
        self.why = ""        # why it is not moving, for the report

        # When this vehicle sets off, on the simulation clock.
        #
        # It used to be "immediately", which meant every fleet AMR had
        # been working for the 45 s of Nav2 bringup before the trial
        # started and the traffic AMR1 met depended on
        # how long that bringup happened to take. Holding them until
        # the rendezvous puts every vehicle on the same clock as the
        # trial and the walking people, and `start_at` then means what
        # a scenario says it means: this many seconds after the trial
        # begins.
        hold = (float(cfg["navigation"].get("trial_start_sim_time", 0.0))
                if shared.get("hold_until_trial", False) else 0.0)
        self.waiting = hold + float(spec.get("start_at", 0.0))

        self.policy, self.people = _comfort(cfg)
        self.prim_path = f"{root}/{name}"
        create_prim(self.prim_path, "Xform", position=[self.x, self.y, 0.0],
                    orientation=_quat(self.yaw))
        _reference_amr(cfg, self.prim_path)
        _stripes(self.prim_path, name)
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
        aisle, for the rest of the trial, with AMR1
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
        # aisle for the whole trial, and AMR1 ran into
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

    @property
    def status(self) -> str:
        if not self.why:
            return "moving"
        return f"held by {self.why}" + (f" ({self.holder})" if self.holder else "")


def _reference_amr(cfg: Dict[str, Any], prim_path: str,
                   namespace: str = "") -> List[str]:
    """The same machine the study is about, driven one of two ways.

    With no namespace the vehicle is scripted: its ActionGraph is
    switched off and its bodies are made kinematic. The graph subscribes
    to /cmd_vel, so left on it would drive itself from the topic meant
    for AMR1; and dynamic bodies fight the solver when
    the Xform above them is teleported every tick, while kinematic ones
    simply go where they are put.

    With a namespace the vehicle runs its own Nav2 stack. Neither
    workaround applies then -- robot_graph.namespace_graph moves it onto
    /a1/cmd_vel so there is nothing left to share, and nothing teleports
    it because it drives on its wheels. The asset is left as it is, and
    gets the same two repairs AMR1 gets.
    """
    from pxr import PhysxSchema, UsdPhysics

    model = f"{prim_path}/model"
    create_prim(model, "Xform",
                usd_path=str(resolve_path(cfg["amr"]["usd_path"])))

    if namespace:
        graph_root = f"{model}/AMR_Controller"
        return (robot_graph.namespace_graph(cfg, graph_root, namespace)
                + robot_graph.instance_fixes(cfg, graph_root))

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
    return []


class Nav2FleetAMR:
    """One AMR with its own Nav2 stack, on its own ROS 2 namespace.

    Nothing here drives it. The vehicle is a physical articulation
    taking /a1/cmd_vel from its own controller, exactly as the measured
    robot takes /cmd_vel from its own, and it reports where it ended up
    so the fleet report reads the same whichever driver is in use.

    Sending the job list to Nav2 as goals is the next piece and does
    not live in the simulator: it is a ROS 2 node, so that a real
    vehicle can run the same one.
    """

    def __init__(self, cfg: Dict[str, Any], name: str, spec: Dict[str, Any],
                 shared: Dict[str, Any], roadmap: Roadmap, root: str):
        self.name = name
        self.roadmap = roadmap
        self.jobs: List[str] = [str(j) for j in spec.get("jobs", [])]
        if not self.jobs:
            raise ValueError(f"fleet.{name} has no jobs")
        self.job = 0
        self.legs: List[Point] = []
        self.holder = ""
        self.why = ""
        self.state = "no goal yet"
        # The same call the Nav2 bringup makes for its map -> a1/odom
        # transform, so the two cannot disagree about where odom starts.
        self.x, self.y, self.yaw = fleet_spawn(cfg, name)
        self.prim_path = f"{root}/{name}"
        # The spawn pose sits on the Xform above the articulation, so
        # physics starts the vehicle there instead of at the origin.
        create_prim(self.prim_path, "Xform", position=[self.x, self.y, 0.0],
                    orientation=_quat(self.yaw))
        self.changes = _reference_amr(cfg, self.prim_path, namespace=name)
        _stripes(self.prim_path, name)

    @property
    def position(self) -> Point:
        return (self.x, self.y)

    @property
    def status(self) -> str:
        return self.state

    def update(self, dt: float, others: List[Point]) -> None:
        """Read back where Nav2 has driven it. It is not steered here."""
        pose = _pose_of(f"{self.prim_path}/model/AMR/base_link")
        if pose is not None:
            self.x, self.y = pose


DRIVERS = {"scripted": FleetAMR, "nav2": Nav2FleetAMR}


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

        # "scripted" is what every result so far was measured with, so
        # it stays the default: an existing config runs bit-identically
        # without being touched. "nav2" gives each vehicle its own
        # stack. See docs/NAV2.md section 5c.
        self.driver = str(spec.get("driver", "scripted"))
        if self.driver not in DRIVERS:
            raise ValueError(
                f"fleet.driver {self.driver!r} is not one of "
                f"{sorted(DRIVERS)}")
        make = DRIVERS[self.driver]

        for name, item in sorted(spec.get("amrs", {}).items()):
            if not item.get("enabled", True):
                continue
            member = make(cfg, name, item, spec, roadmap, root)
            for line in getattr(member, "changes", []):
                print(f"  [fleet] {name}: {line}", flush=True)
            self.members.append(member)
        if self.members:
            print(f"  [fleet] driver={self.driver}, "
                  f"{len(self.members)} AMR(s)", flush=True)

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
                      f"  {amr.status}", flush=True)
        # AMR1 counts as traffic to everyone else.
        measured = _pose_of(self.chassis) if self.chassis else None
        for amr in self.members:
            others = [m.position for m in self.members if m is not amr]
            if measured is not None:
                others.append(measured)
            amr.update(dt, others)

    def __len__(self) -> int:
        return len(self.members)
