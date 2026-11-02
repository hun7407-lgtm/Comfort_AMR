"""The paper scenario: experimental factors in, one resolved trial out.

Pure Python. Importable from BOTH interpreters (see comfort_core
docstring): standard library and PyYAML only.

Why this module exists
----------------------
A comparison of B1, B2, B3-R and B3 is worth nothing unless the four
runs meet the same bay, the same people, the same walks and the same
mission. Up to now a trial's conditions lived in a hand-written config
(configs/factory_u.yaml) or were drawn at run time
(scripts/random_scenario.py). Neither can promise that four policies saw
the same thing, and neither records what the fifth run would have to
reproduce.

So the random draw is pulled out of the run entirely:

    ExperimentFactors        what the experimenter chose
        |                    human_count, amr_count, policy, provider, seed
        v
    ScenarioGenerator        the only place a random number is drawn
        |
        v
    ResolvedScenario         every value fixed, nothing left to chance
        |
        v
    a config file            what Isaac Sim and the ROS 2 nodes read

The policy is NOT an input to the draw. `generate` is handed the
factors and uses only (seed, human_count, amr_count) to seed its
generator, so B1, B2, B3-R and B3 of the same seed are the same
scenario by construction rather than by care. `same_draw_as` asserts it.

The eight people
----------------
Six at the belt and two out on the storage side, named by where they
stand rather than by a serial number:

    N1 N2 N3   north leg of the U, west to east
    S1 S2 S3   south leg
    E1 E2      the storage side, in the gap between two cart blocks

Every one of those positions already exists in the bay. N1..N3 and
S1..S3 are declared working positions off `equipment.stations`, taken
from the layout rather than written down here, and E1/E2 are the only
floor on the storage side that is neither a cart slot, a rack nor a
working position -- which is what makes a person standing there
`stationary` rather than `station_engaged`. E1 is where
configs/factory_u.yaml already puts w11.

The exchange
------------
One per side. A worker leaves the belt and crosses the aisle to the
storage side; once they are clear, their counterpart crosses the other
way and takes the post. Two state transitions per exchange:

    station_engaged -> moving -> stationary        the belt worker
    stationary      -> moving -> station_engaged   the storage worker

and both of them cross the lane the AMR is driving, which is the whole
encounter this study measures.
"""

from __future__ import annotations

import itertools
import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from comfort_core.config import load_config, resolve_path
from comfort_core.layout import work_spots

BASE_CONFIG = "configs/factory_u.yaml"

# The eight, in the order they are reported.
BELT_ROLES = ("N1", "N2", "N3", "S1", "S2", "S3")
EDGE_ROLES = ("E1", "E2")
ROLES = BELT_ROLES + EDGE_ROLES

# Who is in the bay at each density. Low density is not "the first four
# of the eight": it is the four who take part in the two exchanges, so
# the difference between the conditions is the number of BYSTANDERS --
# four extra people standing at their posts -- and not the number of
# encounters. Both conditions have two exchanges and four walkers.
DENSITY = {
    4: ("N2", "S2", "E1", "E2"),
    8: ROLES,
}

Point = Tuple[float, float]


# ---------------------------------------------------------------
# what the experimenter chooses
# ---------------------------------------------------------------
@dataclass(frozen=True)
class ExperimentFactors:
    """The levels of the experiment. None of these is ever randomised."""

    seed: int
    human_count: int = 8          # 4 or 8
    amr_count: int = 1            # 1, 2 or 3
    policy: str = "b3"            # b1, b2, b3r, b3
    provider: str = "scripted"    # scripted (Layer 1) or sensor (Layer 2)

    def validate(self) -> None:
        if self.human_count not in DENSITY:
            raise ValueError(
                f"human_count {self.human_count} is not one of "
                f"{sorted(DENSITY)}")
        if not 1 <= self.amr_count <= 3:
            raise ValueError(f"amr_count {self.amr_count} is not 1, 2 or 3")
        if self.policy not in ("b1", "b2", "b3r", "b3"):
            raise ValueError(f"policy {self.policy!r} is not b1/b2/b3r/b3")
        if self.provider not in ("scripted", "sensor", "ground_truth"):
            raise ValueError(
                f"provider {self.provider!r} is not scripted/sensor/"
                f"ground_truth")

    @property
    def label(self) -> str:
        """Names the generated CONFIG, so the policy is absent.

        One config is run under all four policies -- that is the whole
        design -- so a file name or an id that carried the policy would
        promise four different scenarios where there is one. The
        provider IS in it, because Layer 1 and Layer 2 really do read
        different configs (`humans.state_source`, `perception.source`).
        """
        return (f"h{self.human_count}_a{self.amr_count}_"
                f"{self.provider[:3]}_s{self.seed:04d}")

    @property
    def draw_key(self) -> str:
        """What the people, the walks and the mission may depend on.

        The policy and the state provider are deliberately absent: four
        policies of one seed have to meet the same scenario, so they
        cannot be allowed to influence a single draw. So is amr_count,
        which draws from `fleet_key` instead -- adding a second vehicle
        must not move the people or change the mission, or the robot
        count would never be isolated either.

        A string rather than a tuple because random.Random accepts one
        and hashes it the same way on every interpreter.
        """
        return f"{self.seed}|{self.human_count}"

    @property
    def fleet_key(self) -> str:
        """What the other AMRs' start offsets may depend on."""
        return f"{self.seed}|{self.human_count}|{self.amr_count}|fleet"


# ---------------------------------------------------------------
# what comes out
# ---------------------------------------------------------------
@dataclass(frozen=True)
class Person:
    """One of the eight, where they start and what they are doing."""

    role: str
    position: Point
    state: str
    facing_deg: float


@dataclass(frozen=True)
class Exchange:
    """Two people changing places, and when each of them sets off."""

    side: str                 # "north" or "south"
    belt_role: str
    edge_role: str
    # Seconds after the trial begins, not after the simulation begins.
    belt_start_at: float
    edge_start_at: float
    belt_route: List[Point]
    edge_route: List[Point]
    belt_speed: float
    edge_speed: float
    belt_ends_as: str = "stationary"
    edge_ends_as: str = "station_engaged"

    def walk_seconds(self, route: Sequence[Point], start: Point,
                     speed: float) -> float:
        points = [start] + list(route)
        return sum(math.dist(a, b) for a, b in zip(points, points[1:])) / speed


@dataclass(frozen=True)
class Mission:
    """Where the AMR under test starts and where it has to park."""

    start_slot: str
    goal_slot: str
    start: Point
    goal: Point
    distance: float           # m along the lane network
    cross_side: bool


@dataclass(frozen=True)
class ResolvedScenario:
    """Everything fixed. Nothing here is decided again at run time."""

    factors: ExperimentFactors
    people: Dict[str, Person]
    exchanges: List[Exchange]
    mission: Mission
    # The other AMRs, by fleet id: where each starts, the places it
    # works and how long after the trial begins it sets off. Empty when
    # amr_count is 1.
    fleet_plan: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    @property
    def amr_offsets(self) -> Dict[str, float]:
        return {name: float(plan["start_at"])
                for name, plan in sorted(self.fleet_plan.items())}

    @property
    def scenario_id(self) -> str:
        return f"paper_{self.factors.label}"

    # -- the config a run reads ---------------------------------
    def to_config(self, base: str = BASE_CONFIG) -> Dict[str, Any]:
        """The generated config, as a plain dict ready for YAML.

        It is a CONFIG, not a scenario overlay: it carries the people,
        the mission and the perception settings, which are controlled
        variables rather than the policy. The policy still comes from
        configs/scenario_b*.yaml, so comfort_core.config's guarantee
        that B1..B3 differ in the policy alone is untouched.
        """
        f = self.factors
        humans: Dict[str, Any] = {}

        # Everybody the base config places and this scenario does not
        # use is switched off by name. Without this the twelve workers
        # of configs/factory_u.yaml would still be standing there and
        # `human_count` would mean nothing.
        for key in _base_human_keys(base):
            humans[key] = {"enabled": False}

        walking = {}
        for ex in self.exchanges:
            walking[ex.belt_role] = (ex.belt_start_at, ex.belt_route,
                                     ex.belt_speed, ex.belt_ends_as)
            walking[ex.edge_role] = (ex.edge_start_at, ex.edge_route,
                                     ex.edge_speed, ex.edge_ends_as)

        for role, person in sorted(self.people.items()):
            entry: Dict[str, Any] = {
                "position": [round(person.position[0], 3),
                             round(person.position[1], 3)],
                "state": person.state,
                "facing_deg": round(person.facing_deg, 1),
            }
            if role in walking:
                start_at, route, speed, ends_as = walking[role]
                entry["walk"] = {
                    "start_at": round(start_at, 2),
                    "speed": round(speed, 2),
                    # Read by comfort_core.human_state.ScriptedStateProvider
                    # to know what a person is doing once they stop.
                    "ends_as": ends_as,
                    "route": [[round(x, 3), round(y, 3)] for x, y in route],
                }
            humans[role] = entry

        # A character per person, alternating, so the bay does not look
        # like one person cloned eight times.
        characters = dict(zip(sorted(self.people),
                              itertools.cycle(("F_Business_02",
                                               "M_Medical_01"))))

        cfg: Dict[str, Any] = {
            "base": base,
            "scenario_meta": self.metadata(),
            "humans": dict(humans, character=characters,
                           state_source=("scripted" if f.provider == "scripted"
                                         else f.provider)),
            "perception": {
                "source": "lidar" if f.provider == "sensor" else "tf"},
            "navigation": {
                "start": [round(self.mission.start[0], 3),
                          round(self.mission.start[1], 3)],
                "goal": [round(self.mission.goal[0], 3),
                         round(self.mission.goal[1], 3)],
                "goal_yaw_deg": -90.0 if self.mission.goal[1] > 0 else 90.0,
            },
            "amr": {"spawn": {
                "yaw_deg": -90.0 if self.mission.start[1] > 0 else 90.0}},
            "recording": {"version": "paper"},
        }
        if self.fleet_plan:
            # Every fleet AMR the base config defines is named, so one
            # that this scenario does not use cannot be left enabled by
            # inheritance. `amr_count` has to mean the number of
            # vehicles in the bay.
            amrs = {name: {"enabled": False}
                    for name in _base_fleet_ids(base)}
            for name, plan in self.fleet_plan.items():
                amrs[name] = {
                    "enabled": True,
                    "start_at": round(float(plan["start_at"]), 2),
                    "start": plan["start"],
                    "jobs": list(plan["jobs"]),
                }
            cfg["fleet"] = {"enabled": True, "hold_until_trial": True,
                            "amrs": amrs}
        else:
            cfg["fleet"] = {"enabled": False}
        return cfg

    def metadata(self) -> Dict[str, Any]:
        """What every trial of this scenario records about itself."""
        f = self.factors
        # No `policy`. This block is written into a config that all four
        # policies read; naming one of them here would be a lie in
        # three runs out of four. The policy reaches a trial through
        # --scenario and is recorded by metrics.py from the policy
        # actually loaded.
        return {
            "scenario_id": self.scenario_id,
            "seed": f.seed,
            "human_count": f.human_count,
            "amr_count": f.amr_count,
            "state_provider": f.provider,
            "roles": {role: list(p.position)
                      for role, p in sorted(self.people.items())},
            "exchanges": [{
                "side": ex.side,
                "belt": ex.belt_role,
                "edge": ex.edge_role,
                "belt_start_at": round(ex.belt_start_at, 2),
                "edge_start_at": round(ex.edge_start_at, 2),
                "belt_speed": round(ex.belt_speed, 2),
                "edge_speed": round(ex.edge_speed, 2),
            } for ex in self.exchanges],
            "mission": {
                "start_slot": self.mission.start_slot,
                "goal_slot": self.mission.goal_slot,
                "distance": round(self.mission.distance, 2),
                "cross_side": self.mission.cross_side,
            },
            "amr_offsets": {k: round(v, 2)
                            for k, v in sorted(self.amr_offsets.items())},
            "fleet_jobs": {k: list(v["jobs"])
                           for k, v in sorted(self.fleet_plan.items())},
        }


# ---------------------------------------------------------------
# the bay, read rather than written down
# ---------------------------------------------------------------
def _base_human_keys(base: str) -> List[str]:
    """Every human the base config places, so they can be switched off."""
    cfg = load_config(common_path=base)
    humans = cfg.get("humans", {})
    return sorted(k for k, v in humans.items()
                  if isinstance(v, dict) and "position" in v)


def _base_fleet_ids(base: str) -> List[str]:
    """Every fleet AMR the base config defines, so each can be named."""
    cfg = load_config(common_path=base)
    return sorted(str(k) for k in cfg.get("fleet", {}).get("amrs", {}))


def belt_posts(cfg: Dict[str, Any], north: bool) -> List[Tuple[float, float, float]]:
    """Working positions along one leg of the U, west to east."""
    return sorted(((x, y, f) for x, y, f in work_spots(cfg)
                   if (y > 0) == north and abs(y) < 4.0),
                  key=lambda s: s[0])


def edge_spot(cfg: Dict[str, Any], north: bool) -> Point:
    """Somewhere on the storage side to stand that is nobody's post.

    The gap between two cart blocks, which is the only floor over there
    that is neither a marked slot nor racking. A person standing here
    reads `stationary` and not `station_engaged`, which is what the
    exchange needs at the storage end.
    """
    from comfort_core.factory_layout import build_layout
    layout = build_layout(cfg)
    side = 1.0 if north else -1.0
    blocks = sorted((b for b in layout.cart_zones if (b.y_min > 0) == north),
                    key=lambda b: b.x_min)
    posts = work_spots(cfg)
    for left, right in zip(blocks, blocks[1:]):
        middle = (left.x_max + right.x_min) / 2.0
        y = side * 5.60
        for dx in (0.0, -0.6, 0.6, -0.9, 0.9):
            x = middle + dx
            if not (left.x_max + 0.3 <= x <= right.x_min - 0.3):
                continue
            if all(math.hypot(x - px, y - py) > 0.80 for px, py, _f in posts):
                return (round(x, 2), y)
    raise ValueError(
        f"no spot on the {'north' if north else 'south'} storage side that "
        f"is clear of every working position")


def role_positions(cfg: Dict[str, Any]) -> Dict[str, Person]:
    """The eight, placed from the layout rather than from a constant.

    N1/N2/N3 are the west, centre and east posts of the north leg --
    the ends and the middle of the belt, 6 m apart, so the three of them
    span the whole run rather than crowding one end of it. S1/S2/S3 are
    the same three x on the south leg.
    """
    out: Dict[str, Person] = {}
    for prefix, north in (("N", True), ("S", False)):
        posts = belt_posts(cfg, north)
        if len(posts) < 3:
            raise ValueError(
                f"the {'north' if north else 'south'} leg has {len(posts)} "
                f"working positions; three are needed for "
                f"{prefix}1/{prefix}2/{prefix}3")
        chosen = (posts[0], posts[len(posts) // 2], posts[-1])
        for index, (x, y, facing) in enumerate(chosen, start=1):
            out[f"{prefix}{index}"] = Person(
                f"{prefix}{index}", (x, y), "station_engaged",
                math.degrees(facing))
    for role, north in (("E1", True), ("E2", False)):
        x, y = edge_spot(cfg, north)
        out[role] = Person(role, (x, y), "stationary",
                           -90.0 if north else 90.0)
    return out


# ---------------------------------------------------------------
# the lane network, for walking and for estimating a mission
# ---------------------------------------------------------------
@dataclass(frozen=True)
class Lanes:
    north: float
    south: float
    west: float
    east: float

    def aisle(self, y: float) -> float:
        return self.north if y > 0 else self.south

    def length(self, a: Point, b: Point) -> float:
        """Distance from a to b along the lanes, which is how the AMR goes.

        Not a Nav2 plan: the roadmap estimate, "out to the lane, round,
        and in". The occupancy map is restricted to the painted lanes
        (`factory.map.lanes_only`), so a plan is this plus the width of
        the corners.
        """
        la, lb = self.aisle(a[1]), self.aisle(b[1])
        spur = abs(a[1] - la) + abs(b[1] - lb)
        if la == lb:
            return spur + abs(b[0] - a[0])
        return spur + min(abs(a[0] - c) + abs(la - lb) + abs(b[0] - c)
                          for c in (self.west, self.east))


def lanes_of(cfg: Dict[str, Any]) -> Lanes:
    from comfort_core.factory_layout import build_layout
    bands = {b.name: b for b in build_layout(cfg).bands}
    crossings = [float(x) for x in cfg["factory"]["cross_aisles"]["at"]]
    return Lanes(bands["aisle_north"].centre, bands["aisle_south"].centre,
                 min(crossings), max(crossings))


def exchange_routes(cfg: Dict[str, Any], post: Point, spot: Point,
                    lane_offset: float = 0.45,
                    end_offset: float = 0.60
                    ) -> Tuple[List[Point], List[Point]]:
    """The two walks of one exchange, laid out so they never collide.

    Both people cross the AMR's aisle -- that is the encounter the study
    exists to measure -- but they are kept apart everywhere by geometry
    rather than by timing, which is the only way it can be guaranteed
    for every seed.

    Along the aisle they use two lines `2 * lane_offset` apart: the one
    leaving the belt keeps to the belt side, the one coming from
    storage keeps to the storage side, so they pass side by side
    instead of head on.

    Where each one CROSSES the other's line is the part that has to be
    got right. Each walker makes its crossing `end_offset` BEYOND the
    point it is heading for, on the far side from where it came, and
    then steps back into it. So the belt worker crosses the
    storage-side line past the spot, and the storage worker crosses the
    belt-side line past the post, and neither crossing falls anywhere
    on the leg the other one walks. The only point they share is the
    one they are swapping, and they are there minutes apart.

    This replaces separating them by timing, which cannot be made to
    work: with the two walks keyed off each other's progress, the
    schedule check found the pair passing 0.00-0.08 m apart in all 240
    generated scenarios. Offsetting the ends but crossing on the
    other's line still left 61 of 120 under 0.70 m -- a perpendicular
    crossing, measured at 0.02 m for seed 20 at t = 17.1 s, with one
    walking along y = -4.66 and the other going straight across it.

    Straight legs, no navigation mesh: omni.anim.people walks directly
    between waypoints, and every leg here is along an aisle, across
    one, or a step sideways.
    """
    lanes = lanes_of(cfg)
    aisle = lanes.aisle(post[1])
    side = math.copysign(1.0, aisle)
    out_y = aisle - side * lane_offset    # belt side of the centreline
    back_y = aisle + side * lane_offset   # storage side of it
    # Which way the post lies from the spot. Each walker turns off on
    # the side they are coming from, so neither has to double back.
    toward = 1.0 if post[0] > spot[0] else -1.0
    # Each walker turns off beyond its destination, away from where it
    # started, so that its crossing of the aisle is outside the stretch
    # the other one walks.
    belt_turn = spot[0] - end_offset * toward     # past the spot
    edge_turn = spot[0] + end_offset * toward     # spot side nearer the post
    edge_arrive = post[0] + end_offset * toward   # past the post

    out_route = [
        (post[0], out_y),             # step off the post into the aisle
        (belt_turn, out_y),           # along the aisle, belt side, past the spot
        (belt_turn, spot[1]),         # up into the storage gap
        spot,                         # back along into the spot
    ]
    back_route = [
        (edge_turn, spot[1]),         # out of the spot, the other way
        (edge_turn, back_y),          # down into the aisle
        (edge_arrive, back_y),        # along the aisle, storage side, past the post
        (edge_arrive, post[1]),       # up to the operator band
        post,                         # back along onto the post
    ]
    return (out_route, back_route)


# ---------------------------------------------------------------
# the mission pool
# ---------------------------------------------------------------
def mission_pool(cfg: Dict[str, Any], people: Dict[str, Person],
                 min_distance: float = 12.0,
                 standoff: float = 1.80,
                 mouth_offset: float = 0.165) -> List[Mission]:
    """Every pair of empty cart slots this scenario could run between.

    A mission is a pair of EMPTY cart slots, because that is where an
    AMR that works the staging blocks actually waits between jobs, and
    because the layout already keeps them clear of everything else.

    Three filters, and none of them is a preference:

    `mouth_offset`  the pose is 0.165 m nearer the slot's mouth than
        its centre. The stopped protective field reaches 0.564 m from
        the robot's centre and a slot centre is 0.525 m from the loaded
        cart in the row behind, so a robot parked in the middle of a
        slot has that cart inside its own field and the collision
        monitor refuses every command -- including the rotation that
        would square it up. Fifteen of sixteen trials that drove the
        whole route once ended 0.25-0.29 m short of a goal they needed
        to be 0.25 m from. See configs/factory_u.yaml navigation.

    `standoff`  no end of a mission within this of anywhere one of the
        eight stands at any point in the trial. 1.80 m clears the
        largest Comfort Space in the study (1.20 m) plus the robot's
        inscribed radius (0.24 m) with 0.36 m over. Without it the two
        slots either side of E1 are 1.62 m away, which is inside a B2
        Comfort Space: the robot could not park, and "could not park"
        would be counted as "could not pass".

    `min_distance`  a trial has to be a journey. Not a fixed number --
        print the pool and choose.
    """
    from comfort_core.factory_layout import build_layout
    layout = build_layout(cfg)
    lanes = lanes_of(cfg)

    # Everywhere a person stands at any time: where they start, and
    # where their counterpart's post is, because an exchange leaves
    # somebody at both ends.
    occupied = [p.position for p in people.values()]

    slots: Dict[str, Point] = {}
    for slot in layout.cart_slots_free:
        x, y = slot.centre
        pose = (x, y - math.copysign(mouth_offset, y))
        if min(math.hypot(pose[0] - px, pose[1] - py)
               for px, py in occupied) < standoff:
            continue
        slots[slot.name] = pose

    out: List[Mission] = []
    for a, b in itertools.permutations(sorted(slots), 2):
        start, goal = slots[a], slots[b]
        distance = lanes.length(start, goal)
        if distance < min_distance:
            continue
        out.append(Mission(a, b, start, goal, distance,
                           (start[1] > 0) != (goal[1] > 0)))
    out.sort(key=lambda m: (-m.distance, m.start_slot, m.goal_slot))
    return out


# ---------------------------------------------------------------
# the generator
# ---------------------------------------------------------------
class ScenarioGenerator:
    """The only place in the experiment a random number is drawn.

    One generator per (seed, human_count, amr_count). The draws happen
    in a fixed order, so adding a factor later cannot silently change
    what an old seed produces; and the policy is not among the inputs,
    so the four policies of one seed get the same scenario.
    """

    def __init__(self, cfg: Dict[str, Any], base: str = BASE_CONFIG):
        self.cfg = cfg
        self.base = base
        self.roles = role_positions(cfg)
        self.lanes = lanes_of(cfg)
        spec = cfg.get("scenario_pool", {})
        self.min_distance = float(spec.get("min_mission_distance", 12.0))
        self.standoff = float(spec.get("mission_standoff", 1.80))
        self.mouth = float(spec.get("slot_mouth_offset", 0.165))
        # Half the distance between the two walking lines. 0.45 m puts
        # them 0.90 m apart, which is a side-by-side pass rather than a
        # head-on one, and still leaves each of them 0.30 m inside the
        # 1.50 m aisle.
        self.lane_offset = float(spec.get("walk_lane_offset", 0.45))
        walk = spec.get("walk_speed", {})
        self.speed_range = (float(walk.get("min", 0.8)),
                            float(walk.get("max", 1.2)))
        # How far to one side each walker approaches the point they are
        # swapping into. 0.60 m keeps a 0.25 m-radius person 0.05 m
        # clear of the cart blocks either side of the 1.80 m storage
        # gap, which is as far as it can go.
        self.end_offset = float(spec.get("walk_end_offset", 0.60))
        swap = spec.get("swap", {})
        self.swap_nominal = float(swap.get("nominal_start", 8.0))
        self.swap_jitter = float(swap.get("jitter", 3.0))
        self.swap_side_offset = float(swap.get("side_offset", 4.0))
        self.handover = float(swap.get("handover_delay", 1.5))
        offs = spec.get("amr_offset", {})
        self.amr_offset_range = (float(offs.get("min", 0.0)),
                                 float(offs.get("max", 6.0)))
        self.fleet_ids = [str(k) for k in
                          sorted(cfg.get("fleet", {}).get("amrs", {}))]

    # -- the draw ---------------------------------------------------
    def generate(self, factors: ExperimentFactors) -> ResolvedScenario:
        factors.validate()
        rng = random.Random(factors.draw_key)
        notes: List[str] = []

        present = DENSITY[factors.human_count]
        people = {role: self.roles[role] for role in present}

        # A. which worker takes part in each exchange. Drawn either
        #    way, and then overridden at low density, so that the draw
        #    order -- and so every value after it -- does not depend on
        #    the density.
        north_role = rng.choice(["N1", "N2", "N3"])
        south_role = rng.choice(["S1", "S2", "S3"])
        if factors.human_count == 4:
            north_role, south_role = "N2", "S2"
            notes.append("low density: the exchange pair is fixed at N2/S2")

        # B. how fast each of the four walkers walks.
        lo, hi = self.speed_range
        speeds = {role: round(rng.uniform(lo, hi), 2)
                  for role in (north_role, "E1", south_role, "E2")}

        # C. when each exchange begins. Independent of where the AMR
        #    is: a worker's task schedule is not a function of the
        #    traffic. The two sides get their own times so the robot
        #    does not always meet both at once.
        north_at = max(0.0, round(self.swap_nominal
                                  + rng.uniform(-self.swap_jitter,
                                                self.swap_jitter), 1))
        south_at = max(0.0, round(self.swap_nominal + self.swap_side_offset
                                  + rng.uniform(-self.swap_jitter,
                                                self.swap_jitter), 1))

        exchanges = [
            self._exchange("north", north_role, "E1", north_at, speeds),
            self._exchange("south", south_role, "E2", south_at, speeds),
        ]

        # D. the mission.
        pool = mission_pool(self.cfg, people, self.min_distance,
                            self.standoff, self.mouth)
        if not pool:
            raise ValueError(
                f"no pair of empty cart slots is {self.min_distance} m apart "
                f"and {self.standoff} m clear of all "
                f"{factors.human_count} people")
        mission = rng.choice(pool)
        notes.append(f"mission drawn from {len(pool)} candidate pairs")

        # E. the other AMRs, and how long after the trial starts each
        #    one sets off. Only when there is more than one.
        plan: Dict[str, Dict[str, Any]] = {}
        extra = factors.amr_count - 1
        if extra > 0:
            if len(self.fleet_ids) < extra:
                raise ValueError(
                    f"amr_count {factors.amr_count} needs {extra} fleet "
                    f"AMR(s) and configs define {len(self.fleet_ids)}")
            olo, ohi = self.amr_offset_range
            # Its own stream: see ExperimentFactors.fleet_key.
            frng = random.Random(factors.fleet_key)
            for name in self.fleet_ids[:extra]:
                jobs = self._jobs_clear_of(name, mission)
                if not jobs:
                    raise ValueError(
                        f"every place fleet.{name} works is within "
                        f"{self.standoff} m of this mission")
                plan[name] = {"start_at": round(frng.uniform(olo, ohi), 1),
                              "start": self._start_clear_of(name, mission),
                              "jobs": jobs}
            notes.append(f"{extra} fleet AMR(s) routed clear of the mission")

        return ResolvedScenario(factors, people, exchanges, mission,
                                plan, notes)

    # -- keeping the traffic off the measured robot's own slots ------
    def _places(self) -> Dict[str, Point]:
        spec = self.cfg.get("fleet", {}).get("places", {})
        return {str(k): (float(v[0]), float(v[1])) for k, v in spec.items()}

    def _clear(self, place: Point, mission: Mission) -> bool:
        """True if a fleet AMR may use this place on this mission.

        A fleet AMR parked in the slot the measured robot is driving to
        deadlocks the trial for a reason that has nothing to do with
        anybody's comfort. configs/factory_u.yaml already leaves one
        slot out by hand for exactly this; now that the mission moves
        with the seed, the exclusion has to move with it.
        """
        return all(math.dist(place, end) >= self.standoff
                   for end in (mission.start, mission.goal))

    def _jobs_clear_of(self, name: str, mission: Mission) -> List[str]:
        spec = self.cfg["fleet"]["amrs"][name]
        places = self._places()
        kept: List[str] = []
        for job in (str(v) for v in spec.get("jobs", [])):
            if job not in places or not self._clear(places[job], mission):
                continue
            # Dropping a place can leave the same one twice in a row,
            # which asks the vehicle to drive to where it already is.
            if kept and kept[-1] == job:
                continue
            kept.append(job)
        return kept

    def _start_clear_of(self, name: str, mission: Mission) -> str:
        spec = self.cfg["fleet"]["amrs"][name]
        places = self._places()
        start = str(spec.get("start", ""))
        if start in places and self._clear(places[start], mission):
            return start
        # Its usual start is the measured robot's slot this time, so
        # start it at the first job it is still allowed to visit.
        for job in self._jobs_clear_of(name, mission):
            return job
        raise ValueError(f"fleet.{name} has nowhere to start on this mission")

    def _exchange(self, side: str, belt_role: str, edge_role: str,
                  start_at: float, speeds: Dict[str, float]) -> Exchange:
        post = self.roles[belt_role].position
        spot = self.roles[edge_role].position
        out_route, back_route = exchange_routes(self.cfg, post, spot,
                                                self.lane_offset,
                                                self.end_offset)
        belt_speed = speeds[belt_role]
        edge_speed = speeds[edge_role]

        # The storage worker sets off once the belt worker has LEFT THE
        # POST, which is computed from the walk rather than guessed: the
        # time to cover the first leg -- out of the working position
        # and into the aisle -- at that person's own speed, plus a short
        # handover pause. So the second departure follows the first
        # person's real progress, not a timer picked by hand.
        #
        # It used to wait until the belt worker had walked the length of
        # the aisle and reached the storage spot. That spot is where the
        # storage worker is STANDING, so the two ended up in the same
        # square metre: the schedule check measured them passing 0.00 to
        # 0.08 m apart in every one of 240 generated scenarios. Letting
        # the second set off early means they pass each other in the
        # aisle instead, on the two offset lines, which is both further
        # apart and what people actually do.
        clear_after = math.dist(post, out_route[0]) / belt_speed
        return Exchange(side, belt_role, edge_role,
                        start_at, round(start_at + clear_after
                                        + self.handover, 2),
                        out_route, back_route, belt_speed, edge_speed)


def generate(factors: ExperimentFactors, base: str = BASE_CONFIG
             ) -> ResolvedScenario:
    """One scenario, from the factors and nothing else."""
    cfg = load_config(common_path=base)
    return ScenarioGenerator(cfg, base).generate(factors)


def same_draw_as(a: ResolvedScenario, b: ResolvedScenario) -> List[str]:
    """Every way two scenarios differ other than in policy or provider.

    Used by the checks: four policies of one seed must come out
    identical here, or the comparison is not a comparison.
    """
    problems = []
    if a.people != b.people:
        problems.append("the people differ")
    if a.exchanges != b.exchanges:
        problems.append("the exchanges differ")
    if a.mission != b.mission:
        problems.append("the mission differs")
    if a.fleet_plan != b.fleet_plan:
        problems.append("the other AMRs' plans differ")
    return problems


# ---------------------------------------------------------------
# validation: everything that can be checked without running anything
# ---------------------------------------------------------------
def _segments(start: Point, route: Sequence[Point]) -> List[Tuple[Point, Point]]:
    points = [start] + [tuple(p) for p in route]
    return list(zip(points, points[1:]))


def _crosses_box(a: Point, b: Point, box: Tuple[float, float, float, float],
                 samples: int = 40) -> bool:
    """Whether the straight leg a->b passes through an axis-aligned box.

    Sampled rather than solved. A walk leg here is a few metres and the
    boxes are machines and parked carts, so 40 samples resolve 0.1 m --
    finer than anything that matters, and it cannot be got wrong the way
    a hand-written slab intersection can.
    """
    x0, x1, y0, y1 = box
    for i in range(samples + 1):
        t = i / samples
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False


def validate_scenario(resolved: ResolvedScenario, cfg: Dict[str, Any]
                      ) -> List[str]:
    """Every reason this scenario is not usable. Empty means it is.

    `cfg` is the LOADED generated config, not the base: the point is to
    check what a run will actually read.
    """
    from comfort_core.factory_layout import build_layout, validate_factory
    from comfort_core.human_state import SensorStateProvider
    from comfort_core.layout import (human_positions, human_states,
                                     obstacle_boxes, plannable_area)

    problems: List[str] = list(validate_factory(cfg, build_layout(cfg)))

    positions = {k: (x, y) for k, x, y in human_positions(cfg)}
    declared = human_states(cfg)

    # 1. the right people, and only them.
    expected = set(resolved.people)
    if set(positions) != expected:
        problems.append(
            f"the config places {sorted(positions)} and the scenario asks "
            f"for {sorted(expected)}")

    # 2. the map-based estimator has to agree with the declared state at
    #    t = 0, or the scene contradicts itself before anybody moves and
    #    the state-estimation figures are meaningless.
    estimator = SensorStateProvider(cfg)
    read = estimator.update(positions, {k: 0.0 for k in positions}, 0.0)
    for key, state in sorted(declared.items()):
        if read.get(key) != state:
            problems.append(f"humans.{key} is declared {state} but the map "
                            f"reads {read.get(key)}")

    # 3. nobody standing on either end of the mission.
    standoff = float(cfg.get("scenario_pool", {})
                     .get("mission_standoff", 1.80))
    for label, point in (("start", resolved.mission.start),
                         ("goal", resolved.mission.goal)):
        for key, (hx, hy) in sorted(positions.items()):
            gap = math.hypot(hx - point[0], hy - point[1])
            if gap < standoff:
                problems.append(f"humans.{key} is {gap:.2f} m from "
                                f"navigation.{label}, inside the "
                                f"{standoff:.2f} m standoff")

    # 4. nobody walks through a machine, a column or a parked cart.
    boxes = obstacle_boxes(cfg)
    for ex in resolved.exchanges:
        for role, route in ((ex.belt_role, ex.belt_route),
                            (ex.edge_role, ex.edge_route)):
            start = resolved.people[role].position
            for index, (a, b) in enumerate(_segments(start, route), start=1):
                for box in boxes:
                    if _crosses_box(a, b, box):
                        problems.append(
                            f"{role} leg {index} from ({a[0]:+.2f},{a[1]:+.2f})"
                            f" to ({b[0]:+.2f},{b[1]:+.2f}) passes through "
                            f"the box x {box[0]:+.2f}..{box[1]:+.2f} "
                            f"y {box[2]:+.2f}..{box[3]:+.2f}")
                        break

    # 5. every waypoint inside the building, and the walk has to cross
    #    the AMR's own aisle.
    #
    #    NOT "inside the navigable floor": people stand where the robot
    #    may not go -- at a machine, in the gap between two cart blocks
    #    -- and the first version of this check rejected all forty
    #    scenarios for exactly that reason. What matters is that a
    #    waypoint is in the bay and not inside something solid, which
    #    check 4 already covers for the whole leg, and that the walk
    #    really does cross the lane the robot drives. A worker exchange
    #    that stayed on the storage side would be an exchange the robot
    #    never meets.
    from comfort_core.layout import scene_bounds
    bounds = scene_bounds(cfg)
    lanes = lanes_of(cfg)
    for ex in resolved.exchanges:
        for role, route in ((ex.belt_role, ex.belt_route),
                            (ex.edge_role, ex.edge_route)):
            for x, y in route:
                if not (bounds.x_min <= x <= bounds.x_max
                        and bounds.y_min <= y <= bounds.y_max):
                    problems.append(
                        f"{role} waypoint ({x:+.2f},{y:+.2f}) is outside the "
                        f"building")
            start = resolved.people[role].position
            ys = [start[1]] + [float(p[1]) for p in route]
            aisle = lanes.aisle(start[1])
            if not (min(ys) <= aisle <= max(ys)):
                problems.append(
                    f"{role} never crosses the aisle at y {aisle:+.2f}; the "
                    f"robot would not meet them")

    # 6. the two halves of an exchange must not be in the same place at
    #    the same time. Checked on the schedule, in 0.1 s steps.
    for ex in resolved.exchanges:
        gap = _closest_approach(resolved, ex)
        if gap is not None and gap < 0.70:
            problems.append(
                f"the {ex.side} exchange has {ex.belt_role} and "
                f"{ex.edge_role} passing {gap:.2f} m apart; they would walk "
                f"through each other")
    return problems


def _closest_approach(resolved: ResolvedScenario, ex: Exchange,
                      step: float = 0.1) -> Optional[float]:
    """How close the two halves of one exchange ever come, in metres.

    Walked rather than reasoned about: each person is advanced along
    their own route at their own speed from their own start time, and
    the distance is sampled. It is the schedule's own answer to "do
    these two meet head on".
    """
    belt_start = resolved.people[ex.belt_role].position
    edge_start = resolved.people[ex.edge_role].position
    belt = _walk(belt_start, ex.belt_route, ex.belt_speed, ex.belt_start_at)
    edge = _walk(edge_start, ex.edge_route, ex.edge_speed, ex.edge_start_at)
    span = max(belt[-1][0], edge[-1][0]) + 2.0
    worst = None
    t = 0.0
    while t <= span:
        a = _at(belt, t)
        b = _at(edge, t)
        gap = math.dist(a, b)
        if worst is None or gap < worst:
            worst = gap
        t += step
    return worst


def _walk(start: Point, route: Sequence[Point], speed: float,
          start_at: float) -> List[Tuple[float, Point]]:
    """(time, point) at every waypoint, given a speed and a start time."""
    out = [(start_at, start)]
    here = start
    clock = start_at
    for point in route:
        point = (float(point[0]), float(point[1]))
        clock += math.dist(here, point) / max(speed, 1e-6)
        out.append((clock, point))
        here = point
    return out


def _at(walk: Sequence[Tuple[float, Point]], t: float) -> Point:
    """Where somebody is at time t, interpolating between waypoints."""
    if t <= walk[0][0]:
        return walk[0][1]
    for (t0, p0), (t1, p1) in zip(walk, walk[1:]):
        if t <= t1:
            f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return (p0[0] + (p1[0] - p0[0]) * f,
                    p0[1] + (p1[1] - p0[1]) * f)
    return walk[-1][1]


def walk_timeline(resolved: ResolvedScenario) -> List[Tuple[str, float, float, float]]:
    """(role, departs, arrives, metres) for each of the four walkers."""
    out = []
    for ex in resolved.exchanges:
        for role, route, speed, start_at in (
                (ex.belt_role, ex.belt_route, ex.belt_speed, ex.belt_start_at),
                (ex.edge_role, ex.edge_route, ex.edge_speed, ex.edge_start_at)):
            walk = _walk(resolved.people[role].position, route, speed, start_at)
            metres = sum(math.dist(a[1], b[1]) for a, b in zip(walk, walk[1:]))
            out.append((role, start_at, walk[-1][0], metres))
    return out
