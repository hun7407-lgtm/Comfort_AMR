"""Comfort Space: how big a circle each human gets.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

This module is the single thing B2 and B3 differ in. Everything else in
the experiment is shared, so if a difference in outcome cannot be traced
to a difference in the radii this returns, something has gone wrong.

Only the circular model is implemented. Gaussian and egg-shaped models
are out of scope; see docs/EXPERIMENT.md section 8.

    C_i = { p : ||p - p_i|| <= R_i }

> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Any, Dict, List, NamedTuple, Optional, Tuple

from comfort_core.layout import human_positions
from comfort_core.shapes import ComfortShape


class Extent(NamedTuple):
    """How far a Comfort Space reaches from the person, by direction.

    Measured in the person's own frame: `front` along their heading,
    `rear` against it, `side` to either side. A round space has all
    three equal.
    """
    front: float
    side: float
    rear: float

    @property
    def axes(self) -> Tuple[float, float, float]:
        """(along, across, offset) for a ComfortShape."""
        return ((self.front + self.rear) / 2.0, self.side,
                (self.front - self.rear) / 2.0)

    @property
    def largest(self) -> float:
        return max(self.front, self.side, self.rear)


# What a policy says a person's Comfort Space points at.
#
#   body      the direction the person is facing, from their config pose
#   velocity  the direction they are actually travelling
#   none      direction does not enter this policy's geometry
HEADING_BODY = "body"
HEADING_VELOCITY = "velocity"
HEADING_NONE = "none"


class ComfortSpacePolicy(ABC):
    """Decides the size and shape of each human's Comfort Space."""

    @abstractmethod
    def radius_for(self, key: str, state: str, speed: float = 0.0) -> float:
        ...

    def shape_for(self, key: str, state: str,
                  speed: float = 0.0) -> Tuple[float, float]:
        """Semi-axes (along the heading, across it).

        Round by default. A policy that stretches the space for someone
        who is moving overrides this.
        """
        radius = self.radius_for(key, state, speed)
        return radius, radius

    def extents_for(self, key: str, state: str,
                    speed: float = 0.0) -> Extent:
        """How far the space reaches in front, to the side and behind.

        Derived from `shape_for` by default, which makes every space
        symmetric front to back -- true of a circle and of the centred
        ellipse. A policy that puts more room behind a person than in
        front of them overrides this instead of `shape_for`.
        """
        along, across = self.shape_for(key, state, speed)
        return Extent(along, across, along)

    def heading_source_for(self, state: str) -> str:
        """Which direction this policy points a person's space along.

        `none` by default: the existing policies are round, or stretch
        along a travel direction the caller already supplies, so they
        never need a standing person's body orientation. Returning
        `none` keeps their behaviour exactly as it was.
        """
        return HEADING_NONE

    @property
    @abstractmethod
    def name(self) -> str:
        """Short label recorded with every trial."""

    def gives_space(self) -> bool:
        """Whether this policy asks for any room around people at all.

        True for everything except B1. It lets a node tell "nobody has
        a Comfort Space because the policy says so" from "nobody has a
        Comfort Space because something is broken", which from the
        outside look identical -- the robot drives straight past
        everybody either way.
        """
        return True

    def radii(self, states: Dict[str, str],
              speeds: Dict[str, float] | None = None) -> Dict[str, float]:
        return {key: self.radius_for(key, state, (speeds or {}).get(key, 0.0))
                for key, state in states.items()}

    def axes(self, states: Dict[str, str],
             speeds: Dict[str, float] | None = None
             ) -> Dict[str, Tuple[float, float, float]]:
        """(along, across, offset) per person.

        Third element is new. It is 0.0 for every policy that does not
        offset the ellipse, and `shapes()` accepts a two-element tuple
        as well, so an older caller building its own axes dict by hand
        still works.
        """
        out = {}
        for key, state in states.items():
            speed = (speeds or {}).get(key, 0.0)
            out[key] = self.extents_for(key, state, speed).axes
        return out

    def extents(self, states: Dict[str, str],
                speeds: Dict[str, float] | None = None) -> Dict[str, Extent]:
        """Per-person extents, for logging and for the preview."""
        return {key: self.extents_for(key, state, (speeds or {}).get(key, 0.0))
                for key, state in states.items()}


class StaticComfortPolicy(ComfortSpacePolicy):
    """One radius for everyone, whatever they are doing. Scenario B2."""

    def __init__(self, cfg: Dict[str, Any]):
        cs = cfg["comfort_space"]
        self._radius = max(float(cs["static_radius"]), float(cs["min_radius"]))

    def radius_for(self, key: str, state: str, speed: float = 0.0) -> float:
        # Speed is ignored, and that is the whole point of this policy.
        return self._radius

    @property
    def name(self) -> str:
        return "static"


class StateAdaptiveComfortPolicy(ComfortSpacePolicy):
    """Radius from the human's state. Scenario B3.

    A person absorbed in their workstation is not attending to the
    corridor, so the space they need kept clear behind them is smaller
    than for someone who might step out into it.
    """

    def __init__(self, cfg: Dict[str, Any]):
        cs = cfg["comfort_space"]
        self._by_state = {str(k): float(v) for k, v in cs["state_radius"].items()}
        self._min = float(cs["min_radius"])
        self._fallback = float(cs["static_radius"])

        dyn = cs.get("dynamic", {})
        self._dynamic = bool(dyn.get("enabled", False))
        self._reference_speed = max(float(dyn.get("reference_speed", 1.0)), 1e-6)
        self._at_rest_state = str(dyn.get("at_rest_state", "stationary"))
        self._elongation = float(cs.get("moving_elongation", 1.0))
        self._use_ellipse = str(cs.get("shape", "circle")) == "ellipse"

    def _blend(self, state: str, speed: float) -> float:
        """How far toward the full moving values this speed sits, 0 to 1.

        Only a person whose state is `moving` scales. Someone standing at
        a workstation is not going to be reinterpreted because of sensor
        noise in a velocity estimate.
        """
        if not self._dynamic or state != "moving":
            return 1.0 if state == "moving" else 0.0
        return min(max(speed / self._reference_speed, 0.0), 1.0)

    def radius_for(self, key: str, state: str, speed: float = 0.0) -> float:
        if state not in self._by_state:
            raise ValueError(
                f"no comfort_space.state_radius entry for state {state!r}; "
                f"have {sorted(self._by_state)}")
        if not self._dynamic or state != "moving":
            return max(self._by_state[state], self._min)

        rest = self._by_state.get(self._at_rest_state,
                                  self._by_state["moving"])
        full = self._by_state["moving"]
        t = self._blend(state, speed)
        return max(rest + (full - rest) * t, self._min)

    def shape_for(self, key: str, state: str,
                  speed: float = 0.0) -> Tuple[float, float]:
        across = self.radius_for(key, state, speed)
        if not self._use_ellipse or state != "moving":
            return across, across
        # The stretch fades out with the radius, so a walker who comes to
        # a halt ends up with the same round space as someone standing.
        t = self._blend(state, speed)
        return across * (1.0 + (self._elongation - 1.0) * t), across

    @property
    def name(self) -> str:
        return "state_adaptive"


class NoComfortPolicy(ComfortSpacePolicy):
    """No Comfort Space at all. Scenario B1, the baseline.

    People are left to Nav2 exactly as any other obstacle: whatever the
    LiDAR marks, inflated by the costmap. Nothing is painted into the
    Comfort Space mask, nothing is drawn, and the yield gate has nobody to
    yield to. What remains is what an AMR does with no notion of personal
    space, which is the reference B2 and B3 are measured against.
    """

    def __init__(self, cfg: Dict[str, Any]):
        pass

    def gives_space(self) -> bool:
        return False

    def radius_for(self, key: str, state: str, speed: float = 0.0) -> float:
        return 0.0

    @property
    def name(self) -> str:
        return "none"



def _unscaled_reference_size(cs: Dict[str, Any]) -> float:
    """The B3-O moving-front extent at the reference speed, unscaled.

    The one number the whole directional geometry is sized relative to:
    1.20 m with the shipped mapping (0.90 + 0.30 * 1.0 m/s). Read from
    the config rather than written down, so a change to the mapping
    cannot leave a stale 1.20 here.
    """
    moving = (cs.get("directional", {}) or {}).get("moving", {}) or {}
    mapping = moving.get("speed_mapping", {}) or {}
    front = mapping.get("front")
    if not front:
        return float(cs.get("static_radius", 1.2))
    speed = float((cs.get("dynamic", {}) or {}).get("reference_speed", 1.0))
    value = float(front["intercept_m"]) + float(front["gain_s"]) * speed
    return min(max(value, float(front["min_m"])), float(front["max_m"]))


def _scaling(cfg: Dict[str, Any]) -> Tuple[bool, float, float]:
    """(enabled, scale, floor) for the optional narrow-aisle squeeze.

    One global scale for every state, pulling each extent toward a floor
    rather than multiplying it, so a space can never shrink below the
    minimum the config already insists on:

        used = floor + scale * (wanted - floor)

    Off by default. It exists because the literature-anchored parameters
    are not measured in a 1.5 m aisle and may have to be brought in to
    fit one, and that is a decision to take deliberately, once, with the
    scaled and unscaled numbers both on the record. It is applied to
    every state alike so the relation between states is untouched, and
    it is NEVER applied to a safety distance -- the protective field is
    sized by scripts/generate_nav2_params.py from the robot's speed and
    knows nothing about this.

    The scale can be given two ways:

        scale: 0.8333            directly, as before
        target_size_m: 1.0       as the size the moving-front extent at
                                 the reference speed should come out at;
                                 scale = target / reference_size_m, and
                                 reference_size_m defaults to the
                                 unscaled value of that same extent
                                 (1.20 m with the shipped mapping)

    The second form is what the Stage 1 size sweep uses: "B3-O at
    1.0 m" means the whole geometry -- every state, every direction,
    the speed mapping and its clips -- multiplied by 1.0 / 1.20, so the
    front/side/rear ratios and the state relations are exactly those
    of the unscaled policy. With `floor_m: 0.0` the squeeze is a pure
    multiplication and only `min_radius` remains as a floor.
    """
    cs = cfg["comfort_space"]
    spec = cs.get("aisle_scaling", {}) or {}
    enabled = bool(spec.get("enabled", False))
    if spec.get("target_size_m") is not None:
        # common.yaml ships `scale: 1.0` (off), and an overlay that sets
        # a target merges on top of it. Two different answers to the
        # same question are refused; a target beside the neutral 1.0 is
        # just the target.
        if spec.get("scale") is not None and abs(float(spec["scale"]) - 1.0) > 1e-12:
            raise ValueError(
                "comfort_space.aisle_scaling gives both `scale` "
                f"({spec['scale']}) and `target_size_m` ({spec['target_size_m']}); "
                "give one or the other")
        reference = float(spec.get("reference_size_m")
                          or _unscaled_reference_size(cs))
        if reference <= 0.0:
            raise ValueError("comfort_space.aisle_scaling.reference_size_m "
                             "must be positive")
        scale = float(spec["target_size_m"]) / reference
    else:
        scale = float(spec.get("scale", 1.0))
    if enabled and scale <= 0.0:
        raise ValueError("comfort_space.aisle_scaling scale must be positive")
    return (enabled, scale,
            float(spec.get("floor_m", cs.get("min_radius", 0.0))))


class StateDirectionalComfortPolicy(ComfortSpacePolicy):
    """State, facing direction and walking speed together. Scenario B3.

    Two asymmetries, in opposite directions, because the reasons for
    them are different.

    A person who is standing and free to look around has more room
    behind them than in front: being passed from behind is reported as
    less comfortable than being passed in front (Neggers et al. 2022).
    So rear > side > front, pointed by their body orientation.

    A person engaged at a machine is the other way round: the space
    behind someone is compressed when they cannot see it (Hayduk 1981),
    and a worker facing a conveyor is attending to the work, not to the
    aisle at their back. So front >= side >= rear, with the rear equal
    to the state_adaptive radius for the same state.

    A person who is walking has more room in front: that is where they
    are about to be, and the faster they walk the further ahead that
    reaches. So front > side ~ rear, pointed by their actual velocity.

    > Literature-anchored engineering parameters. The qualitative
    > relations are from the passing-distance literature; the numbers
    > and the speed mapping are engineering choices, configurable, and
    > not measured comfort distances.
    """

    def __init__(self, cfg: Dict[str, Any]):
        cs = cfg["comfort_space"]
        spec = cs.get("directional")
        if not spec:
            raise ValueError(
                "comfort_space.policy is 'state_directional' but there is no "
                "comfort_space.directional block; see configs/common.yaml")
        self._static: Dict[str, Extent] = {}
        self._source: Dict[str, str] = {}
        self._speed_map: Dict[str, Dict[str, Dict[str, float]]] = {}
        self._min_heading_speed: Dict[str, float] = {}

        for state, entry in spec.items():
            entry = entry or {}
            self._source[str(state)] = str(entry.get("heading_source",
                                                     HEADING_BODY))
            mapping = entry.get("speed_mapping")
            if mapping:
                self._speed_map[str(state)] = {
                    str(k): {str(kk): float(vv) for kk, vv in (v or {}).items()}
                    for k, v in mapping.items()}
                self._min_heading_speed[str(state)] = float(
                    entry.get("heading_min_speed_mps", 0.0))
            else:
                self._static[str(state)] = Extent(
                    float(entry["front_m"]),
                    float(entry["side_m"]),
                    float(entry["rear_m"]))
        self._min = float(cs["min_radius"])
        self._scaled, self._scale, self._floor = _scaling(cfg)
        self._reference_speed = float(
            (cs.get("dynamic", {}) or {}).get("reference_speed", 1.0))

    # -- the speed mapping -------------------------------------------
    @staticmethod
    def _ramp(spec: Dict[str, float], speed: float) -> float:
        """intercept + gain * speed, clipped to [min, max]."""
        value = spec["intercept_m"] + spec["gain_s"] * speed
        return min(max(value, spec["min_m"]), spec["max_m"])

    def _squeeze(self, value: float) -> float:
        if self._scaled:
            value = self._floor + self._scale * (value - self._floor)
        return max(value, self._min)

    def extents_for(self, key: str, state: str,
                    speed: float = 0.0) -> Extent:
        if state in self._speed_map:
            m = self._speed_map[state]
            raw = Extent(self._ramp(m["front"], speed),
                         self._ramp(m["side"], speed),
                         self._ramp(m["rear"], speed))
        elif state in self._static:
            raw = self._static[state]
        else:
            raise ValueError(
                f"no comfort_space.directional entry for state {state!r}; "
                f"have {sorted(set(self._static) | set(self._speed_map))}")
        return Extent(self._squeeze(raw.front), self._squeeze(raw.side),
                      self._squeeze(raw.rear))

    def heading_source_for(self, state: str) -> str:
        return self._source.get(state, HEADING_BODY)

    def heading_min_speed(self, state: str) -> float:
        """Below this the travel direction is too noisy to trust."""
        return self._min_heading_speed.get(state, 0.0)

    def radius_for(self, key: str, state: str, speed: float = 0.0) -> float:
        """The largest extent, for callers that want one number.

        isaac_sim/utils/fleet.py and the clearance table in
        scripts/make_scenario.py ask a policy for a radius. Giving them
        the largest of the three keeps them conservative: they will
        leave at least as much room as the real shape needs, never less.
        The reported figure is therefore an upper bound for this policy,
        not the exact passage.
        """
        return self.extents_for(key, state, speed).largest

    def shape_for(self, key: str, state: str,
                  speed: float = 0.0) -> Tuple[float, float]:
        along, across, _offset = self.extents_for(key, state, speed).axes
        return along, across

    @property
    def name(self) -> str:
        return "state_directional"

    @property
    def scale_factor(self) -> float:
        """The global multiplier in force, 1.0 when scaling is off."""
        return self._scale if self._scaled else 1.0

    @property
    def reference_speed(self) -> float:
        return self._reference_speed

    def describe(self) -> List[str]:
        """One line per state, for the startup log."""
        lines = []
        for state in sorted(set(self._static) | set(self._speed_map)):
            src = self.heading_source_for(state)
            if state in self._speed_map:
                e = self.extents_for("_", state, 1.0)
                lines.append(f"{state}: heading from {src}, at 1.0 m/s "
                             f"F={e.front:.2f} S={e.side:.2f} R={e.rear:.2f}")
            else:
                e = self.extents_for("_", state)
                lines.append(f"{state}: heading from {src}, "
                             f"F={e.front:.2f} S={e.side:.2f} R={e.rear:.2f}")
        if self._scaled:
            lines.append(f"aisle scaling ON: scale={self._scale:.4f} "
                         f"floor={self._floor:.2f} m min_radius={self._min:.2f} m "
                         f"(extents above are scaled)")
        return lines


class HeadingResolver:
    """Which way each person's Comfort Space points, tick by tick.

    Separate from the policy because it needs memory. A walker's
    velocity direction is meaningless as they come to a stop -- the
    estimate flips about while the speed falls through a few centimetres
    per second -- so the last direction they were definitely travelling
    in is held until they are moving again. Without that the Comfort
    Space spins on the spot every time somebody arrives at a machine.

    A standing person points along their body orientation, read from
    their configured pose. If nothing in the scene says which way a
    person faces -- which is the normal case for somebody found by the
    LiDAR, since it reports a position and not an orientation -- the
    resolver says so by returning None rather than inventing a
    direction, and `shapes()` falls back to a round space.
    """

    def __init__(self, cfg: Dict[str, Any], policy: ComfortSpacePolicy):
        self.cfg = cfg
        self.policy = policy
        humans = cfg.get("humans", {}) or {}
        self.body: Dict[str, float] = {}
        for key, _x, _y in human_positions(cfg):
            spec = humans.get(key)
            if isinstance(spec, dict) and spec.get("facing_deg") is not None:
                self.body[key] = math.radians(float(spec["facing_deg"]))
        self._held: Dict[str, float] = {}

    def resolve(self, states: Dict[str, str],
                motion_headings: Dict[str, float] | None = None,
                speeds: Dict[str, float] | None = None
                ) -> Dict[str, Optional[float]]:
        motion = motion_headings or {}
        speed = speeds or {}
        out: Dict[str, Optional[float]] = {}
        for key, state in states.items():
            source = self.policy.heading_source_for(state)
            if source == HEADING_VELOCITY:
                floor = getattr(self.policy, "heading_min_speed",
                                lambda _s: 0.0)(state)
                if key in motion and float(speed.get(key, 0.0)) >= floor:
                    self._held[key] = float(motion[key])
                    out[key] = self._held[key]
                elif key in self._held:
                    out[key] = self._held[key]
                else:
                    out[key] = self.body.get(key)
            elif source == HEADING_BODY:
                # A body orientation only. Dropping any travel direction
                # here is the point: somebody who has just stopped must
                # not keep a space pointed the way they were walking.
                out[key] = self.body.get(key)
            else:
                # Policies that do not use direction behave exactly as
                # before: whatever the caller supplied, else +x.
                out[key] = float(motion.get(key, 0.0))
        return out


POLICIES = {
    "none": NoComfortPolicy,
    "static": StaticComfortPolicy,
    "state_adaptive": StateAdaptiveComfortPolicy,
    "state_directional": StateDirectionalComfortPolicy,
}


def make_policy(cfg: Dict[str, Any]) -> ComfortSpacePolicy:
    """Build the policy named by comfort_space.policy."""
    name = str(cfg["comfort_space"]["policy"])
    try:
        return POLICIES[name](cfg)
    except KeyError:
        raise ValueError(
            f"comfort_space.policy is {name!r}; expected one of {list(POLICIES)}")


# The columns `geometry_record` fills, in the order they are written.
# Metres actually applied, after scaling and the min_radius floor, so a
# results file says what the robot kept clear and not only which scale
# was asked for. `comfort_size_m` is the one number a sweep is indexed
# by: the static radius for `static`, the moving-front extent at the
# reference speed for `state_directional` (1.20 m unscaled), and the
# moving radius for `state_adaptive`.
GEOMETRY_FIELDS: Tuple[str, ...] = (
    "comfort_size_m",
    "comfort_scale",
    "cs_engaged_front_m",
    "cs_engaged_side_m",
    "cs_engaged_rear_m",
    "cs_stationary_front_m",
    "cs_stationary_side_m",
    "cs_stationary_rear_m",
    "cs_moving_front_m",
    "cs_moving_side_m",
    "cs_moving_rear_m",
    "cs_moving_reference_speed",
)


def geometry_record(cfg: Dict[str, Any],
                    policy: Optional[ComfortSpacePolicy] = None
                    ) -> Dict[str, Any]:
    """The Comfort Space extents a trial actually ran with, in metres.

    One flat dict, keys GEOMETRY_FIELDS, for the trial row. A policy
    with no direction (static, state_adaptive) fills front, side and
    rear with the same radius; `none` leaves everything empty except a
    zero size, so B1 rows still say what they were.

    `comfort_scale` is relative to the unscaled directional reference
    (1.20 m shipped) for every policy, so static 1.20 and B3-O 1.20 both
    read 1.0 and a size sweep of the two shares one axis.
    """
    cs = cfg["comfort_space"]
    policy = policy or make_policy(cfg)
    reference = _unscaled_reference_size(cs)
    empty = {k: "" for k in GEOMETRY_FIELDS}
    if not policy.gives_space():
        empty["comfort_size_m"] = 0.0
        empty["comfort_scale"] = 0.0
        return empty

    speed = float((cs.get("dynamic", {}) or {}).get("reference_speed", 1.0))
    ext = {state: policy.extents_for("_", state, speed if state == "moving"
                                     else 0.0)
           for state in ("station_engaged", "stationary", "moving")}
    if isinstance(policy, StateDirectionalComfortPolicy):
        size = ext["moving"].front
    elif isinstance(policy, StaticComfortPolicy):
        size = ext["moving"].front  # the one radius
    else:
        size = policy.radius_for("_", "moving", speed)
    r = lambda v: round(float(v), 4)  # noqa: E731
    return {
        "comfort_size_m": r(size),
        "comfort_scale": r(size / reference) if reference > 0 else "",
        "cs_engaged_front_m": r(ext["station_engaged"].front),
        "cs_engaged_side_m": r(ext["station_engaged"].side),
        "cs_engaged_rear_m": r(ext["station_engaged"].rear),
        "cs_stationary_front_m": r(ext["stationary"].front),
        "cs_stationary_side_m": r(ext["stationary"].side),
        "cs_stationary_rear_m": r(ext["stationary"].rear),
        "cs_moving_front_m": r(ext["moving"].front),
        "cs_moving_side_m": r(ext["moving"].side),
        "cs_moving_rear_m": r(ext["moving"].rear),
        "cs_moving_reference_speed": r(speed),
    }


def robot_grow(cfg: Dict[str, Any]) -> float:
    """How much to enlarge a Comfort Space for a planner.

    The configured radius is the distance from the human to the nearest
    part of the AMR. A planner works with a point at the robot's centre,
    so the shape it avoids has to be bigger by the AMR's inscribed
    radius. Defined here, once, so no caller has to remember it.
    """
    return float(cfg["amr"]["footprint_width"]) / 2.0


def shapes(cfg: Dict[str, Any], axes: Dict[str, Tuple[float, ...]],
           positions: Dict[str, tuple] | None = None,
           headings: Dict[str, Optional[float]] | None = None,
           grow: float = 0.0) -> List[ComfortShape]:
    """Build one Comfort Space per human from the policy's semi-axes.

    `positions` and `headings` override the configured pose, which is
    what a walking person needs: the caller passes where they are and
    which way they are going, and everything downstream is unchanged.

    `axes` holds (along, across) or (along, across, offset). A
    two-element entry means a centred shape, so a caller that builds
    the dict by hand keeps working.

    A heading of None means nothing in the scene says which way this
    person faces. Rather than pick a direction, the space is made round
    at its largest extent -- the conservative reading, and the one that
    cannot claim an orientation the sensor never measured.

    `grow` adds the AMR's inscribed radius, turning the space a person
    experiences into the one a planner working on the robot's centre
    point has to avoid.
    """
    out: List[ComfortShape] = []
    # Over the people the POLICY was given, not the people the config
    # declares.
    #
    # They are the same set when the poses come from the simulator, and
    # a different set entirely when they come from the robot's own
    # LiDAR: a detection is `p1`, not `w4`, because the sensor does not
    # know anyone's name. Iterating the config silently produced no
    # Comfort Spaces at all for detected people -- B2 and B3 then drove
    # exactly like B1 and the trial summary read "yield 0.0 s" with
    # nothing else out of place. Measured over 12 trials before anyone
    # noticed.
    configured = {key: (x, y) for key, x, y in human_positions(cfg)}
    for key in axes:
        if positions and key in positions:
            x, y = positions[key][0], positions[key][1]
        elif key in configured:
            x, y = configured[key]
        else:
            # Nothing says where this person is, so there is nothing to
            # keep clear of. Better to leave them out than to invent a
            # position for them.
            continue
        entry = tuple(axes[key])
        along, across = float(entry[0]), float(entry[1])
        offset = float(entry[2]) if len(entry) > 2 else 0.0
        if along <= 0.0 and across <= 0.0:
            # No Comfort Space for this person (policy "none"). Skipped
            # here rather than grown into a robot-sized circle, which
            # would quietly give B1 a Comfort Space after all.
            continue
        raw = (headings or {}).get(key, 0.0)
        if raw is None:
            # Unknown orientation: round, at the largest extent the
            # policy asked for anywhere.
            radius = max(along + abs(offset), across)
            out.append(ComfortShape(key, float(x), float(y),
                                    radius + grow, radius + grow, 0.0, 0.0))
            continue
        out.append(ComfortShape(key, float(x), float(y),
                                along + grow, across + grow,
                                float(raw), offset))
    return out
