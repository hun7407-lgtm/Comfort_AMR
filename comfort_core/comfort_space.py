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

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Tuple

from comfort_core.layout import human_positions
from comfort_core.shapes import ComfortShape


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

    @property
    @abstractmethod
    def name(self) -> str:
        """Short label recorded with every trial."""

    def radii(self, states: Dict[str, str],
              speeds: Dict[str, float] | None = None) -> Dict[str, float]:
        return {key: self.radius_for(key, state, (speeds or {}).get(key, 0.0))
                for key, state in states.items()}

    def axes(self, states: Dict[str, str],
             speeds: Dict[str, float] | None = None) -> Dict[str, Tuple[float, float]]:
        return {key: self.shape_for(key, state, (speeds or {}).get(key, 0.0))
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


POLICIES = {
    "static": StaticComfortPolicy,
    "state_adaptive": StateAdaptiveComfortPolicy,
}


def make_policy(cfg: Dict[str, Any]) -> ComfortSpacePolicy:
    """Build the policy named by comfort_space.policy."""
    name = str(cfg["comfort_space"]["policy"])
    try:
        return POLICIES[name](cfg)
    except KeyError:
        raise ValueError(
            f"comfort_space.policy is {name!r}; expected one of {list(POLICIES)}")


def robot_grow(cfg: Dict[str, Any]) -> float:
    """How much to enlarge a Comfort Space for a planner.

    The configured radius is the distance from the human to the nearest
    part of the AMR. A planner works with a point at the robot's centre,
    so the shape it avoids has to be bigger by the AMR's inscribed
    radius. Defined here, once, so no caller has to remember it.
    """
    return float(cfg["amr"]["footprint_width"]) / 2.0


def costmap_radii(cfg: Dict[str, Any], radii: Dict[str, float]) -> Dict[str, float]:
    """Convert comfort radii into the radii to paint into a costmap.

    The configured radius is the distance from the human to the nearest
    part of the AMR. A planner works with a point at the robot's centre,
    so the painted circle has to be bigger by the AMR's inscribed radius,
    or the robot's body would end up closer than the radius allows.

    Done here, once, so no caller has to remember it.
    """
    inscribed = robot_grow(cfg)
    return {key: r + inscribed for key, r in radii.items()}


def shapes(cfg: Dict[str, Any], axes: Dict[str, Tuple[float, float]],
           positions: Dict[str, tuple] | None = None,
           headings: Dict[str, float] | None = None,
           grow: float = 0.0) -> List[ComfortShape]:
    """Build one Comfort Space per human from the policy's semi-axes.

    `positions` and `headings` override the configured pose, which is
    what a walking person needs: the caller passes where they are and
    which way they are going, and everything downstream is unchanged.

    `grow` adds the AMR's inscribed radius, turning the space a person
    experiences into the one a planner working on the robot's centre
    point has to avoid.
    """
    out: List[ComfortShape] = []
    for key, x, y in human_positions(cfg):
        if key not in axes:
            continue
        if positions and key in positions:
            x, y = positions[key][0], positions[key][1]
        along, across = axes[key]
        heading = float((headings or {}).get(key, 0.0))
        out.append(ComfortShape(key, float(x), float(y),
                                along + grow, across + grow, heading))
    return out


def circles(cfg: Dict[str, Any], radii: Dict[str, float],
            positions: Dict[str, tuple] | None = None):
    """Yield (key, x, y, radius) for every human, in a stable order.

    Kept for callers that only need round Comfort Spaces. New code
    should use `shapes`, which also covers the moving case.
    """
    for key, x, y in human_positions(cfg):
        if key not in radii:
            continue
        if positions and key in positions:
            x, y = positions[key][0], positions[key][1]
        yield key, float(x), float(y), radii[key]
