"""Where a human's state comes from.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

The point of this module is the seam. Today the state is read from the
config as simulation ground truth. Later it will be estimated from a 4D
LiDAR. Navigation code should never know which, so it asks a
HumanStateProvider and gets back a mapping of human key to state.

Keeping the seam here also protects the experiment. Detection error is a
confound: if the Comfort Space flickers because a person was missed for
three frames, B2 and B3 no longer differ only in policy. The B2/B3
comparison runs on ground truth on purpose, and the sensor provider is
evaluated separately against it.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict

import math

from comfort_core.layout import human_states, work_spots

# The states the study recognises. `moving` is defined but unused in the
# B2/B3 comparison, where both humans stand still.
KNOWN_STATES = ("moving", "stationary", "station_engaged")


class HumanStateProvider(ABC):
    """Supplies the current state of every human, keyed as in the config."""

    @abstractmethod
    def states(self) -> Dict[str, str]:
        ...

    @property
    @abstractmethod
    def source(self) -> str:
        """Short label for the results file, so a trial records how the
        state was obtained."""


class GroundTruthStateProvider(HumanStateProvider):
    """Reads the states straight out of the config.

    Used for the B2/B3 comparison.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self._states = human_states(cfg)
        unknown = {k: v for k, v in self._states.items() if v not in KNOWN_STATES}
        if unknown:
            raise ValueError(
                f"humans have unrecognised states {unknown}; "
                f"expected one of {list(KNOWN_STATES)}")

    def states(self) -> Dict[str, str]:
        return dict(self._states)

    @property
    def source(self) -> str:
        return "ground_truth"


class SensorStateProvider(HumanStateProvider):
    """State from what a LiDAR can actually measure, plus the map.

    A range sensor gives two things about a person: where they are and
    how fast they are going. It cannot give `station_engaged`, which is
    about what somebody is attending to. The map can, indirectly: the
    floor plan knows where a machine is and where a person stands to
    use it, so

        moving          speed over the threshold
        station_engaged stopped, AND within `station_radius` of a spot
                        the map marks as a working position
        stationary      stopped anywhere else

    This is a heuristic over prior knowledge, NOT perception, and has to
    be described that way: the robot is not seeing that someone is
    working, it is seeing that someone is standing where work happens.

    What it does not use is the person's identity. It is handed
    positions and speeds, never the configured state, so it can be
    wrong -- and being able to be wrong is the point. How wrong is the
    confusion matrix a trial prints.

    Fed by `update()` each time new observations arrive; `states()`
    returns the latest estimate. Before the first update it knows
    nothing and returns nothing, which is the honest answer.
    """

    def __init__(self, cfg: Dict[str, Any]):
        spec = cfg.get("humans", {}).get("state_estimator", {})
        self.spots = work_spots(cfg, float(spec.get("stand_gap", 0.30)))
        # Two speeds, not one. With a single threshold a person walking
        # at about the threshold flickers between moving and stationary
        # several times a second, and the Comfort Space flickers with
        # them.
        self.enter = float(spec.get("moving_speed", 0.15))
        self.leave = float(spec.get("settle_speed", 0.10))
        self.radius = float(spec.get("station_radius", 0.40))
        # Optional second test: is the person facing the machine? A
        # person standing at a bench with their back to it is not using
        # it, and position alone cannot tell the two apart. None turns
        # the test off, which is the default because it needs an
        # orientation estimate and a plain 2D scan does not give one.
        tol = spec.get("facing_tolerance_deg")
        self.facing_tolerance = None if tol is None else math.radians(float(tol))
        # How long a new reading has to hold before it is believed.
        self.hold = float(spec.get("hold", 0.6))
        self._state: Dict[str, str] = {}
        self._candidate: Dict[str, tuple] = {}
        self.margin = _station_radius_margin(cfg, self.spots, self.radius)
        if self.margin is not None and self.margin < 0.0:
            raise ValueError(
                f"humans.state_estimator.station_radius is {self.radius:.2f} m "
                f"and a working position is only {self.radius + self.margin:.2f} m "
                f"from the edge of an aisle; anybody who stopped in the lane "
                f"would be called a worker")

    def update(self, positions: Dict[str, tuple], speeds: Dict[str, float],
               now: float,
               facings: Dict[str, float] | None = None) -> Dict[str, str]:
        """Take one set of observations. Returns the state in force."""
        for key, (x, y) in positions.items():
            guess = self._classify(self._state.get(key), x, y,
                                   speeds.get(key, 0.0),
                                   (facings or {}).get(key))
            if guess == self._state.get(key):
                self._candidate.pop(key, None)
                continue
            since = self._candidate.get(key)
            if since is None or since[0] != guess:
                self._candidate[key] = (guess, now)
                # Nothing is in force yet: take the first reading as it
                # comes rather than report nothing for the first second.
                if key not in self._state:
                    self._state[key] = guess
                continue
            if now - since[1] >= self.hold:
                self._state[key] = guess
                self._candidate.pop(key, None)

        # Forget anybody this observation did not mention.
        #
        # Both callers pass the complete current set every time -- all
        # the people in the scene from /tf, or everybody the sensor can
        # see right now -- so a key that is missing is a person who is
        # not there any more, not a person who went unmentioned. Left
        # in, their state outlived them: a trial accumulated forty-four
        # people in a bay that holds twelve, every one of them still
        # being handed to the policy.
        gone = set(self._state) - set(positions)
        for key in gone:
            self._state.pop(key, None)
            self._candidate.pop(key, None)
        return dict(self._state)

    def _classify(self, held: str | None, x: float, y: float,
                  speed: float, facing: float | None) -> str:
        if speed >= (self.leave if held == "moving" else self.enter):
            return "moving"
        spot = self.nearest_spot(x, y)
        if spot is None or spot[0] > self.radius:
            return "stationary"
        if self.facing_tolerance is not None and facing is not None:
            want = spot[1][2]
            off = abs(math.atan2(math.sin(facing - want),
                                 math.cos(facing - want)))
            if off > self.facing_tolerance:
                return "stationary"
        return "station_engaged"

    def nearest_spot(self, x: float, y: float):
        """(distance, spot) for the closest working position, or None."""
        if not self.spots:
            return None
        return min(((math.hypot(x - sx, y - sy), (sx, sy, f))
                    for sx, sy, f in self.spots), key=lambda it: it[0])

    def distance_to_spot(self, x: float, y: float) -> float:
        """Metres from the nearest working position. inf with no map."""
        spot = self.nearest_spot(x, y)
        return float("inf") if spot is None else spot[0]

    def states(self) -> Dict[str, str]:
        return dict(self._state)

    @property
    def source(self) -> str:
        return "sensor:station_map"


def _station_radius_margin(cfg: Dict[str, Any], spots, radius: float):
    """How much room is left between the radius and the nearest aisle.

    The failure this guards against is specific: make the radius large
    enough and it reaches into the traffic lane, so a person who stops
    in the aisle is read as a worker at a machine. Returns None when
    the layout has no aisles to measure against.
    """
    if "factory" not in cfg or not spots:
        return None
    from comfort_core.factory_layout import build_layout
    aisles = build_layout(cfg).aisles
    if not aisles:
        return None
    worst = None
    for sx, sy, _facing in spots:
        for band in aisles:
            # Distance from the spot to the nearer edge of this aisle.
            if band.y_min <= sy <= band.y_max:
                gap = 0.0
            else:
                gap = min(abs(sy - band.y_min), abs(sy - band.y_max))
            worst = gap if worst is None else min(worst, gap)
    return None if worst is None else worst - radius


PROVIDERS = {
    "ground_truth": GroundTruthStateProvider,
    "sensor": SensorStateProvider,
}


def make_state_provider(cfg: Dict[str, Any]) -> HumanStateProvider:
    """Build the provider named by humans.state_source, default ground truth."""
    name = str(cfg.get("humans", {}).get("state_source", "ground_truth"))
    try:
        return PROVIDERS[name](cfg)
    except KeyError:
        raise ValueError(
            f"humans.state_source is {name!r}; expected one of {list(PROVIDERS)}")
