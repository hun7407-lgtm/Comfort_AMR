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

from comfort_core.layout import human_states

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
    """Placeholder for 4D LiDAR based estimation. Not implemented.

    Two things have to be settled before this is written, and both are
    research decisions rather than coding ones:

    1. A LiDAR gives position and velocity. It does not give
       `station_engaged`, which is about what the person is attending to.
       Either body orientation gets estimated as well, or the state is
       defined operationally as a function of position, velocity and
       distance to a station. The second is legitimate but must be
       described as a heuristic, not as perception.

    2. Detection dropouts have to be handled, or the Comfort Space
       flickers and the comparison picks up detection noise.
    """

    def __init__(self, cfg: Dict[str, Any]):
        raise NotImplementedError(
            "SensorStateProvider is not implemented. Use ground truth for "
            "the B2/B3 comparison; see the class docstring.")

    def states(self) -> Dict[str, str]:
        raise NotImplementedError

    @property
    def source(self) -> str:
        return "sensor"


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
