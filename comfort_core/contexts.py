"""Human Context: a person's state plus which side of them the AMR passes.

Pure Python. Shared by Stage 1 (scripts/stage1a_plan.py), Stage 2
(comfort_core/stage2.py) and the selection rule (comfort_core/selection.py)
so the three cannot disagree about what "stationary-rear" means.

    c1  station_engaged, passed behind     extent read: rear
    c2  stationary,      passed behind     extent read: rear
    c3  stationary,      passed in front   extent read: front
    c4  stationary,      passed beside     extent read: side

Moving people are not a context here: the B3-O moving/speed mapping is an
engineering, literature-based extension that no human study in this
project calibrates (docs/STAGES.md).

`facing_offset_deg` is the person's facing relative to the direction from
the person TOWARD the passage: 0 = facing the passage (front toward it),
180 = back to it, 90 = side to it.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

CONTEXTS: Dict[str, Dict[str, Any]] = {
    "c1": {"name": "c1_engaged_rear", "state": "station_engaged", "facing_offset_deg": 180.0,
           "extent": "rear", "label": "engaged-rear (back to the aisle)"},
    "c2": {"name": "c2_stationary_rear", "state": "stationary", "facing_offset_deg": 180.0,
           "extent": "rear", "label": "stationary-rear (back to the aisle)"},
    "c3": {"name": "c3_stationary_front", "state": "stationary", "facing_offset_deg": 0.0,
           "extent": "front", "label": "stationary-front (facing the aisle)"},
    "c4": {"name": "c4_stationary_side", "state": "stationary", "facing_offset_deg": 90.0,
           "extent": "side", "label": "stationary-side (side to the aisle)"},
}
BY_NAME = {v["name"]: k for k, v in CONTEXTS.items()}

# B3-O "size" = moving-front extent at the reference speed, unscaled.
REFERENCE_SIZE_M = 1.20


def key_of(context: str) -> str:
    """'c2' or 'c2_stationary_rear' -> 'c2'."""
    if context in CONTEXTS:
        return context
    if context in BY_NAME:
        return BY_NAME[context]
    raise ValueError(f"unknown Human Context {context!r}; one of {list(CONTEXTS)}")


def facing_deg(context: str, passage_bearing_deg: float) -> float:
    """World facing of the person, given the bearing person -> passage."""
    return (passage_bearing_deg + CONTEXTS[key_of(context)]["facing_offset_deg"]) % 360.0


def extent_ratio(cfg: Dict[str, Any], context: str) -> float:
    """Effective extent per unit of B3-O target size, from the UNSCALED
    directional block: e = ratio * target_size_m (above the min_radius floor).
    c1 0.50, c2 0.875, c3 0.667, c4 0.75 with configs/common.yaml."""
    c = CONTEXTS[key_of(context)]
    d = cfg["comfort_space"]["directional"][c["state"]]
    return float(d[f"{c['extent']}_m"]) / REFERENCE_SIZE_M


def target_for_extent(cfg: Dict[str, Any], context: str, e: float) -> float:
    return float(e) / extent_ratio(cfg, context)


def context_of(state: str, facing_rad: Optional[float], toward_passage_rad: float) -> Optional[str]:
    """Which context a person is in, from their state and facing relative to
    the passage. None for a moving person or an unknown facing."""
    if state not in ("station_engaged", "stationary") or facing_rad is None:
        return None
    rel = abs((math.degrees(facing_rad - toward_passage_rad) + 180.0) % 360.0 - 180.0)
    if state == "station_engaged":
        return "c1"
    if rel <= 45.0:
        return "c3"
    if rel >= 135.0:
        return "c2"
    return "c4"
