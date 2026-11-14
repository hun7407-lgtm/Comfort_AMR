"""B3-O Dynamic Selection: the Comfort Space policy of Stage 3.

Pure Python. Registered as comfort_space.policy `b3o_selection`
(comfort_core.comfort_space.make_policy).

B3-O's directional geometry (state, facing, speed) with one change: for a
person standing or working at a known spot, the whole shape is scaled so
that its extent TOWARD THE PASSAGE equals R*, chosen by
comfort_core.selection.select:

    R_comfort_min(H) <= R* <= R_nav_safe(E, H)      or PASSAGE_INFEASIBLE

R_comfort_min / R_pref come from the frozen Stage 2 file
(configs/frozen/comfort_requirements.yaml); R_nav_safe per spot and state
comes from the Stage 1 execution model, computed offline and written into
the condition overlay as `comfort_space.selection.spots`
(scripts/stage3_make_configs.py). Under PASSAGE_INFEASIBLE the shape stays
at R_comfort_min -- never smaller -- and the existing gate makes the robot
wait; Nav2 reroutes if it can.

A walking person keeps B3-O's moving/speed mapping unchanged: that part is
an engineering, literature-based extension that Stage 2 does not
calibrate. So does anyone not at a listed spot.

The decisions taken are kept in `decisions` (person -> Selection.as_row())
for the record.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from comfort_core.comfort_space import Extent, StateDirectionalComfortPolicy
from comfort_core.contexts import CONTEXTS, REFERENCE_SIZE_M, extent_ratio, key_of
from comfort_core.selection import frozen_inputs, select

SELECTABLE_STATES = ("station_engaged", "stationary")


class SelectedDirectionalPolicy(StateDirectionalComfortPolicy):
    uses_positions = True

    def __init__(self, cfg: Dict[str, Any]):
        super().__init__(cfg)
        if self._scaled:
            raise ValueError("b3o_selection scales per person; comfort_space.aisle_scaling "
                             "must be disabled")
        sel = cfg["comfort_space"].get("selection") or {}
        self.spot_radius = float(sel.get("spot_radius_m", 0.6))
        self.spots = list(sel.get("spots") or [])
        req_path = sel.get("requirements_file")
        if sel.get("requirements"):            # inline, for tests only
            self.requirements = dict(sel["requirements"])
        else:
            data = frozen_inputs(Path(req_path)) if req_path else frozen_inputs()
            self.requirements = data["contexts"]
        self._cfg = cfg
        self.decisions: Dict[str, Dict[str, Any]] = {}

    @property
    def name(self) -> str:
        return "b3o_selection"

    def _spot(self, pos: Optional[Tuple[float, float]]):
        if pos is None or not self.spots:
            return None
        best = min(self.spots, key=lambda s: math.hypot(s["x"] - pos[0], s["y"] - pos[1]))
        if math.hypot(best["x"] - pos[0], best["y"] - pos[1]) > self.spot_radius:
            return None
        return best

    def factor(self, key: str, state: str, pos: Optional[Tuple[float, float]]) -> float:
        """Multiplier on this person's unscaled shape; 1.0 when not selected."""
        if state not in SELECTABLE_STATES:
            self.decisions.pop(key, None)
            return 1.0
        spot = self._spot(pos)
        entry = (spot or {}).get("states", {}).get(state)
        if not entry:
            self.decisions[key] = {"status": "NOT_SELECTED", "reason": "no spot entry"}
            return 1.0
        ctx = key_of(entry["context"])
        req = self.requirements.get(CONTEXTS[ctx]["name"]) or {}
        s = select(CONTEXTS[ctx]["name"], float(entry["r_nav_safe_m"]),
                   req.get("r_comfort_min_m"), req.get("r_pref_m"))
        self.decisions[key] = {**s.as_row(), "spot": spot.get("name", "")}
        unscaled = extent_ratio(self._cfg, ctx) * REFERENCE_SIZE_M
        return s.r_star / unscaled

    def axes(self, states: Dict[str, str], speeds: Dict[str, float] | None = None,
             positions: Dict[str, Tuple[float, float]] | None = None
             ) -> Dict[str, Tuple[float, float, float]]:
        out = {}
        for key, state in states.items():
            speed = (speeds or {}).get(key, 0.0)
            e = self.extents_for(key, state, speed)
            f = self.factor(key, state, (positions or {}).get(key))
            if f != 1.0:
                e = Extent(max(e.front * f, self._min), max(e.side * f, self._min),
                           max(e.rear * f, self._min))
            out[key] = e.axes
        return out
