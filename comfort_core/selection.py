"""R* selection: Human Comfort requirement within the Navigation bound.

Pure Python. No ROS.

    R_comfort_min(H) <= R* <= R_nav_safe(E, H)

    if R_nav_safe < R_comfort_min:   PASSAGE_INFEASIBLE
        the Comfort Space stays at R_comfort_min (never smaller); the
        robot waits / yields at the existing gate, and Nav2's global
        planner reroutes if another way exists. Nothing is shrunk to make
        the passage fit.
    else:
        primary   R* = min(R_pref, R_nav_safe)      when R_pref is defined
        fallback  R* = R_comfort_min                 when it is not

All quantities are EFFECTIVE corridor-side extents in metres (person
centre to the nearest surface of the AMR), the axis of Stage 1 and the
distance Stage 2 measures.

The per-context inputs are read from a FROZEN file
(configs/frozen/comfort_requirements.yaml), filled from Stage 2 and
committed before Stage 3. Every value starts null: no comfort number is
invented here. `frozen_inputs` refuses to return anything while a value
the rule needs is null, so Stage 3 cannot run on placeholders.

> R_nav_safe is a conservative bound of this navigation stack, checked
> in Stage 1. R_comfort_min / R_pref come from the human study. None of
> them is an ISO or "validated" comfort distance.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

FEASIBLE = "FEASIBLE"
INFEASIBLE = "PASSAGE_INFEASIBLE"

FROZEN = Path(__file__).resolve().parents[1] / "configs" / "frozen" / "comfort_requirements.yaml"


@dataclass(frozen=True)
class Selection:
    context: str
    r_nav_safe: float
    r_comfort_min: float
    r_pref: Optional[float]
    r_star: float
    status: str          # FEASIBLE / PASSAGE_INFEASIBLE
    rule: str            # primary / fallback / infeasible_hold

    def as_row(self) -> Dict[str, Any]:
        return {"context": self.context, "r_nav_safe_m": round(self.r_nav_safe, 4),
                "r_comfort_min_m": round(self.r_comfort_min, 4),
                "r_pref_m": None if self.r_pref is None else round(self.r_pref, 4),
                "r_star_m": round(self.r_star, 4), "status": self.status, "rule": self.rule}


def select(context: str, r_nav_safe: float, r_comfort_min: float,
           r_pref: Optional[float] = None) -> Selection:
    if r_comfort_min is None:
        raise ValueError(f"R_comfort_min for context {context!r} is not set")
    if r_pref is not None and r_pref < r_comfort_min:
        raise ValueError(f"R_pref {r_pref} below R_comfort_min {r_comfort_min} for {context!r}")
    if r_nav_safe < r_comfort_min:
        return Selection(context, r_nav_safe, r_comfort_min, r_pref,
                         r_comfort_min, INFEASIBLE, "infeasible_hold")
    if r_pref is None:
        return Selection(context, r_nav_safe, r_comfort_min, None,
                         r_comfort_min, FEASIBLE, "fallback")
    return Selection(context, r_nav_safe, r_comfort_min, r_pref,
                     min(r_pref, r_nav_safe), FEASIBLE, "primary")


def static_calibrated(requirements: Dict[str, Dict[str, Optional[float]]]) -> float:
    """B2's single radius: the smallest one value that meets every
    context's comfort requirement, max_H R_comfort_min(H)."""
    vals = [v.get("r_comfort_min_m") for v in requirements.values()]
    if not vals or any(v is None for v in vals):
        raise ValueError("every context needs r_comfort_min_m before B2 can be calibrated")
    return max(float(v) for v in vals)


def frozen_inputs(path: Path = FROZEN) -> Dict[str, Any]:
    """The frozen Stage 2 requirements; raises while any needed value is null."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    reqs = data.get("contexts") or {}
    missing = [f"contexts.{c}.r_comfort_min_m" for c, v in reqs.items()
               if (v or {}).get("r_comfort_min_m") is None]
    if not reqs or missing:
        raise ValueError(f"{path}: not frozen yet; null: {missing or ['contexts']}")
    return data
