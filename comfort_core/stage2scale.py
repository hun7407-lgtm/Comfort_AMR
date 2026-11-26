"""Stage 2 uniform-scale experiment: shapes, geometry, per-launch outcome,
rates and selection. Plain Python, no ROS / Isaac Sim, so every rule here
is unit-tested (scripts/check_stage2scale.py).

Design authority: docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md.

Shapes
------
Each human state keeps its own engineering template E_raw = (F, S, R)
(configs/common.yaml comfort_space.directional; moving evaluated at the
fixed reference walking speed). It is normalised so its largest extent is
the Stage 1 anchor R_nav_upper (0.90 m): E(1) = 0.90 / max(E_raw) * E_raw,
and ONE scalar alpha multiplies all three axes: E(alpha) = alpha * E(1).
Directions are never shrunk independently. The extents are written into
the condition config directly (aisle_scaling off), because the policy's
own squeeze is floor + s * (x - floor), which is not a multiplication.

Scoring
-------
The only comfort number is comfort_reference.comfort() of the ACTUAL
logged closest centre distance and bearing. alpha, configured extents and
masks never enter it. A launch that does not complete the intended
passage is a joint failure however far it stayed from the person.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

AXES = ("front", "side", "rear")
STATES = ("station_engaged", "stationary", "moving")
EPS = 1e-9

# identification status labels (protocol section 6)
IDENTIFIED_BRACKET = "IDENTIFIED_BRACKET"
FLOOR_LIMITED = "FLOOR_LIMITED_BOUND_NOT_OBSERVED"
OPERATIONAL_ONLY = "OPERATIONAL_MIN_NOT_COMFORT_BOUND"
UNIDENTIFIABLE = "UNIDENTIFIABLE_GEOMETRY"
NO_FEASIBLE = "NO_FEASIBLE_ALPHA"
NO_COMMON = "NO_COMMON_FEASIBLE_ALPHA"
INSUFFICIENT = "INSUFFICIENT_DATA"


# ------------------------------------------------------------------ shapes
def ramp(spec: Dict[str, float], speed: float) -> float:
    """comfort_space.StateDirectionalComfortPolicy._ramp, same clip."""
    v = float(spec["intercept_m"]) + float(spec["gain_s"]) * speed
    return min(max(v, float(spec["min_m"])), float(spec["max_m"]))


def raw_templates(cs: Dict[str, Any], v_ref: float) -> Dict[str, Dict[str, float]]:
    """(F, S, R) per state from comfort_space.directional; moving at v_ref."""
    out = {}
    for st in STATES:
        e = cs["directional"][st]
        if e.get("speed_mapping"):
            out[st] = {a: ramp(e["speed_mapping"][a], v_ref) for a in AXES}
        else:
            out[st] = {a: float(e[f"{a}_m"]) for a in AXES}
    return out


def normalise(raw: Dict[str, Dict[str, float]], anchor: float) -> Dict[str, Dict[str, float]]:
    out = {}
    for st, e in raw.items():
        k = anchor / max(e.values())
        out[st] = {a: e[a] * k for a in AXES}
    return out


def scaled(norm: Dict[str, Dict[str, float]], alpha: float) -> Dict[str, Dict[str, float]]:
    return {st: {a: alpha * e[a] for a in AXES} for st, e in norm.items()}


def min_alpha_without_clip(norm: Dict[str, Dict[str, float]], min_radius: float) -> float:
    return max(min_radius / min(e.values()) for e in norm.values())


def alpha_grid(norm: Dict[str, Dict[str, float]], min_radius: float, step: float = 0.10,
               fine: float = 0.05) -> List[float]:
    """1.00, 0.90, ... down to the smallest unclipped alpha; if that is not
    on the 0.10 grid, its value rounded UP to 0.05 is appended."""
    lo = min_alpha_without_clip(norm, min_radius)
    out, k = [], 0
    while True:
        a = round(1.0 - step * k, 4)
        if a < lo - 1e-9:
            break
        out.append(a)
        k += 1
    last = round(math.ceil(lo / fine - 1e-9) * fine, 4)
    if last < out[-1] - 1e-9:
        out.append(last)
    return out


def bounding_radius(e: Dict[str, float]) -> float:
    """Farthest reach from the person: comfort_core.shapes.from_extents ->
    ComfortShape.bounding_radius, plus a dense numeric check of the offset
    ellipse (the property's closed form is a bound used by rasterisers)."""
    from comfort_core.shapes import from_extents
    sh = from_extents("_", 0.0, 0.0, 0.0, e["front"], e["side"], e["rear"])
    a, b, d = sh.along, sh.across, sh.center_offset_along
    num = max(math.hypot(d + a * math.cos(t), b * math.sin(t))
              for t in (2 * math.pi * i / 3600 for i in range(3600)))
    return max(sh.bounding_radius, num)


def comfort_block(ext: Dict[str, Dict[str, float]], heading_min_speed: float) -> Dict[str, Any]:
    """comfort_space overlay for state_directional with these exact extents.
    The moving template is frozen: gain 0, min = max = value, so a walker's
    realised animation speed cannot change the ratios."""
    r6 = lambda x: round(x, 6)  # noqa: E731
    d = {st: {"front_m": r6(ext[st]["front"]), "side_m": r6(ext[st]["side"]),
              "rear_m": r6(ext[st]["rear"]), "heading_source": "body"}
         for st in ("station_engaged", "stationary")}
    d["moving"] = {"heading_source": "velocity", "heading_min_speed_mps": heading_min_speed,
                   "speed_mapping": {a: {"intercept_m": r6(ext["moving"][a]), "gain_s": 0.0,
                                         "min_m": r6(ext["moving"][a]), "max_m": r6(ext["moving"][a])}
                                     for a in AXES}}
    return {"policy": "state_directional", "directional": d,
            "aisle_scaling": {"enabled": False, "scale": 1.0}}


# ---------------------------------------------------------------- geometry
def static_env(sc: Dict[str, Any], g: Dict[str, Any], width: float, facing: float) -> Dict[str, Any]:
    """One standing / engaged person 'wall_gap' from the +y wall; AMR start-goal
    line 'path_offset' below the person (so the Comfort Space, not the line,
    decides the clearance). Independent of alpha by construction."""
    py = round(width / 2.0 - g["person_wall_gap_m"], 4)
    ly = round(py - g["path_offset_m"], 4)
    return {
        "corridor": {"width": width, "length": g["corridor_length_m"], "center": [0.0, 0.0]},
        "humans": {"a": {"position": [0.0, py], "state": sc["state"], "facing_deg": facing},
                   "b": {"enabled": False}, "c": {"enabled": False}},
        "navigation": {"start": [g["start_x"], ly], "goal": [g["goal_x"], ly]},
    }


def moving_env(m: Dict[str, Any], width: float, lane_gap: float) -> Dict[str, Any]:
    """Oncoming pass in two parallel lanes: walker on human_lane_y walking
    -x, AMR on amr_lane_y driving +x. Lanes are symmetric about lane_centre_y;
    `human_side` (+1 / -1) says which side of it the walker is."""
    c, s = float(m.get("lane_centre_y", 0.0)), float(m.get("human_side", 1.0))
    hy, ay = round(c + s * lane_gap / 2.0, 4), round(c - s * lane_gap / 2.0, 4)
    return {
        "corridor": {"width": width, "length": m["corridor_length_m"], "center": [0.0, 0.0]},
        "humans": {"a": {"position": [m["walker_start_x"], hy], "state": "moving",
                         "route": [[m["walker_end_x"], hy]], "avoid_robot": False},
                   "b": {"enabled": False}, "c": {"enabled": False}},
        "navigation": {"start": [m["amr_start_x"], ay], "goal": [m["amr_goal_x"], ay]},
        "_lanes": {"human_lane_y": hy, "amr_lane_y": ay, "lane_gap_m": lane_gap},
    }


POLICY_KEYS = ("policy", "static_radius", "state_radius", "directional", "aisle_scaling")


def physical_signature(cfg: Dict[str, Any]) -> str:
    """Hash of everything in a merged config except the Comfort Space POLICY
    (policy, sizes, shapes) and the scenario label: walls, people, start/goal,
    timing, Nav2, safety, control mode, costmap buffer / mask value."""
    keep = {k: v for k, v in cfg.items() if k not in ("comfort_space", "scenario")}
    keep["comfort_space"] = {k: v for k, v in (cfg.get("comfort_space") or {}).items() if k not in POLICY_KEYS}
    return hashlib.sha256(json.dumps(keep, sort_keys=True, default=str).encode()).hexdigest()[:16]


# ----------------------------------------------------------- per launch
def passage_outcome(row: Dict[str, Any], target: float) -> Dict[str, Any]:
    """J and its parts from one scored launch.

    row keys: completed (bool: nav class success and goal reached),
    contact (bool), side_ok (bool|None), sector, intended_sector,
    state_ok (bool|None), comfort (float|None).
    Missing evidence never counts in favour.
    """
    completed = bool(row.get("completed"))
    contact = bool(row.get("contact"))
    side_ok = row.get("side_ok") is True
    sector_ok = row.get("sector") is not None and row.get("sector") == row.get("intended_sector")
    state_ok = row.get("state_ok") is True
    encounter_ok = completed and not contact and side_ok and sector_ok and state_ok
    c = row.get("comfort")
    comfort_ok = c is not None and c >= target - EPS
    reasons = []
    if not completed:
        reasons.append("not_completed")
    if contact:
        reasons.append("contact")
    if completed and not side_ok:
        reasons.append("wrong_side")
    if completed and not sector_ok:
        reasons.append("wrong_sector")
    if completed and not state_ok:
        reasons.append("invalid_state")
    if encounter_ok and not comfort_ok:
        reasons.append("comfort_below_target" if c is not None else "comfort_unscorable")
    return {"nav_ok": completed and not contact, "encounter_ok": encounter_ok,
            "comfort_ok": comfort_ok, "J": int(encounter_ok and comfort_ok), "reasons": reasons}


def wilson(k: int, n: int, z: float = 1.959964) -> Tuple[Optional[float], Optional[float]]:
    if n <= 0:
        return None, None
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return round(max(0.0, c - h), 4), round(min(1.0, c + h), 4)


def rates(outcomes: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    n = len(outcomes)
    nav = sum(o["nav_ok"] for o in outcomes)
    enc = sum(o["encounter_ok"] for o in outcomes)
    cok = sum(o["encounter_ok"] and o["comfort_ok"] for o in outcomes)
    j = sum(o["J"] for o in outcomes)
    return {"n": n, "nav_k": nav, "nav_rate": nav / n if n else None, "nav_ci": wilson(nav, n),
            "encounter_k": enc,
            "comfort_k": cok, "comfort_rate_given_completion": cok / enc if enc else None,
            "joint_k": j, "joint_rate": j / n if n else None, "joint_ci": wilson(j, n)}


# ------------------------------------------------------------- selection
def cell_passes(r: Dict[str, Any], joint_min: float, nav_min: float) -> bool:
    return (r["n"] > 0 and r["joint_rate"] is not None and r["joint_rate"] >= joint_min - EPS
            and r["nav_rate"] >= nav_min - EPS)


def select_context(by_alpha: Dict[float, Dict[str, Any]], joint_min: float, nav_min: float,
                   d_median: Dict[float, Optional[float]], min_response_m: float) -> Dict[str, Any]:
    """by_alpha: tested alpha -> rates(); d_median: alpha -> median actual
    centre distance of completed passages. No 'all larger must pass'."""
    alphas = sorted(by_alpha, reverse=True)
    if not alphas:
        return {"status": INSUFFICIENT}
    passing = [a for a in alphas if cell_passes(by_alpha[a], joint_min, nav_min)]
    hi, lo = alphas[0], alphas[-1]
    resp = None
    if d_median.get(hi) is not None and d_median.get(lo) is not None:
        resp = round(d_median[hi] - d_median[lo], 4)
    out = {"tested_alpha": alphas, "passing_alpha": passing, "distance_response_m": resp,
           "min_response_m": min_response_m}
    if resp is None:
        out["status"] = INSUFFICIENT
        out["note"] = "no completed passage at the largest or the smallest alpha: distance response unknown"
        out["alpha_star"] = min(passing) if passing else None
        return out
    if resp < min_response_m - EPS:
        out["status"] = UNIDENTIFIABLE
        out["alpha_star"] = None
        out["operational_min_alpha"] = min(passing) if passing else None
        return out
    if not passing:
        out["status"] = NO_FEASIBLE
        out["alpha_star"] = None
        return out
    best = min(passing)
    out["alpha_star"] = best
    smaller = [a for a in alphas if a < best - EPS]
    if not smaller:
        out["status"] = FLOOR_LIMITED
        return out
    nxt = max(smaller)
    r = by_alpha[nxt]
    comfort_fail = (r["comfort_rate_given_completion"] is not None
                    and r["comfort_rate_given_completion"] < joint_min - EPS)
    out["next_smaller_alpha"] = nxt
    if comfort_fail:
        out["status"] = IDENTIFIED_BRACKET
        out["bracket"] = [nxt, best]
    else:
        out["status"] = OPERATIONAL_ONLY
        out["note"] = (f"alpha {nxt} fails on navigation / encounter validity, not on predicted comfort: "
                       "an operational minimum, not a comfort lower bound")
    return out


COMMON_IDENTIFIED = "COMMON_ALPHA_IDENTIFIED"
COMMON_OPERATIONAL = "COMMON_ALPHA_OPERATIONAL_ONLY"


def select_state(per_context: Dict[str, Dict[float, Dict[str, Any]]], joint_min: float,
                 nav_min: float, context_status: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    """Smallest tested alpha passing EVERY primary context of the state.

    With context_status (context -> select_context status): any context
    UNIDENTIFIABLE_GEOMETRY / INSUFFICIENT_DATA makes the state the same,
    with no alpha*; a common alpha is IDENTIFIED only if every context is
    an identified bracket, otherwise OPERATIONAL_ONLY (not a comfort bound).
    """
    if not per_context:
        return {"status": INSUFFICIENT}
    common = None
    for ctx, by_alpha in per_context.items():
        ok = {a for a, r in by_alpha.items() if cell_passes(r, joint_min, nav_min)}
        common = ok if common is None else (common & ok)
    base = {"contexts": sorted(per_context), "common_alpha": sorted(common or [])}
    st = context_status or {}
    for bad in (UNIDENTIFIABLE, INSUFFICIENT):
        if any(v == bad for v in st.values()):
            return {**base, "status": bad, "alpha_star": None,
                    "operational_min_alpha": min(common) if common else None}
    if not common:
        return {**base, "status": NO_COMMON, "alpha_star": None}
    ident = bool(st) and all(v == IDENTIFIED_BRACKET for v in st.values())
    return {**base, "status": COMMON_IDENTIFIED if ident else COMMON_OPERATIONAL, "alpha_star": min(common)}


def median(xs: Iterable[Optional[float]]) -> Optional[float]:
    v = sorted(x for x in xs if x is not None)
    if not v:
        return None
    n = len(v)
    return round(v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2]), 4)
