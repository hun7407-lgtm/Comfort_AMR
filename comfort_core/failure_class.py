"""Stage 3 failure classes, decided from a trial row.

Pure Python. Reads a trials.csv row (dict of strings); never re-runs anything.

Order matters: the first class that applies wins, so an invalid run is
never counted as a method failure and a robot-robot block never as a
comfort one.

    F  INVALID                 result INVALID / valid False
    A  PASSAGE_INFEASIBLE      comfort gate hold (or a stall in front of a
                               Comfort Space) at a person for whom the
                               selection said R_nav_safe < R_comfort_min:
                               there was no space meeting both, waiting is
                               the intended behaviour
    A2 STATIC_OVER_RESERVATION B2 only: the same, but R_comfort_min <=
                               R_nav_safe < R_B2 -- a space meeting both
                               existed and the single static value was
                               larger than it
    B  SELECTION_FAILURE       B3-O comfort gate hold / comfort stall at a
                               person judged FEASIBLE: the navigation model
                               or the rule was wrong here
    E  ROBOT_ROBOT             stall / Nav2 abort with a fleet AMR within
                               `robot_robot_gap_m` of AMR 1 at the verdict
    C  POST_EVADE_STALL        deadlock_mechanism post_evade_stall
    D  NAV2_FAILURE            nav2_abort / planning_failure / timeout
    U  UNCLASSIFIED_STALL      anything else that did not succeed; reported
                               on its own, never merged into another class

Columns used (all written by comfort_core/metrics.py):
    result, valid, deadlock_mechanism, policy,
    verdict_comfort_ahead       a Comfort Space lay on the plan ahead
    verdict_person              person the gate held at / in front of
    verdict_nearest_fleet_m     AMR 1 to nearest fleet AMR, centre to centre
    selection_status_<person>   FEASIBLE / PASSAGE_INFEASIBLE, from the
    selection_r_nav_safe_<p>,   selector's log for that person
    selection_r_comfort_min_<p>
    r_b2_m                      B2's calibrated radius (B2 rows)
"""

from __future__ import annotations

from typing import Dict, Optional

CLASSES = ("SUCCESS", "INVALID", "PASSAGE_INFEASIBLE", "STATIC_OVER_RESERVATION",
           "SELECTION_FAILURE", "ROBOT_ROBOT", "POST_EVADE_STALL", "NAV2_FAILURE",
           "UNCLASSIFIED_STALL")

ROBOT_ROBOT_GAP_M = 1.5   # centre to centre; two 0.70 m bodies plus both protective fields


def _f(v) -> Optional[float]:
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return None if x != x else x


def _truthy(v) -> bool:
    return str(v).strip().lower() in ("true", "1", "yes")


def classify(row: Dict[str, str], robot_robot_gap_m: float = ROBOT_ROBOT_GAP_M) -> str:
    if row.get("result") == "SUCCESS":
        return "SUCCESS"
    if row.get("result") == "INVALID" or not _truthy(row.get("valid", "True")) \
            or row.get("deadlock_mechanism") == "invalid":
        return "INVALID"
    mech = row.get("deadlock_mechanism") or ""
    comfort_block = mech == "comfort_gate_hold" or (
        mech == "stall" and _truthy(row.get("verdict_comfort_ahead")))
    if comfort_block:
        p = row.get("verdict_person") or ""
        status = row.get(f"selection_status_{p}") or ""
        r_nav = _f(row.get(f"selection_r_nav_safe_{p}"))
        r_min = _f(row.get(f"selection_r_comfort_min_{p}"))
        if status == "PASSAGE_INFEASIBLE" or (r_nav is not None and r_min is not None
                                              and r_nav < r_min):
            return "PASSAGE_INFEASIBLE"
        if row.get("policy") == "static":
            r_b2 = _f(row.get("r_b2_m"))
            if r_nav is not None and r_b2 is not None and r_nav < r_b2:
                return "STATIC_OVER_RESERVATION"
            return "UNCLASSIFIED_STALL" if r_nav is None else "SELECTION_FAILURE"
        return "SELECTION_FAILURE"
    gap = _f(row.get("verdict_nearest_fleet_m"))
    if mech in ("stall", "nav2_abort") and gap is not None and gap <= robot_robot_gap_m:
        return "ROBOT_ROBOT"
    if mech == "post_evade_stall":
        return "POST_EVADE_STALL"
    if mech in ("nav2_abort", "planning_failure", "timeout"):
        return "NAV2_FAILURE"
    return "UNCLASSIFIED_STALL"
