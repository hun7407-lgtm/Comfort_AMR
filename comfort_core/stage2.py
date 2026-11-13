"""Stage 2: the real-environment human comfort study. Schema and rules.

Pure Python, no ROS. Shared by

    scripts/stage2_make_conditions.py   size ladder from R_nav_max
    scripts/stage2_trial.py             ROS 2 trial recorder on the real AMR
    scripts/stage2_questionnaire.py     response entry after each passing
    scripts/stage2_analysis.py          comfort-size curve, R_comfort_min

so that the file written on the robot and the file read on the desk
cannot disagree about a column.

What Stage 2 measures
---------------------
Stage 1 gives R_nav_max, the largest Comfort Space the aisle can take
before the Comfort Space itself deadlocks the robot. Stage 2 starts the
B3-O geometry AT that size and brings it down one step at a time:

    R_nav_max, R_nav_max - step, R_nav_max - 2 step, ...

A participant stands (or works, or walks -- their state is part of the
condition) while the real AMR passes under each size, and rates how
comfortable the passing felt. The smallest size that still rates as
acceptable is R_comfort_min. Primary item after Neggers et al. (2022,
Int J Soc Robotics, doi 10.1007/s12369-021-00805-6), who had a robot
pass seated and standing participants and asked for comfort on a
7-point scale.

Nothing in this module decides what "acceptable" is: the cutoff is
`stage2.comfort_acceptance_score` in the config and ships as null. It is
set after the pilot, from the data and the reference, not here.

> No real trial has been run with this code. Every coordinate, map and
> participant value is a placeholder until the scenario is decided.
"""

from __future__ import annotations

import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from comfort_core.config import resolve_path
from comfort_core.size_sweep import ladder

# ---------------------------------------------------------------- questionnaire
# The 7-point Likert anchors, as the user will see them. Korean and
# English, the same scale. Only the primary item is fixed; the two
# secondary items are kept to the same 7 points so one answer sheet
# serves all three, and their wording is to be confirmed against the
# reference protocol before the first participant (TODO below).
LIKERT_MIN, LIKERT_MAX = 1, 7
QUESTIONNAIRE_ITEMS: Tuple[Dict[str, Any], ...] = (
    {"key": "comfort_score",
     "primary": True,
     "ko": "로봇이 지나갈 때 얼마나 편안하게 느꼈습니까?",
     "en": "How comfortable did you feel while the robot passed you?",
     "anchors_ko": ("1 = 전혀 편안하지 않음", "7 = 매우 편안함"),
     "anchors_en": ("1 = not at all comfortable", "7 = very comfortable"),
     "source": "Neggers et al. 2022, perceived comfort, 7-point"},
    {"key": "perceived_safety",
     "primary": False,
     "ko": "로봇이 지나갈 때 얼마나 안전하다고 느꼈습니까?",
     "en": "How safe did you feel while the robot passed you?",
     "anchors_ko": ("1 = 전혀 안전하지 않음", "7 = 매우 안전함"),
     "anchors_en": ("1 = not at all safe", "7 = very safe"),
     "source": "TODO confirm wording against the HRI reference before use"},
    {"key": "distance_acceptability",
     "primary": False,
     "ko": "로봇이 지나간 거리는 어땠습니까?",
     "en": "How was the distance at which the robot passed?",
     "anchors_ko": ("1 = 너무 가까움", "4 = 적당함", "7 = 너무 멈"),
     "anchors_en": ("1 = much too close", "4 = just right", "7 = much too far"),
     "source": "TODO confirm wording against the HRI reference before use"},
)

QUESTIONNAIRE_FIELDS: Tuple[str, ...] = (
    "participant_id",      # pseudonymous code (P01 ...), never a name
    "trial_id",            # <session>_<participant>_<order:02d>
    "condition",           # b3o_r1.00
    "comfort_size",        # m, the condition's size
    "comfort_score",       # 1..7, primary
    "perceived_safety",    # 1..7 or ""
    "distance_acceptability",  # 1..7 or ""
    "trial_order",         # 1-based position in this participant's sequence
    "timestamp",
    "notes",
)

# ---------------------------------------------------------------- trial row
# One row per passing, written by scripts/stage2_trial.py on the robot.
# Mirrors the simulation row where the same quantity exists, so the
# Stage 1 / Stage 3 reports can read it with the same names.
TRIAL_FIELDS: Tuple[str, ...] = (
    "stamp",
    "session",
    "participant_id",
    "trial_id",
    "trial_order",
    "condition",
    "policy",
    "comfort_size_m",
    "comfort_scale",
    "config",
    "scenario_file",
    "map",
    "commit",
    # outcome
    "result",              # SUCCESS / DEADLOCK / ABORTED / TIMEOUT / CANCELLED
    "nav2_status",
    "travel_time",
    "travel_distance",
    "planned_path_length",
    "final_distance_to_goal",
    # the person
    "human_key",
    "human_state",         # as configured / observed
    "human_facing_deg",
    "human_speed_mps",     # mean observed, "" if static
    "min_clearance",       # footprint to person, m
    "min_clearance_t_s",
    "passing_side",        # left / right of the person, from their facing
    # applied Comfort Space, m
    "cs_front_m",
    "cs_side_m",
    "cs_rear_m",
    "worst_intrusion",     # into the applied space, m
    "comfort_seconds",     # inside the applied space
    # the layers
    "yield_seconds",
    "safety_stop_seconds",
    "safety_slowdown_seconds",
    "notes",
)

TRAJECTORY_FIELDS: Tuple[str, ...] = (
    "t", "x", "y", "yaw", "v", "human_x", "human_y", "human_state",
    "clearance", "yielding", "safety_action",
)


# ---------------------------------------------------------------- config
def stage2_block(cfg: Dict[str, Any]) -> Dict[str, Any]:
    block = cfg.get("stage2")
    if not isinstance(block, dict):
        raise ValueError("config has no `stage2:` block; see configs/stage2/stage2_real.yaml")
    return block


def size_ladder(cfg: Dict[str, Any]) -> List[float]:
    """R_nav_max, R_nav_max - step, ... from the config. Nothing hard-coded.

    Raises if R_nav_max has not been filled in from Stage 1: a ladder
    with an invented top would be a result with no measurement under it.
    """
    s2 = stage2_block(cfg)
    top = s2.get("r_nav_max_m")
    if top is None:
        raise ValueError(
            "stage2.r_nav_max_m is null. Fill it from the Stage 1 report "
            "(thresholds.json, B3-O R_nav_max) before building the ladder.")
    step = s2.get("comfort_step_m")
    if step is None:
        raise ValueError("stage2.comfort_step_m is null; set the descent step (e.g. 0.1)")
    count = int(s2.get("n_conditions") or 0)
    floor = float(s2.get("size_floor_m") or 0.0)
    if count <= 0:
        raise ValueError("stage2.n_conditions must be >= 1")
    return ladder(float(top), float(step), count, floor)


def missing_placeholders(cfg: Dict[str, Any]) -> List[str]:
    """Every value that must be filled before a real trial may run.

    The scenario and the SLAM map are not decided yet, so the shipped
    config leaves them null. This lists what is still null, by path, so
    scripts/stage2_trial.py can refuse to start with a precise message
    instead of driving a real robot toward a null goal.
    """
    todo: List[str] = []
    nav2 = cfg.get("nav2", {}) or {}
    if not nav2.get("map"):
        todo.append("nav2.map  (SLAM map yaml of the real corridor)")
    nav = cfg.get("navigation", {}) or {}
    for key in ("start", "goal"):
        if nav.get(key) is None:
            todo.append(f"navigation.{key}  ([x, y] in the map frame)")
    humans = {k: v for k, v in (cfg.get("humans", {}) or {}).items()
              if isinstance(v, dict) and "position" in v and v.get("enabled", True)}
    if not humans:
        todo.append("humans.<key>  (at least one participant position/state/facing_deg)")
    for key, h in humans.items():
        if h.get("position") is None or any(p is None for p in h.get("position", [None])):
            todo.append(f"humans.{key}.position")
        if h.get("state") is None:
            todo.append(f"humans.{key}.state  (moving / stationary / station_engaged)")
        if h.get("facing_deg") is None:
            todo.append(f"humans.{key}.facing_deg  (needed: B3-O is directional)")
    try:
        size_ladder(cfg)
    except ValueError as exc:
        todo.append(str(exc))
    return todo


# ---------------------------------------------------------------- files
def session_dir(cfg: Dict[str, Any], session: str) -> Path:
    root = str(stage2_block(cfg).get("results_dir") or "results/stage2")
    return resolve_path(root) / session


def append_row(path: Path, row: Dict[str, Any], fields: Tuple[str, ...]) -> None:
    """Append one row, refusing a file of another shape (as metrics._append)."""
    exists = path.is_file()
    if exists:
        with path.open(newline="", encoding="utf-8") as handle:
            header = next(csv.reader(handle), [])
        if header and tuple(header) != fields:
            raise ValueError(f"{path} has columns {header[:4]}..., this writes "
                             f"{list(fields)[:4]}...; move the old file aside")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", newline="", encoding="utf-8") as handle:
        w = csv.DictWriter(handle, fieldnames=fields)
        if not exists:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in fields})


def read_rows(path: Path) -> List[Dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


# ---------------------------------------------------------------- questionnaire
def validate_response(row: Dict[str, Any]) -> List[str]:
    """Problems with one questionnaire row; empty when it is fine."""
    problems = []
    for key in ("participant_id", "trial_id", "condition", "comfort_size", "trial_order"):
        if row.get(key) in (None, ""):
            problems.append(f"{key} missing")
    for item in QUESTIONNAIRE_ITEMS:
        v = row.get(item["key"])
        if v in (None, ""):
            if item["primary"]:
                problems.append(f"{item['key']} (primary item) missing")
            continue
        try:
            iv = int(v)
        except (TypeError, ValueError):
            problems.append(f"{item['key']} not an integer: {v!r}")
            continue
        if not LIKERT_MIN <= iv <= LIKERT_MAX:
            problems.append(f"{item['key']} out of range {LIKERT_MIN}..{LIKERT_MAX}: {iv}")
    return problems


def make_response(participant_id: str, trial_id: str, condition: str,
                  comfort_size: float, trial_order: int, comfort_score: int,
                  perceived_safety: Optional[int] = None,
                  distance_acceptability: Optional[int] = None,
                  notes: str = "") -> Dict[str, Any]:
    row = {
        "participant_id": participant_id, "trial_id": trial_id, "condition": condition,
        "comfort_size": round(float(comfort_size), 4), "comfort_score": int(comfort_score),
        "perceived_safety": "" if perceived_safety is None else int(perceived_safety),
        "distance_acceptability": ("" if distance_acceptability is None
                                   else int(distance_acceptability)),
        "trial_order": int(trial_order),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "notes": notes,
    }
    problems = validate_response(row)
    if problems:
        raise ValueError("; ".join(problems))
    return row


# ---------------------------------------------------------------- analysis
def _median(xs: List[float]) -> float:
    xs = sorted(xs)
    if not xs:
        return math.nan
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def by_size(rows: Iterable[Dict[str, str]], key: str = "comfort_score"
            ) -> Dict[float, Dict[str, Any]]:
    """Per size: n, mean, median, sd, min, max, distribution over 1..7, participants."""
    groups: Dict[float, List[Tuple[str, int]]] = {}
    for r in rows:
        if r.get(key) in ("", None):
            continue
        groups.setdefault(round(float(r["comfort_size"]), 4), []).append(
            (r["participant_id"], int(r[key])))
    out = {}
    for size, items in sorted(groups.items()):
        vals = [v for _p, v in items]
        m = sum(vals) / len(vals)
        sd = (math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))
              if len(vals) > 1 else math.nan)
        out[size] = {
            "n": len(vals), "mean": m, "median": _median([float(v) for v in vals]),
            "sd": sd, "min": min(vals), "max": max(vals),
            "distribution": {k: sum(1 for v in vals if v == k)
                             for k in range(LIKERT_MIN, LIKERT_MAX + 1)},
            "participants": sorted({p for p, _v in items}),
        }
    return out


def paired_table(rows: Iterable[Dict[str, str]], key: str = "comfort_score"
                 ) -> Dict[str, Dict[float, float]]:
    """participant -> size -> mean score (repeated sizes averaged)."""
    acc: Dict[str, Dict[float, List[int]]] = {}
    for r in rows:
        if r.get(key) in ("", None):
            continue
        acc.setdefault(r["participant_id"], {}).setdefault(
            round(float(r["comfort_size"]), 4), []).append(int(r[key]))
    return {p: {s: sum(v) / len(v) for s, v in sorted(d.items())} for p, d in acc.items()}


def r_comfort_min(stats: Dict[float, Dict[str, Any]], cutoff: Optional[float],
                  statistic: str = "median") -> Dict[str, Any]:
    """The smallest size whose `statistic` score is >= cutoff, if a cutoff is set.

    Returns the verdict and the full size -> score list it rests on.
    With cutoff None it returns no R_comfort_min and says why; that is
    the shipped state, not an error.
    """
    sizes = sorted(stats)
    scores = {s: stats[s][statistic] for s in sizes}
    if cutoff is None:
        return {"R_comfort_min_m": None, "cutoff": None, "statistic": statistic,
                "scores_by_size": scores,
                "note": ("stage2.comfort_acceptance_score is null: no cutoff has been "
                         "decided, so no R_comfort_min is claimed. Decide it from the "
                         "pilot distribution and the reference, then rerun.")}
    acceptable = [s for s in sizes if scores[s] >= cutoff]
    if not acceptable:
        return {"R_comfort_min_m": None, "cutoff": cutoff, "statistic": statistic,
                "scores_by_size": scores,
                "note": f"no swept size reaches {statistic} >= {cutoff}"}
    smallest = min(acceptable)
    below = [s for s in sizes if s < smallest]
    non_monotonic = [s for s in sizes if s > smallest and scores[s] < cutoff]
    return {"R_comfort_min_m": smallest, "cutoff": cutoff, "statistic": statistic,
            "scores_by_size": scores,
            "largest_unacceptable_m": max(below) if below else None,
            "non_monotonic_sizes": non_monotonic,
            "note": (f"smallest size with {statistic} score >= {cutoff}"
                     + (f"; sizes above it that fall below the cutoff: {non_monotonic}"
                        if non_monotonic else ""))}


def breakpoint_estimate(stats: Dict[float, Dict[str, Any]], statistic: str = "mean"
                        ) -> Optional[Dict[str, Any]]:
    """Two-segment piecewise-linear fit over size; the knee is the breakpoint.

    Exploratory: least squares over the per-size means, knee tried at
    every interior size. Needs >= 4 sizes. Reported with its residual
    beside the single-segment fit so a knee that explains nothing is
    visible as such.
    """
    xs = sorted(stats)
    if len(xs) < 4:
        return None
    ys = [stats[x][statistic] for x in xs]

    def fit(px, py):
        n = len(px)
        mx, my = sum(px) / n, sum(py) / n
        sxx = sum((x - mx) ** 2 for x in px)
        slope = sum((x - mx) * (y - my) for x, y in zip(px, py)) / sxx if sxx else 0.0
        icpt = my - slope * mx
        return slope, icpt, sum((y - (slope * x + icpt)) ** 2 for x, y in zip(px, py))

    _s, _i, single = fit(xs, ys)
    best = None
    for k in range(2, len(xs) - 1):
        s1, i1, r1 = fit(xs[:k], ys[:k])
        s2, i2, r2 = fit(xs[k - 1:], ys[k - 1:])
        if best is None or r1 + r2 < best["residual"]:
            best = {"knee_m": xs[k - 1], "residual": r1 + r2, "slope_below": s1,
                    "slope_above": s2}
    best["single_segment_residual"] = single
    best["statistic"] = statistic
    return best
