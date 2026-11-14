"""Stage 2: the real-environment human comfort study. Schema and rules.

Pure Python, no ROS. Shared by

    scripts/stage2_make_conditions.py   participant orders (context x distance)
    scripts/stage2_trial.py             ROS 2 trial recorder on the real AMR
    scripts/stage2_questionnaire.py     response entry after each passing
    scripts/stage2_analysis.py          comfort-size curve, R_comfort_min

so that the file written on the robot and the file read on the desk
cannot disagree about a column.

What Stage 2 measures (revised 2026-11-14)
------------------------------------------
The perceived-comfort curve per Human Context, as independent of the
navigation constraint as possible:

    passing distance d  ->  perceived comfort (7-point)

for each context in `stage2.contexts` (comfort_core/contexts.py:
c1 engaged-rear, c2 stationary-rear, c3 stationary-front, c4
stationary-side). d is the EFFECTIVE corridor-side extent, person
centre to the nearest surface of the AMR -- the same quantity as the
Stage 1 x axis. Each (context, d) is realised by a B3-O condition whose
target size gives that extent in that context
(contexts.target_for_extent), and the distance actually achieved is
recorded beside it (measured_distance_m).

The levels are `stage2.distance_levels_m` and ship as null: the lower end
has to be checked against the real AMR's protective field and passing
speed before any participant is near it. The area should be wide (or the
participant in a niche) so the largest level is reachable without the
navigation limit cutting the curve off (the earlier size ladder that
descended from a Stage 1 navigation limit is superseded and removed).

Per participant: contexts in blocks, block order counterbalanced
(Williams design over contexts), distance levels within a block in a
Williams order, `stage2.trial_repeats` times (`participant_orders`).
Participants are anonymous study codes (`participant_ids`), never names.

The questionnaire's wording and scales are config (stage2.questionnaire;
primary item after Neggers et al. 2022, Int J Soc Robotics, doi
10.1007/s12369-021-00805-6); only the column keys are fixed here. Each
trial carries a status (TRIAL_STATUSES); only NORMAL trials enter the
comfort curve (`usable_ratings`), a participant's stop is counted on its
own. The primary distance is person centre -> nearest AMR surface
(`distance_summary`).

Nothing in this module decides what "acceptable" is: the cutoff is
`stage2.comfort_acceptance_score` and the rules `stage2.analysis.*` in the
config, all null. `comfort_candidates` lists what every possible cutoff
would give; the choice is made after the pilot, not here.

> No real trial has been run with this code. Every coordinate, map and
> participant value is a placeholder until the scenario is decided.
"""

from __future__ import annotations

import csv
import math
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from comfort_core.config import resolve_path

# ---------------------------------------------------------------- questionnaire
# The item TEXT and SCALE live in the config (stage2.questionnaire), so a
# pilot can change wording or scale without touching code. Only the column
# keys are fixed here, so every file has the same shape.
RATING_KEYS: Tuple[str, ...] = ("comfort_score", "perceived_safety", "acceptability")
PRIMARY_KEY = "comfort_score"
COMMENT_KEY = "optional_comment"

QUESTIONNAIRE_FIELDS: Tuple[str, ...] = (
    "participant_id",      # anonymous study code (P01 ...), never a name
    "trial_id",            # <session>_<participant>_<order:02d>
    "condition",           # b3o_r1.00
    "context",             # c1_engaged_rear ... (comfort_core/contexts.py)
    "passing_distance_m",  # planned effective extent d, m (person centre -> AMR surface)
    "repeat",              # 1-based repeat of this (context, d) for the participant
    "comfort_size",        # m, the condition's B3-O target size
    *RATING_KEYS,          # integers on the scale stage2.questionnaire gives
    COMMENT_KEY,           # free text, optional
    "trial_order",         # 1-based position in this participant's sequence
    "questionnaire_version",  # stage2.questionnaire.version at the time
    "timestamp",
)

# ---------------------------------------------------------------- trial row
# One row per passing, written by scripts/stage2_trial.py on the robot.
# Mirrors the simulation row where the same quantity exists, so the
# Stage 1 / Stage 3 reports can read it with the same names.
#
# Trial status (one of TRIAL_STATUSES) is separate from the navigation
# result. Only NORMAL trials carry a comfort rating into the analysis; a
# PARTICIPANT_STOP is stored and counted on its own, never as a failed
# comfort trial.
TRIAL_STATUSES: Tuple[str, ...] = ("NORMAL", "OPERATOR_ABORT", "PARTICIPANT_STOP", "E_STOP",
                                   "NAVIGATION_FAILURE", "INVALID_TRIAL")
OPERATOR_STATUSES: Tuple[str, ...] = ("OPERATOR_ABORT", "PARTICIPANT_STOP", "E_STOP",
                                      "INVALID_TRIAL")

TRIAL_FIELDS: Tuple[str, ...] = (
    "stamp",
    "session",
    "participant_id",
    "trial_id",
    "trial_order",
    "repeat",
    "condition",
    "context",
    "planned_distance_m",  # effective extent the condition was built for
    "policy",
    "comfort_size_m",
    "comfort_scale",
    "config",
    "scenario_file",
    "map",
    "commit",
    "trial_start_time",    # ISO time the goal was sent
    "trial_end_time",      # ISO time the trial ended (result, abort or stop)
    # status
    "trial_status",        # TRIAL_STATUSES
    "stop_reason",         # free text when the status is not NORMAL
    "operator_note",
    # outcome
    "result",              # SUCCESS / ABORTED / TIMEOUT / REJECTED / NO_POSE / INTERRUPTED
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
    "min_clearance",       # AMR footprint to the person's body (radius humans.radius), m
    "actual_min_distance_m",  # PRIMARY: person CENTRE -> nearest AMR surface, m
                              # = min_clearance + humans.radius (Stage 1 x axis)
    "min_center_distance_m",  # raw: person centre -> AMR centre, m (minimum)
    "min_clearance_t_s",
    "passing_side",        # left / right of the person, from their facing
    # the AMR
    "amr_speed_mean_mps",  # while moving (> 0.02 m/s)
    "amr_speed_max_mps",
    "amr_speed_at_min_distance_mps",
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
)

TRAJECTORY_FIELDS: Tuple[str, ...] = (
    "t", "x", "y", "yaw", "v", "human_x", "human_y", "human_state",
    "clearance", "center_distance", "yielding", "safety_action",
)

STATUS_AMENDMENT_FIELDS: Tuple[str, ...] = (
    "trial_id", "trial_status", "stop_reason", "operator_note", "timestamp")


def trial_status(result: str, operator_status: Optional[str] = None) -> str:
    """The trial status from the navigation result and what the operator
    reported. An operator report always wins (a participant who asked to
    stop is PARTICIPANT_STOP whatever Nav2 was doing)."""
    if operator_status:
        if operator_status not in TRIAL_STATUSES:
            raise ValueError(f"unknown trial status {operator_status!r}; one of {TRIAL_STATUSES}")
        return operator_status
    if result == "SUCCESS":
        return "NORMAL"
    if result in ("REJECTED", "NO_POSE"):
        return "INVALID_TRIAL"          # the robot never drove the passing
    if result == "INTERRUPTED":
        return "OPERATOR_ABORT"
    return "NAVIGATION_FAILURE"         # ABORTED / TIMEOUT / anything else


def distance_summary(samples: Iterable[Dict[str, float]], human_radius: float,
                     moving_speed: float = 0.02) -> Dict[str, Any]:
    """Distances and speeds of one passing from the recorder's samples.

    samples: dicts with `clearance` (footprint to body, m), `center_distance`
    (AMR centre to person centre, m) and `v` (AMR speed, m/s). NaN entries
    (no person reading) are skipped. Returns the PRIMARY distance
    actual_min_distance_m = min clearance + body radius (person centre to
    the nearest AMR surface), the raw centre distance, and AMR speeds.
    """
    best = None
    min_center = math.inf
    speeds = []
    for s in samples:
        v = float(s.get("v", math.nan))
        if math.isfinite(v):
            speeds.append(v)
        c = float(s.get("clearance", math.nan))
        if math.isfinite(c) and (best is None or c < best["clearance"]):
            best = {"clearance": c, "v": v, "t": s.get("t")}
        cd = float(s.get("center_distance", math.nan))
        if math.isfinite(cd):
            min_center = min(min_center, cd)
    moving = [v for v in speeds if v > moving_speed]
    return {
        "min_clearance": None if best is None else best["clearance"],
        "actual_min_distance_m": None if best is None else best["clearance"] + float(human_radius),
        "min_clearance_t_s": None if best is None else best["t"],
        "min_center_distance_m": min_center if math.isfinite(min_center) else None,
        "amr_speed_mean_mps": sum(moving) / len(moving) if moving else None,
        "amr_speed_max_mps": max(speeds) if speeds else None,
        "amr_speed_at_min_distance_mps": (None if best is None or not math.isfinite(best["v"])
                                          else best["v"]),
    }


# ---------------------------------------------------------------- config
def stage2_block(cfg: Dict[str, Any]) -> Dict[str, Any]:
    block = cfg.get("stage2")
    if not isinstance(block, dict):
        raise ValueError("config has no `stage2:` block; see configs/stage2/stage2_real.yaml")
    return block


def distance_levels(cfg: Dict[str, Any]) -> List[float]:
    """stage2.distance_levels_m, ascending. Raises while null."""
    levels = stage2_block(cfg).get("distance_levels_m")
    if not levels or any(v is None for v in levels):
        raise ValueError(
            "stage2.distance_levels_m is null. Set the passing distances (m, person "
            "centre to AMR surface) after checking the lowest one against the real "
            "AMR's protective field and passing speed.")
    return sorted(float(v) for v in levels)


def study_contexts(cfg: Dict[str, Any]) -> List[str]:
    """Context keys of the study, in config order. `stage2.contexts` is either
    a list of names or a mapping name -> true/false (enable/disable, e.g.
    C4 after the pilot). Nothing is hard-coded here."""
    from comfort_core.contexts import key_of
    ctx = stage2_block(cfg).get("contexts") or []
    if isinstance(ctx, dict):
        ctx = [c for c, on in ctx.items() if on]
    if not ctx:
        raise ValueError("stage2.contexts enables no context")
    keys = [key_of(c) for c in ctx]
    if len(set(keys)) != len(keys):
        raise ValueError(f"stage2.contexts repeats a context: {ctx}")
    return keys


def participant_ids(cfg: Dict[str, Any], ids: Iterable[str]) -> List[str]:
    """Anonymous study IDs, checked against stage2.participant_id_pattern
    (default P + 2-3 digits). Anything else -- a name, an e-mail -- is
    refused, so no personal data reaches a config or result file."""
    pattern = str(stage2_block(cfg).get("participant_id_pattern") or r"^P[0-9]{2,3}$")
    out = []
    for pid in ids:
        pid = str(pid).strip()
        if not re.fullmatch(pattern, pid):
            raise ValueError(f"participant id {pid!r} does not match {pattern!r}: use the "
                             "anonymous study code only, never a name")
        out.append(pid)
    if len(set(out)) != len(out):
        raise ValueError("duplicate participant id")
    return out


def williams(n: int) -> List[List[int]]:
    """Williams design: rows are orders of 0..n-1 balanced for first-order
    carry-over. n even -> n rows; n odd -> 2n rows (the square and its mirror)."""
    first = [0]
    lo, hi = 1, n - 1
    for i in range(1, n):
        if i % 2:
            first.append(lo)
            lo += 1
        else:
            first.append(hi)
            hi -= 1
    rows = [[(v + r) % n for v in first] for r in range(n)]
    if n % 2:
        rows += [list(reversed(r)) for r in rows]
    return rows


def participant_orders(participants: Any, contexts: List[str], levels: List[float],
                       repeats: int = 1) -> List[Dict[str, Any]]:
    """Trial sequence per participant: context blocks in a Williams order over
    contexts, distance levels in a Williams order within each block, each
    level `repeats` times per block (every repeat a further Williams row).
    The level square advances with block and repeat, so a participant does
    not see the same level order twice in a row.

    participants: a count (P01..PNN) or a list of anonymous IDs; the i-th
    participant takes the i-th row of the Williams squares.
    """
    ids = ([f"P{p + 1:02d}" for p in range(int(participants))]
           if isinstance(participants, int) else list(participants))
    if repeats < 1:
        raise ValueError("repeats must be >= 1")
    out = []
    ctx_rows = williams(len(contexts))
    lvl_rows = williams(len(levels))
    for p, pid in enumerate(ids):
        crow = ctx_rows[p % len(ctx_rows)]
        order = 0
        for b, ci in enumerate(crow):
            for r in range(repeats):
                lrow = lvl_rows[(p + b + r) % len(lvl_rows)]
                for li in lrow:
                    order += 1
                    out.append({"participant_id": pid, "trial_id": f"{pid}_{order:02d}",
                                "block": b + 1, "context": contexts[ci],
                                "passing_distance_m": levels[li], "order_index": order,
                                "trial_order": order, "repeat": r + 1})
    return out


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
        distance_levels(cfg)
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
def questionnaire_items(cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    """stage2.questionnaire.items, checked: each has a key in RATING_KEYS,
    integer scale min < max, and texts. Exactly one is primary and it is
    comfort_score. Wording and scale are the config's, not the code's."""
    q = stage2_block(cfg).get("questionnaire") or {}
    items = q.get("items") or []
    if not items:
        raise ValueError("stage2.questionnaire.items is empty")
    seen = set()
    for it in items:
        k = it.get("key")
        if k not in RATING_KEYS:
            raise ValueError(f"questionnaire item key {k!r} not one of {RATING_KEYS}")
        if k in seen:
            raise ValueError(f"questionnaire item {k!r} twice")
        seen.add(k)
        lo, hi = it.get("scale_min"), it.get("scale_max")
        if not (isinstance(lo, int) and isinstance(hi, int) and lo < hi):
            raise ValueError(f"questionnaire item {k}: scale_min/scale_max must be integers, min < max")
        mid = it.get("midpoint")
        if mid is not None and not lo <= mid <= hi:
            raise ValueError(f"questionnaire item {k}: midpoint outside the scale")
        if not it.get("text"):
            raise ValueError(f"questionnaire item {k}: no text")
    primary = [it["key"] for it in items if it.get("primary")]
    if primary != [PRIMARY_KEY]:
        raise ValueError(f"exactly one primary item, {PRIMARY_KEY!r}; got {primary}")
    return items


def questionnaire_version(cfg: Dict[str, Any]) -> str:
    return str((stage2_block(cfg).get("questionnaire") or {}).get("version") or "")


def validate_response(row: Dict[str, Any], items: List[Dict[str, Any]]) -> List[str]:
    """Problems with one questionnaire row; empty when it is fine."""
    problems = []
    for key in ("participant_id", "trial_id", "condition", "comfort_size", "trial_order"):
        if row.get(key) in (None, ""):
            problems.append(f"{key} missing")
    for item in items:
        v = row.get(item["key"])
        if v in (None, ""):
            if item.get("primary") or item.get("required"):
                problems.append(f"{item['key']} (required) missing")
            continue
        try:
            iv = int(v)
        except (TypeError, ValueError):
            problems.append(f"{item['key']} not an integer: {v!r}")
            continue
        if not item["scale_min"] <= iv <= item["scale_max"]:
            problems.append(f"{item['key']} out of range {item['scale_min']}..{item['scale_max']}: {iv}")
    return problems


def make_response(cfg: Dict[str, Any], participant_id: str, trial_id: str, condition: str,
                  comfort_size: float, trial_order: int, ratings: Dict[str, Optional[int]],
                  comment: str = "", context: str = "",
                  passing_distance_m: Optional[float] = None, repeat: int = 1) -> Dict[str, Any]:
    items = questionnaire_items(cfg)
    participant_ids(cfg, [participant_id])
    row = {
        "participant_id": participant_id, "trial_id": trial_id, "condition": condition,
        "context": context,
        "passing_distance_m": "" if passing_distance_m is None else round(float(passing_distance_m), 4),
        "repeat": int(repeat),
        "comfort_size": round(float(comfort_size), 4),
        **{k: ("" if ratings.get(k) is None else int(ratings[k])) for k in RATING_KEYS},
        COMMENT_KEY: comment,
        "trial_order": int(trial_order),
        "questionnaire_version": questionnaire_version(cfg),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    problems = validate_response(row, items)
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


def by_size(rows: Iterable[Dict[str, str]], key: str = "comfort_score",
            scale: Tuple[int, int] = (1, 7)) -> Dict[float, Dict[str, Any]]:
    """Per size: n, mean, median, sd, min, max, distribution over the scale, participants."""
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
                             for k in range(scale[0], scale[1] + 1)},
            "participants": sorted({p for p, _v in items}),
        }
    return out


def by_context_distance(rows: Iterable[Dict[str, str]], key: str = "comfort_score",
                        scale: Tuple[int, int] = (1, 7)) -> Dict[str, Dict[float, Dict[str, Any]]]:
    """context -> passing distance -> the same statistics as by_size.
    The comfort curve per Human Context; raw rows are never altered."""
    groups: Dict[str, List[Dict[str, str]]] = {}
    for r in rows:
        if r.get("passing_distance_m") in ("", None):
            continue
        groups.setdefault(r.get("context") or "?", []).append(
            {**r, "comfort_size": r["passing_distance_m"]})
    return {c: by_size(rs, key, scale) for c, rs in sorted(groups.items())}


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


def usable_ratings(questionnaire: Iterable[Dict[str, str]], trials: Iterable[Dict[str, str]],
                   amendments: Iterable[Dict[str, str]] = ()) -> Tuple[List[Dict[str, str]],
                                                                       Dict[str, List[str]]]:
    """Questionnaire rows of NORMAL trials only.

    The trial status comes from trials.csv, overridden by the latest row
    of status_amendments.csv for that trial (scripts/stage2_mark_trial.py).
    A rating whose trial has no trials.csv row is kept (the questionnaire
    can be run on its own) but listed under "no_trial_row". Every other
    status is returned by trial_id, never mixed into the comfort curve:
    a participant who asked to stop is not a low comfort score.
    """
    status = {t.get("trial_id"): t.get("trial_status") or "" for t in trials}
    for a in amendments:
        status[a.get("trial_id")] = a.get("trial_status") or ""
    kept, excluded = [], {}
    for r in questionnaire:
        st = status.get(r.get("trial_id"))
        if st is None:
            excluded.setdefault("no_trial_row", []).append(r.get("trial_id"))
            kept.append(r)
        elif st == "NORMAL":
            kept.append(r)
        else:
            excluded.setdefault(st or "UNKNOWN", []).append(r.get("trial_id"))
    return kept, excluded


def comfort_candidates(stats: Dict[float, Dict[str, Any]], scale: Tuple[int, int],
                       statistic: str = "median", midpoint: Optional[float] = None
                       ) -> Dict[str, Any]:
    """CANDIDATES for R_comfort_min and R_pref from one context's curve.
    Nothing here is the final definition (stage2.analysis.*_rule, null
    until the pilot); it lays out what each possible rule would give.

    r_comfort_min_by_cutoff  for every cutoff c on the scale: the smallest
                             distance whose statistic is >= c, and whether
                             a larger distance falls below c again
    r_pref_argmax            the distance with the highest statistic
                             (ties -> the smallest such distance)
    r_pref_midpoint          for a bipolar item (too close .. just right ..
                             too far): the distances whose statistic equals
                             the midpoint, and the linear crossing of it
    """
    ds = sorted(stats)
    vals = {d: stats[d][statistic] for d in ds}
    by_cut = {}
    for c in range(scale[0], scale[1] + 1):
        ok = [d for d in ds if vals[d] >= c]
        first = min(ok) if ok else None
        by_cut[str(c)] = {"distance_m": first,
                          "non_monotonic_above": [d for d in ds if first is not None
                                                  and d > first and vals[d] < c]}
    best = max(vals.values()) if vals else None
    out = {"statistic": statistic, "distances_m": ds, "values": vals,
           "r_comfort_min_by_cutoff": by_cut,
           "r_pref_argmax": None if best is None else min(d for d in ds if vals[d] == best)}
    if midpoint is not None:
        exact = [d for d in ds if vals[d] == midpoint]
        cross = None
        for a, b in zip(ds, ds[1:]):
            va, vb = vals[a] - midpoint, vals[b] - midpoint
            if va * vb < 0:
                cross = a + (b - a) * (-va) / (vb - va)
                break
        out["r_pref_midpoint"] = {"midpoint": midpoint, "at_midpoint_m": exact,
                                  "linear_crossing_m": cross}
    return out
