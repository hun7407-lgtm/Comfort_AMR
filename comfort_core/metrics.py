"""One trial, one row.

Pure Python. No ROS, no omni: a trial hands this module a plain dict and
it writes the files.

Why this exists
---------------
Everything a trial measures was being printed and then lost. A study
needs the numbers in a file that can be counted, grouped and plotted
months later, and it needs to be obvious which configuration produced
them -- "B3 was better" is worth nothing without the radii, the layout
and the map that B3 ran on.

What is written, per trial
--------------------------
    results/trials/<version>/trials.csv       one row, appended
    results/trials/<version>/<stamp>.json     everything, including the
                                              per-person numbers that do
                                              not fit a flat row

The CSV columns are fixed in `FIELDS` below. If an existing file's
header does not match, this refuses to append to it rather than writing
rows whose columns have quietly shifted by one -- a results file that is
wrong in a way nobody notices is worse than no results file.

Trials that were discarded are written too, with `valid` false and the
reason. A run that vanishes from the record because it diverged looks
exactly like a run that was never done.
"""

from __future__ import annotations

import csv
import json
import math
import subprocess
from datetime import datetime
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from comfort_core.config import resolve_path

# The row. Order is the file's column order and must not be reordered
# once results exist; new columns go on the end.
FIELDS: Tuple[str, ...] = (
    "stamp",
    "scenario",
    "policy",
    "config",
    "map",
    # -- outcome ---------------------------------------------------
    "result",
    "valid",
    "invalid_reason",
    "goal_reached",
    "planning_success",
    "deadlock",
    "deadlock_cause",
    # -- cost of getting there -------------------------------------
    "travel_time",
    "travel_distance",
    "final_distance_to_goal",
    "yield_seconds",
    "waited_for_humans",
    "evade_seconds",
    # -- safety ----------------------------------------------------
    "safety_seconds",
    "safety_polygon",
    "contact_seconds",
    "contact_robot_seconds",
    "contact_person_seconds",
    "min_clearance",
    "min_clearance_who",
    # -- comfort ---------------------------------------------------
    "comfort_reference",
    "worst_intrusion",
    "worst_intrusion_who",
    "comfort_seconds_total",
    "people_intruded",
    # -- state estimation ------------------------------------------
    "state_rule",
    "state_agreement",
    "state_readings",
    # -- what it ran on --------------------------------------------
    "people",
    "walkers",
    "start_x",
    "start_y",
    "goal_x",
    "goal_y",
    "max_trial_time",
    "static_radius",
    "radius_moving",
    "radius_stationary",
    "radius_engaged",
    "commit",
    # -- times the gate had to get the robot unstuck ---------------
    # Faults worked around, not policy. A cell that needs these often
    # is saying something about the layout rather than about Comfort
    # Space, and a trial that only finished because it reversed out of
    # a corner should not read as a clean run.
    "backoffs",
    "off_lane_recoveries",
    # -- the safety layer, split by what it actually did -----------
    # `safety_seconds` above is the sum and keeps its meaning, so every
    # file written before these existed still reads the same. Only a
    # STOP refuses the command; a SLOWDOWN scales it and an APPROACH
    # only warns, so only the first can explain a robot that is not
    # moving. Counting all three in one number is why the 261023
    # comparison could not separate a B2 Comfort Space deadlock from a
    # protective-field stop.
    "safety_stop_only_seconds",
    "safety_slowdown_seconds",
    "safety_approach_seconds",
    # -- which condition this row belongs to -----------------------
    # Written by the scenario generator (comfort_core/scenario.py) and
    # carried through so a results file can be grouped by factor
    # without going back to the configs. Empty for a hand-written
    # config, which is every trial run before the generator existed.
    "comfort_shape",
    "scenario_id",
    "seed",
    "human_count",
    "amr_count",
    "state_provider",
    "north_exchange",
    "south_exchange",
    "mission_start_slot",
    "mission_goal_slot",
    "mission_distance",
    # The machine's one-minute load when the trial finished.
    #
    # A trial measures a policy only if the simulation ran at a sane
    # rate. Twenty-nine orphaned detector processes were once found
    # running at 90% of a core each on a 24-core machine; the simulator
    # dropped to an eighth of real time and every control loop in it
    # missed its deadlines. Nothing in the result said so. Recording
    # the load means a starved trial can be spotted afterwards instead
    # of being averaged in.
    "host_load",
)


def _git_commit() -> str:
    """The commit the trial ran at, or "" outside a checkout.

    Worth a subprocess call once per trial: it is the only thing that
    says which version of the policy produced a number.
    """
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=str(Path(__file__).resolve().parents[1]),
                             capture_output=True, text=True, timeout=5)
        if out.returncode != 0:
            return ""
        dirty = subprocess.run(["git", "status", "--porcelain"],
                               cwd=str(Path(__file__).resolve().parents[1]),
                               capture_output=True, text=True, timeout=5)
        mark = "+" if dirty.stdout.strip() else ""
        return out.stdout.strip() + mark
    except Exception:  # noqa: BLE001 - no git is a normal case
        return ""


def _worst(items: Dict[str, Any], index: int,
           pick_min: bool) -> Tuple[Optional[float], str]:
    """The extreme value in a {key: tuple} map, and whose it was."""
    best, who = None, ""
    for key, value in items.items():
        v = float(value[index])
        if not math.isfinite(v):
            continue
        if best is None or (v < best if pick_min else v > best):
            best, who = v, key
    return best, who


def _round(value: Any, places: int = 3) -> Any:
    if isinstance(value, float) and math.isfinite(value):
        return round(value, places)
    return value


def row_for(cfg: Dict[str, Any], result: Dict[str, Any],
            config_path: str, map_path: str = "") -> Dict[str, Any]:
    """Flatten one trial into the CSV row."""
    nav = cfg.get("navigation", {})
    cs = cfg.get("comfort_space", {})
    by_state = cs.get("state_radius", {})
    humans = cfg.get("humans", {})
    people = [k for k, v in humans.items()
              if isinstance(v, dict) and "position" in v
              and v.get("enabled", True)]
    # Anybody who moves at all, which is not the same as anybody
    # DECLARED moving. `state: moving` is somebody already walking when
    # the trial starts; `walk:` is somebody who leaves a machine part
    # way through, and in the U bay that is how every walker is
    # written. Counting only the first read 0 walkers in every trial of
    # the 261023 comparison while two people crossed the aisle.
    walkers = [k for k in people
               if humans[k].get("state") == "moving" or humans[k].get("walk")]

    clearance, clearance_who = _worst(result.get("min_clearance", {}), 0, True)
    intrusion, intrusion_who = _worst(result.get("comfort_worst", {}), 0, False)
    # Nobody's space was entered: naming the person whose number happened
    # to sort first would read as "this one was intruded on by 0 m".
    if not intrusion:
        intrusion_who = ""
    inside = result.get("comfort_seconds", {})

    confusion = result.get("state_confusion", {})
    readings = sum(confusion.values())
    agreed = sum(n for key, n in confusion.items() if key[0] == key[1])

    start = result.get("start_pose", (float("nan"),) * 3)
    goal = nav.get("goal", [float("nan"), float("nan")])

    return {
        "stamp": datetime.now().isoformat(timespec="seconds"),
        "scenario": cfg.get("scenario", {}).get("name", "common"),
        "policy": cs.get("policy", ""),
        "config": config_path,
        "map": map_path,

        "result": result.get("result", ""),
        # A diverged simulation is recorded and marked, never dropped.
        "valid": not result.get("invalid_reason"),
        "invalid_reason": result.get("invalid_reason", ""),
        "goal_reached": result.get("goal_reached", False),
        "planning_success": result.get("planning_success", False),
        "deadlock": result.get("deadlock_detected", False),
        "deadlock_cause": result.get("deadlock_cause", ""),

        "travel_time": _round(result.get("travel_time", 0.0), 2),
        "travel_distance": _round(result.get("travel_distance", 0.0), 2),
        "final_distance_to_goal": _round(
            result.get("final_distance_to_goal", float("nan"))),
        "yield_seconds": _round(result.get("yield_seconds", 0.0), 2),
        "waited_for_humans": _round(result.get("waited_for_humans", 0.0), 2),
        "evade_seconds": _round(result.get("evade_seconds", 0.0), 2),

        "host_load": _round(os.getloadavg()[0], 1),
        "backoffs": int(result.get("backoffs", 0)),
        "off_lane_recoveries": int(result.get("off_lane_recoveries", 0)),

        "safety_seconds": _round(result.get("safety_stop_seconds", 0.0), 2),
        "safety_stop_only_seconds": _round(
            result.get("safety_stop_only_seconds", 0.0), 2),
        "safety_slowdown_seconds": _round(
            result.get("safety_slowdown_seconds", 0.0), 2),
        "safety_approach_seconds": _round(
            result.get("safety_approach_seconds", 0.0), 2),
        "safety_polygon": result.get("safety_polygon", ""),
        "contact_seconds": _round(result.get("contact_seconds", 0.0), 2),
        # The one that decides whether a baseline is standard-compliant.
        "contact_robot_seconds": _round(
            result.get("contact_robot_seconds", 0.0), 2),
        "contact_person_seconds": _round(
            result.get("contact_person_seconds", 0.0), 2),
        "min_clearance": _round(clearance),
        "min_clearance_who": clearance_who,

        "comfort_reference": result.get("comfort_reference", ""),
        "worst_intrusion": _round(intrusion),
        "worst_intrusion_who": intrusion_who,
        "comfort_seconds_total": _round(sum(inside.values()), 2),
        "people_intruded": sum(1 for v in inside.values() if v > 0.0),

        "state_rule": result.get("state_rule", ""),
        "state_agreement": _round(agreed / readings, 4) if readings else "",
        "state_readings": readings,

        "people": len(people),
        "walkers": len(walkers),
        "start_x": _round(start[0], 2),
        "start_y": _round(start[1], 2),
        "goal_x": _round(float(goal[0]), 2),
        "goal_y": _round(float(goal[1]), 2),
        "max_trial_time": nav.get("max_trial_time", ""),
        "static_radius": cs.get("static_radius", ""),
        "radius_moving": by_state.get("moving", ""),
        "radius_stationary": by_state.get("stationary", ""),
        "radius_engaged": by_state.get("station_engaged", ""),
        "commit": _git_commit(),

        # Round or stretched, which is the only difference between B3-R
        # and B3 and is otherwise invisible: both report the policy
        # `state_adaptive`.
        "comfort_shape": (cs.get("shape", "") if cs.get("policy")
                          == "state_adaptive" else "circle"),
        **_scenario_columns(cfg),
    }


def _scenario_columns(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """The generated scenario's own description of itself.

    Empty strings for a hand-written config: the columns exist for
    every row, and a blank one means "this trial was not generated",
    which is true of everything run before comfort_core/scenario.py.
    """
    meta = cfg.get("scenario_meta") or {}
    exchanges = {str(e.get("side")): e for e in meta.get("exchanges", [])}
    mission = meta.get("mission") or {}

    def pair(side: str) -> str:
        item = exchanges.get(side)
        if not item:
            return ""
        return f"{item.get('belt', '')}->{item.get('edge', '')}"

    return {
        "scenario_id": meta.get("scenario_id", ""),
        "seed": meta.get("seed", ""),
        "human_count": meta.get("human_count", ""),
        "amr_count": meta.get("amr_count", ""),
        "state_provider": meta.get("state_provider", ""),
        "north_exchange": pair("north"),
        "south_exchange": pair("south"),
        "mission_start_slot": mission.get("start_slot", ""),
        "mission_goal_slot": mission.get("goal_slot", ""),
        "mission_distance": mission.get("distance", ""),
    }


def _jsonable(value: Any) -> Any:
    """Tuple keys and tuple values into something JSON can hold."""
    if isinstance(value, dict):
        return {("%s>%s" % k if isinstance(k, tuple) else str(k)): _jsonable(v)
                for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def record_trial(cfg: Dict[str, Any], result: Dict[str, Any],
                 config_path: str, map_path: str = "",
                 out_dir: Optional[str] = None) -> Tuple[Path, Path]:
    """Write one trial's row and its detail file. Returns both paths."""
    version = str(cfg.get("recording", {}).get("version", "unversioned"))
    folder = resolve_path(out_dir or f"results/trials/{version}")
    folder.mkdir(parents=True, exist_ok=True)

    row = row_for(cfg, result, config_path, map_path)
    csv_path = folder / "trials.csv"
    _append(csv_path, row)

    stamp = row["stamp"].replace(":", "").replace("-", "")
    name = f"{stamp}_{row['scenario']}_{Path(config_path).stem}.json"
    json_path = folder / name
    json_path.write_text(json.dumps({
        "row": row,
        # Everything the row had to flatten away: who was closest, how
        # deep into whose Comfort Space, the whole confusion matrix.
        "min_clearance": _jsonable(result.get("min_clearance", {})),
        "comfort_worst": _jsonable(result.get("comfort_worst", {})),
        "comfort_seconds": _jsonable(result.get("comfort_seconds", {})),
        "state_confusion": _jsonable(result.get("state_confusion", {})),
        "final_pose": _jsonable(result.get("final_pose", ())),
    }, indent=2), encoding="utf-8")
    return (csv_path, json_path)


def _append(path: Path, row: Dict[str, Any],
            fields: Tuple[str, ...] = FIELDS) -> None:
    """Append one row, refusing to write into a file of another shape."""
    exists = path.is_file()
    if exists:
        with path.open(newline="", encoding="utf-8") as handle:
            header = next(csv.reader(handle), [])
        if header and tuple(header) != fields:
            raise ValueError(
                f"{path} has columns {header[:4]}... and this run writes "
                f"{list(fields)[:4]}...; move the old file aside rather than "
                f"mixing two shapes of row in one file")
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        if not exists:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in fields})


# ---------------------------------------------------------------------------
# The other AMRs. One row per vehicle per trial, in its own file next
# to trials.csv: the same yardstick (reference Comfort Space,
# clearance, contact, safety layer) applied to a1, a2, ... under the
# same policy. They are measured on the same footing as AMR1; the file
# is separate only because the columns differ -- AMR1 has a mission
# with one goal, these have a job list and other robots to meet -- and
# because trials.csv's columns must not move under results that exist.
#
# Joined to trials.csv by (scenario, config, stamp within the minute):
# both rows are written at the end of the same run.
# ---------------------------------------------------------------------------
FLEET_FIELDS: Tuple[str, ...] = (
    "stamp",
    "scenario",
    "policy",
    "config",
    "robot",
    # -- outcome, the vehicle's own ---------------------------------
    # SUCCESS   every leg it attempted in the trial was reached or was
    #           still being driven when the trial ended
    # DEADLOCK  the same rule as AMR1's (navigation.deadlock): a goal
    #           active, no progress towards it for `duration` seconds
    # ABORTED   Nav2 gave a leg up twice
    "result",
    "deadlock_cause",
    # -- the job list ----------------------------------------------
    "legs_total",
    "legs_done",
    "legs_skipped",
    "set_off_at_s",            # trial time the first goal went out
    "first_leg_s",             # trial time the first leg was reached
    "travel_distance_m",
    "observed_s",              # how long this row watched the vehicle
    # -- people ----------------------------------------------------
    "min_clearance_m",
    "min_clearance_with",
    "contact_s",
    "contact_robot_s",
    "contact_person_s",
    "comfort_reference",
    "worst_intrusion_m",
    "worst_intrusion_k",
    "worst_intrusion_into",
    "comfort_seconds",         # summed over people, k < 1
    # -- the safety layer ------------------------------------------
    "safety_stop_s",
    "safety_slowdown_s",
    "safety_approach_s",
    # -- other robots ----------------------------------------------
    "min_gap_amr1_m",          # centre-to-centre
    "min_gap_fleet_m",
    "min_gap_fleet_with",
    "git_commit",
    # Added after the first rows: WHEN the closest approach and the
    # deepest intrusion happened, trial time, so a row can be lined up
    # with the costmap video.
    "min_clearance_t_s",
    "worst_intrusion_t_s",
    # Where the robot was, seen from the person, at the deepest
    # intrusion: front / side / rear by the shape's heading.
    "worst_intrusion_bearing",
)


def record_fleet_trial(cfg: Dict[str, Any], rows: List[Dict[str, Any]],
                       config_path: str, scenario: str,
                       out_dir: Optional[str] = None) -> Path:
    """Append one row per fleet vehicle. Returns the csv path."""
    version = str(cfg.get("recording", {}).get("version", "unversioned"))
    folder = resolve_path(out_dir or f"results/trials/{version}")
    folder.mkdir(parents=True, exist_ok=True)
    csv_path = folder / "fleet_trials.csv"
    stamp = datetime.now().isoformat(timespec="seconds")
    commit = _git_commit()
    policy = str(cfg.get("comfort_space", {}).get("policy", ""))
    for row in rows:
        full = {"stamp": stamp, "scenario": scenario, "policy": policy,
                "config": config_path, "git_commit": commit}
        full.update({k: _round(v) if isinstance(v, float) else v
                     for k, v in row.items()})
        _append(csv_path, full, FLEET_FIELDS)
    return csv_path


def read_rows(path: str) -> List[Dict[str, str]]:
    """Every row of a results file, as written."""
    with resolve_path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
