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
    walkers = [k for k in people if humans[k].get("state") == "moving"]

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


def _append(path: Path, row: Dict[str, Any]) -> None:
    """Append one row, refusing to write into a file of another shape."""
    exists = path.is_file()
    if exists:
        with path.open(newline="", encoding="utf-8") as handle:
            header = next(csv.reader(handle), [])
        if header and tuple(header) != FIELDS:
            raise ValueError(
                f"{path} has columns {header[:4]}... and this run writes "
                f"{list(FIELDS)[:4]}...; move the old file aside rather than "
                f"mixing two shapes of row in one file")
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in FIELDS})


def read_rows(path: str) -> List[Dict[str, str]]:
    """Every row of a results file, as written."""
    with resolve_path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
