#!/usr/bin/env python3
"""STEP 3: spawn the AMR in Isaac Sim and verify it stands up.

This is a physics check, nothing more. There is no corridor, no human,
no Comfort Space and no navigation. It answers three questions:

  1. Does the AMR USD load and produce a valid articulation?
  2. Are the two drive joints present and named as the config says?
  3. Does the robot rest on the floor instead of falling or sinking?

Run it through scripts/run_amr_test.sh, which sets the interpreter and
the project path for you. Exit code 0 means all checks passed.
"""

from __future__ import annotations

import argparse
import os
import sys
import traceback
from pathlib import Path

# The project root must be importable before anything from comfort_core.
# This file lives at <root>/isaac_sim/scripts/, hence three parents up.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without a viewport window (faster, no visual inspection)",
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="keep the window open after the checks so you can look around",
    )
    parser.add_argument(
        "--config",
        default="configs/common.yaml",
        help="path to the common config (default: configs/common.yaml)",
    )
    return parser.parse_args()


args = parse_args()

# SimulationApp must be constructed before any other Isaac Sim import.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage, get_current_stage  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from isaac_sim.environments import people  # noqa: E402
from isaac_sim.utils import scene_setup  # noqa: E402

FAILURES: list[str] = []


def check(passed: bool, label: str, detail: str = "") -> None:
    mark = "[ OK ]" if passed else "[FAIL]"
    print(f"  {mark}   {label}")
    if detail:
        print(f"           {detail}")
    if not passed:
        FAILURES.append(label)


def main() -> int:
    cfg = load_config(common_path=args.config)
    amr_cfg = cfg["amr"]
    sim_cfg = cfg["simulation"]

    usd_path = resolve_path(amr_cfg["usd_path"])
    print("\n=== Configuration ===")
    print(f"  project root : {PROJECT_ROOT}")
    print(f"  AMR USD      : {usd_path}")
    if not usd_path.is_file():
        print(f"  [FAIL]   USD not found at {usd_path}")
        return 1

    if cfg["ros"]["enable_bridge"]:
        # Enable before the stage plays: the AMR_Controller ActionGraph
        # contains ROS 2 nodes that fail to initialise without it.
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        print("  ROS 2 bridge : enabled")

    # Must happen before the stage is created; see the docstring on
    # people.enable_extensions.
    if people.enable_extensions(cfg):
        print("  character extensions enabled", flush=True)

    print("\n=== Building scene ===")
    create_new_stage()
    _amr_prim, walkers = scene_setup.build_scene(cfg, usd_path)

    simulation_context = SimulationContext(
        physics_dt=float(sim_cfg["physics_dt"]),
        rendering_dt=float(sim_cfg["rendering_dt"]),
        stage_units_in_meters=float(sim_cfg["stage_units_in_meters"]),
    )

    articulation = SingleArticulation(prim_path=amr_cfg["articulation_path"])
    chassis = SingleRigidPrim(prim_path=amr_cfg["chassis_path"])

    simulation_context.reset()
    articulation.initialize()
    chassis.initialize()

    print("\n=== Articulation ===")
    dof_names = list(articulation.dof_names or [])
    check(len(dof_names) > 0, "articulation initialised",
          f"{articulation.num_dof} DOF: {dof_names}")

    for joint in amr_cfg["drive_joints"]:
        check(joint in dof_names, f"drive joint '{joint}' present")

    print("\n=== Settling under gravity ===")
    check_cfg = cfg["spawn_check"]
    steps = int(float(check_cfg["settle_seconds"]) / float(sim_cfg["physics_dt"]))

    start_pos, _ = chassis.get_world_pose()
    start_pos = np.array(start_pos, dtype=float)
    print(f"  start base_link position : {np.round(start_pos, 4).tolist()}")

    for _ in range(steps):
        # This check only cares about physics, so a non-rendering step is
        # fine and faster. Note that it also means the AMR_Controller
        # ActionGraph does not tick here; ROS 2 output is STEP 4's job and
        # run_amr_sim.py renders every step for exactly that reason.
        simulation_context.step(render=not args.headless)
        walkers.update()

    end_pos, _ = chassis.get_world_pose()
    end_pos = np.array(end_pos, dtype=float)
    print(f"  after {check_cfg['settle_seconds']} s            : "
          f"{np.round(end_pos, 4).tolist()}")

    z_drift = abs(end_pos[2] - start_pos[2])
    xy_drift = float(np.linalg.norm(end_pos[:2] - start_pos[:2]))

    check(z_drift <= float(check_cfg["max_z_drift"]),
          "AMR rests on the ground",
          f"z drift {z_drift:.4f} m, limit {check_cfg['max_z_drift']} m")
    check(xy_drift <= float(check_cfg["max_xy_drift"]),
          "AMR stays put with no command",
          f"xy drift {xy_drift:.4f} m, limit {check_cfg['max_xy_drift']} m")

    print("\n=== Summary ===")
    if FAILURES:
        print(f"  {len(FAILURES)} check(s) failed:")
        for f in FAILURES:
            print(f"    - {f}")
        return 1

    print("  All checks passed. STEP 3 complete.")
    if args.hold and not args.headless:
        print("\n  --hold given: close the Isaac Sim window to exit.")
        while simulation_app.is_running():
            simulation_context.step(render=True)
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except Exception:
        traceback.print_exc()
        exit_code = 1

    print(f"\nRESULT: {'PASS' if exit_code == 0 else 'FAIL'}", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()

    # simulation_app.close() shuts the Kit framework down at the native
    # level and ends the process with status 0, so anything after it is
    # unreachable and the check result would be lost. Exit first instead;
    # the OS reclaims the GPU and file handles either way.
    os._exit(exit_code)
