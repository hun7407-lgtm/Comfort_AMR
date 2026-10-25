#!/usr/bin/env python3
"""Run the AMR simulation and keep it running.

This is the long-lived counterpart to spawn_amr.py. It builds the same
scene and then steps forever, so a ROS 2 process in another terminal can
drive the robot over /cmd_vel and read /odom back.

From STEP 6 onward the corridor, humans and Comfort Space are added to
the same scene, so this stays the single Isaac Sim entry point.

Launch it through scripts/run_sim.sh. Stop it with Ctrl+C.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true",
                        help="run without a viewport window")
    parser.add_argument("--config", default="configs/common.yaml",
                        help="path to the common config")
    parser.add_argument("--scenario", default=None,
                        help="b1, b2, b3 or a path to a scenario config")
    parser.add_argument("--max-seconds", type=float, default=0.0,
                        help="stop after this many simulated seconds "
                             "(0 = run until interrupted)")
    parser.add_argument("--record", default=None, metavar="DIR",
                        help="save camera frames of the run into DIR; "
                             "see isaac_sim/utils/recorder.py")
    return parser.parse_args()


args = parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from isaac_sim.environments import people  # noqa: E402
from isaac_sim.utils import fleet as fleet_mod  # noqa: E402
from isaac_sim.utils import scene_setup, towing  # noqa: E402

_stop = False


def _handle_sigint(signum, frame):
    global _stop
    _stop = True
    print("\n  interrupt received, shutting down", flush=True)


def main() -> int:
    cfg = load_config(scenario=args.scenario, common_path=args.config)
    sim_cfg = cfg["simulation"]

    usd_path = resolve_path(cfg["amr"]["usd_path"])
    if not usd_path.is_file():
        print(f"[ERROR] AMR USD not found at {usd_path}", flush=True)
        return 1

    if cfg["ros"]["enable_bridge"]:
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()

    # Must happen before the stage is created; see the docstring on
    # people.enable_extensions.
    if people.enable_extensions(cfg):
        print("  character extensions enabled", flush=True)

    print("\n=== Building scene ===", flush=True)
    create_new_stage()
    _amr_prim, walkers = scene_setup.build_scene(cfg, usd_path)

    simulation_context = SimulationContext(
        physics_dt=float(sim_cfg["physics_dt"]),
        rendering_dt=float(sim_cfg["rendering_dt"]),
        stage_units_in_meters=float(sim_cfg["stage_units_in_meters"]),
    )
    simulation_context.reset()

    # Start the timeline, or nobody walks.
    #
    # omni.anim.people drives a character through the animation graph,
    # and that graph is only evaluated while the timeline is playing.
    # Stepping physics is not enough. Without this the walking people
    # stand on their start squares for the whole run: measured, 200 s
    # with every walker at 0.00-0.02 m/s.
    #
    # The scene builder has always done this (see wait_for_assets in
    # isaac_sim/scripts/build_factory_scene.py). The simulation never
    # did, so every trial so far has been run against people who were
    # standing still, whatever their route said.
    import omni.timeline  # noqa: PLC0415 - needs the app to be up

    omni.timeline.get_timeline_interface().play()

    # The towed cart is built after reset so it can read where the AMR
    # actually settled, not where it was asked to spawn.
    fleet = fleet_mod.Fleet(cfg)
    if len(fleet):
        print(f"  {len(fleet)} fleet AMR(s) routing on the lane network",
              flush=True)

    cart = towing.TowedCart(cfg)
    if cart.enabled:
        print(f"  towing a {cart.length:.2f} x {cart.depth:.2f} m cart on a "
              f"{cart.bar:.2f} m drawbar", flush=True)

    recorder = None
    if args.record:
        from isaac_sim.utils.recorder import FrameRecorder
        label = cfg.get("scenario", {}).get("name", "common").upper()
        recorder = FrameRecorder(cfg, Path(args.record), label)
        print(f"  recording frames to {args.record} from "
              f"{recorder.start:.0f} simulated s", flush=True)

    topics = cfg["ros"]["topics"]
    print("\n=== Simulation running ===", flush=True)
    print(f"  ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '0')}", flush=True)
    print(f"  drive it with : {topics['cmd_vel']}", flush=True)
    print(f"  feedback on   : {topics['odom']}, {topics['joint_states']}, "
          f"{topics['clock']}", flush=True)
    print("  press Ctrl+C to stop\n", flush=True)

    signal.signal(signal.SIGINT, _handle_sigint)

    physics_dt = float(sim_cfg["physics_dt"])
    max_steps = int(args.max_seconds / physics_dt) if args.max_seconds > 0 else 0
    steps = 0

    while simulation_app.is_running() and not _stop:
        # render=True is required even when headless. SimulationContext
        # only calls app.update() on a rendering step, and app.update()
        # is what drives OmniGraph's OnPlaybackTick. With render=False
        # the physics advances but the AMR_Controller graph never fires,
        # so no ROS 2 publisher is ever created and no topic appears.
        simulation_context.step(render=True)
        # Keep the walking humans' colliders under their characters.
        walkers.update(physics_dt)
        cart.update(physics_dt)
        fleet.update(physics_dt)
        if recorder is not None:
            recorder.maybe_capture(simulation_context.current_time)
        steps += 1
        if max_steps and steps >= max_steps:
            print(f"  reached --max-seconds {args.max_seconds}", flush=True)
            break

    print(f"  stopped after {steps * physics_dt:.1f} simulated seconds", flush=True)
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except Exception:
        traceback.print_exc()
        exit_code = 1

    sys.stdout.flush()
    sys.stderr.flush()
    # See spawn_amr.py: Kit's shutdown ends the process itself, so exit
    # here to keep the status code.
    os._exit(exit_code)
