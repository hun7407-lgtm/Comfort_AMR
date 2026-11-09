"""Photograph the scene from several viewpoints.

    ~/isaacsim/python.sh isaac_sim/scripts/scene_shots.py
    ~/isaacsim/python.sh isaac_sim/scripts/scene_shots.py \
        --config configs/factory_u.yaml --out results/notion/261027

Builds the scene exactly as a trial does -- same config, same people,
same robot at its start pose -- and renders stills from a fixed set of
cameras. The floor markers are forced ON here: this is a picture, not
a measurement.

Nothing is simulated. The physics step runs a few frames so the robot
settles onto its wheels and the renderer has something warmed up to
draw, then each camera is rendered in turn.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/factory_u.yaml")
    parser.add_argument("--scenario", default="b3")
    parser.add_argument("--out", default="results/notion/261027")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--no-markers", action="store_true",
                        help="leave the start/goal plates out (a clean "
                             "plan for scenario_figure.py to draw on)")
    parser.add_argument("--settle", type=int, default=240,
                        help="physics steps before the first shot")
    return parser.parse_args()


# Each view is (name, camera position, what it looks at, focal length).
#
# The bay is 24 x 16 m, the north aisle runs at y = +4.215 and carries
# the robot's whole route, and the two cart blocks the trial starts and
# ends in are on the north side.
# The bay has a roof. From outside it is the only thing a camera sees,
# so the overhead views hide it; the lights are a separate group
# (/World/Factory/Lights) and stay on, so hiding it costs nothing but
# the lid. Views from inside keep it, because it is what a bay looks
# like from the floor.
CEILING = "/World/Factory/Ceiling"


def views(cfg):
    nav = cfg["navigation"]
    sx, sy = (float(v) for v in nav["start"][:2])
    gx, gy = (float(v) for v in nav["goal"][:2])
    return [
        # The plan view.
        #
        # `look_at` straight down leaves the roll undefined and Isaac
        # resolves it with +x up the frame, so the bay came out on its
        # side and both ends were cropped. Passing rotation=(0,0,0)
        # instead rendered nothing at all. What works is to shoot it
        # portrait and turn the image a quarter turn on the way out:
        # the frame then reads +x right, +y up, like the drawing the
        # layout comes from. See `plan` below.
        # In a portrait render product the 36 mm aperture maps to the
        # LONG side, so the lens has to be chosen for the bay's 24 m
        # length, not its 16 m width: 34 m up at 40 mm covers 30.6 m
        # that way and 17.2 m across, and the building is 24 by 16.
        # At 60 mm the ends were cut off.
        ("01_top",
         (0.0, 0.0, 34.0), (0.0, 0.0, 0.0), 40.0, True),
        ("02_iso_southeast",
         (16.0, -14.0, 11.0), (0.0, 0.0, 1.0), 20.0, True),
        ("03_iso_northwest",
         (-16.0, 13.0, 11.0), (0.0, 0.5, 1.0), 20.0, True),
        ("04_north_aisle",
         (11.5, 4.21, 1.8), (-10.0, 4.21, 0.9), 22.0, False),
        ("05_south_aisle",
         (-11.5, -4.21, 1.8), (10.0, -4.21, 0.9), 22.0, False),
        ("06_start_slot",
         (sx + 3.2, sy - 3.4, 2.6), (sx, sy, 0.3), 28.0, False),
        ("07_goal_slot",
         (gx - 3.2, gy - 3.6, 2.6), (gx, gy, 0.3), 28.0, False),
        ("08_route_from_start",
         (sx + 1.6, sy - 1.2, 1.5), (gx, gy + 0.6, 0.6), 18.0, False),
        # Inside the south wall, not outside it: the bay ends at
        # y = -8.0 and a camera beyond that photographs brickwork.
        ("09_belt_and_workers",
         (0.0, -7.2, 3.0), (0.0, -1.5, 1.0), 22.0, False),
        ("10_cart_blocks",
         (-5.0, 1.0, 5.5), (-5.2, 6.0, 0.4), 24.0, True),
        # The fleet's west dock, and one worker's Comfort Space close
        # enough to judge the drawing.
        ("11_fleet_dock_west",
         (-8.6, -2.2, 1.9), (-10.5, 0.0, 0.3), 30.0, False),
        ("12_comfort_space_closeup",
         (-6.0, 7.6, 3.6), (-2.6, 3.2, 0.2), 26.0, False),
    ]


# Orthographic plan: 30 m across the 1920 px frame, centred on the
# bay's origin, +x right and +y up. 64 px per metre. The 16 m bay fits
# the 1080 px height with a margin.
ORTHO_WIDTH_M = 30.0
ORTHO_CENTRE = (0.0, 0.0)
ORTHO_HEIGHT_Z = 30.0


def _plan_ortho(stage, world, rep, np, Image, out, label, roof, args):
    """Render the floor from straight above with an orthographic camera."""
    from pxr import Gf, UsdGeom
    if roof and roof.IsValid():
        UsdGeom.Imageable(roof).MakeInvisible()
    cam_path = "/World/PlanCamera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
    # Apertures are in tenths of a stage unit; the stage is in metres.
    aspect = args.height / args.width
    cam.CreateHorizontalApertureAttr(ORTHO_WIDTH_M * 10.0)
    cam.CreateVerticalApertureAttr(ORTHO_WIDTH_M * aspect * 10.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))
    # An unrotated camera looks down its own -z, which is world -z,
    # with image-up along world +y: exactly the map orientation.
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(ORTHO_CENTRE[0], ORTHO_CENTRE[1], ORTHO_HEIGHT_Z))
    product = rep.create.render_product(cam_path, (args.width, args.height))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach(product)
    frame = None
    for _ in range(12):
        world.step(render=True)
        data = annot.get_data()
        if data is not None and np.asarray(data).size:
            frame = np.asarray(data)[..., :3]
    annot.detach(product)
    product.destroy()
    if frame is None:
        print("  [WARN] plan_ortho: no frame came back", flush=True)
        return None
    path = out / f"scene_{label}_00_plan_ortho.png"
    Image.fromarray(np.array(frame, copy=True)).save(path)
    return path


def main() -> int:
    args = parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from PIL import Image
    import omni.replicator.core as rep
    from isaacsim.core.api import World

    from comfort_core import load_config, resolve_path
    from isaac_sim.utils import scene_setup

    cfg = load_config(scenario=args.scenario,
                      common_path=str(PROJECT_ROOT / args.config))
    # A picture, not a measurement: see configs/common.yaml markers.
    cfg.setdefault("markers", {})["enabled"] = not args.no_markers

    # The scene build publishes the humans' TF frames through the ROS 2
    # bridge, so the extension has to be on even though a screenshot
    # uses none of it. Without it build_scene dies on an unrecognised
    # node type half way through.
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(10):
        app.update()

    from isaac_sim.environments import people as people_mod
    if people_mod.enable_extensions(cfg):
        for _ in range(10):
            app.update()

    # A FRESH stage, after the character extensions are on and before
    # anything is built on it. SimulationApp leaves a default stage
    # open, and a character added to that one is never registered with
    # omni.anim.graph.core: get_character() returns None, no animation
    # plays, and everybody stands in bind pose with their arms out.
    # See the docstring on people.enable_extensions -- run_amr_sim.py
    # does the same thing for the same reason.
    from isaacsim.core.utils.stage import create_new_stage
    create_new_stage()

    world = World(stage_units_in_meters=1.0,
                  physics_dt=float(cfg["simulation"]["physics_dt"]),
                  rendering_dt=float(cfg["simulation"]["rendering_dt"]))
    scene_setup.build_scene(cfg, str(resolve_path(cfg["amr"]["usd_path"])),
                            verbose=True)
    # The other AMRs too, as a trial has them: the picture should show
    # the traffic AMR1 shares the floor with.
    from isaac_sim.utils import fleet as fleet_mod
    fleet = fleet_mod.Fleet(cfg)
    if len(fleet):
        print(f"  {len(fleet)} fleet AMR(s) at their docks", flush=True)
    world.reset()

    # Play, or everybody stands in the character rig's bind pose with
    # their arms out. The idle animation is driven by the behaviour
    # graph, and that graph is only evaluated while the timeline runs.
    # A second of it is enough to settle into idle and far short of the
    # 5 s at which the two walkers set off, so the shot is the scene at
    # its start.
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for i in range(args.settle):
        # World.reset() stops the timeline, and anything that calls it
        # again later stops the characters again, so this checks rather
        # than assumes.
        if not timeline.is_playing():
            timeline.play()
        world.step(render=True)
        if i == args.settle - 1:
            print(f"  timeline playing: {timeline.is_playing()}, "
                  f"sim time {timeline.get_current_time():.2f} s", flush=True)

    out = PROJECT_ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    label = Path(args.config).stem

    from pxr import UsdGeom
    from isaacsim.core.utils.stage import get_current_stage
    stage = get_current_stage()
    roof = stage.GetPrimAtPath(CEILING)

    # The plan, orthographic: a true map of the floor, so a drawing can
    # be laid over it with x_px = (x - X0) * PX_PER_M and nothing more.
    # scripts/scenario_figure.py does exactly that; keep ORTHO_* and
    # the image size in step with it.
    ortho_path = _plan_ortho(stage, world, rep, np, Image, out, label, roof, args)
    if ortho_path:
        print(f"  wrote {ortho_path.relative_to(PROJECT_ROOT)}", flush=True)

    for name, pos, look, focal, hide_roof in views(cfg):
        if roof and roof.IsValid():
            imageable = UsdGeom.Imageable(roof)
            if hide_roof:
                imageable.MakeInvisible()
            else:
                imageable.MakeVisible()
        # The plan view is shot portrait and turned a quarter turn
        # anticlockwise when it is written, which lands the bay's
        # 24 m length across a landscape frame.
        plan = name.endswith("_top")
        cam = rep.create.camera(position=tuple(float(v) for v in pos),
                                look_at=tuple(float(v) for v in look),
                                focal_length=float(focal))
        size = ((args.height, args.width) if plan
                else (args.width, args.height))
        product = rep.create.render_product(cam, size)
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(product)
        # The first frames of a new render product come back empty or
        # half-shaded; a few steps let the path tracer catch up.
        frame = None
        for _ in range(12):
            world.step(render=True)
            data = annot.get_data()
            if data is not None and np.asarray(data).size:
                frame = np.asarray(data)[..., :3]
        if frame is None:
            print(f"  [WARN] {name}: no frame came back", flush=True)
            continue
        path = out / f"scene_{label}_{name}.jpg"
        image = Image.fromarray(np.array(frame, copy=True))
        if plan:
            image = image.rotate(90, expand=True)
        image.save(path, quality=92)
        print(f"  wrote {path.relative_to(PROJECT_ROOT)}", flush=True)
        annot.detach(product)
        product.destroy()

    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
