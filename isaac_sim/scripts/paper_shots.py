"""Still captures for the paper: one scene, several Comfort overlays, fixed cameras.

    ~/isaacsim/python.sh isaac_sim/scripts/paper_shots.py --job <job.json>

A job (JSON) names one scene and what to shoot in it:

    {"config":   "configs/stage1a/envs/w2.20_y0.35_c3.yaml",
     "scenario": "configs/stage1a/conditions/static_r0.82.yaml",    (or null: config only)
     "amr_pose": [-1.6, 0.0, 0.0],             x, y, yaw_deg; null = navigation.start
     "out":      "results/captures/paper_261114/raw",
     "prefix":   "S1_static_W22",
     "views":    [{"name": "top", "ortho": true, "centre": [0, 0], "width_m": 12.0},
                  {"name": "persp", "position": [..], "look_at": [..], "focal": 24}],
     "overlays": [{"tag": "human_only", "comfort": false, "amr": false},
                  {"tag": "static", "scenario": "<overlay yaml>", "buffer": 0.10},
                  ...]}

Builds the scene as a trial does (scene_setup.build_scene: same people,
same robot, same Comfort Space drawing), runs the timeline a moment so
the characters settle into their idle pose, then for every overlay
redraws the Comfort Space (isaac_sim/utils/comfort_viz.draw, the trial's
own drawing) from the overlay's config and shows/hides the AMR, and
renders every view. Nothing is simulated beyond that settle; no Nav2.

Each PNG gets a sidecar JSON: camera, overlay, the Comfort extents drawn,
and for orthographic views the metres -> pixel mapping, so annotations
can be added afterwards at exact positions (scripts/paper_annotate.py).

The floor disc is the trial's drawing: the Comfort Space grown by the AMR
half width (the region the AMR CENTRE keeps out of); the paler outer band
is that plus the costmap buffer. Neither is a collider or seen by LiDAR.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

CEILING = "/World/Factory/Ceiling"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--job", required=True)
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--settle", type=int, default=120)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    job = json.loads(Path(args.job).read_text())

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from PIL import Image
    import omni.replicator.core as rep
    from isaacsim.core.api import World

    from comfort_core import load_config, resolve_path
    from comfort_core.comfort_space import (HeadingResolver, make_policy, robot_grow,
                                            shapes as build_shapes)
    from comfort_core.human_state import make_state_provider
    from isaac_sim.utils import scene_setup

    def cfg_for(scenario):
        c = load_config(scenario=scenario, common_path=str(PROJECT_ROOT / job["config"]))
        c.setdefault("markers", {})["enabled"] = bool(job.get("markers", False))
        if job.get("amr_pose"):
            x, y, yaw = job["amr_pose"]
            c["navigation"]["start"] = [float(x), float(y)]
            c["amr"]["spawn"]["yaw_deg"] = float(yaw)
        return c

    cfg = cfg_for(job.get("scenario"))

    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(10):
        app.update()
    from isaac_sim.environments import people as people_mod
    if people_mod.enable_extensions(cfg):
        for _ in range(10):
            app.update()
    from isaacsim.core.utils.stage import create_new_stage, get_current_stage
    create_new_stage()
    world = World(stage_units_in_meters=1.0,
                  physics_dt=float(cfg["simulation"]["physics_dt"]),
                  rendering_dt=float(cfg["simulation"]["rendering_dt"]))
    scene_setup.build_scene(cfg, str(resolve_path(cfg["amr"]["usd_path"])), verbose=False)
    from isaac_sim.utils import fleet as fleet_mod
    fleet = fleet_mod.Fleet(cfg)
    world.reset()
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(args.settle):
        if not timeline.is_playing():
            timeline.play()
        world.step(render=True)

    from pxr import Gf, UsdGeom
    from isaac_sim.utils import comfort_viz
    stage = get_current_stage()
    roof = stage.GetPrimAtPath(CEILING)
    if roof and roof.IsValid():
        UsdGeom.Imageable(roof).MakeInvisible()
    amr = stage.GetPrimAtPath(cfg["amr"]["articulation_path"])
    out = PROJECT_ROOT / job["out"]
    out.mkdir(parents=True, exist_ok=True)
    robots = {"AMR 1": [float(v) for v in cfg["navigation"]["start"]]}
    for m in getattr(fleet, "members", []):
        from comfort_core.labels import robot_label
        robots[robot_label(m.name)] = [float(m.x), float(m.y)]
    people = {k: list(map(float, v["position"])) for k, v in cfg["humans"].items()
              if isinstance(v, dict) and "position" in v and v.get("enabled", True)}

    # Orthographic plans are lit by the dome light only (the distant light's
    # shadows from the 1.5 m walls would otherwise split the corridor floor
    # and the Comfort disc in two tones); perspective views keep the
    # distant light. Same rule for every image, so every comparison group
    # is lit alike.
    from pxr import UsdLux
    lights = [UsdGeom.Imageable(p) for p in stage.Traverse() if p.IsA(UsdLux.DistantLight)]

    def shadows(on):
        for s in lights:
            s.MakeVisible() if on else s.MakeInvisible()

    def render(cam_path=None, view=None):
        shadows(not view.get("ortho"))
        if view.get("ortho"):
            cam = UsdGeom.Camera.Define(stage, "/World/PaperOrtho")
            cam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
            w = float(view["width_m"])
            cam.CreateHorizontalApertureAttr(w * 10.0)
            cam.CreateVerticalApertureAttr(w * args.height / args.width * 10.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 200.0))
            xf = UsdGeom.Xformable(cam.GetPrim())
            xf.ClearXformOpOrder()
            cx, cy = view.get("centre", [0.0, 0.0])
            xf.AddTranslateOp().Set(Gf.Vec3d(float(cx), float(cy), float(view.get("z", 30.0))))
            product = rep.create.render_product("/World/PaperOrtho", (args.width, args.height))
        else:
            cam = rep.create.camera(position=tuple(float(v) for v in view["position"]),
                                    look_at=tuple(float(v) for v in view["look_at"]),
                                    focal_length=float(view.get("focal", 24.0)))
            product = rep.create.render_product(cam, (args.width, args.height))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(product)
        frame = None
        for _ in range(int(view.get("warm", 16))):
            world.step(render=True)
            data = annot.get_data()
            if data is not None and np.asarray(data).size:
                frame = np.asarray(data)[..., :3]
        annot.detach(product)
        product.destroy()
        return frame

    written = []
    for ov in job["overlays"]:
        c = cfg_for(ov.get("scenario", job.get("scenario")))
        if ov.get("buffer") is not None:
            c["comfort_space"].setdefault("costmap", {})["buffer_width"] = float(ov["buffer"])
        comfort_viz.clear(stage)
        extents = {}
        if ov.get("comfort", True):
            states = make_state_provider(c).states()
            policy = make_policy(c)
            shp = build_shapes(c, policy.axes(states),
                               headings=HeadingResolver(c, policy).resolve(states),
                               grow=robot_grow(c))
            comfort_viz.draw(stage, c, shp, states)
            for s in build_shapes(c, policy.axes(states),
                                  headings=HeadingResolver(c, policy).resolve(states), grow=0.0):
                extents[s.key] = {"centre": list(s.centre), "along": s.along, "across": s.across,
                                  "heading_rad": s.heading}
        if amr and amr.IsValid():
            img = UsdGeom.Imageable(amr)
            img.MakeVisible() if ov.get("amr", True) else img.MakeInvisible()
        for view in job["views"]:
            frame = render(view=view)
            name = f"{job['prefix']}_{ov['tag']}_{view['name']}"
            if frame is None:
                print(f"  [WARN] {name}: no frame", flush=True)
                continue
            path = out / f"{name}.png"
            Image.fromarray(np.array(frame, copy=True)).save(path)
            meta = {"file": str(path.relative_to(PROJECT_ROOT)), "config": job["config"],
                    "scenario": ov.get("scenario", job.get("scenario")), "overlay": ov,
                    "view": view, "size": [args.width, args.height],
                    "amr_pose": job.get("amr_pose") or cfg["navigation"]["start"],
                    "amr_visible": ov.get("amr", True), "comfort_drawn": ov.get("comfort", True),
                    "buffer_width": float((c["comfort_space"].get("costmap") or {}).get("buffer_width", 0) or 0),
                    "policy": c["comfort_space"]["policy"], "comfort_shapes_ungrown": extents,
                    "robots": robots, "people": people,
                    "note": "floor disc = Comfort Space grown by the AMR half width; outer band = + costmap buffer"}
            if view.get("ortho"):
                w = float(view["width_m"])
                cx, cy = view.get("centre", [0.0, 0.0])
                meta["px_per_m"] = args.width / w
                meta["ortho_origin_px"] = [args.width / 2 - cx * args.width / w,
                                           args.height / 2 + cy * args.width / w]
            path.with_suffix(".json").write_text(json.dumps(meta, indent=2))
            written.append(str(path.relative_to(PROJECT_ROOT)))
            print(f"  wrote {path.relative_to(PROJECT_ROOT)}", flush=True)
    app.close()
    print(f"[paper_shots] {len(written)} images", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
