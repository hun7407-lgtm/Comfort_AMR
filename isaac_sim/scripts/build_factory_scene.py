"""Build the factory bay and save it as a USD you can open and edit.

    ./scripts/build_factory_scene.sh
    ./scripts/build_factory_scene.sh --screenshot results/factory_preview.png

Writes the scene to `factory.usd_path` (configs/factory.yaml). Open that
file in Isaac Sim, move things, save, and the experiment picks up the
edit: the scene is a USD on disk, not something rebuilt from code at
every launch.

Rebuilding overwrites the file, so hand edits are lost on the next
build. That is the intended direction of travel while the layout is
still being drafted; once it settles, the generated scene becomes the
base layer and edits live in a sublayer.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/factory.yaml")
    parser.add_argument("--out", default=None,
                        help="where to write the USD (default: factory.usd_path)")
    parser.add_argument("--screenshot", default=None,
                        help="also render a preview PNG here")
    parser.add_argument("--gui", action="store_true",
                        help="open a window instead of running headless")
    return parser.parse_args()


args = parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": not args.gui})

import numpy as np  # noqa: E402
from isaacsim.core.utils.stage import (create_new_stage, get_current_stage,  # noqa: E402
                                       is_stage_loading)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from comfort_core.factory_layout import clearance_summary, summary  # noqa: E402
from isaac_sim.environments import factory, people, props  # noqa: E402
from isaac_sim.utils import scene_setup  # noqa: E402


def add_light(cfg) -> None:
    """Light the bay: a warehouse sky, and a fitting over each aisle.

    The dome carries an indoor HDRI, so surfaces pick up the colour and
    the direction of a real building instead of a flat grey wash --
    which is most of the difference between a render that reads as a
    factory and one that reads as a toy. Falls back to a plain dome
    when the asset server is unreachable.

    Kept outside the factory root prim: referencing the bay into another
    scene should bring the building, not someone else's lighting.
    """
    stage = get_current_stage()
    shell = cfg.get("assets", {}).get("shell", {})
    sky = shell.get("sky", {})
    base = props.root() if props.enabled(cfg) else None

    light = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/dome"))
    if base and sky.get("path"):
        light.CreateTextureFileAttr(base + str(sky["path"]))
        light.CreateTextureFormatAttr("latlong")
        light.CreateIntensityAttr(float(sky.get("intensity", 900.0)))
        # The sky is here to light the bay and to put something in the
        # reflections, not to be looked at: with it visible you see a
        # photograph of another warehouse through the glazing.
        if not sky.get("visible", True):
            light.GetPrim().CreateAttribute(
                "visibleInPrimaryRay", Sdf.ValueTypeNames.Bool).Set(False)
    else:
        light.CreateIntensityAttr(900.0)
        light.CreateColorAttr((0.95, 0.96, 1.0))

    rig = cfg["factory"].get("ceiling", {}).get("lights")
    if not rig or not cfg["factory"].get("ceiling", {}).get("enabled", True):
        return
    from comfort_core.factory_layout import build_layout

    # The row lights are built with the fittings, in
    # isaac_sim/environments/factory.build_ceiling_lights, so the
    # running simulation gets them too. Making them here as well
    # would light the saved file twice as brightly as the live one.


def add_physics(cfg) -> None:
    """A physics scene and a ground plane, so the file can be played.

    Without them, pressing Play in the GUI raises "no active physics
    scene found" and the simulation view fails to start. Both use the
    same settings as the corridor scene, from configs/common.yaml, so
    the bay does not quietly run under different physics.
    """
    scene_setup.setup_physics_scene(cfg)
    scene_setup.add_ground_plane(cfg)


def add_viewport_camera(layout) -> None:
    """Aim the GUI camera at the bay, and leave a named camera behind.

    Kit's default perspective camera sits hundreds of metres from the
    origin, so opening the file showed an empty dark viewport: a 12 m
    bay is a speck at that distance, not a missing scene. Authoring
    /OmniverseKit_Persp into the file means the viewport opens looking
    at the building, and /World/Cameras/overview gives the same view
    back after the camera has been flown somewhere else.
    """
    stage = get_current_stage()
    eye = Gf.Vec3d(0.0, -layout.width * 1.15, max(layout.length, layout.width) * 0.75)
    look_at = Gf.Vec3d(0.0, 0.0, 1.0)
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    pose = Gf.Matrix4d().SetLookAt(eye, look_at, up).GetInverse()

    for path in ("/OmniverseKit_Persp", "/World/Cameras/overview"):
        camera = UsdGeom.Camera.Define(stage, Sdf.Path(path))
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500.0))
        camera.AddTransformOp().Set(pose)


def wait_for_assets(limit: int = 1500) -> int:
    """Let the models finish streaming in.

    The library is on a remote server and loads in the background. Save
    or render before it lands and machines are quietly missing -- which
    looks exactly like a layout bug, and was one for about ten minutes.
    """
    frames = 0
    while is_stage_loading() and frames < limit:
        simulation_app.update()
        frames += 1
    # The characters hold their bind pose -- arms out, a T -- until the
    # timeline runs: the animation graph only evaluates while playing,
    # and the behaviour script's Idle command arrives on the play
    # event. So the clock is started for long enough to settle them,
    # and left running through the export and the renders.
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(150):
        simulation_app.update()
    return frames


def screenshot(path: Path, layout, root: str) -> None:
    """Three views, so a wrong number shows up as a wrong picture.

    One angled view to look at, one from directly above to check the
    layout against the drawing, and one at the height of a person in
    the aisle, which is the view the study is actually about.
    """
    import omni.replicator.core as rep
    from PIL import Image

    span = max(layout.length, layout.width)
    # Above about thirty metres the renderer returns depth but no
    # shading, and every frame comes back black. Measured on the 24 m
    # bay: fine at 26 m, black at 32 m. So the camera stays low and the
    # lens does the work instead.
    aperture = 20.955  # replicator's default horizontal aperture, mm

    def lens(distance: float, coverage: float) -> float:
        return aperture * distance / max(coverage, 0.1)

    over_z = min(span * 1.1, 24.0)
    over_y = -(layout.width * 0.55 + span * 0.35)
    over_d = (over_y ** 2 + over_z ** 2) ** 0.5
    top_z = min(span * 1.5, 25.0)

    aisle_y = layout.aisles[0].centre
    views = {
        # A wide shot from inside, which is how the plant is actually
        # seen. An outside camera now meets a wall: they run to the
        # ceiling, as they do in the building.
        "": ((layout.x_max - 1.2, aisle_y - 0.55, 2.9),
             (layout.x_min * 0.4, aisle_y + 0.2, 1.0), 15.0),
        # Almost straight down, tilted just enough that the up vector is
        # well defined: +y ends up at the top of the picture, so it can
        # be held next to the drawing. The roof comes off for it.
        # The frame is 1600 x 1000, so fitting the bay's width needs
        # 1.6 times that much horizontal coverage.
        "_top": ((0.0, -0.9, top_z), (0.0, 0.0, 0.0),
                 lens(top_z, max(layout.length, layout.width * 1.6) + 3.0)),
        "_aisle": ((-layout.length / 2.0 + 0.6, aisle_y, 1.60),
                   (layout.length / 2.0, aisle_y, 1.10), 18.0),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = get_current_stage()
    shell = stage.GetPrimAtPath(f"{root}/Shell")
    near_walls = [c for c in (shell.GetChildren() if shell else [])
                  if c.GetName().startswith("wall_s")]
    ceiling = stage.GetPrimAtPath(f"{root}/Ceiling")

    # Now that the walls reach the ceiling, a camera outside the
    # building sees a wall. The overview and the plan are cutaways: the
    # roof comes off, and for the overview the near wall with it.
    hide = {"_top": [ceiling]}

    for suffix, (eye, look_at, focal) in views.items():
        for prim in [ceiling] + near_walls:
            if not prim:
                continue
            image = UsdGeom.Imageable(prim)
            if prim in hide.get(suffix, []):
                image.MakeInvisible()
            else:
                image.MakeVisible()
        camera = rep.create.camera(position=eye, look_at=look_at,
                                   focal_length=focal)
        product = rep.create.render_product(camera, (1600, 1000))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach(product)
        # app.update() alone leaves the render product empty here; the
        # orchestrator is what actually drives a replicator capture in a
        # standalone script with no simulation running. A big scene also
        # needs several passes before the first frame is anything but
        # black, which is what a too-eager capture returns.
        for _ in range(30):
            simulation_app.update()
        data = np.zeros(0)
        for attempt in range(6):
            rep.orchestrator.step(rt_subframes=8)
            data = np.asarray(annotator.get_data())
            if data.size and float(data[..., :3].mean()) > 2.0:
                break
        annotator.detach(product)
        if data.size == 0 or float(data[..., :3].mean()) <= 2.0:
            print(f"  [warn] {suffix or 'overview'} came out black after "
                  f"{attempt + 1} passes", flush=True)
            continue
        out = path.with_name(f"{path.stem}{suffix}{path.suffix}")
        Image.fromarray(data[..., :3]).save(out)
        print(f"  preview  : {out}", flush=True)


def check_lanes_clear(cfg, layout) -> int:
    """Nothing may stand in the aisles within the scan slice.

    The layout checks cover machines, people and parked props, each
    against its own list. This one looks at the scene that actually
    came out: every piece of geometry between 0.11 m and 1.25 m -- the
    band the LiDAR sees -- that overlaps an aisle, a crossing or a
    refuge. A mezzanine post in a cross aisle got through every other
    check and deadlocked two trials.
    """
    from comfort_core.factory_layout import drivable_rects

    stage = get_current_stage()
    rects = drivable_rects(layout)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              includedPurposes=[UsdGeom.Tokens.default_])
    skip = ("/World/Characters", f"{cfg['factory']['root_prim']}/Floor",
            f"{cfg['factory']['root_prim']}/Waypoints")
    # Things bolted to the wall are not obstructions in a lane that
    # happens to run along that wall: a roller shutter stands 130 mm
    # proud and nobody drives into the wall anyway.
    edge = 0.25
    offenders = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if any(path.startswith(s) for s in skip) or not prim.IsA(UsdGeom.Boundable):
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        if box.IsEmpty():
            continue
        lo, hi = box.GetMin(), box.GetMax()
        if hi[2] < 0.11 or lo[2] > 1.25:
            continue
        if (lo[0] < layout.x_min + edge or hi[0] > layout.x_max - edge
                or lo[1] < layout.y_min + edge or hi[1] > layout.y_max - edge):
            continue
        for r in rects:
            over_x = min(hi[0], r.x_max) - max(lo[0], r.x_min)
            over_y = min(hi[1], r.y_max) - max(lo[1], r.y_min)
            # A hand's breadth or less is something mounted on the wall
            # that the aisle rectangle happens to touch, not an
            # obstruction in it.
            if over_x > 0.08 and over_y > 0.08:
                offenders.append((path, r.name, over_x, over_y))
                break
    seen = set()
    for path, where, ox, oy in offenders:
        group = "/".join(path.split("/")[:6])
        if group in seen:
            continue
        seen.add(group)
        print(f"  [warn] {path} is {ox:.2f} x {oy:.2f} m into {where}", flush=True)
    return len(seen)


def verify(path: Path) -> None:
    """Re-open what was written and say what is in it.

    Exporting a live stage is not the same as authoring a file: a missing
    physics scene or a prim left in the session layer only shows up when
    the file is opened on its own, which is how it will be used.
    """
    stage = Usd.Stage.Open(str(path))
    scenes = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    camera = "camera yes" if stage.GetPrimAtPath("/OmniverseKit_Persp") else "camera MISSING"
    counts = []
    root = stage.GetPrimAtPath("/World/Factory")
    if root:
        for child in root.GetChildren():
            counts.append(f"{child.GetName()} {len(child.GetChildren())}")
    print(f"  contains : physics scene {scenes if scenes else 'MISSING'}, {camera}, "
          f"{', '.join(counts) if counts else 'no factory prims'}", flush=True)


def main() -> int:
    cfg = load_config(common_path=args.config)
    if "factory" not in cfg:
        print(f"[ERROR] {args.config} has no `factory` section", flush=True)
        return 1

    # Must happen before the stage exists; see people.enable_extensions.
    if people.enable_extensions(cfg):
        print("  characters: animation extensions enabled", flush=True)

    create_new_stage()
    stage = get_current_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    add_light(cfg)
    add_physics(cfg)

    print("\n=== Factory bay ===", flush=True)
    if props.enabled(cfg):
        props.check_module_sizes(cfg)
        print(f"  assets   : {props.root()}", flush=True)
    else:
        print("  assets   : asset server unavailable, building from boxes",
              flush=True)
    layout = factory.build_bay(cfg, say=lambda m: print(f"  {m}", flush=True))
    for line in summary(layout).splitlines():
        print(f"  {line}", flush=True)
    for line in clearance_summary(cfg, layout).splitlines():
        print(f"  {line}", flush=True)

    add_viewport_camera(layout)
    world = stage.GetPrimAtPath("/World")
    if world:
        stage.SetDefaultPrim(world)

    blocking = check_lanes_clear(cfg, layout)
    print(f"  lanes    : {'clear' if not blocking else str(blocking) + ' obstruction(s)'}",
          flush=True)

    frames = wait_for_assets()
    print(f"  loaded   : models streamed in after {frames} frames", flush=True)

    out = Path(args.out) if args.out else resolve_path(cfg["factory"]["usd_path"])
    out.parent.mkdir(parents=True, exist_ok=True)
    simulation_app.update()
    # The root layer, not the composed stage: Export() flattens every
    # referenced model into the file and turns a 90 KB scene into 130 MB
    # of duplicated warehouse props. Writing the layer keeps the
    # references as references, which is also what makes the file
    # editable -- moving a machine moves a reference, not a copy of a
    # conveyor.
    stage.GetRootLayer().Export(str(out))
    print(f"\n  saved    : {out}", flush=True)

    verify(out)

    if args.screenshot:
        screenshot(Path(args.screenshot), layout,
                   str(cfg['factory']['root_prim']))

    print("  open it  : isaac-sim.sh, File > Open. If the viewport is dark,\n"
          "             click /World/Factory in the Stage panel and press F.\n", flush=True)
    return 0


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception:
        import traceback
        traceback.print_exc()
    sys.stdout.flush()
    # Kit ends the process itself on close; exit here to keep the status.
    os._exit(code)
