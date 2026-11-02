"""Draw Comfort Spaces in the Isaac Sim viewport.

Isaac Sim only: imports omni and pxr.

Drawing only. Nothing here creates a collider, a rigid body or a physics
API, so turning visualisation off cannot change a trial result. What the
planner sees comes from the costmap, written elsewhere. Keeping the two
apart is a requirement of the experiment, not a style preference: if the
picture could alter the outcome, the picture would be part of the
method.

Each human gets a thin translucent disc on the floor:

  * the inner, stronger disc is the Comfort Space radius, the distance
    the AMR's body is meant to stay outside of;
  * the outer, fainter disc is that radius plus the AMR's inscribed
    radius, which is the circle a planner working on the robot's centre
    point actually has to avoid. Seeing both makes it obvious why Nav2
    sometimes refuses a gap that looks wide enough.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from isaacsim.core.utils.prims import delete_prim, is_prim_path_valid
from pxr import Gf, Sdf, UsdGeom, UsdShade

from comfort_core.comfort_space import robot_grow, shapes as build_shapes

VIZ_ROOT = "/Environment/ComfortSpace"
LOOKS = f"{VIZ_ROOT}/Looks"

# Which state each drawn disc is currently coloured for, by prim path.
# A walker changes state part way through, and rebinding a material
# every frame for sixty frames of no change is waste, so this remembers
# what is already bound. Reset by clear().
_BOUND: Dict[str, str] = {}


def _translucent_material(stage, name: str, color, opacity: float):
    """A UsdPreviewSurface that renders as coloured glass.

    Reused across discs so the stage does not accumulate one material
    per human per redraw.
    """
    path = f"{LOOKS}/{name}"
    if is_prim_path_valid(path):
        return UsdShade.Material.Get(stage, path)

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*[float(c) for c in color]))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*[float(c) * 0.4 for c in color]))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _disc(stage, path: str, shape, z: float, height: float, material) -> None:
    """A unit-radius disc scaled into the Comfort Space's ellipse.

    Built at radius 1 and shaped entirely by its transform, so the same
    prim can be moved, turned and resized every frame as a walking
    person speeds up and slows down. Rebuilding the geometry instead
    would churn prims sixty times a second.
    """
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateRadiusAttr(1.0)
    cylinder.CreateHeightAttr(float(height))
    # Without an explicit extent the viewport culls the disc at odd angles.
    cylinder.CreateExtentAttr([
        Gf.Vec3f(-1.0, -1.0, -height / 2.0),
        Gf.Vec3f(1.0, 1.0, height / 2.0),
    ])
    xf = UsdGeom.Xformable(cylinder.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(float(shape.x), float(shape.y), float(z)))
    xf.AddRotateZOp().Set(float(math.degrees(shape.heading)))
    xf.AddScaleOp().Set(Gf.Vec3f(float(shape.along), float(shape.across), 1.0))
    UsdShade.MaterialBindingAPI(cylinder.GetPrim()).Bind(material)


def _state_colours(cfg: Dict[str, Any]):
    """state -> colour, or None to draw every space the one colour."""
    viz = cfg["comfort_space"].get("visualization", {})
    table = viz.get("state_colour") or {}
    return {str(k): v for k, v in table.items()} or None


def _materials_for(stage, cfg: Dict[str, Any], state: str):
    """The (inner, outer) material a person in `state` is drawn with.

    Size alone already separates the three states, but only once you
    know the numbers; colour says which is which at a glance, which is
    what a screenshot has to do. The palette is the one the floor
    decals use (configs/common.yaml preview.state_colour), so the two
    never disagree about what blue means.
    """
    viz = cfg["comfort_space"].get("visualization", {})
    opacity = float(viz.get("opacity", 0.35))
    table = _state_colours(cfg)
    known = bool(table) and state in table
    colour = table[state] if known else viz.get("color", [0.15, 0.55, 0.95])
    tag = state if known else "Default"
    return (_translucent_material(stage, f"ComfortInner_{tag}", colour,
                                  opacity),
            _translucent_material(stage, f"ComfortOuter_{tag}", colour,
                                  opacity * 0.45))


def clear(stage) -> None:
    """Remove every Comfort Space prim, leaving the materials in place."""
    _BOUND.clear()
    for name in ("Discs",):
        path = f"{VIZ_ROOT}/{name}"
        if is_prim_path_valid(path):
            delete_prim(path)


def check_below_scan(cfg: Dict[str, Any]) -> str:
    """Make sure the drawn discs cannot end up in the laser scan.

    Isaac Sim's RTX sensor traces against anything that renders, and a
    Comfort Space disc is renderable. If one rose into the slice of the
    cloud that becomes /scan, a region that exists only as a drawing
    would start marking obstacles in the costmap and stopping the
    collision monitor. Drawing must never change what the robot
    perceives, so this is checked rather than assumed.

    Returns an empty string when safe, otherwise the problem.
    """
    viz = cfg["comfort_space"].get("visualization", {})
    if not viz.get("enabled", True):
        return ""
    scan = cfg["ros"].get("scan", {})
    if not scan:
        return ""

    disc_top = (float(viz.get("z_offset", 0.0))
                + 2.0 * float(viz.get("height", 0.0)))
    scan_floor = (float(scan["lidar_world_z"]) + float(scan["min_height"]))
    if disc_top >= scan_floor:
        return (f"Comfort Space discs reach z={disc_top:.3f} m but the laser "
                f"slice starts at z={scan_floor:.3f} m, so the drawing would "
                f"be seen as an obstacle. Lower comfort_space.visualization"
                f".z_offset or height, or raise ros.scan.min_height.")
    return ""


def draw(stage, cfg: Dict[str, Any], shapes: List,
         states: Dict[str, str] | None = None) -> int:
    """Draw one Comfort Space per human. Returns how many were drawn.

    Safe to call repeatedly: the previous discs are removed first.
    """
    viz = cfg["comfort_space"].get("visualization", {})
    if not viz.get("enabled", True):
        return 0

    clear(stage)
    UsdGeom.Scope.Define(stage, VIZ_ROOT)
    UsdGeom.Scope.Define(stage, LOOKS)
    UsdGeom.Scope.Define(stage, f"{VIZ_ROOT}/Discs")

    height = float(viz.get("height", 0.02))
    z = float(viz.get("z_offset", 0.01)) + height / 2.0

    buffer_width = float(
        cfg["comfort_space"].get("costmap", {}).get("buffer_width", 0.0))

    for shape in shapes:
        state = str((states or {}).get(shape.key, ""))
        inner_mat, outer_mat = _materials_for(stage, cfg, state)
        # Outer first and slightly lower, so the inner disc reads on top.
        planner_path = f"{VIZ_ROOT}/Discs/planner_{shape.key}"
        comfort_path = f"{VIZ_ROOT}/Discs/comfort_{shape.key}"
        _disc(stage, planner_path,
              shape.grown(buffer_width), z, height * 0.5, outer_mat)
        _disc(stage, comfort_path, shape, z + height, height, inner_mat)
        _BOUND[planner_path] = state
        _BOUND[comfort_path] = state
    return len(shapes)


def update(stage, shapes: List, buffer_width: float = 0.0,
           cfg: Dict[str, Any] | None = None,
           states: Dict[str, str] | None = None) -> int:
    """Move, turn and resize the existing discs. Returns how many moved.

    The discs are drawn once when the scene is built, which is fine
    while everyone stands still and wrong the moment someone walks: the
    Comfort Space stayed where the person had been, and it did not grow
    or shrink with their speed. Rewriting the transforms is cheap;
    rebuilding the prims would not be.
    """
    moved = 0
    for shape in shapes:
        state = str((states or {}).get(shape.key, ""))
        for prefix, drawn in (("comfort", shape),
                              ("planner", shape.grown(buffer_width))):
            path = f"{VIZ_ROOT}/Discs/{prefix}_{shape.key}"
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            # Recolour only when the person's state has actually
            # changed. A walker who sets off goes from the engaged
            # colour to the moving one here; standing still rebinds
            # nothing.
            if cfg is not None and states is not None \
                    and _BOUND.get(path) != state:
                inner_mat, outer_mat = _materials_for(stage, cfg, state)
                UsdShade.MaterialBindingAPI(prim).Bind(
                    inner_mat if prefix == "comfort" else outer_mat)
                _BOUND[path] = state
            ops = {op.GetOpName(): op
                   for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
            translate = ops.get("xformOp:translate")
            if translate is not None:
                current = translate.Get()
                translate.Set(Gf.Vec3d(float(drawn.x), float(drawn.y),
                                       float(current[2])))
            rotate = ops.get("xformOp:rotateZ")
            if rotate is not None:
                rotate.Set(float(math.degrees(drawn.heading)))
            scale = ops.get("xformOp:scale")
            if scale is not None:
                scale.Set(Gf.Vec3f(float(drawn.along), float(drawn.across), 1.0))
            moved += 1
    return moved


def summary(shapes: List, states: Dict[str, str],
            speeds: Dict[str, float] | None = None) -> str:
    """One line per human, for the simulation's startup log."""
    lines = []
    for shape in shapes:
        extent = (f"R={shape.across:.2f} m" if shape.is_circle
                  else f"{shape.along:.2f} x {shape.across:.2f} m at "
                       f"{math.degrees(shape.heading):+.0f} deg")
        speed = (speeds or {}).get(shape.key)
        moving = f", {speed:.2f} m/s" if speed else ""
        lines.append(f"human {shape.key} at ({shape.x:+.2f}, {shape.y:+.2f}) "
                     f"state={states.get(shape.key, '?')} {extent}{moving}")
    if not lines:
        lines.append("no humans configured")
    return "\n".join(lines)
