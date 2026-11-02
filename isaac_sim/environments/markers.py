"""Floor markers that say where a trial starts and ends.

For looking at, not for measuring. They are drawn flat on the floor,
carry no collider and no rigid body, and are off unless a config turns
them on.

Why that is safe to have in the scene at all
--------------------------------------------
An RTX LiDAR traces whatever renders, so anything added to the scene
is a potential sensor return. These cannot be:

    the sensor sits 0.31 m up and its lowest beam rises at 1.9 deg,
    so at any horizontal distance d it is already 0.31 + 0.033 * d
    above the floor. A 4 mm decal is never sampled, at any range.

That is the physical argument. The flag is the belt and braces: with
`markers.enabled` false -- the default, and what every batch runs --
the prims are not created at all, so a trial cannot differ because of
them. Turn them on for a window, a screenshot or a video, not for a
measurement.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple

from pxr import Gf, Sdf, UsdGeom

ROOT = "/World/Markers"


def _xform(stage, path: str, position, yaw_deg: float = 0.0):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    if yaw_deg:
        xf.AddRotateZOp().Set(float(yaw_deg))
    return xf


def _colour(prim, rgb: Tuple[float, float, float]) -> None:
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    gprim.CreateDisplayOpacityAttr([1.0])
    # Round, not octagonal. A cylinder is tessellated at the stage's
    # refinement level, which is 0 by default and turns a 0.45 m ring
    # into an eight-sided plate at this scale.
    prim.CreateAttribute("refinementEnableOverride",
                         Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("refinementLevel",
                         Sdf.ValueTypeNames.Int).Set(2)


def _disc(stage, path: str, radius: float, thickness: float,
          rgb, z: float = 0.0):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(thickness))
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([(-radius, -radius, -thickness / 2),
                          (radius, radius, thickness / 2)])
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z))
    _colour(cyl.GetPrim(), rgb)
    return cyl


def _arrow(stage, path: str, length: float, width: float,
           thickness: float, rgb, z: float = 0.0):
    """A shaft and a head, lying flat, pointing along +x."""
    shaft = UsdGeom.Cube.Define(stage, f"{path}/shaft")
    shaft.CreateSizeAttr(1.0)
    sx = UsdGeom.Xformable(shaft)
    sx.AddTranslateOp().Set(Gf.Vec3d(length * 0.30, 0.0, z))
    sx.AddScaleOp().Set(Gf.Vec3f(length * 0.60, width, thickness))
    _colour(shaft.GetPrim(), rgb)

    head = UsdGeom.Cone.Define(stage, f"{path}/head")
    head.CreateRadiusAttr(width * 1.6)
    head.CreateHeightAttr(length * 0.40)
    head.CreateAxisAttr("X")
    hx = UsdGeom.Xformable(head)
    hx.AddTranslateOp().Set(Gf.Vec3d(length * 0.80, 0.0, z))
    hx.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, thickness / (width * 3.2)))
    _colour(head.GetPrim(), rgb)


def build(cfg: Dict[str, Any], stage) -> list[str]:
    """Draw the start and goal markers. Returns what it drew."""
    spec = cfg.get("markers", {})
    if not spec.get("enabled", False):
        return []

    nav = cfg["navigation"]
    thickness = float(spec.get("thickness", 0.004))
    radius = float(spec.get("radius", 0.45))
    z = float(spec.get("z", 0.002))
    drawn = []

    stage.DefinePrim(Sdf.Path(ROOT), "Xform")

    places = (
        ("Start", nav["start"][:2],
         float(cfg["amr"].get("spawn", {}).get("yaw_deg", 0.0)),
         tuple(spec.get("start_colour", (0.10, 0.75, 0.25)))),
        ("Goal", nav["goal"][:2],
         float(nav.get("goal_yaw_deg") or 0.0),
         tuple(spec.get("goal_colour", (0.95, 0.35, 0.10)))),
    )
    for name, (x, y), yaw, rgb in places:
        base = f"{ROOT}/{name}"
        _xform(stage, base, (float(x), float(y), 0.0), yaw)
        # A ring reads better from above than a filled disc: the pose
        # it marks is the middle, and a filled one hides it.
        # Each layer sits a little above the one under it. Drawn at the
        # same height they z-fight, and the ring's edge came back
        # stippled.
        _disc(stage, f"{base}/outer", radius, thickness, rgb, z)
        _disc(stage, f"{base}/inner", radius * 0.72, thickness,
              tuple(spec.get("floor_colour", (0.22, 0.22, 0.24))),
              z + thickness)
        _disc(stage, f"{base}/pip", radius * 0.18, thickness, rgb,
              z + thickness * 2)
        # Which way the robot faces here. At the goal this is the
        # difference between parking and being trapped by its own
        # protective field, so it is worth being able to see.
        _arrow(stage, f"{base}/heading", radius * 2.2, 0.075,
               thickness, rgb, z + thickness * 2)
        drawn.append(f"{name} at ({float(x):+.2f}, {float(y):+.2f}) "
                     f"facing {yaw:+.0f} deg")
    return drawn
