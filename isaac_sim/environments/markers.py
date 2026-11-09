"""Floor markers that say where every AMR starts and is going.

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

What is drawn
-------------
AMR1's start (a ring) and goal (a filled disc), each
with the heading it will have there, in comfort_core/palette.py's
mission colour; and for every fleet AMR its start (ring) and each
job (small disc) in that vehicle's own colour -- the colour of the
stripe on its body and of its labels in the costmap video. Every
point carries a text plate: "AMR1 START", "A1 START", "A1-2 n2_r2c1".
A job list that returns to where it began shares one plate.
"""

from __future__ import annotations

import hashlib
import math
from typing import Any, Dict, List, Tuple

from pxr import Gf, Sdf, UsdGeom, UsdShade

from comfort_core import project_root
from comfort_core.palette import MISSION_COLOUR, fleet_colour

ROOT = "/World/Markers"
LABEL_DIR = project_root() / "results" / "cache" / "markers"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
# Text plate on the floor, metres. Tall enough to read from the
# overhead and the aisle cameras alike.
PLATE_W, PLATE_H = 1.10, 0.30


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


def _label_texture(text: str, rgb) -> str:
    """A PNG of `text` in white on the AMR's colour, drawn once."""
    from PIL import Image, ImageDraw, ImageFont
    key = hashlib.md5(f"{text}|{rgb}".encode()).hexdigest()[:12]
    path = LABEL_DIR / f"{key}.png"
    if not path.is_file():
        LABEL_DIR.mkdir(parents=True, exist_ok=True)
        w, h = 1024, int(1024 * PLATE_H / PLATE_W)
        back = tuple(int(round(c * 255)) for c in rgb)
        image = Image.new("RGB", (w, h), back)
        draw = ImageDraw.Draw(image)
        size = int(h * 0.62)
        font = ImageFont.truetype(FONT, size)
        while font.getlength(text) > w * 0.92 and size > 10:
            size -= 4
            font = ImageFont.truetype(FONT, size)
        box = draw.textbbox((0, 0), text, font=font)
        draw.text(((w - (box[2] - box[0])) / 2 - box[0],
                   (h - (box[3] - box[1])) / 2 - box[1]),
                  text, fill=(255, 255, 255), font=font)
        image.save(path)
    return str(path)


def _plate(stage, path: str, text: str, rgb, at, z: float,
           yaw_deg: float = 0.0) -> None:
    """A flat quad on the floor carrying the label as its texture."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    hw, hh = PLATE_W / 2.0, PLATE_H / 2.0
    mesh.CreatePointsAttr([Gf.Vec3f(-hw, -hh, 0), Gf.Vec3f(hw, -hh, 0),
                           Gf.Vec3f(hw, hh, 0), Gf.Vec3f(-hw, hh, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([Gf.Vec3f(-hw, -hh, 0), Gf.Vec3f(hw, hh, 0)])
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(at[0]), float(at[1]), float(z)))
    if yaw_deg:
        xf.AddRotateZOp().Set(float(yaw_deg))

    mat_path = f"{path}/Material"
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, f"{mat_path}/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(_label_texture(text, rgb))
    # The plate is the AMR's colour even if the image does not load.
    texture.CreateInput("fallback", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0))
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def _toward_aisle(y: float) -> float:
    """Which way to push a plate so it lands on the lane side of a slot."""
    return -1.0 if y > 0 else 1.0


def _points(cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every start and destination, with coincident ones merged."""
    nav = cfg["navigation"]
    raw: List[Dict[str, Any]] = []
    sx, sy = (float(v) for v in nav["start"][:2])
    gx, gy = (float(v) for v in nav["goal"][:2])
    raw.append({"xy": (sx, sy), "text": "AMR1 START", "rgb": MISSION_COLOUR,
                "kind": "start",
                "yaw": float(cfg["amr"].get("spawn", {}).get("yaw_deg", 0.0))})
    raw.append({"xy": (gx, gy), "text": "AMR1 GOAL", "rgb": MISSION_COLOUR,
                "kind": "goal", "yaw": float(nav.get("goal_yaw_deg") or 0.0)})

    fleet = cfg.get("fleet", {})
    places = fleet.get("places", {}) if fleet.get("enabled", False) else {}
    for name, spec in sorted(fleet.get("amrs", {}).items()):
        if not places or not spec.get("enabled", True):
            continue
        tag = name.upper()
        rgb = fleet_colour(name)
        start = places.get(str(spec.get("start", "")))
        if start is not None:
            raw.append({"xy": (float(start[0]), float(start[1])),
                        "text": f"{tag} START", "rgb": rgb, "kind": "start"})
        for order, job in enumerate(spec.get("jobs", []), start=1):
            point = places.get(str(job))
            if point is None:
                continue
            raw.append({"xy": (float(point[0]), float(point[1])),
                        "text": f"{tag}-{order} {job}", "rgb": rgb, "kind": "job"})

    merged: Dict[Tuple[float, float], Dict[str, Any]] = {}
    for item in raw:
        key = (round(item["xy"][0], 2), round(item["xy"][1], 2))
        if key in merged:
            merged[key]["text"] += " / " + item["text"]
        else:
            merged[key] = dict(item)
    return list(merged.values())


def build(cfg: Dict[str, Any], stage) -> list[str]:
    """Draw every AMR's start and destinations. Returns what it drew."""
    spec = cfg.get("markers", {})
    if not spec.get("enabled", False):
        return []

    thickness = float(spec.get("thickness", 0.004))
    radius = float(spec.get("radius", 0.45))
    z = float(spec.get("z", 0.002))
    floor = tuple(spec.get("floor_colour", (0.22, 0.22, 0.24)))
    drawn = []

    stage.DefinePrim(Sdf.Path(ROOT), "Xform")

    for index, item in enumerate(_points(cfg)):
        x, y = item["xy"]
        rgb = item["rgb"]
        base = f"{ROOT}/mark_{index:02d}"
        _xform(stage, base, (x, y, 0.0), float(item.get("yaw", 0.0)))
        # Each layer sits a little above the one under it. Drawn at the
        # same height they z-fight, and the ring's edge came back
        # stippled.
        if item["kind"] == "start":
            # A ring: the pose it marks is the middle, and a filled one
            # would hide it.
            _disc(stage, f"{base}/outer", radius, thickness, rgb, z)
            _disc(stage, f"{base}/inner", radius * 0.72, thickness, floor,
                  z + thickness)
            _disc(stage, f"{base}/pip", radius * 0.18, thickness, rgb,
                  z + thickness * 2)
        elif item["kind"] == "goal":
            _disc(stage, f"{base}/disc", radius, thickness, rgb, z)
        else:
            _disc(stage, f"{base}/disc", radius * 0.55, thickness, rgb, z)
        if "yaw" in item:
            # Which way the robot faces here. At the goal this is the
            # difference between parking and being trapped by its own
            # protective field, so it is worth being able to see.
            _arrow(stage, f"{base}/heading", radius * 2.2, 0.075,
                   thickness, rgb, z + thickness * 2)
        # The plate sits on the aisle side of the point, clear of the
        # ring, where the overhead camera and a person in the aisle
        # both see it the right way up.
        side = _toward_aisle(y)
        _plate(stage, f"{ROOT}/label_{index:02d}", item["text"], rgb,
               (x, y + side * (radius + PLATE_H * 0.5 + 0.08)),
               z + thickness * 3)
        drawn.append(f"{item['text']} at ({x:+.2f}, {y:+.2f})")
    return drawn
