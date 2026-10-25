"""Build the factory bay shell in Isaac Sim.

Isaac Sim only: imports omni and pxr. Never import this from a ROS 2
node.

Shell here: floor, paint, walls and columns. The machines that fill the
work zones are built by isaac_sim/environments/equipment.py, so that
changing a machine cannot move a wall.

Geometry comes from comfort_core/factory_layout.py, which derives it all
from configs/factory.yaml. Nothing in this file decides where anything
goes.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple

import numpy as np
from isaacsim.core.api.objects import FixedCuboid, FixedCylinder, VisualCuboid
from pxr import Gf, Sdf, UsdGeom, UsdLux
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_current_stage

from comfort_core.factory_layout import (BayLayout, Rect, build_layout,
                                        drivable_rects)

from isaac_sim.environments import (equipment, factory_workers, machines,
                                    materials, props)


def _visual(prim_path: str, centre, size, color) -> VisualCuboid:
    """Something to look at, with no collider: paint, mostly."""
    box = VisualCuboid(
        prim_path=prim_path,
        position=np.array([float(centre[0]), float(centre[1]), float(centre[2])]),
        scale=np.array([float(size[0]), float(size[1]), float(size[2])]),
        color=np.array(color, dtype=float),
    )
    materials.fix_extent(box)
    return box


def _solid(prim_path: str, centre, size, color) -> FixedCuboid:
    """Something the robot can hit and the LiDAR can see."""
    box = FixedCuboid(
        prim_path=prim_path,
        position=np.array([float(centre[0]), float(centre[1]), float(centre[2])]),
        scale=np.array([float(size[0]), float(size[1]), float(size[2])]),
        color=np.array(color, dtype=float),
    )
    materials.fix_extent(box)
    return box


def _group(prim_path: str) -> None:
    """Make a group an Xform, not a typeless prim.

    USD will happily create /World/Factory/Ceiling implicitly as a prim
    with no type, and a typeless prim is not Imageable: hiding it for a
    plan render does nothing at all, silently, and the render comes back
    as a picture of the ceiling.
    """
    from isaacsim.core.utils.prims import create_prim, is_prim_path_valid

    if not is_prim_path_valid(prim_path):
        create_prim(prim_path, "Xform")


def _hide_ground(cfg: Dict[str, Any]) -> None:
    """Stop the physics ground plane rendering under the epoxy."""
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid

    path = cfg.get("environment", {}).get("ground", {}).get("prim_path")
    if path and is_prim_path_valid(path):
        UsdGeom.Imageable(get_prim_at_path(path)).MakeInvisible()


def _paint(cfg, prim_path: str, centre, size, material: str,
           yaw_deg: float = 0.0):
    """A flat marking on the floor, with a real finish on it."""
    box = VisualCuboid(
        prim_path=prim_path,
        position=np.array([float(centre[0]), float(centre[1]), float(centre[2])]),
        orientation=props.quat(yaw_deg),
        scale=np.array([float(size[0]), float(size[1]), float(size[2])]),
        color=np.array([1.0, 1.0, 1.0]),
    )
    materials.apply(cfg, box, material)
    return box


def _arrow(cfg, prim_path: str, x: float, y: float, z: float,
           heading_deg: float, size: float = 0.55) -> int:
    """A direction chevron, the kind painted along a traffic lane."""
    for tag, sign in (("l", 1.0), ("r", -1.0)):
        _paint(cfg, f"{prim_path}_{tag}",
               (x - size * 0.28, y + sign * size * 0.28, z / 2.0),
               (size * 0.8, 0.08, z), "epoxy_line",
               yaw_deg=heading_deg + sign * 40.0)
    _paint(cfg, f"{prim_path}_stem", (x + size * 0.18, y, z / 2.0),
           (size * 0.75, 0.08, z), "epoxy_line", yaw_deg=heading_deg)
    return 3


def _stripe(cfg, prim_path: str, x0: float, y0: float, x1: float, y1: float,
            width: float, z: float, material: str = "epoxy_line") -> int:
    """One painted line between two points. Axis-aligned only."""
    if abs(y1 - y0) < 1e-9:
        _paint(cfg, prim_path, ((x0 + x1) / 2.0, y0, z / 2.0),
               (abs(x1 - x0), width, z), material)
    else:
        _paint(cfg, prim_path, (x0, (y0 + y1) / 2.0, z / 2.0),
               (width, abs(y1 - y0), z), material)
    return 1


def _dashed(cfg, prim_path: str, x0: float, y0: float, x1: float, y1: float,
            width: float, z: float, dash: float = 0.25, gap: float = 0.18,
            material: str = "epoxy_line") -> int:
    """A dashed line, the way a parking box is marked out on the floor."""
    length = max(abs(x1 - x0), abs(y1 - y0))
    n = max(int(length // (dash + gap)), 1)
    step = length / n
    drawn = 0
    along_x = abs(y1 - y0) < 1e-9
    for i in range(n):
        start = i * step + (step - dash) / 2.0
        if along_x:
            a = min(x0, x1) + start
            drawn += _stripe(cfg, f"{prim_path}_{i:02d}", a, y0, a + dash, y0,
                             width, z, material)
        else:
            b = min(y0, y1) + start
            drawn += _stripe(cfg, f"{prim_path}_{i:02d}", x0, b, x0, b + dash,
                             width, z, material)
    return drawn


def build_floor(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Green epoxy, wall to wall, with the lanes painted on it.

    This is what the floor of the plant actually looks like: one glossy
    green coat over the whole bay, lanes marked with a double yellow
    line, and parking for anything that stands in a work zone marked
    out with a dashed box. Nothing is a different colour because it is
    a different kind of space -- the paint does that work.

    Markings sit 4 mm proud of the slab. The scan slice starts at
    0.11 m (ros.scan.min_height in common.yaml), so paint cannot become
    an obstacle, the same rule the Comfort Space visualisation follows.
    """
    f = cfg["factory"]
    paint = f.get("paint", {})
    if not paint.get("enabled", True):
        return 0

    thickness = float(f["floor_thickness"])
    z = float(paint.get("line_height", 0.004)) + float(paint.get("slab_lift", 0.004))
    line_width = float(paint.get("line_width", 0.10))
    outer = float(paint.get("outer_line_width", 0.05))
    offset = float(paint.get("double_line_gap", 0.10))

    # The slab's top face sits a few millimetres above z = 0. The
    # physics ground plane is at z = 0 and coplanar faces fight: the
    # grey ground showed through the green in patches and the floor
    # came out looking like a washed-out texture.
    _group(f"{root}/Floor")
    lift = float(paint.get("slab_lift", 0.004))
    slab = _visual(f"{root}/Floor/slab", (0.0, 0.0, lift - thickness / 2.0),
                   (layout.length, layout.width, thickness), (1.0, 1.0, 1.0))
    materials.apply(cfg, slab, "epoxy_green")
    drawn = 1
    _hide_ground(cfg)

    # Lane edges: a 10 cm line with a thinner one just outside it, which
    # is how the aisles in the video are marked.
    for band in layout.aisles:
        for edge, y, away in (("s", band.y_min, -1.0), ("n", band.y_max, +1.0)):
            drawn += _stripe(cfg, f"{root}/Floor/{band.name}_line_{edge}",
                             layout.x_min, y, layout.x_max, y, line_width, z)
            drawn += _stripe(cfg, f"{root}/Floor/{band.name}_outer_{edge}",
                             layout.x_min, y + away * offset,
                             layout.x_max, y + away * offset, outer, z)

    for cross in layout.crosses:
        for edge, x in (("w", cross.x_min), ("e", cross.x_max)):
            drawn += _stripe(cfg, f"{root}/Floor/{cross.name}_line_{edge}",
                             x, cross.y_min, x, cross.y_max, line_width, z)

    # Refuges: a dashed box, not a filled patch. A pull-off is marked
    # out on a real floor, not painted in.
    for refuge in layout.refuges:
        drawn += _dashed(cfg, f"{root}/Floor/{refuge.name}_n",
                         refuge.x_min, refuge.y_max, refuge.x_max, refuge.y_max,
                         outer, z)
        drawn += _dashed(cfg, f"{root}/Floor/{refuge.name}_s",
                         refuge.x_min, refuge.y_min, refuge.x_max, refuge.y_min,
                         outer, z)
        drawn += _dashed(cfg, f"{root}/Floor/{refuge.name}_w",
                         refuge.x_min, refuge.y_min, refuge.x_min, refuge.y_max,
                         outer, z)
        drawn += _dashed(cfg, f"{root}/Floor/{refuge.name}_e",
                         refuge.x_max, refuge.y_min, refuge.x_max, refuge.y_max,
                         outer, z)

    # Cart staging: a solid box, not a dashed one. A pull-off is a
    # courtesy marking; a parking block for carts is a hard boundary,
    # and on a real floor it is painted as one.
    for zone in layout.cart_zones:
        for name, x0, y0, x1, y1 in (
                ("n", zone.x_min, zone.y_max, zone.x_max, zone.y_max),
                ("s", zone.x_min, zone.y_min, zone.x_max, zone.y_min),
                ("w", zone.x_min, zone.y_min, zone.x_min, zone.y_max),
                ("e", zone.x_max, zone.y_min, zone.x_max, zone.y_max)):
            drawn += _stripe(cfg, f"{root}/Floor/{zone.name}_{name}",
                             x0, y0, x1, y1, line_width, z,
                             material="epoxy_line_dark")
    # The slot divisions inside each block. Six marked bays read as six
    # places to put something; one big rectangle reads as a region, and
    # then nobody can tell from the floor whether a gap is a free slot
    # or just the space between two badly parked carts.
    #
    # Only the dividers between slots are drawn -- the block outline
    # above already has the perimeter.
    for slot in layout.cart_slots_taken + layout.cart_slots_free:
        block = next((b for b in layout.cart_zones
                      if b.x_min <= slot.x_min + 1e-6
                      and slot.x_max <= b.x_max + 1e-6
                      and b.y_min <= slot.y_min + 1e-6
                      and slot.y_max <= b.y_max + 1e-6), None)
        if block is None:
            continue
        if slot.x_max < block.x_max - 1e-6:
            drawn += _stripe(cfg, f"{root}/Floor/{slot.name}_div_e",
                             slot.x_max, slot.y_min, slot.x_max, slot.y_max,
                             line_width, z, material="epoxy_line_dark")
        if slot.y_max < block.y_max - 1e-6:
            drawn += _stripe(cfg, f"{root}/Floor/{slot.name}_div_n",
                             slot.x_min, slot.y_max, slot.x_max, slot.y_max,
                             line_width, z, material="epoxy_line_dark")
        if slot.y_min > block.y_min + 1e-6:
            drawn += _stripe(cfg, f"{root}/Floor/{slot.name}_div_s",
                             slot.x_min, slot.y_min, slot.x_max, slot.y_min,
                             line_width, z, material="epoxy_line_dark")
    # Direction arrows along each lane, every few metres.
    if paint.get("arrows", True):
        step = float(paint.get("arrow_spacing", 5.0))
        for band in layout.aisles:
            count = max(int(layout.length // step), 1)
            for i in range(count):
                x = layout.x_min + (i + 0.5) * (layout.length / count)
                drawn += _arrow(cfg, f"{root}/Floor/arrow_{band.name}_{i:02d}",
                                x, band.centre, z, 0.0)

    # Machines get a dashed outline too, so the floor says where
    # everything belongs even when it has been wheeled away.
    if paint.get("station_outlines", True):
        for station in layout.stations:
            box = station.footprint
            if min(box.size) < 0.4:
                continue
            drawn += _dashed(cfg, f"{root}/Floor/{station.name}_front",
                             box.x_min, box.y_min, box.x_max, box.y_min,
                             outer, z, material="epoxy_line_dark")
            drawn += _dashed(cfg, f"{root}/Floor/{station.name}_back",
                             box.x_min, box.y_max, box.x_max, box.y_max,
                             outer, z, material="epoxy_line_dark")
    return drawn


def build_shell(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Perimeter walls and the column grid."""
    shell = cfg.get("assets", {}).get("shell", {})
    if props.enabled(cfg) and "wall" in shell:
        return _shell_from_assets(cfg, layout, root, shell)
    f = cfg["factory"]
    h = float(f["wall_height"])
    t = float(f["wall_thickness"])
    colour = (0.78, 0.78, 0.80)

    built = 0
    for name, centre, size in (
        ("wall_south", (0.0, layout.y_min - t / 2.0, h / 2.0),
         (layout.length + 2 * t, t, h)),
        ("wall_north", (0.0, layout.y_max + t / 2.0, h / 2.0),
         (layout.length + 2 * t, t, h)),
        ("wall_west", (layout.x_min - t / 2.0, 0.0, h / 2.0),
         (t, layout.width, h)),
        ("wall_east", (layout.x_max + t / 2.0, 0.0, h / 2.0),
         (t, layout.width, h)),
    ):
        _solid(f"{root}/Shell/{name}", centre, size, colour)
        built += 1

    size = float(f["column_size"])
    height = float(f["column_height"])
    for i, (x, y) in enumerate(layout.columns, start=1):
        _solid(f"{root}/Shell/column_{i:02d}", (x, y, height / 2.0),
               (size, size, height), (0.62, 0.62, 0.66))
        built += 1
    return built


def _shell_from_assets(cfg: Dict[str, Any], layout: BayLayout, root: str,
                       shell: Dict[str, Any]) -> int:
    """Warehouse wall panels, pillars and light fittings.

    The panels are 6 m wide, which is why the bay is a whole number of
    them: two a side. Their inner faces land on the bay bounds, so the
    free width is still exactly what configs/factory.yaml says.
    """
    _group(f"{root}/Shell")
    wall = dict(shell["wall"])
    span = float(wall.get("length", 6.0))
    half_t = float(wall.get("thickness", 0.2)) / 2.0
    # Walls run to the ceiling. Left at the panel's own 3.1 m they
    # stopped short of the 5 m deck and daylight poured through the gap,
    # which is not what the inside of a plant looks like.
    ceiling = cfg["factory"].get("ceiling", {})
    if ceiling.get("enabled", True):
        wall["scale_z_ratio"] = (float(ceiling.get("height", 5.0)) + 0.2) \
            / float(wall.get("height", 3.1))
    built = 0

    # Round up, not to nearest: a 7.5 m end wall rounded to one 6 m
    # panel and left a metre and a half of hole, which with the sky
    # hidden renders as a black slot.
    n = max(math.ceil(layout.length / span - 1e-6), 1)
    step = layout.length / n
    for i in range(n):
        along = layout.x_min + step * (i + 0.5)
        for name, y, yaw in (("wall_s", layout.y_min - half_t, 90.0),
                             ("wall_n", layout.y_max + half_t, -90.0)):
            path = f"{root}/Shell/{name}_{i}"
            props.reference(path, wall, (along, y, 0.0), yaw_deg=yaw)
            materials.bind_over(cfg, path, "wall_cream")
        built += 2
    m = max(math.ceil(layout.width / span - 1e-6), 1)
    step = layout.width / m
    for j in range(m):
        along = layout.y_min + step * (j + 0.5)
        for name, x, yaw in (("wall_w", layout.x_min - half_t, 180.0),
                             ("wall_e", layout.x_max + half_t, 0.0)):
            path = f"{root}/Shell/{name}_{j}"
            props.reference(path, wall, (x, along, 0.0), yaw_deg=yaw)
            materials.bind_over(cfg, path, "wall_cream")
        built += 2

    pillar = shell.get("pillar")
    if pillar:
        # The model is a nine metre hall pillar; this bay is not that
        # tall, so it is cut down to the configured column height.
        ratio = float(cfg["factory"]["column_height"]) / float(pillar["natural"][2])
        spec = dict(pillar, scale_z_ratio=ratio / float(pillar.get("scale", 1.0)))
        for i, (x, y) in enumerate(layout.columns, start=1):
            path = f"{root}/Shell/column_{i:02d}"
            props.reference(path, spec, (x, y, 0.0))
            materials.bind_over(cfg, path, "column_cream")
            built += 1

    # Hazard banding round the foot of every column, which is on every
    # column in the building and reads from across the bay.
    size = float(cfg["factory"]["column_size"]) + 0.015
    for i, (x, y) in enumerate(layout.columns, start=1):
        for k in range(4):
            band = _visual(f"{root}/Shell/hazard_{i:02d}_{k}",
                           (x, y, 0.16 + k * 0.22), (size, size, 0.22),
                           (1.0, 1.0, 1.0))
            materials.apply(cfg, band,
                            "epoxy_line" if k % 2 == 0 else "hazard_black")
            built += 1

    lamp = shell.get("lamp")
    if lamp:
        for i, (x, y) in enumerate(lamp.get("at", []), start=1):
            props.reference(f"{root}/Shell/lamp_{i:02d}", lamp,
                            (float(x), float(y), float(lamp.get("height", 4.2))))
            built += 1

    ceiling = shell.get("ceiling", {})
    if ceiling.get("enabled", False):
        tile = float(ceiling.get("tile", 6.0))
        for i in range(max(int(round(layout.length / tile)), 1)):
            for j in range(max(int(round(layout.width / tile)), 1)):
                props.reference(
                    f"{root}/Shell/ceiling_{i}{j}", ceiling,
                    (layout.x_min + tile * (i + 0.5), layout.y_min + tile * (j + 0.5),
                     float(ceiling.get("height", 4.5))))
                built += 1
    return built


def build_ceiling(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Deck, beams, ducts and the rows of lights under them.

    A bay with nothing overhead reads as a film set: in every shot of
    the real floor the ceiling is right there, with linear fittings
    down the aisles and ducting across them. The fittings are geometry
    only -- the lights that do the work are in the build script -- but
    a bright aisle under a dark fitting is the first thing that looks
    wrong.

    Everything lives under {root}/Ceiling so a plan render can hide it.
    """
    spec = cfg["factory"].get("ceiling", {})
    if not spec.get("enabled", True):
        return 0
    h = float(spec.get("height", 5.0))
    _group(f"{root}/Ceiling")
    made = 0

    deck = _visual(f"{root}/Ceiling/deck", (0.0, 0.0, h + 0.06),
                   (layout.length, layout.width, 0.12), (1.0, 1.0, 1.0))
    materials.apply(cfg, deck, "ceiling_white")
    made += 1

    # Roof beams across the short way, on the column pitch.
    pitch = float(cfg["factory"]["column_pitch"])
    n = max(int(round(layout.length / pitch)), 1)
    for i in range(n + 1):
        x = layout.x_min + i * pitch
        beam = _visual(f"{root}/Ceiling/beam_{i:02d}", (x, 0.0, h - 0.18),
                       (0.26, layout.width, 0.36), (1.0, 1.0, 1.0))
        materials.apply(cfg, beam, "steel_light")
        made += 1

    # Purlins the short way between the beams, so the deck is not one
    # blank plane when you look up.
    step = pitch / 3.0
    k = 0
    for i in range(int(layout.length / step) + 1):
        x = layout.x_min + i * step
        if abs((x - layout.x_min) % pitch) < 1e-6:
            continue
        k += 1
        purlin = _visual(f"{root}/Ceiling/purlin_{k:02d}", (x, 0.0, h - 0.06),
                         (0.10, layout.width, 0.14), (1, 1, 1))
        materials.apply(cfg, purlin, "ceiling_white")
        made += 1

    # Ducting over the work zones, the way it runs in the plant.
    for i, band in enumerate([b for b in layout.bands if b.kind == "zone"], start=1):
        if band.depth < 2.0:
            continue
        duct = FixedCylinder(
            prim_path=f"{root}/Ceiling/duct_{i:02d}",
            position=np.array([0.0, band.centre, h - 0.62]),
            orientation=np.array([0.7071068, 0.0, 0.7071068, 0.0]),
            radius=float(spec.get("duct_radius", 0.22)),
            height=layout.length,
        )
        duct.set_collision_enabled(False)
        materials.apply(cfg, duct, "aluminium")
        made += 1

    # Pendant fittings down each aisle and over each work zone. The
    # library has the fitting, chains and all, so it is referenced
    # rather than drawn; the light itself is a rect light in the build
    # script, since geometry does not illuminate anything.
    fitting = cfg.get("assets", {}).get("fitting")
    if not (props.enabled(cfg) and fitting):
        return made
    spacing = float(spec.get("fitting_spacing", 2.6))
    rows = [b.centre for b in layout.bands if b.kind == "aisle"]
    rows += [b.centre for b in layout.bands if b.kind == "zone" and b.depth >= 2.0]
    count = max(int(layout.length // spacing), 1)
    for r, y in enumerate(rows, start=1):
        for i in range(count):
            x = layout.x_min + (i + 0.5) * (layout.length / count)
            props.reference(f"{root}/Ceiling/fitting_{r}_{i:02d}", fitting,
                            (x, y, h - 0.02))
            made += 1
    return made


def build_ceiling_lights(cfg: Dict[str, Any], layout: BayLayout,
                        root: str) -> int:
    """The lights that actually light the bay, one per row of fittings.

    These used to live in the build script, which meant the saved USD
    was lit and the running simulation was not: scene_setup only adds a
    dome and a sun, and the sun cannot get through a ceiling. The bay
    came up almost black in the viewport and could only be looked at
    with the camera light on.

    They belong with the fittings they represent, so they are here, and
    both the saved file and the live simulation get exactly one set.
    """
    spec = cfg["factory"].get("ceiling", {})
    if not spec.get("enabled", True):
        return 0
    rig = spec.get("lights", {})
    if not rig.get("enabled", True):
        return 0
    height = float(spec.get("height", 5.0))
    drop = float(spec.get("fitting_drop", 0.45))

    rows = [b.centre for b in layout.bands if b.kind == "aisle"]
    rows += [b.centre for b in layout.bands
             if b.kind == "zone" and b.depth >= 2.0]
    stage = get_current_stage()
    _group(f"{root}/Lights")
    made = 0
    for i, y in enumerate(sorted(rows), start=1):
        path = f"{root}/Lights/row_{i:02d}"
        rect = UsdLux.RectLight.Define(stage, Sdf.Path(path))
        rect.CreateWidthAttr(layout.length * 0.95)
        rect.CreateHeightAttr(float(rig.get("width", 1.1)))
        rect.CreateIntensityAttr(float(rig.get("intensity", 2600.0)))
        rect.CreateColorAttr((1.0, 0.98, 0.95))
        # Not normalised: these are long sources and the bay should be
        # as evenly lit as the real one, which is very.
        rect.CreateNormalizeAttr(False)
        xform = UsdGeom.Xformable(rect.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, y, height - drop - 0.05))
        made += 1
    return made


def build_walls_fittings(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """What is on the walls: glazing, cable tray, pipework, boards.

    Bare walls are what gave the first renders away. A plant wall has a
    strip of glazing up near the roof, a cable tray under it, service
    pipes along it and a notice board at head height, and all of it is
    what the eye uses to judge how big the room is.
    """
    spec = cfg["factory"].get("wall_fittings", {})
    if not spec.get("enabled", True):
        return 0
    ceiling = float(cfg["factory"].get("ceiling", {}).get("height", 5.0))
    _group(f"{root}/Walls")
    made = 0

    glass_z = float(spec.get("glazing_height", ceiling - 1.1))
    glass_h = float(spec.get("glazing_depth", 0.9))
    tray_z = float(spec.get("tray_height", ceiling - 1.9))
    for tag, y in (("s", layout.y_min + 0.06), ("n", layout.y_max - 0.06)):
        glass = _visual(f"{root}/Walls/glazing_{tag}", (0.0, y, glass_z),
                        (layout.length - 1.0, 0.05, glass_h), (1.0, 1.0, 1.0))
        glass.apply_visual_material(
            materials.emissive(cfg, "glazing", (0.86, 0.92, 1.0), 2200.0))
        frame = _visual(f"{root}/Walls/glazing_frame_{tag}", (0.0, y, glass_z),
                        (layout.length - 0.94, 0.07, glass_h + 0.08), (1, 1, 1))
        materials.apply(cfg, frame, "aluminium_dark")
        tray = _visual(f"{root}/Walls/tray_{tag}", (0.0, y, tray_z),
                       (layout.length - 0.6, 0.22, 0.10), (1, 1, 1))
        materials.apply(cfg, tray, "tray_grey")
        made += 3
        for i, (radius, colour, dz) in enumerate(((0.07, "pipe_grey", -0.28),
                                                  (0.05, "pipe_red", -0.46))):
            pipe = FixedCylinder(
                prim_path=f"{root}/Walls/pipe_{tag}_{i}",
                position=np.array([0.0, y + 0.10, tray_z + dz]),
                orientation=np.array([0.7071068, 0.0, 0.7071068, 0.0]),
                radius=radius, height=layout.length - 0.4)
            pipe.set_collision_enabled(False)
            materials.apply(cfg, pipe, colour)
            made += 1

    # Notice boards and a first-aid style sign, at head height.
    for i, (x, y, tag) in enumerate(((-layout.length * 0.28, layout.y_max - 0.09, "n"),
                                     (layout.length * 0.30, layout.y_min + 0.09, "s")),
                                    start=1):
        face = y + (0.03 if tag == "s" else -0.03)
        board = _visual(f"{root}/Walls/board_{i}", (x, y, 1.70),
                        (1.24, 0.04, 0.89), (1, 1, 1))
        materials.apply(cfg, board, "aluminium_dark")
        sheet = _visual(f"{root}/Walls/board_paper_{i}", (x, face, 1.70),
                        (1.16, 0.01, 0.81), (1, 1, 1))
        materials.apply(cfg, sheet, "machine_white")
        # Printed sheets pinned to it, which is what makes a board a
        # board rather than a white rectangle.
        for k, (dx, dz, colour) in enumerate((
                (-0.38, 0.17, "paper"), (-0.02, 0.17, "tote_blue"),
                (0.34, 0.17, "paper"), (-0.30, -0.18, "light_amber"),
                (0.10, -0.18, "paper"), (0.40, -0.16, "paper"))):
            pin = _visual(f"{root}/Walls/board_sheet_{i}_{k}",
                          (x + dx, face + (0.004 if tag == "s" else -0.004),
                           1.70 + dz), (0.30, 0.004, 0.30), (1, 1, 1))
            materials.apply(cfg, pin, colour)
            made += 1
        sign = _visual(f"{root}/Walls/sign_{i}", (x + 1.1, y, 2.20),
                       (0.42, 0.03, 0.30), (1, 1, 1))
        materials.apply(cfg, sign, "sign_green")
        made += 3

    # A mezzanine office along one end, which is what the blank end
    # wall in the first renders was missing.
    mez = spec.get("mezzanine", {})
    if mez.get("enabled", True):
        depth = float(mez.get("depth", 1.3))
        floor_z = float(mez.get("height", 3.2))
        x = layout.x_min + depth / 2.0
        deck = _visual(f"{root}/Walls/mez_deck", (x, 0.0, floor_z),
                       (depth, layout.width * 0.7, 0.18), (1, 1, 1))
        materials.apply(cfg, deck, "machine_white")
        rail = _visual(f"{root}/Walls/mez_rail", (x + depth / 2.0 - 0.04, 0.0,
                                                  floor_z + 0.55),
                       (0.06, layout.width * 0.7, 1.0), (1, 1, 1))
        rail.apply_visual_material(
            materials.translucent(cfg, "rail", (0.86, 0.92, 0.95), 0.25))
        # The posts hold the deck up, and one of them stood in the
        # cross aisle: the robot reversed into its pull-off, met a
        # 120 mm post that the map knew nothing about, and the
        # collision monitor froze it in the lane. Anything standing in
        # the space the robot drives in is skipped.
        posts = 0
        keep_clear = drivable_rects(layout)
        for i in range(4):
            y = -layout.width * 0.3 + i * (layout.width * 0.6 / 3.0)
            px = x + depth / 2.0 - 0.1
            if any(r.x_min - 0.2 < px < r.x_max + 0.2
                   and r.y_min - 0.2 < y < r.y_max + 0.2 for r in keep_clear):
                continue
            post = _visual(f"{root}/Walls/mez_post_{i}", (px, y, floor_z / 2.0),
                           (0.12, 0.12, floor_z), (1, 1, 1))
            materials.apply(cfg, post, "aluminium_dark")
            posts += 1
        glass = _visual(f"{root}/Walls/mez_glass", (x, 0.0, floor_z + 1.3),
                        (0.06, layout.width * 0.6, 1.1), (1, 1, 1))
        glass.apply_visual_material(
            materials.emissive(cfg, "office", (0.92, 0.94, 0.98), 1400.0))
        made += 3 + posts

    # Extractor fans high on the end wall.
    for i, y in enumerate((-layout.width * 0.22, layout.width * 0.22), start=1):
        housing = _visual(f"{root}/Walls/fan_{i}", (layout.x_max - 0.10, y, 3.9),
                          (0.14, 0.9, 0.9), (1, 1, 1))
        materials.apply(cfg, housing, "aluminium_dark")
        for k in range(4):
            blade = _visual(f"{root}/Walls/fan_{i}_b{k}",
                            (layout.x_max - 0.16, y, 3.9), (0.03, 0.72, 0.16),
                            (1, 1, 1))
            UsdGeom.Xformable(blade.prim).AddRotateXOp().Set(45.0 * k)
            materials.apply(cfg, blade, "tray_grey")
        made += 5

    # A roller shutter at one end, where the material comes in.
    door_w = min(3.2, layout.width * 0.4)
    door = _visual(f"{root}/Walls/shutter", (layout.x_max - 0.08, 0.0, 1.6),
                   (0.10, door_w, 3.2), (1, 1, 1))
    materials.apply(cfg, door, "door_grey")
    rail = _visual(f"{root}/Walls/shutter_head", (layout.x_max - 0.08, 0.0, 3.32),
                   (0.18, door_w + 0.3, 0.26), (1, 1, 1))
    materials.apply(cfg, rail, "aluminium_dark")
    made += 2
    return made


def _clutter_clear(layout: BayLayout, label: str, x: float, y: float,
                   size) -> None:
    """Nothing parked may stand in an aisle, a crossing or a refuge.

    A refuge with a trolley in it is not a refuge, and the robot finds
    that out by driving at it and giving up: both B2 and B3 deadlocked
    on a pull-off that had a crate stack in it. Cheap to check here,
    expensive to debug from a trial log.
    """
    half_x, half_y = size[0] / 2.0, size[1] / 2.0
    box = Rect(label, x - half_x, x + half_x, y - half_y, y + half_y)
    for keep in layout.in_aisle_space + list(layout.refuges):
        if (box.x_min < keep.x_max - 1e-6 and box.x_max > keep.x_min + 1e-6
                and box.y_min < keep.y_max - 1e-6
                and box.y_max > keep.y_min + 1e-6):
            raise ValueError(
                f"{label} at ({x:+.2f}, {y:+.2f}) stands in {keep.name}; "
                f"the robot has to be able to pull off there")


def build_clutter(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Trolleys parked in the zones and stock stacked on the floor.

    A work zone with nothing in it but its machine is the giveaway
    that a scene was laid out rather than used. These are listed in the
    config, one line each, and checked against the aisles, the
    crossings and the refuges like any other prop.
    """
    spec = cfg["factory"].get("clutter", {})
    if not spec.get("enabled", True):
        return 0
    _group(f"{root}/Clutter")
    made = 0
    for i, item in enumerate(spec.get("trolleys", []), start=1):
        x, y = float(item[0]), float(item[1])
        yaw = float(item[2]) if len(item) > 2 else 0.0
        kind_ = str(item[3]) if len(item) > 3 else "trolley"
        reach = (1.15, 0.80) if kind_ == "panel_cart" else (1.15, 0.70)
        if abs(yaw) % 180.0 == 90.0:
            reach = (reach[1], reach[0])
        _clutter_clear(layout, f"factory.clutter.trolleys[{i}]", x, y, reach)
        prim = f"{root}/Clutter/trolley_{i:02d}"
        create_prim(prim, "Xform", position=[x, y, 0.0],
                    orientation=machines_quat(yaw))
        kind = str(item[3]) if len(item) > 3 else "trolley"
        size = (1.10, 0.75, 1.15) if kind == "panel_cart" else (1.10, 0.65, 1.00)
        made += machines.build(cfg, kind, prim, size)
    for i, item in enumerate(spec.get("stacks", []), start=1):
        x, y = float(item[0]), float(item[1])
        _clutter_clear(layout, f"factory.clutter.stacks[{i}]", x, y, (0.60, 0.50))
        prim = f"{root}/Clutter/stack_{i:02d}"
        create_prim(prim, "Xform", position=[x, y, 0.0],
                    orientation=machines_quat(float(item[2]) if len(item) > 2 else 0.0))
        made += machines.stack(cfg, prim, int(item[3]) if len(item) > 3 else 3)
    return made


def build_cart_zones(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Stand one parts cart in each taken slot of each staging block.

    Which slots are taken is the layout's decision, not this
    function's: `comfort_core.factory_layout` fills the back row first
    so that the empty slots end up on the aisle frontage, where the
    robot can pull into one. Here they just get drawn.

    A cart is centred in its slot. It is 1.26 x 0.85 m in a
    1.40 x 1.05 m slot, so the paint still shows either side of it --
    which is how a slot reads as a slot rather than as a cart-shaped
    hole.
    """
    slots = layout.cart_slots_taken
    if not slots:
        return 0
    _group(f"{root}/Carts")
    # The same cart the AMR tows, from the same module entry.
    dims = cfg.get("equipment", {}).get("modules", {}).get(
        "cart", {"length": 1.10, "depth": 0.65, "height": 1.00})
    length = float(dims["length"])
    depth = float(dims["depth"])
    height = float(dims["height"])
    for index, slot in enumerate(slots):
        span_x, span_y = slot.size
        if span_x < length - 1e-6 or span_y < depth - 1e-6:
            raise ValueError(
                f"slot {slot.name} is {span_x:.2f} x {span_y:.2f} m and the "
                f"cart is {length:.2f} x {depth:.2f} m; widen cart_bays.slot")
        x, y = slot.centre
        prim = f"{root}/Carts/{slot.name}"
        create_prim(prim, "Xform", position=[x, y, 0.0],
                    orientation=machines_quat(0.0))
        machines.build(cfg, "cart", prim, (length, depth, height))
        loaded = _load_cart(cfg, prim, index, (length, depth, height))
        _cart_collider(prim, slot.centre, (length, depth, height), loaded)
    return len(slots)


def _cart_collider(prim: str, centre, size, loaded: int) -> None:
    """An invisible box that makes the cart solid.

    The library dolly is drawn geometry and nothing else: measured, it
    carries no collision prims at all. The LiDAR sees it, because the
    LiDAR renders, and the occupancy map marks its slot, because the
    map is built from the layout -- so everything LOOKED right while
    the AMR drove straight through a parked cart.
    """
    length, depth, height = size
    # Up to the top of the load when there is one. An empty frame is
    # 0.44 m; a cart with a pallet of cases on it is over a metre, and
    # a robot that can pass under the load is not stopped by it.
    tall = height + (0.59 if loaded else 0.0)
    body = FixedCuboid(
        prim_path=f"{prim}/collider",
        position=np.array([float(centre[0]), float(centre[1]), tall / 2.0]),
        scale=np.array([length, depth, tall]),
    )
    UsdGeom.Imageable(body.prim).MakeInvisible()


def _load_cart(cfg: Dict[str, Any], prim: str, index: int,
               deck: Tuple[float, float, float]) -> int:
    """Put parts on one cart, out of the asset library.

    A cart standing empty in every slot says the block is a car park,
    not a staging area, and the LiDAR sees an open frame where a real
    one would be full of bins. The patterns come from the config and
    repeat in order down the list of slots, so what is on a given cart
    is the same in every run -- randomised scenery would mean two runs
    of one scenario were not the same scenario.

    The load sits on the deck, inset from its edges, laid out as a
    cols x rows grid. Anything that does not fit is a config mistake
    and says so here rather than clipping through the frame in a
    render nobody looks at closely.
    """
    if not props.enabled(cfg):
        return 0
    patterns = cfg["factory"].get("cart_bays", {}).get("loads", [])
    if not patterns:
        return 0
    item = patterns[index % len(patterns)]
    kind = item.get("prop")
    if not kind:
        return 0
    spec = cfg.get("assets", {}).get("modules", {}).get(kind)
    if spec is None:
        raise ValueError(f"cart load {kind!r} has no assets.modules entry")

    yaw = float(item.get("yaw_deg", 0.0))
    size_x, size_y, _size_z = props.placed_size(spec)
    if abs(yaw) % 180.0 == 90.0:
        size_x, size_y = size_y, size_x
    cols, rows = int(item.get("cols", 1)), int(item.get("rows", 1))
    # Off the edge of the deck, so nothing overhangs.
    margin = float(item.get("margin", 0.06))
    free_x = deck[0] - 2.0 * margin
    free_y = deck[1] - 2.0 * margin
    if cols * size_x > free_x + 1e-6 or rows * size_y > free_y + 1e-6:
        raise ValueError(
            f"{cols} x {rows} of {kind} is "
            f"{cols * size_x:.2f} x {rows * size_y:.2f} m and the cart deck "
            f"has {free_x:.2f} x {free_y:.2f} m")
    pitch_x, pitch_y = free_x / cols, free_y / rows
    z = deck[2] + float(spec.get("base_offset", 0.0))
    made = 0
    for row in range(rows):
        for col in range(cols):
            dx = -free_x / 2.0 + (col + 0.5) * pitch_x
            dy = -free_y / 2.0 + (row + 0.5) * pitch_y
            props.reference(f"{prim}/load_r{row + 1}c{col + 1}", spec,
                            (dx, dy, z), yaw_deg=yaw)
            made += 1
    return made


def machines_quat(yaw_deg: float):
    return props.quat(yaw_deg)


def build_dressing(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Cones, crates, an extinguisher: what a shop floor has lying about.

    Checked against the aisles, because a cone left in the aisle is an
    obstacle the experiment never agreed to.
    """
    if not props.enabled(cfg):
        return 0
    aisles = layout.aisles
    _group(f"{root}/Dressing")
    made = 0
    for i, item in enumerate(cfg.get("assets", {}).get("dressing", []), start=1):
        x, y = (float(v) for v in item["at"])
        if any(b.y_min < y < b.y_max for b in aisles):
            raise ValueError(
                f"assets.dressing[{i}] sits at ({x}, {y}), which is in an aisle")
        props.reference(f"{root}/Dressing/prop_{i:02d}",
                        {"path": item["path"], "pivot": item.get("pivot", (0.0, 0.0)),
                         "scale": item.get("scale", 1.0)},
                        (x, y, float(item.get("z", 0.0))),
                        yaw_deg=float(item.get("yaw_deg", 0.0)))
        made += 1
    return made


def build_markers(cfg: Dict[str, Any], layout: BayLayout, root: str) -> int:
    """Named prims for the refuges and the ends of the run.

    They are Xforms with a flat visual patch, 2 cm proud of the floor --
    well under the 0.11 m the scan slice starts at, so they cannot
    become obstacles. Their point is to be selectable: drag
    /World/Factory/Waypoints/goal somewhere else and
    read_factory_scene.py can pick the new position up.
    """
    made = 0
    for waypoint in layout.waypoints:
        prim = f"{root}/Waypoints/{waypoint.name}"
        create_prim(prim, "Xform", position=[waypoint.x, waypoint.y, 0.0])
        _visual(f"{prim}/marker", (waypoint.x, waypoint.y, 0.01),
                (0.45, 0.45, 0.02), (0.20, 0.45, 0.85))
        made += 1
    for refuge in layout.refuges:
        cx, cy = refuge.centre
        create_prim(f"{root}/Refuges/{refuge.name}", "Xform",
                    position=[cx, cy, 0.0])
        made += 1
    return made


def build_bay(cfg: Dict[str, Any], say=None, workers: bool = True) -> BayLayout:
    """Build the whole bay. Returns the layout, for whoever fills it.

    `workers` draws the block figures that make the saved scene read on
    its own. An experiment passes False: its people come from the
    `humans` section, with the collider, the Comfort Space and the /tf
    frame that go with them.
    """
    layout = build_layout(cfg)
    root = str(cfg["factory"]["root_prim"])
    painted = build_floor(cfg, layout, root)
    built = build_shell(cfg, layout, root)
    if say:
        say(f"factory shell: {built} walls and columns, {painted} floor pieces")
    carts = build_cart_zones(cfg, layout, root)
    if say and carts:
        say(f"cart staging: {carts} parts carts in "
            f"{len(layout.cart_zones)} block(s), "
            f"{len(layout.cart_slots_free)} slot(s) left free to yield into")
    clutter = build_clutter(cfg, layout, root)
    if clutter and say:
        say(f"clutter: {clutter} pieces parked")
    fittings = build_walls_fittings(cfg, layout, root)
    if fittings and say:
        say(f"walls: {fittings} fittings, glazing and services")
    roof = build_ceiling(cfg, layout, root)
    roof += build_ceiling_lights(cfg, layout, root)
    if roof and say:
        say(f"ceiling: {roof} pieces including light fittings")
    dressed = build_dressing(cfg, layout, root)
    if dressed and say:
        say(f"dressing: {dressed} props")
    equipment.build_equipment(cfg, layout, root, say=say)
    if workers:
        factory_workers.build_workers(cfg, layout, root, say=say)
    build_markers(cfg, layout, root)
    return layout
