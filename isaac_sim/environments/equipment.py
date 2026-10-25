"""The machines that fill the work zones.

Isaac Sim only: imports omni and pxr. Never import this from a ROS 2
node. Where each machine goes is decided in
comfort_core/factory_layout.py; this file only builds them.

Each module is a handful of boxes, detailed enough to be recognisable
and to give the LiDAR something honest to hit at 0.31 m -- bench legs
and pallet blocks read very differently from a solid wall, and that
difference is what the costmap sees.

Physics is kept cheap: the detail is visual, and each module gets one
invisible box collider around what the robot must not drive into. The
RTX LiDAR raytraces rendered geometry, so the scan still sees every
leg and rail.

> Generic equipment at generic sizes. Not a reproduction of any
> particular machine on the site.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Tuple

import numpy as np
from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
from isaacsim.core.utils.prims import create_prim
from pxr import UsdGeom

from comfort_core.factory_layout import BayLayout, Station
from isaac_sim.environments import machines, materials, props

Vec3 = Tuple[float, float, float]

# Shared palette. Config can override any of these by name.
PALETTE: Dict[str, Vec3] = {
    "frame": (0.42, 0.46, 0.52),      # painted steel
    "surface": (0.72, 0.74, 0.76),    # bench tops, panels
    "machine": (0.30, 0.42, 0.55),    # machine bodies
    "belt": (0.16, 0.17, 0.19),       # conveyor belt
    "guard": (0.88, 0.72, 0.12),      # safety yellow
    "box": (0.72, 0.58, 0.40),        # cardboard
    "bin": (0.20, 0.35, 0.60),        # parts bins
    "pallet": (0.62, 0.50, 0.34),     # timber
    "screen": (0.10, 0.12, 0.14),     # HMI
    "light_red": (0.80, 0.12, 0.10),
    "light_amber": (0.90, 0.60, 0.05),
    "light_green": (0.15, 0.70, 0.25),
}


@dataclass(frozen=True)
class Part:
    """One box in a module's local frame.

    Local frame: +x along the length, +y out of the front face, z up
    from the floor. The front is what the worker stands at.
    """
    name: str
    centre: Vec3
    size: Vec3
    color: str = "frame"


def _angle(station: Station) -> float:
    """How far the module's local frame is turned from the world frame.

    The local frame has the front at +y, so a station facing +y (yaw =
    pi/2) is not turned at all. Offsets and the prims themselves have
    to use this same angle: turning one and not the other leaves every
    box ninety degrees out, which is what it looks like.
    """
    return station.yaw - math.pi / 2.0


def _quat(angle: float) -> np.ndarray:
    """wxyz for a rotation about z."""
    return np.array([math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)])


def _colour(cfg: Dict[str, Any], name: str) -> np.ndarray:
    override = cfg.get("equipment", {}).get("colors", {})
    return np.array(override.get(name, PALETTE.get(name, PALETTE["frame"])),
                    dtype=float)


# --------------------------------------------------------------- modules
# Every builder takes the station (for its dimensions) and returns the
# boxes to draw. Sizes that are not in the config are details, and are
# chosen to look right rather than to match anything measured.

def workbench(st: Station) -> List[Part]:
    """A bench with a tool board, bins and a terminal: assembly by hand."""
    L, D, H = st.length, st.depth, st.height
    top = 0.05
    parts = [
        Part("top", (0.0, 0.0, H - top / 2.0), (L, D, top), "surface"),
        Part("shelf", (0.0, -0.05, 0.25), (L - 0.20, D - 0.20, 0.04), "frame"),
        Part("board", (0.0, -D / 2.0 + 0.03, H + 0.35), (L, 0.05, 0.70), "surface"),
        Part("rail", (0.0, -D / 2.0 + 0.03, H + 0.72), (L, 0.08, 0.04), "frame"),
    ]
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)), start=1):
        parts.append(Part(f"leg_{i}",
                          (sx * (L / 2.0 - 0.10), sy * (D / 2.0 - 0.10),
                           (H - top) / 2.0),
                          (0.06, 0.06, H - top), "frame"))
    # Bins along the back of the bench top, where the parts live.
    for i in range(3):
        parts.append(Part(f"bin_{i + 1}",
                          (-L / 2.0 + 0.35 + i * 0.40, -D / 2.0 + 0.20,
                           H + 0.09),
                          (0.30, 0.22, 0.18), "bin"))
    # Terminal on a post at one end.
    parts += [
        Part("post", (L / 2.0 - 0.18, -D / 2.0 + 0.12, H + 0.25),
             (0.06, 0.06, 0.50), "frame"),
        Part("screen", (L / 2.0 - 0.18, -D / 2.0 + 0.14, H + 0.62),
             (0.36, 0.05, 0.26), "screen"),
    ]
    return parts


def rack(st: Station) -> List[Part]:
    """Pallet racking against the wall, with stock on the shelves."""
    L, D, H = st.length, st.depth, st.height
    parts: List[Part] = []
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)), start=1):
        parts.append(Part(f"upright_{i}",
                          (sx * (L / 2.0 - 0.05), sy * (D / 2.0 - 0.05), H / 2.0),
                          (0.09, 0.09, H), "guard"))
    levels = 3
    for level in range(levels):
        z = 0.35 + level * (H - 0.45) / max(levels - 1, 1)
        parts.append(Part(f"shelf_{level + 1}", (0.0, 0.0, z), (L, D, 0.05),
                          "frame"))
        for j in range(2):
            parts.append(Part(f"stock_{level + 1}_{j + 1}",
                              (-L / 4.0 + j * L / 2.0, 0.0, z + 0.22),
                              (L / 2.0 - 0.18, D - 0.12, 0.38), "box"))
    return parts


def conveyor(st: Station) -> List[Part]:
    """A belt run with side rails and a drive at one end."""
    L, D, H = st.length, st.depth, st.height
    parts = [
        Part("belt", (0.0, 0.0, H), (L, D, 0.08), "belt"),
        Part("bed", (0.0, 0.0, H - 0.09), (L, D + 0.04, 0.10), "frame"),
        Part("drive", (L / 2.0 - 0.25, 0.0, H - 0.30), (0.45, D, 0.36),
             "machine"),
    ]
    for edge, sy in (("front", 1.0), ("back", -1.0)):
        parts.append(Part(f"rail_{edge}", (0.0, sy * (D / 2.0 + 0.02), H + 0.10),
                          (L, 0.04, 0.14), "guard"))
    legs = max(int(L // 1.2), 2)
    for i in range(legs + 1):
        x = -L / 2.0 + 0.2 + i * (L - 0.4) / legs
        for sy in (-1.0, 1.0):
            parts.append(Part(f"leg_{i}_{'f' if sy > 0 else 'b'}",
                              (x, sy * (D / 2.0 - 0.08), (H - 0.14) / 2.0),
                              (0.07, 0.07, H - 0.14), "frame"))
    # Totes travelling down it, so the line does not look switched off.
    for i in range(3):
        parts.append(Part(f"tote_{i + 1}",
                          (-L / 2.0 + 0.8 + i * 1.1, 0.0, H + 0.19),
                          (0.45, D - 0.18, 0.30), "bin"))
    return parts


def assembly_cell(st: Station) -> List[Part]:
    """A fenced cell with a gate at the front and a light tower on top."""
    L, D, H = st.length, st.depth, st.height
    post, mesh = 0.09, 0.03
    parts: List[Part] = []
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)), start=1):
        parts.append(Part(f"post_{i}",
                          (sx * (L / 2.0 - post / 2.0), sy * (D / 2.0 - post / 2.0),
                           H / 2.0), (post, post, H), "guard"))
    parts.append(Part("mesh_back", (0.0, -D / 2.0 + mesh, H / 2.0 + 0.05),
                      (L, mesh, H - 0.15), "frame"))
    for edge, sx in (("left", -1.0), ("right", 1.0)):
        parts.append(Part(f"mesh_{edge}", (sx * (L / 2.0 - mesh), 0.0,
                                           H / 2.0 + 0.05),
                          (mesh, D, H - 0.15), "frame"))
    # Front: two panels with a gateway between them.
    gate = 0.9
    panel = (L - gate) / 2.0
    for edge, sx in (("l", -1.0), ("r", 1.0)):
        parts.append(Part(f"mesh_front_{edge}",
                          (sx * (L / 2.0 - panel / 2.0), D / 2.0 - mesh,
                           H / 2.0 + 0.05),
                          (panel, mesh, H - 0.15), "frame"))
    # The machine inside, and a table at the gate.
    parts += [
        Part("body", (0.0, -0.15, 0.65), (L - 0.7, D - 0.7, 1.30), "machine"),
        Part("head", (0.0, -0.15, 1.45), (0.5, 0.5, 0.35), "frame"),
        Part("table", (0.0, D / 2.0 - 0.45, 0.42), (L - 1.0, 0.5, 0.05),
             "surface"),
        Part("tower_post", (L / 2.0 - 0.25, -D / 2.0 + 0.25, H + 0.12),
             (0.07, 0.07, 0.24), "frame"),
    ]
    for i, colour in enumerate(("light_green", "light_amber", "light_red")):
        parts.append(Part(f"tower_{i + 1}",
                          (L / 2.0 - 0.25, -D / 2.0 + 0.25, H + 0.28 + i * 0.13),
                          (0.11, 0.11, 0.12), colour))
    return parts


def control_cabinet(st: Station) -> List[Part]:
    """A control cabinet: door, handle, vents, a lamp on top."""
    L, D, H = st.length, st.depth, st.height
    return [
        Part("body", (0.0, 0.0, H / 2.0), (L, D, H), "frame"),
        Part("door", (0.0, D / 2.0 + 0.01, H / 2.0 + 0.05),
             (L - 0.08, 0.02, H - 0.20), "surface"),
        Part("handle", (L / 2.0 - 0.14, D / 2.0 + 0.04, H / 2.0),
             (0.04, 0.06, 0.22), "machine"),
        Part("panel", (0.0, D / 2.0 + 0.03, H - 0.35), (0.26, 0.03, 0.18),
             "screen"),
        Part("lamp", (0.0, 0.0, H + 0.06), (0.10, 0.10, 0.12), "light_green"),
    ]


def pallet_stack(st: Station) -> List[Part]:
    """A loaded pallet: blocks, deck, and boxes stacked on it."""
    L, D, H = st.length, st.depth, st.height
    parts = [Part("deck", (0.0, 0.0, 0.145), (L, D, 0.03), "pallet")]
    for i, x in enumerate((-L / 2.0 + 0.09, 0.0, L / 2.0 - 0.09), start=1):
        parts.append(Part(f"block_{i}", (x, 0.0, 0.075), (0.14, D, 0.09),
                          "pallet"))
    parts.append(Part("base", (0.0, 0.0, 0.015), (L, D, 0.03), "pallet"))
    layers = max(int((H - 0.16) // 0.36), 1)
    for level in range(layers):
        z = 0.16 + 0.18 + level * 0.36
        for j, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)),
                                     start=1):
            parts.append(Part(f"box_{level + 1}_{j}",
                              (sx * L / 4.0, sy * D / 4.0, z),
                              (L / 2.0 - 0.05, D / 2.0 - 0.05, 0.34), "box"))
    return parts


def cart(st: Station) -> List[Part]:
    """A picking cart parked at the edge of a zone."""
    L, D, H = st.length, st.depth, st.height
    deck = 0.36
    parts = [
        Part("deck", (0.0, 0.0, deck), (L, D, 0.05), "surface"),
        Part("shelf", (0.0, 0.0, 0.14), (L - 0.10, D - 0.08, 0.04), "frame"),
        Part("tote", (0.0, 0.0, deck + 0.20), (L - 0.25, D - 0.14, 0.34), "bin"),
        Part("handle_bar", (0.0, -D / 2.0 + 0.05, H), (L - 0.16, 0.04, 0.04),
             "frame"),
    ]
    for i, sx in enumerate((-1.0, 1.0), start=1):
        parts.append(Part(f"handle_post_{i}",
                          (sx * (L / 2.0 - 0.08), -D / 2.0 + 0.05,
                           deck + (H - deck) / 2.0),
                          (0.04, 0.04, H - deck), "frame"))
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)), start=1):
        parts.append(Part(f"caster_{i}",
                          (sx * (L / 2.0 - 0.10), sy * (D / 2.0 - 0.08), 0.06),
                          (0.09, 0.06, 0.12), "belt"))
    return parts


MODULES: Dict[str, Callable[[Station], List[Part]]] = {
    "workbench": workbench,
    "machine": assembly_cell,
    "rack": rack,
    "conveyor": conveyor,
    "assembly_cell": assembly_cell,
    "control_cabinet": control_cabinet,
    "pallet_stack": pallet_stack,
    "cart": cart,
}

# What the robot must not drive into, per module: the box to wrap a
# collider around, as (length, depth, height) scale factors and a z
# offset in metres. Anything finer is visual only.
COLLIDERS: Dict[str, Tuple[float, float, float]] = {
    "workbench": (1.0, 1.0, 1.0),
    "machine": (1.0, 1.0, 1.0),
    "rack": (1.0, 1.0, 1.0),
    "conveyor": (1.0, 1.06, 1.15),
    "assembly_cell": (1.0, 1.0, 1.0),
    "control_cabinet": (1.0, 1.0, 1.0),
    "pallet_stack": (1.0, 1.0, 1.0),
    "cart": (1.0, 1.0, 1.0),
}


def build_station(cfg: Dict[str, Any], station: Station, root: str) -> int:
    """Draw one machine: detail as visuals, one invisible box collider.

    Every part is a child of one Xform sitting at the machine's pose, in
    local coordinates. That is what makes the scene editable: dragging
    /World/Factory/Stations/bench_n1 in the viewport moves the whole
    bench, and isaac_sim/scripts/read_factory_scene.py can read that one
    transform back into the config.
    """
    prim = f"{root}/Stations/{station.name}"
    create_prim(prim, "Xform",
                position=[station.x, station.y, 0.0],
                orientation=_quat(_angle(station)))

    spec = cfg.get("assets", {}).get("modules", {}).get(station.module)
    if spec is not None and spec.get("build"):
        # Built here out of aluminium profile rather than referenced:
        # the asset library has no machine that looks like the ones on
        # this floor. See isaac_sim/environments/machines.py.
        drawn = machines.build(cfg, str(spec["build"]), prim,
                               (station.length, station.depth, station.height))
    elif props.enabled(cfg) and spec is not None:
        drawn = _build_from_asset(prim, spec, station)
    else:
        drawn = _build_from_boxes(cfg, prim, station)

    sl, sd, sh = COLLIDERS.get(station.module, (1.0, 1.0, 1.0))
    body = FixedCuboid(
        prim_path=f"{prim}/collider",
        translation=np.array([0.0, 0.0, station.height * sh / 2.0]),
        scale=np.array([station.length * sl, station.depth * sd,
                        station.height * sh]),
    )
    # Invisible, not absent: the scan should see the legs and rails that
    # are actually there, not a solid block. Physics still stops the
    # robot at the machine's footprint.
    materials.fix_extent(body)
    UsdGeom.Imageable(body.prim).MakeInvisible()
    return drawn


def _build_from_asset(prim: str, spec: Dict[str, Any], station: Station) -> int:
    """The real model, plus whatever is stacked on it.

    A station longer than its model is that model repeated end to end:
    the conveyor is one 2.7 m segment, and the assembly line is three
    of them in a row rather than three separate stubs.
    """
    unit = props.placed_size(spec)[0]
    n = max(int(round(station.length / unit)), 1) if unit > 0 else 1
    for i in range(n):
        along = (i - (n - 1) / 2.0) * unit
        props.reference(f"{prim}/model_{i:02d}" if n > 1 else f"{prim}/model",
                        spec, (along, 0.0, 0.0))
    drawn = n
    for i, item in enumerate(spec.get("stack", []), start=1):
        at = item["at"]
        props.reference(f"{prim}/stack_{i:02d}",
                        {"path": item["path"], "pivot": item.get("pivot", (0.0, 0.0)),
                         "scale": item.get("scale", 1.0)},
                        (float(at[0]), float(at[1]), float(at[2])),
                        yaw_deg=float(item.get("yaw_deg", 0.0)))
        drawn += 1
    return drawn


def _build_from_boxes(cfg: Dict[str, Any], prim: str, station: Station) -> int:
    """The fallback: a machine drawn as a handful of boxes.

    Used when the asset server is unreachable, and by anything that has
    to build the bay without the network. Recognisable, not realistic.
    """
    builder = MODULES.get(station.module)
    if builder is None:
        raise ValueError(f"no box drawing for module {station.module!r}; have "
                         f"{', '.join(sorted(MODULES))}")
    drawn = 0
    for part in builder(station):
        VisualCuboid(
            prim_path=f"{prim}/{part.name}",
            translation=np.array([float(v) for v in part.centre]),
            scale=np.array([float(v) for v in part.size]),
            color=_colour(cfg, part.color),
        )
        drawn += 1
    return drawn


def build_equipment(cfg: Dict[str, Any], layout: BayLayout, root: str,
                    say=None) -> int:
    """Every machine in the bay."""
    if not cfg.get("equipment", {}).get("enabled", True):
        return 0
    drawn = 0
    for station in layout.stations:
        drawn += build_station(cfg, station, root)
    if say:
        built = sum(1 for st in layout.stations
                    if cfg.get("assets", {}).get("modules", {})
                    .get(st.module, {}).get("build"))
        how = (f"{built} built, {len(layout.stations) - built} referenced"
               if props.enabled(cfg) else "boxes (no asset server)")
        say(f"equipment: {len(layout.stations)} machines from {how}, {drawn} parts")
    return drawn
