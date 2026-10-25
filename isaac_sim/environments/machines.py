"""Machines built out of aluminium extrusion, the way the plant's are.

Isaac Sim only: imports omni and pxr.

The asset library has warehouse furniture -- pallet racking, roller
conveyors, blue steel. The floor in the site video has none of that. It
has cells and trolleys built from 40 mm aluminium profile with clear
polycarbonate panels, black and yellow totes on every shelf, a terminal
and a light tower on top of each cell. That is what this module builds.

Everything is a box, but the proportions are the point: a 45 mm upright
and a 6 mm panel read as a machine, and a 200 mm one reads as a toy.
Sizes come from the module's footprint, so a cell that has to fit a
narrower zone shrinks rather than overflowing it.

Each builder returns the number of pieces it drew. Colliders are not its
business: the station already has one invisible box around it.

> Generic equipment at generic sizes. Not a reproduction of any machine
> on the site.
"""

from __future__ import annotations


from typing import Any, Callable, Dict, Tuple

import numpy as np
from isaacsim.core.api.objects import VisualCuboid, VisualCylinder

from isaac_sim.environments import materials, props

PROFILE = 0.045      # aluminium extrusion, 45 mm
PANEL = 0.006        # polycarbonate sheet
SHELF = 0.022


def _box(cfg, prim_path: str, centre, size, material: str):
    box = VisualCuboid(
        prim_path=prim_path,
        translation=np.array([float(centre[0]), float(centre[1]), float(centre[2])]),
        scale=np.array([float(size[0]), float(size[1]), float(size[2])]),
        color=np.array([0.8, 0.8, 0.8]),
    )
    materials.apply(cfg, box, material)
    return box


def _cyl(cfg, prim_path: str, centre, radius: float, height: float,
         material: str, axis: str = "z"):
    orientation = {
        "z": (1.0, 0.0, 0.0, 0.0),
        "x": (0.7071068, 0.0, 0.7071068, 0.0),
        "y": (0.7071068, -0.7071068, 0.0, 0.0),
    }[axis]
    cyl = VisualCylinder(
        prim_path=prim_path,
        translation=np.array([float(c) for c in centre]),
        orientation=np.array(orientation),
        radius=float(radius),
        height=float(height),
    )
    materials.apply(cfg, cyl, material)
    return cyl


def _frame(cfg, prim: str, w: float, d: float, h: float,
           heights: Tuple[float, ...] = (0.1,)) -> int:
    """Four uprights and a rail at each named height, plus one on top."""
    n = 0
    half = PROFILE / 2.0
    for tag, sx, sy in (("a", -1, -1), ("b", -1, 1), ("c", 1, -1), ("d", 1, 1)):
        _box(cfg, f"{prim}/post_{tag}",
             (sx * (w / 2.0 - half), sy * (d / 2.0 - half), h / 2.0),
             (PROFILE, PROFILE, h), "aluminium")
        n += 1
    for i, z in enumerate(tuple(heights) + (h - half,)):
        for tag, sy in (("f", 1.0), ("b", -1.0)):
            _box(cfg, f"{prim}/rail_{i}{tag}",
                 (0.0, sy * (d / 2.0 - half), z),
                 (w - PROFILE, PROFILE, PROFILE), "aluminium")
            n += 1
        for tag, sx in (("l", -1.0), ("r", 1.0)):
            _box(cfg, f"{prim}/side_{i}{tag}",
                 (sx * (w / 2.0 - half), 0.0, z),
                 (PROFILE, d - PROFILE, PROFILE), "aluminium")
            n += 1
    return n


def _tote(cfg, prim: str, centre, size, colour: str) -> int:
    """One stacking tote: a box with a rim, which is what makes it read.

    A plain cuboid at this size looks like a painted brick. The lip
    around the top is 3 cm of geometry and the difference between a
    crate and a block.
    """
    w, d, h = size
    _box(cfg, f"{prim}/body", (centre[0], centre[1], centre[2]), (w, d, h), colour)
    _box(cfg, f"{prim}/rim", (centre[0], centre[1], centre[2] + h / 2.0 - 0.012),
         (w + 0.022, d + 0.022, 0.03), colour)
    return 2


def _placed(spec: Dict[str, Any]) -> Tuple[float, float, float]:
    """The footprint of a tote asset after its own rotation."""
    sx, sy, sz = (float(v) for v in spec["size"])
    if abs(float(spec.get("yaw_deg", 0.0))) % 180.0 == 90.0:
        sx, sy = sy, sx
    return sx, sy, sz


def _totes(cfg, prim: str, w: float, d: float, z: float, count: int,
           colours: Tuple[str, ...] = ("tote_blue", "tote_yellow", "tote_black",
                                       "tote_blue", "tote_teal", "tote_yellow"),
           tag: str = "tote", stack: int = 1) -> int:
    """A row of stacking totes on a shelf, as they sit on the real ones.

    Real crates and bins from the asset library when it is reachable.
    They were boxes with a painted rim before, which is exactly the
    thing the library already has done properly. The drawn version
    stays as the offline fallback.
    """
    catalogue = [t for t in cfg.get("assets", {}).get("totes", [])
                 if _placed(t)[1] <= d - 0.06]
    if props.enabled(cfg) and catalogue:
        # Fill the shelf greedily. The crates are real assets and come
        # in whatever sizes they come in, so dividing the shelf into
        # equal slots just skipped every crate wider than its slot and
        # left the shelf bare.
        palette = ("tote_blue", "tote_yellow", "tote_black", "tote_grey",
                   "tote_teal")
        n = 0
        cursor = -w / 2.0 + 0.07
        limit = w / 2.0 - 0.07
        i = 0
        while cursor < limit and n < count * 2:
            spec = catalogue[(i + len(tag)) % len(catalogue)]
            tw, _td, th = _placed(spec)
            if cursor + tw > limit:
                i += 1
                if i > len(catalogue) * 2:
                    break
                continue
            x = cursor + tw / 2.0
            for k in range(stack if i % 2 == 0 else 1):
                path = f"{prim}/{tag}_{n:02d}_{k}"
                props.reference(path, spec,
                                (x, 0.0, z + k * (th + 0.004)
                                 - float(spec.get("z_base", 0.0))))
                # Repainted into the plant's own palette: the library
                # crates come in olive and purple, the floor in the
                # video is blue, yellow and black.
                materials.bind_over(cfg, path, palette[(n + k) % len(palette)])
                n += 1
            cursor += tw + 0.04
            i += 1
        return n

    tw = min(0.38, (w - 0.14) / max(count, 1))
    td = min(0.28, d - 0.14)
    th = 0.21
    n = 0
    for i in range(count):
        x = -((count - 1) / 2.0) * (tw + 0.035) + i * (tw + 0.035)
        for k in range(stack if i % 2 == 0 else 1):
            n += _tote(cfg, f"{prim}/{tag}_{i:02d}_{k}",
                       (x, 0.0, z + th / 2.0 + k * (th + 0.005)),
                       (tw, td, th), colours[(i + k) % len(colours)])
    return n


def _screen(cfg, prim: str, centre, size, yaw_sign: float = 1.0) -> int:
    """A terminal that is switched on.

    A black rectangle reads as a dead panel. Three lit bands on a pale
    ground is enough to say "there is a screen there" from across the
    aisle, which is all the camera ever sees of it.
    """
    w, d, h = size
    _box(cfg, f"{prim}/case", centre, (w, d, h), "screen")
    face = (centre[0], centre[1] + yaw_sign * (d / 2.0 + 0.004), centre[2])
    glow = VisualCuboid(
        prim_path=f"{prim}/face",
        translation=np.array([float(v) for v in face]),
        scale=np.array([w * 0.86, 0.004, h * 0.78]),
        color=np.array([0.8, 0.9, 1.0]))
    glow.apply_visual_material(
        materials.emissive(cfg, "screen_on", (0.62, 0.78, 0.92), 900.0))
    rows = 0
    for i, colour in enumerate(("light_green", "tote_blue", "light_amber")):
        bar = (face[0] - w * 0.18 + i * w * 0.2, face[1] + yaw_sign * 0.003,
               face[2] + h * 0.18 - i * h * 0.16)
        _box(cfg, f"{prim}/bar_{i}", bar, (w * 0.16, 0.003, h * 0.1), colour)
        rows += 1
    return 2 + rows


def panel_cart(cfg, prim: str, w: float, d: float, h: float) -> int:
    """A trolley carrying sheet parts on edge, as the aisle ones do."""
    n = _frame(cfg, prim, w, d, h, heights=(0.12,))
    _box(cfg, f"{prim}/deck", (0.0, 0.0, 0.20), (w - 0.06, d - 0.06, SHELF),
         "aluminium_dark")
    n += 1
    sheets = max(int((d - 0.12) // 0.085), 2)
    for i in range(sheets):
        y = -(d - 0.2) / 2.0 + i * ((d - 0.2) / max(sheets - 1, 1))
        _box(cfg, f"{prim}/sheet_{i:02d}", (0.0, y, 0.20 + (h - 0.3) / 2.0),
             (w - 0.16, 0.018, h - 0.3),
             "machine_white" if i % 3 else "aluminium")
        n += 1
    _box(cfg, f"{prim}/handle", (0.0, -d / 2.0 + 0.03, h + 0.06),
         (w - 0.1, 0.035, 0.035), "aluminium")
    n += 1
    for tag, sx, sy in (("a", -1, -1), ("b", -1, 1), ("c", 1, -1), ("d", 1, 1)):
        _cyl(cfg, f"{prim}/caster_{tag}",
             (sx * (w / 2.0 - 0.09), sy * (d / 2.0 - 0.09), 0.05),
             0.045, 0.035, "rubber", axis="y")
        n += 1
    return n


def cell(cfg, prim: str, w: float, d: float, h: float) -> int:
    """A fenced assembly cell: profile frame, clear panels, terminal, tower."""
    n = _frame(cfg, prim, w, d, h, heights=(0.12, 1.05))

    inner_h = h - 0.30
    # Clear panels: back and both sides, and two at the front with a
    # gap between them for the operator.
    _box(cfg, f"{prim}/panel_back", (0.0, -d / 2.0 + PANEL, inner_h / 2.0 + 0.2),
         (w - 0.1, PANEL, inner_h), "panel_glass")
    for tag, sx in (("l", -1.0), ("r", 1.0)):
        _box(cfg, f"{prim}/panel_{tag}", (sx * (w / 2.0 - PANEL), 0.0,
                                          inner_h / 2.0 + 0.2),
             (PANEL, d - 0.1, inner_h), "panel_glass")
    gap = min(0.75, w * 0.4)
    wing = (w - gap) / 2.0
    for tag, sx in (("l", -1.0), ("r", 1.0)):
        _box(cfg, f"{prim}/front_{tag}",
             (sx * (w / 2.0 - wing / 2.0), d / 2.0 - PANEL, inner_h / 2.0 + 0.2),
             (wing, PANEL, inner_h), "panel_glass")
    n += 5

    # What is inside: a machine body on a base, a fixture on top of it.
    body_w, body_d = w - 0.45, d - 0.40
    _box(cfg, f"{prim}/body", (0.0, -0.04, 0.52), (body_w, body_d, 0.80),
         "machine_white")
    _box(cfg, f"{prim}/bed", (0.0, -0.04, 0.95), (body_w - 0.15, body_d - 0.12, 0.07),
         "aluminium_dark")
    _box(cfg, f"{prim}/head", (0.0, -0.10, 1.35), (0.34, 0.34, 0.70), "machine_grey")
    _box(cfg, f"{prim}/clamp", (0.0, -0.10, 1.02), (0.22, 0.22, 0.12), "steel_dark")
    # A spindle on a slide, a cable chain back to the frame, and a
    # control box on the side: the things that fill a real cell.
    _cyl(cfg, f"{prim}/spindle", (0.0, -0.10, 1.07), 0.05, 0.22, "steel_dark")
    _box(cfg, f"{prim}/slide", (0.0, -0.30, 1.35), (0.10, 0.10, 0.66), "aluminium")
    for i in range(5):
        _box(cfg, f"{prim}/chain_{i}", (0.16 + i * 0.07, -0.30, 1.62 - i * 0.015),
             (0.06, 0.07, 0.05), "steel_dark")
    _box(cfg, f"{prim}/control", (w / 2.0 - 0.22, -d / 2.0 + 0.16, 0.95),
         (0.26, 0.20, 0.52), "machine_grey")
    _box(cfg, f"{prim}/control_face", (w / 2.0 - 0.22, -d / 2.0 + 0.05, 1.05),
         (0.20, 0.02, 0.14), "screen")
    n += 11

    # Terminal on the front post, and the paperwork beside it.
    screen_z = 1.48
    _box(cfg, f"{prim}/hmi_arm", (w / 2.0 - 0.14, d / 2.0 - 0.10, screen_z),
         (0.05, 0.16, 0.05), "aluminium_dark")
    _screen(cfg, f"{prim}/hmi", (w / 2.0 - 0.14, d / 2.0 + 0.02, screen_z),
            (0.38, 0.04, 0.26))
    _box(cfg, f"{prim}/sheet", (-w / 2.0 + 0.22, d / 2.0 + 0.01, 1.35),
         (0.21, 0.002, 0.30), "paper")
    n += 3

    # Light tower on a corner post.
    tx, ty = -w / 2.0 + 0.12, -d / 2.0 + 0.12
    _box(cfg, f"{prim}/tower_post", (tx, ty, h + 0.10), (0.05, 0.05, 0.20),
         "aluminium_dark")
    for i, colour in enumerate(("light_green", "light_amber", "light_red")):
        _cyl(cfg, f"{prim}/tower_{i}", (tx, ty, h + 0.24 + i * 0.09),
             0.045, 0.085, colour)
    n += 4

    # Totes waiting at the mouth of the cell.
    n += _totes(cfg, prim, gap, 0.34, 0.14, 2, tag="feed")
    return n


def trolley(cfg, prim: str, w: float, d: float, h: float) -> int:
    """A three-shelf material trolley on casters, loaded with totes."""
    deck = 0.08
    n = _frame(cfg, prim, w, d, h, heights=())
    levels = (deck + 0.06, deck + 0.42, deck + 0.78)
    for i, z in enumerate(levels):
        if z + 0.1 > h:
            continue
        _box(cfg, f"{prim}/shelf_{i}", (0.0, 0.0, z), (w - 0.06, d - 0.06, SHELF),
             "aluminium_dark")
        n += 1
        n += _totes(cfg, prim, w, d, z + SHELF / 2.0, 2, tag=f"tote{i}",
                    stack=2 if i == 0 else 1)

    # Push handle at one end.
    _box(cfg, f"{prim}/handle", (0.0, -d / 2.0 + 0.03, h + 0.06),
         (w - 0.1, 0.035, 0.035), "aluminium")
    n += 1

    for tag, sx, sy in (("a", -1, -1), ("b", -1, 1), ("c", 1, -1), ("d", 1, 1)):
        _cyl(cfg, f"{prim}/caster_{tag}",
             (sx * (w / 2.0 - 0.09), sy * (d / 2.0 - 0.09), deck / 2.0),
             0.045, 0.035, "rubber", axis="y")
        _box(cfg, f"{prim}/leg_{tag}",
             (sx * (w / 2.0 - 0.09), sy * (d / 2.0 - 0.09), deck - 0.01),
             (0.05, 0.05, 0.06), "aluminium_dark")
        n += 2
    return n


def shelf(cfg, prim: str, w: float, d: float, h: float) -> int:
    """Profile shelving against a wall, full of totes and boxes."""
    n = _frame(cfg, prim, w, d, h, heights=(0.08,))
    levels = [0.35, 0.80, 1.25, 1.70]
    for i, z in enumerate(levels):
        if z + 0.25 > h:
            continue
        _box(cfg, f"{prim}/shelf_{i}", (0.0, 0.0, z), (w - 0.06, d - 0.06, SHELF),
             "aluminium_dark")
        n += 1
        n += _totes(cfg, prim, w, d, z + SHELF / 2.0, max(int(w // 0.46), 2),
                    tag=f"tote{i}", stack=2 if i == 0 else 1)
    # A label strip along the front of each level, the way stock is marked.
    for i, z in enumerate(levels):
        if z + 0.25 > h:
            continue
        _box(cfg, f"{prim}/label_{i}", (0.0, d / 2.0 - 0.01, z + 0.05),
             (w - 0.2, 0.004, 0.04), "paper")
        n += 1
    return n


def bench(cfg, prim: str, w: float, d: float, h: float) -> int:
    """A work bench: steel top, tote rack under it, task light and terminal."""
    top = 0.04
    n = 0
    _box(cfg, f"{prim}/top", (0.0, 0.0, h - top / 2.0), (w, d, top), "aluminium")
    _box(cfg, f"{prim}/apron", (0.0, -d / 2.0 + 0.03, h - 0.12), (w, 0.04, 0.16),
         "aluminium_dark")
    n += 2
    for tag, sx, sy in (("a", -1, -1), ("b", -1, 1), ("c", 1, -1), ("d", 1, 1)):
        _box(cfg, f"{prim}/leg_{tag}",
             (sx * (w / 2.0 - 0.08), sy * (d / 2.0 - 0.08), (h - top) / 2.0),
             (0.05, 0.05, h - top), "aluminium_dark")
        n += 1
    _box(cfg, f"{prim}/under", (0.0, 0.0, 0.26), (w - 0.12, d - 0.12, SHELF),
         "aluminium_dark")
    n += 1
    n += _totes(cfg, prim, w, d, 0.27, max(int(w // 0.46), 2), tag="under")

    # Back board with bins and paperwork, which is what every bench has.
    _box(cfg, f"{prim}/board", (0.0, -d / 2.0 + 0.04, h + 0.42),
         (w, 0.03, 0.80), "machine_white")
    _box(cfg, f"{prim}/board_rail", (0.0, -d / 2.0 + 0.04, h + 0.80),
         (w, 0.05, 0.04), "aluminium")
    _box(cfg, f"{prim}/sheet", (-w / 2.0 + 0.28, -d / 2.0 + 0.06, h + 0.52),
         (0.21, 0.002, 0.30), "paper")
    n += 3
    n += _totes(cfg, prim, w * 0.7, 0.26, h + 0.02, 3, tag="bin")

    _box(cfg, f"{prim}/task_arm", (w / 2.0 - 0.16, -d / 2.0 + 0.08, h + 0.60),
         (0.04, 0.04, 0.36), "aluminium_dark")
    _box(cfg, f"{prim}/task_lamp", (w / 2.0 - 0.16, -d / 2.0 + 0.22, h + 0.76),
         (0.30, 0.10, 0.05), "lamp_white")
    n += 2
    return n


def line(cfg, prim: str, w: float, d: float, h: float) -> int:
    """A length of assembly line: profile frame, rollers, parts beside it."""
    n = _frame(cfg, prim, w, d, h, heights=(0.15, 0.55))
    bed = h - 0.12
    _box(cfg, f"{prim}/bed", (0.0, 0.0, bed - 0.05), (w - 0.08, d - 0.14, 0.08),
         "aluminium_dark")
    n += 1
    rollers = max(int((w - 0.2) // 0.12), 1)
    for i in range(rollers):
        x = -(w - 0.3) / 2.0 + i * ((w - 0.3) / max(rollers - 1, 1))
        _cyl(cfg, f"{prim}/roller_{i:02d}", (x, 0.0, bed + 0.02), 0.028,
             d - 0.16, "aluminium", axis="y")
        n += 1
    for tag, sy in (("f", 1.0), ("b", -1.0)):
        _box(cfg, f"{prim}/guide_{tag}", (0.0, sy * (d / 2.0 - 0.07), bed + 0.08),
             (w - 0.1, 0.03, 0.10), "aluminium")
        n += 1
    # Work in progress travelling down it.
    n += _totes(cfg, prim, w * 0.8, d * 0.6, bed + 0.05, max(int(w // 1.2), 2),
                colours=("tote_black", "tote_grey"), tag="wip")
    # A terminal every few metres, like the stations on the real line.
    for i in range(max(int(w // 2.5), 1)):
        x = -w / 2.0 + (i + 0.5) * (w / max(int(w // 2.5), 1))
        _box(cfg, f"{prim}/post_{i:02d}", (x, -d / 2.0 + 0.05, h + 0.25),
             (0.05, 0.05, 0.5), "aluminium")
        _screen(cfg, f"{prim}/hmi_{i:02d}", (x, -d / 2.0 + 0.02, h + 0.48),
                (0.34, 0.04, 0.24), yaw_sign=-1.0)
        n += 2
    return n


def cart(cfg: Dict[str, Any], prim: str, length: float, depth: float,
         height: float) -> int:
    """A parts cart: the library dolly if it can be fetched, else a box.

    One function, so the cart the AMR tows and the carts standing in the
    staging blocks are the same object. They were drawn by two different
    code paths once and came out different sizes.
    """
    spec = cfg.get("assets", {}).get("modules", {}).get("cart")
    if spec and spec.get("path") and props.enabled(cfg):
        # Under the caller's Xform, not at it: the caller has already
        # created that prim and placed it, and referencing onto the
        # same path fails. Equipment does the same thing.
        if props.reference(f"{prim}/model", spec, (0.0, 0.0, 0.0)) is not None:
            return 1
    return trolley(cfg, prim, length, depth, height)


BUILDERS: Dict[str, Callable[..., int]] = {
    "cell": cell,
    "panel_cart": panel_cart,
    "trolley": trolley,
    "cart": cart,
    "shelf": shelf,
    "bench": bench,
    "line": line,
}


def build(cfg: Dict[str, Any], kind: str, prim: str,
          size: Tuple[float, float, float]) -> int:
    """Draw one machine of the named kind, filling the given footprint."""
    builder = BUILDERS.get(kind)
    if builder is None:
        raise ValueError(f"no parametric machine called {kind!r}; have "
                         f"{', '.join(sorted(BUILDERS))}")
    return builder(cfg, prim, float(size[0]), float(size[1]), float(size[2]))


def stack(cfg, prim: str, levels: int = 3) -> int:
    """A column of crates left on the floor, waiting to be taken away."""
    catalogue = cfg.get("assets", {}).get("totes", [])
    if props.enabled(cfg) and catalogue:
        spec = catalogue[0]
        _tw, _td, th = _placed(spec)
        for k in range(max(levels, 1)):
            props.reference(f"{prim}/crate_{k}", spec,
                            (0.0, 0.0, k * (th + 0.004)
                             - float(spec.get("z_base", 0.0))))
        return max(levels, 1)
    n = 0
    for k in range(max(levels, 1)):
        n += _tote(cfg, f"{prim}/tote_{k}", (0.0, 0.0, 0.12 + k * 0.215),
                   (0.58, 0.38, 0.21),
                   ("tote_yellow", "tote_blue", "tote_black")[k % 3])
    return n
