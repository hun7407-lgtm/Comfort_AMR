"""Scenario preview: who is where, who walks where, drawn on the floor.

Isaac Sim only: imports pxr.

For looking at before a batch, not for measuring. Everything here is a
flat decal with no collider and no rigid body, and none of it is
created unless `preview.enabled` is true -- the same two guarantees
isaac_sim/environments/markers.py carries, for the same reason:

    the LiDAR sits 0.31 m up and its lowest beam rises at 1.9 deg, so
    at any horizontal distance d it is already 0.31 + 0.033 * d above
    the floor. A 4 mm decal is never sampled, at any range.

and with the flag off the prims do not exist, so a trial cannot differ
because of them. `run_scenario.sh` never turns it on.

What it draws
-------------
    a plate under each person, coloured by their state
    their id, as a bar code of pips beside the plate
    the walk each exchanging person takes, as a stripe with an arrow
    the outline of every empty cart slot, so the mission pool is visible

There is no text. USD has no text primitive and a textured billboard
would mean generating an image and a material per label, which is a lot
of machinery for a legend that scripts/make_scenario.py already prints
and scripts/preview_figure.py draws properly. The pips are enough to
match a plate to a row of that table: one pip for N1/S1/E1, two for
N2/S2/E2, three for N3/S3, and the plate's colour says the state.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

from pxr import Gf, Sdf, UsdGeom

ROOT = "/World/Preview"


def _refine(prim) -> None:
    """Round, not octagonal: the stage's refinement level is 0."""
    prim.CreateAttribute("refinementEnableOverride",
                         Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int).Set(2)


def _paint(prim, rgb: Sequence[float], opacity: float = 1.0) -> None:
    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr([Gf.Vec3f(*[float(v) for v in rgb])])
    gprim.CreateDisplayOpacityAttr([float(opacity)])
    _refine(prim)


def _disc(stage, path: str, centre: Tuple[float, float], radius: float,
          thickness: float, rgb, z: float) -> None:
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(thickness))
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([(-radius, -radius, -thickness / 2.0),
                          (radius, radius, thickness / 2.0)])
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(
        Gf.Vec3d(float(centre[0]), float(centre[1]), float(z)))
    _paint(cyl.GetPrim(), rgb)


def _bar(stage, path: str, a: Tuple[float, float], b: Tuple[float, float],
         width: float, thickness: float, rgb, z: float) -> None:
    """One straight stripe from a to b, lying flat."""
    length = math.dist(a, b)
    if length < 1e-6:
        return
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d((a[0] + b[0]) / 2.0,
                                     (a[1] + b[1]) / 2.0, float(z)))
    xf.AddRotateZOp().Set(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))
    xf.AddScaleOp().Set(Gf.Vec3f(float(length), float(width),
                                 float(thickness)))
    _paint(cube.GetPrim(), rgb)


def _outline(stage, path: str, box, width: float, thickness: float,
             rgb, z: float) -> None:
    """Four stripes round a rectangle (x_min, x_max, y_min, y_max)."""
    x0, x1, y0, y1 = box
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for index, (a, b) in enumerate(zip(corners, corners[1:] + corners[:1])):
        _bar(stage, f"{path}/side_{index}", a, b, width, thickness, rgb, z)


def _arrowhead(stage, path: str, at: Tuple[float, float], heading: float,
               size: float, thickness: float, rgb, z: float) -> None:
    cone = UsdGeom.Cone.Define(stage, path)
    cone.CreateRadiusAttr(float(size))
    cone.CreateHeightAttr(float(size) * 2.0)
    cone.CreateAxisAttr("X")
    xf = UsdGeom.Xformable(cone)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(at[0]), float(at[1]), float(z)))
    xf.AddRotateZOp().Set(math.degrees(heading))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, thickness / (size * 2.0)))
    _paint(cone.GetPrim(), rgb)


def _pips(stage, path: str, centre: Tuple[float, float], count: int,
          radius: float, thickness: float, rgb, z: float) -> None:
    """`count` small discs in a row beside the plate, as an id."""
    for index in range(count):
        offset = (index - (count - 1) / 2.0) * radius * 3.0
        _disc(stage, f"{path}/pip_{index}",
              (centre[0] + offset, centre[1] - radius * 4.5),
              radius, thickness, rgb, z)


def build(cfg: Dict[str, Any], stage) -> List[str]:
    """Draw the preview. Returns a line per thing drawn, for the log."""
    spec = cfg.get("preview", {})
    if not spec.get("enabled", False):
        return []

    from comfort_core.layout import human_positions, human_states

    thickness = float(spec.get("thickness", 0.004))
    plate = float(spec.get("plate_radius", 0.40))
    z = float(spec.get("z", 0.006))
    route_width = float(spec.get("route_width", 0.10))
    by_state = spec.get("state_colour", {})
    by_role = spec.get("route_colour", {})
    drawn: List[str] = []

    stage.DefinePrim(Sdf.Path(ROOT), "Xform")

    # Empty cart slots: the pool a mission is drawn from.
    if spec.get("slots", True) and "factory" in cfg:
        from comfort_core.factory_layout import build_layout
        colour = spec.get("slot_colour", (0.55, 0.55, 0.58))
        for slot in build_layout(cfg).cart_slots_free:
            _outline(stage, f"{ROOT}/Slots/{slot.name}",
                     (slot.x_min, slot.x_max, slot.y_min, slot.y_max),
                     route_width * 0.6, thickness, colour, z)
        drawn.append(f"outlined {len(build_layout(cfg).cart_slots_free)} "
                     f"empty cart slots")

    states = human_states(cfg)
    humans = cfg.get("humans", {})

    # The Comfort Space extents each person is about to be given, so the
    # log beside the picture says what the picture is showing. Reading
    # them from the policy rather than from the config means the two can
    # never disagree.
    facing_len = float(spec.get("facing_arrow_m", 0.9))
    facing_rgb = spec.get("facing_colour", (0.95, 0.95, 0.95))
    policy = None
    try:
        from comfort_core.comfort_space import make_policy
        policy = make_policy(cfg)
    except Exception as exc:                                  # noqa: BLE001
        drawn.append(f"no Comfort Space extents in this log ({exc})")

    for key, x, y in human_positions(cfg):
        state = states.get(key, "stationary")
        colour = by_state.get(state, (0.6, 0.6, 0.6))
        base = f"{ROOT}/People/{_safe(key)}"
        # Ring rather than a filled plate: the pose it marks is the
        # middle, and a filled one hides the person standing on it.
        _disc(stage, f"{base}/outer", (x, y), plate, thickness, colour, z)
        _disc(stage, f"{base}/inner", (x, y), plate * 0.72, thickness,
              spec.get("floor_colour", (0.22, 0.22, 0.24)), z + thickness)
        count = _pip_count(key)
        if count:
            _pips(stage, f"{base}/id", (x, y), count, plate * 0.13,
                  thickness, colour, z + thickness * 2)

        own = humans.get(key) or {}

        # Which way the person's body is facing. Drawn for everybody,
        # because for anyone standing this is the direction their
        # Comfort Space is turned by -- and in this bay the belt
        # workers face AWAY from the aisle, which is the whole reason a
        # rear-biased space reaches into it.
        if own.get("facing_deg") is not None and facing_len > 0.0:
            face = math.radians(float(own["facing_deg"]))
            tip = (x + facing_len * math.cos(face),
                   y + facing_len * math.sin(face))
            _bar(stage, f"{base}/facing", (x, y), tip,
                 route_width * 0.45, thickness, facing_rgb, z + thickness * 3)
            _arrowhead(stage, f"{base}/facing_head", tip, face,
                       route_width * 0.9, thickness, facing_rgb,
                       z + thickness * 4)

        walk = own.get("walk") or {}
        route = walk.get("route") or []

        # One line per person: state, facing, and the extents the policy
        # gives them. For somebody who walks, also the extents at the
        # speed this scenario drew for them, since that is the shape
        # that will appear once they set off.
        if policy is not None:
            try:
                rest = policy.extents_for(key, state, 0.0)
                line = (f"{key}: state={state}"
                        + (f" facing={float(own['facing_deg']):+.0f}deg"
                           if own.get("facing_deg") is not None else "")
                        + f" F={rest.front:.2f} S={rest.side:.2f} "
                        f"R={rest.rear:.2f}")
                speed = float(walk.get("speed", 0.0) or 0.0)
                if speed > 0.0:
                    go = policy.extents_for(key, "moving", speed)
                    line += (f" | walking at v={speed:.2f}: "
                             f"F={go.front:.2f} S={go.side:.2f} "
                             f"R={go.rear:.2f}")
                drawn.append(line)
            except Exception as exc:                          # noqa: BLE001
                drawn.append(f"{key}: no extents ({exc})")

        if not route:
            continue
        # Which half of the exchange this is: somebody leaving a
        # working position, or somebody arriving at one.
        half = "edge" if str(walk.get("ends_as")) == "station_engaged" else "belt"
        rgb = by_role.get(half, (0.9, 0.5, 0.1))
        points = [(x, y)] + [(float(p[0]), float(p[1])) for p in route]
        for index, (a, b) in enumerate(zip(points, points[1:])):
            _bar(stage, f"{base}/leg_{index}", a, b, route_width, thickness,
                 rgb, z + thickness)
        last, end = points[-2], points[-1]
        _arrowhead(stage, f"{base}/head", end,
                   math.atan2(end[1] - last[1], end[0] - last[0]),
                   route_width * 1.8, thickness, rgb, z + thickness * 2)
        drawn.append(f"{key}: walks {len(route)} legs to "
                     f"({end[0]:+.2f}, {end[1]:+.2f})")

    return drawn


def _pip_count(key: str) -> int:
    """How many pips mark this person, from the last digit of their id."""
    tail = key[-1]
    return int(tail) if tail.isdigit() and tail != "0" else 0


def _safe(key: str) -> str:
    """A prim name USD will accept."""
    return "".join(c if c.isalnum() else "_" for c in key)
