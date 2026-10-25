"""Reference real models from the Isaac Sim asset library.

Isaac Sim only: imports omni and pxr.

The bay used to be built from coloured boxes. That was enough to check
the geometry and it looked like it: the point of this module is that a
workbench is a workbench, a rack is a rack, and the LiDAR gets returns
off the shape a machine actually has.

Two things have to be handled for every model:

  pivot   where the centre of the model's bounding box sits relative to
          its origin. A conveyor's origin is at one end; a fuse box is
          off centre. The layout says where the centre goes, so the
          offset is cancelled here.
  yaw     which way the model was authored. The bay's module frame puts
          the front at +y, and each model says how far it has to be
          turned to match.

Both are measured values in configs/factory.yaml, not guesses.

If the asset server cannot be reached, `root()` returns None and the
callers fall back to the boxes. That keeps the scene buildable offline
and keeps the geometry checks running with no network at all.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np
from pxr import Usd, UsdPhysics
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.storage.native import get_assets_root_path

Vec2 = Tuple[float, float]

_ROOT: Optional[str] = None
_LOOKED = False


def root() -> Optional[str]:
    """The asset server, or None if it is not reachable.

    Looked up once: the lookup talks to the network, and it is asked for
    on every prop.
    """
    global _ROOT, _LOOKED
    if not _LOOKED:
        _LOOKED = True
        try:
            _ROOT = get_assets_root_path()
        except Exception:  # noqa: BLE001 - offline is a normal case here
            _ROOT = None
    return _ROOT


def enabled(cfg: Dict[str, Any]) -> bool:
    """Whether to build from models rather than boxes."""
    return bool(cfg.get("assets", {}).get("enabled", False)) and root() is not None


def quat(yaw_deg: float) -> np.ndarray:
    """wxyz for a rotation about z."""
    a = math.radians(yaw_deg) / 2.0
    return np.array([math.cos(a), 0.0, 0.0, math.sin(a)])


def placed_size(spec: Dict[str, Any]) -> Tuple[float, float, float]:
    """The model's footprint after its own rotation and scale."""
    nx, ny, nz = (float(v) for v in spec["natural"])
    scale = float(spec.get("scale", 1.0))
    if abs(float(spec.get("yaw_deg", 0.0))) % 180.0 == 90.0:
        nx, ny = ny, nx
    return (nx * scale, ny * scale, nz * scale)


def _offset(pivot: Vec2, yaw_deg: float, scale: float) -> Tuple[float, float]:
    """Where the model's origin goes so its centre lands on the target."""
    a = math.radians(yaw_deg)
    px, py = (float(v) * scale for v in pivot)
    return (-(px * math.cos(a) - py * math.sin(a)),
            -(px * math.sin(a) + py * math.cos(a)))


def reference(prim_path: str, spec: Dict[str, Any], position: Sequence[float],
              yaw_deg: float = 0.0) -> Optional[str]:
    """Put one model in the scene, centred on `position`.

    `position` is where the model's bounding-box centre should sit in x
    and y; z is its base, because everything in this library is authored
    standing on the floor. `yaw_deg` is added to the model's own
    authored rotation.

    The position is applied as a LOCAL translation, so a model under a
    station's Xform lands where that station is. create_prim's
    `position` argument is a world pose instead, which quietly piles
    every machine on the origin.
    """
    base = root()
    if base is None:
        return None
    own_yaw = float(spec.get("yaw_deg", 0.0))
    scale = float(spec.get("scale", 1.0))
    total_yaw = own_yaw + yaw_deg
    dx, dy = _offset(spec.get("pivot", (0.0, 0.0)), total_yaw, scale)
    z = float(position[2]) if len(position) > 2 else 0.0
    # `prim` names one prim INSIDE the file, for an asset that is a
    # little scene rather than a single model. The loaded pallet is
    # one: the file holds /pallet_loaded and a 50 m ground plane, and
    # referencing the file whole would lay that floor through the bay.
    inner = spec.get("prim")
    create_prim(
        prim_path, "Xform",
        translation=[float(position[0]) + dx, float(position[1]) + dy, z],
        orientation=quat(total_yaw),
        scale=[scale, scale, scale * float(spec.get("scale_z_ratio", 1.0))],
        usd_path=None if inner else base + str(spec["path"]),
    )
    if inner:
        # The reference goes on a CHILD, not on the prim just placed: a
        # referenced prim brings its own xformOp:translate with it, and
        # that opinion beat the pose set above -- the loaded pallet came
        # back to z = 0 and sat on the floor with the cart's frame
        # through the middle of it.
        #
        # Defined straight on the stage, NOT through create_prim.
        # create_prim ends by constructing an XFormPrim even when no
        # pose is passed, and that writes a local transform which
        # cancels the parent's: the child came out with
        # translate z = -0.44 under a parent at +0.44, back on the
        # floor again. Only the asset reference belongs on this prim.
        child = get_current_stage().DefinePrim(f"{prim_path}/model", "Xform")
        child.GetReferences().AddReference(assetPath=base + str(spec["path"]),
                                           primPath=str(inner))
    if spec.get("static"):
        freeze(prim_path)
    return prim_path


def freeze(prim_path: str) -> int:
    """Turn off any rigid body under `prim_path`. Returns how many.

    Some library assets are built for physics demos and carry a rigid
    body: the loaded pallet does. The scene build plays the timeline
    for a few seconds to settle the characters, and in those seconds
    every one of those pallets fell off the cart it had been placed on
    and landed on the floor at a slight angle, which looked exactly
    like the pallet having been put at z = 0 by mistake.

    Scenery in this study does not move. Anything that can fall over
    between the start of two trials is a difference between those
    trials.
    """
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return 0
    frozen = 0
    for child in Usd.PrimRange(prim):
        if not child.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        attr = child.GetAttribute("physics:rigidBodyEnabled")
        if not attr:
            attr = UsdPhysics.RigidBodyAPI(child).CreateRigidBodyEnabledAttr()
        attr.Set(False)
        frozen += 1
    return frozen


def check_module_sizes(cfg: Dict[str, Any], tolerance: float = 0.03) -> None:
    """The layout's footprints and the models have to agree.

    The layout maths is pure Python and cannot open a USD, so the
    footprint of every machine is written down twice: once as
    equipment.modules, once as the model's measured size. Two numbers
    that are supposed to be the same are worth checking, especially
    since a wrong one puts a machine through a wall in a way that only
    shows up in a render.
    """
    modules = cfg.get("equipment", {}).get("modules", {})
    problems = []
    for name, spec in cfg.get("assets", {}).get("modules", {}).items():
        dims = modules.get(name)
        if dims is None:
            problems.append(f"assets.modules.{name} has no equipment.modules entry")
            continue
        want = (float(dims["length"]), float(dims["depth"]), float(dims["height"]))
        if spec.get("build"):
            # Built to order: it fills whatever footprint it is given,
            # so there is only the declared size to agree with.
            got = tuple(float(v) for v in spec.get("size", want))
        else:
            got = placed_size(spec)
        # Height is only comparable when the model is the whole machine.
        # A pallet is 0.21 m tall and the module is 1.21 m, because the
        # boxes stacked on it are what the robot would hit.
        n = 2 if spec.get("stack") else 3
        if any(abs(a - b) > tolerance for a, b in zip(want[:n], got[:n])):
            problems.append(
                f"{name}: equipment.modules says "
                f"{want[0]:.2f} x {want[1]:.2f} x {want[2]:.2f}, "
                f"the model is {got[0]:.2f} x {got[1]:.2f} x {got[2]:.2f}")
    if problems:
        raise ValueError("model and layout disagree:\n  - " + "\n  - ".join(problems))
