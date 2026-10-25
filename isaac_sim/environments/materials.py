"""The materials the bay is made of.

Isaac Sim only: imports omni and pxr.

A flat colour on a box reads as a toy no matter how well the box is
placed. What makes a plant floor look like a plant floor is that it is
glossy: the epoxy reflects the lights, the racks and the people on it,
and the yellow lines are duller than the green around them. So every
surface here gets a real material with a roughness, not a colour.

Materials are shared. One green is bound to the whole floor rather than
one per piece, which keeps the scene small and means a change to the
green is a change in one place.

> Colours and finishes are taken from a site video of the real plant:
> vivid green epoxy, safety-yellow lanes, white aluminium framing,
> cream walls. Nothing here reproduces a specific machine.
"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np
from isaacsim.core.api.materials import OmniPBR
from pxr import Sdf, UsdShade

from isaac_sim.environments import props

LOOKS = "/World/Looks"

# name -> (rgb, roughness, metallic). Glass is separate, below.
DEFAULTS: Dict[str, tuple] = {
    # Floor. Epoxy is close to a mirror at grazing angles, which is what
    # makes the aisle read as a floor rather than a green rectangle.
    "epoxy_green": ((0.040, 0.32, 0.11), 0.08, 0.0),
    "epoxy_line": ((0.86, 0.68, 0.04), 0.26, 0.0),
    "epoxy_line_dark": ((0.80, 0.62, 0.05), 0.38, 0.0),
    # Building.
    "wall_cream": ((0.84, 0.82, 0.76), 0.62, 0.0),
    "column_cream": ((0.80, 0.78, 0.70), 0.55, 0.0),
    "ceiling_white": ((0.88, 0.89, 0.90), 0.70, 0.0),
    "steel_light": ((0.72, 0.73, 0.74), 0.45, 0.35),
    "hazard_black": ((0.09, 0.09, 0.10), 0.55, 0.0),
    # Equipment. The frames on the floor are extruded aluminium, which
    # is bright, slightly rough and unmistakably metal.
    "aluminium": ((0.78, 0.79, 0.81), 0.28, 0.85),
    "aluminium_dark": ((0.46, 0.48, 0.50), 0.35, 0.80),
    "steel_dark": ((0.22, 0.23, 0.25), 0.40, 0.70),
    "machine_white": ((0.86, 0.87, 0.88), 0.35, 0.0),
    "machine_grey": ((0.38, 0.40, 0.43), 0.42, 0.10),
    "screen": ((0.04, 0.05, 0.06), 0.18, 0.0),
    "rubber": ((0.10, 0.10, 0.11), 0.75, 0.0),
    # Totes and bins, the colours that are actually on the floor there.
    # Measured off the video: the yellow is a worn safety yellow, the
    # blue is a deep plastic blue, and about a third of the totes on
    # any shelf are black.
    "tote_yellow": ((0.72, 0.55, 0.05), 0.46, 0.0),
    "tote_blue": ((0.06, 0.20, 0.44), 0.46, 0.0),
    "tote_black": ((0.08, 0.08, 0.09), 0.52, 0.0),
    "tote_grey": ((0.42, 0.44, 0.46), 0.50, 0.0),
    "tote_teal": ((0.07, 0.33, 0.37), 0.46, 0.0),
    "cardboard": ((0.64, 0.50, 0.33), 0.75, 0.0),
    "paper": ((0.92, 0.92, 0.90), 0.65, 0.0),
    # Lamps and towers.
    "lamp_white": ((0.95, 0.96, 0.97), 0.30, 0.0),
    "glazing": ((0.80, 0.86, 0.90), 0.10, 0.0),
    "tray_grey": ((0.52, 0.54, 0.56), 0.45, 0.30),
    "pipe_grey": ((0.62, 0.64, 0.66), 0.35, 0.40),
    "pipe_red": ((0.55, 0.12, 0.10), 0.40, 0.20),
    "door_grey": ((0.70, 0.71, 0.72), 0.45, 0.20),
    "sign_green": ((0.07, 0.42, 0.22), 0.45, 0.0),
    "light_red": ((0.72, 0.08, 0.06), 0.35, 0.0),
    "light_amber": ((0.85, 0.52, 0.03), 0.35, 0.0),
    "light_green": ((0.10, 0.62, 0.20), 0.35, 0.0),
}

_CACHE: Dict[str, Any] = {}


def reset() -> None:
    """Forget the materials. Call when a new stage is created."""
    _CACHE.clear()


def _spec(cfg: Dict[str, Any], name: str) -> tuple:
    override = cfg.get("assets", {}).get("materials", {}).get(name)
    if override is None:
        return DEFAULTS[name]
    base = DEFAULTS.get(name, ((0.5, 0.5, 0.5), 0.5, 0.0))
    return (tuple(override.get("color", base[0])),
            float(override.get("roughness", base[1])),
            float(override.get("metallic", base[2])))


def get(cfg: Dict[str, Any], name: str):
    """The shared material called `name`, made on first use."""
    if name in _CACHE:
        return _CACHE[name]
    if name == "panel_glass":
        panel = translucent(cfg, "panel", (0.80, 0.88, 0.92), 0.14)
        _CACHE[name] = panel
        return panel
    colour, roughness, metallic = _spec(cfg, name)
    material = OmniPBR(prim_path=f"{LOOKS}/{name}", name=name,
                       color=np.array(colour, dtype=float))
    material.set_reflection_roughness(float(roughness))
    material.set_metallic_constant(float(metallic))
    _CACHE[name] = material
    return material


def fix_extent(prim_object) -> None:
    """Author a unit cube's extent as a unit cube's extent.

    The core API writes the extent of a scaled cuboid in scaled units
    while leaving `size` at 1, so USD multiplies the scale in twice and
    every world bound comes out scale times too big: a 7.4 m conveyor
    reports as 55 m. Nothing renders wrong, but every bounds query --
    including the check that nothing stands in an aisle -- is useless
    until this is put right.
    """
    prim = getattr(prim_object, "prim", None)
    if prim is None:
        return
    size = prim.GetAttribute("size")
    extent = prim.GetAttribute("extent")
    if not size or not extent:
        return
    half = float(size.Get() or 1.0) / 2.0
    extent.Set([(-half, -half, -half), (half, half, half)])


def apply(cfg: Dict[str, Any], prim_object, name: str) -> None:
    """Paint one of the core-API objects (a cuboid, a cylinder)."""
    prim_object.apply_visual_material(get(cfg, name))
    fix_extent(prim_object)


def bind_over(cfg: Dict[str, Any], prim_path: str, name: str) -> bool:
    """Paint a referenced model, overriding the materials it came with.

    An ordinary binding on the parent loses to the bindings the asset
    already has on its own meshes, so the wall panels stay brick. The
    `strongerThanDescendants` strength is what makes a repaint stick.
    """
    from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid

    if not is_prim_path_valid(prim_path):
        return False
    material = get(cfg, name)
    prim = get_prim_at_path(prim_path)
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(UsdShade.Material(material.prim),
                 bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    return True


def _shader_of(material):
    for child in material.prim.GetChildren():
        if child.GetTypeName() == "Shader":
            return UsdShade.Shader(child)
    return None


def translucent(cfg: Dict[str, Any], name: str, colour, opacity: float,
                roughness: float = 0.08):
    """A see-through panel: machine guarding, not window glass.

    Real glass renders dark here -- it reflects the inside of a machine,
    which is dark -- and the cells ended up looking like aquariums. A
    PBR with an opacity cut is brighter, cheaper to trace and closer to
    the polycarbonate that is actually on the guards.
    """
    key = f"clear_{name}"
    if key in _CACHE:
        return _CACHE[key]
    material = OmniPBR(prim_path=f"{LOOKS}/{key}", name=key,
                       color=np.array(colour, dtype=float))
    material.set_reflection_roughness(float(roughness))
    shader = _shader_of(material)
    if shader:
        shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(
            float(opacity))
        # Threshold 0 means blend rather than cut out. Left at its
        # default the panels came out solid white, which is worse than
        # no panel at all.
        shader.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
    _CACHE[key] = material
    return material


def emissive(cfg: Dict[str, Any], name: str, colour, intensity: float):
    """A material that glows: light fittings, indicator lamps.

    Emission does not light the scene -- that is what the lights are
    for -- but a fitting that is dark where it should be bright is the
    first thing the eye catches in a photo of a factory.
    """
    key = f"emit_{name}"
    if key in _CACHE:
        return _CACHE[key]
    material = OmniPBR(prim_path=f"{LOOKS}/{key}", name=key,
                       color=np.array(colour, dtype=float))
    material.set_reflection_roughness(0.4)
    shader = UsdShade.Shader(material.shaders_list[0].GetPrim()) \
        if getattr(material, "shaders_list", None) else None
    prim = material.prim
    for child in prim.GetChildren():
        if child.GetTypeName() == "Shader":
            shader = UsdShade.Shader(child)
            break
    if shader:
        shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(
            tuple(float(c) for c in colour))
        shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(
            float(intensity))
    _CACHE[key] = material
    return material


def available(cfg: Dict[str, Any]) -> bool:
    """Materials need no asset server, but the scene may ask for boxes."""
    return bool(cfg.get("assets", {}).get("materials_enabled", True)) or props.enabled(cfg)
