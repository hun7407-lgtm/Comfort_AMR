"""Put animated characters and station equipment into the scene.

Isaac Sim only: imports omni and pxr.

Appearance only. The colliders stay exactly where isaac_sim/environments/
corridor.py put them, and they keep the sizes written in
configs/common.yaml. Characters and equipment are referenced on top and
made non-physical, for three reasons:

  * the character assets ship with no physics at all, so they could not
    act as obstacles even if we wanted them to;
  * the size of a person and of a workstation are experiment parameters.
    Burying them inside an art asset would take them out of the config
    and out of the paper;
  * the UR10e is an articulation. Adding it as a physical object would
    hand PhysX six more joints on top of an AMR whose solver stability
    had to be worked at.

The LiDAR still sees the real shapes, because the RTX sensor traces
against renderable geometry. So the point cloud gets a person-shaped
return while the collision and the geometry checks stay on the cylinder.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

import carb
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import create_prim, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage, is_stage_loading
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom

from comfort_core.human_state import make_state_provider
from comfort_core.layout import human_positions, placed_entries

CHARACTERS_ROOT = "/World/Characters"
BIPED_PRIM = f"{CHARACTERS_ROOT}/Biped_Setup"
EQUIPMENT_ROOT = "/Environment/Corridor/Equipment"

# Extensions the animated characters need. Loading them costs a few
# seconds, so they are only enabled when characters are switched on.
CHARACTER_EXTENSIONS = (
    "omni.anim.graph.core",
    "omni.anim.retarget.core",
    "omni.anim.people",
    "omni.kit.scripting",
)

COMMAND_FILE_SETTING = "/exts/omni.anim.people/command_settings/command_file_path"
# omni.anim.people resolves a character's name by taking the path segment
# directly under this root. With the root at /World/Characters, a
# SkelRoot at /World/Characters/human_a/<asset>/... answers to "human_a".
CHARACTER_PRIM_PATH_SETTING = "/persistent/exts/omni.anim.people/character_prim_path"
NAVMESH_SETTING = "/exts/omni.anim.people/navigation_settings/navmesh_enabled"
AVOIDANCE_SETTING = "/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled"


def enable_extensions(cfg: Dict[str, Any]) -> bool:
    """Turn on the character extensions, before the stage is created.

    Order matters and the failure is silent. Enabling these part-way
    through building a scene leaves omni.anim.graph.core unable to
    register any character: get_character() returns None for every
    SkelRoot, no animation ever plays, and the characters stand in bind
    pose looking like a T. Enabling them immediately after SimulationApp
    and before the stage exists works.

    Call this from the entry point, not from build_scene.
    """
    if not cfg["humans"].get("character", {}).get("enabled", False):
        return False

    import omni.kit.app
    for ext in CHARACTER_EXTENSIONS:
        enable_extension(ext)
    app = omni.kit.app.get_app()
    for _ in range(20):
        app.update()
    return True


def _hide(prim: Usd.Prim) -> None:
    UsdGeom.Imageable(prim).MakeInvisible()


def _settle_stage(frames: int = 6) -> None:
    """Let referenced payloads finish loading before inspecting them.

    Character and biped assets come from the cloud, and create_prim
    returns before the payload is composed. Searching for the
    AnimationGraph prim too early finds nothing, the graph never gets
    bound, and the characters stand in bind pose. That is what a T-pose
    on screen means.
    """
    import omni.kit.app
    app = omni.kit.app.get_app()
    for _ in range(frames):
        app.update()
    guard = 0
    while is_stage_loading() and guard < 600:
        app.update()
        guard += 1


def _facing_yaw(cfg: Dict[str, Any], key: str, hx: float, hy: float) -> float:
    """Degrees of rotation about Z so the character faces their station."""
    if str(cfg["humans"]["character"].get("facing", "station")) != "station":
        return 0.0
    offset = float(cfg["humans"]["character"].get("facing_offset_deg", 0.0))
    # A person can say which way they face. The factory bay has no
    # `stations` entry per person -- the machines are scene geometry --
    # so without this everyone there would face the same way.
    own = cfg["humans"].get(key)
    if isinstance(own, dict) and own.get("facing_deg") is not None:
        return float(own["facing_deg"]) + offset
    station = cfg["stations"].get(key)
    if not isinstance(station, dict):
        return 0.0
    sx, sy = (float(v) for v in station["position"])
    return math.degrees(math.atan2(sy - hy, sx - hx)) + offset


def hide_human_cylinders(cfg: Dict[str, Any]) -> int:
    """Make the human colliders invisible, leaving their physics intact.

    The RTX LiDAR traces what renders, so an invisible collider is also
    invisible to the sensor and the characters are what the robot sees.
    Set humans.character.hide_colliders false to keep them visible, which
    is how their contribution to /scan can be measured.
    """
    if not cfg["humans"]["character"].get("hide_colliders", True):
        return 0
    stage = get_current_stage()
    hidden = 0
    for key, _x, _y in human_positions(cfg):
        prim = stage.GetPrimAtPath(f"/Environment/Corridor/human_{key}")
        if prim.IsValid():
            _hide(prim)
            hidden += 1
    return hidden


def add_station_equipment(cfg: Dict[str, Any]) -> List[str]:
    """Reference a piece of equipment onto the top of each station."""
    eq = cfg["stations"].get("equipment", {})
    if not eq.get("enabled", False):
        return []
    # No corridor, no corridor stations: in the factory bay these would
    # be two boxes floating over the assembly line.
    if not cfg.get("corridor", {}).get("enabled", True):
        return []

    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_warn("Isaac assets root unavailable; skipping station equipment")
        return []

    stations = cfg["stations"]
    _sx, _sy, station_h = (float(v) for v in stations["size"])
    scale = float(eq.get("scale", 1.0))
    lift = float(eq.get("z_lift", 0.0))

    added = []
    for key in placed_entries(stations):
        rel = eq.get(key)
        if not rel:
            continue
        x, y = (float(v) for v in stations[key]["position"])
        path = f"{EQUIPMENT_ROOT}/{key}"
        prim = create_prim(
            path, "Xform",
            position=[x, y, station_h + lift],
            scale=[scale, scale, scale],
            usd_path=assets_root + rel,
        )
        # Strip physics: this is scenery. Anything left enabled would add
        # colliders and, for the UR10e, a whole articulation.
        for child in Usd.PrimRange(prim):
            for attr_name in ("physics:collisionEnabled", "physics:rigidBodyEnabled"):
                attr = child.GetAttribute(attr_name)
                if attr:
                    attr.Set(False)
        added.append(f"{key}: {rel.rsplit('/', 1)[-1]}")
    return added


def spawn_characters(cfg: Dict[str, Any]) -> List[str]:
    """Load an animated character in front of each station.

    Follows the same sequence isaacsim.replicator.agent uses: load
    Biped_Setup once for its animation graph, reference each character,
    bind the graph to the character's SkelRoot, attach omni.anim.people's
    behaviour script, and point that script at a command file.
    """
    char_cfg = cfg["humans"].get("character", {})
    if not char_cfg.get("enabled", False):
        return []

    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_warn("Isaac assets root unavailable; skipping characters")
        return []

    from omni.anim.people.scripts.custom_command.populate_anim_graph import (  # noqa: E402
        populate_anim_graph,
    )

    settings = carb.settings.get_settings()
    settings.set(CHARACTER_PRIM_PATH_SETTING, CHARACTERS_ROOT)
    # The corridor has no navigation mesh and the characters do not walk
    # yet, so both are off. They come back when a walking human is added.
    settings.set(NAVMESH_SETTING, False)
    # Character-to-character avoidance stays off. The walker must follow
    # the same script in B2 and in B3; if it re-routed around anything
    # the two runs would stop being comparable.
    settings.set(AVOIDANCE_SETTING, False)
    # No looping. The walker crosses once and then stands where they
    # arrived. Pacing back and forth would keep re-blocking the corridor
    # and make every trial a question of when the robot happened to
    # start, rather than what the Comfort Space policy does.
    settings.set("/exts/omni.anim.people/command_settings/number_of_loop", "0")

    stage = get_current_stage()
    if not is_prim_path_valid(CHARACTERS_ROOT):
        create_prim(CHARACTERS_ROOT, "Xform")

    if not is_prim_path_valid(BIPED_PRIM):
        biped = create_prim(
            BIPED_PRIM, "Xform",
            usd_path=assets_root + "/Isaac/People/Characters/Biped_Setup.usd",
        )
        _hide(biped)

    # The biped asset carries the animation graph the characters are
    # retargeted onto. It has to be composed before populate_anim_graph
    # can find anything to populate.
    _settle_stage()
    populate_anim_graph()
    _settle_stage(2)

    anim_graph = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(BIPED_PRIM)):
        if prim.GetTypeName() == "AnimationGraph":
            anim_graph = prim
            break
    if anim_graph is None:
        carb.log_warn(
            "no AnimationGraph found under the biped setup; characters will "
            "stand in bind pose")

    behavior_script = _behavior_script_path()
    command_lines = []
    skel_roots = []
    loaded = []

    for key, hx, hy in human_positions(cfg):
        name = char_cfg.get(key)
        if not name:
            continue
        prim_path = f"{CHARACTERS_ROOT}/human_{key}"
        yaw = _facing_yaw(cfg, key, hx, hy)
        create_prim(
            prim_path, "Xform",
            position=[hx, hy, 0.0],
            usd_path=f"{assets_root}/Isaac/People/Characters/{name}/{name}.usd",
        )
        _settle_stage(2)
        _face(stage.GetPrimAtPath(prim_path), hx, hy, yaw)

        skel = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
            if prim.GetTypeName() == "SkelRoot":
                skel = prim
                break
        if skel is not None:
            skel_roots.append(skel)
            # The command file is keyed on the folder under
            # CHARACTERS_ROOT, which is what character_behavior.py's
            # get_agent_name() returns. Using the SkelRoot's own name
            # here matched nothing, no command ever ran, and the
            # characters stayed in bind pose looking like a T.
            agent_name = f"human_{key}"
            state = make_state_provider(cfg).states().get(key, "stationary")
            spec = cfg["humans"][key]
            rendezvous = float(cfg["navigation"].get("trial_start_sim_time", 0.0))
            # Two ways to be given a walk.
            #
            #   route:            set off at the rendezvous. A person who
            #                     is walking when the trial begins.
            #   walk.start_at:    set off that many seconds INTO the
            #                     trial. A person who is working, stops,
            #                     and walks off mid-trial -- which is
            #                     what makes an obstacle appear in a
            #                     lane that was clear a moment ago.
            walk = spec.get("walk") or {}
            route = walk.get("route") or spec.get("route") or []
            delay = rendezvous + float(walk.get("start_at", 0.0))
            if route:
                command_lines.extend(_route_commands(agent_name, route, delay))
            else:
                command = char_cfg.get("commands", {}).get(state, "Idle 100000")
                command_lines.append(f"{agent_name} {command}")

        loaded.append(f"{key}: {name} facing {yaw:+.0f} deg")

    missing = [k for k, _x, _y in human_positions(cfg)
               if char_cfg.get(k) and not any(
                   f"human_{k}" in str(p.GetPrimPath()) for p in skel_roots)]
    if missing:
        carb.log_warn(f"no SkelRoot found for character(s) {missing}")

    if skel_roots and anim_graph is not None:
        paths = [Sdf.Path(p.GetPrimPath()) for p in skel_roots]
        omni.kit.commands.execute("RemoveAnimationGraphAPICommand", paths=paths)
        omni.kit.commands.execute(
            "ApplyAnimationGraphAPICommand", paths=paths,
            animation_graph_path=Sdf.Path(anim_graph.GetPrimPath()))

        if behavior_script:
            omni.kit.commands.execute("RemoveScriptingAPICommand", paths=paths)
            omni.kit.commands.execute("ApplyScriptingAPICommand", paths=paths)
            for prim in skel_roots:
                prim.GetAttribute("omni:scripting:scripts").Set([behavior_script])
            # omni.kit.scripting instantiates a behaviour object on the
            # next app update, and that object is what receives on_play.
            # Attaching the script and then starting the timeline in the
            # same breath means the objects do not exist yet, nobody sees
            # the play event, no command is ever read, and the characters
            # hold their bind pose.
            _settle_stage(10)

    loaded.append(
        f"animation graph {'bound to' if anim_graph is not None else 'MISSING for'} "
        f"{len(skel_roots)} character(s)")

    if command_lines:
        command_file = _write_command_file(command_lines)
        settings.set(COMMAND_FILE_SETTING, command_file)
        loaded.append(f"commands: {command_file}")

    return loaded


def _route_commands(agent_name: str, route, start_delay: float = 0.0) -> List[str]:
    """One GoTo per waypoint, so the walk is a sequence of straight legs.

    The trailing "_" tells omni.anim.people to leave the final heading
    alone, which keeps the character facing along the corridor instead of
    snapping to a fixed angle at every waypoint.

    A navigation mesh is not needed. With navmesh disabled the extension
    walks straight from point to point, and the corridor is straight.
    Baking a mesh would add a step and change nothing.
    """
    lines = []
    if start_delay > 0.0:
        # Wait until the trial begins. The AMR is given its goal at the
        # same point on the simulation clock, so the two set off
        # together instead of the person having finished crossing while
        # Nav2 was still starting up.
        lines.append(f"{agent_name} Idle {start_delay}")
    for point in route:
        x, y = (float(v) for v in point[:2])
        lines.append(f"{agent_name} GoTo {x} {y} 0 _")
    # Stand still at the far end for the rest of the trial. Without a
    # trailing command the behaviour script runs out of work and the
    # character can snap back to its bind pose.
    lines.append(f"{agent_name} Idle 100000")
    return lines


def _face(prim: Usd.Prim, x: float, y: float, yaw_deg: float) -> None:
    """Place and rotate a referenced character.

    create_prim's `orientation` argument goes unused here. Referencing an
    asset brings its own xformOpOrder along, and if the asset authored
    rotateXYZ rather than orient the quaternion is written to an op that
    is not in the order and has no effect. The characters then all faced
    +x regardless of what was asked for. Writing whichever ops the asset
    actually has avoids guessing.
    """
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}

    if "xformOp:translate" in ops:
        ops["xformOp:translate"].Set(Gf.Vec3d(float(x), float(y), 0.0))
    else:
        xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0))

    half = math.radians(yaw_deg) / 2.0
    quat = Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))
    if "xformOp:orient" in ops:
        op = ops["xformOp:orient"]
        current = op.Get()
        op.Set(Gf.Quatf(quat) if isinstance(current, Gf.Quatf) else quat)
    elif "xformOp:rotateXYZ" in ops:
        ops["xformOp:rotateXYZ"].Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    else:
        xf.AddOrientOp().Set(quat)


def _behavior_script_path() -> str:
    import omni.kit.app
    manager = omni.kit.app.get_app().get_extension_manager()
    ext_path = manager.get_extension_path_by_module("omni.anim.people")
    if not ext_path:
        return ""
    return f"{ext_path}/omni/anim/people/scripts/character_behavior.py"


def _write_command_file(lines: List[str]) -> str:
    """Write the omni.anim.people command file next to the results.

    Kept on disk rather than in memory because the behaviour script reads
    it by path, and because having the exact commands a trial ran with is
    worth keeping alongside the trial.
    """
    from comfort_core import resolve_path
    out = resolve_path("results/character_commands.txt")
    out.parent.mkdir(parents=True, exist_ok=True)

    # MERGED, not overwritten. Two things call this: the experiment's
    # own people, and the figures that belong to the machines. They are
    # spawned by separate passes, and whichever wrote the file last used
    # to erase the other one's commands -- the symptom was a worker who
    # had been given a walk standing at her machine for the whole trial
    # with no error anywhere.
    #
    # Commands are keyed on the agent name in the first column. A pass
    # that names an agent replaces every line that agent had; agents it
    # does not mention are left alone.
    fresh = {line.split(" ", 1)[0] for line in lines if line.strip()}
    kept = []
    if out.is_file():
        for line in out.read_text(encoding="utf-8").splitlines():
            if line.strip() and line.split(" ", 1)[0] not in fresh:
                kept.append(line)
    out.write_text("\n".join(kept + list(lines)) + "\n", encoding="utf-8")
    return str(out)


def populate(cfg: Dict[str, Any]) -> List[str]:
    """Add every appearance-only prop. Returns lines describing what was added."""
    notes: List[str] = []

    equipment = add_station_equipment(cfg)
    if equipment:
        notes.append("station equipment: " + ", ".join(equipment))

    characters = spawn_characters(cfg)
    if characters:
        hidden = hide_human_cylinders(cfg)
        notes.append(f"hid {hidden} human collider(s), now invisible but still solid")
        notes.extend("character " + c for c in characters)

    return notes
