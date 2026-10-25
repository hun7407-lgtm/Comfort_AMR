"""The people who work at the machines.

Isaac Sim only. A worker is an invisible cylinder collider -- the same
person the corridor scene uses, at the same radius and height from
configs/common.yaml -- with a simple figure drawn around it so the
scene reads at a glance and the LiDAR gets a person-shaped return at
0.31 m instead of a bare post.

When the asset server is reachable, these are the same animated
characters the experiment uses, standing at their machines. Without it
-- or if the character extensions are not loaded -- they fall back to
a figure made of boxes, which is enough to see where a person is.

Each worker is one Xform at their standing spot, so moving a person in
the viewport is dragging one prim.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

import copy

import numpy as np
from isaacsim.core.api.objects import FixedCylinder, VisualCuboid
from isaacsim.core.utils.prims import create_prim
from pxr import UsdGeom

from comfort_core.factory_layout import BayLayout, Station
from isaac_sim.environments import props

Vec3 = Tuple[float, float, float]

BODY: Dict[str, Vec3] = {
    "skin": (0.80, 0.64, 0.52),
    "vest": (0.85, 0.80, 0.15),      # hi-vis
    "shirt": (0.30, 0.42, 0.58),
    "trousers": (0.22, 0.24, 0.30),
    "helmet": (0.92, 0.92, 0.94),
    "boots": (0.14, 0.14, 0.16),
}


def _quat(angle: float) -> np.ndarray:
    return np.array([math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)])


def _figure(height: float) -> List[Tuple[str, Vec3, Vec3, str]]:
    """A person as eight boxes, scaled to the configured height.

    Proportions are the usual 7.5-head figure, rounded off. Local frame:
    +y is the way they are facing, z up from the floor.
    """
    h = height
    return [
        ("boot_l", (-0.10, 0.02, 0.04 * h / 1.7), (0.15, 0.26, 0.08), "boots"),
        ("boot_r", (+0.10, 0.02, 0.04 * h / 1.7), (0.15, 0.26, 0.08), "boots"),
        ("leg_l", (-0.10, 0.0, 0.26 * h / 1.7), (0.15, 0.17, 0.42), "trousers"),
        ("leg_r", (+0.10, 0.0, 0.26 * h / 1.7), (0.15, 0.17, 0.42), "trousers"),
        ("hips", (0.0, 0.0, 0.52 * h / 1.7), (0.34, 0.20, 0.18), "trousers"),
        ("torso", (0.0, 0.0, 0.76 * h / 1.7), (0.40, 0.23, 0.34), "shirt"),
        ("vest", (0.0, 0.0, 0.76 * h / 1.7), (0.43, 0.26, 0.26), "vest"),
        ("arm_l", (-0.25, 0.02, 0.74 * h / 1.7), (0.11, 0.14, 0.44), "shirt"),
        ("arm_r", (+0.25, 0.02, 0.74 * h / 1.7), (0.11, 0.14, 0.44), "shirt"),
        ("neck", (0.0, 0.0, 0.94 * h / 1.7), (0.10, 0.10, 0.06), "skin"),
        ("head", (0.0, 0.0, 1.02 * h / 1.7), (0.18, 0.21, 0.22), "skin"),
        ("helmet", (0.0, -0.01, 1.11 * h / 1.7), (0.21, 0.24, 0.09), "helmet"),
    ]


def _colour(cfg: Dict[str, Any], name: str) -> np.ndarray:
    override = cfg.get("equipment", {}).get("worker_colors", {})
    return np.array(override.get(name, BODY[name]), dtype=float)


def _posts(stations: List[Station]):
    """Every (station, worker) pair, in a stable order.

    A Station carries all of its people, and most of this file used to
    read `station.worker`, which is only the first. One belt with ten
    posts along it then put one person on the floor.
    """
    for station in stations:
        for worker in station.workers:
            yield station, worker


def _walkers_at_posts(cfg: Dict[str, Any], stations: List[Station]):
    """Posts whose person is one of the experiment's own walking people.

    A worker at a machine is drawn twice over in this scene: once as a
    post belonging to the machine, and once as an entry in `humans`
    with a Comfort Space and a tracked collider. While everybody stands
    still that costs nothing. The moment one of them is given a walk it
    matters, because only an entry in `humans` can be given one -- the
    post is a fixture of the machine.

    So a post standing where a walking person stands is left out
    entirely, character and collider both, and `humans` owns that
    person. Otherwise the post's own figure would stay at the machine
    while the person walked away from it.
    """
    humans = cfg.get("humans", {})
    walking = [(k, v["position"]) for k, v in humans.items()
               if isinstance(v, dict) and v.get("walk") and "position" in v
               and v.get("enabled", True)]
    skip = set()
    for station in stations:
        for i, (wx, wy, _f) in enumerate(station.workers, start=1):
            for key, (hx, hy) in walking:
                if abs(float(hx) - wx) < 0.15 and abs(float(hy) - wy) < 0.15:
                    skip.add((station.name, i))
    return skip


def _character_config(cfg: Dict[str, Any],
                      stations: List[Station]) -> Dict[str, Any]:
    """A humans section describing the people at the machines.

    isaac_sim/environments/people.py spawns whoever the `humans`
    section names. The bay's workers are not in that section -- they
    come from the machines -- so one is written for it here, with the
    experiment's own people left out so nobody is spawned twice.
    """
    names = cfg.get("equipment", {}).get(
        "worker_characters", ["F_Business_02", "M_Medical_01"])
    humans = {k: v for k, v in cfg.get("humans", {}).items()
              if not (isinstance(v, dict) and "position" in v)}
    character = dict(humans.get("character", {}))
    character["enabled"] = True
    # Every post at every machine. A belt with five people along it
    # used to contribute one: `station.worker` is only the first of
    # them, and reading it here left most of the line unstaffed.
    skip = _walkers_at_posts(cfg, stations)
    seen: Dict[str, int] = {}
    for i, (station, (wx, wy, facing)) in enumerate(list(_posts(stations))):
        # Numbered within the station, which is how the collider and
        # the skip list count them.
        seen[station.name] = seen.get(station.name, 0) + 1
        if (station.name, seen[station.name]) in skip:
            continue
        key = f"at_{station.name}_{i + 1:02d}"
        humans[key] = {
            "position": [float(wx), float(wy)],
            "state": "station_engaged",
            "facing_deg": math.degrees(facing),
        }
        character[key] = names[i % len(names)]
    humans["character"] = character
    out = copy.copy(cfg)
    out["humans"] = humans
    return out


def _collider(cfg: Dict[str, Any], station: Station, root: str,
              skip=frozenset()) -> None:
    """The cylinder the study has always used for a person.

    Kept whichever way the person is drawn, so the geometry the planner
    and the clearance metric reason about does not change with the art.
    """
    humans = cfg.get("humans", {})
    radius = float(humans.get("radius", 0.25))
    height = float(humans.get("height", 1.70))
    for i, (x, y, _facing) in enumerate(station.workers, start=1):
        if (station.name, i) in skip:
            continue
        body = FixedCylinder(
            prim_path=f"{root}/Workers/{station.name}/post_{i:02d}/collider",
            position=np.array([x, y, height / 2.0]),
            radius=radius,
            height=height,
        )
        UsdGeom.Imageable(body.prim).MakeInvisible()


def build_worker(cfg: Dict[str, Any], station: Station, root: str) -> int:
    """One person, standing at their machine and facing it."""
    humans = cfg.get("humans", {})
    radius = float(humans.get("radius", 0.25))
    height = float(humans.get("height", 1.70))

    built = 0
    for i, (x, y, facing) in enumerate(station.workers, start=1):
        # They face their machine: it fronts onto the aisle, so their
        # back is to the aisle. Each post carries its own facing, which
        # is how a belt worked from both sides comes out right.
        prim = f"{root}/Workers/{station.name}/post_{i:02d}"
        create_prim(prim, "Xform", position=[x, y, 0.0],
                    orientation=_quat(facing - math.pi / 2.0))

        for name, centre, size, colour in _figure(height):
            VisualCuboid(
                prim_path=f"{prim}/{name}",
                translation=np.array([float(v) for v in centre]),
                scale=np.array([float(v) for v in size]),
                color=_colour(cfg, colour),
            )

        # The collider is the cylinder the study has always used, so the
        # geometry the planner and the clearance metric reason about
        # does not change with how a person is drawn.
        body = FixedCylinder(
            prim_path=f"{prim}/collider",
            translation=np.array([0.0, 0.0, height / 2.0]),
            radius=radius,
            height=height,
        )
        UsdGeom.Imageable(body.prim).MakeInvisible()
        built += 1
    return built


def build_workers(cfg: Dict[str, Any], layout: BayLayout, root: str,
                  say=None) -> int:
    """Everyone who works in the bay."""
    spec = cfg.get("equipment", {})
    if not spec.get("enabled", True) or not spec.get("workers", True):
        return 0
    stations = [st for st in layout.stations if st.has_worker]
    if not stations:
        return 0

    if props.enabled(cfg) and cfg.get("humans", {}).get(
            "character", {}).get("enabled", False):
        from isaac_sim.environments import people
        try:
            loaded = people.spawn_characters(_character_config(cfg, stations))
        except Exception as exc:  # noqa: BLE001 - art is not worth failing over
            loaded = []
            if say:
                say(f"characters unavailable ({exc}); drawing figures instead")
        if loaded:
            skip = _walkers_at_posts(cfg, stations)
            for station in stations:
                _collider(cfg, station, root, skip)
            if say:
                say(f"workers: {len(loaded)} characters at machines")
            return len(loaded)

    built = sum(build_worker(cfg, st, root) for st in stations)
    if say:
        say(f"workers: {built} figures at machines")
    return built
