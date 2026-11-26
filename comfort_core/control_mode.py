"""Which motion-control chain a run uses, and the checks that prove it.

Two modes, chosen per run config by `control.mode`:

    legacy_gate  (default; every run before 2026-11-26 night)
                 AMR1: Nav2 controller -> cmd_vel_nav -> comfort_gate (yield /
                 evade / backoff / off-lane) -> cmd_vel_safe -> collision_monitor
    nav2_only    docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md
                 every AMR: Nav2 controller / behaviors -> cmd_vel_safe ->
                 collision_monitor -> cmd_vel. comfort_gate is not launched, so
                 nothing else writes a velocity. The Comfort Space reaches the
                 robots only through the costmap filter mask.

The default keeps every older config exactly as it ran.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Sequence

LEGACY = "legacy_gate"
NAV2_ONLY = "nav2_only"
BUFFER_M = 0.10

# nodes allowed to publish the velocity that enters the collision monitor
NAV2_VELOCITY_SOURCES = ("controller_server", "behavior_server")
FORBIDDEN_NODES = ("comfort_gate",)


def mode(cfg: Dict[str, Any]) -> str:
    return str((cfg.get("control") or {}).get("mode", LEGACY))


def nav2_only(cfg: Dict[str, Any]) -> bool:
    return mode(cfg) == NAV2_ONLY


def config_problems(cfg: Dict[str, Any]) -> List[str]:
    """Merged-config requirements of nav2_only (empty list = OK)."""
    if not nav2_only(cfg):
        return []
    cs = cfg.get("comfort_space", {}) or {}
    y = cs.get("yield", {}) or {}
    rec = cfg.get("recovery", {}) or {}
    p = []
    if y.get("enabled", True):
        p.append("comfort_space.yield.enabled is not false")
    if (y.get("evade", {}) or {}).get("enabled", True):
        p.append("comfort_space.yield.evade.enabled is not false")
    if (rec.get("backoff", {}) or {}).get("enabled", True):
        p.append("recovery.backoff.enabled is not false")
    if (rec.get("off_lane", {}) or {}).get("enabled", True):
        p.append("recovery.off_lane.enabled is not false")
    if (cfg.get("traffic_rule", {}) or {}).get("enabled", False):
        p.append("traffic_rule.enabled is not false")
    if abs(float((cs.get("costmap", {}) or {}).get("buffer_width", -1)) - BUFFER_M) > 1e-9:
        p.append(f"comfort_space.costmap.buffer_width != {BUFFER_M}")
    if not ((cfg.get("navigation", {}) or {}).get("collision_monitor", {}) or {}).get("enabled", False):
        p.append("navigation.collision_monitor.enabled is not true")
    return p


def graph_problems(nodes: Iterable[str], pubs: Dict[str, Sequence[str]],
                   subs: Dict[str, Sequence[str]], namespaces: Sequence[str]) -> List[str]:
    """Runtime ROS graph requirements of nav2_only.

    nodes: every node name in the graph; pubs / subs: topic -> node names.
    namespaces: '' for AMR1 plus every fleet namespace ('a1', ...).
    """
    p = []
    names = list(nodes)
    for bad in FORBIDDEN_NODES:
        if any(n == bad or n.endswith("/" + bad) for n in names):
            p.append(f"node {bad} is running")
    for ns in namespaces:
        pre = f"/{ns}" if ns else ""
        safe, out, nav = f"{pre}/cmd_vel_safe", f"{pre}/cmd_vel", f"{pre}/cmd_vel_nav"
        sp = list(pubs.get(safe, []))
        if "controller_server" not in sp:
            p.append(f"{safe}: controller_server does not publish it (publishers {sp})")
        extra = [n for n in sp if n not in NAV2_VELOCITY_SOURCES]
        if extra:
            p.append(f"{safe}: non-Nav2 publishers {extra}")
        if "collision_monitor" not in list(subs.get(safe, [])):
            p.append(f"{safe}: collision_monitor does not subscribe")
        op = list(pubs.get(out, []))
        if op != ["collision_monitor"] and sorted(set(op)) != ["collision_monitor"]:
            p.append(f"{out}: publishers {op}, expected only collision_monitor")
        if pubs.get(nav):
            p.append(f"{nav}: still published by {list(pubs[nav])}")
    return p
