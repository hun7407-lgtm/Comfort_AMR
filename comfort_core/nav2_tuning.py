"""Nav2 tuning arithmetic, shared by the simulator and the real AMR.

The two robots cannot share a parameter file. The simulator runs Jazzy and
renders configs/nav2/nav2_params.template.yaml through
scripts/generate_nav2_params.py; the real AMR runs Humble and builds its
params on the robot, in results/real/amr_deploy/bin/make_nav2_params.py,
starting from Humble's own installed defaults. Plugin names differ between
the distros ("nav2_navfn_planner/NavfnPlanner" against
"nav2_navfn_planner::NavfnPlanner") and Jazzy's `navigators:` block does
not exist in Humble at all.

What they CAN share is the arithmetic, and they have to: a tuning change
that lands on one robot and not the other turns the digital twin into two
unrelated robots. Every value below is computed from the loaded config, so
the simulated and the real machine get DIFFERENT numbers out of the SAME
formula -- which is the point, because they are different sizes and drive
at different speeds.

    simulated   footprint 0.70 x 0.48    0.80 m/s
    real        footprint 0.864 x 0.570  0.20 m/s

Nothing here is specific to a distro or to a scene.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Tuple


# DWB's own defaults, used when a config predates this block.
_DWB_DEFAULTS: Dict[str, Any] = {
    "sim_time": 1.7,
    "linear_granularity": 0.05,
    "angular_granularity": 0.025,
    "vx_samples": 20,
    "vtheta_samples": 20,
    "path_dist_scale": 32.0,
    "path_align_scale": 32.0,
    "goal_dist_scale": 24.0,
    "goal_align_scale": 24.0,
    "rotate_to_goal_scale": 32.0,
    "cost_scaling_factor": 5.0,
}


def _dwb(cfg: Mapping[str, Any]) -> Dict[str, Any]:
    block = dict(_DWB_DEFAULTS)
    block.update(cfg.get("navigation", {}).get("dwb", {}) or {})
    return block


def _base_obstacle(cfg: Mapping[str, Any]) -> Dict[str, Any]:
    block = {"sum_scores": True, "avoid_margin": 1.5, "scale": None}
    block.update(_dwb(cfg).get("base_obstacle", {}) or {})
    return block


def half_extents(cfg: Mapping[str, Any]) -> Tuple[float, float]:
    """Half the AMR's length and width, in metres."""
    amr = cfg["amr"]
    return (float(amr["footprint_length"]) / 2.0,
            float(amr["footprint_width"]) / 2.0)


def footprint_polygon(cfg: Mapping[str, Any]) -> str:
    """The footprint as Nav2 wants it: a polygon string, not a radius."""
    half_len, half_wid = half_extents(cfg)
    return (f"[[{half_len:.3f}, {half_wid:.3f}], "
            f"[{half_len:.3f}, {-half_wid:.3f}], "
            f"[{-half_len:.3f}, {-half_wid:.3f}], "
            f"[{-half_len:.3f}, {half_wid:.3f}]]")


def clearance_radius(cfg: Mapping[str, Any]) -> float:
    """What the robot's centre needs free on either side to pass.

    The same number comfort_core.layout uses to decide whether a gap is
    passable, so "the geometry says this gap is open" and "Nav2 will drive
    through it" cannot drift apart.
    """
    _, half_wid = half_extents(cfg)
    margin = float(cfg["navigation"].get("passage_margin", 0.10))
    return half_wid + margin


def inflation_radius(cfg: Mapping[str, Any]) -> float:
    """Inflation has to reach past the inscribed radius or the planner
    clips corners; much past it and the robot refuses gaps it fits
    through. Derived as the clearance above.

    There is a hard floor at the inscribed radius -- inside it the cost is
    lethal whatever this says, which for the real AMR is 0.285 m -- and a
    soft ceiling set by the corridor rather than by the robot: in a 2.2 m
    corridor the derived 0.385 m leaves less centre channel than the
    0.35 m measured on the real machine. That is a property of the
    corridor, so a config that drives a narrow one pins the number with
    navigation.dwb.inflation_radius and says why.
    """
    pinned = _dwb(cfg).get("inflation_radius")
    if pinned is not None:
        return max(float(pinned), half_extents(cfg)[1])
    return clearance_radius(cfg)


def cost_scaling_factor(cfg: Mapping[str, Any]) -> float:
    """Higher means cost falls off faster with distance, which leaves a
    clearer low-cost channel down the middle of a narrow gap. At DWB's
    default 3.0 the cost from two facing obstacles overlapped across the
    whole opening and there was no good trajectory to pick.
    """
    return float(_dwb(cfg)["cost_scaling_factor"])


def horizon_poses(cfg: Mapping[str, Any]) -> float:
    """How many poses DWB puts in one candidate trajectory.

    sim_time seconds at the top speed, sampled every linear_granularity
    metres. This is what makes a summed critic speed-dependent, and the
    reason BaseObstacle.scale cannot be one number for both robots:
    the simulated AMR covers 1.36 m in its horizon and the real one
    0.34 m, so the same trajectory accumulates four times the cost in
    simulation.
    """
    dwb = _dwb(cfg)
    speed = float(cfg["navigation"]["max_linear_speed"])
    reach = float(dwb["sim_time"]) * speed
    return max(1.0, reach / float(dwb["linear_granularity"]))


def path_pressure_scale(cfg: Mapping[str, Any]) -> float:
    """Total weight pulling the robot back onto the global path.

    PathDist and PathAlign are both MapGridCritics and both respond to
    lateral offset, so the pressure to stay on the path is their sum.
    """
    dwb = _dwb(cfg)
    return float(dwb["path_dist_scale"]) + float(dwb["path_align_scale"])


def mask_value(cfg: Mapping[str, Any]) -> float:
    """The cost comfort_publisher.py paints inside a Comfort Space.

    Deliberately short of 100, which KeepoutFilter turns into lethal: the
    Comfort Space is a preference, not an obstacle.
    """
    costmap = cfg.get("comfort_space", {}).get("costmap", {}) or {}
    return float(costmap.get("mask_value", 98))


def base_obstacle_sum_scores(cfg: Mapping[str, Any]) -> bool:
    """Whether BaseObstacle sums the cost along the trajectory.

    DWB's default is False, and dwb_critics/base_obstacle.hpp says what
    that means: the critic "will return either the final pose's cost, or
    the sum of all poses, depending on the sum_scores parameter". The
    final pose only -- so a trajectory that drives clean through a
    Comfort Space and ends outside it scores zero. In simulation the
    horizon reaches 1.36 m, further than a Comfort Space is wide, so
    crossing one was free.

    True makes the cost proportional to how far INTO the space the
    trajectory goes and how long it stays, which is the graded response
    the Comfort Space is supposed to produce. It also means the scale has
    to be recomputed, because the numbers are now a sum over ~27 poses
    rather than one.
    """
    return bool(_base_obstacle(cfg)["sum_scores"])


def base_obstacle_scale(cfg: Mapping[str, Any]) -> float:
    """What one unit of costmap cost is worth against staying on the path.

    Verified against the installed headers rather than assumed:

      * MapGridCritic (PathDist, PathAlign, GoalDist, GoalAlign) scores in
        CELLS and overrides getScale() as `resolution * 0.5 * scale`
        (dwb_critics/map_grid.hpp:69). So deviating d metres costs
        (d / resolution) * (resolution * 0.5 * scale) = d * 0.5 * scale.
        The resolution cancels: the figure is in metres and the same at
        any grid size.

      * BaseObstacleCritic does NOT override getScale(), so its score is
        the raw costmap value times the scale.

    With sum_scores on, a trajectory of n poses sitting in a mask of value
    m costs n * m * scale. Set that equal to `avoid_margin` times the
    cheapest sidestep that actually clears the space:

        n * m * scale = avoid_margin * d_clear * 0.5 * path_pressure

    and solve for the scale. A trajectory that spends its whole horizon
    inside someone's Comfort Space therefore costs avoid_margin times
    more than driving around it, while one that merely clips the edge
    costs proportionally less -- it is tolerated, which is what makes the
    response graded instead of binary.

    Set navigation.dwb.base_obstacle.scale to pin the number instead.
    """
    block = _base_obstacle(cfg)
    pinned = block.get("scale")
    if pinned is not None:
        return float(pinned)

    sidestep = (float(block["avoid_margin"]) * clearance_radius(cfg)
                * 0.5 * path_pressure_scale(cfg))
    if not base_obstacle_sum_scores(cfg):
        # One pose is scored, not the horizon, so there is nothing to
        # divide by. This is DWB's own behaviour and it cannot grade an
        # intrusion; kept only so an older config still renders.
        return sidestep / mask_value(cfg)
    return sidestep / (horizon_poses(cfg) * mask_value(cfg))


def observation_ranges(cfg: Mapping[str, Any]) -> Tuple[float, float]:
    """How far the obstacle layer marks and raytrace-clears, in metres.

    This bounds the Comfort Space too, and not obviously: CostmapFilter
    does not widen the costmap's update bounds itself, so a mask painted
    beyond the obstacle layer's reach never arrives. Measured on the real
    AMR -- a mask correctly published at x=5.28 left the costmap reading 0
    there, while the same mask at x=2.24 came through at 96.

    Defaults are the simulated LiDAR's. The real machine's reach is
    longer and is set in its own config, not here.
    """
    obs = cfg["navigation"].get("observation", {}) or {}
    return (float(obs.get("obstacle_max_range", 6.0)),
            float(obs.get("raytrace_max_range", 8.0)))


def speed_limits(cfg: Mapping[str, Any]) -> Tuple[float, float]:
    """Top linear and angular speed the controller may command."""
    nav = cfg["navigation"]
    return (float(nav["max_linear_speed"]), float(nav["max_angular_speed"]))


def dwb_horizon(cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """The trajectory-generator settings, which the scale derivation reads
    and so cannot be left sitting in the template as literals."""
    dwb = _dwb(cfg)
    return {
        "sim_time": float(dwb["sim_time"]),
        "linear_granularity": float(dwb["linear_granularity"]),
        "angular_granularity": float(dwb["angular_granularity"]),
        "vx_samples": int(dwb["vx_samples"]),
        "vtheta_samples": int(dwb["vtheta_samples"]),
    }


def critic_scales(cfg: Mapping[str, Any]) -> Dict[str, float]:
    """Every critic weight, so neither generator keeps its own copy."""
    dwb = _dwb(cfg)
    return {
        "BaseObstacle.scale": base_obstacle_scale(cfg),
        "PathDist.scale": float(dwb["path_dist_scale"]),
        "PathAlign.scale": float(dwb["path_align_scale"]),
        "GoalDist.scale": float(dwb["goal_dist_scale"]),
        "GoalAlign.scale": float(dwb["goal_align_scale"]),
        "RotateToGoal.scale": float(dwb["rotate_to_goal_scale"]),
    }


def summary(cfg: Mapping[str, Any], label: Optional[str] = None) -> str:
    """One block of text both generators print, so a tuning change can be
    compared between the two robots by reading their logs side by side."""
    half_len, half_wid = half_extents(cfg)
    v_lin, v_ang = speed_limits(cfg)
    obstacle_range, raytrace_range = observation_ranges(cfg)
    n = horizon_poses(cfg)
    dwb = dwb_horizon(cfg)
    traverse = n * mask_value(cfg) * base_obstacle_scale(cfg)
    sidestep = clearance_radius(cfg) * 0.5 * path_pressure_scale(cfg)
    lines = [
        f"[nav2_tuning] {label or 'derived'}",
        f"  footprint          {2 * half_len:.3f} x {2 * half_wid:.3f} m",
        f"  speed              {v_lin:.3f} m/s, {v_ang:.3f} rad/s",
        f"  clearance radius   {clearance_radius(cfg):.3f} m",
        f"  inflation radius   {inflation_radius(cfg):.3f} m"
        f" (scaling {cost_scaling_factor(cfg):.1f})",
        f"  observation        mark {obstacle_range:.1f} m,"
        f" raytrace {raytrace_range:.1f} m",
        f"  DWB horizon        {dwb['sim_time']:.2f} s ="
        f" {dwb['sim_time'] * v_lin:.3f} m = {n:.1f} poses",
        f"  mask value         {mask_value(cfg):.0f}",
        f"  BaseObstacle       scale {base_obstacle_scale(cfg):.6f},"
        f" sum_scores {base_obstacle_sum_scores(cfg)}",
        f"  full traversal     {traverse:.2f}"
        f"  vs sidestep {sidestep:.2f}"
        f"  (ratio {traverse / sidestep if sidestep else float('nan'):.2f})",
    ]
    return "\n".join(lines)
