"""Scene construction helpers for Isaac Sim.

Isaac Sim only. This module imports omni and pxr, so it must never be
imported from a ROS 2 node. See docs/SETUP.md section 2.

The AMR USD from the reference repository contains the robot and its
ActionGraph but no PhysicsScene, no ground and no lighting. Everything
here supplies those missing pieces.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

import numpy as np
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

from comfort_core.comfort_space import make_policy, robot_grow, shapes as build_shapes
from comfort_core.human_state import make_state_provider
from comfort_core.layout import passage_report, validate
from isaac_sim.utils import comfort_viz, human_motion, human_tf
from isaac_sim.environments import corridor, people


def setup_physics_scene(cfg: Dict[str, Any], prim_path: str = "/physicsScene") -> Usd.Prim:
    """Create the PhysicsScene the AMR USD does not provide.

    The source USD ships the robot alone: no PhysicsScene, no ground and
    no lighting. Everything the solver needs is set here, from config, so
    B2 and B3 cannot end up running under different physics.
    """
    sim_cfg = cfg["simulation"]
    physx_cfg = sim_cfg.get("physx", {})

    stage = get_current_stage()
    scene = UsdPhysics.Scene.Define(stage, prim_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(abs(float(sim_cfg["gravity"])))

    physx = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(prim_path))

    # Pin the solver step. Left unset, a slow viewport frame can be
    # integrated as one large step, which destabilises the articulation.
    steps = int(physx_cfg.get("time_steps_per_second", 0) or
                round(1.0 / float(sim_cfg["physics_dt"])))
    physx.CreateTimeStepsPerSecondAttr().Set(steps)

    physx.CreateSolverTypeAttr().Set(str(physx_cfg.get("solver_type", "TGS")))
    physx.CreateEnableStabilizationAttr().Set(
        bool(physx_cfg.get("enable_stabilization", True)))
    physx.CreateEnableCCDAttr().Set(bool(physx_cfg.get("enable_ccd", False)))
    physx.CreateMinPositionIterationCountAttr().Set(
        int(physx_cfg.get("min_position_iterations", 8)))
    physx.CreateMinVelocityIterationCountAttr().Set(
        int(physx_cfg.get("min_velocity_iterations", 1)))

    return stage.GetPrimAtPath(prim_path)


def add_ground_plane(cfg: Dict[str, Any]) -> GroundPlane:
    """Add a collision ground plane. Without it the AMR falls forever."""
    ground_cfg = cfg["environment"]["ground"]
    return GroundPlane(
        prim_path=ground_cfg["prim_path"],
        size=float(ground_cfg["size"]),
        z_position=float(ground_cfg["z_position"]),
        color=np.array([0.35, 0.35, 0.38]),
    )


def add_lighting(cfg: Dict[str, Any]) -> None:
    """Add a dome light for ambient fill and a distant light for shape.

    Both are procedural UsdLux prims, so nothing is fetched from the
    Omniverse asset server and the scene builds offline.
    """
    stage = get_current_stage()
    light_cfg = cfg["environment"]["lighting"]

    dome = UsdLux.DomeLight.Define(stage, light_cfg["dome_prim_path"])
    dome.CreateIntensityAttr(float(light_cfg["dome_intensity"]))

    distant = UsdLux.DistantLight.Define(stage, light_cfg["distant_prim_path"])
    distant.CreateIntensityAttr(float(light_cfg["distant_intensity"]))
    distant.CreateAngleAttr(0.53)
    # Rotate the light down from the horizon by the configured elevation.
    elevation = float(light_cfg["distant_elevation_deg"])
    UsdGeom.Xformable(distant.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-(90.0 - elevation), 0.0, 0.0)
    )


def _set_pose(prim: Usd.Prim, position, yaw_deg: float) -> None:
    """Write translate and orient on a prim that already has those ops.

    The AMR USD authors xformOp:translate, xformOp:orient and
    xformOp:scale in that order. Reusing the existing ops keeps the
    xformOpOrder intact; adding new ones would silently stack.
    """
    xformable = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}

    missing = {"xformOp:translate", "xformOp:orient"} - set(ops)
    if missing:
        raise RuntimeError(
            f"{prim.GetPath()} is missing transform ops {sorted(missing)}; "
            f"found {sorted(ops)}. The AMR USD may have changed."
        )

    ops["xformOp:translate"].Set(Gf.Vec3d(*[float(v) for v in position]))

    half = math.radians(float(yaw_deg)) / 2.0
    ops["xformOp:orient"].Set(
        Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))
    )


def spawn_amr(cfg: Dict[str, Any], usd_path: str) -> Usd.Prim:
    """Reference the AMR USD into the current stage and pose it.

    The reference lands on `amr.reference_prim_path` (/World), which is
    the source USD's own default prim path. Keeping the path identical
    means the ActionGraph's absolute relationship targets resolve
    without USD having to remap them.
    """
    amr_cfg = cfg["amr"]
    add_reference_to_stage(usd_path=str(usd_path), prim_path=amr_cfg["reference_prim_path"])

    stage = get_current_stage()
    amr_prim = stage.GetPrimAtPath(amr_cfg["articulation_path"])
    if not amr_prim.IsValid():
        raise RuntimeError(
            f"expected {amr_cfg['articulation_path']} after referencing {usd_path}, "
            f"but the prim does not exist"
        )

    # x and y come from navigation.start, not from the amr section, so
    # the robot always begins where the trial definition says it does.
    spawn = amr_cfg["spawn"]
    start_x, start_y = cfg["navigation"]["start"]
    _set_pose(amr_prim, (start_x, start_y, spawn["z"]), spawn["yaw_deg"])
    return amr_prim


def stabilise_articulation(cfg: Dict[str, Any]) -> list[str]:
    """Clamp link velocities and stiffen the solver on the AMR.

    The source USD authors no mass, no inertia and no velocity limits.
    Its four caster wheels are small cylinders that spin freely, so PhysX
    gives them a tiny inertia and lets them reach its default velocity
    ceiling. Twice during development the articulation diverged to NaN:
    PhysX logged "Illegal BroadPhaseUpdateData", every wheel transform
    became nan, and the odometry reported the robot kilometres away. One
    of those took the simulator down with it.

    Everything here is a limit, not a change to the model: the AMR drives
    at 0.4 m/s with wheels turning near 3 rad/s, far below the clamps.
    See docs/AMR_DIGITAL_TWIN_REFERENCE.md section 4.6.
    """
    art_cfg = cfg["simulation"].get("articulation", {})
    if not art_cfg:
        return []

    stage = get_current_stage()
    root = stage.GetPrimAtPath(cfg["amr"]["articulation_path"])
    if not root.IsValid():
        return []

    max_ang = float(art_cfg.get("max_angular_velocity", 30.0))
    max_lin = float(art_cfg.get("max_linear_velocity", 10.0))
    damping = float(art_cfg.get("caster_joint_damping", 0.0))

    bodies = 0
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxAngularVelocityAttr().Set(max_ang)
        rb.CreateMaxLinearVelocityAttr().Set(max_lin)
        bodies += 1

    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateSolverPositionIterationCountAttr().Set(
        int(art_cfg.get("solver_position_iterations", 32)))
    art.CreateSolverVelocityIterationCountAttr().Set(
        int(art_cfg.get("solver_velocity_iterations", 4)))

    joints = 0
    if damping > 0.0:
        for prim in Usd.PrimRange(root):
            name = prim.GetName()
            # Wheel joints only. Damping the swivel joints stops the
            # casters from steering, which makes them skid and the rear
            # of the AMR hop. See configs/common.yaml.
            if "caster" not in name or "wheel_joint" not in name:
                continue
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr().Set("force")
            drive.CreateDampingAttr().Set(damping)
            drive.CreateStiffnessAttr().Set(0.0)
            drive.CreateTargetVelocityAttr().Set(0.0)
            joints += 1

    return [f"clamped {bodies} link(s) to {max_ang:.0f} rad/s and "
            f"{max_lin:.0f} m/s, damped {joints} caster joint(s), "
            f"solver iterations {art_cfg.get('solver_position_iterations', 32)}"]


def patch_odometry_graph(cfg: Dict[str, Any]) -> list[str]:
    """Repair two defects in the source USD's odometry publisher.

    The upstream ActionGraph publishes /odom with:

      1. chassisFrameId spelled "bsae_footprint", so the message names a
         child frame that exists in no TF tree.
      2. inputs:orientation left unconnected, so every message reports
         the identity quaternion no matter how the robot is turned.
         Observed directly: commanding 0.5 rad/s produced 0.000 rad of
         reported yaw while the robot physically rotated.

    Both are fixed on the composed stage. The source USD stays
    byte-identical on disk so the change is visible here in code rather
    than buried in a binary asset.
    See docs/AMR_DIGITAL_TWIN_REFERENCE.md section 4.1.

    Returns a list of human-readable descriptions of what was changed.
    """
    stage = get_current_stage()
    graph_root = f"{cfg['amr']['reference_prim_path']}/AMR_Controller"
    pub = stage.GetPrimAtPath(f"{graph_root}/ros2_publish_odometry")
    src = stage.GetPrimAtPath(f"{graph_root}/isaac_compute_odometry_node")

    changes: list[str] = []
    if not pub.IsValid():
        return changes

    # 1. chassis frame id typo
    frame_attr = pub.GetAttribute("inputs:chassisFrameId")
    expected_frame = cfg["ros"]["frames"]["base"]
    if frame_attr and frame_attr.Get() != expected_frame:
        changes.append(
            f"odometry chassisFrameId {frame_attr.Get()!r} -> {expected_frame!r}"
        )
        frame_attr.Set(expected_frame)

    # 2. missing orientation connection
    orient_attr = pub.GetAttribute("inputs:orientation")
    if orient_attr and src.IsValid() and not orient_attr.GetConnections():
        orient_attr.AddConnection(src.GetPath().AppendProperty("outputs:orientation"))
        changes.append("connected odometry orientation from IsaacComputeOdometry")

    # 3. odomFrameId is left empty upstream; make it explicit so the
    #    message and the TF tree cannot disagree.
    odom_attr = pub.GetAttribute("inputs:odomFrameId")
    expected_odom = cfg["ros"]["frames"]["odom"]
    if odom_attr and not odom_attr.Get():
        odom_attr.Set(expected_odom)
        changes.append(f"set odometry odomFrameId -> {expected_odom!r}")

    return changes


def patch_lidar_graph(cfg: Dict[str, Any]) -> list[str]:
    """Make the RTX LiDAR publish complete sweeps instead of slices.

    ROS2RtxLidarHelper defaults inputs:fullScan to False, which publishes
    whatever the sensor swept during the last rendered frame. A rotary
    LiDAR turning at 10 Hz rendered at 60 Hz therefore emits about a
    sixth of a revolution per message.

    pointcloud_to_laserscan turns each of those slices into a full
    LaserScan whose other five sixths are infinity, and Nav2's obstacle
    layer treats an infinite return as "nothing there all the way to
    max range" and raytrace-clears that whole ray. The costmap ends up
    erasing obstacles faster than it marks them.

    Measured before this fix, with a person standing 1.34 m directly in
    front of the robot: 723 beams, 133 finite, all between 2.40 m and
    2.95 m. The person was absent from the global costmap entirely, and
    Nav2 planned straight through them.

    See docs/NAV2.md section 7.
    """
    stage = get_current_stage()
    node_path = (f"{cfg['amr']['reference_prim_path']}"
                 "/AMR_Controller/ros2_rtx_lidar_helper")
    prim = stage.GetPrimAtPath(node_path)
    if not prim.IsValid():
        return []

    attr = prim.GetAttribute("inputs:fullScan")
    if not attr:
        attr = prim.CreateAttribute("inputs:fullScan", Sdf.ValueTypeNames.Bool, True)
    if attr.Get() is True:
        return []
    attr.Set(True)
    return ["RTX LiDAR fullScan enabled (was publishing partial sweeps)"]


def patch_lidar_sensor(cfg: Dict[str, Any]) -> list[str]:
    """Set the LiDAR's minimum measuring distance and detection threshold.

    The AMR USD authors omni:sensor:Core:nearRangeM as 1.0 m, so the
    sensor reports nothing closer than a metre. Measured with a person
    walking into the robot: 3 returns at 1.10 m, none at 0.69 m, and the
    person vanished from the costmap for the last stretch before contact,
    which is exactly where the robot needs to see them. Real scanning
    LiDARs start at 0.1-0.5 m, so the asset's value is the outlier.

    The asset itself is left alone; the change is made on the stage after
    it is referenced, like the graph patches above.
    """
    scan = cfg["ros"]["scan"]
    wanted = {
        "omni:sensor:Core:nearRangeM": ("minimum range", scan.get("sensor_near_range"), "m"),
        "omni:sensor:Core:minReflectance": ("minimum reflectance",
                                            scan.get("sensor_min_reflectance"), ""),
    }
    if all(v[1] is None for v in wanted.values()):
        return []
    stage = get_current_stage()
    path = f"{cfg['amr']['chassis_path']}/{cfg['ros']['frames']['lidar']}"
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return [f"no LiDAR prim at {path}; sensor left as authored"]

    notes = []
    for name, (label, value, unit) in wanted.items():
        if value is None:
            continue
        attr = prim.GetAttribute(name)
        if not attr:
            notes.append(f"{path} has no {name.split(':')[-1]}; left as authored")
            continue
        was = attr.Get()
        if was is not None and abs(float(was) - float(value)) < 1e-9:
            continue
        attr.Set(float(value))
        suffix = f" {unit}" if unit else ""
        notes.append(f"LiDAR {label} {float(was):.3g}{suffix} -> {float(value):.3g}{suffix}")
    return notes


def build_scene(cfg: Dict[str, Any], usd_path: str, verbose: bool = True):
    """Build the complete STEP 3/4 scene: physics, ground, light, AMR.

    Every Isaac Sim entry point goes through here so the scene the
    checks run against and the scene the experiments run against are
    the same one. Returns the AMR articulation prim.
    """
    def say(msg: str) -> None:
        if verbose:
            print(f"  {msg}", flush=True)

    setup_physics_scene(cfg)
    add_ground_plane(cfg)
    add_lighting(cfg)
    px = cfg["simulation"].get("physx", {})
    say(f"physics scene ({px.get('solver_type', 'TGS')}, "
        f"{round(1.0 / float(cfg['simulation']['physics_dt']))} Hz, "
        f"CCD {'on' if px.get('enable_ccd') else 'off'}), "
        f"ground plane and lighting created")

    problems = validate(cfg)
    viz_problem = comfort_viz.check_below_scan(cfg)
    if viz_problem:
        problems = list(problems) + [viz_problem]
    if problems:
        # A human inside a wall or a goal outside the corridor produces a
        # trial that fails for the wrong reason, which is worse than not
        # running at all.
        raise ValueError(
            "layout problems in the config:\n  - " + "\n  - ".join(problems))

    counts = corridor.build_corridor(cfg)
    if any(counts.values()):
        say(f"corridor built: {counts['walls']} walls, "
            f"{counts['stations']} stations, {counts['humans']} humans")
        report = passage_report(cfg)
        say(f"tightest cross-section: {report.clearance:.2f} m at "
            f"x={report.tightest_x:+.2f}, AMR needs {report.required:.2f} m "
            f"-> {'PASSABLE' if report.passable else 'BLOCKED'}")

    for note in people.populate(cfg):
        say(note)

    amr_prim = spawn_amr(cfg, usd_path)
    start_x, start_y = cfg["navigation"]["start"]
    say(f"AMR referenced at {cfg['amr']['reference_prim_path']}, "
        f"spawned at start ({start_x}, {start_y})")

    for change in (patch_odometry_graph(cfg) + patch_lidar_graph(cfg)
                   + patch_lidar_sensor(cfg) + stabilise_articulation(cfg)):
        say(f"patched: {change}")

    for note in human_tf.publish_human_frames(cfg):
        say(note)

    walkers = human_motion.WalkerSync(cfg)
    for note in walkers.describe():
        say(note)

    draw_comfort_spaces(cfg, say)

    return amr_prim, walkers


def draw_comfort_spaces(cfg: Dict[str, Any], say=None) -> List:
    """Work out each human's Comfort Space and draw it.

    Returns the shapes, so the caller can report them. Drawing is visual
    only; what the planner avoids is written into the costmap separately.
    """
    def emit(msg: str) -> None:
        if say:
            say(msg)

    if not cfg["corridor"].get("enabled", True):
        return []

    provider = make_state_provider(cfg)
    states = provider.states()
    policy = make_policy(cfg)
    # Nothing is moving yet at build time, so every Comfort Space starts
    # at its at-rest size. A walker's grows once they set off, and
    # isaac_sim/utils/human_motion.py keeps the drawing in step.
    shapes = build_shapes(cfg, policy.axes(states), grow=robot_grow(cfg))

    emit(f"comfort space: policy={policy.name}, shape="
         f"{cfg['comfort_space'].get('shape', 'circle')}, "
         f"states from {provider.source}")
    for line in comfort_viz.summary(shapes, states).splitlines():
        emit(f"  {line}")

    report = passage_report(cfg, shapes=shapes)
    emit(f"  at rest: {report.clearance:.2f} m at "
         f"x={report.tightest_x:+.2f}, AMR needs {report.required:.2f} m "
         f"-> {'PASSABLE' if report.passable else 'BLOCKED'}")

    drawn = comfort_viz.draw(get_current_stage(), cfg, shapes)
    if drawn:
        emit(f"  drew {drawn} Comfort Space")

    return shapes
