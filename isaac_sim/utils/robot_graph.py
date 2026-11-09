"""Give one AMR instance's ActionGraph its own ROS 2 namespace.

The study used to run one Nav2 stack. The asset carries an ActionGraph
that subscribes to `/cmd_vel` and publishes `/odom`, `/joint_states`
and the LiDAR cloud on absolute topic names, so a second copy of the
asset drove itself from the first one's commands. `fleet.py` worked
around that by switching the graph off and moving those vehicles by
writing their pose every tick.

Every ROS 2 node in the graph already has an `inputs:nodeNamespace`,
left empty upstream. Setting it per instance is what separates the
topics, so the workaround is not needed once this runs:

    /cmd_vel   ->  /a1/cmd_vel, /a2/cmd_vel
    /odom      ->  /a1/odom,    /a2/odom

Two details decide whether that actually works.

**Absolute topic names ignore the namespace.** `inputs:topicName` is
`/cmd_vel` with a leading slash, which ROS 2 reads as fully qualified.
The name has to be made relative or the namespace is silently dropped
and every instance lands back on the same topic.

**The transform tree node has no frame prefix.** ROS2PublishTransformTree
names frames after the prims it is pointed at, and a referenced copy of
the asset keeps the prim names `base_link`, `lidar_link` and so on. Three
instances would publish three different poses for one frame name and tf
would take whichever arrived last. It is switched off here. The three
ROS2PublishRawTransformTree nodes carry the same chain as explicit
strings -- odom -> base_footprint -> base_link -> lidar_link -- which is
every frame Nav2 needs, and those strings can be prefixed.

The source USD stays byte-identical on disk. Like the fixes in
scene_setup.py, the change is made on the composed stage so it is
visible in code rather than buried in a binary asset.
"""

from __future__ import annotations

from typing import Any, Dict, List

from isaacsim.core.utils.stage import get_current_stage
from pxr import Sdf, Usd

# Only the bridge's own nodes carry a namespace; the maths nodes do not.
ROS2_NODE_TYPE_PREFIX = "isaacsim.ros2.bridge."

# Which frame each node names, by the role it plays in the chain. The
# node names come from the asset and are already hardcoded elsewhere in
# this package (scene_setup.patch_odometry_graph, patch_lidar_graph).
FRAME_ROLES: Dict[str, Dict[str, str]] = {
    "ros2_publish_odometry": {"odomFrameId": "odom", "chassisFrameId": "base"},
    "ros2_publish_raw_transform_tree": {"parentFrameId": "odom",
                                        "childFrameId": "base"},
    "ros2_publish_raw_transform_tree_01": {"parentFrameId": "base",
                                           "childFrameId": "chassis"},
    "ros2_publish_raw_transform_tree_02": {"parentFrameId": "chassis",
                                           "childFrameId": "lidar"},
    "ros2_rtx_lidar_helper": {"frameId": "lidar"},
}

# Publishes the whole articulation under prim names that every instance
# shares. Nothing Nav2 reads comes from it.
TREE_NODE = "ros2_publish_transform_tree"

# One simulation has one clock. Namespaced it would become /a1/clock,
# which nothing subscribes to, and left alone it would be a third
# publisher racing the other two on /clock. AMR1 keeps
# publishing it for everybody.
CLOCK_NODE = "ros2_publish_clock"

# tf is the one topic that must NOT be namespaced. These nodes keep an
# empty nodeNamespace and publish on /tf like everybody else; only
# their frame ids are prefixed.
#
# The namespace cannot be escaped from the topic name. The bridge
# strips a leading slash and prepends the namespace again, so
# `topicName = "/tf"` on a node whose namespace is "a1" still lands on
# /a1/tf. Measured both ways: with the namespace set, /a1/tf and /a2/tf
# existed, /tf carried no `a1/` frame at all, and every can_transform
# on a1/odom -> a1/base_footprint was false.
#
# The alternative is Nav2's usual multi-robot arrangement, one tf tree
# per namespace. One global tree with prefixed frames is chosen here
# because the trial recorder, the costmap recorder and the metrics all
# read a single buffer, and fleet vehicles are measured too. Each
# namespaced Nav2 stack is then remapped back onto /tf.
TF_NODES = ("ros2_publish_raw_transform_tree",
            "ros2_publish_raw_transform_tree_01",
            "ros2_publish_raw_transform_tree_02")


def prefixed(namespace: str, frame: str) -> str:
    """tf2's multi-robot convention: `a1/base_link`, no leading slash."""
    return f"{namespace}/{frame}" if namespace else frame


def _node_type(prim: Usd.Prim) -> str:
    attr = prim.GetAttribute("node:type")
    return str(attr.Get()) if attr and attr.Get() else ""


def _set(prim: Usd.Prim, name: str, value: str,
         type_name=Sdf.ValueTypeNames.String) -> bool:
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, True)
    if attr.Get() == value:
        return False
    attr.Set(value)
    return True


def instance_fixes(cfg: Dict[str, Any], graph_root: str) -> List[str]:
    """The fixes scene_setup makes to AMR1, for a copy.

    scene_setup.patch_odometry_graph and patch_lidar_graph repair two
    faults in the asset, and they only ever ran on the one instance the
    study measured. A second instance carries the same faults:

    * the odometry publisher's `inputs:orientation` is not connected, so
      it reports the identity quaternion however the robot is turned.
      Nav2 cannot localise against that.
    * the RTX LiDAR publishes the slice swept since the last render, not
      a full revolution, and Nav2's obstacle layer raytrace-clears the
      missing five sixths. See section 7 of docs/NAV2.md.

    Both are repaired on the composed stage, per instance.
    """
    stage = get_current_stage()
    changes: List[str] = []

    pub = stage.GetPrimAtPath(f"{graph_root}/ros2_publish_odometry")
    src = stage.GetPrimAtPath(f"{graph_root}/isaac_compute_odometry_node")
    if pub.IsValid() and src.IsValid():
        orient = pub.GetAttribute("inputs:orientation")
        if orient and not orient.GetConnections():
            orient.AddConnection(src.GetPath().AppendProperty("outputs:orientation"))
            changes.append("connected odometry orientation")

    lidar = stage.GetPrimAtPath(f"{graph_root}/ros2_rtx_lidar_helper")
    if lidar.IsValid():
        attr = lidar.GetAttribute("inputs:fullScan")
        if not attr:
            attr = lidar.CreateAttribute("inputs:fullScan",
                                         Sdf.ValueTypeNames.Bool, True)
        if attr.Get() is not True:
            attr.Set(True)
            changes.append("RTX LiDAR fullScan on")

    return changes


def namespace_graph(cfg: Dict[str, Any], graph_root: str,
                    namespace: str) -> List[str]:
    """Namespace one instance of the AMR ActionGraph.

    `graph_root` is that instance's AMR_Controller prim. `namespace` is
    the robot's name, or "" to leave the graph on the bare topics --
    which is what AMR1 keeps, so existing recordings,
    launch files and scripts go on reading `/scan` and `/odom`.

    Returns a description of every change, for the scene report.
    """
    stage = get_current_stage()
    root = stage.GetPrimAtPath(graph_root)
    if not root.IsValid():
        return [f"no ActionGraph at {graph_root}"]

    frames = cfg["ros"]["frames"]
    changes: List[str] = []

    for prim in Usd.PrimRange(root):
        if not _node_type(prim).startswith(ROS2_NODE_TYPE_PREFIX):
            continue
        name = prim.GetName()

        if name == CLOCK_NODE:
            if namespace and prim.IsActive():
                prim.SetActive(False)
                changes.append(f"{name}: off (one clock, published by the "
                               f"AMR1)")
            continue

        if name == TREE_NODE:
            # Switching the node off is what keeps the frame names from
            # colliding; see the module docstring.
            if namespace and prim.IsActive():
                prim.SetActive(False)
                changes.append(f"{name}: off (frame names are not per-instance)")
            continue

        # Only the bridge nodes that actually declare a namespace get
        # one; creating the attribute on a node without it, such as
        # ros2_context, just leaves a property nothing reads.
        if (namespace and name not in TF_NODES
                and prim.GetAttribute("inputs:nodeNamespace")
                and _set(prim, "inputs:nodeNamespace", namespace)):
            changes.append(f"{name}: nodeNamespace -> {namespace!r}")

        if name in TF_NODES:
            if namespace:
                changes.append(f"{name}: left on /tf, frames prefixed")
        else:
            # An absolute topic name would ignore the namespace
            # entirely. Only rewritten for a namespaced instance: the
            # AMR1 keeps the attribute it has always had,
            # so nothing about the existing measurement path changes.
            topic = prim.GetAttribute("inputs:topicName")
            if (namespace and topic and isinstance(topic.Get(), str)
                    and topic.Get().startswith("/")):
                relative = topic.Get().lstrip("/")
                topic.Set(relative)
                changes.append(
                    f"{name}: topicName -> {relative!r} (was absolute)")

        for attr_name, role in FRAME_ROLES.get(name, {}).items():
            want = prefixed(namespace, frames[role])
            if _set(prim, f"inputs:{attr_name}", want):
                changes.append(f"{name}: {attr_name} -> {want!r}")

    return changes
