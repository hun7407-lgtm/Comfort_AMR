"""Publish each human's pose on /tf so the ROS 2 side can track them.

Isaac Sim only: imports omni and pxr.

Why this exists
---------------
Up to now every consumer of a human's position read it out of
configs/common.yaml, which is correct only while the humans stand still.
A walking human breaks that: the Comfort Space has to move with the
person, the passage calculation becomes a function of time, and the
yield policy has to compare the robot against where the person is now,
not where they started.

Publishing the poses on /tf keeps one source of truth in the simulator
and lets the ROS 2 nodes ask for it. It is also the shape the eventual
sensor-based pipeline takes: a perception node would publish the same
frames, and nothing downstream would change.

A node is added to the AMR's existing ActionGraph rather than a new
graph of its own, so it shares the ROS context and the playback tick
that are already wired up and correct.
"""

from __future__ import annotations

from typing import Any, Dict, List

import omni.graph.core as og
from isaacsim.core.utils.stage import get_current_stage
from pxr import Sdf

from comfort_core.layout import human_positions

NODE_NAME = "ros2_publish_human_tf"

# Frame the poses are published relative to. Leaving parentPrim empty
# makes the bridge use the stage's world frame and name it "world".
WORLD_FRAME = "world"


GRAPH_PATH = "/Environment/HumanTFGraph"


def publish_human_frames(cfg: Dict[str, Any]) -> List[str]:
    """Add a TransformTree publisher for the human colliders.

    Built as its own small graph rather than as an extra node inside the
    AMR's ActionGraph. That graph arrives by USD reference, and
    og.get_graph_by_path could not resolve it at scene build time, so
    editing it was not an option; passing its path to Controller.edit
    instead tried to create a second graph over the top of it and failed
    with "A prim already exists at this path".

    The colliders are the right thing to publish, not the character
    meshes: the collider is what the config sizes, what the geometry
    checks reason about, and what stays in step with the person whether
    or not a character asset is loaded.
    """
    stage = get_current_stage()

    targets = []
    for key, _x, _y in human_positions(cfg):
        path = f"/Environment/Corridor/human_{key}"
        if stage.GetPrimAtPath(path).IsValid():
            targets.append(Sdf.Path(path))
    if not targets:
        return ["no human colliders found; not publishing human frames"]

    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        return []

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("human_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "human_tf.inputs:execIn"),
                ("context.outputs:context", "human_tf.inputs:context"),
                ("sim_time.outputs:simulationTime", "human_tf.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("human_tf.inputs:topicName", "/tf"),
            ],
        },
    )

    # parentPrim is left empty on purpose: the bridge then publishes each
    # target relative to the stage's world frame.
    node_prim = stage.GetPrimAtPath(f"{GRAPH_PATH}/human_tf")
    rel = node_prim.GetRelationship("inputs:targetPrims")
    if not rel:
        rel = node_prim.CreateRelationship("inputs:targetPrims")
    rel.SetTargets(targets)

    names = [str(t).rsplit("/", 1)[-1] for t in targets]
    return [f"publishing {len(targets)} human frame(s) on /tf under "
            f"'{WORLD_FRAME}': {', '.join(names)}"]
