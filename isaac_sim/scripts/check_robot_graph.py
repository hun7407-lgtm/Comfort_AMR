#!/usr/bin/env python3
"""Does one AMR asset, referenced three times, give three ROS 2 robots?

    ~/isaacsim/python.sh isaac_sim/scripts/check_robot_graph.py

Checks isaac_sim/utils/robot_graph.py on the composed stage, with no
physics and no Nav2: references the asset once bare and twice under a
namespace, then reads back what the ActionGraphs would publish.

What it is looking for is the three ways this silently fails. A missed
`nodeNamespace` puts two robots on one topic. A topic name left
absolute ignores the namespace that was set. A frame id left unprefixed
makes tf take whichever `base_link` arrived last.

Exit code 0 means every check passed.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

from isaacsim.core.utils.prims import create_prim  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage, get_current_stage  # noqa: E402
from pxr import Usd  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from isaac_sim.utils import robot_graph  # noqa: E402

FAILS: list[str] = []
PASSES = 0
# Isaac Sim replaces stdout, and plain prints from here do not survive
# into a redirected log. Everything also goes to a file.
REPORT: list[str] = []
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else None


def say(line: str = "") -> None:
    REPORT.append(line)
    print(line, flush=True)


def check(label: str, got, want) -> None:
    global PASSES
    if got == want:
        PASSES += 1
        say(f"  ok    {label}: {got!r}")
    else:
        FAILS.append(f"{label}: got {got!r}, want {want!r}")
        say(f"  FAIL  {label}: got {got!r}, want {want!r}")


def attr(prim_path: str, name: str):
    a = get_current_stage().GetPrimAtPath(prim_path).GetAttribute(name)
    return a.Get() if a else None


def main() -> int:
    cfg = load_config()
    frames = cfg["ros"]["frames"]
    usd = str(resolve_path(cfg["amr"]["usd_path"]))

    create_new_stage()
    # AMR1, referenced the way scene_setup does it.
    create_prim("/World", "Xform", usd_path=usd)
    # Two more, each under its own root, the way fleet.py does it.
    for name in ("a1", "a2"):
        create_prim(f"/World/Fleet/{name}/model", "Xform", usd_path=usd)

    say("\n[1] namespacing the two fleet instances")
    for name in ("a1", "a2"):
        root = f"/World/Fleet/{name}/model/AMR_Controller"
        for line in robot_graph.namespace_graph(cfg, root, name):
            say(f"       {name}: {line}")

    say("\n[2] fleet instance a1 is on its own topics")
    g = "/World/Fleet/a1/model/AMR_Controller"
    check("a1 twist nodeNamespace", attr(f"{g}/ros2_subscribe_twist",
                                        "inputs:nodeNamespace"), "a1")
    check("a1 twist topicName", attr(f"{g}/ros2_subscribe_twist",
                                     "inputs:topicName"), "cmd_vel")
    check("a1 odom topicName", attr(f"{g}/ros2_publish_odometry",
                                    "inputs:topicName"), "odom")
    check("a1 lidar topicName", attr(f"{g}/ros2_rtx_lidar_helper",
                                     "inputs:topicName"), "lidar/cloud")

    say("\n[3] a1's tf chain is prefixed end to end")
    check("odom -> base  parent", attr(f"{g}/ros2_publish_raw_transform_tree",
                                       "inputs:parentFrameId"),
          f"a1/{frames['odom']}")
    check("odom -> base  child", attr(f"{g}/ros2_publish_raw_transform_tree",
                                      "inputs:childFrameId"),
          f"a1/{frames['base']}")
    check("base -> chassis parent",
          attr(f"{g}/ros2_publish_raw_transform_tree_01", "inputs:parentFrameId"),
          f"a1/{frames['base']}")
    check("base -> chassis child",
          attr(f"{g}/ros2_publish_raw_transform_tree_01", "inputs:childFrameId"),
          f"a1/{frames['chassis']}")
    check("chassis -> lidar parent",
          attr(f"{g}/ros2_publish_raw_transform_tree_02", "inputs:parentFrameId"),
          f"a1/{frames['chassis']}")
    check("chassis -> lidar child",
          attr(f"{g}/ros2_publish_raw_transform_tree_02", "inputs:childFrameId"),
          f"a1/{frames['lidar']}")
    check("lidar frameId", attr(f"{g}/ros2_rtx_lidar_helper", "inputs:frameId"),
          f"a1/{frames['lidar']}")
    check("odometry odomFrameId", attr(f"{g}/ros2_publish_odometry",
                                       "inputs:odomFrameId"), f"a1/{frames['odom']}")
    check("odometry chassisFrameId", attr(f"{g}/ros2_publish_odometry",
                                          "inputs:chassisFrameId"),
          f"a1/{frames['base']}")

    say("\n[4] the colliding tf tree publisher is off")
    tree = get_current_stage().GetPrimAtPath(f"{g}/{robot_graph.TREE_NODE}")
    check("a1 transform_tree active", tree.IsActive(), False)

    say("\n[3b] tf stays on one global tree")
    for node in robot_graph.TF_NODES:
        check(f"a1 {node} nodeNamespace",
              attr(f"{g}/{node}", "inputs:nodeNamespace"), None)
        check(f"a1 {node} topicName",
              attr(f"{g}/{node}", "inputs:topicName"), None)

    say("\n[4b] the clock is published once, by AMR1")
    check("a1 clock active", get_current_stage().GetPrimAtPath(
        f"{g}/{robot_graph.CLOCK_NODE}").IsActive(), False)
    check("AMR1 clock active", get_current_stage().GetPrimAtPath(
        f"/World/AMR_Controller/{robot_graph.CLOCK_NODE}").IsActive(), True)

    say("\n[5] a1 and a2 do not share a topic")
    g2 = "/World/Fleet/a2/model/AMR_Controller"
    check("a2 twist nodeNamespace", attr(f"{g2}/ros2_subscribe_twist",
                                         "inputs:nodeNamespace"), "a2")
    check("a2 base frame", attr(f"{g2}/ros2_publish_raw_transform_tree",
                                "inputs:childFrameId"), f"a2/{frames['base']}")

    say("\n[6] AMR1 is untouched")
    m = "/World/AMR_Controller"
    check("AMR1 twist nodeNamespace", attr(f"{m}/ros2_subscribe_twist",
                                           "inputs:nodeNamespace"), None)
    check("AMR1 twist topicName", attr(f"{m}/ros2_subscribe_twist",
                                       "inputs:topicName"), "/cmd_vel")
    check("AMR1 transform_tree active",
          get_current_stage().GetPrimAtPath(
              f"{m}/{robot_graph.TREE_NODE}").IsActive(), True)

    say("\n[7] USD rebased each instance's own prims")
    rel = get_current_stage().GetPrimAtPath(
        f"{g}/articulation_controller").GetRelationship("inputs:targetPrim")
    check("a1 articulation target", [str(t) for t in rel.GetTargets()],
          ["/World/Fleet/a1/model/AMR"])

    say(f"\n{PASSES} passed, {len(FAILS)} failed")
    for line in FAILS:
        say(f"  FAILED  {line}")
    if OUT is not None:
        OUT.write_text("\n".join(REPORT) + "\n", encoding="utf-8")
    return 1 if FAILS else 0


code = main()
app.close()
sys.exit(code)
