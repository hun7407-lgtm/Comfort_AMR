#!/usr/bin/env python3
"""Do the fleet AMRs each have a working Nav2 of their own?

    ./scripts/run_sim.sh --headless --config configs/fleet_nav2.yaml --scenario b2 &
    ./scripts/run_nav2.sh --config configs/fleet_nav2.yaml --scenario b2 &
    ./scripts/run_fleet_nav2.sh --config configs/fleet_nav2.yaml --scenario b2 &
    python3 scripts/check_fleet_stacks.py

Stage 3 of docs/NAV2.md section 5c. check_fleet_nav2.py proved the
vehicles are separate robots; this proves each one's navigation stack
is separate too, and that the three stacks have not bled into each
other's topics.

    1. every stack reports itself active
    2. map -> aN/base_footprint resolves: the static map -> aN/odom
       joins the simulator's prefixed chain in the one global tree
    3. every robot has its own NavigateToPose server and its own /scan
    4. a goal to a1 moves a1, and nobody else
    5. no fleet node publishes on AMR1's /cmd_vel

Point 4 is again the one that matters: everything else can look right
while two controller_servers argue over one set of wheels.

Exit code 0 means every check passed.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

FAILS: list[str] = []
PASSES = 0


def check(label: str, ok: bool, detail: str = "") -> None:
    global PASSES
    if ok:
        PASSES += 1
        print(f"  ok    {label}{'  ' + detail if detail else ''}")
    else:
        FAILS.append(f"{label}  {detail}")
        print(f"  FAIL  {label}{'  ' + detail if detail else ''}")


def ns_of(robot: str) -> str:
    return f"/{robot}" if robot else ""


class Watcher(Node):
    def __init__(self, robots: list[str]):
        super().__init__("check_fleet_stacks")
        self.robots = robots
        self.odom: dict[str, Odometry] = {}
        self.scans: dict[str, int] = {r: 0 for r in robots}
        for r in robots:
            self.create_subscription(
                Odometry, f"{ns_of(r)}/odom",
                lambda m, r=r: self.odom.__setitem__(r, m), 10)
            # pointcloud_to_laserscan publishes best-effort; a reliable
            # subscription is silently never matched.
            self.create_subscription(
                LaserScan, f"{ns_of(r)}/scan",
                lambda m, r=r: self.scans.__setitem__(r, self.scans[r] + 1),
                qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=False)

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def xy(self, robot: str):
        m = self.odom.get(robot)
        if m is None:
            return None
        p = m.pose.pose.position
        return (p.x, p.y)

    def is_active(self, robot: str, timeout: float) -> bool:
        cli = self.create_client(
            Trigger, f"{ns_of(robot)}/lifecycle_manager_navigation/is_active")
        if not cli.wait_for_service(timeout_sec=timeout):
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return bool(fut.result() and fut.result().success)

    def node_state(self, robot: str, node: str) -> str:
        cli = self.create_client(GetState, f"{ns_of(robot)}/{node}/get_state")
        if not cli.wait_for_service(timeout_sec=2.0):
            return "no service"
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result().current_state.label if fut.result() else "no reply"

    def navigate(self, robot: str, x: float, y: float, yaw: float,
                 timeout: float) -> str:
        client = ActionClient(self, NavigateToPose,
                              f"{ns_of(robot)}/navigate_to_pose")
        if not client.wait_for_server(timeout_sec=10.0):
            return "no server"
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return "rejected"
        res = handle.get_result_async()
        end = time.monotonic() + timeout
        while not res.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not res.done():
            handle.cancel_goal_async()
            return "timed out"
        status = res.result().status
        return {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled"}.get(status, str(status))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", nargs="+", default=["a1", "a2"])
    ap.add_argument("--wait", type=float, default=120.0,
                    help="seconds to allow the stacks to activate")
    ap.add_argument("--goal", nargs=3, type=float, default=[-10.5, 2.0, 1.5708],
                    metavar=("X", "Y", "YAW"),
                    help="map-frame goal for the first robot: 2 m up its "
                         "own lane by default")
    ap.add_argument("--goal-timeout", type=float, default=60.0)
    args = ap.parse_args()
    robots = list(args.robots)
    everyone = [""] + robots          # "" is AMR1

    rclpy.init()
    node = Watcher(everyone)
    try:
        print(f"[1] waiting up to {args.wait:.0f} s for the stacks to activate")
        deadline = time.monotonic() + args.wait
        active = {r: False for r in everyone}
        while time.monotonic() < deadline and not all(active.values()):
            for r in everyone:
                if not active[r]:
                    active[r] = node.is_active(r, timeout=3.0)
            node.spin(1.0)
        for r in everyone:
            label = r or "AMR1"
            detail = "" if active[r] else "  ".join(
                f"{n}={node.node_state(r, n)}" for n in
                ("map_server", "controller_server", "bt_navigator"))
            check(f"{label} lifecycle active", active[r], detail)

        print("\n[2] map -> aN/base_footprint resolves in the global tree")
        node.spin(3.0)
        for r in robots:
            base = f"{r}/base_footprint"
            ok = node.tf.can_transform("map", base, rclpy.time.Time())
            detail = ""
            if ok:
                t = node.tf.lookup_transform("map", base, rclpy.time.Time())
                detail = (f"({t.transform.translation.x:+.2f}, "
                          f"{t.transform.translation.y:+.2f})")
            check(f"map -> {base}", ok, detail)

        print("\n[3] each robot has its own action server and scan")
        for r in everyone:
            label = r or "AMR1"
            srv = any(n for n, t in node.get_service_names_and_types()
                      if n == f"{ns_of(r)}/navigate_to_pose/_action/send_goal")
            check(f"{label} navigate_to_pose server", srv)
        node.spin(3.0)
        for r in everyone:
            label = r or "AMR1"
            check(f"{label} scan receiving", node.scans[r] > 0,
                  f"{node.scans[r]} message(s)")

        print("\n[4] a goal to the first robot moves that robot only")
        first = robots[0]
        before = {r: node.xy(r) for r in everyone}
        check("odometry from everyone before the goal",
              all(v is not None for v in before.values()))
        gx, gy, gyaw = args.goal
        outcome = node.navigate(first, gx, gy, gyaw, args.goal_timeout)
        node.spin(1.0)
        after = {r: node.xy(r) for r in everyone}
        for r in everyone:
            if before[r] is None or after[r] is None:
                continue
            moved = math.dist(before[r], after[r])
            label = r or "AMR1"
            if r == first:
                check(f"{label} drove to the goal", moved > 0.5 and
                      outcome == "succeeded", f"{moved:.3f} m, {outcome}")
            else:
                check(f"{label} stayed put", moved < 0.05, f"{moved:.3f} m")

        print("\n[5] nothing from the fleet reaches AMR1")
        pubs = [i.node_namespace.rstrip("/") + "/" + i.node_name
                for i in node.get_publishers_info_by_topic("/cmd_vel")]
        strays = [p for p in pubs if any(p.startswith(f"/{r}/") for r in robots)]
        check("/cmd_vel has no fleet publisher", not strays, ", ".join(pubs))
        for r in robots:
            own = [i.node_namespace.rstrip("/") + "/" + i.node_name
                   for i in node.get_publishers_info_by_topic(f"/{r}/cmd_vel")]
            check(f"/{r}/cmd_vel published by its own collision monitor",
                  any(p == f"/{r}/collision_monitor" for p in own),
                  ", ".join(own))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{PASSES} passed, {len(FAILS)} failed")
    for line in FAILS:
        print(f"  FAILED  {line}")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
