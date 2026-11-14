#!/usr/bin/env python3
"""Stage 2: one passing of the real AMR past a participant, recorded.

    ./scripts/stage2_trial.sh --scene configs/stage2/scenes/<corridor>.yaml \
        --scenario configs/stage2/conditions/b3o_r1.00.yaml \
        --session 261201_pilot --participant P01 --order 3 \
        --context c2_stationary_rear --distance 0.75
    ./scripts/stage2_trial.sh --scene ... --session ... --participant P01 --order 3 \
        --plan configs/stage2/conditions/orders.csv      context/distance/condition/repeat
                                                         read from the generated plan
    ./scripts/stage2_trial.sh ... --check          validate the config, drive nothing
    ./scripts/stage2_trial.sh ... --no-questionnaire

Runs in the SYSTEM Python with ROS 2, on (or beside) the real robot,
with Nav2, scripts/comfort_publisher.py and scripts/comfort_gate.py
already up for the same --scene / --scenario (the usual pipeline:
comfort_publisher paints the B3-O Comfort Space into the costmap,
comfort_gate yields, Nav2 drives). This node only sends the goal and
watches:

    Real AMR pose (tf map -> base)      Human pose / state / orientation
             |                                     |
             +--------- B3-O Dynamic Comfort Space (comfort_publisher) ---> costmap -> Nav2 -> AMR
             |
             v
    this recorder: trajectory, travel time/distance, planned path length,
    minimum human distance, applied Comfort Space extents and intrusion,
    yield seconds, safety stop/slowdown seconds, Nav2 status
             |
             v
    results/stage2/<session>/trials.csv, trajectories/<trial_id>.csv, <trial_id>.json
    then scripts/stage2_questionnaire.py for the participant's rating

Trial status (comfort_core.stage2.TRIAL_STATUSES): NORMAL on success,
NAVIGATION_FAILURE on a Nav2 abort/timeout, INVALID_TRIAL when the robot
never drove. Ctrl+C during the passing cancels the goal and asks the
operator which status applies (OPERATOR_ABORT / PARTICIPANT_STOP / E_STOP /
INVALID_TRIAL) and why; with stage2.safety.estop_topic set, a pressed
e-stop ends the trial as E_STOP by itself. Only NORMAL trials go on to
the questionnaire; a participant's stop is stored as its own status, not
as a failed comfort trial. Later corrections: scripts/stage2_mark_trial.py.

Primary distance: person CENTRE -> nearest AMR surface (min footprint
clearance + humans.radius), the Stage 1 x axis; the AMR centre -> person
centre distance is stored beside it as a raw field.

It refuses to start while the scene has null placeholders
(comfort_core.stage2.missing_placeholders): nothing drives toward a
goal that was never decided. The physical safety layer is Nav2's
collision monitor, configured from configs/real_amr.yaml; this script
neither reads nor changes it.

The human's position comes from /tf frame human_<key> when a tracker
publishes one (a walking participant; TODO: the real tracker node) and
from the scene file otherwise (a standing participant on a floor mark).

> No real trial has been run with this script yet.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import rclpy                                                   # noqa: E402
from action_msgs.msg import GoalStatus                         # noqa: E402
from geometry_msgs.msg import PoseStamped                      # noqa: E402
from nav2_msgs.action import NavigateToPose                    # noqa: E402
from nav2_msgs.msg import CollisionMonitorState                # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from nav_msgs.msg import Path as PathMsg                       # noqa: E402
from rclpy.action import ActionClient                          # noqa: E402
from rclpy.duration import Duration                            # noqa: E402
from rclpy.node import Node                                    # noqa: E402
from rclpy.signals import SignalHandlerOptions                 # noqa: E402
from std_msgs.msg import Bool                                  # noqa: E402
from tf2_ros import Buffer, TransformListener                  # noqa: E402

from comfort_core import load_config, nav2_map                 # noqa: E402
from comfort_core.clearance import clearance, footprint_half_extents  # noqa: E402
from comfort_core.comfort_space import (HeadingResolver, geometry_record,  # noqa: E402
                                        make_policy, shapes as build_shapes)
from comfort_core.human_state import make_state_provider       # noqa: E402
from comfort_core.intrusion import footprint_outline, intrusion  # noqa: E402
from comfort_core.layout import human_positions                # noqa: E402
from comfort_core.size_sweep import parse_condition            # noqa: E402
from comfort_core.contexts import CONTEXTS, key_of              # noqa: E402
from comfort_core.stage2 import (OPERATOR_STATUSES, TRAJECTORY_FIELDS, TRIAL_FIELDS,  # noqa: E402
                                 append_row, distance_summary, missing_placeholders,
                                 participant_ids, read_rows, session_dir, stage2_block,
                                 trial_status)
from human_tracker import HumanTracker                         # noqa: E402

MAP_FRAME = "map"
SAMPLE_PERIOD_S = 0.1


def yaw_of(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def git_commit() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=str(PROJECT_ROOT),
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


class Stage2Trial(Node):
    def __init__(self, cfg, scene: str):
        super().__init__("comfort_amr_stage2_trial")
        self.cfg = cfg
        nav = cfg["navigation"]
        self.goal = (float(nav["goal"][0]), float(nav["goal"][1]))
        self.goal_yaw = math.radians(float(nav.get("goal_yaw_deg") or 0.0))
        self.tolerance = float(nav["goal_tolerance"])
        self.max_trial_time = float(stage2_block(cfg).get("max_trial_time") or 120.0)
        self.base_frame = cfg["ros"]["frames"]["base"]
        self.half_extents = footprint_half_extents(cfg)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom = None
        self.create_subscription(Odometry, cfg["ros"]["topics"].get("odom", "/odom"),
                                 self._on_odom, 10)
        self.plan_xy = []
        self.first_plan_xy = None
        self.create_subscription(PathMsg, "/plan", self._on_plan, 10)
        self.yielding = False
        self.create_subscription(Bool, "/comfort_yield", self._on_yield, 10)
        self.safety_action = ""
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                 self._on_safety, 10)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # Hardware e-stop, if the AMR publishes one (stage2.safety.estop_topic).
        self.estop = False
        estop_topic = (stage2_block(cfg).get("safety") or {}).get("estop_topic")
        if estop_topic:
            self.create_subscription(Bool, estop_topic, self._on_estop, 10)

        # The participant: configured post, /tf override when tracked.
        self.tracker = HumanTracker(self, cfg, world_frame=MAP_FRAME)
        self.human_keys = [k for k, _x, _y in human_positions(cfg)]
        self.states_provider = make_state_provider(cfg)
        self.policy = make_policy(cfg)
        self.headings = HeadingResolver(cfg, self.policy)

    # -- callbacks
    def _on_odom(self, msg):
        self.latest_odom = msg

    def _on_plan(self, msg):
        self.plan_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.first_plan_xy is None and len(self.plan_xy) >= 2:
            self.first_plan_xy = list(self.plan_xy)

    def _on_yield(self, msg):
        self.yielding = bool(msg.data)

    def _on_estop(self, msg):
        self.estop = self.estop or bool(msg.data)

    def _on_safety(self, msg):
        names = {CollisionMonitorState.STOP: "STOP", CollisionMonitorState.SLOWDOWN: "SLOWDOWN",
                 CollisionMonitorState.APPROACH: "APPROACH"}
        self.safety_action = names.get(msg.action_type, "")

    # -- readings
    def pose(self):
        """map -> base from tf (AMCL), or None until it exists."""
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, self.base_frame, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.0))
        except Exception:  # noqa: BLE001
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, yaw_of(q))

    def speed(self) -> float:
        if self.latest_odom is None:
            return 0.0
        t = self.latest_odom.twist.twist
        return math.hypot(t.linear.x, t.linear.y)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def send_goal(self, timeout=30.0):
        if not self.client.wait_for_server(timeout_sec=timeout):
            return None
        msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.goal
        pose.pose.orientation.z = math.sin(self.goal_yaw / 2)
        pose.pose.orientation.w = math.cos(self.goal_yaw / 2)
        msg.pose = pose
        fut = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        handle = fut.result()
        return handle if handle is not None and handle.accepted else None


def run(node: Stage2Trial, verbose: bool = True) -> dict:
    """Drive once; return everything measured."""
    out = {"result": "TIMEOUT", "nav2_status": "", "trajectory": [], "min_clearance": math.inf,
           "min_clearance_t_s": math.nan, "worst_intrusion": 0.0, "comfort_seconds": 0.0,
           "yield_seconds": 0.0, "safety_stop_seconds": 0.0, "safety_slowdown_seconds": 0.0,
           "travel_distance": 0.0, "human_speeds": [], "human_state": "", "human_x": math.nan,
           "human_y": math.nan, "samples": [], "operator_status": None}
    deadline = time.time() + 30.0
    while node.pose() is None and time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose() is None:
        out["result"] = "NO_POSE"
        out["nav2_status"] = f"no tf {MAP_FRAME} -> {node.base_frame}; is AMCL localised?"
        return out
    settle = float(stage2_block(node.cfg).get("settle_seconds") or 0.0)
    if settle > 0:
        if verbose:
            print(f"  settling {settle:.1f} s", flush=True)
        t0 = time.time()
        while time.time() - t0 < settle and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    handle = node.send_goal()
    if handle is None:
        out["result"] = "REJECTED"
        out["nav2_status"] = "navigate_to_pose unavailable or rejected the goal"
        return out
    result_future = handle.get_result_async()
    x, y, yaw = node.pose()
    t_start = node.now()
    out["trial_start_time"] = datetime.now().isoformat(timespec="milliseconds")
    key = node.human_keys[0] if node.human_keys else None
    try:
        _drive(node, out, handle, result_future, t_start, key, (x, y), verbose)
    except KeyboardInterrupt:
        # The operator stopped the passing: stop the robot first, ask later.
        handle.cancel_goal_async()
        rclpy.spin_once(node, timeout_sec=0.2)
        out["result"] = "INTERRUPTED"
        out["nav2_status"] = "cancelled by the operator (Ctrl+C)"
    out["trial_end_time"] = datetime.now().isoformat(timespec="milliseconds")
    out["travel_time"] = node.now() - t_start
    p = node.pose() or (*out.get("last_xy", (math.nan, math.nan)), 0.0)
    out["final_distance_to_goal"] = math.hypot(node.goal[0] - p[0], node.goal[1] - p[1])
    if node.first_plan_xy:
        out["planned_path"] = node.first_plan_xy
        out["planned_path_length"] = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                         for a, b in zip(node.first_plan_xy, node.first_plan_xy[1:]))
    return out


def _drive(node, out, handle, result_future, t_start, key, last, verbose):
    status_names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
                    GoalStatus.STATUS_CANCELED: "CANCELED"}
    last_tick = t_start
    last_sample = -math.inf
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        p = node.pose()
        if p is None:
            continue
        x, y, yaw = p
        now = node.now()
        dt, last_tick = now - last_tick, now
        out["travel_distance"] += math.hypot(x - last[0], y - last[1])
        last = (x, y)
        out["last_xy"] = last
        if node.estop:
            handle.cancel_goal_async()
            out["result"] = "E_STOP"
            out["operator_status"] = "E_STOP"
            out["nav2_status"] = "hardware e-stop pressed"
            break
        if node.yielding:
            out["yield_seconds"] += dt
        if node.safety_action == "STOP":
            out["safety_stop_seconds"] += dt
        elif node.safety_action == "SLOWDOWN":
            out["safety_slowdown_seconds"] += dt
        states = node.states_provider.states()
        speeds = node.tracker.speeds()
        # /tf human_<key> when a tracker publishes it, the configured
        # floor mark otherwise (HumanTracker falls back by itself).
        positions = node.tracker.positions()
        hx, hy = positions.get(key, (math.nan, math.nan)) if key else (math.nan, math.nan)
        clr = clearance(node.cfg, (x, y, yaw), hx, hy) if key else math.nan
        ctr = math.hypot(x - hx, y - hy) if key else math.nan
        if key and clr < out["min_clearance"]:
            out["min_clearance"], out["min_clearance_t_s"] = clr, now - t_start
        out["samples"].append({"t": round(now - t_start, 3), "clearance": clr,
                               "center_distance": ctr, "v": node.speed()})
        if key:
            axes = node.policy.axes(states, speeds)
            heads = node.headings.resolve(states, node.tracker.headings(), speeds)
            for shape in build_shapes(node.cfg, axes, {key: (hx, hy)}, heads, 0.0):
                k, depth = intrusion(shape, footprint_outline((x, y, yaw), *node.half_extents))
                out["worst_intrusion"] = max(out["worst_intrusion"], depth)
                if k < 1.0:
                    out["comfort_seconds"] += dt
            out["human_state"] = states.get(key, "")
            out["human_x"], out["human_y"] = hx, hy
            if key in speeds:
                out["human_speeds"].append(float(speeds[key]))
        if now - last_sample >= SAMPLE_PERIOD_S:
            out["trajectory"].append([round(now - t_start, 2), round(x, 3), round(y, 3),
                                      round(yaw, 3), round(node.speed(), 3), round(hx, 3),
                                      round(hy, 3), out["human_state"], round(clr, 3),
                                      round(ctr, 3), int(node.yielding), node.safety_action])
            last_sample = now
        if result_future.done():
            st = result_future.result().status
            out["nav2_status"] = status_names.get(st, str(st))
            out["result"] = "SUCCESS" if st == GoalStatus.STATUS_SUCCEEDED else "ABORTED"
            break
        if now - t_start >= node.max_trial_time:
            handle.cancel_goal_async()
            out["result"] = "TIMEOUT"
            break


def ask_status(preset_status, preset_reason):
    """Which non-NORMAL status applies, and why. Preset values (flags) win."""
    if preset_status:
        return preset_status, preset_reason or ""
    print("\n  trial interrupted. Status?")
    for i, st in enumerate(OPERATOR_STATUSES, 1):
        print(f"    {i}  {st}")
    while True:
        raw = input("  > ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(OPERATOR_STATUSES):
            st = OPERATOR_STATUSES[int(raw) - 1]
            break
        if raw in OPERATOR_STATUSES:
            st = raw
            break
    return st, preset_reason or input("  reason (short, no personal data) > ").strip()


def from_plan(path, participant, order):
    """The plan row (scripts/stage2_make_conditions.py orders.csv) of this trial."""
    rows = [r for r in read_rows(Path(path))
            if r["participant_id"] == participant and int(r["order_index"]) == order]
    if len(rows) != 1:
        raise ValueError(f"{path}: {len(rows)} rows for {participant} order {order}")
    return rows[0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True, help="configs/stage2/scenes/<corridor>.yaml")
    ap.add_argument("--scenario", default=None, help="configs/stage2/conditions/b3o_r<size>.yaml "
                    "(from --plan when omitted)")
    ap.add_argument("--session", required=True)
    ap.add_argument("--participant", required=True, help="pseudonymous code, e.g. P01")
    ap.add_argument("--order", type=int, required=True, help="1-based position in the sequence")
    ap.add_argument("--plan", default=None,
                    help="orders.csv: context, distance, repeat and condition from the plan")
    ap.add_argument("--context", default=None,
                    help="Human Context of this trial, e.g. c2_stationary_rear (orders.csv)")
    ap.add_argument("--distance", type=float, default=None,
                    help="planned passing distance d, m (orders.csv passing_distance_m)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--operator-note", default="")
    ap.add_argument("--status", choices=OPERATOR_STATUSES, default=None,
                    help="status to record if the trial is interrupted (else asked)")
    ap.add_argument("--stop-reason", default="")
    ap.add_argument("--check", action="store_true", help="validate and exit, drive nothing")
    ap.add_argument("--no-questionnaire", action="store_true")
    args = ap.parse_args()

    if args.plan:
        row = from_plan(args.plan, args.participant, args.order)
        args.context = args.context or row["context"]
        args.distance = args.distance if args.distance is not None else float(row["passing_distance_m"])
        args.repeat = int(row.get("repeat") or args.repeat)
        args.scenario = args.scenario or str(Path(args.plan).parent / f"{row['condition']}.yaml")
    for need in ("scenario", "context", "distance"):
        if getattr(args, need) is None:
            ap.error(f"--{need} is required (or give --plan)")
    cfg = load_config(scenario=args.scenario, common_path=args.scene)
    todo = missing_placeholders(cfg)
    try:
        participant_ids(cfg, [args.participant])
    except ValueError as exc:
        todo.append(str(exc))
    cond = parse_condition(args.scenario)
    if cond is None or cond[0] != "b3o":
        todo.append(f"--scenario must be a b3o_r<size>.yaml condition, got {args.scenario}")
    try:
        ctx = key_of(args.context)
        if cfg["humans"].get("P", {}).get("state") not in (None, CONTEXTS[ctx]["state"]):
            todo.append(f"scene humans.P.state is {cfg['humans']['P']['state']!r} but context "
                        f"{args.context} needs {CONTEXTS[ctx]['state']!r}")
    except ValueError as exc:
        todo.append(str(exc))
        ctx = None
    if todo:
        print("[stage2_trial] refusing to run; fill these first:")
        for t in todo:
            print(f"   - {t}")
        return 2
    geom = geometry_record(cfg)
    trial_id = f"{args.session}_{args.participant}_{args.order:02d}"
    print(f"=== Stage 2 trial {trial_id} ===")
    print(f"  scene      : {args.scene}")
    print(f"  condition  : {cond[0]}_r{cond[1]:.2f}  size {geom['comfort_size_m']} m, "
          f"scale {geom['comfort_scale']}")
    print(f"  map        : {nav2_map(cfg)}")
    print(f"  participant: {args.participant}  order {args.order}  repeat {args.repeat}")
    print(f"  context    : {CONTEXTS[ctx]['name']}  planned d {args.distance:.2f} m")
    for line in getattr(make_policy(cfg), "describe", list)():
        print(f"      {line}")
    if args.check:
        print("  --check: config is complete; nothing driven")
        return 0

    # Ctrl+C must reach run() as KeyboardInterrupt with ROS still up, so the
    # goal can be cancelled; rclpy's own handler would shut the context down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Stage2Trial(cfg, args.scene)
    try:
        out = run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    status = trial_status(out["result"], out["operator_status"])
    reason = ""
    if out["result"] in ("INTERRUPTED", "E_STOP"):
        status, reason = ask_status(out["operator_status"] or args.status, args.stop_reason)
    elif status != "NORMAL":
        reason = out["nav2_status"]
    dist = distance_summary(out["samples"], float(cfg["humans"]["radius"]))

    def r3(v):
        return "" if v is None else round(v, 3)

    key = node.human_keys[0] if node.human_keys else ""
    h = cfg["humans"].get(key, {}) if key else {}
    state = out["human_state"] or h.get("state", "")
    ext = node.policy.extents_for(key, state, (sum(out["human_speeds"]) / len(out["human_speeds"]))
                                  if out["human_speeds"] else 0.0) if key and state else None
    facing = h.get("facing_deg")
    passing_side = ""
    if key and facing is not None and not math.isnan(out["human_x"]):
        # the robot's closest sample relative to the participant's facing
        best = min(out["trajectory"], key=lambda s: s[8], default=None)
        if best:
            ang = math.atan2(best[2] - out["human_y"], best[1] - out["human_x"]) - math.radians(float(facing))
            passing_side = "left" if math.sin(ang) > 0 else "right"
    row = {
        "stamp": datetime.now().isoformat(timespec="seconds"), "session": args.session,
        "participant_id": args.participant, "trial_id": trial_id, "trial_order": args.order,
        "repeat": args.repeat,
        "condition": f"{cond[0]}_r{cond[1]:.2f}", "context": CONTEXTS[ctx]["name"],
        "planned_distance_m": round(args.distance, 3), "policy": cfg["comfort_space"]["policy"],
        "comfort_size_m": geom["comfort_size_m"], "comfort_scale": geom["comfort_scale"],
        "config": args.scene, "scenario_file": args.scenario, "map": nav2_map(cfg),
        "commit": git_commit(),
        "trial_start_time": out.get("trial_start_time", ""),
        "trial_end_time": out.get("trial_end_time", ""),
        "trial_status": status, "stop_reason": reason, "operator_note": args.operator_note,
        "result": out["result"], "nav2_status": out["nav2_status"],
        "travel_time": round(out.get("travel_time", 0.0), 2),
        "travel_distance": round(out["travel_distance"], 3),
        "planned_path_length": round(out["planned_path_length"], 3) if "planned_path_length" in out else "",
        "final_distance_to_goal": round(out.get("final_distance_to_goal", math.nan), 3),
        "human_key": key, "human_state": state, "human_facing_deg": facing if facing is not None else "",
        "human_speed_mps": (round(sum(out["human_speeds"]) / len(out["human_speeds"]), 3)
                            if out["human_speeds"] else ""),
        "min_clearance": r3(dist["min_clearance"]),
        "actual_min_distance_m": r3(dist["actual_min_distance_m"]),
        "min_center_distance_m": r3(dist["min_center_distance_m"]),
        "min_clearance_t_s": r3(dist["min_clearance_t_s"]),
        "passing_side": passing_side,
        "amr_speed_mean_mps": r3(dist["amr_speed_mean_mps"]),
        "amr_speed_max_mps": r3(dist["amr_speed_max_mps"]),
        "amr_speed_at_min_distance_mps": r3(dist["amr_speed_at_min_distance_mps"]),
        "cs_front_m": round(ext.front, 3) if ext else "", "cs_side_m": round(ext.side, 3) if ext else "",
        "cs_rear_m": round(ext.rear, 3) if ext else "",
        "worst_intrusion": round(out["worst_intrusion"], 3),
        "comfort_seconds": round(out["comfort_seconds"], 2),
        "yield_seconds": round(out["yield_seconds"], 2),
        "safety_stop_seconds": round(out["safety_stop_seconds"], 2),
        "safety_slowdown_seconds": round(out["safety_slowdown_seconds"], 2),
    }
    folder = session_dir(cfg, args.session)
    append_row(folder / "trials.csv", row, TRIAL_FIELDS)
    if folder.joinpath(f"{trial_id}.json").exists():
        print(f"[WARN] {trial_id} was recorded before; both rows are kept, mark one "
              "INVALID_TRIAL with scripts/stage2_mark_trial.py")
    traj = folder / "trajectories" / f"{trial_id}.csv"
    traj.parent.mkdir(parents=True, exist_ok=True)
    with traj.open("w", encoding="utf-8") as fh:
        fh.write(",".join(TRAJECTORY_FIELDS) + "\n")
        for s in out["trajectory"]:
            fh.write(",".join(str(v) for v in s) + "\n")
    (folder / f"{trial_id}.json").write_text(json.dumps(
        {"row": row, "planned_path": out.get("planned_path", []),
         "applied_geometry": geom}, indent=2), encoding="utf-8")
    print(f"\n=== Result ===\n  status        : {status}{(' (' + reason + ')') if reason else ''}")
    print(f"  result        : {out['result']}  ({out['nav2_status']})")
    print(f"  min distance  : {row['actual_min_distance_m']} m person centre -> AMR surface "
          f"(centre-centre {row['min_center_distance_m']} m), AMR {row['amr_speed_at_min_distance_mps']} m/s")
    print(f"  travel        : {row['travel_time']} s, {row['travel_distance']} m")
    print(f"  min clearance : {row['min_clearance']} m at t={row['min_clearance_t_s']} s "
          f"({passing_side or 'side unknown'})")
    print(f"  comfort space : F {row['cs_front_m']} S {row['cs_side_m']} R {row['cs_rear_m']} m; "
          f"intrusion {row['worst_intrusion']} m, {row['comfort_seconds']} s inside")
    print(f"  yield {row['yield_seconds']} s, safety stop {row['safety_stop_seconds']} s")
    print(f"  recorded      : {folder / 'trials.csv'}")
    if status != "NORMAL":
        print(f"  no questionnaire: status {status} (stored separately, not a comfort rating)")
        return 1
    if not args.no_questionnaire:
        cmd = [sys.executable, str(PROJECT_ROOT / "scripts/stage2_questionnaire.py"),
               "--config", args.scene,
               "--session-dir", str(folder), "--participant", args.participant,
               "--repeat", str(args.repeat),
               "--trial-id", trial_id, "--condition", row["condition"],
               "--size", str(row["comfort_size_m"]), "--order", str(args.order),
               "--context", row["context"], "--passing-distance", str(args.distance)]
        print("\n  questionnaire: " + " ".join(cmd[1:]))
        return subprocess.call(cmd)
    return 0 if out["result"] == "SUCCESS" else 1


if __name__ == "__main__":
    sys.exit(main())
