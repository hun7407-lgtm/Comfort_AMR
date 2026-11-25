#!/usr/bin/env python3
"""Per-trial interaction log for the final Stage 1-3 experiments.

    TRIAL_LOG=<file.jsonl> ./scripts/run_scenario.sh ...   (started by run_scenario.sh)
    python3 scripts/trial_logger.py --config <cfg> --scenario <overlay> --out <file.jsonl>

Runs in the SYSTEM Python with ROS 2 Jazzy. Listens only; publishes
nothing and is not in any control loop.

Writes one JSON object per line, on the simulation clock:

    {"ev": "tick", "t": ..., "amr": {name: [x, y, yaw]}, "cm": {name: action_type},
     "gate": "<AMR 1 gate state>", "people": {key: {s, o, x, y, v, h, F, S, R}}}   at 5 Hz
    {"ev": "plan", "t": ..., "amr": name, "len": metres, "n": poses, "pts": [[x, y], ...]}
    {"ev": "fleet", "t": ..., "msg": "<line of /fleet/status>"}

`people` is /comfort_state_log from comfort_publisher (declared and observed
state, position, heading, speed and the person's own Comfort Space extents).
`pts` is the plan thinned to at most 80 points. Every line is flushed, so a
trial that is killed keeps everything written up to that moment.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from comfort_core import load_config


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrialLogger(Node):
    def __init__(self, cfg, out: Path, rate_hz: float = 5.0):
        super().__init__("trial_logger")
        self.fh = open(out, "a", buffering=1)
        self.sim_t = None
        self.people = {}
        self.gate = None
        self.cm = {}
        fleet = cfg.get("fleet", {}) or {}
        self.vehicles = [n for n, v in (fleet.get("amrs") or {}).items()
                         if fleet.get("enabled") and isinstance(v, dict) and v.get("enabled", True)]
        self.frames = {"AMR1": cfg["ros"]["frames"]["base"],
                       **{n: f"{n}/base_footprint" for n in self.vehicles}}
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, "/comfort_state_log", self._on_people, 10)
        self.create_subscription(String, "/comfort_gate_state", self._on_gate, 10)
        self.create_subscription(String, "/fleet/status", self._on_fleet, 50)
        for name, prefix in [("AMR1", "")] + [(n, f"/{n}") for n in self.vehicles]:
            self.create_subscription(PathMsg, f"{prefix}/plan",
                                     lambda m, n=name: self._on_plan(n, m), 10)
            self.create_subscription(CollisionMonitorState, f"{prefix}/collision_monitor_state",
                                     lambda m, n=name: self.cm.__setitem__(n, int(m.action_type)), 10)
        self._last_tick = None
        self.period = 1.0 / rate_hz
        self.create_timer(0.05, self._tick)

    def _write(self, obj) -> None:
        self.fh.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def _on_clock(self, msg: Clock) -> None:
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_people(self, msg: String) -> None:
        try:
            self.people = json.loads(msg.data)
        except ValueError:
            pass

    def _on_gate(self, msg: String) -> None:
        self.gate = msg.data

    def _on_fleet(self, msg: String) -> None:
        self._write({"ev": "fleet", "t": self.sim_t, "msg": msg.data})

    def _on_plan(self, name: str, msg: PathMsg) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        step = max(1, len(pts) // 80)
        thin = pts[::step]
        if pts and thin[-1] != pts[-1]:
            thin.append(pts[-1])
        self._write({"ev": "plan", "t": self.sim_t, "amr": name, "len": round(length, 3), "n": len(pts),
                     "pts": [[round(x, 2), round(y, 2)] for x, y in thin]})

    def _pose(self, frame: str):
        try:
            tf = self.tf.lookup_transform("map", frame, rclpy.time.Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        t = tf.transform.translation
        return [round(t.x, 3), round(t.y, 3), round(_yaw(tf.transform.rotation), 4)]

    def _tick(self) -> None:
        if self.sim_t is None:
            return
        if self._last_tick is not None and self.sim_t - self._last_tick < self.period:
            return
        self._last_tick = self.sim_t
        amr = {}
        for name, frame in self.frames.items():
            p = self._pose(frame)
            if p is not None:
                amr[name] = p
        self._write({"ev": "tick", "t": round(self.sim_t, 3), "amr": amr, "cm": dict(self.cm),
                     "gate": self.gate, "people": self.people})


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default="configs/common.yaml")
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rate", type=float, default=5.0)
    args = ap.parse_args()
    cfg = load_config(scenario=args.scenario, common_path=args.config)
    rclpy.init()
    node = TrialLogger(cfg, Path(args.out), args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.fh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
