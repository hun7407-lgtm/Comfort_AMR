"""Track where the humans are right now, from /tf.

Runs in the SYSTEM Python with ROS 2 Jazzy. Imported by both
comfort_publisher.py and comfort_gate.py so they cannot disagree about
where a person is.

Isaac Sim publishes a frame per human collider, parented to the stage's
world frame, which coincides with the map frame in this project. Reading
the pose from there rather than from the config is what lets a Comfort
Space follow a walking person.

Falls back to the configured position for any human whose frame has not
arrived yet, so a scene with only stationary humans behaves exactly as
it did before this existed.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from comfort_core.layout import human_positions

WORLD_FRAME = "world"


class HumanTracker:
    def __init__(self, node: Node, cfg: Dict[str, Any],
                 world_frame: str = WORLD_FRAME,
                 refresh_interval: float = 0.1):
        self.node = node
        self.cfg = cfg
        self.world_frame = world_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

        self._configured = {k: (x, y) for k, x, y in human_positions(cfg)}
        self._seen: set[str] = set()

        # Positions are cached and refreshed at most this often.
        #
        # comfort_gate calls this from its /cmd_vel callback, which runs
        # at the controller rate. Doing a TF lookup per human per message
        # there, with an exception raised for every frame that has not
        # arrived yet, delayed the velocity command enough to make Nav2
        # stutter: a B3 that reached the goal in 26 s started deadlocking.
        # Humans move far slower than the control loop, so 10 Hz is ample.
        self._refresh_interval = float(refresh_interval)
        self._cache: Dict[str, Tuple[float, float]] = dict(self._configured)
        self._cache_time = 0.0

        # Speed and heading, estimated by differencing successive
        # positions. A person's state is read from what they are doing
        # rather than from what the config declared, which is the same
        # thing a sensor would have to do and is what lets the Comfort
        # Space shrink when a walker stops.
        self._velocity: Dict[str, Tuple[float, float]] = {}
        self._prev: Dict[str, Tuple[float, float]] = dict(self._configured)
        self._prev_time = 0.0
        self._last_heading: Dict[str, float] = {}

    def positions(self) -> Dict[str, Tuple[float, float]]:
        """Current (x, y) per human, in world coordinates. Cached."""
        now = time.monotonic()
        if now - self._cache_time < self._refresh_interval:
            return self._cache
        fresh = self._lookup()
        dt = now - self._prev_time if self._prev_time else 0.0
        if dt > 1e-3:
            for key, (x, y) in fresh.items():
                px, py = self._prev.get(key, (x, y))
                self._velocity[key] = ((x - px) / dt, (y - py) / dt)
            self._prev = dict(fresh)
            self._prev_time = now
        elif not self._prev_time:
            self._prev_time = now

        self._cache = fresh
        self._cache_time = now
        return self._cache

    def speeds(self) -> Dict[str, float]:
        """Estimated ground speed per human, m/s."""
        self.positions()
        return {k: (vx * vx + vy * vy) ** 0.5
                for k, (vx, vy) in self._velocity.items()}

    def headings(self) -> Dict[str, float]:
        """Direction of travel per human, radians from +x.

        Held at the last travelled direction when a person stops, so the
        ellipse does not spin about while they stand still.
        """
        import math
        self.positions()
        out = {}
        for key, (vx, vy) in self._velocity.items():
            if (vx * vx + vy * vy) ** 0.5 > 1e-3:
                out[key] = math.atan2(vy, vx)
                self._last_heading[key] = out[key]
            else:
                out[key] = self._last_heading.get(key, 0.0)
        return out

    def observed_states(self, declared: Dict[str, str],
                        threshold: float) -> Dict[str, str]:
        """Declared states, with `moving` downgraded when nobody is moving.

        Only `moving` is reinterpreted. A person the config calls
        stationary or station_engaged is not going to start walking, and
        second-guessing that would let measurement noise change the
        experiment.
        """
        speeds = self.speeds()
        out = dict(declared)
        for key, state in declared.items():
            if state != "moving":
                continue
            if speeds.get(key, 0.0) < threshold:
                out[key] = "stationary"
        return out

    def _lookup(self) -> Dict[str, Tuple[float, float]]:
        out: Dict[str, Tuple[float, float]] = {}
        for key, cx, cy in human_positions(self.cfg):
            frame = f"human_{key}"
            try:
                tf = self.buffer.lookup_transform(
                    self.world_frame, frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.0))
            except Exception:
                out[key] = (cx, cy)
                continue
            t = tf.transform.translation
            out[key] = (t.x, t.y)
            if key not in self._seen:
                self._seen.add(key)
                self.node.get_logger().info(
                    f"tracking human {key} from /tf ({self.world_frame} -> {frame})")
        return out

    def moved(self, previous: Dict[str, Tuple[float, float]],
              current: Dict[str, Tuple[float, float]],
              tolerance: float = 0.02) -> bool:
        """True if any human has shifted by more than `tolerance`."""
        if set(previous) != set(current):
            return True
        for key, (px, py) in previous.items():
            cx, cy = current[key]
            if abs(cx - px) > tolerance or abs(cy - py) > tolerance:
                return True
        return False
