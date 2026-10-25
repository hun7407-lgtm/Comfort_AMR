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
from collections import deque
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
        self._latest_stamp = 0.0

        # Velocity is taken across this much simulated time, not between
        # two consecutive samples. A walking character sways side to
        # side; differenced over 0.1 s that sway swings the heading by
        # many degrees, and the yield gate extends the heading several
        # metres to find a walker's lane. In a stress test of 216 trials
        # with 2 cm of position noise, a 0.1 s window let the walker reach
        # the robot 3 times and a 1.0 s window never did.
        self._window = float(cfg["comfort_space"].get("velocity_window_seconds", 1.0))
        self._history: Dict[str, deque] = {}

    def positions(self) -> Dict[str, Tuple[float, float]]:
        """Current (x, y) per human, in world coordinates. Cached."""
        now = time.monotonic()
        if now - self._cache_time < self._refresh_interval:
            return self._cache
        fresh = self._lookup()
        # Differentiate over SIMULATED time, from the /tf stamps. Isaac
        # Sim does not run at real-time factor 1, so a wall-clock
        # difference misstates every speed by that factor, and the gate
        # decides how soon a walker arrives from this speed.
        stamp = self._latest_stamp
        if stamp and stamp > self._prev_time + 1e-6:
            for key, (x, y) in fresh.items():
                hist = self._history.setdefault(key, deque())
                hist.append((stamp, x, y))
                while len(hist) > 2 and stamp - hist[1][0] >= self._window:
                    hist.popleft()
                t0, x0, y0 = hist[0]
                # Nothing until the window has filled. A walker who has
                # just set off has a few tenths of a second of samples, and
                # a heading taken from those was measured 5 deg or more out;
                # the gate extends that heading ten metres to find their
                # lane and then chose a spot inside it.
                if stamp - t0 >= 0.9 * self._window:
                    self._velocity[key] = ((x - x0) / (stamp - t0),
                                           (y - y0) / (stamp - t0))
            self._prev = dict(fresh)
            self._prev_time = stamp

        self._cache = fresh
        self._cache_time = now
        return self._cache

    def speeds(self) -> Dict[str, float]:
        """Estimated ground speed per human, m/s."""
        self.positions()
        return {k: (vx * vx + vy * vy) ** 0.5
                for k, (vx, vy) in self._velocity.items()}

    def velocities(self) -> Dict[str, tuple]:
        """Estimated ground velocity per human, (vx, vy) in m/s."""
        self.positions()
        return dict(self._velocity)

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
            st = tf.header.stamp
            self._latest_stamp = max(self._latest_stamp,
                                     st.sec + st.nanosec * 1e-9)
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
