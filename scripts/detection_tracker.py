"""Track where the humans are right now, from the LiDAR.

The drop-in alternative to `human_tracker.HumanTracker`, which reads
the true poses off /tf. This one subscribes to `/comfort_detections`,
the output of `scripts/people_tracker.py`, and therefore knows only
what the robot's own sensor can establish:

    a person the sensor cannot see does not exist,
    a person is wherever the returns say, to about 0.05 m,
    and a person has no name.

The last one is the awkward part. Everything downstream was written
against config keys -- `w1`, `w4` -- because the Comfort Space policy
asks a provider what state each named person is in. A detection has a
track number instead, so with this tracker the states have to come
from `state_source: sensor`, which works them out from position and
speed against the map. Declared states are not available and must not
be faked: the point of running this way is that nothing in the loop
knows more than the robot does.

Measured against the true poses in the U bay, 0.8 m/s, B3:

    position error   median 0.05 m, 90th pct 0.12 m
    speed error      median 0.02 m/s (0.04 for people actually walking)
    found            100% within 2 m, 93% at 2-4 m, 67% at 4-6 m
    false positives  none in 496 sweeps -- no pillar or cart was ever
                     reported as a person

The fall-off with range is the sensor, not the detector: this LiDAR
sits 0.31 m up and looks upward, so past about 6 m its beams are over
a person's head, and most of the people further away than that are
behind a machine. What matters for Comfort Space is the near field,
where it is reliable.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Dict, Tuple

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class DetectionTracker:
    """Positions, speeds and headings of whoever the sensor can see."""

    def __init__(self, node: Node, cfg: Dict[str, Any],
                 topic: str = "/comfort_detections",
                 forget_after: float = 1.0):
        self.node = node
        self.cfg = cfg
        # How long a track is kept after the sensor stops reporting it.
        #
        # Not zero: people pass behind a machine for a second and come
        # back, and dropping them makes their Comfort Space blink out
        # and the robot drive into where they were about to be. Not
        # long either, because a remembered person the robot keeps
        # yielding to is a deadlock that no sensor reading can end.
        self.forget_after = float(forget_after)

        # How long a name is reserved after its track is dropped.
        #
        # Keeping identities stable is not cosmetic. A person who steps
        # behind a machine for two seconds and comes back is the same
        # person, and the upstream tracker cannot say so: it drops a
        # track after a few missed sweeps and the next detection starts
        # a new one, which has to be seen three times before it is
        # published again. Measured in the U bay: twelve people
        # produced forty-four track identities in one trial, the state
        # estimate "changed" 197 times where the ground-truth loop
        # changed 0 times, and B3 never reached the goal because a
        # fresh identity arrives with a fresh Comfort Space.
        #
        # A retired name is only a name. It reserves nothing in space
        # and produces no Comfort Space, so remembering one for a few
        # seconds cannot block the robot the way holding the track
        # itself would.
        self.name_memory = 5.0
        self._retired: Dict[str, Tuple[float, float, float]] = {}

        self._tracks: Dict[str, Tuple[float, float, float, float, float]] = {}
        self._last_heading: Dict[str, float] = {}
        self._history: Dict[str, deque] = {}
        self._window = float(cfg["comfort_space"].get("velocity_window_seconds", 1.0))
        self._now = 0.0
        self._seen: set[str] = set()

        node.create_subscription(Float32MultiArray, topic, self._on_detections, 10)

    # -- the HumanTracker interface -----------------------------------

    def positions(self) -> Dict[str, Tuple[float, float]]:
        self._forget()
        return {k: (t[0], t[1]) for k, t in self._tracks.items()}

    def speeds(self) -> Dict[str, float]:
        self._forget()
        return {k: math.hypot(t[2], t[3]) for k, t in self._tracks.items()}

    def velocities(self) -> Dict[str, tuple]:
        self._forget()
        return {k: (t[2], t[3]) for k, t in self._tracks.items()}

    def headings(self) -> Dict[str, float]:
        self._forget()
        out = {}
        for key, t in self._tracks.items():
            if math.hypot(t[2], t[3]) > 1e-3:
                out[key] = math.atan2(t[3], t[2])
                self._last_heading[key] = out[key]
            else:
                out[key] = self._last_heading.get(key, 0.0)
        return out

    def observed_states(self, declared: Dict[str, str],
                        threshold: float) -> Dict[str, str]:
        """What the people are doing, for reporting only.

        With this tracker `declared` already came from the sensor, so
        there is nothing to downgrade -- it is returned as it arrived.
        """
        return dict(declared)

    def moved(self, previous: Dict[str, Tuple[float, float]],
              tolerance: float = 0.05) -> bool:
        current = self.positions()
        if set(current) != set(previous):
            return True
        return any(math.hypot(current[k][0] - previous[k][0],
                              current[k][1] - previous[k][1]) > tolerance
                   for k in current)

    # -- internals ----------------------------------------------------

    def _on_detections(self, msg: Float32MultiArray) -> None:
        """x, y, vx, vy, hits per track, as people_tracker publishes it."""
        data = list(msg.data)
        if len(data) % 5:
            return
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self._now = now

        fresh: Dict[str, Tuple[float, float, float, float, float]] = {}
        for i in range(0, len(data), 5):
            x, y, vx, vy = (float(v) for v in data[i:i + 4])
            key = self._match(x, y, fresh)
            fresh[key] = (x, y, vx, vy, now)
            if key not in self._seen:
                self._seen.add(key)
                self.node.get_logger().info(
                    f"tracking person {key} from the LiDAR "
                    f"at ({x:+.2f}, {y:+.2f})")
        # Keep anyone the sensor has lately lost sight of, until
        # _forget drops them.
        for key, t in self._tracks.items():
            if key not in fresh:
                fresh[key] = t
        self._tracks = fresh

    def _match(self, x: float, y: float, fresh) -> str:
        """The name of whoever is already at this position, or a new one.

        people_tracker assigns its own track numbers, but it does not
        promise to keep them: a person lost behind a machine for a
        second comes back as a new number. Matching on position instead
        keeps one person one person, which is what the Comfort Space
        history and the velocity window need.
        """
        best, best_d = None, 0.8
        for key, t in self._tracks.items():
            if key in fresh:
                continue
            d = math.hypot(t[0] - x, t[1] - y)
            if d < best_d:
                best, best_d = key, d
        if best is not None:
            return best

        # Nobody live is here. Somebody who WAS here, recently, is the
        # better answer than a new stranger.
        for key, (rx, ry, when) in sorted(
                self._retired.items(),
                key=lambda kv: math.hypot(kv[1][0] - x, kv[1][1] - y)):
            if key in fresh or key in self._tracks:
                continue
            if self._now - when > self.name_memory:
                continue
            if math.hypot(rx - x, ry - y) < 1.5:
                self._retired.pop(key, None)
                return key

        n = 1
        while (f"p{n}" in self._tracks or f"p{n}" in fresh
               or f"p{n}" in self._retired):
            n += 1
        return f"p{n}"

    def _forget(self) -> None:
        if not self._tracks:
            return
        now = max(self._now,
                  self.node.get_clock().now().nanoseconds * 1e-9)
        keep, drop = {}, {}
        for k, t in self._tracks.items():
            (keep if now - t[4] <= self.forget_after else drop)[k] = t
        self._tracks = keep
        for k, t in drop.items():
            self._retired[k] = (t[0], t[1], t[4])
        self._retired = {k: v for k, v in self._retired.items()
                         if now - v[2] <= self.name_memory}
