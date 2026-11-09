#!/usr/bin/env python3
"""Does the Comfort Space costmap change the trajectory DWB picks?

Reading configs cannot answer that, and neither can watching the robot:
a trajectory that avoids someone looks the same whether the costmap
steered it or the global path happened to go that way. DWB publishes its
own scoring, so ask it.

    ./scripts/check_dwb_scores.py                     # until Ctrl-C
    ./scripts/check_dwb_scores.py --seconds 60
    ./scripts/check_dwb_scores.py --topic /controller_server/evaluation

Every control cycle DWB publishes a LocalPlanEvaluation: every candidate
trajectory, and for each one a CriticScore per critic with its raw_score
and its scale. The chosen trajectory is the lowest total.

Two things are counted.

  share        BaseObstacle's contribution as a fraction of the winning
               trajectory's total cost. Says whether the costmap is
               visible at all.

  decisive     the cycles where removing BaseObstacle's contribution
               changes which trajectory wins. This is the one that
               matters: it is the number of times the Comfort Space
               actually steered the robot, rather than agreeing with a
               choice already made on other grounds.

BaseObstacle is the only critic that reads the costmap, so it is the only
route by which a Comfort Space can reach the controller.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from dwb_msgs.msg import LocalPlanEvaluation

CRITIC = "BaseObstacle"


def discover(preferred: str, wait: float) -> str:
    """Find the controller's evaluation topic.

    DWB publishes it under the controller node, so the name depends on how
    the stack was brought up. Waiting matters as much as searching: this
    is normally started alongside the trial, and Nav2 takes a while.
    """
    probe = rclpy.create_node("check_dwb_scores_probe")
    try:
        deadline = probe.get_clock().now().nanoseconds + wait * 1e9
        while rclpy.ok():
            names = dict(probe.get_topic_names_and_types())
            if any(t.endswith("/LocalPlanEvaluation")
                   for t in names.get(preferred, [])):
                return preferred
            for name, types in sorted(names.items()):
                if any(t.endswith("/LocalPlanEvaluation") for t in types):
                    return name
            if probe.get_clock().now().nanoseconds >= deadline:
                return preferred
            rclpy.spin_once(probe, timeout_sec=0.5)
        return preferred
    finally:
        probe.destroy_node()


class ScoreWatch(Node):
    def __init__(self, topic: str, critic: str):
        super().__init__("check_dwb_scores")
        self.critic = critic
        self.cycles = 0
        self.rejected_only = 0
        self.critic_present = 0
        self.critic_nonzero = 0
        self.decisive = 0
        self.share_sum = 0.0
        self.share_max = 0.0
        self.totals = defaultdict(float)
        self.seen_critics: Counter = Counter()
        self.sub = self.create_subscription(
            LocalPlanEvaluation, topic, self.on_eval,
            QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.get_logger().info(f"listening on {topic}")

    @staticmethod
    def _total(score, skip=None) -> float:
        return sum(c.raw_score * c.scale for c in score.scores
                   if c.name != skip)

    @staticmethod
    def _contribution(score, name) -> float:
        for c in score.scores:
            if c.name == name:
                return c.raw_score * c.scale
        return None

    def on_eval(self, msg: LocalPlanEvaluation) -> None:
        if not msg.twists:
            return
        self.cycles += 1
        best = msg.twists[msg.best_index]

        for c in best.scores:
            self.seen_critics[c.name] += 1
            self.totals[c.name] += abs(c.raw_score * c.scale)

        mine = self._contribution(best, self.critic)
        if mine is None:
            return
        self.critic_present += 1
        if abs(mine) > 1e-12:
            self.critic_nonzero += 1

        total = self._total(best)
        if abs(total) > 1e-12:
            share = abs(mine) / abs(total)
            self.share_sum += share
            self.share_max = max(self.share_max, share)

        # The counterfactual. A trajectory DWB rejected outright keeps a
        # negative score, so only the candidates it was willing to use are
        # comparable; if every candidate was rejected there is nothing to
        # choose between.
        usable = [(i, t) for i, t in enumerate(msg.twists)
                  if all(c.raw_score >= 0.0 for c in t.scores)]
        if len(usable) < 2:
            self.rejected_only += 1
            return
        without = min(usable, key=lambda it: self._total(it[1], skip=self.critic))
        if without[0] != msg.best_index:
            self.decisive += 1

    def report(self) -> None:
        print()
        print(f"[check_dwb_scores] {self.cycles} control cycle(s)")
        if not self.cycles:
            print("  nothing arrived. Is the controller running, and is")
            print("  publish_evaluation left at its default of true?")
            return
        print(f"  critics seen       "
              f"{', '.join(sorted(self.seen_critics))}")
        if not self.critic_present:
            print(f"  {self.critic} never appeared -- it is not in the "
                  f"critics list, so the costmap cannot reach the controller")
            return
        mean = self.share_sum / max(self.critic_present, 1)
        print(f"  {self.critic:<18} nonzero in "
              f"{self.critic_nonzero}/{self.critic_present} cycles")
        print(f"  cost share         mean {mean * 100:.2f}%, "
              f"peak {self.share_max * 100:.2f}%")
        print(f"  DECISIVE           {self.decisive}/{self.cycles} cycles "
              f"({self.decisive / self.cycles * 100:.1f}%) -- removing it "
              f"would have changed the chosen trajectory")
        if self.rejected_only:
            print(f"  (in {self.rejected_only} cycle(s) fewer than two "
                  f"candidates were usable, so nothing was comparable)")
        print()
        print("  mean |raw_score x scale| on the winning trajectory:")
        for name in sorted(self.totals, key=lambda k: -self.totals[k]):
            print(f"    {name:<16} {self.totals[name] / self.cycles:10.3f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/controller_server/evaluation")
    ap.add_argument("--critic", default=CRITIC)
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this long; default runs until Ctrl-C")
    ap.add_argument("--wait", type=float, default=60.0,
                    help="seconds to wait for the controller to appear")
    ap.add_argument("--no-discover", action="store_true",
                    help="use --topic exactly, do not go looking")
    args = ap.parse_args()

    rclpy.init()
    topic = args.topic
    if not args.no_discover:
        found = discover(topic, args.wait)
        if found and found != topic:
            print(f"[check_dwb_scores] {topic} has no publisher; using {found}")
            topic = found
    node = ScoreWatch(topic, args.critic)
    deadline = None
    if args.seconds is not None:
        deadline = node.get_clock().now().nanoseconds + args.seconds * 1e9
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if deadline and node.get_clock().now().nanoseconds >= deadline:
                break
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
