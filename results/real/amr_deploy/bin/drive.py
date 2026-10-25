#!/usr/bin/env python3
"""Drive the AMR a fixed distance in a straight line, then stop. Runs ON the AMR.

    python3 drive.py <metres>            positive forward, negative reverse
    python3 drive.py <metres> <speed>    speed in m/s, default 0.15

Stops on whichever comes first:
  * the odometry says the distance has been covered
  * a time limit sized from the distance plus 50 %, so a stalled or NaN
    odometry cannot leave the robot driving
  * the LiDAR sees something closer than STOP_RANGE ahead (or behind, in
    reverse) within the width of the robot
  * the process is killed -- a zero Twist is published from a finally block

Why a time limit AND an odometry limit: the encoders on this robot report
NaN when the drive board is unpowered, and `distance >= target` is false
forever when distance is NaN. The clock is the backstop.

The LiDAR is blind below 0.31 m and this robot publishes no ultrasonic
topic, so the range check cannot see a low obstacle. It is a backstop for
walls and people, not a substitute for the operator holding the E-STOP.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Geometry of the swept corridor ahead of (or behind) the robot. All measured
# in lidar_link, which sits at the robot's centre.
#
# FRONT_FACE matters more than it looks. An earlier version counted any return
# with x > 0 and |y| <= HALF_WIDTH and took x as "distance ahead". A point
# 0.25 m away at 60 degrees off axis is BESIDE the robot, but its x is 0.13,
# so the robot reported an obstacle 0.13 m ahead and refused to move -- three
# times, in a corridor that was empty for five metres. Only what is beyond the
# front face is ahead.
STOP_RANGE = 0.50       # m of clearance required in front of the front face
HALF_WIDTH = 0.40       # m, a little wider than the robot's 0.285 half width
FRONT_FACE = 0.432      # m, half the 0.864 m footprint length


class Drive(Node):
    def __init__(self, target, speed):
        super().__init__('drive')
        self.target = abs(target)
        self.sign = 1.0 if target >= 0 else -1.0
        self.speed = abs(speed)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.p0 = None
        self.dist = 0.0
        # Blockage is judged from the two most recent scans, never latched.
        # An upward-looking LiDAR flattened into 2D puts overhead structure at
        # a short HORIZONTAL range -- a ceiling return at 1.02 m and 66
        # degrees elevation lands 0.41 m in front of the robot. One such frame
        # used to set a sticky flag and the robot then refused to move at all.
        self.near_hist = [float('inf'), float('inf')]

    def odom(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if any(math.isnan(v) for v in p):
            return
        if self.p0 is None:
            self.p0 = p
        self.dist = math.hypot(p[0] - self.p0[0], p[1] - self.p0[1])

    def scan(self, m):
        near = float('inf')
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            a = m.angle_min + i * m.angle_increment
            x, y = r * math.cos(a), r * math.sin(a)
            if abs(y) > HALF_WIDTH:
                continue
            # Clearance from the face the robot is driving towards.
            ahead = (x - FRONT_FACE) if self.sign > 0 else (-x - FRONT_FACE)
            if ahead <= 0:
                continue        # beside or behind the footprint, not ahead
            near = min(near, ahead)
        self.near_hist = [self.near_hist[-1], near]

    @property
    def near(self):
        return self.near_hist[-1]

    def blocked(self):
        return all(n < STOP_RANGE for n in self.near_hist)

    def stop(self):
        t = Twist()
        for _ in range(10):
            self.pub.publish(t)
            time.sleep(0.02)


def main():
    target = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    rclpy.init()
    d = Drive(target, speed)

    deadline = time.time() + abs(target) / speed * 1.5 + 4.0
    print(f"target {target:+.2f} m at {speed:.2f} m/s, hard stop at "
          f"{deadline - time.time():.1f} s", flush=True)

    # Let the subscriptions fill before committing to motion.
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(d, timeout_sec=0.1)
    if d.p0 is None:
        print("ABORT: no finite odometry. Drive board unpowered (encoders NaN)?",
              flush=True)
        d.stop(); d.destroy_node(); rclpy.shutdown(); return 1
    if d.blocked():
        print(f"ABORT: obstacle at {d.near:.2f} m before starting "
              f"(last two scans {d.near_hist[0]:.2f}, {d.near_hist[1]:.2f})", flush=True)
        d.stop(); d.destroy_node(); rclpy.shutdown(); return 1

    reason = "distance"
    t = Twist()
    try:
        while rclpy.ok():
            rclpy.spin_once(d, timeout_sec=0.05)
            if d.dist >= d.target:
                break
            if time.time() > deadline:
                reason = "TIME LIMIT"; break
            if d.blocked():
                reason = f"OBSTACLE at {d.near:.2f} m"; break
            t.linear.x = d.sign * d.speed
            d.pub.publish(t)
    finally:
        d.stop()
    print(f"stopped: {reason}. travelled {d.dist:.3f} m of {d.target:.2f} m",
          flush=True)
    d.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
