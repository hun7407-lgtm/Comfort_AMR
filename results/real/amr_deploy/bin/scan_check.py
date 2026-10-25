#!/usr/bin/env python3
"""Report how usable /scan is for 2D SLAM. Run on the AMR (Humble)."""
import math, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class Check(Node):
    def __init__(self, n):
        super().__init__('scan_check')
        self.n = n; self.scans = []; self.clouds = []
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/cloud', self.cb2, qos_profile_sensor_data)
    def cb(self, m):
        if len(self.scans) < self.n: self.scans.append(m)
    def cb2(self, m):
        if len(self.clouds) < 5: self.clouds.append(m)

rclpy.init()
n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
c = Check(n)
import time; t0 = time.time()
while (len(c.scans) < n or len(c.clouds) < 5) and time.time()-t0 < 25:
    rclpy.spin_once(c, timeout_sec=0.2)

print(f"--- /lidar/cloud ({len(c.clouds)} msgs) ---")
if c.clouds:
    m = c.clouds[-1]
    pts = np.array([[p[0],p[1],p[2]] for p in pc2.read_points(m, field_names=('x','y','z'), skip_nans=True)])
    r = np.linalg.norm(pts, axis=1)
    el = np.degrees(np.arcsin(np.clip(pts[:,2]/np.maximum(r,1e-6), -1, 1)))
    print(f"  frame={m.header.frame_id} points/msg={len(pts)}")
    print(f"  elevation {el.min():+.1f}..{el.max():+.1f} deg   range {r.min():.2f}..{r.max():.2f} m")

print(f"--- /scan ({len(c.scans)} msgs) ---")
if not c.scans:
    print("  NO SCAN -- pointcloud_to_laserscan not running?"); raise SystemExit(1)
s = c.scans[-1]
nb = len(s.ranges)
print(f"  frame={s.header.frame_id} bins={nb} angle_inc={math.degrees(s.angle_increment):.3f} deg")
print(f"  range limits {s.range_min:.2f}..{s.range_max:.2f} m")
valid = []
for sc in c.scans:
    a = np.array(sc.ranges)
    v = np.isfinite(a) & (a > sc.range_min) & (a < sc.range_max)
    valid.append(v.sum())
valid = np.array(valid)
print(f"  valid beams per msg: min {valid.min()} median {int(np.median(valid))} max {valid.max()}")
a = np.array(s.ranges); v = np.isfinite(a) & (a > s.range_min) & (a < s.range_max)
if v.sum():
    rr = a[v]
    print(f"  valid ranges {rr.min():.2f}..{rr.max():.2f} m, median {np.median(rr):.2f}")
    ang = s.angle_min + np.arange(nb)*s.angle_increment
    # nearest return each side -> corridor width estimate
    for lbl, lo, hi in (("left (+60..+120)", 60, 120), ("right (-120..-60)", -120, -60),
                        ("front (-15..+15)", -15, 15), ("back (165..195)", 165, 195)):
        m2 = v & (np.degrees(ang) >= lo) & (np.degrees(ang) <= hi)
        print(f"    {lbl:20s} n={m2.sum():4d} nearest={rr.min() if False else (a[m2].min() if m2.sum() else float('nan')):.2f} m")
# accumulate N scans into one -> what multi-sweep accumulation would give
acc = np.full(nb, np.inf)
for sc in c.scans[:3]:
    a2 = np.array(sc.ranges); v2 = np.isfinite(a2)&(a2>sc.range_min)&(a2<sc.range_max)
    acc[v2] = np.minimum(acc[v2], a2[v2])
print(f"  if 3 sweeps accumulated: {np.isfinite(acc).sum()} valid beams")
c.destroy_node(); rclpy.shutdown()
