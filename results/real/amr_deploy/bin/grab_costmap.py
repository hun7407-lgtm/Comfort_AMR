#!/usr/bin/env python3
"""Save the global costmap, the Comfort Space mask and the robot pose.

    grab_costmap.py <out.npz>

Saved rather than rendered here: the robot has no display, and keeping the
raw grids means the picture can be redrawn without going back to the corridor.
"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)

class Grab(Node):
    def __init__(s):
        super().__init__('grab_costmap')
        s.got = {}
        s.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                              lambda m: s.got.setdefault('costmap', m), LATCHED)
        s.create_subscription(OccupancyGrid, '/comfort_mask',
                              lambda m: s.got.setdefault('mask', m), LATCHED)
        s.create_subscription(OccupancyGrid, '/map',
                              lambda m: s.got.setdefault('map', m), LATCHED)
        s.create_subscription(LaserScan, '/scan',
                              lambda m: s.got.setdefault('scan', m), qos_profile_sensor_data)
        s.buf = Buffer(); s.tfl = TransformListener(s.buf, s)

def grid(m):
    a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    return dict(data=a, res=m.info.resolution,
                ox=m.info.origin.position.x, oy=m.info.origin.position.y)

out = sys.argv[1]
rclpy.init(); g = Grab(); t0 = time.time()
while len(g.got) < 4 and time.time() - t0 < 25:
    rclpy.spin_once(g, timeout_sec=0.2)

save = {}
for k in ('costmap', 'mask', 'map'):
    if k in g.got:
        d = grid(g.got[k])
        save[k] = d['data']; save[k + '_meta'] = np.array([d['res'], d['ox'], d['oy']])
        print(f"  {k}: {d['data'].shape} res {d['res']:.3f} origin ({d['ox']:.2f},{d['oy']:.2f})"
              f" max {d['data'].max()}")
    else:
        print(f"  {k}: MISSING")
if 'scan' in g.got:
    sc = g.got['scan']; a = np.array(sc.ranges)
    v = np.isfinite(a) & (a > sc.range_min) & (a < sc.range_max)
    ang = sc.angle_min + np.arange(len(a)) * sc.angle_increment
    save['scan_xy'] = np.c_[a[v]*np.cos(ang[v]), a[v]*np.sin(ang[v])]
try:
    tr = g.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    t = tr.transform.translation; q = tr.transform.rotation
    import math
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    save['robot'] = np.array([t.x, t.y, yaw])
    print(f"  robot at ({t.x:+.2f},{t.y:+.2f}) yaw {math.degrees(yaw):+.1f} deg")
except Exception as e:
    print("  robot pose: ", e)
np.savez_compressed(out, **save)
print("wrote", out)
g.destroy_node(); rclpy.shutdown()
