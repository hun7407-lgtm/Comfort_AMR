#!/usr/bin/env python3
"""Dump one accumulated /scan as a numpy file. Run on the AMR."""
import numpy as np, rclpy, time, sys
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
class G(Node):
    def __init__(s):
        super().__init__('scan_png'); s.m=[]
        s.create_subscription(LaserScan,'/scan',lambda x: s.m.append(x), qos_profile_sensor_data)
rclpy.init(); g=G(); t0=time.time()
while len(g.m)<int(sys.argv[1]) and time.time()-t0<30: rclpy.spin_once(g,timeout_sec=0.2)
out=[]
for sc in g.m:
    a=np.array(sc.ranges); v=np.isfinite(a)&(a>sc.range_min)&(a<sc.range_max)
    ang=sc.angle_min+np.arange(len(a))*sc.angle_increment
    out.append(np.c_[a[v]*np.cos(ang[v]), a[v]*np.sin(ang[v])])
pts=np.vstack(out); np.save('/tmp/scan_pts.npy', pts)
print('msgs',len(g.m),'points',len(pts))
g.destroy_node(); rclpy.shutdown()
