#!/usr/bin/env python3
"""Track the person while they walk, and measure how fast.

    track_person.py [seconds] [window]

Prints one line per window and finally a speed, then saves the track.

Window length is a real constraint, not a tuning knob. This LiDAR is
non-repetitive and sparse: measured on this corridor, a standing person
returns about 2 points per sweep at 5 m and 3 at 3 m, so a 0.3 s window holds
too few points to cluster. A 1.0 s window holds roughly 10-30, which clusters
reliably -- at the cost of smearing a walker by however far they move in that
second. Walk slowly and the smear is small.
"""
import math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

LIDAR_Z = 0.31
Z_MIN, Z_MAX = 0.20, 2.00
MIN_TOP, MIN_EXTENT = 0.80, 0.40
MIN_SPREAD, MAX_SPREAD = 0.15, 1.10
MIN_PTS = 8
GAP = 0.45
BG_RES, BG_PAD = 0.10, 2
BG_FILE = "/home/clabil/corridor_background.npz"
OUT = "/tmp/track.npz"


class G(Node):
    def __init__(s):
        super().__init__('track_person'); s.buf_pts = []
        s.create_subscription(PointCloud2, '/lidar/cloud', s.cb, qos_profile_sensor_data)
        s.tfbuf = Buffer(); s.tfl = TransformListener(s.tfbuf, s)
        s.tf = None
    def cb(s, m):
        s.buf_pts.append((time.time(), m))


def cluster(xy, gap):
    n = len(xy); lab = -np.ones(n, int); c = 0
    for i in range(n):
        if lab[i] >= 0: continue
        st = [i]; lab[i] = c
        while st:
            j = st.pop()
            d = np.hypot(xy[:,0]-xy[j,0], xy[:,1]-xy[j,1])
            for k in np.where((d < gap) & (lab < 0))[0]:
                lab[k] = c; st.append(k)
        c += 1
    return lab, c


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    win  = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    bg = np.load(BG_FILE)
    bgset = {(int(a), int(b)) for a, b in bg["cells"]}

    rclpy.init(); g = G()
    t0 = time.time()
    while g.tf is None and time.time() - t0 < 5:
        rclpy.spin_once(g, timeout_sec=0.1)
        try:
            tr = g.tfbuf.lookup_transform('map', 'lidar_link', rclpy.time.Time())
            t = tr.transform.translation; q = tr.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            g.tf = (t.x, t.y, yaw)
        except Exception:
            pass
    if g.tf is None:
        print("no map TF"); return 1
    tx, ty, yaw = g.tf
    cs, sn = math.cos(yaw), math.sin(yaw)

    track = []
    t0 = time.time()
    print(f"{'t(s)':>6s} {'pts':>4s} {'x':>7s} {'y':>7s} {'top':>5s} {'wide':>5s}")
    while time.time() - t0 < secs:
        g.buf_pts.clear()
        tw = time.time()
        while time.time() - tw < win:
            rclpy.spin_once(g, timeout_sec=0.05)
        if not g.buf_pts:
            continue
        arrs = []
        for _, m in g.buf_pts:
            if not m.width: continue
            arrs.append(np.array([[p[0],p[1],p[2]] for p in
                        pc2.read_points(m, field_names=('x','y','z'), skip_nans=True)]))
        if not arrs: continue
        P = np.vstack(arrs); P = P[np.hypot(P[:,0],P[:,1]) < 8.0]
        X = tx + P[:,0]*cs - P[:,1]*sn
        Y = ty + P[:,0]*sn + P[:,1]*cs
        Z = P[:,2] + LIDAR_Z
        band = (Z >= Z_MIN) & (Z <= Z_MAX)
        Q = np.c_[X[band], Y[band], Z[band]]
        cx = np.floor(Q[:,0]/BG_RES).astype(int); cy = np.floor(Q[:,1]/BG_RES).astype(int)
        known = np.array([any((int(a)+dx, int(b)+dy) in bgset
                              for dx in range(-BG_PAD, BG_PAD+1)
                              for dy in range(-BG_PAD, BG_PAD+1))
                          for a, b in zip(cx, cy)]) if len(Q) else np.zeros(0, bool)
        R = Q[~known] if len(Q) else Q
        now = time.time() - t0
        if len(R) < MIN_PTS:
            print(f"{now:6.1f} {len(R):4d}       -       -     -     -")
            continue
        lab, n = cluster(R[:,:2], GAP)
        best = None
        for c in range(n):
            C = R[lab == c]
            if len(C) < MIN_PTS: continue
            top = float(C[:,2].max()); ext = top - float(C[:,2].min())
            sp = float(max(C[:,0].ptp(), C[:,1].ptp()))
            if top >= MIN_TOP and ext >= MIN_EXTENT and MIN_SPREAD <= sp <= MAX_SPREAD:
                if best is None or len(C) > best[0]:
                    best = (len(C), float(C[:,0].mean()), float(C[:,1].mean()), top, sp)
        if best is None:
            print(f"{now:6.1f} {len(R):4d}       -       -     -     -")
            continue
        k, x, y, top, sp = best
        track.append((now, x, y))
        print(f"{now:6.1f} {k:4d} {x:+7.2f} {y:+7.2f} {top:5.2f} {sp:5.2f}")

    g.destroy_node(); rclpy.shutdown()
    if len(track) < 2:
        print("\ntoo few fixes to measure speed"); return 2
    T = np.array(track)
    d = np.hypot(np.diff(T[:,1]), np.diff(T[:,2]))
    dt = np.diff(T[:,0])
    v = d/dt
    np.savez(OUT, track=T)
    print(f"\n{len(T)} fixes over {T[-1,0]-T[0,0]:.1f} s")
    print(f"path {d.sum():.2f} m, net {math.hypot(T[-1,1]-T[0,1], T[-1,2]-T[0,2]):.2f} m")
    print(f"speed per step: " + " ".join(f"{x:.2f}" for x in v))
    print(f"MEAN_SPEED {d.sum()/(T[-1,0]-T[0,0]):.2f}")
    mid = T[len(T)//2]
    print(f"MIDPOINT {mid[1]:+.2f} {mid[2]:+.2f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
