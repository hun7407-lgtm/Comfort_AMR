#!/usr/bin/env python3
"""Find the person in the corridor, and check they are person-shaped.

    detect_person.py --learn [seconds]    record the empty corridor
    detect_person.py [seconds]            find whoever is in it now

Prints `PERSON <x> <y>` in the map frame as the last line, so a capture
script can read it.

Three things do the work, each added because the previous attempt failed on
real data from this corridor:

1. HEIGHT BAND 0.20-2.00 m. This corridor's ceiling returns at 2.7 m, and
   without the band every cluster linked into one blob 14 m across.

2. BACKGROUND FROM THE SENSOR, not from the SLAM map. The saved map had 70
   occupied cells in the whole corridor -- slam_toolbox marks sparsely when
   the robot is stationary -- so subtracting it left the walls standing and
   they came back as "people" 0.1 m wide. The background here is an occupancy
   histogram of the empty corridor built from the cloud itself, same sensor,
   same geometry.

3. SHAPE, both ways. A person is a column AT LEAST 0.20 m across and AT MOST
   0.90 m, reaching past 0.90 m with at least 0.50 m of vertical extent. The
   lower width bound is what rejects the wall slivers; the upper one rejects
   the desk, which is wide and stops low. This LiDAR looks upward from 0.31 m,
   so that vertical profile is the one cue that separates a person from
   furniture -- and flattening the cloud into /scan destroys it, which is why
   this reads the cloud directly.
"""
import math, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

LIDAR_Z    = 0.31
Z_MIN, Z_MAX = 0.20, 2.00
MIN_TOP    = 0.90     # m above floor the column must reach
MIN_EXTENT = 0.50     # m of vertical extent
MIN_SPREAD = 0.20     # m: narrower than this is a wall sliver
MAX_SPREAD = 0.90     # m: wider than this is furniture
MIN_PTS    = 25
GAP        = 0.35
BG_RES     = 0.10     # m, background histogram cell
BG_PAD     = 2        # cells of slack
BG_FILE    = "/home/clabil/corridor_background.npz"


class Grab(Node):
    def __init__(s):
        super().__init__('detect_person')
        s.clouds = []
        s.create_subscription(PointCloud2, '/lidar/cloud',
                              lambda m: s.clouds.append(m), qos_profile_sensor_data)
        s.buf = Buffer(); s.tfl = TransformListener(s.buf, s)


def collect(secs):
    rclpy.init(); g = Grab(); t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(g, timeout_sec=0.1)
    if not g.clouds:
        raise SystemExit("no cloud")
    P = np.vstack([np.array([[p[0], p[1], p[2]] for p in
                   pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)])
                   for m in g.clouds if m.width])
    P = P[np.hypot(P[:, 0], P[:, 1]) < 8.0]
    tr = g.buf.lookup_transform('map', 'lidar_link', rclpy.time.Time(),
                                timeout=rclpy.duration.Duration(seconds=3.0))
    t = tr.transform.translation; q = tr.transform.rotation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s_ = math.cos(yaw), math.sin(yaw)
    X = t.x + P[:, 0]*c - P[:, 1]*s_
    Y = t.y + P[:, 0]*s_ + P[:, 1]*c
    Z = P[:, 2] + LIDAR_Z
    band = (Z >= Z_MIN) & (Z <= Z_MAX)
    n = len(g.clouds)
    g.destroy_node(); rclpy.shutdown()
    return np.c_[X[band], Y[band], Z[band]], n, len(P)


def cells(Q, res):
    return np.floor(Q[:, 0]/res).astype(int), np.floor(Q[:, 1]/res).astype(int)


def cluster(xy, gap):
    n = len(xy); lab = -np.ones(n, int); c = 0
    for i in range(n):
        if lab[i] >= 0: continue
        stack = [i]; lab[i] = c
        while stack:
            j = stack.pop()
            d = np.hypot(xy[:, 0]-xy[j, 0], xy[:, 1]-xy[j, 1])
            for k in np.where((d < gap) & (lab < 0))[0]:
                lab[k] = c; stack.append(k)
        c += 1
    return lab, c


def main():
    argv = sys.argv[1:]
    learn = "--learn" in argv
    argv = [a for a in argv if a != "--learn"]
    secs = float(argv[0]) if argv else 4.0

    Q, nclouds, npts = collect(secs)
    print(f"{nclouds} clouds, {npts} pts, {len(Q)} in {Z_MIN}-{Z_MAX} m band")

    if learn:
        cx, cy = cells(Q, BG_RES)
        np.savez_compressed(BG_FILE, cells=np.unique(np.c_[cx, cy], axis=0),
                            res=BG_RES)
        print(f"background: {len(np.unique(np.c_[cx, cy], axis=0))} cells -> {BG_FILE}")
        return 0

    bg = np.load(BG_FILE)
    bgset = {(int(a), int(b)) for a, b in bg["cells"]}
    cx, cy = cells(Q, BG_RES)
    known = np.array([any((int(a)+dx, int(b)+dy) in bgset
                          for dx in range(-BG_PAD, BG_PAD+1)
                          for dy in range(-BG_PAD, BG_PAD+1))
                      for a, b in zip(cx, cy)])
    R = Q[~known]
    print(f"{known.sum()} on background, {len(R)} left")
    if len(R) < MIN_PTS:
        print("\nno person-shaped cluster"); return 2

    lab, n = cluster(R[:, :2], GAP)
    cands = []
    for c in range(n):
        C = R[lab == c]
        if len(C) < MIN_PTS: continue
        top = float(C[:, 2].max()); bot = float(C[:, 2].min())
        spread = float(max(C[:, 0].ptp(), C[:, 1].ptp()))
        ok = bool(top >= MIN_TOP and (top - bot) >= MIN_EXTENT
                  and MIN_SPREAD <= spread <= MAX_SPREAD)
        cands.append((ok, len(C), float(C[:, 0].mean()), float(C[:, 1].mean()),
                      top, top-bot, spread))
    cands.sort(key=lambda r: (not r[0], -r[1]))
    print(f"{'person?':8s} {'pts':>5s} {'x':>7s} {'y':>7s} {'top':>6s} {'vext':>6s} {'wide':>6s}")
    for ok, k, x, y, top, ext, sp in cands[:8]:
        print(f"{'YES' if ok else 'no':8s} {k:5d} {x:+7.2f} {y:+7.2f} {top:6.2f} {ext:6.2f} {sp:6.2f}")
    hits = [c for c in cands if c[0]]
    if not hits:
        print("\nno person-shaped cluster"); return 2
    print(f"\nPERSON {hits[0][2]:+.2f} {hits[0][3]:+.2f}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
