#!/usr/bin/env python3
"""Accumulate a bag's LiDAR clouds into the map frame, KEEPING frame boundaries.

    bag_to_3d_t.py <bag_0.db3> <viewer_dir>

Writes cloud.bin (xyz float32, in capture order) and frames.json, which records
how many points each sweep contributed and where the robot was at the time.
Those two feed cloud_viewer/, whose timeline is a setDrawRange over one buffer:
because the points stay in capture order, "everything up to time t" is a range,
and no per-point timestamp is needed.

bag_to_3d.py next to this file does the same accumulation without the frame
boundaries, for a single static cloud.

The bag is read straight out of sqlite rather than through `ros2 bag`: the
2026-10-18 recording was killed with its tmux session and never wrote
metadata.yaml, so rosbag2 refuses to open it. session_stop.sh now sends SIGINT
instead, but older bags still need this path.
"""
import json
import math
import sqlite3
import sys

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

MAX_RANGE = 12.0          # m, beyond the L1's useful return in this corridor


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    db, outdir = sys.argv[1], sys.argv[2]

    con = sqlite3.connect(db)
    tid = {n: i for i, n in con.execute("select id,name from topics")}

    buf = Buffer(cache_time=Duration(seconds=200))
    for topic, static in (("/tf_static", True), ("/tf", False)):
        for (blob,) in con.execute(
                "select data from messages where topic_id=? order by timestamp",
                (tid[topic],)):
            for tr in deserialize_message(bytes(blob), TFMessage).transforms:
                (buf.set_transform_static if static else buf.set_transform)(tr, "bag")

    rows = con.execute(
        "select timestamp,data from messages where topic_id=? order by timestamp",
        (tid["/lidar/cloud"],))

    chunks, frames, t0 = [], [], None
    for ts, blob in rows:
        msg = deserialize_message(bytes(blob), PointCloud2)
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tr = buf.lookup_transform("map", msg.header.frame_id or "lidar_link", stamp)
            rb = buf.lookup_transform("map", "base_footprint", stamp)
        except Exception:
            continue
        t, q = tr.transform.translation, tr.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                      [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
        P = np.array([[p[0], p[1], p[2]] for p in
                      pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)],
                     dtype=np.float32)
        if not len(P):
            continue
        P = P[np.linalg.norm(P, axis=1) < MAX_RANGE]
        if not len(P):
            continue
        chunks.append((P @ R.T + np.array([t.x, t.y, t.z])).astype(np.float32))
        if t0 is None:
            t0 = ts
        rt, rq = rb.transform.translation, rb.transform.rotation
        yaw = math.atan2(2*(rq.w*rq.z + rq.x*rq.y), 1 - 2*(rq.y*rq.y + rq.z*rq.z))
        frames.append({"t": round((ts - t0) / 1e9, 3), "n": int(len(P)),
                       "x": rt.x, "y": rt.y, "yaw": round(yaw, 4)})

    if not chunks:
        print("no cloud could be placed; is /tf in the bag?")
        return 1

    A = np.vstack(chunks)
    # Centre on the cloud so the viewer's orbit pivot sits in the corridor.
    c = A.mean(0)
    c[2] = 0.0
    A -= c
    for f in frames:
        f["x"] = round(f["x"] - float(c[0]), 3)
        f["y"] = round(f["y"] - float(c[1]), 3)

    A.tofile(f"{outdir}/cloud.bin")
    json.dump({"frames": frames, "total": int(len(A)),
               "duration": frames[-1]["t"]}, open(f"{outdir}/frames.json", "w"))
    n = [f["n"] for f in frames]
    print(f"{len(frames)} sweeps, {len(A)} points, {frames[-1]['t']:.1f} s")
    print(f"points per sweep: min {min(n)} median {sorted(n)[len(n)//2]} max {max(n)}")
    print(f"wrote {outdir}/cloud.bin and frames.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
