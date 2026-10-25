#!/usr/bin/env python3
"""Accumulate a bag's LiDAR clouds into one 3D point cloud in the map frame.

    bag_to_3d.py <bag.db3> <out.npz>

slam_toolbox only ever makes a 2D occupancy grid. The 3D map is the raw cloud
replayed along the trajectory the same session produced, which is why the bag
has to carry /tf (map->odom from SLAM, odom->base from the wheels) and
/tf_static (base->lidar from the URDF) together with /lidar/cloud.

The bag is read straight out of sqlite because the recording was killed with
its tmux session and never wrote metadata.yaml, so `ros2 bag` will not open it.
"""
import sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer

LIDAR_FRAME = "lidar_link"
MAP_FRAME = "map"

db, out = sys.argv[1], sys.argv[2]
stride = int(sys.argv[3]) if len(sys.argv) > 3 else 1

con = sqlite3.connect(db)
tid = {n: i for i, n in con.execute("select id,name from topics")}

buf = Buffer(cache_time=Duration(seconds=200))
n_static = n_tf = 0
for topic, auth in (("/tf_static", True), ("/tf", False)):
    for (blob,) in con.execute("select data from messages where topic_id=? order by timestamp",
                               (tid[topic],)):
        msg = deserialize_message(bytes(blob), TFMessage)
        for tr in msg.transforms:
            if auth:
                buf.set_transform_static(tr, "bag"); n_static += 1
            else:
                buf.set_transform(tr, "bag"); n_tf += 1
print(f"tf: {n_static} static, {n_tf} dynamic")

rows = list(con.execute("select timestamp,data from messages where topic_id=? order by timestamp",
                        (tid["/lidar/cloud"],)))
print(f"clouds: {len(rows)}, using every {stride}")

chunks = []
ok = skipped = 0
for i, (ts, blob) in enumerate(rows):
    if i % stride:
        continue
    msg = deserialize_message(bytes(blob), PointCloud2)
    stamp = Time.from_msg(msg.header.stamp)
    try:
        tr = buf.lookup_transform(MAP_FRAME, msg.header.frame_id or LIDAR_FRAME, stamp)
    except Exception:
        skipped += 1
        continue
    t = tr.transform.translation
    q = tr.transform.rotation
    # quaternion -> rotation matrix
    x, y, z, wq = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*wq),   2*(x*z+y*wq)],
        [2*(x*y+z*wq),  1-2*(x*x+z*z),  2*(y*z-x*wq)],
        [2*(x*z-y*wq),  2*(y*z+x*wq),   1-2*(x*x+y*y)]])
    P = np.array([[p[0], p[1], p[2]] for p in
                  pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)],
                 dtype=np.float32)
    if not len(P):
        continue
    P = P[np.linalg.norm(P, axis=1) < 12.0]
    chunks.append((P @ R.T + np.array([t.x, t.y, t.z])).astype(np.float32))
    ok += 1

A = np.vstack(chunks)
print(f"placed {ok} clouds ({skipped} had no transform), {len(A)} points")
print(f"extent  x {A[:,0].min():+.2f}..{A[:,0].max():+.2f}"
      f"  y {A[:,1].min():+.2f}..{A[:,1].max():+.2f}"
      f"  z {A[:,2].min():+.2f}..{A[:,2].max():+.2f}")
np.savez_compressed(out, xyz=A)
print("wrote", out)
