#!/usr/bin/env python3
"""Measure a real LiDAR from a rosbag, so the simulated one can match it.

Runs in the SYSTEM Python with system ROS 2 Jazzy. No Isaac Sim.

    source /opt/ros/jazzy/setup.bash
    ./nav3d/analyze_lidar_bag.py ~/Downloads/amr_bag

Why measure instead of reading the datasheet
--------------------------------------------
The real AMR's LiDAR is a Unitree 4D unit, but the manuals do not say
which one, and L1 and L2 differ in the one number that matters most
here -- vertical field of view, 90 vs 96 degrees. A single recorded
sweep settles it, and gives the scan pattern itself rather than a
rounded figure from a brochure. What comes out of this is what goes
into nav3d/lidar_configs/*.json.

What it reports
---------------
    topics, rates      which topic is the cloud, and how fast
    fields             x/y/z/intensity/ring/time, and the point stride
    shape              organised (height > 1) or a flat list
    vertical FOV       from the elevation of every returned point
    line structure     whether elevations cluster into rings, or are
                       scattered -- a prism scanner looks scattered,
                       which is how a Unitree differs from a Velodyne
    range              nearest and furthest return
    points per second  measured, not claimed
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path


def open_reader(path: Path):
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(path), storage_id="")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    return reader


def summarise_topics(path: Path):
    """Every topic in the bag with its type, count and rate."""
    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts: Counter = Counter()
    first: dict = {}
    last: dict = {}
    while reader.has_next():
        topic, _data, stamp = reader.read_next()
        counts[topic] += 1
        first.setdefault(topic, stamp)
        last[topic] = stamp
    rows = []
    for topic, n in sorted(counts.items(), key=lambda kv: -kv[1]):
        span = (last[topic] - first[topic]) / 1e9
        hz = (n - 1) / span if span > 0 and n > 1 else 0.0
        rows.append((topic, types.get(topic, "?"), n, span, hz))
    return rows


def cloud_topic(rows, wanted: str | None):
    if wanted:
        return wanted
    for topic, type_name, *_ in rows:
        if type_name == "sensor_msgs/msg/PointCloud2":
            return topic
    return None


def read_clouds(path: Path, topic: str, limit: int):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = open_reader(path)
    out = []
    while reader.has_next() and len(out) < limit:
        name, data, stamp = reader.read_next()
        if name == topic:
            out.append((stamp, deserialize_message(data, PointCloud2)))
    return out


def geometry(clouds):
    """Elevation, azimuth and range of every point, over several sweeps."""
    from sensor_msgs_py import point_cloud2

    elevations, ranges, total = [], [], 0
    for _stamp, msg in clouds:
        for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            d = math.sqrt(x * x + y * y + z * z)
            if d < 1e-3:
                continue
            total += 1
            elevations.append(math.degrees(math.asin(max(-1.0, min(1.0, z / d)))))
            ranges.append(d)
    return elevations, ranges, total


def ring_structure(elevations, tolerance=0.25):
    """Do the elevations fall into a few discrete rings, or spread out?

    A conventional rotating multi-line LiDAR has one elevation per laser,
    so the distinct values number in the tens. A prism scanner paints a
    different elevation almost every shot. Telling these apart decides
    how the Isaac profile has to be written.
    """
    buckets = defaultdict(int)
    for e in elevations:
        buckets[round(e / tolerance)] += 1
    return len(buckets)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="bag directory (or .mcap/.db3 file)")
    parser.add_argument("--topic", default=None,
                        help="cloud topic; the first PointCloud2 by default")
    parser.add_argument("--sweeps", type=int, default=20,
                        help="how many messages to measure geometry over")
    args = parser.parse_args()

    path = Path(args.bag).expanduser()
    if not path.exists():
        print(f"[ERROR] no bag at {path}", file=sys.stderr)
        return 1

    rows = summarise_topics(path)
    print(f"=== {path.name}")
    print(f"{'topic':<34s} {'type':<36s} {'msgs':>7s} {'s':>7s} {'Hz':>7s}")
    for topic, type_name, n, span, hz in rows:
        print(f"{topic:<34s} {type_name:<36s} {n:>7d} {span:>7.1f} {hz:>7.1f}")

    topic = cloud_topic(rows, args.topic)
    if topic is None:
        print("\nno PointCloud2 in this bag; nothing to measure")
        return 0

    clouds = read_clouds(path, topic, args.sweeps)
    if not clouds:
        print(f"\nno messages on {topic}")
        return 1

    first = clouds[0][1]
    print(f"\n=== {topic}")
    print(f"  frame_id     : {first.header.frame_id}")
    print(f"  shape        : height {first.height} x width {first.width}"
          f"   ({'organised' if first.height > 1 else 'unordered'})")
    print(f"  point_step   : {first.point_step} bytes")
    print("  fields       : " + ", ".join(f.name for f in first.fields))

    elevations, ranges, total = geometry(clouds)
    if not elevations:
        print("  no finite points")
        return 1

    span = (clouds[-1][0] - clouds[0][0]) / 1e9
    per_msg = total / len(clouds)
    print(f"\n  points/msg   : {per_msg:,.0f}")
    if span > 0:
        print(f"  points/s     : {total / span:,.0f}  (measured over "
              f"{len(clouds)} msgs, {span:.2f} s)")
    lo, hi = min(elevations), max(elevations)
    print(f"  elevation    : {lo:+.1f} deg .. {hi:+.1f} deg"
          f"   -> vertical FOV {hi - lo:.1f} deg")
    print(f"  range        : {min(ranges):.3f} m .. {max(ranges):.1f} m")
    distinct = ring_structure(elevations)
    print(f"  elevation bins (0.25 deg): {distinct}")
    if distinct > 200:
        print("    -> scattered: a prism / non-repetitive scanner, which is "
              "what a Unitree 4D unit is")
    else:
        print(f"    -> clusters into about {distinct} rings: a conventional "
              f"multi-line rotating LiDAR")

    print("\n  Unitree L1 is 360 x 90 deg, L2 is 360 x 96 deg. The vertical")
    print("  FOV above decides which one this is.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
