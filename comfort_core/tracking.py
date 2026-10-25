"""Find people in a LiDAR scan and estimate how fast they are walking.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

This is the sensor side of the seam described in
comfort_core/human_state.py. Ground truth still drives the B2/B3
comparison; this estimates the same quantities from the scan so the two
can be compared, and so a Comfort Space could eventually be sized from
what the robot sees rather than from what the scene knows.

The pipeline is the usual one for a planar scan:

    ordered returns -> split where the range jumps -> clusters
    clusters -> matched to existing tracks by nearest centre
    track centres -> velocity, smoothed over a short window

Splitting on consecutive beams costs one pass, which matters because
this has to keep up with a 10 Hz scan of several hundred returns.

> Cluster sizes and gates are engineering parameters for prototype
> validation, not validated pedestrian detection thresholds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class TrackingParams:
    cluster_gap: float          # split a cluster where consecutive returns are this far apart
    min_points: int             # clusters with fewer returns are noise
    min_extent: float           # a person is at least this wide
    max_extent: float           # and no wider; walls and stations are longer
    gate: float                 # how far a cluster may move between scans and still be the same
    velocity_window: float      # seconds of history the velocity is taken over
    confirm_hits: int           # matched this many times before a track is reported
    max_misses: int             # dropped after this many scans without a match
    merge_distance: float       # clusters closer than this are one person (two legs)

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "TrackingParams":
        p = cfg.get("perception", {}).get("tracking", {})
        return cls(
            cluster_gap=float(p.get("cluster_gap", 0.25)),
            min_points=int(p.get("min_points", 2)),
            min_extent=float(p.get("min_extent", 0.05)),
            max_extent=float(p.get("max_extent", 1.20)),
            gate=float(p.get("gate", 0.80)),
            velocity_window=float(p.get("velocity_window", 1.0)),
            confirm_hits=int(p.get("confirm_hits", 3)),
            max_misses=int(p.get("max_misses", 5)),
            merge_distance=float(p.get("merge_distance", 0.45)),
        )


@dataclass
class Cluster:
    points: List[Point]

    @property
    def centre(self) -> Point:
        n = len(self.points)
        return (sum(p[0] for p in self.points) / n, sum(p[1] for p in self.points) / n)

    @property
    def extent(self) -> float:
        """Distance between the two end points, which for a scan is the width seen."""
        (x0, y0), (x1, y1) = self.points[0], self.points[-1]
        return math.hypot(x1 - x0, y1 - y0)


def cluster_scan(points: Sequence[Point], p: TrackingParams) -> List[Cluster]:
    """Split ordered returns wherever the gap between neighbours is large.

    The returns arrive in beam order, so a person standing clear of the
    wall shows up as a run of neighbouring returns with big gaps either
    side. Anything too small (noise) or too wide (a wall) is dropped.
    """
    clusters: List[Cluster] = []
    current: List[Point] = []
    for i, pt in enumerate(points):
        if current and math.hypot(pt[0] - current[-1][0], pt[1] - current[-1][1]) > p.cluster_gap:
            clusters.append(Cluster(current))
            current = []
        current.append(pt)
    if current:
        clusters.append(Cluster(current))

    kept = [c for c in clusters
            if len(c.points) >= p.min_points and p.min_extent <= c.extent <= p.max_extent]
    return merge_clusters(kept, p)


def merge_clusters(clusters: List[Cluster], p: TrackingParams) -> List[Cluster]:
    """Join clusters that are close enough to be one person.

    A walking person gives one cluster per leg, and the gap between the
    legs is wider than the gap within one. Left apart, each leg became its
    own track and the same person was reported twice, with the centre
    jumping between them: standing people showed 0.5 m/s.
    """
    merged: List[Cluster] = []
    for cluster in clusters:
        cx, cy = cluster.centre
        for other in merged:
            ox, oy = other.centre
            if math.hypot(cx - ox, cy - oy) <= p.merge_distance:
                other.points = other.points + cluster.points
                break
        else:
            merged.append(Cluster(list(cluster.points)))
    return merged


@dataclass
class Track:
    id: int
    x: float
    y: float
    last_seen: float
    history: List[Tuple[float, float, float]] = field(default_factory=list)  # (t, x, y)
    hits: int = 1
    misses: int = 0

    @property
    def confirmed_speed(self) -> Tuple[float, float]:
        """(vx, vy) over the history held, zero until there are two samples."""
        if len(self.history) < 2:
            return 0.0, 0.0
        t0, x0, y0 = self.history[0]
        t1, x1, y1 = self.history[-1]
        dt = t1 - t0
        if dt <= 1e-3:
            return 0.0, 0.0
        return (x1 - x0) / dt, (y1 - y0) / dt

    @property
    def speed(self) -> float:
        vx, vy = self.confirmed_speed
        return math.hypot(vx, vy)

    @property
    def heading(self) -> float:
        vx, vy = self.confirmed_speed
        return math.atan2(vy, vx)


class PeopleTracker:
    """Clusters each scan and follows the clusters from scan to scan."""

    def __init__(self, params: TrackingParams):
        self.p = params
        self.tracks: List[Track] = []
        self._next_id = 1

    def update(self, points: Sequence[Point], now: float) -> List[Track]:
        """Feed one scan, in world coordinates and beam order. Returns
        the tracks seen often enough to be believed.

        Clusters are matched to the track's last seen position, not to
        where its velocity says it should be by now. Predicting first was
        tried against the walker scenario and was worse on almost
        everything: wrong tracks 1 -> 12, worst position error 0.42 ->
        0.87 m, worst speed error 1.03 -> 1.41 m/s.
        """
        clusters = cluster_scan(points, self.p)
        return self.update_detections([c.centre for c in clusters], now)

    def update_detections(self, centres: Sequence[Point], now: float) -> List[Track]:
        """Feed positions that have already been found, one per person.

        The 3D detector clusters the cloud itself and tests the shape of
        each cluster before it reports anything, so its output is one
        position per person and must not be clustered again: a single
        position is one point with no extent, and `cluster_scan` drops
        it for having too few returns and too little extent. Measured --
        the detector reported two correctly shaped people per frame and
        the tracker reported nobody.
        """
        centres = list(centres)
        claimed = set()

        for track in self.tracks:
            best, best_d = None, self.p.gate
            for i, (cx, cy) in enumerate(centres):
                if i in claimed:
                    continue
                d = math.hypot(cx - track.x, cy - track.y)
                if d < best_d:
                    best, best_d = i, d
            if best is None:
                track.misses += 1
                continue
            claimed.add(best)
            track.x, track.y = centres[best]
            track.last_seen = now
            track.hits += 1
            track.misses = 0
            track.history.append((now, track.x, track.y))
            while len(track.history) > 2 and now - track.history[1][0] >= self.p.velocity_window:
                track.history.pop(0)

        for i, (cx, cy) in enumerate(centres):
            if i in claimed:
                continue
            self.tracks.append(Track(self._next_id, cx, cy, now, [(now, cx, cy)]))
            self._next_id += 1

        self.tracks = [t for t in self.tracks if t.misses <= self.p.max_misses]
        return [t for t in self.tracks if t.hits >= self.p.confirm_hits and t.misses == 0]
