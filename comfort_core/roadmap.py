"""The lane network, and how to get from anywhere on it to a place.

Pure Python: used by the scripted fleet driver in the simulator and by
the scenario figure on the ROS side, so it lives where both can import
it.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

Point = Tuple[float, float]


class Roadmap:
    """The lanes, and how to get from anywhere on them to a place.

    The network is a rectangle: the two aisle centrelines joined by the
    two crossings. Anything off it -- a delivery point in a refuge, a
    cart block -- hangs off a spur at right angles, so a trip is always
    "back to the lane, round, then in".
    """

    def __init__(self, spec: Dict[str, Any]):
        lanes = spec.get("lanes", {})
        self.north = float(lanes["north_y"])
        self.south = float(lanes["south_y"])
        self.west = float(lanes["west_x"])
        self.east = float(lanes["east_x"])
        # The loop, clockwise from the north-west corner. Distance
        # round it is the only coordinate the router needs.
        self.corners: List[Point] = [
            (self.west, self.north), (self.east, self.north),
            (self.east, self.south), (self.west, self.south)]
        self.edges = [math.dist(self.corners[i],
                                self.corners[(i + 1) % 4]) for i in range(4)]
        self.perimeter = sum(self.edges)

        self.places: Dict[str, Point] = {
            name: (float(p[0]), float(p[1]))
            for name, p in spec.get("places", {}).items()}

    def access(self, place: Point) -> Point:
        """Where the spur to a place leaves the lane."""
        x, y = place
        if y > self.north:
            return (x, self.north)
        if y < self.south:
            return (x, self.south)
        return place

    def _distance_round(self, point: Point) -> float:
        """How far round the loop a point on it lies."""
        best, best_s = float("inf"), 0.0
        s = 0.0
        for i in range(4):
            a, b = self.corners[i], self.corners[(i + 1) % 4]
            seg = self.edges[i]
            # Project onto this edge.
            dx, dy = b[0] - a[0], b[1] - a[1]
            t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (seg * seg)
            t = max(0.0, min(1.0, t))
            px, py = a[0] + dx * t, a[1] + dy * t
            d = math.hypot(point[0] - px, point[1] - py)
            if d < best:
                best, best_s = d, s + seg * t
            s += seg
        return best_s

    def route(self, start: Point, target_name: str) -> List[Point]:
        """Waypoints from where the AMR is to the named place.

        Off the spur, round the loop the short way, then in. The only
        thing to work out is which corners lie between the two access
        points in the chosen direction.
        """
        place = self.places[target_name]
        here = self.access(start)
        there = self.access(place)

        out: List[Point] = []
        if here != start:
            out.append(here)

        s0 = self._distance_round(here)
        s1 = self._distance_round(there)
        forward = (s1 - s0) % self.perimeter
        backward = self.perimeter - forward
        if forward <= backward:
            ahead = lambda c: (self._distance_round(c) - s0) % self.perimeter
            span = forward
        else:
            ahead = lambda c: (s0 - self._distance_round(c)) % self.perimeter
            span = backward
        between = [c for c in self.corners if 1e-6 < ahead(c) < span - 1e-6]
        out += sorted(between, key=ahead)

        out.append(there)
        if there != place:
            out.append(place)
        return out
