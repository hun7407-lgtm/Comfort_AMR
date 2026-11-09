"""Comfort Space shapes: circles, ellipses, and offset ellipses.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

Three cases, one representation.

A circle is an ellipse with equal semi-axes. A walking person's space is
longer along their heading than across it, because that is where they
are about to be. And neither of those can express a space that is not
the same in front of a person as behind them, which is what the
literature on being passed by a robot reports: for someone standing,
being passed behind is rated less comfortable than being passed in
front.

So the ellipse may be pushed off the person along their heading.
`center_offset_along` is that push, measured in the heading direction:
negative puts the ellipse behind the person (more room at their back),
positive puts it in front (more room ahead of a walker). With the offset
at zero this is exactly the centred ellipse it replaces.

Given the extent wanted in front, to the side and behind:

    along  = (front + rear) / 2
    across = side
    offset = (front - rear) / 2

`across` is the half-width at the ELLIPSE's centre, not at the person.
At the person it is `across * sqrt(1 - (offset/along)^2)`, slightly less
-- 0.892 m where `across` is 0.90 and the offset 0.125 m. The difference
is 8 mm and is documented rather than corrected, because `across` as the
maximum half-width is the quantity the planner and the costmap care
about.

> Literature-anchored engineering parameters.
> Not experimentally validated human comfort distances, and not safety
> distances: the physical safety layer is sized separately and is never
> reduced along with these.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class ComfortShape:
    """One person's Comfort Space.

    `x`, `y` is where the PERSON is, which is not in general the centre
    of the ellipse: see `center_offset_along` and `centre`. `along` is
    the semi-axis in the heading direction, `across` the one
    perpendicular to it, and `heading` is measured from the +x axis.
    """
    key: str
    x: float
    y: float
    along: float
    across: float
    heading: float = 0.0
    # How far the ellipse sits from the person along their heading.
    # Last, with a default, so every existing positional construction
    # keeps building the centred shape it used to.
    center_offset_along: float = 0.0

    @property
    def centre(self) -> Tuple[float, float]:
        """Where the ellipse itself is centred."""
        if not self.center_offset_along:
            return self.x, self.y
        return (self.x + self.center_offset_along * math.cos(self.heading),
                self.y + self.center_offset_along * math.sin(self.heading))

    @property
    def extents(self) -> Tuple[float, float, float]:
        """(front, side, rear) measured from the person."""
        d = self.center_offset_along
        return self.along + d, self.across, self.along - d

    @property
    def is_circle(self) -> bool:
        return abs(self.along - self.across) < 1e-9

    @property
    def bounding_radius(self) -> float:
        """Farthest the shape reaches from the PERSON, any direction.

        Callers use this to size a raster window or a first cheap
        distance test, so it has to bound the offset ellipse from the
        person's position, not from the ellipse's own centre.
        """
        d = abs(self.center_offset_along)
        if not d:
            return max(self.along, self.across)
        return max(self.along + d, math.hypot(d, self.across))

    def contains(self, px: float, py: float, margin: float = 0.0) -> bool:
        """Is (px, py) inside, allowing an extra `margin` all round?"""
        a = self.along + margin
        b = self.across + margin
        if a <= 0.0 or b <= 0.0:
            return False
        c, s = math.cos(self.heading), math.sin(self.heading)
        cx, cy = self.centre
        dx, dy = px - cx, py - cy
        u = dx * c + dy * s
        v = -dx * s + dy * c
        return (u / a) ** 2 + (v / b) ** 2 <= 1.0

    def grown(self, amount: float) -> "ComfortShape":
        """The same shape with both semi-axes enlarged.

        Used to turn the radius a person experiences into the radius a
        planner working on the robot's centre point has to avoid.

        The offset is deliberately NOT scaled. Adding the same amount in
        front and behind moves neither end's share of the centre, so
        front, side and rear each grow by `amount` and the asymmetry the
        policy asked for is preserved exactly.
        """
        return ComfortShape(self.key, self.x, self.y,
                            self.along + amount, self.across + amount,
                            self.heading, self.center_offset_along)

    def y_span_at(self, x: float) -> Optional[Tuple[float, float]]:
        """The band of y the shape covers at this x, or None if it misses.

        Solving for the intersection rather than sampling keeps the
        passage calculation exact, which matters because the whole
        B2/B3 result turns on whether a gap is a few centimetres wide or
        closed.
        """
        a, b = self.along, self.across
        if a <= 0.0 or b <= 0.0:
            return None

        c, s = math.cos(self.heading), math.sin(self.heading)
        cx, cy = self.centre
        dx = x - cx

        # Substituting the rotated ellipse equation and collecting in
        # t = y - self.y gives a quadratic A t^2 + B t + C = 0.
        A = (s * s) / (a * a) + (c * c) / (b * b)
        B = 2.0 * dx * c * s * (1.0 / (a * a) - 1.0 / (b * b))
        C = dx * dx * ((c * c) / (a * a) + (s * s) / (b * b)) - 1.0

        if abs(A) < 1e-12:
            return None
        disc = B * B - 4.0 * A * C
        if disc <= 0.0:
            return None
        root = math.sqrt(disc)
        lo = (-B - root) / (2.0 * A)
        hi = (-B + root) / (2.0 * A)
        return (cy + min(lo, hi), cy + max(lo, hi))


def circle(key: str, x: float, y: float, radius: float) -> ComfortShape:
    return ComfortShape(key, x, y, radius, radius, 0.0, 0.0)


def from_extents(key: str, x: float, y: float, heading: float,
                 front: float, side: float, rear: float) -> ComfortShape:
    """Build the offset ellipse that has these extents from the person."""
    return ComfortShape(key, x, y,
                        (front + rear) / 2.0, side, heading,
                        (front - rear) / 2.0)


def blocked_spans_at(shapes: List[ComfortShape], x: float) -> List[Tuple[float, float]]:
    """Every y band covered at this x, sorted."""
    spans = []
    for shape in shapes:
        span = shape.y_span_at(x)
        if span is not None:
            spans.append(span)
    spans.sort()
    return spans
