"""Comfort Space shapes: circles and forward-stretched ellipses.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

A standing person's comfort zone is reasonably modelled as a circle. A
walking one's is not: the space they need kept clear is longer in the
direction they are heading than to either side, because that is where
they are about to be. The ellipse here is the smallest change that
captures it, with the long axis along the direction of travel.

A circle is just an ellipse with both semi-axes equal, so everything
downstream handles one case.

> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class ComfortShape:
    """One person's Comfort Space.

    `along` is the semi-axis in the heading direction, `across` the one
    perpendicular to it, and `heading` is measured from the +x axis.
    """
    key: str
    x: float
    y: float
    along: float
    across: float
    heading: float = 0.0

    @property
    def is_circle(self) -> bool:
        return abs(self.along - self.across) < 1e-9

    @property
    def bounding_radius(self) -> float:
        return max(self.along, self.across)

    def contains(self, px: float, py: float, margin: float = 0.0) -> bool:
        """Is (px, py) inside, allowing an extra `margin` all round?"""
        a = self.along + margin
        b = self.across + margin
        if a <= 0.0 or b <= 0.0:
            return False
        c, s = math.cos(self.heading), math.sin(self.heading)
        dx, dy = px - self.x, py - self.y
        u = dx * c + dy * s
        v = -dx * s + dy * c
        return (u / a) ** 2 + (v / b) ** 2 <= 1.0

    def grown(self, amount: float) -> "ComfortShape":
        """The same shape with both semi-axes enlarged.

        Used to turn the radius a person experiences into the radius a
        planner working on the robot's centre point has to avoid.
        """
        return ComfortShape(self.key, self.x, self.y,
                            self.along + amount, self.across + amount,
                            self.heading)

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
        dx = x - self.x

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
        return (self.y + min(lo, hi), self.y + max(lo, hi))


def circle(key: str, x: float, y: float, radius: float) -> ComfortShape:
    return ComfortShape(key, x, y, radius, radius, 0.0)


def blocked_spans_at(shapes: List[ComfortShape], x: float) -> List[Tuple[float, float]]:
    """Every y band covered at this x, sorted."""
    spans = []
    for shape in shapes:
        span = shape.y_span_at(x)
        if span is not None:
            spans.append(span)
    spans.sort()
    return spans
