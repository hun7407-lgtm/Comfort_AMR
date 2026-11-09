#!/usr/bin/env python3
"""Check the offset-ellipse Comfort Space, without a simulator.

    python3 scripts/check_comfort_shapes.py

Pure Python: no ROS, no Isaac Sim, no network. Seventeen checks in three
groups -- the geometry, the policies, and where a heading comes from.

The point of the geometry group is that `front`, `side` and `rear` come
back out of the shape exactly as the policy asked for them, including
after the planner grow, because the whole B2/B3 result turns on whether
a gap is a few centimetres wide or closed.

The point of the policy group is that B1, B2 and B3-R are untouched. A
new policy that quietly moved an old one would make every recorded batch
mean something else.

Exit code 0 means every check passed.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config                                 # noqa: E402
from comfort_core.comfort_space import (Extent, HeadingResolver,      # noqa: E402
                                        make_policy, robot_grow,
                                        shapes as build_shapes)
from comfort_core.shapes import ComfortShape, circle, from_extents    # noqa: E402

TOL = 1e-9
failures: list[str] = []
checks = 0


def near(label: str, got: float, want: float, tol: float = TOL) -> None:
    global checks
    checks += 1
    if abs(got - want) > tol:
        failures.append(f"{label}: got {got:.6f}, want {want:.6f}")


def true(label: str, value: bool) -> None:
    global checks
    checks += 1
    if not value:
        failures.append(label)


def group(n: int, title: str) -> None:
    print(f"\n{n:2d}. {title}")


def cfg_for(scenario: str):
    return load_config(scenario=scenario,
                       common_path=str(PROJECT_ROOT / "configs" / "scenarios"
                                       / "paper_h8_a1_scr_s0001.yaml"))


# ---------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------
group(1, "a zero offset is the shape that was there before")
old = ComfortShape("k", 1.0, 2.0, 1.5, 0.8, 0.3)
new = ComfortShape("k", 1.0, 2.0, 1.5, 0.8, 0.3, 0.0)
true("constructing without an offset differs from constructing with 0.0",
     old == new)
true("a five-argument shape is not centred on its person",
     old.centre == (1.0, 2.0))
near("an unoffset bounding radius is the larger semi-axis",
     old.bounding_radius, 1.5)

group(2, "stationary F0.80 S0.90 R1.05, facing +x")
st = from_extents("S", 0.0, 0.0, 0.0, 0.80, 0.90, 1.05)
near("along", st.along, 0.925)
near("across", st.across, 0.90)
near("offset", st.center_offset_along, -0.125)
near("front extent", st.extents[0], 0.80)
near("side extent", st.extents[1], 0.90)
near("rear extent", st.extents[2], 1.05)
near("the ellipse centre sits behind the person",
     st.centre[0], -0.125)
# The half-width AT the person is slightly less than `side`, which is
# the half-width at the ellipse's own centre. 8 mm here. Documented in
# comfort_core/shapes.py rather than corrected, because `side` as the
# maximum is the figure the costmap and the planner care about.
near("half-width measured at the person",
     st.y_span_at(0.0)[1], 0.90 * math.sqrt(1.0 - (0.125 / 0.925) ** 2))
near("half-width measured at the ellipse centre",
     st.y_span_at(-0.125)[1], 0.90)
near("bounding radius reaches the rear extent", st.bounding_radius, 1.05)

group(3, "rotating the shape rotates its extents exactly")
for deg in (0.0, 90.0, 180.0, -37.0):
    h = math.radians(deg)
    r = from_extents("S", 3.0, -2.0, h, 0.80, 0.90, 1.05)
    fx, fy = 3.0 + 0.80 * math.cos(h), -2.0 + 0.80 * math.sin(h)
    rx, ry = 3.0 - 1.05 * math.cos(h), -2.0 - 1.05 * math.sin(h)
    true(f"front boundary at {deg:+.0f} deg",
         r.contains(fx - 1e-4 * math.cos(h), fy - 1e-4 * math.sin(h))
         and not r.contains(fx + 1e-3 * math.cos(h), fy + 1e-3 * math.sin(h)))
    true(f"rear boundary at {deg:+.0f} deg",
         r.contains(rx + 1e-4 * math.cos(h), ry + 1e-4 * math.sin(h))
         and not r.contains(rx - 1e-3 * math.cos(h), ry - 1e-3 * math.sin(h)))

group(4, "contains() agrees with the boundary it claims")
true("the person is inside their own space", st.contains(0.0, 0.0))
true("a point just past the front is outside", not st.contains(0.81, 0.0))
true("a point just past the rear is outside", not st.contains(-1.06, 0.0))
true("a margin pushes the boundary out",
     st.contains(0.81, 0.0, margin=0.05))

group(5, "y_span_at() is exact for an offset shape")
# Sample the analytic span against a brute-force containment scan.
worst = 0.0
for i in range(-30, 31):
    x = i * 0.05
    span = st.y_span_at(x)
    hits = [j * 0.001 for j in range(-1500, 1501)
            if st.contains(x, j * 0.001)]
    if not hits:
        true(f"no span claimed at x={x:+.2f} where nothing is inside",
             span is None)
        continue
    width = max(hits) - min(hits)
    if span is None:
        # At the front and rear extremes the ellipse is tangent to the
        # line and the band has no width. y_span_at solves a quadratic
        # and reports nothing when the discriminant vanishes, which is
        # the behaviour it has always had; a zero-width band paints no
        # costmap cell and closes no passage, so it is left alone.
        true(f"only a tangent is missing at x={x:+.2f}", width <= 2e-3)
        continue
    true(f"a span is claimed at x={x:+.2f}", span is not None)
    worst = max(worst, abs(span[0] - min(hits)), abs(span[1] - max(hits)))
near("analytic span matches a 1 mm containment scan", worst, 0.0, 1.5e-3)

group(6, "grown() adds the same amount to all three extents")
g = 0.24
gr = st.grown(g)
near("front + grow", gr.extents[0], 0.80 + g)
near("side + grow", gr.extents[1], 0.90 + g)
near("rear + grow", gr.extents[2], 1.05 + g)
near("the offset is untouched",
     gr.center_offset_along, st.center_offset_along)
near("growing then shrinking returns the original front",
     gr.grown(-g).extents[0], 0.80)

# ---------------------------------------------------------------
# Policies
# ---------------------------------------------------------------
states = {"N1": "station_engaged", "E1": "stationary", "S2": "moving"}
speeds = {"S2": 1.0}

group(7, "B1 asks for no space at all, as before")
p1 = make_policy(cfg_for("b1"))
true("name", p1.name == "none")
true("gives_space() is False", p1.gives_space() is False)
true("every axis is zero",
     all(v == (0.0, 0.0, 0.0) for v in p1.axes(states, speeds).values()))
true("no shape is built for anybody",
     build_shapes(cfg_for("b1"), p1.axes(states, speeds)) == [])

group(8, "B2 is one round radius for everybody, as before")
c2 = cfg_for("b2")
p2 = make_policy(c2)
true("name", p2.name == "static")
for key in states:
    near(f"{key} along", p2.axes(states, speeds)[key][0], 1.20)
    near(f"{key} across", p2.axes(states, speeds)[key][1], 1.20)
    near(f"{key} offset", p2.axes(states, speeds)[key][2], 0.0)
true("every B2 shape is a circle",
     all(s.is_circle and not s.center_offset_along
         for s in build_shapes(c2, p2.axes(states, speeds), grow=robot_grow(c2))))

group(9, "B3-R is state-dependent and still round, as before")
c3r = cfg_for("b3r")
p3r = make_policy(c3r)
want_r = {"N1": 0.60, "E1": 0.90, "S2": 1.20}
for key, radius in want_r.items():
    a = p3r.axes(states, speeds)[key]
    near(f"{key} along", a[0], radius)
    near(f"{key} across", a[1], radius)
    near(f"{key} offset", a[2], 0.0)
true("every B3-R shape is a circle",
     all(s.is_circle and not s.center_offset_along
         for s in build_shapes(c3r, p3r.axes(states, speeds))))

group(10, "B3 keeps its centred moving ellipse, as before")
c3 = cfg_for("b3")
p3 = make_policy(c3)
a3 = p3.axes(states, speeds)
near("a moving person is stretched 1.8x along the heading",
     a3["S2"][0], 1.20 * 1.8)
near("across is the state radius", a3["S2"][1], 1.20)
near("and it is still centred on them", a3["S2"][2], 0.0)
near("a standing person stays round", a3["E1"][0], a3["E1"][1])

group(11, "B3-D standing is rear-biased, engaged is rear-compressed")
c3d = cfg_for("b3d")
p3d = make_policy(c3d)
true("name", p3d.name == "state_directional")
e = p3d.extents_for("x", "stationary")
near("stationary front", e.front, 0.80)
near("stationary side", e.side, 0.90)
near("stationary rear", e.rear, 1.05)
true("stationary is rear > side > front", e.rear > e.side > e.front)
e = p3d.extents_for("x", "station_engaged")
near("station_engaged front", e.front, 0.80)
near("station_engaged side", e.side, 0.70)
near("station_engaged rear", e.rear, 0.60)
true("station_engaged is front >= side >= rear", e.front >= e.side >= e.rear)
near("engaged rear equals the B3 station_engaged radius", e.rear,
     float(c3d["comfort_space"]["state_radius"]["station_engaged"]))

group(12, "B3-D maps walking speed to a forward-biased space")
for v, (f, s, r) in ((0.6, (1.08, 0.86, 0.86)),
                     (0.8, (1.14, 0.88, 0.88)),
                     (1.0, (1.20, 0.90, 0.90)),
                     (1.2, (1.26, 0.92, 0.92))):
    e = p3d.extents_for("x", "moving", v)
    near(f"v={v} front", e.front, f, 5e-9)
    near(f"v={v} side", e.side, s, 5e-9)
    near(f"v={v} rear", e.rear, r, 5e-9)
    true(f"v={v} is front > side ~ rear", e.front > e.side and e.side == e.rear)
near("the offset points forward for a walker",
     p3d.extents_for("x", "moving", 1.0).axes[2], 0.15)

group(13, "B3-D clips the mapping at both ends")
fast = p3d.extents_for("x", "moving", 1.5)
near("front is capped", fast.front, 1.35, 5e-9)
near("side is still under its cap", fast.side, 0.95, 5e-9)
near("front stays capped well past the cap speed",
     p3d.extents_for("x", "moving", 9.0).front, 1.35, 5e-9)
near("front holds its floor at a standstill",
     p3d.extents_for("x", "moving", 0.0).front, 0.95, 5e-9)

group(14, "an unknown state still raises, in every policy")
for label, policy in (("state_adaptive", p3), ("state_directional", p3d)):
    try:
        policy.extents_for("x", "dozing", 0.0)
        failures.append(f"{label} accepted an unknown state")
        checks += 1
    except ValueError:
        checks += 1

# ---------------------------------------------------------------
# Heading
# ---------------------------------------------------------------
group(15, "standing people point along their body orientation")
res = HeadingResolver(c3d, p3d)
body = res.resolve({"N1": "station_engaged", "E1": "stationary"})
near("N1 faces -90 deg, from facing_deg", body["N1"], math.radians(-90.0))
near("E1 faces -90 deg, from facing_deg", body["E1"], math.radians(-90.0))
near("S1 faces +90 deg, from facing_deg",
     res.resolve({"S1": "station_engaged"})["S1"], math.radians(90.0))
# A travel direction must not leak into a standing person's space.
leaked = res.resolve({"N1": "station_engaged"}, {"N1": 1.234}, {"N1": 1.0})
near("a supplied travel direction is ignored while standing",
     leaked["N1"], math.radians(-90.0))

group(16, "walking people point along their velocity")
walk = res.resolve({"S2": "moving"}, {"S2": 0.5}, {"S2": 1.0})
near("the velocity direction is used", walk["S2"], 0.5)

group(17, "a walker slowing to a stop does not spin")
# Walking: direction believed and remembered.
res.resolve({"S2": "moving"}, {"S2": 0.80}, {"S2": 1.00})
held = []
for noisy, speed in ((0.80, 1.00), (2.90, 0.04), (-1.70, 0.01),
                     (0.20, 0.00), (3.05, 0.02)):
    held.append(res.resolve({"S2": "moving"}, {"S2": noisy},
                            {"S2": speed})["S2"])
true("the direction is held through the noise below 0.10 m/s",
     all(abs(h - 0.80) < TOL for h in held[1:]))
near("and the believable direction is the one held", held[0], 0.80)
# Once they are properly moving again the new direction is taken.
near("a new believable direction replaces it",
     res.resolve({"S2": "moving"}, {"S2": -0.60}, {"S2": 0.9})["S2"], -0.60)
# Transition: the walk direction must be dropped the moment they stop.
near("arriving at a machine drops the walk direction",
     res.resolve({"S2": "station_engaged"}, {"S2": -0.60},
                 {"S2": 0.0})["S2"], math.radians(90.0))

# A person nobody has an orientation for gets a round space rather than
# an invented direction.
group(0, "an unknown orientation gives a round space, not a guess")
unknown = build_shapes(c3d, {"p1": (0.925, 0.90, -0.125)},
                       positions={"p1": (1.0, 1.0)},
                       headings={"p1": None})
true("one shape is built", len(unknown) == 1)
if unknown:
    true("it is a circle", unknown[0].is_circle)
    near("at the largest extent asked for",
         unknown[0].across, 1.05)
    near("with no offset", unknown[0].center_offset_along, 0.0)

# ---------------------------------------------------------------
print()
if failures:
    print(f"[check_comfort_shapes] {len(failures)} of {checks} checks FAILED")
    for line in failures:
        print(f"  - {line}")
    sys.exit(1)
print(f"[check_comfort_shapes] {checks} checks passed")
sys.exit(0)
