"""Getting out of the way of someone who will not get out of yours.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

The yield gate's basic rule, "stop rather than enter a Comfort Space",
is right for a person standing still. For a person walking straight at
the robot it is wrong: stopping inside their line of travel means waiting
for them to walk into you. Measured in this project, a walker at 1.1 m/s
reached a robot the gate had parked in their lane and the footprint and
the person overlapped by 6 cm for a full second.

So for a moving person the decision is taken on *time*, not distance.
If they will reach the robot within `horizon_seconds` and the robot is in
their lane, the robot drives sideways out of the lane and waits there
until they have gone past. Physical safety outranks the Comfort Space
while it does, and the escape point is checked against every other
person's body so it does not trade one collision for another.

Only people whose measured speed is above the moving threshold are
considered, so a scene of standing people (B2 and B3 as compared) never
triggers any of this.

> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

from comfort_core.clearance import footprint_half_extents
from comfort_core.layout import (drivable_area, inside_drivable,
                                 obstacle_boxes, refuge_centres, scene_bounds)


@dataclass(frozen=True)
class Person:
    key: str
    x: float
    y: float
    heading: float
    speed: float


@dataclass(frozen=True)
class EvasionParams:
    horizon_seconds: float
    lateral_margin: float
    arrive_tolerance: float
    max_linear: float
    max_angular: float
    heading_gain: float
    human_radius: float
    robot_radius: float      # circumscribed: correct whatever the robot's yaw
    speed_threshold: float

    @property
    def lane_half_width(self) -> float:
        """How far from a walker's line the robot's centre must be to be clear."""
        return self.human_radius + self.robot_radius + self.lateral_margin

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "EvasionParams":
        cs = cfg["comfort_space"]
        ev = cs["yield"].get("evade", {})
        nav = cfg["navigation"]
        half_length, half_width = footprint_half_extents(cfg)
        return cls(
            horizon_seconds=float(ev.get("horizon_seconds", 6.0)),
            lateral_margin=float(ev.get("lateral_margin", 0.10)),
            arrive_tolerance=float(ev.get("arrive_tolerance", 0.08)),
            max_linear=float(ev.get("max_linear_speed", nav["max_linear_speed"])),
            max_angular=float(ev.get("max_angular_speed", nav["max_angular_speed"])),
            heading_gain=float(ev.get("heading_gain", 2.0)),
            human_radius=float(cfg["humans"]["radius"]),
            robot_radius=math.hypot(half_length, half_width),
            speed_threshold=float(cs.get("moving_speed_threshold", 0.15)),
        )


def lane_coordinates(person: Person, px: float, py: float) -> Tuple[float, float]:
    """(ahead, aside) of a point in the walker's frame.

    `ahead` is along their heading, positive in front of them; `aside` is
    to their left.
    """
    c, s = math.cos(person.heading), math.sin(person.heading)
    dx, dy = px - person.x, py - person.y
    return dx * c + dy * s, -dx * s + dy * c


def threat(person: Person, robot_xy: Tuple[float, float],
           p: EvasionParams) -> Optional[float]:
    """Seconds until this walker reaches the robot, or None if they will not.

    Assumes the robot stays where it is, which is the conservative case:
    a robot that is already moving away only gains time.
    """
    if person.speed < p.speed_threshold:
        return None
    ahead, aside = lane_coordinates(person, *robot_xy)
    if ahead <= 0.0 or abs(aside) >= p.lane_half_width:
        return None
    contact_gap = max(ahead - (p.human_radius + p.robot_radius), 0.0)
    eta = contact_gap / person.speed
    return eta if eta <= p.horizon_seconds else None


def has_passed(person: Person, robot_xy: Tuple[float, float],
               p: EvasionParams) -> bool:
    """The walker is behind the robot, or has stopped walking."""
    if person.speed < p.speed_threshold:
        return True
    ahead, _ = lane_coordinates(person, *robot_xy)
    return ahead < -(p.human_radius + p.robot_radius)


def choose_target(cfg: Dict[str, Any], person: Person,
                  robot_xy: Tuple[float, float],
                  others: Iterable[Tuple[float, float]],
                  p: EvasionParams,
                  side: Optional[int] = None):
    """Where to wait, and on which side of the walker's line.

    Returns (x, y, side, detail). The point is level with the robot along
    the walker's line and just outside their lane. Both sides are tried
    unless `side` is fixed; a side is rejected if the corridor walls leave
    no room there or if waiting there would put the robot against another
    person. Of the acceptable sides the nearer one wins.
    """
    bounds = scene_bounds(cfg)
    inset = p.robot_radius + 0.05
    ahead, aside = lane_coordinates(person, *robot_xy)
    c, s = math.cos(person.heading), math.sin(person.heading)
    offset = p.lane_half_width + 0.05

    options = []
    for candidate in ((side,) if side else (1, -1)):
        v = candidate * offset
        tx = person.x + ahead * c - v * s
        ty = person.y + ahead * s + v * c
        tx = min(max(tx, bounds.x_min + inset), bounds.x_max - inset)
        ty = min(max(ty, bounds.y_min + inset), bounds.y_max - inset)
        _, clamped_aside = lane_coordinates(person, tx, ty)
        lane_ok = abs(clamped_aside) >= p.lane_half_width
        worst_body = min(
            (math.hypot(tx - ox, ty - oy) - p.human_radius - p.robot_radius
             for ox, oy in others), default=math.inf)
        body_ok = worst_body >= p.lateral_margin
        travel = math.hypot(tx - robot_xy[0], ty - robot_xy[1])
        options.append({
            "ok": lane_ok and body_ok, "lane_ok": lane_ok, "body": worst_body,
            "travel": travel, "side": candidate, "x": tx, "y": ty,
            "text": (f"side {candidate:+d}: lane {'clear' if lane_ok else 'blocked by wall'}, "
                     f"nearest other person {worst_body:+.2f} m")})

    good = [o for o in options if o["ok"]]
    if good:
        pick = min(good, key=lambda o: o["travel"])
    else:
        # Nothing is clean. Getting out of the walker's lane matters most,
        # then keeping as far from everyone else as the room allows.
        pick = max(options, key=lambda o: (o["lane_ok"], o["body"]))
    tx, ty, chosen = pick["x"], pick["y"], pick["side"]
    detail = "; ".join(o["text"] for o in options) + ("" if pick["ok"] else "  -> no clean option")
    return tx, ty, chosen, detail


def refresh_target(cfg: Dict[str, Any], person: Person,
                   robot_xy: Tuple[float, float],
                   others: Iterable[Tuple[float, float]],
                   p: EvasionParams, side: int,
                   current: Tuple[float, float]) -> Tuple[Tuple[float, float], bool]:
    """Move the waiting point further out if the latest estimate needs it.

    The point is first chosen while the walker is still several metres
    away, by extending their heading to the robot. A heading error of one
    degree at 6.8 m moves that line by 12 cm, and two otherwise identical
    trials here picked points 16 cm apart, one of which the walker then
    brushed. As they come closer the extension shortens and the estimate
    settles, so the point is re-derived every tick on the side already
    chosen. It only ever moves away from their line, never back towards
    it, so the robot does not chase noise.

    Returns (point, moved).
    """
    _, current_aside = lane_coordinates(person, *current)
    if abs(current_aside) >= p.lane_half_width + 0.05 - 1e-3:
        return current, False
    tx, ty, _side, _detail = choose_target(cfg, person, robot_xy, others, p, side=side)
    _, new_aside = lane_coordinates(person, tx, ty)
    if abs(new_aside) > abs(current_aside) + 1e-3:
        return (tx, ty), True
    return current, False


def drive_to(pose: Tuple[float, float, float], target: Tuple[float, float],
             p: EvasionParams) -> Tuple[float, float, bool]:
    """(linear, angular, arrived) for a differential drive heading to a point.

    Drives forwards or in reverse, whichever needs less turning, so the
    robot never spends a quarter turn rotating when backing off a short
    way would do.
    """
    x, y, yaw = pose
    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    if distance <= p.arrive_tolerance:
        return 0.0, 0.0, True

    bearing = math.atan2(dy, dx)
    error = _wrap(bearing - yaw)
    direction = 1.0
    if abs(error) > math.pi / 2.0:
        direction = -1.0
        error = _wrap(error - math.pi)

    angular = max(-p.max_angular, min(p.max_angular, p.heading_gain * error))
    # Only creep while badly misaligned; drive properly once roughly lined up.
    alignment = max(math.cos(error), 0.0) ** 4
    linear = direction * min(p.max_linear, 1.5 * distance) * alignment
    return linear, angular, False


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# ----------------------------------------------------------------------
# Comfort-aware waiting: predict where the walker's Comfort Space will
# sweep, and wait where it intrudes least.
#
# The physical-only sidestep above keeps bodies apart but leaves the
# robot deep inside the walker's Comfort Space: 0.63 m from their centre
# against a 1.20 m radius, measured. The corridor has room to do better,
# because a robot parked against the far wall, parallel to it, sits
# outside a 1.20 m space around someone walking down the middle.
#
# The same sweep is what comfort_publisher paints into the costmap, so
# the planner and the gate act on one prediction.
# ----------------------------------------------------------------------

from comfort_core.intrusion import footprint_outline, intrusion  # noqa: E402
from comfort_core.shapes import ComfortShape  # noqa: E402


@dataclass(frozen=True)
class ComfortParams:
    horizon_seconds: float
    sweep_step: float
    lateral_step: float
    back_offsets: Tuple[float, ...]
    reach_linear: float
    reach_angular: float
    reach_slack: float
    reevaluate_period: float
    switch_margin: float
    wall_margin: float
    body_margin: float
    yaw_tolerance: float
    heading_sigma: float
    enough_k: float
    soonest_when_late: bool
    rechoose_k: float           # re-pick the spot only if its score falls below this
    hold_drift: float           # how far the robot may drift before correcting again
    resume_delay: float         # stay still this long after the walker has passed
    lane_inset: float           # keep the footprint this far inside the lane
    release_distance: float     # give up waiting once they are this far off
    max_wait: float             # and give up anyway after this long

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "ComfortParams":
        c = cfg["comfort_space"]["yield"]["evade"].get("comfort", {})
        return cls(
            horizon_seconds=float(c.get("horizon_seconds", 10.0)),
            sweep_step=float(c.get("sweep_step", 0.25)),
            lateral_step=float(c.get("lateral_step", 0.10)),
            back_offsets=tuple(float(v) for v in c.get("back_offsets", [0.0, 0.5])),
            reach_linear=float(c.get("reach_linear_speed", 0.35)),
            reach_angular=float(c.get("reach_angular_speed", 0.70)),
            reach_slack=float(c.get("reach_slack_seconds", 1.0)),
            reevaluate_period=float(c.get("reevaluate_period", 0.5)),
            switch_margin=float(c.get("switch_margin", 0.05)),
            wall_margin=float(c.get("wall_margin", 0.15)),
            body_margin=float(c.get("body_margin", 0.10)),
            yaw_tolerance=float(c.get("yaw_tolerance", 0.08)),
            heading_sigma=math.radians(float(c.get("heading_sigma_deg", 3.0))),
            enough_k=float(c.get("enough_k", 1.15)),
            soonest_when_late=bool(c.get("soonest_when_late", False)),
            rechoose_k=float(c.get("rechoose_below_k", 1.0)),
            hold_drift=float(c.get("hold", {}).get("drift_tolerance", 0.25)),
            resume_delay=float(c.get("hold", {}).get("resume_delay_seconds", 0.7)),
            lane_inset=float(c.get("lane_inset", 0.15)),
            release_distance=float(c.get("release_distance", 5.0)),
            max_wait=float(c.get("max_wait_seconds", 20.0)),
        )


def predicted_sweep(person: Person, along: float, across: float,
                    horizon: float, step: float) -> List[Tuple[float, ComfortShape]]:
    """(time, Comfort Space) at regular times along a straight walk."""
    c, s = math.cos(person.heading), math.sin(person.heading)
    out = []
    n = max(int(horizon / step), 0)
    for i in range(n + 1):
        t = i * step
        out.append((t, ComfortShape(person.key, person.x + c * person.speed * t,
                                    person.y + s * person.speed * t,
                                    along, across, person.heading)))
    return out


def first_intrusion(sweep, outline) -> Optional[Tuple[float, float]]:
    """(time, k) of the first predicted shape the outline is inside, if any."""
    for t, shape in sweep:
        k, _ = intrusion(shape, outline)
        if k < 1.0:
            return t, k
    return None


def reach_position(pose: Tuple[float, float, float],
                   target: Tuple[float, float], cp: "ComfortParams") -> float:
    """Time to be standing on a spot, whatever way the robot ends up facing.

    This is what the walker cares about: out of the lane. Asking
    reach_time for a final heading equal to the bearing quietly charges
    a half turn whenever the quickest way there is in reverse -- 11.4 s
    for a spot the robot reaches in 6.9 -- and every candidate then
    looks hopeless.
    """
    x, y, yaw = pose
    tx, ty = target
    dist = math.hypot(tx - x, ty - y)
    if dist < 0.05:
        return 0.0
    bearing = math.atan2(ty - y, tx - x)
    e = abs(_wrap(bearing - yaw))
    turn = min(e, math.pi - e)
    return turn / cp.reach_angular + dist / cp.reach_linear


def reach_time(pose: Tuple[float, float, float], target: Tuple[float, float, float],
               cp: ComfortParams) -> float:
    """Rough time for a differential drive to get to a pose.

    Turn toward it (or away from it, driving in reverse), drive, turn to
    the final heading. Deliberately pessimistic speeds: arriving early
    costs nothing, arriving late means waiting inside someone's space.
    """
    x, y, yaw = pose
    tx, ty, tyaw = target
    dist = math.hypot(tx - x, ty - y)
    if dist < 0.05:
        return abs(_wrap(tyaw - yaw)) / cp.reach_angular
    bearing = math.atan2(ty - y, tx - x)
    e = abs(_wrap(bearing - yaw))
    turn1 = min(e, math.pi - e)
    heading_on_arrival = bearing if e <= math.pi / 2 else _wrap(bearing + math.pi)
    turn2 = abs(_wrap(tyaw - heading_on_arrival))
    return (turn1 + turn2) / cp.reach_angular + dist / cp.reach_linear


@dataclass(frozen=True)
class Spot:
    x: float
    y: float
    yaw: float
    side: int         # +1 left of the walker's line, -1 right
    k: float          # worst Comfort Space scale factor while waiting; >= 1 is outside
    reach: float      # estimated seconds to get there
    deadline: float   # seconds before the walker's space arrives there


# Kept as a name because scripts import it; the scene decides what a
# static obstacle is now, not the corridor.
station_boxes = obstacle_boxes


def choose_comfort_spot(cfg: Dict[str, Any], person: Person, along: float, across: float,
                        pose: Tuple[float, float, float],
                        others: List[ComfortShape],
                        p: EvasionParams, cp: ComfortParams,
                        side: Optional[int] = None,
                        anchor_x: Optional[float] = None) -> Optional[Spot]:
    """Best reachable place to wait while this walker goes past.

    Candidates lie on a grid across the corridor at the robot's position
    and a little further from the walker, each facing along the
    corridor. A candidate must be physically clear of walls, stations and
    people. It is scored by the worst Comfort Space intrusion it would
    suffer while the walker's predicted space sweeps past, counting the
    standing people's spaces too, so stepping out of one person's way
    into another's does not score well.

    `side` (+1 left of the walker, -1 right) restricts the search to one
    side of their line. Once the robot has picked a side it keeps it:
    from several metres away a few degrees of heading error moves the
    predicted sweep by tens of centimetres, and a robot that followed
    every revision crossed the walker's lane mid-approach. In an offline
    test of 108 approaches that crossing caused 36 collisions.

    A candidate does not have to be reached before the walker's space
    arrives. Moving away from their line only ever helps, so a far spot
    that is reached late still beats a near one that is reached early;
    each candidate is scored on the part of the sweep that happens after
    the robot gets there. The one hard timing rule is for spots across
    the walker's line: the robot must be over before they arrive.

    `anchor_x` fixes where along the corridor the candidates sit. Without
    it they are placed relative to wherever the robot is now, and since
    one of the offsets is "further from the walker", each re-evaluation
    put the best spot half a metre behind the robot, the robot reversed
    onto it, and the next re-evaluation moved it back again. Measured in
    Isaac Sim: the robot walked backwards from x = -4.2 to -5.4 while the
    walker approached, which is neither waiting nor yielding.

    Returns None when no candidate qualifies.
    """
    bounds = scene_bounds(cfg)
    half_length, half_width = footprint_half_extents(cfg)
    boxes = station_boxes(cfg)
    drivable = drivable_area(cfg)
    x0, y0, yaw0 = pose
    h = person.heading
    hc = math.cos(h)

    # Wait until the walker is well past the candidate.
    ahead_robot, aside_robot = lane_coordinates(person, x0, y0)
    cross_deadline = (ahead_robot - p.human_radius - p.robot_radius
                      - p.lateral_margin) / max(person.speed, 1e-3)
    span = ahead_robot + max(cp.back_offsets) + along + 1.0
    horizon = min(max(span / max(person.speed, 1e-3), 0.0), 30.0)
    sweep = predicted_sweep(person, along, across, horizon, cp.sweep_step)

    # Face along the corridor, toward the goal, so Nav2 can drive straight
    # off when it gets the robot back. Facing the other way meant a half
    # turn beside the wall, which the planner could not find room for: the
    # robot crept along the wall until the trial called it a deadlock.
    yaw = goal_facing_yaw(cfg, x0)

    # Far enough from the wall to turn on the spot: the robot arrives at
    # whatever angle it drove in on and turns to face the goal there.
    inset = max(half_width + cp.wall_margin, p.robot_radius + 0.03)
    y_lo = bounds.y_min + inset
    y_hi = bounds.y_max - inset
    n_lat = int((y_hi - y_lo) / cp.lateral_step) + 1

    # Where along the corridor to try. Beside the robot, a little
    # further from the walker -- and, in a bay too narrow to stand
    # aside in, at the refuges cut into the work zones. Without those
    # the search never looks more than half a metre from where the
    # robot already is, and in a 1.5 m aisle there is nothing there.
    anchor = x0 if anchor_x is None else anchor_x
    candidate_x = [anchor + hc * back for back in cp.back_offsets]
    candidate_x += [rx for rx, _ in refuge_centres(cfg)]

    best, best_score = None, None
    for cx in candidate_x:
        for i in range(n_lat):
            cy = y_lo + i * cp.lateral_step
            if not (bounds.x_min + half_length + cp.wall_margin <= cx
                    <= bounds.x_max - half_length - cp.wall_margin):
                continue
            outline = footprint_outline((cx, cy, yaw), half_length, half_width)
            # Aisles and refuges only, where the scene says so -- and a
            # way of getting there that stays on them. Without the path
            # check the search will happily pick a refuge on the far
            # side of the bay and set off across the shop floor.
            # Inside the drivable floor with room to spare, not just
            # inside it. A spot whose footprint ends exactly on the
            # painted line becomes a spot OUTSIDE the line as soon as
            # the drive to it overshoots by a few centimetres, and once
            # the robot is off the lane the planner will not plan from
            # where it stands: three stress waves in a row ended with a
            # trial sitting 0.2 to 0.3 m outside an aisle, yielding,
            # with nav2 ABORTED and no cause recorded.
            #
            # The margin is negative here -- the footprint has to be
            # this far INSIDE -- which is why it is subtracted.
            if not inside_drivable(drivable, outline, -cp.lane_inset):
                continue
            if not _path_drivable(drivable, (x0, y0), (cx, cy)):
                continue
            # And the way there must not go through anybody.
            #
            # The spot itself was checked against the other people;
            # the route to it was not, so the robot drove straight at a
            # worker standing beside the aisle on its way to a
            # perfectly good waiting point. It approached her almost
            # sideways, which is exactly where a forward protective
            # field cannot help: measured, 31 s pressed against w3 in
            # stress seed 5, the field never fired because she was
            # 0.76 m abeam and the field reaches 0.34 m to the side.
            if not _path_clear_of(others, (x0, y0), (cx, cy),
                                  p.human_radius + p.robot_radius):
                continue
            if any(bx0 - cp.body_margin <= ox <= bx1 + cp.body_margin
                   and by0 - cp.body_margin <= oy <= by1 + cp.body_margin
                   for ox, oy in outline for bx0, bx1, by0, by1 in boxes):
                continue
            # Clear of other people WHATEVER WAY THE ROBOT ENDS UP
            # FACING, so the circumscribed circle rather than the
            # outline. The robot turns on the spot once it gets there
            # -- to face the goal again before handing back -- and a
            # spot checked at one heading is not clear at another: a
            # 0.70 x 0.48 m body swings its corners 0.42 m from its
            # centre. Measured: it waited 0.80 m from a standing
            # worker, which the outline test passed, then turned and
            # its corner touched her.
            body_ok = all(math.hypot(cx - o.x, cy - o.y)
                          >= p.human_radius + p.robot_radius + cp.body_margin
                          for o in others)
            if not body_ok:
                continue
            # Physically clear of the walker's line too, with room for
            # the heading being a few degrees out: extended over `ahead`
            # metres, that error moves the line sideways by ahead * tan.
            # The extra 0.15 m keeps a chosen spot from sitting right on
            # the limit at which it would be abandoned again.
            #
            # Measured from the robot's outline, not from a circle round
            # it. Waiting square to the aisle, the robot puts 0.24 m of
            # itself toward the walker, not the 0.42 m of its
            # circumscribed radius, and charging it the difference ruled
            # out refuges it fits in with room to spare.
            ahead_c, aside = lane_coordinates(person, cx, cy)
            slack = (p.human_radius + cp.body_margin + 0.15
                     + max(ahead_c, 0.0) * math.tan(cp.heading_sigma))
            if min(abs(lane_coordinates(person, ox, oy)[1])
                   for ox, oy in outline) < slack:
                continue
            if side is not None and aside * side < 0.0:
                continue

            reach = reach_time(pose, (cx, cy, yaw), cp)
            # Time to be out of the lane, which is what the walker
            # cares about: squaring up to the corridor afterwards can
            # happen while waiting.
            settle = reach_position(pose, (cx, cy), cp)
            crosses = aside * aside_robot < 0.0
            if crosses and reach + cp.reach_slack > cross_deadline:
                continue

            after = [shape for t, shape in sweep if t >= reach]
            k = min([intrusion(shape, outline)[0] for shape in after]
                    + [intrusion(o, outline)[0] for o in others], default=math.inf)

            deadline = (ahead_c - along - half_length) / max(person.speed, 1e-3)
            spot = Spot(cx, cy, yaw, 1 if aside > 0 else -1, k, reach, deadline)
            # More room beyond the boundary is worth having, up to a
            # point, because the prediction is a few degrees out. Past
            # that, stay on the robot's own side of the walker, then go
            # the shortest way.
            #
            # When nothing can be reached before the walker arrives --
            # a 1.5 m aisle with a pull-off a metre and a half away --
            # the roomiest spot is the wrong answer: the robot is still
            # crossing the lane when they meet. Then the one it can get
            # to soonest wins instead. Off by default, because in a
            # corridor wide enough to stand aside in, arriving a second
            # late costs nothing and the extra room is worth having.
            in_time = settle + cp.reach_slack <= deadline
            if cp.soonest_when_late and not in_time:
                score = (False, -settle, min(round(k, 2), cp.enough_k),
                         not crosses)
            else:
                score = (True, min(round(k, 2), cp.enough_k), not crosses,
                         -reach)
            if best is None or score > best_score:
                best, best_score = spot, score
    return best


def _path_clear_of(people, start: Tuple[float, float],
                   end: Tuple[float, float], radius: float,
                   step: float = 0.15) -> bool:
    """Whether the straight line between two points misses everybody.

    `radius` is the distance to keep from a person's centre: their
    radius plus the robot's circumscribed radius, because the robot may
    be at any heading along the way.
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    n = max(int(math.hypot(dx, dy) / step), 1)
    for i in range(n + 1):
        px = start[0] + dx * i / n
        py = start[1] + dy * i / n
        for person in people:
            if math.hypot(px - person.x, py - person.y) < radius:
                return False
    return True


def _path_drivable(rects, start: Tuple[float, float],
                   end: Tuple[float, float], step: float = 0.2) -> bool:
    """Whether the straight line between two points stays drivable.

    A straight line is not the path Nav2 will take, but if even that
    leaves the aisles then the spot is on the wrong side of something.
    """
    if rects is None:
        return True
    dx, dy = end[0] - start[0], end[1] - start[1]
    n = max(int(math.hypot(dx, dy) / step), 1)
    points = [(start[0] + dx * i / n, start[1] + dy * i / n)
              for i in range(n + 1)]
    return inside_drivable(rects, points)


def goal_facing_yaw(cfg: Dict[str, Any], x: float) -> float:
    """Heading along the corridor axis that points at the navigation goal."""
    return 0.0 if float(cfg["navigation"]["goal"][0]) >= x else math.pi


def snap_to_corridor(person: Person, snap_deg: float) -> Person:
    """Treat a walker heading nearly along the corridor as heading along it.

    The corridor runs along x. People walking down it walk along it; the
    walls leave little else. The estimated heading, on the other hand,
    wobbles by several degrees with the character's sway, and the gate
    extends it ten metres to find their lane: 3 deg there is 0.6 m. In
    Isaac Sim that put the lane on the wrong side of the robot and it
    crossed over to the narrow side of the corridor in both runs.

    Within `snap_deg` of the axis the heading is set to the axis, which
    makes the lane simply the walker's current lateral position. A person
    clearly turning (beyond `snap_deg`) keeps their measured heading.
    """
    if snap_deg <= 0.0:
        return person
    for axis in (0.0, math.pi):
        if abs(_wrap(person.heading - axis)) <= math.radians(snap_deg):
            return Person(person.key, person.x, person.y, axis, person.speed)
    return person


class EvasionController:
    """The yield gate's decisions about walkers, free of ROS.

    Kept here, not in the node, so the offline stress test exercises the
    very code the gate runs.

        step(now, pose, people, others_raw) -> (linear, angular) or None

    None means "no walker to deal with, let Nav2 drive". `people` are the
    measured walkers and standers; `others_raw` maps each person's key to
    their raw Comfort Space (not enlarged for the planner).
    """

    def __init__(self, cfg: Dict[str, Any], log=print):
        self.cfg = cfg
        self.p = EvasionParams.from_config(cfg)
        self.cp = ComfortParams.from_config(cfg)
        ev = cfg["comfort_space"]["yield"].get("evade", {})
        self.comfort_mode = bool(ev.get("comfort", {}).get("enabled", True))
        self.snap_deg = float(ev.get("corridor_heading_snap_deg", 15.0))
        self.half = footprint_half_extents(cfg)
        self.log = log
        self.reset()

    def reset(self) -> None:
        self.walker: Optional[str] = None
        self.mode: Optional[str] = None      # "comfort" or "physical"
        self.target: Optional[Tuple[float, float, float]] = None
        self.side: Optional[int] = None
        self.spot: Optional[Spot] = None
        self._last_eval = -math.inf
        # Once in position the robot stops dead and stays there. Yielding
        # should read as yielding: before this latch existed it kept
        # nudging itself toward a waiting point that moved a few
        # centimetres every half second, and the result looked like
        # twitching rather than waiting.
        self.holding = False
        self._phase = "turn"        # turn -> go -> align, see _drive
        self._direction = 1.0
        self._last_trace = -math.inf
        self._passed_at: Optional[float] = None
        # When this sidestep began, and whether the walker has already
        # been reported as out of range. Both only exist so that
        # waiting for somebody can end for a reason other than their
        # walking past.
        self._started_at: Optional[float] = None
        self._said_far = False
        self.anchor_x: Optional[float] = None

    @property
    def state(self) -> str:
        """One word for what the robot is doing, for logs and overlays."""
        if self.walker is None:
            return "DRIVING"
        if self.mode == "finishing":
            return "RESUMING"
        if self.holding:
            return "WAITING"
        return "STEPPING ASIDE"

    @property
    def active(self) -> bool:
        return self.walker is not None

    def step(self, now: float, pose, people: List[Person],
             raw: Dict[str, ComfortShape]):
        x, y, _ = pose
        people = [snap_to_corridor(q, self.snap_deg) if q.speed >= self.p.speed_threshold
                  else q for q in people]
        by_key = {q.key: q for q in people}

        if self.walker is None:
            self._maybe_start(now, pose, people, raw)
            if self.walker is None:
                return None
            self._started_at = now

        person = by_key.get(self.walker)
        # Two more ways to be done with somebody, besides their walking
        # past. Without them the robot waits for a walker who is never
        # going to pass it: these people walk 30 m round a U-shaped
        # bay, so one can spend half a minute in another aisle
        # entirely, still "ahead" of the robot along her own line,
        # while the robot stands still waiting for her. Measured --
        # one sidestep at t = 8 s and then nothing for the next 40 s.
        gone = far = False
        if person is not None:
            gap = math.hypot(person.x - x, person.y - y)
            far = gap > self.cp.release_distance
            gone = (self._started_at is not None
                    and now - self._started_at > self.cp.max_wait)
            if far and not self._said_far:
                self._said_far = True
                self.log(f"walker {self.walker} is {gap:.1f} m away, which is "
                         f"further than waiting for them is worth")
            if gone:
                self.log(f"waited {now - self._started_at:.0f} s for walker "
                         f"{self.walker} and they have not passed; driving on")
        if (self.mode == "finishing" or person is None or far or gone
                or has_passed(person, (x, y), self.p)):
            # A beat of stillness after they pass, so the restart reads as
            # a decision rather than a twitch.
            if self._passed_at is None:
                self._passed_at = now
                self.holding = True
                self.log(f"walker {self.walker} is past; waiting {self.cp.resume_delay:.1f} s "
                         f"before moving off")
            if now - self._passed_at < self.cp.resume_delay and self.mode != "finishing":
                return 0.0, 0.0
            # Face the goal before letting go. Handed back mid-turn, facing
            # the wall, the planner nudged the robot into the wall's
            # high-cost band and it crept there until the trial timed out.
            yaw = goal_facing_yaw(self.cfg, x)
            error = _wrap(yaw - pose[2])
            if abs(error) > self.cp.yaw_tolerance:
                if self.mode != "finishing":
                    self.mode = "finishing"
                    self.log(f"walker {self.walker} has gone past; turning to face "
                             f"the goal before handing back")
                return 0.0, max(-self.p.max_angular,
                                min(self.p.max_angular, self.p.heading_gain * error))
            self.log(f"walker {self.walker} has gone past, handing back to Nav2 "
                     f"at ({x:+.2f}, {y:+.2f})")
            self.reset()
            return None

        if self.mode == "comfort":
            _, aside = lane_coordinates(person, *self.target[:2])
            unsafe = abs(aside) < self.p.lane_half_width + 0.05
            if unsafe:
                # The walker's line, now better estimated, runs closer to
                # the chosen spot than is safe. Physical safety first, and
                # it is worth breaking the hold for.
                self._release_hold("the walker's line now runs too close")
                self._rescue(now, pose, person, people, raw)
            elif self.holding:
                if self._drifted(pose):
                    self._release_hold("drifted off the waiting point")
                else:
                    return 0.0, 0.0
            elif now - self._last_eval >= self.cp.reevaluate_period:
                self._last_eval = now
                self._reconsider(pose, person, people, raw)
            if self.mode == "comfort":
                lin, ang, arrived = self._drive(pose, self.target)
                self._trace(now, pose, lin, ang)
                if arrived:
                    self._hold(person, pose)
                    return 0.0, 0.0
                return lin, ang

        others = [(q.x, q.y) for q in people if q.key != person.key]
        point, moved = refresh_target(self.cfg, person, (x, y), others, self.p,
                                      self.side, self.target[:2])
        if moved and self.holding:
            # Only a safety reason breaks the hold; refresh_target only ever
            # moves further from the walker, so take it and stay put until
            # the next drive cycle needs it.
            self._release_hold("a safer waiting point opened up")
        if moved:
            self.target = (point[0], point[1], 0.0)
            self.log(f"walker {person.key} now at ({person.x:+.2f}, {person.y:+.2f}); "
                     f"waiting point moved out to ({self.target[0]:+.2f}, {self.target[1]:+.2f})")
        yaw = goal_facing_yaw(self.cfg, pose[0])
        if self.holding:
            if not self._drifted(pose):
                return 0.0, 0.0
            self._release_hold("drifted off the waiting point")
        lin, ang, arrived = self._drive(pose, (self.target[0], self.target[1], yaw))
        if arrived:
            self._hold(person, pose)
            return 0.0, 0.0
        return lin, ang

    # ------------------------------------------------------------------
    def _trace(self, now: float, pose, lin: float, ang: float) -> None:
        """A line a second while stepping aside, so a run can be read back."""
        if now - self._last_trace < 1.0:
            return
        self._last_trace = now
        dx = self.target[0] - pose[0]
        dy = self.target[1] - pose[1]
        self.log(f"[{self.state}/{self._phase}] at ({pose[0]:+.2f},{pose[1]:+.2f}) "
                 f"yaw {math.degrees(pose[2]):+.0f} -> target "
                 f"({self.target[0]:+.2f},{self.target[1]:+.2f}) "
                 f"{math.hypot(dx, dy):.2f} m away, cmd {lin:+.2f} m/s {ang:+.2f} rad/s")

    def _drive(self, pose, target) -> Tuple[float, float, bool]:
        """Turn, drive straight, line up: get to the waiting pose and stop.

        Steering continuously toward the point does not work for the last
        stretch. Within a few centimetres the bearing swings with every
        wobble, the steering chases it, and forward speed is throttled by
        how badly aligned the robot is, so the gap never closes: in Isaac
        Sim the robot shuffled around a point 0.2 m away for four seconds.

        Turning first and then driving straight avoids that. The last
        phase lines the robot up along the corridor, which is worth doing:
        parked across it the footprint reaches 0.42 m toward the walker
        instead of 0.24 m. That turn slides this robot about 0.2 m, so
        once it starts, position is no longer checked -- checking it was
        the earlier bug, where the slide put the robot outside the arrival
        tolerance and it drove, turned and slid in a circle.

        A curved approach that arrives already lined up was tried instead,
        to avoid the two standing turns. It reached the waiting point 2.5 s
        later, which is most of the margin before the walker arrives, so
        this stayed.
        """
        x, y, yaw = pose
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)

        if self._phase != "align" and distance <= self.p.arrive_tolerance:
            self._phase = "align"

        if self._phase == "turn":
            bearing = math.atan2(dy, dx)
            error = _wrap(bearing - yaw)
            self._direction = 1.0
            if abs(error) > math.pi / 2.0:
                self._direction = -1.0
                error = _wrap(error - math.pi)
            # Only a big misalignment is worth a standing turn. Insisting
            # on 0.1 rad before moving cost about a second, and the walker
            # arrived while the robot was still on its way.
            if abs(error) > 0.45:
                return 0.0, max(-self.p.max_angular,
                                min(self.p.max_angular,
                                    self.p.heading_gain * error)), False
            self._phase = "go"

        if self._phase == "go":
            along = (dx * math.cos(yaw) + dy * math.sin(yaw)) * self._direction
            if along <= self.p.arrive_tolerance:
                self._phase = "align"
            else:
                # Gentle steering while there is still room, then straight
                # in. Chasing the bearing over the last half metre is what
                # produced the shuffle.
                angular = 0.0
                if distance > 0.4:
                    bearing = math.atan2(dy, dx)
                    error = _wrap(bearing - yaw)
                    if self._direction < 0:
                        error = _wrap(error - math.pi)
                    angular = max(-0.4, min(0.4, self.p.heading_gain * error))
                return (self._direction * min(self.p.max_linear, max(1.2 * along, 0.08)),
                        angular, False)

        error = _wrap(target[2] - yaw)
        if abs(error) > self.cp.yaw_tolerance:
            return 0.0, max(-self.p.max_angular,
                            min(self.p.max_angular, self.p.heading_gain * error)), False
        return 0.0, 0.0, True

    def _hold(self, person: Person, pose) -> None:
        if self.holding:
            return
        self.holding = True
        self.log(f"in position at ({pose[0]:+.2f}, {pose[1]:+.2f}); holding still until "
                 f"walker {person.key} has passed")

    def _release_hold(self, why: str) -> None:
        if not self.holding:
            return
        self.holding = False
        self._phase = "turn"
        self.log(f"leaving the waiting point: {why}")

    def _drifted(self, pose) -> bool:
        """Has the robot slid far enough off the waiting point to correct it?

        Position only, and generously: correcting centimetre errors is
        what made the wait look like a twitch, and the robot deliberately
        keeps whatever heading it arrived at.
        """
        return math.hypot(pose[0] - self.target[0],
                          pose[1] - self.target[1]) > self.cp.hold_drift

    def _others_raw(self, person: Person, people: List[Person], raw):
        return [raw[q.key] for q in people if q.key != person.key and q.key in raw]

    def _maybe_start(self, now, pose, people, raw) -> None:
        x, y, _ = pose
        outline = footprint_outline(pose, *self.half)
        for person in sorted(people, key=lambda q: -q.speed):
            if person.speed < self.p.speed_threshold or person.key not in raw:
                continue
            ahead, _ = lane_coordinates(person, x, y)
            if ahead <= 0.0:
                continue
            shape = raw[person.key]
            if self.comfort_mode:
                sweep = predicted_sweep(person, shape.along, shape.across,
                                        self.cp.horizon_seconds, self.cp.sweep_step)
                hit = first_intrusion(sweep, outline)
                if hit is not None:
                    others = self._others_raw(person, people, raw)
                    spot = choose_comfort_spot(self.cfg, person, shape.along, shape.across,
                                               pose, others, self.p, self.cp)
                    if spot is not None:
                        self.walker, self.mode, self.spot = person.key, "comfort", spot
                        self.side = spot.side
                        self.anchor_x = pose[0]
                        self.target = (spot.x, spot.y, spot.yaw)
                        self._last_eval = now
                        self.log(
                            f"walker {person.key} at ({person.x:+.2f}, {person.y:+.2f}) "
                            f"{person.speed:.2f} m/s will reach the robot's space in "
                            f"{hit[0]:.1f} s: waiting at ({spot.x:+.2f}, {spot.y:+.2f}) "
                            f"facing {math.degrees(spot.yaw):+.0f} deg, predicted k_min "
                            f"{spot.k:.2f}, reach {spot.reach:.1f} s of {spot.deadline:.1f} s")
                        return
            eta = threat(person, (x, y), self.p)
            if eta is not None:
                others = [(q.x, q.y) for q in people if q.key != person.key]
                tx, ty, side, detail = choose_target(self.cfg, person, (x, y), others, self.p)
                self.walker, self.mode, self.side = person.key, "physical", side
                self.target = (tx, ty, 0.0)
                self.log(f"stepping aside for walker {person.key} at "
                         f"({person.x:+.2f}, {person.y:+.2f}) {person.speed:.2f} m/s, "
                         f"arriving in {eta:.1f} s: robot at ({x:+.2f}, {y:+.2f}) -> "
                         f"({tx:+.2f}, {ty:+.2f}). {detail}"
                         + ("  [no comfortable spot reachable in time]" if self.comfort_mode else ""))
                return

    def _rescue(self, now, pose, person, people, raw) -> None:
        """Replace a spot that turned out to be in the walker's lane."""
        shape = raw.get(person.key)
        spot = None
        if shape is not None:
            spot = choose_comfort_spot(self.cfg, person, shape.along, shape.across, pose,
                                       self._others_raw(person, people, raw),
                                       self.p, self.cp, side=self.side,
                                       anchor_x=self.anchor_x)
        if spot is not None:
            self.target, self.spot, self._last_eval = (spot.x, spot.y, spot.yaw), spot, now
            self.log(f"walker {person.key} line re-estimated; moving to "
                     f"({spot.x:+.2f}, {spot.y:+.2f}), k_min {spot.k:.2f}")
            return
        x, y, _ = pose
        others = [(q.x, q.y) for q in people if q.key != person.key]
        tx, ty, _side, detail = choose_target(self.cfg, person, (x, y), others, self.p,
                                              side=self.side)
        self.mode, self.target = "physical", (tx, ty, 0.0)
        self.log(f"walker {person.key} line re-estimated and no comfortable spot is "
                 f"reachable now; stepping aside to ({tx:+.2f}, {ty:+.2f}). {detail}")

    def _reconsider(self, pose, person, people, raw) -> None:
        """Keep the chosen spot unless it has stopped being a good one.

        The spot is picked once and committed to. Moving to whatever
        currently scores best looks like indecision: as the walker's
        heading estimate settled, a slightly nearer spot kept becoming
        acceptable and the robot crept inward, 1.40 m to 1.20 m from the
        walker's line, at a few centimetres per second. A waiting robot
        should be still. Only a spot that the walker is now predicted to
        intrude on is worth leaving.
        """
        shape = raw.get(person.key)
        if shape is None:
            return
        others = self._others_raw(person, people, raw)
        half_length, half_width = self.half
        outline = footprint_outline(self.target, half_length, half_width)
        sweep = predicted_sweep(person, shape.along, shape.across,
                                self.cp.horizon_seconds, self.cp.sweep_step)
        current_k = min([intrusion(s_, outline)[0] for _t, s_ in sweep]
                        + [intrusion(o, outline)[0] for o in others])
        if current_k >= self.cp.rechoose_k:
            return

        spot = choose_comfort_spot(self.cfg, person, shape.along, shape.across,
                                   pose, others, self.p, self.cp, side=self.side,
                                   anchor_x=self.anchor_x)
        if spot is None or spot.k <= current_k + self.cp.switch_margin:
            return
        self._release_hold("the walker's space now reaches the waiting point")
        self.target = (spot.x, spot.y, spot.yaw)
        self.spot = spot
        self.log(f"walker {person.key} now at ({person.x:+.2f}, {person.y:+.2f}); "
                 f"the waiting point would be intruded on (k {current_k:.2f}), "
                 f"moving to ({spot.x:+.2f}, {spot.y:+.2f}) k {spot.k:.2f}")
