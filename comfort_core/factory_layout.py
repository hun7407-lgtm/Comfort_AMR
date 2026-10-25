"""Where everything sits in the factory bay.

Pure Python. Importable from both the Isaac Sim and the ROS 2 side.

The bay is described as a cross section repeated along its length: bands
of work zone and aisle, given in order from one wall to the other. Every
position in the scene is derived from that, so moving a wall or widening
an aisle is a config change rather than an edit in twenty places, the
same rule the corridor scene follows.

    row        a band across the bay: a work zone or an aisle
    bay        one column-to-column section along the length
    refuge     a gap left in a work zone, beside an aisle, wide enough
               for the AMR to pull out of the way
    station    one piece of equipment, with the spot where the person
               who works at it stands

Shapes only. What is built inside a work zone is
isaac_sim/environments/factory.py; what the experiment does with the
positions is the scenario config.

> Dimensions are an engineering approximation of a real plant layout,
> deliberately altered. Not a reproduction of the site.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple


@dataclass(frozen=True)
class Band:
    """One row of the cross section, as the y interval it occupies."""
    name: str
    kind: str          # "zone" or "aisle"
    y_min: float
    y_max: float

    @property
    def depth(self) -> float:
        return self.y_max - self.y_min

    @property
    def centre(self) -> float:
        return (self.y_min + self.y_max) / 2.0


@dataclass(frozen=True)
class Rect:
    """An axis-aligned patch of floor, used for zones and refuges."""
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def centre(self) -> Tuple[float, float]:
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

    @property
    def size(self) -> Tuple[float, float]:
        return (self.x_max - self.x_min, self.y_max - self.y_min)


@dataclass(frozen=True)
class Station:
    """One piece of equipment and the person who works at it.

    `yaw` is where the front of the machine points, in radians, world
    frame: 0 is +x, pi/2 is +y. The worker stands in front of it, which
    is what puts them near the aisle -- the whole point of the layout.
    """
    name: str
    module: str
    x: float
    y: float
    yaw: float
    length: float
    depth: float
    height: float
    # Everyone working at this machine: (x, y, the way they face). A
    # line worked from both sides has several; a rack against the wall
    # has none.
    workers: Tuple[Tuple[float, float, float], ...] = ()

    @property
    def has_worker(self) -> bool:
        return bool(self.workers)

    @property
    def worker(self) -> Tuple[float, float]:
        """Where the first person stands. Convenience for single-post machines."""
        return (self.workers[0][0], self.workers[0][1]) if self.workers else (self.x, self.y)

    @property
    def footprint(self) -> Rect:
        """Axis-aligned box the machine occupies, for overlap checks.

        The hull of the turned box, so a station dragged round in the
        viewport is still checked against the aisles and the refuges.
        """
        angle = self.yaw - math.pi / 2.0
        c, s = abs(math.cos(angle)), abs(math.sin(angle))
        half_x = (self.length * c + self.depth * s) / 2.0
        half_y = (self.length * s + self.depth * c) / 2.0
        return Rect(self.name, self.x - half_x, self.x + half_x,
                    self.y - half_y, self.y + half_y)


@dataclass(frozen=True)
class Waypoint:
    """Somewhere the AMR is asked to be: the ends of its run, mostly."""
    name: str
    x: float
    y: float


@dataclass(frozen=True)
class BayLayout:
    length: float
    width: float
    bands: List[Band]
    columns: List[Tuple[float, float]]
    zones: List[Rect]
    refuges: List[Rect]
    crosses: List[Rect]
    stations: List[Station]
    waypoints: List[Waypoint]
    cart_zones: List[Rect] = field(default_factory=list)
    # The slots inside those blocks, one cart per slot. A taken slot is
    # an obstacle; a free one is where the robot gets out of the way,
    # which is why the free ones are also in `refuges`.
    cart_slots_taken: List[Rect] = field(default_factory=list)
    cart_slots_free: List[Rect] = field(default_factory=list)

    @property
    def x_min(self) -> float:
        return -self.length / 2.0

    @property
    def x_max(self) -> float:
        return self.length / 2.0

    @property
    def y_min(self) -> float:
        return -self.width / 2.0

    @property
    def y_max(self) -> float:
        return self.width / 2.0

    @property
    def aisles(self) -> List[Band]:
        return [b for b in self.bands if b.kind == "aisle"]

    @property
    def in_aisle_space(self):
        """Every rectangle the robot may drive in: aisles and crossings."""
        out = [Rect(b.name, self.x_min, self.x_max, b.y_min, b.y_max)
               for b in self.aisles]
        return out + list(self.crosses)


def _bands(cfg: Dict[str, Any], width: float) -> List[Band]:
    """Turn the configured cross section into y intervals.

    The rows are given wall to wall, so their depths have to add up to
    the bay width. Getting that wrong silently would leave a gap or an
    overlap that every later position inherits.
    """
    rows = cfg["rows"]
    total = sum(float(r["depth"]) for r in rows)
    if abs(total - width) > 1e-6:
        raise ValueError(
            f"factory.rows depths add up to {total:.2f} m but factory.width "
            f"is {width:.2f} m; they have to match")

    bands, y = [], -width / 2.0
    for row in rows:
        depth = float(row["depth"])
        kind = str(row.get("kind", "zone"))
        if kind not in ("zone", "aisle"):
            raise ValueError(f"row kind {kind!r}; expected 'zone' or 'aisle'")
        bands.append(Band(str(row["name"]), kind, y, y + depth))
        y += depth
    return bands


def _columns(length: float, width: float, pitch: float,
             inset: float) -> List[Tuple[float, float]]:
    """Column centres down both long walls, at the given pitch."""
    n = int(round(length / pitch))
    xs = [-length / 2.0 + i * pitch for i in range(n + 1)]
    ys = (-width / 2.0 + inset, width / 2.0 - inset)
    return [(x, y) for x in xs for y in ys]


def _refuges(cfg: Dict[str, Any], bands: List[Band], length: float) -> List[Rect]:
    """Gaps in the work zones where the AMR can pull off the aisle.

    In a 1.5 m aisle a robot cannot let someone walk past it: hugging one
    side leaves 3 cm too little. It has to leave the aisle altogether, so
    the zones beside each aisle are broken at intervals, which is what
    the gaps between equipment blocks are on the real floor anyway.
    """
    spec = cfg.get("refuges", {})
    if not spec.get("enabled", True):
        return []
    spacing = float(spec.get("spacing", 12.0))
    width = float(spec.get("width", 1.5))
    depth = float(spec.get("depth", 0.9))
    margin = float(spec.get("end_margin", 3.0))
    # Which bands may be broken for a refuge. Every neighbour of an
    # aisle, unless the layout says otherwise. The U-shaped bay does:
    # there the aisle has the assembly line on one side and storage on
    # the other, and a gap cut into the line would be a gap where
    # somebody is standing. Pulling off the aisle belongs on the
    # storage side, which is also where it belongs on a real floor.
    allowed = spec.get("in")
    if allowed is not None:
        allowed = {str(name) for name in allowed}

    out: List[Rect] = []
    x = -length / 2.0 + margin
    index = 0
    while x + width <= length / 2.0 - margin:
        for band in bands:
            if band.kind != "aisle":
                continue
            for side, neighbour in (("upper", _band_above(bands, band)),
                                    ("lower", _band_below(bands, band))):
                if neighbour is None or neighbour.depth < depth:
                    continue
                if allowed is not None and neighbour.name not in allowed:
                    continue
                index += 1
                if side == "upper":
                    y0, y1 = band.y_max, band.y_max + depth
                else:
                    y0, y1 = band.y_min - depth, band.y_min
                out.append(Rect(f"refuge_{index:02d}", x, x + width, y0, y1))
        x += spacing
    return out


def _band_above(bands: List[Band], band: Band) -> Band | None:
    i = bands.index(band)
    return bands[i + 1] if i + 1 < len(bands) else None


def _band_below(bands: List[Band], band: Band) -> Band | None:
    i = bands.index(band)
    return bands[i - 1] if i > 0 else None


def _crosses(cfg: Dict[str, Any], bands: List[Band]) -> List[Rect]:
    """Cross aisles joining the long ones.

    Two aisles running side by side and never meeting is not a route
    anyone would lay out: at the ends they join, and the floor becomes
    a loop the AMR can go round. The crossing only spans the aisles and
    what is between them, so the work zones keep their full frontage.
    """
    spec = cfg.get("cross_aisles", {})
    if not spec.get("enabled", True):
        return []
    width = float(spec.get("width", 1.5))
    aisles = [b for b in bands if b.kind == "aisle"]
    if len(aisles) < 2:
        return []
    y0 = min(b.y_min for b in aisles)
    y1 = max(b.y_max for b in aisles)
    return [Rect(f"cross_{i:02d}", float(x) - width / 2.0, float(x) + width / 2.0,
                 y0, y1)
            for i, x in enumerate(spec.get("at", []), start=1)]


def _cart_zones(cfg: Dict[str, Any]) -> List[Rect]:
    """Floor set aside for parts carts, and nothing else.

    Carts scattered along the racking are hard to find, hard to paint
    round and hard to give the AMR a sensible destination in. A real
    plant gives them a marked block, so this does too: a named
    rectangle the robot is allowed to drive into, the carts stand in,
    and no machine may overlap.

    Drivable, unlike a work zone -- the whole point is that the robot
    goes in to collect one.
    """
    out: List[Rect] = []
    for i, item in enumerate(cfg.get("cart_zones", []), start=1):
        x0, x1 = (float(v) for v in item["x"])
        y0, y1 = (float(v) for v in item["y"])
        out.append(Rect(str(item.get("name", f"carts_{i:02d}")),
                        min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)))
    return out


def _cart_bays(cfg: Dict[str, Any], bands: List[Band],
               length: float) -> Tuple[List[Rect], List[Rect], List[Rect]]:
    """Cart staging blocks beside the aisles, and the slots in them.

    Returns (blocks, taken slots, free slots).

    This replaces having a pull-off refuge and a cart park as two
    different things. On a real floor they are the same thing: a block
    of marked slots off the aisle, some with a cart in them and some
    empty, and the empty ones are where a vehicle waits. Keeping them
    separate meant the bay had narrow refuges nobody could put a cart
    in and a cart park the robot was forbidden to wait in, and the
    robot had one 1.5 x 0.9 m box every 4 m to yield into.

    A block is `cols` slots along the aisle by `rows` deep, with its
    near edge on the aisle. Carts fill the BACK row first, left to
    right, and only then the front row. That ordering is the whole
    design: whatever is left empty is on the aisle frontage, so the
    robot can actually reach it, and the cart the robot is sent to
    collect is never parked behind another one.
    """
    spec = cfg.get("cart_bays", {})
    if not spec.get("enabled", False):
        return ([], [], [])
    cols = int(spec.get("cols", 3))
    rows = int(spec.get("rows", 2))
    slot_x, slot_y = (float(v) for v in spec.get("slot", [1.30, 1.00]))
    centres = [float(v) for v in spec.get("at", [])]
    allowed = spec.get("in")
    if allowed is not None:
        allowed = {str(name) for name in allowed}

    # How many of the cols * rows slots hold a cart. One number for
    # every block, or one per entry in `at`.
    taken_spec = spec.get("carts", 0)
    if isinstance(taken_spec, (list, tuple)):
        if len(taken_spec) != len(centres):
            raise ValueError(
                f"cart_bays.carts has {len(taken_spec)} entries for "
                f"{len(centres)} blocks in cart_bays.at")
        counts = [int(v) for v in taken_spec]
    else:
        counts = [int(taken_spec)] * len(centres)

    blocks: List[Rect] = []
    taken: List[Rect] = []
    free: List[Rect] = []
    for band in bands:
        if band.kind != "aisle":
            continue
        for neighbour, away in ((_band_above(bands, band), +1.0),
                                (_band_below(bands, band), -1.0)):
            if neighbour is None:
                continue
            if allowed is not None and neighbour.name not in allowed:
                continue
            if neighbour.depth < rows * slot_y:
                raise ValueError(
                    f"{neighbour.name} is {neighbour.depth:.2f} m deep and a "
                    f"cart block needs {rows * slot_y:.2f} m")
            side = "n" if away > 0 else "s"
            edge = band.y_max if away > 0 else band.y_min
            for index, (cx, count) in enumerate(zip(centres, counts), start=1):
                if count > cols * rows:
                    raise ValueError(
                        f"cart block {side}{index} has {cols * rows} slots "
                        f"and asks for {count} carts")
                x0 = cx - cols * slot_x / 2.0
                x1 = cx + cols * slot_x / 2.0
                if x0 < -length / 2.0 or x1 > length / 2.0:
                    raise ValueError(
                        f"cart block at x {cx:+.2f} runs outside the bay")
                name = f"carts_{side}{index}"
                near, far = edge, edge + away * rows * slot_y
                blocks.append(Rect(name, x0, x1,
                                   min(near, far), max(near, far)))
                # Row 0 is the one against the wall, filled first.
                for row in range(rows):
                    ry0 = edge + away * (rows - row - 1) * slot_y
                    ry1 = ry0 + away * slot_y
                    for col in range(cols):
                        sx0 = x0 + col * slot_x
                        slot = Rect(f"{name}_r{row + 1}c{col + 1}",
                                    sx0, sx0 + slot_x,
                                    min(ry0, ry1), max(ry0, ry1))
                        if row * cols + col < count:
                            taken.append(slot)
                        else:
                            free.append(slot)
    return (blocks, taken, free)


def _zones(bands: List[Band], length: float, refuges: List[Rect]) -> List[Rect]:
    """Work zones: the full length of each zone band, minus the refuges."""
    out: List[Rect] = []
    for band in bands:
        if band.kind != "zone":
            continue
        cuts = sorted((r.x_min, r.x_max) for r in refuges
                      if r.y_min >= band.y_min - 1e-6 and r.y_max <= band.y_max + 1e-6)
        x = -length / 2.0
        piece = 0
        for x0, x1 in cuts:
            if x0 - x > 0.5:
                piece += 1
                out.append(Rect(f"{band.name}_{piece}", x, x0, band.y_min, band.y_max))
            x = max(x, x1)
        if length / 2.0 - x > 0.5:
            piece += 1
            out.append(Rect(f"{band.name}_{piece}", x, length / 2.0,
                            band.y_min, band.y_max))
    return out


def _band_by_name(bands: List[Band], name: str) -> Band:
    for band in bands:
        if band.name == name:
            return band
    raise ValueError(f"no row called {name!r} in factory.rows")


def _nearest_aisle_edge(bands: List[Band], band: Band) -> Tuple[float, float]:
    """The band edge that touches an aisle, and which way the aisle is.

    Returns (y of the edge, +1 if the aisle is on the +y side). A zone
    between two aisles -- the assembly line down the middle -- has no
    single answer, so the caller has to say which side it wants.
    """
    below, above = _band_below(bands, band), _band_above(bands, band)
    if above is not None and above.kind == "aisle":
        return band.y_max, +1.0
    if below is not None and below.kind == "aisle":
        return band.y_min, -1.0
    raise ValueError(f"row {band.name!r} does not touch an aisle; "
                     f"give the station an explicit `side`")


def _stations(cfg: Dict[str, Any], bands: List[Band]) -> List[Station]:
    """Place the equipment listed in configs/factory.yaml.

    Two rows per work zone. `align: aisle` puts the machine close enough
    to the aisle that the person working at it stands right on its edge,
    which is what the Comfort Space has to deal with; `align: wall` puts
    storage against the wall behind them, out of the way.

    x is given in the config, y is derived, so widening an aisle moves
    every machine and every worker with it. A station that has been
    moved in the viewport carries explicit `y` and `yaw_deg` instead,
    written back by isaac_sim/scripts/read_factory_scene.py; those win
    over the derivation, which is what makes the round trip lossless.
    """
    spec = cfg.get("equipment", {})
    if not spec.get("enabled", True):
        return []
    modules = spec.get("modules", {})
    stand_gap = float(spec.get("stand_gap", 0.55))
    worker_offset = float(spec.get("worker_offset", 0.45))
    wall_gap = float(spec.get("wall_gap", 0.15))

    out: List[Station] = []
    for i, item in enumerate(spec.get("stations", []), start=1):
        name = str(item["module"])
        if name not in modules:
            raise ValueError(f"station {i} wants module {name!r}, which is not "
                             f"in equipment.modules")
        dims = modules[name]
        length = float(dims["length"])
        depth = float(dims["depth"])
        height = float(dims["height"])

        band = _band_by_name(bands, str(item["band"]))
        if "side" in item:
            towards = +1.0 if str(item["side"]) in ("north", "+y", "up") else -1.0
            edge = band.y_max if towards > 0 else band.y_min
        else:
            edge, towards = _nearest_aisle_edge(bands, band)

        segments = int(item.get("segments", 1))
        length *= segments

        align = str(item.get("align", "aisle"))
        if "y" in item:
            # Moved by hand. Keep the worker the same distance in front
            # of the machine as they would have been.
            y = float(item["y"])
            worker_y = y + towards * (depth / 2.0 + stand_gap)
        elif align == "aisle":
            # Worker first, machine behind them: the distance that
            # matters is the worker's back to the aisle edge.
            worker_y = edge - towards * worker_offset
            front = worker_y - towards * stand_gap
            y = front - towards * depth / 2.0
        elif align == "wall":
            # Back to the wall, facing the aisle. No worker of its own;
            # the anchor is still useful as somewhere to walk to.
            far = band.y_min if towards > 0 else band.y_max
            y = far + towards * (wall_gap + depth / 2.0)
            worker_y = y + towards * (depth / 2.0 + stand_gap)
        else:
            raise ValueError(f"station {i}: align {align!r}; expected "
                             f"'aisle' or 'wall'")

        yaw = (math.radians(float(item["yaw_deg"])) if "yaw_deg" in item
               else 1.5707963267948966 * (1.0 if towards > 0 else -1.0))
        x = float(item["x"])
        workers = _workers_for(item, x, y, worker_y, towards, depth, stand_gap,
                              align)
        out.append(Station(
            name=str(item.get("name", f"{name}_{i:02d}")),
            module=name, x=x, y=y, yaw=yaw,
            length=length, depth=depth, height=height,
            workers=workers))
    return out


def _overlaps(a: Rect, b: Rect) -> bool:
    return (a.x_min < b.x_max - 1e-6 and a.x_max > b.x_min + 1e-6
            and a.y_min < b.y_max - 1e-6 and a.y_max > b.y_min + 1e-6)


def _workers_for(item: Dict[str, Any], x: float, y: float, worker_y: float,
                 towards: float, depth: float, stand_gap: float,
                 align: str) -> Tuple[Tuple[float, float, float], ...]:
    """Who stands at this machine, and where.

    By default one person in front of it, on the aisle side. A station
    can list several instead, each `along` the machine and on a named
    `side`, which is how an assembly line worked from both aisles ends
    up with people facing each other across it.
    """
    listed = item.get("workers")
    if listed is None:
        if not bool(item.get("worker", align == "aisle")):
            return ()
        return ((x, worker_y, -towards * math.pi / 2.0),)

    out = []
    for spec in listed:
        along = float(spec.get("along", 0.0))
        side = str(spec.get("side", "front"))
        if side in ("north", "+y", "up"):
            sign = 1.0
        elif side in ("south", "-y", "down"):
            sign = -1.0
        elif side == "back":
            sign = -towards
        else:
            sign = towards
        wy = y + sign * (depth / 2.0 + stand_gap)
        # They face the machine, which is the way they are not standing.
        out.append((x + along, wy, -sign * math.pi / 2.0))
    return tuple(out)


def _check_clear(stations: List[Station], refuges: List[Rect],
                 bands: List[Band], columns: List[Tuple[float, float]],
                 column_size: float) -> None:
    """Refuges, aisles and columns have to stay clear.

    A machine that creeps over a refuge or into an aisle only shows up
    as the robot failing to find a way past someone, hours later; a
    machine inside a column looks fine until someone opens the scene.
    Both are config mistakes, so they are caught at build time.
    """
    aisles = [b for b in bands if b.kind == "aisle"]
    half = column_size / 2.0
    for station in stations:
        box = station.footprint
        for keep_clear in refuges:
            if _overlaps(box, keep_clear):
                raise ValueError(
                    f"station {station.name} at x {station.x:+.2f} overlaps "
                    f"{keep_clear.name}; move it along x")
        for other in stations:
            if other is station or other.name <= station.name:
                continue
            if _overlaps(box, other.footprint):
                raise ValueError(
                    f"station {station.name} overlaps {other.name}; "
                    f"they cannot both be there")
        for cx, cy in columns:
            column = Rect("column", cx - half, cx + half, cy - half, cy + half)
            if _overlaps(box, column):
                raise ValueError(
                    f"station {station.name} at x {station.x:+.2f} runs into "
                    f"the column at ({cx:+.1f}, {cy:+.1f}); move it along x")
        for aisle in aisles:
            if box.y_min < aisle.y_max - 1e-6 and box.y_max > aisle.y_min + 1e-6:
                raise ValueError(
                    f"station {station.name} sticks into {aisle.name}; "
                    f"the aisle has to stay clear")


def _waypoints(cfg: Dict[str, Any], bands: List[Band],
               length: float) -> List[Waypoint]:
    """Start and goal, down the middle of whichever aisle is named.

    Derived rather than written down, so the run still starts in the
    aisle after the aisle has been moved or widened.
    """
    spec = cfg.get("route")
    if not spec:
        return []
    band = _band_by_name(bands, str(spec.get("aisle", "aisle_north")))
    margin = float(spec.get("end_margin", 1.0))
    y = band.centre + float(spec.get("offset", 0.0))
    return [Waypoint("start", -length / 2.0 + margin, y),
            Waypoint("goal", length / 2.0 - margin, y)]


_LAYOUT_CACHE: Dict[int, Tuple[Any, "BayLayout"]] = {}


def build_layout(cfg: Dict[str, Any]) -> BayLayout:
    """The whole bay, derived from configs/factory.yaml.

    Cached against the config object it came from. The yield controller
    asks for the layout several times a second, and rebuilding fourteen
    machines each time to find out where the aisles are is waste. The
    config is loaded once per process and never mutated, so identity is
    a safe key; the config is kept in the cache so its id cannot be
    reused by something else.
    """
    hit = _LAYOUT_CACHE.get(id(cfg))
    if hit is not None and hit[0] is cfg:
        return hit[1]
    layout = _build_layout(cfg)
    _LAYOUT_CACHE[id(cfg)] = (cfg, layout)
    return layout


def _build_layout(cfg: Dict[str, Any]) -> BayLayout:
    f = cfg["factory"]
    pitch = float(f["column_pitch"])
    length = float(f["bays"]) * pitch
    width = float(f["width"])

    bands = _bands(f, width)
    refuges = _refuges(f, bands, length)
    crosses = _crosses(f, bands)
    columns = _columns(length, width, pitch, float(f.get("column_inset", 0.3)))
    stations = _stations(cfg, bands)
    blocks, taken, free = _cart_bays(f, bands, length)
    # A free slot IS a refuge. Everything that asks the layout where
    # the robot may pull over -- the yield policy, the occupancy map,
    # the floor paint -- goes on working unchanged.
    refuges = refuges + free
    carts = _cart_zones(f) + blocks
    _check_clear(stations, refuges + crosses + carts, bands, columns,
                 float(f["column_size"]))
    return BayLayout(
        length=length,
        width=width,
        bands=bands,
        columns=columns,
        # Cut by the blocks, not by the slots: a block is one hole in
        # the racking, and cutting slot by slot would leave 0 mm-wide
        # slivers of work zone between them.
        zones=_zones(bands, length, refuges + carts),
        refuges=refuges,
        crosses=crosses,
        stations=stations,
        waypoints=_waypoints(f, bands, length),
        cart_zones=carts,
        cart_slots_taken=taken,
        cart_slots_free=free,
    )


@dataclass(frozen=True)
class AisleReport:
    """How much room is left in an aisle once the people are in it."""
    aisle: str
    radius: float
    free: float
    at_x: float
    needed: float

    @property
    def fits(self) -> bool:
        return self.free >= self.needed


def aisle_clearance(cfg: Dict[str, Any], layout: BayLayout,
                    radius: float, step: float = 0.1) -> List[AisleReport]:
    """The narrowest gap left in each aisle by the workers beside it.

    A worker stands `worker_offset` back from the aisle edge, so their
    body is clear of it; their Comfort Space is not. Whatever of it
    reaches past the edge is width the robot has to give up, from one
    side or from both at once where two workers face each other across
    the aisle.

    This is what decides the radii in a 1.5 m aisle, and it is cheap
    enough to print at every build, so a radius that quietly seals the
    bay cannot go unnoticed.
    """
    amr = cfg.get("amr", {})
    needed = float(amr.get("footprint_width", 0.48)) + 2.0 * float(
        cfg.get("nav2", {}).get("inflation_margin", 0.05))

    reports: List[AisleReport] = []
    for band in layout.aisles:
        worst, worst_x = band.depth, 0.0
        x = layout.x_min
        while x <= layout.x_max + 1e-9:
            low, high = band.y_min, band.y_max
            for station in layout.stations:
                # Every person at the machine, not just the first. A
                # belt with five posts along it pinches the aisle at
                # five places, and counting only one of them reported a
                # bay as roomier than it is.
                for wx, wy, _facing in station.workers:
                    if abs(wx - x) >= radius:
                        continue
                    # How far the circle reaches at this x.
                    reach = math.sqrt(max(radius ** 2 - (wx - x) ** 2, 0.0))
                    if wy >= band.y_max:
                        high = min(high, wy - reach)
                    elif wy <= band.y_min:
                        low = max(low, wy + reach)
                    else:
                        # Standing in the aisle: take the wider of the
                        # two sides they leave.
                        if (wy - band.y_min) > (band.y_max - wy):
                            low = max(low, wy + reach)
                        else:
                            high = min(high, wy - reach)
            free = max(high - low, 0.0)
            if free < worst:
                worst, worst_x = free, x
            x += step
        reports.append(AisleReport(band.name, radius, worst, worst_x, needed))
    return reports


def clearance_summary(cfg: Dict[str, Any], layout: BayLayout) -> str:
    """One line per policy radius, for the build log."""
    comfort = cfg.get("comfort_space", {})
    radii = {"static": float(comfort.get("static_radius", 1.2))}
    for state, value in (comfort.get("state_radius", {}) or {}).items():
        radii[f"state_adaptive/{state}"] = float(value)

    lines = ["aisle clearance with people at the machines:"]
    for label, radius in sorted(radii.items(), key=lambda kv: kv[1]):
        for report in aisle_clearance(cfg, layout, radius):
            verdict = "fits" if report.fits else "BLOCKED"
            lines.append(
                f"  r {radius:4.2f}  {report.aisle:<12s} free "
                f"{report.free:4.2f} m at x {report.at_x:+5.1f}  "
                f"(needs {report.needed:.2f}) {verdict}   [{label}]")
    return "\n".join(lines)


def drivable_rects(layout: BayLayout) -> List[Rect]:
    """Where the AMR belongs: the aisles and crossings, plus the refuges.

    The work zones are floor the robot can physically cross, and the
    occupancy map says so, but they are where people work. Pulling off
    the aisle into a refuge is the manoeuvre the bay is laid out for;
    parking between two benches is not.

    Cart staging blocks are deliberately NOT here. The robot can be
    sent into one to collect a cart, but it must never choose one as
    somewhere to wait: a block that is doing its job has carts parked
    in it, and standing off the aisle inside another vehicle is not
    getting out of the way.
    """
    return layout.in_aisle_space + list(layout.refuges)


def navigable_rects(layout: BayLayout) -> List[Rect]:
    """Every piece of floor the robot is allowed to drive on.

    The lanes and crossings, the free slots in the staging blocks, and
    any staging block that has no slots of its own (a block written
    straight into the config, which is the older way of describing
    one).

    A block WITH slots is deliberately not here in one piece. Opening
    the whole thing makes its frontage a second corridor beside the
    aisle, and the planner uses it: measured, the route left the lane,
    ran the length of a block through its empty slots, clipped the
    inflation of a parked cart and ended with the protective field
    stopping the robot for good. The floor a robot may use inside a
    block is the empty slots, one at a time, entered from the lane.
    """
    slotted = {(z.x_min, z.y_min, z.x_max, z.y_max)
               for slot in layout.cart_slots_taken + layout.cart_slots_free
               for z in layout.cart_zones
               if z.x_min <= slot.x_min + 1e-6 and slot.x_max <= z.x_max + 1e-6
               and z.y_min <= slot.y_min + 1e-6 and slot.y_max <= z.y_max + 1e-6}
    loose = [z for z in layout.cart_zones
             if (z.x_min, z.y_min, z.x_max, z.y_max) not in slotted]
    return drivable_rects(layout) + loose


def validate_factory(cfg: Dict[str, Any], layout: BayLayout) -> List[str]:
    """Check a factory run's start, goal and people. Returns problems.

    comfort_core/layout.validate() only knows about the corridor and
    steps aside when it is switched off, so without this a factory
    config would build with nobody checking that the robot starts
    somewhere it can actually drive.
    """
    problems: List[str] = []
    bay = Rect("bay", layout.x_min, layout.x_max, layout.y_min, layout.y_max)
    aisles = layout.aisles
    radius = float(cfg.get("humans", {}).get("radius", 0.25))

    def inside(x: float, y: float, margin: float = 0.0) -> bool:
        return (bay.x_min + margin <= x <= bay.x_max - margin
                and bay.y_min + margin <= y <= bay.y_max - margin)

    ends = []
    for name in ("start", "goal"):
        value = cfg.get("navigation", {}).get(name)
        if value is None:
            continue
        x, y = (float(v) for v in value)
        ends.append((name, x, y))
        if not inside(x, y):
            problems.append(f"navigation.{name} ({x}, {y}) is outside the bay")
        elif not (any(b.y_min <= y <= b.y_max for b in aisles)
                  or any(r.x_min <= x <= r.x_max and r.y_min <= y <= r.y_max
                         for r in layout.cart_zones)):
            problems.append(
                f"navigation.{name} ({x}, {y}) is neither in an aisle nor in "
                f"a cart block; the aisles are "
                + ", ".join(f"{b.name} y[{b.y_min:+.2f},{b.y_max:+.2f}]"
                            for b in aisles))
        for station in layout.stations:
            box = station.footprint
            if (box.x_min <= x <= box.x_max and box.y_min <= y <= box.y_max):
                problems.append(
                    f"navigation.{name} ({x}, {y}) is inside {station.name}")

    for key, hx, hy in _human_positions(cfg):
        if not inside(hx, hy, radius):
            problems.append(f"humans.{key} at ({hx}, {hy}) is outside the bay")
        for station in layout.stations:
            box = station.footprint
            if (box.x_min - radius < hx < box.x_max + radius
                    and box.y_min - radius < hy < box.y_max + radius):
                problems.append(
                    f"humans.{key} at ({hx}, {hy}) is standing inside "
                    f"{station.name}")
        for name, ex, ey in ends:
            if math.hypot(ex - hx, ey - hy) < radius + 0.5:
                problems.append(
                    f"humans.{key} at ({hx}, {hy}) is on top of "
                    f"navigation.{name}")
        for refuge in layout.refuges:
            if (refuge.x_min - radius < hx < refuge.x_max + radius
                    and refuge.y_min - radius < hy < refuge.y_max + radius):
                problems.append(
                    f"humans.{key} at ({hx}, {hy}) is standing in "
                    f"{refuge.name}; the robot pulls off the aisle into it")

    # The machines' own workers, whether or not the experiment places a
    # person there: a refuge with a workbench in front of it is not one.
    for station in layout.stations:
        if not station.has_worker:
            continue
        wx, wy = station.worker
        for refuge in layout.refuges:
            if (refuge.x_min < wx < refuge.x_max
                    and refuge.y_min < wy < refuge.y_max):
                problems.append(
                    f"the worker at {station.name} stands in {refuge.name}; "
                    f"move the machine along x")
    return problems


def _human_positions(cfg: Dict[str, Any]) -> List[Tuple[str, float, float]]:
    """humans entries that have a position, without importing layout.py."""
    humans = cfg.get("humans", {})
    out = []
    for key in sorted(humans):
        value = humans[key]
        if (isinstance(value, dict) and "position" in value
                and value.get("enabled", True)):
            x, y = value["position"]
            out.append((key, float(x), float(y)))
    return out


def summary(layout: BayLayout) -> str:
    """A few lines for the build log, so a wrong number shows up early."""
    lines = [f"bay {layout.length:.1f} x {layout.width:.1f} m, "
             f"{len(layout.columns)} columns"]
    for band in layout.bands:
        lines.append(f"  {band.kind:<5s} {band.name:<16s} "
                     f"y {band.y_min:+.2f} .. {band.y_max:+.2f}  "
                     f"({band.depth:.2f} m)")
    if layout.stations:
        workers = sum(len(s.workers) for s in layout.stations)
        lines.append(f"  {workers} workers at machines")
        kinds: Dict[str, int] = {}
        for station in layout.stations:
            kinds[station.module] = kinds.get(station.module, 0) + 1
        lines.append("  equipment: " + ", ".join(
            f"{n}x {k}" for k, n in sorted(kinds.items())))
    if layout.waypoints:
        lines.append("  route: " + " -> ".join(
            f"{w.name} ({w.x:+.1f}, {w.y:+.1f})" for w in layout.waypoints))
    if layout.crosses:
        lines.append("  cross aisles at x " + ", ".join(
            f"{(r.x_min + r.x_max) / 2:+.1f}" for r in layout.crosses))
    lines.append(f"  {len(layout.zones)} work zones, {len(layout.refuges)} refuges "
                 f"({', '.join(f'{r.size[0]:.1f}x{r.size[1]:.1f}' for r in layout.refuges[:3])}"
                 f"{' ...' if len(layout.refuges) > 3 else ''})")
    return "\n".join(lines)
