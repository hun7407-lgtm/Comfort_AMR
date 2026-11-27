"""Per-trial metrics and failure classes for the final Stage 1-3 experiments.

Pure Python 3 + numpy (+ Pillow for the map). Reads one trial folder written
by scripts/final_run.py; never writes into it.

    from comfort_core.final_metrics import analyse_trial
    rec = analyse_trial(Path("results/batches/261125_final_stage1/coarse/trials/s001_B2_r120"))

Fixed on 2026-11-25 BEFORE any Stage 1 data was seen (docs/FINAL_261125_PLAN.md).

Sources: trials.csv (AMR 1, nav2_goal.py), fleet_trials.csv (a1, a2,
fleet_goals.py), trial_log.jsonl (scripts/trial_logger.py, 5 Hz on the sim
clock: poses, people with declared/observed state and their own Comfort
extents F/S/R, collision monitor action, AMR 1 gate state, every /plan).

Distances
    clearance  = person CENTRE -> nearest point of the AMR footprint rectangle
                 (0.70 x 0.48 m); the same quantity the Comfort parameter is
                 defined in (comfort_space.robot_grow).
    direction  = bearing of the AMR centre in the person's frame:
                 |a| <= 45 deg front, >= 135 deg rear, else side. Heading =
                 velocity direction while the person walks (speed >= 0.15 m/s),
                 otherwise the configured facing_deg (held last travel
                 direction if the config has none).
    state      = observed state `o` of comfort_publisher (declared state,
                 `moving` replaced by its speed rule).

Interaction (per person x AMR): a contiguous stretch with clearance < 2.5 m.
    min_clearance, t, state/direction at the minimum, AMR speed there,
    penetration = max over the stretch of (Comfort extent of that person toward
    the AMR - clearance), > 0 only when the person had a Comfort Space;
    intrusion_s = time with penetration > 0; hold_s = time the AMR stood
    (gate YIELDING for AMR 1; speed < 0.05 m/s for a1/a2) during the stretch.

Spatial (1 Hz, policies with Comfort Spaces): rasterised at 0.05 m on the
plannable floor (comfort_core.layout.plannable_area minus map obstacles):
    corridor_intrusion_m2  floor area covered by >= 1 Comfort Space
    overlap_m2             floor area covered by >= 2 Comfort Spaces
    overlap_humans         most Comfort Spaces covering one cell
    residual_width_m       per AMR: narrowest free width across its current plan
                           over the next 4 m (0.2 m steps): the aisle cross-section
                           (walls / map obstacles) through each path point, longest
                           run not covered by a STANDING person's Comfort Space
                           (speed < 0.15); residual_width_all_m counts walkers too.

Hold / yield per AMR: AMR 1 = gate YIELDING seconds (trials.csv
yield_seconds); a1/a2 = seconds standing (speed < 0.05) between set-off and
the end of observation, outside their goal tolerance.

Classes, per AMR, in this order:
    success      SUCCESS (hold < 1 s), NORMAL_YIELD (1-10 s), TEMPORARY_HOLD (>= 10 s)
    failure      window = the 12 s before the AMR's verdict
      NAV2_ABORT                Nav2 aborted / planning failure
      PROTECTIVE_STOP           collision monitor STOP >= 50 % of the window
      comfort  (blocking set = people whose Comfort Space, grown by the AMR
               half-width + 0.05 m, meets the AMR's plan within 3 m ahead or
               holds the AMR's position; needs >= 50 % of the window)
        HUMAN_MOTION_BLOCK      a blocking person walked (>= 0.15 m/s) for >= 50 %
        COMFORT_OVERLAP_BLOCK   >= 2 people block
        SINGLE_HUMAN_COMFORT_BLOCK  exactly one standing / engaged person blocks
        PERSISTENT_COMFORT_HOLD AMR 1 gate YIELDING >= 50 % but no blocking set found
      ROBOT_ROBOT_BLOCK         another AMR's centre within 1.2 m for >= 50 %
      HUMAN_MOTION_BLOCK        a person's body within 0.35 m of the footprint
      TIMEOUT                   time limit reached while still moving
      UNKNOWN
    COMFORT_CLASSES (comfort-induced) = PERSISTENT_COMFORT_HOLD,
    COMFORT_OVERLAP_BLOCK, SINGLE_HUMAN_COMFORT_BLOCK.
"""

from __future__ import annotations

import csv
import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

COMFORT_CLASSES = ("PERSISTENT_COMFORT_HOLD", "COMFORT_OVERLAP_BLOCK", "SINGLE_HUMAN_COMFORT_BLOCK")
# NORMAL_YIELD / TEMPORARY_HOLD: legacy names (a gate could YIELD). Under
# control.mode nav2_only there is no yield; the same idle bands are
# SUCCESS_IDLE / SUCCESS_LONG_IDLE (standing = speed < 0.05 m/s, Nav2's own
# behaviour or the collision monitor, never a custom yield).
SUCCESS_CLASSES = ("SUCCESS", "NORMAL_YIELD", "TEMPORARY_HOLD", "SUCCESS_IDLE", "SUCCESS_LONG_IDLE")
CM_STOP = 1
NEAR_M = 2.5
RES = 0.05
WINDOW_S = 12.0
STATES = ("moving", "stationary", "station_engaged")
DIRECTIONS = ("front", "side", "rear")


# ------------------------------------------------------------------ loading
def _rows(p: Path) -> List[Dict[str, str]]:
    return list(csv.DictReader(p.open())) if p.exists() else []


def load(tdir: Path) -> Dict[str, Any]:
    done = json.loads((tdir / "DONE").read_text())
    ticks, plans, fleet_ev = [], [], []
    log = tdir / "trial_log.jsonl"
    for line in (log.read_text().splitlines() if log.exists() else []):
        try:
            o = json.loads(line)
        except ValueError:
            continue
        ev = o.get("ev")
        if ev == "tick":
            ticks.append(o)
        elif ev == "plan" and o.get("t") is not None:
            plans.append(o)
        elif ev == "fleet":
            fleet_ev.append(o)
    amr1 = _rows(tdir / "trials.csv")
    return {"done": done, "amr1": amr1[-1] if amr1 else None, "fleet": _rows(tdir / "fleet_trials.csv"),
            "ticks": ticks, "plans": plans, "fleet_ev": fleet_ev}


@lru_cache(maxsize=256)
def config(cond: str, cfg_path: str):
    from comfort_core import load_config
    return load_config(scenario=cond, common_path=cfg_path)


# ------------------------------------------------------------------ geometry
def rect_clearance(pose, hl, hw, px, py) -> float:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    u, v = dx * c + dy * s, -dx * s + dy * c
    qu, qv = abs(u) - hl, abs(v) - hw
    if qu <= 0 and qv <= 0:
        return max(qu, qv)
    return math.hypot(max(qu, 0.0), max(qv, 0.0))


def sector(angle: float) -> str:
    a = abs(math.atan2(math.sin(angle), math.cos(angle)))
    return "front" if a <= math.pi / 4 else ("rear" if a >= 3 * math.pi / 4 else "side")


def extent_toward(F: float, S: float, R: float, rel: float) -> float:
    """Distance from the person to the boundary of their (offset) ellipse along
    `rel` (radians from their heading). along=(F+R)/2, across=S, offset=(F-R)/2."""
    a, b, d = (F + R) / 2.0, S, (F - R) / 2.0
    if a <= 0 or b <= 0:
        return 0.0
    ux, uy = math.cos(rel), math.sin(rel)
    # solve ((t*ux - d)/a)^2 + (t*uy/b)^2 = 1 for t > 0
    A = (ux / a) ** 2 + (uy / b) ** 2
    B = -2 * d * ux / a ** 2
    C = (d / a) ** 2 - 1
    disc = B * B - 4 * A * C
    if disc < 0:
        return 0.0
    return max((-B + math.sqrt(disc)) / (2 * A), 0.0)


def wilson(k: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


# ------------------------------------------------------------------ floor raster
class Floor:
    """Plannable floor (aisles) minus map obstacles, at RES."""

    def __init__(self, cfg):
        from comfort_core.layout import plannable_area
        rects = plannable_area(cfg) or []
        self.x0 = min(r[0] for r in rects) - 0.5
        self.y0 = min(r[2] for r in rects) - 0.5
        x1 = max(r[1] for r in rects) + 0.5
        y1 = max(r[3] for r in rects) + 0.5
        self.nx, self.ny = int((x1 - self.x0) / RES) + 1, int((y1 - self.y0) / RES) + 1
        xs = self.x0 + (np.arange(self.nx) + 0.5) * RES
        ys = self.y0 + (np.arange(self.ny) + 0.5) * RES
        self.X, self.Y = np.meshgrid(xs, ys)
        free = np.zeros((self.ny, self.nx), bool)
        for a, b, c, d in rects:
            free |= (self.X >= a) & (self.X <= b) & (self.Y >= c) & (self.Y <= d)
        free &= ~self._map_obstacles(cfg)
        self.free = free

    def _map_obstacles(self, cfg):
        try:
            import yaml
            from PIL import Image
            from comfort_core import nav2_map, resolve_path
            my = resolve_path(nav2_map(cfg))
            meta = yaml.safe_load(Path(my).read_text())
            img = np.array(Image.open(Path(my).parent / meta["image"]).convert("L"))
            res = float(meta["resolution"])
            ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
            h = img.shape[0]
            col = ((self.X - ox) / res).astype(int)
            row = (h - 1 - ((self.Y - oy) / res)).astype(int)
            ok = (col >= 0) & (col < img.shape[1]) & (row >= 0) & (row < h)
            occ = np.zeros_like(ok)
            occ[ok] = img[row[ok], col[ok]] < 128
            return occ
        except Exception:
            return np.zeros((self.ny, self.nx), bool)

    def shape_mask(self, x, y, h, F, S, R, grow=0.0):
        a, b, d = (F + R) / 2.0 + grow, S + grow, (F - R) / 2.0
        cx, cy = x + d * math.cos(h), y + d * math.sin(h)
        r = max(a, b)
        i0 = max(int((cy - r - self.y0) / RES), 0)
        i1 = min(int((cy + r - self.y0) / RES) + 1, self.ny)
        j0 = max(int((cx - r - self.x0) / RES), 0)
        j1 = min(int((cx + r - self.x0) / RES) + 1, self.nx)
        m = np.zeros((self.ny, self.nx), bool)
        if i0 >= i1 or j0 >= j1:
            return m
        X, Y = self.X[i0:i1, j0:j1] - cx, self.Y[i0:i1, j0:j1] - cy
        c, s = math.cos(h), math.sin(h)
        u, v = X * c + Y * s, -X * s + Y * c
        m[i0:i1, j0:j1] = (u / a) ** 2 + (v / b) ** 2 <= 1.0
        return m

    def blocked(self, x, y, extra):
        j, i = int((x - self.x0) / RES), int((y - self.y0) / RES)
        if not (0 <= i < self.ny and 0 <= j < self.nx):
            return True
        return (not self.free[i, j]) or bool(extra[i, j])

    def width_at(self, x, y, nx_, ny_, extra, reach=3.0):
        """Free width of the aisle cross-section through (x, y) along (nx_, ny_):
        the floor segment containing the point (bounded by walls / map
        obstacles, Comfort Spaces ignored), then the longest run in it that no
        Comfort Space covers. None when the point is off the floor."""
        ts = np.arange(-reach, reach + RES / 2, RES)
        floor, comfort = [], []
        for t in ts:
            px, py = x + nx_ * t, y + ny_ * t
            j, i = int((px - self.x0) / RES), int((py - self.y0) / RES)
            inside = 0 <= i < self.ny and 0 <= j < self.nx
            floor.append(inside and bool(self.free[i, j]))
            comfort.append(inside and bool(extra[i, j]))
        c = len(ts) // 2
        if not floor[c]:
            return None
        a = c
        while a > 0 and floor[a - 1]:
            a -= 1
        b = c
        while b < len(ts) - 1 and floor[b + 1]:
            b += 1
        best = run = 0
        for k in range(a, b + 1):
            run = run + 1 if not comfort[k] else 0
            best = max(best, run)
        return best * RES


@lru_cache(maxsize=8)
def floor_for(cond: str, cfg_path: str) -> Floor:
    return Floor(config(cond, cfg_path))


# ------------------------------------------------------------------ helpers
def facing(cfg) -> Dict[str, Optional[float]]:
    out = {}
    for k, v in (cfg.get("humans") or {}).items():
        if isinstance(v, dict) and v.get("facing_deg") is not None:
            out[k] = math.radians(float(v["facing_deg"]))
    return out


def headings(ticks, cfg) -> List[Dict[str, Optional[float]]]:
    face = facing(cfg)
    last: Dict[str, float] = {}
    out = []
    for i, tk in enumerate(ticks):
        h = {}
        for k, p in (tk.get("people") or {}).items():
            j = max(0, i - 5)
            q = (ticks[j].get("people") or {}).get(k)
            if q and float(p.get("v", 0.0)) >= 0.15 and (p["x"] - q["x"]) ** 2 + (p["y"] - q["y"]) ** 2 > 1e-4:
                last[k] = math.atan2(p["y"] - q["y"], p["x"] - q["x"])
                h[k] = last[k]
            else:
                h[k] = face.get(k, last.get(k))
        out.append(h)
    return out


def speeds_of(ticks, name) -> List[float]:
    out = []
    for i, tk in enumerate(ticks):
        j = max(0, i - 2)
        a, b = (ticks[j].get("amr") or {}).get(name), (tk.get("amr") or {}).get(name)
        dt = tk["t"] - ticks[j]["t"]
        out.append(math.hypot(b[0] - a[0], b[1] - a[1]) / dt if a and b and dt > 0 else 0.0)
    return out


def plan_at(plans, name, t):
    best = None
    for p in plans:
        if p["amr"] == name and p["t"] <= t:
            best = p
    return best


def path_ahead(pts, pose, ahead=3.0, step=0.2):
    if not pts or len(pts) < 2:
        return []
    k = min(range(len(pts)), key=lambda i: (pts[i][0] - pose[0]) ** 2 + (pts[i][1] - pose[1]) ** 2)
    out, gone = [], 0.0
    for a, b in zip(pts[k:], pts[k + 1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-6:
            continue
        n = max(1, int(seg / step))
        for i in range(n):
            f = i / n
            out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, (b[0] - a[0]) / seg, (b[1] - a[1]) / seg))
        gone += seg
        if gone >= ahead:
            break
    return out


# ------------------------------------------------------------------ main analysis
def analyse_trial(tdir: Path, spatial: bool = True) -> Dict[str, Any]:
    D = load(tdir)
    done, ticks = D["done"], D["ticks"]
    cfg = config(done["condition"], done["config"])
    from comfort_core.control_mode import nav2_only as _nav2_only
    nav2only = _nav2_only(cfg)
    hl, hw = float(cfg["amr"]["footprint_length"]) / 2, float(cfg["amr"]["footprint_width"]) / 2
    grow = hw
    t_start = float(cfg["navigation"].get("trial_start_sim_time", 0.0))
    names = ["AMR1"] + [f["robot"] for f in D["fleet"]]
    heads = headings(ticks, cfg) if ticks else []
    spd = {n: speeds_of(ticks, n) for n in names}
    has_space = any("F" in p for tk in ticks for p in (tk.get("people") or {}).values())

    # ---- per-AMR verdict times and outcome
    amrs: Dict[str, Dict[str, Any]] = {}
    a1 = D["amr1"] or {}
    res1 = a1.get("result") or "NO_ROW"
    t_end_log = ticks[-1]["t"] if ticks else 0.0
    try:
        t_v1 = t_start + float(a1.get("travel_time") or 0.0)
    except ValueError:
        t_v1 = t_end_log
    amrs["AMR1"] = {"result": res1, "t_verdict": t_v1, "t_begin": t_start, "travel_time": _f(a1.get("travel_time")),
                    "path_length": _f(a1.get("travel_distance")), "hold_s": _f(a1.get("yield_seconds")) or 0.0,
                    "contact_s": _f(a1.get("contact_seconds")) or 0.0,
                    "protective_stop_s": _f(a1.get("safety_stop_only_seconds")) or 0.0,
                    "planning_success": a1.get("planning_success"),
                    "nav2_abort": (a1.get("deadlock_mechanism") == "nav2_abort") or res1 == "PLANNING_FAILURE",
                    "min_human_m": None, "min_robot_m": None}
    for f in D["fleet"]:
        n = f["robot"]
        tv = _fleet_fail_time(D["fleet_ev"], n) if f["result"] != "SUCCESS" else _fleet_event(D["fleet_ev"], n, "finished:")
        set_off = t_start + (_f(f.get("set_off_at_s")) or 0.0)
        tv = tv if tv is not None else t_end_log
        amrs[n] = {"result": f["result"], "t_verdict": tv, "t_begin": set_off,
                   "travel_time": round(tv - set_off, 2), "path_length": _f(f.get("travel_distance_m")),
                   "hold_s": None, "contact_s": _f(f.get("contact_s")) or 0.0,
                   "protective_stop_s": _f(f.get("safety_stop_s")) or 0.0,
                   "planning_success": None, "nav2_abort": f["result"] == "ABORTED",
                   "set_off": set_off, "legs_done": f.get("legs_done"), "legs_total": f.get("legs_total"),
                   "min_human_m": None, "min_robot_m": _f(f.get("min_gap_fleet_m"))}

    # ---- plans: count, replans, length of first plan
    for n in names:
        ps = [p for p in D["plans"] if p["amr"] == n and p.get("n", 0) > 1]
        ends = {}
        replans = 0
        prev = None
        for p in ps:
            end = tuple(p["pts"][-1]) if p["pts"] else None
            if prev is not None and end == prev[0]:
                # same goal: a replan counts when the route changed materially
                if _route_changed(prev[1], p["pts"]):
                    replans += 1
            prev = (end, p["pts"])
            ends[end] = ends.get(end, 0) + 1
        amrs[n].update({"plans": len(ps), "replans": replans, "goals_planned": len(ends),
                        "first_plan_m": ps[0]["len"] if ps else None, "planned": bool(ps)})

    # ---- standing time (every AMR, same rule): speed < 0.05 m/s between
    # set-off and verdict. AMR 1's gate seconds stay in gate_yield_s
    # (legacy only: under nav2_only there is no gate, so it is None).
    amrs["AMR1"]["gate_yield_s"] = None if nav2only else amrs["AMR1"]["hold_s"]
    for n in names:
        a = amrs[n]
        hold, prev_t = 0.0, None
        for i, tk in enumerate(ticks):
            t = tk["t"]
            if prev_t is not None and a["t_begin"] <= t <= a["t_verdict"] and spd[n][i] < 0.05:
                hold += t - prev_t
            prev_t = t
        a["hold_s"] = round(hold, 2)
        a["idle_s"] = a["hold_s"]

    # ---- interactions
    inters = []
    for n in names:
        open_ = {}
        for i, tk in enumerate(ticks):
            pose = (tk.get("amr") or {}).get(n)
            if not pose or not (amrs[n]["t_begin"] <= tk["t"] <= amrs[n]["t_verdict"]):
                continue
            gate = tk.get("gate")
            for k, p in (tk.get("people") or {}).items():
                c = rect_clearance(pose, hl, hw, p["x"], p["y"])
                if c < NEAR_M:
                    e = open_.setdefault(k, {"amr": n, "human": k, "t0": tk["t"], "min": 9e9, "pen": 0.0,
                                              "intr_s": 0.0, "hold_s": 0.0, "last_t": tk["t"]})
                    dt = tk["t"] - e["last_t"]
                    e["last_t"] = tk["t"]
                    h = heads[i].get(k)
                    rel = (math.atan2(pose[1] - p["y"], pose[0] - p["x"]) - h) if h is not None else None
                    pen = 0.0
                    if "F" in p and rel is not None:
                        pen = extent_toward(p["F"], p["S"], p["R"], rel) - c
                    if pen > 0:
                        e["intr_s"] += dt
                        e["pen"] = max(e["pen"], pen)
                    standing = (gate == "YIELDING") if (n == "AMR1" and not nav2only) else spd[n][i] < 0.05
                    if standing:
                        e["hold_s"] += dt
                    if c < e["min"]:
                        e.update({"min": c, "t": tk["t"], "state": p.get("o") or p.get("s"),
                                  "direction": sector(rel) if rel is not None else None,
                                  "amr_speed": round(spd[n][i], 3), "human_speed": p.get("v"),
                                  "extent": (round(extent_toward(p["F"], p["S"], p["R"], rel), 3)
                                             if "F" in p and rel is not None else None)})
                elif k in open_:
                    inters.append(_close(open_.pop(k)))
        inters += [_close(e) for e in open_.values()]
    for n in names:
        mine = [e["min_clearance"] for e in inters if e["amr"] == n]
        amrs[n]["min_human_m"] = round(min(mine), 3) if mine else None

    # ---- robot-robot min distance (centre-centre minus footprint, approx)
    for n in names:
        best = None
        for tk in ticks:
            a = (tk.get("amr") or {}).get(n)
            if not (amrs[n]["t_begin"] <= tk["t"] <= amrs[n]["t_verdict"]):
                continue
            for m in names:
                b = (tk.get("amr") or {}).get(m)
                if m == n or not a or not b:
                    continue
                d = math.hypot(a[0] - b[0], a[1] - b[1])
                best = d if best is None else min(best, d)
        amrs[n]["min_robot_centre_m"] = round(best, 3) if best is not None else None

    # ---- spatial (1 Hz)
    sp = {"corridor_intrusion_max_m2": None, "corridor_intrusion_mean_m2": None, "overlap_max_m2": None,
          "overlap_s": 0.0, "overlap_humans_max": 0}
    resid: Dict[str, List[Tuple[float, float]]] = {n: [] for n in names}
    if spatial and ticks and has_space:
        fl = floor_for(done["condition"], done["config"])
        intr, ovl, last_t = [], [], None
        for i, tk in enumerate(ticks):
            if last_t is not None and tk["t"] - last_t < 0.999:
                continue
            dt = 0 if last_t is None else tk["t"] - last_t
            last_t = tk["t"]
            cover = np.zeros((fl.ny, fl.nx), np.uint8)
            for k, p in (tk.get("people") or {}).items():
                if "F" in p:
                    cover += fl.shape_mask(p["x"], p["y"], p.get("h", 0.0), p["F"], p["S"], p["R"]).astype(np.uint8)
            on = cover * fl.free
            a_int = float((on >= 1).sum()) * RES * RES
            a_ovl = float((on >= 2).sum()) * RES * RES
            intr.append(a_int)
            ovl.append(a_ovl)
            if a_ovl > 0.01:
                sp["overlap_s"] += dt
            sp["overlap_humans_max"] = max(sp["overlap_humans_max"], int(on.max()) if on.size else 0)
            extra = cover > 0
            still = np.zeros_like(extra)
            for k, p in (tk.get("people") or {}).items():
                if "F" in p and float(p.get("v", 0.0)) < 0.15:
                    still |= fl.shape_mask(p["x"], p["y"], p.get("h", 0.0), p["F"], p["S"], p["R"])
            for n in names:
                pose = (tk.get("amr") or {}).get(n)
                pl = plan_at(D["plans"], n, tk["t"])
                if not pose or not pl or not (amrs[n]["t_begin"] <= tk["t"] <= amrs[n]["t_verdict"] + 1):
                    continue
                pts = path_ahead(pl["pts"], pose, 4.0)
                ws = [w for w in (fl.width_at(x, y, -uy, ux, still) for x, y, ux, uy in pts) if w is not None]
                wa = [w for w in (fl.width_at(x, y, -uy, ux, extra) for x, y, ux, uy in pts) if w is not None]
                if ws:
                    resid[n].append((tk["t"], round(min(ws), 2), round(min(wa), 2) if wa else None))
        sp["corridor_intrusion_max_m2"] = round(max(intr), 3) if intr else None
        sp["corridor_intrusion_mean_m2"] = round(sum(intr) / len(intr), 3) if intr else None
        sp["overlap_max_m2"] = round(max(ovl), 3) if ovl else None
        sp["overlap_s"] = round(sp["overlap_s"], 1)
    for n in names:
        # standing people's Comfort Spaces only (the structural blockage), and everyone's
        amrs[n]["residual_width_min_m"] = min((w for _, w, _a in resid[n]), default=None)
        amrs[n]["residual_width_all_min_m"] = min((a for _, _w, a in resid[n] if a is not None), default=None)

    # ---- classes
    for n in names:
        amrs[n]["class"], amrs[n]["class_detail"] = _classify(n, amrs[n], ticks, heads, D["plans"], spd,
                                                              hl, hw, grow, names, cfg, nav2only)
        if True:
            w = [x for t, x, _a in resid[n] if amrs[n]["t_verdict"] - WINDOW_S <= t <= amrs[n]["t_verdict"]]
            amrs[n]["residual_width_at_verdict_m"] = min(w) if w else None
    system = all(amrs[n]["class"] in SUCCESS_CLASSES for n in names)
    comfort = [n for n in names if amrs[n]["class"] in COMFORT_CLASSES]
    return {"trial_id": done["trial_id"], "seed": done["seed"], "condition": Path(done["condition"]).stem,
            "infra_failure": done.get("infra_failure"), "system_success": system,
            "comfort_deadlock": bool(comfort), "comfort_deadlock_amrs": comfort,
            "any_failure": not system, "amrs": amrs, "spatial": sp, "interactions": inters,
            "has_comfort_space": has_space, "n_ticks": len(ticks),
            "control_mode": "nav2_only" if nav2only else "legacy_gate"}


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _close(e):
    return {"amr": e["amr"], "human": e["human"], "t0": e["t0"], "t_min": e.get("t"),
            "min_clearance": round(e["min"], 3), "state": e.get("state"), "direction": e.get("direction"),
            "amr_speed": e.get("amr_speed"), "human_speed": e.get("human_speed"), "extent": e.get("extent"),
            "penetration": round(e["pen"], 3), "intrusion_s": round(e["intr_s"], 2), "hold_s": round(e["hold_s"], 2)}


def _route_changed(a, b, tol=0.5) -> bool:
    if not a or not b:
        return False
    # max distance from b's points to a's polyline vertices (thin plans: good enough)
    A = np.array(a)
    worst = 0.0
    for p in b[:: max(1, len(b) // 20)]:
        worst = max(worst, float(np.min(np.hypot(A[:, 0] - p[0], A[:, 1] - p[1]))))
    return worst > tol


def _fleet_event(evs, name, text):
    for e in evs:
        if (e.get("msg") or "").startswith(f"[fleet] {name} ") and text in e["msg"]:
            return e.get("t")
    return None


def _fleet_fail_time(evs, name):
    for e in evs:
        m = (e.get("msg") or "").lower()
        if m.startswith(f"[fleet] {name} ") and ("deadlock" in m or "no progress" in m or "abort" in m
                                                 or "skipp" in m):
            return e.get("t")
    return None


def success_class(hold_s: float, nav2only: bool) -> str:
    h = hold_s or 0.0
    if nav2only:
        return "SUCCESS" if h < 1.0 else "SUCCESS_IDLE" if h < 10.0 else "SUCCESS_LONG_IDLE"
    return "SUCCESS" if h < 1.0 else "NORMAL_YIELD" if h < 10.0 else "TEMPORARY_HOLD"


def _classify(n, a, ticks, heads, plans, spd, hl, hw, grow, names, cfg, nav2only=False):
    res = a["result"]
    if res == "SUCCESS":
        h = a.get("hold_s") or 0.0
        return success_class(h, nav2only), f"{'idle' if nav2only else 'hold'} {h:.1f} s"
    if a.get("nav2_abort") or res in ("ABORTED", "PLANNING_FAILURE"):
        return "NAV2_ABORT", res
    tv = a["t_verdict"]
    win = [(i, tk) for i, tk in enumerate(ticks) if tv - WINDOW_S <= tk["t"] <= tv and (tk.get("amr") or {}).get(n)]
    if not win:
        return "UNKNOWN", "no log in the verdict window"
    N = len(win)
    stop = sum(1 for _, tk in win if (tk.get("cm") or {}).get(n) == CM_STOP)
    if stop >= 0.5 * N:
        return "PROTECTIVE_STOP", f"collision monitor STOP {stop}/{N} ticks"
    if res == "TIMEOUT":
        p0, p1 = win[0][1]["amr"][n], win[-1][1]["amr"][n]
        if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > 0.5:
            return "TIMEOUT", "time limit while still moving"
    block_counts: Dict[str, int] = {}
    moving_counts: Dict[str, int] = {}
    yielding = 0
    robot_near = 0
    body_near = 0
    for i, tk in win:
        pose = tk["amr"][n]
        if n == "AMR1" and not nav2only and tk.get("gate") == "YIELDING":
            yielding += 1
        pl = plan_at(plans, n, tk["t"])
        ahead = path_ahead(pl["pts"], pose, 3.0) if pl else []
        probe = [(pose[0], pose[1])] + [(x, y) for x, y, _, _ in ahead]
        for k, p in (tk.get("people") or {}).items():
            if rect_clearance(pose, hl, hw, p["x"], p["y"]) < 0.35:
                body_near += 1
            if "F" not in p:
                continue
            h = p.get("h", 0.0)
            hit = False
            for x, y in probe:
                rel = math.atan2(y - p["y"], x - p["x"]) - h
                if math.hypot(x - p["x"], y - p["y"]) <= extent_toward(p["F"], p["S"], p["R"], rel) + grow + 0.05:
                    hit = True
                    break
            if hit:
                block_counts[k] = block_counts.get(k, 0) + 1
                if float(p.get("v", 0.0)) >= 0.15:
                    moving_counts[k] = moving_counts.get(k, 0) + 1
        for m in names:
            b = (tk.get("amr") or {}).get(m)
            if m != n and b and math.hypot(pose[0] - b[0], pose[1] - b[1]) < 1.2:
                robot_near += 1
                break
    blockers = [k for k, c in block_counts.items() if c >= 0.5 * N]
    if blockers:
        movers = [k for k in blockers if moving_counts.get(k, 0) >= 0.5 * N]
        det = f"blocking {','.join(sorted(blockers))}"
        if movers:
            return "HUMAN_MOTION_BLOCK", det + f"; walking {','.join(sorted(movers))}"
        if len(blockers) >= 2:
            return "COMFORT_OVERLAP_BLOCK", det
        return "SINGLE_HUMAN_COMFORT_BLOCK", det
    if n == "AMR1" and yielding >= 0.5 * N:
        return "PERSISTENT_COMFORT_HOLD", f"gate YIELDING {yielding}/{N} ticks, no blocking set on the plan"
    if robot_near >= 0.5 * N:
        return "ROBOT_ROBOT_BLOCK", f"another AMR within 1.2 m {robot_near}/{N} ticks"
    if body_near >= 1:
        return "HUMAN_MOTION_BLOCK", "a person's body within 0.35 m of the footprint"
    if res == "TIMEOUT":
        return "TIMEOUT", "time limit"
    return "UNKNOWN", f"result {res}"
