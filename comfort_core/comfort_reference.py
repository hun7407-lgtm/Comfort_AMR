"""External reference model of perceived comfort, from published human studies.

Pure Python (math only). The SAME model scores every policy (B1, Static,
Adaptive), so a policy is never judged by its own geometry.

What it is
----------
A lookup of comfort-vs-passing-distance curves that were FITTED TO HUMAN
RATINGS in published experiments. It predicts what those participants
would have rated a passage with the same closest distance and side. It
is NOT a measurement of comfort in this simulation, and a simulated trial
that scores well is not evidence that a person would be comfortable.
Call the output "literature-predicted comfort", never "comfort".

Sources (only values printed in the papers are used; nothing is read off
a figure, nothing is fitted here)
---------------------------------------------------------------------
[N22a] Neggers, Cuijpers, Ruijten, IJsselsteijn (2022). Determining Shape
       and Size of Personal Space of a Human when Passed by a Robot.
       Int J Soc Robot 14:561-572. doi:10.1007/s12369-021-00805-6
       - participants STANDING STILL on footprints, told to keep shoulders
         straight (not to look back); robot drives a straight line past
         them at 0.35 m/s on the right / front / left / back.
       - Eq. 1: C = 1 + a0 (1 - exp(-d^2 / (2 sigma^2))), 7-point scale
         ("not at all" .. "very much"); d = CENTRE-TO-CENTRE distance
         human-robot at the closest point, in cm; theta = polar angle of
         that point: right 0, front pi/2, left pi, back 3pi/2 (Fig. 7).
       - Table 1 (a0, sigma[cm]) per side, study A humanoid Pepper
         (L 425 W 480 mm), study B custom AGV (L 730 W 650 mm).
       - between the four sides: discrete Fourier interpolation (Fig. 8);
         reproduced here as the exact 4-point trigonometric interpolant.
[N22b] Neggers et al. (2022). The effect of robot speed on comfortable
       passing distances. Front Robot AI 9:915972.
       doi:10.3389/frobt.2022.915972
       - Exp. 1: participants WALKING down a 2.2 m corridor, Pepper passing
         head-on at 0.35 m/s, 50-130 cm. Eq. 2, Table 2 (passing side
         dropped, a1 = 0): sigma 44.2, a0 5.71, a2 (sequence dummy) -10.4.
         The sequence term is a design nuisance; the eta = 0 curve is used.
         CAVEAT (checked 2026-11-26): the paper says comfort 6 <-> 80 cm
         centre-to-centre, but Table 2 with eta = 0 gives C(80 cm) = 5.60
         and C = 6 at ~90 cm. With the sequence term inside the width
         (sigma + a2 eta, eta = 1) C = 6 falls at ~69 cm, so the 80 cm is
         probably an average over sequences. eta = 0 is the WIDER (more
         conservative) curve; the exact Eq. 2 form should be confirmed
         with the authors.
       - Exp. 2 (AGV, 0.35/0.8/1.25/1.7 m/s, passing and overtaking): the
         per-speed a0 and sigma are only in Fig. 10/11, not in the text, so
         they are NOT used. Speed dependence and overtaking are therefore
         not modelled (TODO: obtain the values from the authors).
[J26]  Jafari, Nguyen, Liu (2026). Empirical Prediction of Pedestrian
       Comfort in Mobile Robot-Pedestrian Encounters. ICRA 2026,
       arXiv:2604.13677. Walking pedestrians, robot at 1.4 / 2.8 m/s
       (faster than this AMR). Binary estimators Eq. 7 (D_min >= 1.0 m)
       and Eq. 8 (min PTTC >= 0.7 s). Only Eq. 7 is reported here, as a
       SECONDARY flag outside its speed range. Eq. 8 is not: the PTTC
       formula is not given in that paper (cited to Jafari & Liu 2024).

Distance basis
--------------
The papers use centre-to-centre distance; this project's Comfort Space is
person centre -> AMR footprint edge. The evaluator therefore uses the
logged centres directly (no conversion). Optional `clearance_matched`
basis: d_equiv = d_cc - hw_this_amr + hw_reference_robot, i.e. same
lateral clearance as the reference robot (N22a Fig. 6 argues clearance
is the common variable across robots). Both are reported.

Not covered (honest gaps)
-------------------------
- station_engaged: no study manipulated task engagement. The N22a
  standing participant is used, plus `engaged_extra_m` (default 0.0) as
  an explicit ASSUMPTION knob.
- moving: only head-on passing at 0.35 m/s (N22b Exp. 1), no direction
  dependence, no speed dependence, no overtaking curve.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

# ----------------------------------------------------------------- sources
# theta order: right 0, front pi/2, left pi, back 3pi/2 (N22a Fig. 7).
SIDES = ("right", "front", "left", "back")

REFERENCES: Dict[str, Dict[str, Any]] = {
    "N22a_B": {  # non-humanoid AGV: closest robot type to an industrial AMR
        "cite": "Neggers et al. 2022, Int J Soc Robot 14:561, Table 1, study B (AGV)",
        "human": "standing still", "robot_speed_mps": 0.35,
        "robot_half_width_m": 0.325, "scale": (1, 7),
        "a0": {"right": 5.39, "front": 5.62, "left": 5.39, "back": 4.58},
        "sigma_cm": {"right": 46.14, "front": 44.13, "left": 47.21, "back": 39.11},
        "tested_cm": (60, 100),
    },
    "N22a_A": {  # humanoid Pepper
        "cite": "Neggers et al. 2022, Int J Soc Robot 14:561, Table 1, study A (Pepper)",
        "human": "standing still", "robot_speed_mps": 0.35,
        "robot_half_width_m": 0.24, "scale": (1, 7),
        "a0": {"right": 6.15, "front": 6.15, "left": 6.04, "back": 3.92},
        "sigma_cm": {"right": 47.25, "front": 41.51, "left": 49.52, "back": 35.65},
        "tested_cm": (50, 90),
    },
    "N22b_E1": {  # walking participant, head-on passing, no side dependence
        "cite": "Neggers et al. 2022, Front Robot AI 9:915972, Exp. 1, Table 2 (eta = 0)",
        "human": "walking, passed head-on", "robot_speed_mps": 0.35,
        "robot_half_width_m": 0.24, "scale": (1, 7),
        "a0": {s: 5.71 for s in SIDES},
        "sigma_cm": {s: 44.2 for s in SIDES},
        "tested_cm": (50, 130),
    },
}

# Which reference scores which human state. `None` = no reference.
DEFAULT_STATE_REFERENCE = {"stationary": "N22a_B", "station_engaged": "N22a_B",
                           "moving": "N22b_E1"}

JAFARI_DMIN_M = 1.0   # [J26] Eq. 7


def _tri4(vals: Dict[str, float], theta: float) -> float:
    """Exact trigonometric interpolant through 4 equally spaced samples
    (theta 0, pi/2, pi, 3pi/2): c0 + c1 cos + s1 sin + c2 cos 2theta."""
    v0, v1, v2, v3 = (vals[s] for s in SIDES)
    c0 = (v0 + v1 + v2 + v3) / 4.0
    c1 = (v0 - v2) / 2.0
    s1 = (v1 - v3) / 2.0
    c2 = (v0 - v1 + v2 - v3) / 4.0
    return c0 + c1 * math.cos(theta) + s1 * math.sin(theta) + c2 * math.cos(2.0 * theta)


def params(ref: str, theta: float) -> Tuple[float, float]:
    r = REFERENCES[ref]
    return _tri4(r["a0"], theta), _tri4(r["sigma_cm"], theta)


def comfort(ref: str, d_m: float, theta: float) -> float:
    """Eq. 1 of [N22a]; d in metres (converted to cm), theta in N22a convention."""
    a0, sig = params(ref, theta)
    d = max(0.0, d_m) * 100.0
    return 1.0 + a0 * (1.0 - math.exp(-d * d / (2.0 * sig * sig)))


def distance_for(ref: str, target: float, theta: float) -> Optional[float]:
    """Centre-to-centre distance [m] at which Eq. 1 reaches `target`;
    None when the curve never reaches it (target - 1 >= a0)."""
    a0, sig = params(ref, theta)
    x = (target - 1.0) / a0
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return None
    return sig * math.sqrt(-2.0 * math.log(1.0 - x)) / 100.0


def max_comfort(ref: str, theta: float) -> float:
    return 1.0 + params(ref, theta)[0]


# ------------------------------------------------------------- directions
def theta_from_rel(rel: float) -> float:
    """Our relative bearing (robot direction in the person's frame,
    0 = straight ahead, + = to the person's left) -> N22a theta."""
    return (rel + math.pi / 2.0) % (2.0 * math.pi)


SECTOR_THETA = {"front": math.pi / 2.0, "left": math.pi, "right": 0.0, "rear": 3.0 * math.pi / 2.0}


def sector_of(rel: float) -> str:
    a = abs((rel + math.pi) % (2.0 * math.pi) - math.pi)
    return "front" if a <= math.pi / 4.0 else ("rear" if a >= 3.0 * math.pi / 4.0 else "side")


# ------------------------------------------------ literature-derived extents
def required_extents(target: float, state: str, amr_half_width: float,
                     state_reference: Optional[Dict[str, Optional[str]]] = None,
                     basis: str = "center_to_center", engaged_extra_m: float = 0.0
                     ) -> Dict[str, Optional[float]]:
    """Comfort Space extent (person centre -> AMR edge) per direction for
    which the reference predicts `target` when the AMR passes there with
    its side toward the person: extent = d_cc(target) - amr_half_width.
    side = the larger of left / right. None = the reference never reaches
    the target there, or there is no reference for this state."""
    ref = (state_reference or DEFAULT_STATE_REFERENCE).get(state)
    if ref is None:
        return {"front": None, "side": None, "rear": None}
    shift = 0.0
    if basis == "clearance_matched":
        shift = amr_half_width - REFERENCES[ref]["robot_half_width_m"]
    out: Dict[str, Optional[float]] = {}
    for key, th in (("front", SECTOR_THETA["front"]), ("rear", SECTOR_THETA["rear"])):
        d = distance_for(ref, target, th)
        out[key] = None if d is None else d + shift - amr_half_width
    ds = [distance_for(ref, target, SECTOR_THETA[s]) for s in ("left", "right")]
    out["side"] = None if any(d is None for d in ds) else max(ds) + shift - amr_half_width
    if state == "station_engaged" and engaged_extra_m:
        out = {k: (None if v is None else v + engaged_extra_m) for k, v in out.items()}
    return {k: (None if v is None else round(max(v, 0.0), 4)) for k, v in out.items()}


def clearance_matched_distance(d_cc: float, amr_half_width: float, reference_half_width: float) -> float:
    """Centre distance at which the REFERENCE robot would have the same lateral
    clearance (d_cc - amr_half_width) as this AMR: d_cc - hw_this + hw_ref."""
    return d_cc - amr_half_width + reference_half_width


# ---------------------------------------------------------- trial scoring
def score_passage(ticks: Sequence[Dict[str, Any]], amr: str, person: str,
                  amr_half_width: float, state_reference: Optional[Dict[str, Optional[str]]] = None,
                  heading_of=None, t_from: float = -1e9, t_to: float = 1e9,
                  contact_m: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """Score the encounter of one AMR with one person from trial_log ticks.

    Primary (as in the papers): predicted comfort at the moment of the
    closest centre-to-centre distance, with theta of that point.
    Secondary: the lowest predicted comfort over the encounter (a pass
    that grazes the back before the closest point is caught here),
    minimum TTC (constant-velocity time until the centre distance reaches
    `contact_m` = person radius + AMR half width; None if never on a
    collision course), relative speed, AMR speed. TTC is a kinematic
    indicator; it is NOT the PTTC of [J26] (formula not in that paper).
    `heading_of(i, person)` -> heading [rad] or None; default the logged h.
    """
    state_reference = state_reference or DEFAULT_STATE_REFERENCE
    best = None
    worst = None
    rows = []
    prev = None
    for i, tk in enumerate(ticks):
        t = tk.get("t")
        if t is None or not (t_from <= t <= t_to):
            continue
        pose = (tk.get("amr") or {}).get(amr)
        p = (tk.get("people") or {}).get(person)
        if not pose or not p:
            continue
        h = heading_of(i, person) if heading_of else p.get("h")
        dx, dy = pose[0] - p["x"], pose[1] - p["y"]
        d = math.hypot(dx, dy)
        rel = None if h is None else math.atan2(dy, dx) - h
        rows.append((t, pose, p, d, rel))
    if not rows:
        return None
    vel = []
    for k, (t, pose, p, d, rel) in enumerate(rows):
        j = max(0, k - 1)
        jn = min(len(rows) - 1, k + 1)
        dt = rows[jn][0] - rows[j][0]
        if dt <= 0:
            vel.append((0.0, 0.0, 0.0, 0.0))
            continue
        va = ((rows[jn][1][0] - rows[j][1][0]) / dt, (rows[jn][1][1] - rows[j][1][1]) / dt)
        vh = ((rows[jn][2]["x"] - rows[j][2]["x"]) / dt, (rows[jn][2]["y"] - rows[j][2]["y"]) / dt)
        vel.append((va[0], va[1], vh[0], vh[1]))
    min_ttc = None
    for k, (t, pose, p, d, rel) in enumerate(rows):
        state = p.get("s")
        ref = state_reference.get(state)
        c = comfort(ref, d, theta_from_rel(rel)) if (ref and rel is not None) else None
        if best is None or d < best["d_cc"]:
            best = {"t": t, "d_cc": d, "rel": rel, "state": state, "ref": ref, "C": c, "k": k}
        if c is not None and (worst is None or c < worst["C"]):
            worst = {"t": t, "d_cc": d, "rel": rel, "C": c}
        vax, vay, vhx, vhy = vel[k]
        rvx, rvy = vax - vhx, vay - vhy
        rv2 = rvx * rvx + rvy * rvy
        dx, dy = pose[0] - p["x"], pose[1] - p["y"]
        if contact_m and rv2 > 1e-6:
            # |r + v t| = contact_m, smallest positive root
            b = dx * rvx + dy * rvy
            c0 = dx * dx + dy * dy - contact_m * contact_m
            disc = b * b - rv2 * c0
            if b < 0 and disc >= 0:
                tc = (-b - math.sqrt(disc)) / rv2
                if tc >= 0 and (min_ttc is None or tc < min_ttc["ttc_s"]):
                    min_ttc = {"ttc_s": round(tc, 3), "t": t, "d_cc": round(math.hypot(dx, dy), 3)}
    k = best["k"]
    vax, vay, vhx, vhy = vel[k]
    rel = best["rel"]
    out = {
        "state": best["state"], "reference": best["ref"],
        "t_closest": round(best["t"], 3),
        "min_cc_m": round(best["d_cc"], 4),
        "min_edge_m_approx": round(best["d_cc"] - amr_half_width, 4),
        "rel_deg": None if rel is None else round(math.degrees((rel + math.pi) % (2 * math.pi) - math.pi), 1),
        "sector": None if rel is None else sector_of(rel),
        "theta_n22": None if rel is None else round(theta_from_rel(rel), 4),
        "predicted_comfort_at_closest": None if best["C"] is None else round(best["C"], 3),
        "predicted_comfort_min": None if worst is None else round(worst["C"], 3),
        "amr_speed_at_closest_mps": round(math.hypot(vax, vay), 3),
        "human_speed_at_closest_mps": round(math.hypot(vhx, vhy), 3),
        "relative_speed_at_closest_mps": round(math.hypot(vax - vhx, vay - vhy), 3),
        "min_ttc": min_ttc, "contact_m": contact_m,
        "jafari_dmin_ok": best["d_cc"] >= JAFARI_DMIN_M,
    }
    if best["ref"]:
        r = REFERENCES[best["ref"]]
        lo, hi = r["tested_cm"]
        out["inside_tested_range"] = lo <= best["d_cc"] * 100.0 <= hi
        out["amr_speed_vs_reference"] = round(out["amr_speed_at_closest_mps"] - r["robot_speed_mps"], 3)
        # documented basis: d_equiv = d_cc - hw_this_amr + hw_reference_robot (same lateral
        # clearance as the reference robot). Fixed 2026-11-27: this line used d_cc + (hw_this - hw_ref),
        # the opposite sign. Secondary metric only; the primary predicted_comfort_at_closest is unchanged.
        d_equiv = clearance_matched_distance(best["d_cc"], amr_half_width, r["robot_half_width_m"])
        out["d_equiv_clearance_matched_m"] = round(d_equiv, 4)
        out["predicted_comfort_clearance_matched"] = (
            None if rel is None else round(comfort(best["ref"], d_equiv, theta_from_rel(rel)), 3))
    return out


def meets(score: Optional[Dict[str, Any]], target: float, key: str = "predicted_comfort_at_closest") -> Optional[bool]:
    if not score or score.get(key) is None:
        return None
    return score[key] >= target - 1e-9


def describe() -> List[str]:
    lines = []
    for k, r in REFERENCES.items():
        lines.append(f"{k}: {r['cite']} ({r['human']}, robot {r['robot_speed_mps']} m/s); "
                     + ", ".join(f"{s} a0 {r['a0'][s]} sigma {r['sigma_cm'][s]} cm" for s in SIDES))
    return lines
