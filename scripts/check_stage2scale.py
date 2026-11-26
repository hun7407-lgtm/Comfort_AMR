#!/usr/bin/env python3
"""Unit / integration checks for the Stage 2 uniform-scale sweep.

    python3 scripts/check_stage2scale.py        (or: python3 -m pytest scripts/check_stage2scale.py)

Covers docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md section 9 that can be checked
without Isaac Sim. No simulation is started.
"""

from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core import comfort_reference as cref  # noqa: E402
from comfort_core import stage2scale as s2  # noqa: E402
from comfort_core.config import load_config  # noqa: E402

S = yaml.safe_load((PROJECT_ROOT / "configs/stage2scale/stage2scale.yaml").read_text())
BASE = load_config(common_path=str(PROJECT_ROOT / "configs/common.yaml"))
CS = BASE["comfort_space"]
RAW = s2.raw_templates(CS, 1.0)
NORM = s2.normalise(RAW, 0.90)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# 1. shapes ---------------------------------------------------------------
def test_templates_and_normalisation():
    want_raw = {"station_engaged": (0.80, 0.70, 0.60), "stationary": (0.80, 0.90, 1.05),
                "moving": (1.20, 0.90, 0.90)}
    want_norm = {"station_engaged": (0.900, 0.7875, 0.675), "stationary": (0.685714, 0.771429, 0.900),
                 "moving": (0.900, 0.675, 0.675)}
    for st in s2.STATES:
        assert all(close(RAW[st][a], v) for a, v in zip(s2.AXES, want_raw[st])), (st, RAW[st])
        assert all(close(NORM[st][a], v, 1e-6) for a, v in zip(s2.AXES, want_norm[st])), (st, NORM[st])
        assert s2.bounding_radius(NORM[st]) <= 0.900001, st


def test_pure_multiplication_and_no_clip():
    assert close(CS["min_radius"], 0.30)            # unchanged
    for al in (0.50, 0.45):
        ex = s2.scaled(NORM, al)
        for st in s2.STATES:
            for a in s2.AXES:
                assert close(ex[st][a], al * NORM[st][a], 1e-12)
                assert ex[st][a] >= CS["min_radius"] - 1e-12, (al, st, a)
    assert close(s2.min_alpha_without_clip(NORM, 0.30), 0.30 / 0.675)
    assert min(S["alphas"]) >= s2.min_alpha_without_clip(NORM, 0.30)


def test_condition_block_is_what_the_policy_uses():
    """The overlay, loaded through the real policy, yields exactly E(alpha)."""
    from comfort_core.comfort_space import make_policy
    for al in (1.0, 0.7, 0.45):
        ex = s2.scaled(NORM, al)
        cfg = {**BASE, "comfort_space": {**CS, **s2.comfort_block(ex, 0.1)}}
        pol = make_policy(cfg)
        for st in s2.STATES:
            for v in (0.0, 0.6, 1.0, 1.4):      # moving template frozen: speed does not matter
                e = pol.extents_for("a", st, v)
                for a in s2.AXES:
                    assert close(getattr(e, a), ex[st][a], 1e-6), (al, st, v, a, e)
        assert pol.heading_source_for("moving") == "velocity"
        assert pol.heading_source_for("stationary") == "body"


# 2. generator: identical physics, lanes ----------------------------------
def test_generator_check_all_phases_that_can_run():
    for ph in ("smoke", "pilot"):
        r = subprocess.run([sys.executable, "scripts/stage2scale_plan.py", "--phase", ph, "--check"],
                           cwd=PROJECT_ROOT, capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr
        assert "check OK" in r.stdout
    r = subprocess.run([sys.executable, "scripts/stage2scale_plan.py", "--phase", "coarse", "--check"],
                       cwd=PROJECT_ROOT, capture_output=True, text=True)
    assert r.returncode != 0 and "not frozen" in (r.stdout + r.stderr)     # no coarse run before the pilot freeze


def test_moving_lanes_distinct_and_opposite():
    m = S["geometry"]["moving"]
    for gap in m["lane_gap_candidates_m"]:
        ov = s2.moving_env(m, 3.0, gap)
        L = ov["_lanes"]
        assert close(abs(L["human_lane_y"] - L["amr_lane_y"]), gap, 1e-6)
        h, n = ov["humans"]["a"], ov["navigation"]
        assert h["position"][1] == h["route"][0][1] == L["human_lane_y"]
        assert n["start"][1] == n["goal"][1] == L["amr_lane_y"]
        assert (h["route"][0][0] - h["position"][0]) * (n["goal"][0] - n["start"][0]) < 0
        assert L["human_lane_y"] > L["amr_lane_y"]          # human_side +1: walker on +y (top of frame)


def test_static_geometry_independent_of_alpha():
    g = S["geometry"]["static"]
    sc = {"state": "stationary"}
    a = s2.static_env(sc, g, 2.4, -90.0)
    b = s2.static_env(sc, g, 2.4, -90.0)
    assert a == b                          # no alpha argument exists at all


def test_physical_signature_ignores_only_comfort_space():
    c1 = {**BASE, "comfort_space": {"policy": "none"}, "scenario": {"name": "x"}}
    c2 = {**BASE, "comfort_space": {"policy": "static"}, "scenario": {"name": "y"}}
    assert s2.physical_signature(c1) == s2.physical_signature(c2)
    c3 = {**c2, "navigation": {**BASE["navigation"], "goal": [9, 9]}}
    assert s2.physical_signature(c3) != s2.physical_signature(c2)


# 3. per-launch outcome ----------------------------------------------------
OK = {"completed": True, "contact": False, "side_ok": True, "sector": "rear", "intended_sector": "rear",
      "state_ok": True}


def test_far_deadlock_is_joint_failure():
    o = s2.passage_outcome({**OK, "completed": False, "comfort": 7.0}, 5.0)
    assert o["J"] == 0 and "not_completed" in o["reasons"]


def test_close_pass_below_target_is_failure_and_good_pass_succeeds():
    assert s2.passage_outcome({**OK, "comfort": 4.2}, 5.0)["J"] == 0
    assert s2.passage_outcome({**OK, "comfort": 5.3}, 5.0)["J"] == 1


def test_missing_or_wrong_evidence_never_counts_in_favour():
    assert s2.passage_outcome({**OK, "comfort": None}, 5.0)["J"] == 0
    assert s2.passage_outcome({**OK, "sector": "side", "comfort": 6.0}, 5.0)["J"] == 0
    assert s2.passage_outcome({**OK, "side_ok": None, "comfort": 6.0}, 5.0)["J"] == 0
    assert s2.passage_outcome({**OK, "contact": True, "comfort": 6.0}, 5.0)["J"] == 0
    assert s2.passage_outcome({**OK, "state_ok": False, "comfort": 6.0}, 5.0)["J"] == 0


def _cell(nav, enc, cok, n=10):
    outs = []
    for i in range(n):
        nv, en, co = i < nav, i < enc, i < cok
        outs.append({"nav_ok": nv, "encounter_ok": en, "comfort_ok": co, "J": int(en and co)})
    return s2.rates(outs)


# 4./5./6. selection --------------------------------------------------------
def test_nonmonotone_selects_middle():
    by = {1.0: _cell(3, 3, 3), 0.7: _cell(10, 10, 10), 0.45: _cell(10, 10, 2)}
    d = {1.0: 1.30, 0.7: 1.10, 0.45: 0.90}
    r = s2.select_context(by, 0.8, 0.8, d, 0.10)
    assert r["alpha_star"] == 0.7 and r["status"] == s2.IDENTIFIED_BRACKET and r["bracket"] == [0.45, 0.7]


def test_flat_distance_is_unidentifiable():
    by = {a: _cell(10, 10, 10) for a in (1.0, 0.7, 0.45)}
    r = s2.select_context(by, 0.8, 0.8, {1.0: 1.02, 0.7: 1.0, 0.45: 0.97}, 0.10)
    assert r["status"] == s2.UNIDENTIFIABLE and r["alpha_star"] is None


def test_floor_limited_is_not_a_bound():
    by = {a: _cell(10, 10, 10) for a in (1.0, 0.7, 0.45)}
    r = s2.select_context(by, 0.8, 0.8, {1.0: 1.3, 0.7: 1.1, 0.45: 0.9}, 0.10)
    assert r["status"] == s2.FLOOR_LIMITED and r["alpha_star"] == 0.45


def test_nav_only_failure_below_is_operational_not_comfort_bound():
    by = {1.0: _cell(10, 10, 10), 0.7: _cell(10, 10, 10), 0.45: _cell(2, 2, 2)}
    r = s2.select_context(by, 0.8, 0.8, {1.0: 1.3, 0.7: 1.1, 0.45: 0.9}, 0.10)
    assert r["status"] == s2.OPERATIONAL_ONLY and r["alpha_star"] == 0.7


def test_no_common_stationary_alpha_when_rear_fails():
    ok, bad = _cell(10, 10, 10), _cell(10, 10, 3)
    ctx = {"ST-F": {0.7: ok, 0.45: ok}, "ST-S": {0.7: ok, 0.45: ok}, "ST-R": {0.7: bad, 0.45: bad}}
    r = s2.select_state(ctx, 0.8, 0.8)
    assert r["status"] == s2.NO_COMMON and r["alpha_star"] is None
    ctx["ST-R"][0.7] = ok
    assert s2.select_state(ctx, 0.8, 0.8)["alpha_star"] == 0.7


def test_state_inherits_unidentifiable_and_floor_is_operational():
    ok = _cell(10, 10, 10)
    ctx = {"MV-P": {1.0: ok, 0.45: ok}}
    r = s2.select_state(ctx, 0.8, 0.8, {"MV-P": s2.UNIDENTIFIABLE})
    assert r["status"] == s2.UNIDENTIFIABLE and r["alpha_star"] is None
    r = s2.select_state(ctx, 0.8, 0.8, {"MV-P": s2.FLOOR_LIMITED})
    assert r["status"] == s2.COMMON_OPERATIONAL and r["alpha_star"] == 0.45
    r = s2.select_state(ctx, 0.8, 0.8, {"MV-P": s2.IDENTIFIED_BRACKET})
    assert r["status"] == s2.COMMON_IDENTIFIED


def test_wilson():
    lo, hi = s2.wilson(24, 30)
    assert 0.62 < lo < 0.64 and 0.90 < hi < 0.92
    assert s2.wilson(0, 0) == (None, None)


# 7. score depends only on the trajectory ----------------------------------
def test_score_independent_of_policy_label():
    def ticks(F):
        out = []
        for i in range(60):
            x = -3.0 + 0.1 * i
            out.append({"t": 0.2 * i, "amr": {"AMR1": [x, -0.9, 0.0]},
                        "people": {"a": {"s": "stationary", "o": "stationary", "x": 0.0, "y": 0.0, "v": 0.0,
                                         "h": math.pi / 2, "F": F, "S": F, "R": F}}})
        return out
    a = cref.score_passage(ticks(0.9), "AMR1", "a", 0.24)
    b = cref.score_passage(ticks(0.3), "AMR1", "a", 0.24)
    assert a["predicted_comfort_at_closest"] == b["predicted_comfort_at_closest"]
    assert a["min_cc_m"] == b["min_cc_m"] == 0.9


def run_all() -> int:
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as e:      # noqa: BLE001
                fails += 1
                print(f"FAIL {name}: {e!r}")
    print(f"{'ALL PASS' if not fails else f'{fails} FAILED'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(run_all())
