#!/usr/bin/env python3
"""Offline checks of the Stage 1 -> Stage 2 -> selection -> Stage 3 interface.
Nothing is simulated; Stage 3 is not run.

    python3 scripts/check_stage3_interface.py

Every comfort number here is a TEST FIXTURE (R_COMFORT_MIN / R_PREF below),
used only in memory or in a temporary directory. None is a study result and
none is written to configs/frozen/.

Checks:
  selection   Case A  R_nav_safe > R_pref > R_comfort_min    -> R* = R_pref
              Case B  R_pref > R_nav_safe >= R_comfort_min   -> R* = R_nav_safe
              Case C  R_nav_safe < R_comfort_min             -> PASSAGE_INFEASIBLE
              fallback (no R_pref) -> R* = R_comfort_min; B2 = max_H R_comfort_min
  pipeline    R_nav_safe from the Stage 1 bound on a Stage 1 environment ->
              fixture comfort values -> selection -> R*
  locks       the real frozen comfort file is null and refused; a missing
              navigation file is refused; stage3_run.sh --main refuses
  verdicts    one synthetic row per verdict (SUCCESS, PASSAGE_INFEASIBLE,
              COMFORT_GATE_HOLD, POST_EVADE_STALL, NAV2_FAILURE,
              ROBOT_ROBOT_DEADLOCK, PROTECTIVE_STOP, INVALID); the verdict_*
              columns exist in the trial schema; real Stage 1 rows classify
  pairing     seeds 1-100: campaign config = generator output (byte for byte),
              its scenario file exists, both policies load the same scene,
              missions, people and timing -- only comfort_space differs
"""

from __future__ import annotations

import copy
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import yaml  # noqa: E402

from comfort_core import feasibility as fz  # noqa: E402
from comfort_core import selection as sel  # noqa: E402
from comfort_core.config import load_config  # noqa: E402
from comfort_core.failure_class import VERDICTS, classify, verdict  # noqa: E402
from comfort_core.metrics import FIELDS  # noqa: E402

# TEST FIXTURES ONLY -- not Stage 2 results.
R_COMFORT_MIN = 0.60
R_PREF = 0.90

N = 0
FAILED = []


def check(cond, what):
    global N
    N += 1
    if not cond:
        FAILED.append(what)
        print(f"  FAIL {what}")


def raises(fn, what):
    try:
        fn()
    except ValueError:
        check(True, what)
        return
    check(False, what + " (did not raise)")


def main() -> int:
    # ---- selection rule
    a = sel.select("c2", r_nav_safe=1.20, r_comfort_min=R_COMFORT_MIN, r_pref=R_PREF)
    check(a.status == sel.FEASIBLE and a.r_star == R_PREF and a.rule == "primary", "Case A: R* = R_pref")
    b = sel.select("c2", r_nav_safe=0.75, r_comfort_min=R_COMFORT_MIN, r_pref=R_PREF)
    check(b.status == sel.FEASIBLE and b.r_star == 0.75 and b.rule == "primary", "Case B: R* = R_nav_safe")
    b2 = sel.select("c2", r_nav_safe=R_COMFORT_MIN, r_comfort_min=R_COMFORT_MIN, r_pref=R_PREF)
    check(b2.status == sel.FEASIBLE and b2.r_star == R_COMFORT_MIN, "Case B edge: R_nav_safe == R_comfort_min")
    c = sel.select("c2", r_nav_safe=0.50, r_comfort_min=R_COMFORT_MIN, r_pref=R_PREF)
    check(c.status == sel.INFEASIBLE and c.r_star == R_COMFORT_MIN and c.rule == "infeasible_hold",
          "Case C: PASSAGE_INFEASIBLE, space kept at R_comfort_min (never shrunk)")
    f = sel.select("c2", r_nav_safe=1.20, r_comfort_min=R_COMFORT_MIN, r_pref=None)
    check(f.rule == "fallback" and f.r_star == R_COMFORT_MIN, "fallback: R* = R_comfort_min")
    raises(lambda: sel.select("c2", 1.0, R_COMFORT_MIN, 0.5), "R_pref below R_comfort_min refused")
    raises(lambda: sel.select("c2", 1.0, None, None), "null R_comfort_min refused")
    for s in (a, b, c, f):
        check(s.r_star >= s.r_comfort_min, f"R* never below R_comfort_min ({s.rule})")
        if s.status == sel.FEASIBLE:
            check(s.r_star <= s.r_nav_safe, f"R* within R_nav_safe ({s.rule})")
    check(sel.static_calibrated({"c1": {"r_comfort_min_m": 0.5}, "c2": {"r_comfort_min_m": 0.7}}) == 0.7,
          "B2 = max over contexts of R_comfort_min")
    raises(lambda: sel.static_calibrated({"c1": {"r_comfort_min_m": None}}), "B2 refused while null")

    # ---- pipeline: Stage 1 bound -> fixture comfort -> R*
    env = load_config(common_path="configs/stage1a/envs/w2.20_y0.35_c2.yaml")
    r_nav = fz.r_path(env, "b3o", "a", "infl", res=0.02)["e"]
    check(r_nav is not None and 1.0 < r_nav < 1.4, f"R_nav_safe (R_path^infl) at W 2.2, C2: {r_nav}")
    s = sel.select("c2_stationary_rear", r_nav, R_COMFORT_MIN, R_PREF)
    check(s.r_star == R_PREF, "pipeline: wide corridor -> R* = R_pref")
    narrow = load_config(common_path="configs/stage1a/envs/w1.50_y0.35_c3.yaml")
    r_n = fz.r_path(narrow, "static", "a", "infl", res=0.02)["e"]
    s = sel.select("static", r_n, R_COMFORT_MIN, R_PREF)
    check(s.status == sel.INFEASIBLE, f"pipeline: W 1.5 (R_nav_safe {r_n:.2f}) < fixture R_comfort_min -> infeasible")
    nav_file = PROJECT_ROOT / "configs/frozen/navigation_requirements.yaml"
    if nav_file.exists():
        nav = sel.frozen_navigation()
        model = json.loads((PROJECT_ROOT / nav["model"]["file"]).read_text())
        check(abs(nav["comfort_buffer_m"] - (model["comfort_extra_m"] - nav["robot"]["inscribed_radius_m (robot_grow)"])) < 1e-6,
              "frozen buffer = the buffer the frozen model was fitted with")
        env2 = copy.deepcopy(env)
        env2["comfort_space"].setdefault("costmap", {})["buffer_width"] = nav["comfort_buffer_m"]
        r_b = fz.r_path(env2, "b3o", "a", nav["r_nav_safe"]["bound"], res=0.02, model=model)["e"]
        check(r_b is not None, f"frozen bound '{nav['r_nav_safe']['bound']}' computes R_nav_safe {r_b}")

    # ---- locks
    raises(lambda: sel.frozen_inputs(), "real comfort_requirements.yaml (null) refused")
    raises(lambda: sel.frozen_navigation(PROJECT_ROOT / "configs/frozen/does_not_exist.yaml"),
           "missing navigation file refused")
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "fixture.yaml"
        p.write_text(yaml.safe_dump({"contexts": {"c2_stationary_rear": {"r_comfort_min_m": R_COMFORT_MIN,
                                                                          "r_pref_m": R_PREF}}}))
        check(sel.frozen_inputs(p)["contexts"]["c2_stationary_rear"]["r_pref_m"] == R_PREF,
              "a filled (fixture) frozen file is accepted")
        p.write_text(yaml.safe_dump({"comfort_buffer_m": 0.1, "model": {"file": "x"},
                                     "r_nav_safe": {"bound": "infl"}}))
        raises(lambda: sel.frozen_navigation(p), "navigation file without wall clearance refused")
    r = subprocess.run(["./scripts/stage3_run.sh", "--main", "--dry-run"], cwd=PROJECT_ROOT,
                       capture_output=True, text=True, timeout=1800)
    check(r.returncode != 0 and "[ERROR]" in (r.stdout + r.stderr), "stage3_run.sh --main refuses (main run lock)")
    print("  main-run lock says: " + next((ln for ln in (r.stdout + r.stderr).splitlines()
                                            if "[ERROR]" in ln), "?").strip())

    # ---- verdicts
    for col in ("verdict_comfort_ahead", "verdict_person", "verdict_nearest_fleet_m"):
        check(col in FIELDS, f"trials.csv schema has {col}")
    base = {"result": "DEADLOCK", "valid": "True", "policy": "b3o_selection"}
    cases = {
        "SUCCESS": {"result": "SUCCESS"},
        "INVALID": {"result": "INVALID", "valid": "False"},
        "PASSAGE_INFEASIBLE": {"deadlock_mechanism": "comfort_gate_hold", "verdict_person": "a",
                               "selection_status_a": "PASSAGE_INFEASIBLE"},
        "COMFORT_GATE_HOLD": {"deadlock_mechanism": "comfort_gate_hold", "verdict_person": "a",
                              "selection_status_a": "FEASIBLE"},
        "ROBOT_ROBOT_DEADLOCK": {"deadlock_mechanism": "stall", "verdict_nearest_fleet_m": "1.1",
                                 "deadlock_cause": "no progress"},
        "PROTECTIVE_STOP": {"deadlock_mechanism": "stall", "verdict_nearest_fleet_m": "6.0",
                            "deadlock_cause": "no progress for 12 s; ProtectiveStop"},
        "POST_EVADE_STALL": {"deadlock_mechanism": "post_evade_stall", "verdict_nearest_fleet_m": "6.0"},
        "NAV2_FAILURE": {"deadlock_mechanism": "nav2_abort", "verdict_nearest_fleet_m": "6.0"},
    }
    for want, extra in cases.items():
        got = verdict({**base, **extra})
        check(got == want, f"verdict {want} (got {got})")
    check(set(cases) <= set(VERDICTS), "every required verdict is a named verdict")
    b2row = {**base, "policy": "static", "deadlock_mechanism": "comfort_gate_hold", "verdict_person": "a",
             "selection_r_nav_safe_a": "0.9", "selection_r_comfort_min_a": "0.6", "r_b2_m": "1.0"}
    check(classify(b2row) == "STATIC_OVER_RESERVATION" and verdict(b2row) == "COMFORT_GATE_HOLD",
          "B2 over-reservation is a comfort gate hold")
    seen = {}
    for d in ("results/batches/261114_stage1a/scan", "results/batches/261114_stage1a_nobuf/1b"):
        p = PROJECT_ROOT / d / "trials.csv"
        if p.exists():
            for row in csv.DictReader(p.open()):
                v = verdict(row)
                seen[v] = seen.get(v, 0) + 1
    check(seen.get("PROTECTIVE_STOP", 0) > 0 and seen.get("SUCCESS", 0) > 0,
          f"real Stage 1 rows classify: {seen}")
    print(f"  real Stage 1 rows by verdict: {seen}")

    # ---- pairing
    r = subprocess.run([sys.executable, "scripts/make_cross_campaign.py", "--verify"], cwd=PROJECT_ROOT,
                       capture_output=True, text=True)
    check(r.returncode == 0, "make_cross_campaign --verify (seeds 1-33 byte-identical)")
    import make_cross_campaign as mcc
    from comfort_core.config import SCENARIO_ALLOWED_SECTIONS
    mismatch = [s for s in range(1, 101)
                if (PROJECT_ROOT / f"configs/campaigns/cross/campaign_h8_a3_cross_s{s}.yaml").read_text()
                != mcc.render(s)]
    check(not mismatch, f"seeds 1-100 campaign configs = generator output (mismatch {mismatch[:5]})")
    starts = set()
    with tempfile.TemporaryDirectory() as tmp:
        b2 = Path(tmp) / "b2.yaml"
        b3 = Path(tmp) / "b3o.yaml"
        # Two overlays shaped like the Stage 3 pair (scenario + comfort_space only).
        b2.write_text(yaml.safe_dump({"scenario": {"name": "b2_fixture"},
                                      "comfort_space": {"policy": "static", "static_radius": 0.9,
                                                        "costmap": {"buffer_width": 0.10}}}))
        b3.write_text(yaml.safe_dump({"scenario": {"name": "b3o_fixture"},
                                      "comfort_space": {"policy": "state_directional",
                                                        "aisle_scaling": {"enabled": False},
                                                        "costmap": {"buffer_width": 0.10}}}))
        for s in range(1, 101):
            camp = f"configs/campaigns/cross/campaign_h8_a3_cross_s{s}.yaml"
            base_scn = yaml.safe_load((PROJECT_ROOT / camp).read_text())["base"]
            check((PROJECT_ROOT / base_scn).exists(), f"seed {s}: scenario {base_scn} exists")
            x = load_config(scenario=str(b2), common_path=camp)
            y = load_config(scenario=str(b3), common_path=camp)
            strip = lambda d: {k: v for k, v in d.items() if k not in ("comfort_space", "scenario")}  # noqa: E731
            check(strip(x) == strip(y), f"seed {s}: B2 and B3-O differ only in comfort_space")
            cs = lambda d: {k: v for k, v in d["comfort_space"].items()  # noqa: E731
                            if k not in ("policy", "static_radius", "aisle_scaling")}
            check(cs(x) == cs(y), f"seed {s}: same Comfort-layer settings apart from the policy (buffer etc.)")
            check(x["navigation"].get("collision_monitor") == y["navigation"].get("collision_monitor"),
                  f"seed {s}: same physical safety layer")
            starts.add(tuple(mcc.offsets(s)))
        check(set(SCENARIO_ALLOWED_SECTIONS) >= {"scenario", "comfort_space"}, "overlay sections allowed")
    check(len(starts) > 1, "fleet start offsets vary across seeds (paired, not identical)")
    again = load_config(common_path="configs/campaigns/cross/campaign_h8_a3_cross_s57.yaml")
    check(again == load_config(common_path="configs/campaigns/cross/campaign_h8_a3_cross_s57.yaml"),
          "same seed loads identically twice (deterministic)")

    print(f"[check_stage3_interface] {N - len(FAILED)}/{N} checks passed")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
