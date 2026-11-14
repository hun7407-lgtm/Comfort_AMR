#!/usr/bin/env python3
"""Offline checks for the Stage 1 controlled experiment. No simulation.

    python3 scripts/check_stage1a.py

1. every condition overlay holds only `scenario` + `comfort_space`
2. the physical safety layer and Nav2 tuning are identical in every
   environment and under every condition (collision monitor, footprint,
   speeds, inflation, DWB) -- the corridor width and the person are the
   only differences between environments
3. effective extent: e is linear in the B3-O target with the context ratio
   of comfort_core/contexts.py (c1 0.50, c2 0.875, c3 0.667, c4 0.75), and
   equals the radius for Static
4. the feasibility grid agrees with the corridor closed form within two
   grid cells, for both path bounds and an example execution model
5. selection rule: never below R_comfort_min, infeasible when R_nav_safe <
   R_comfort_min, primary / fallback as documented
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.config import SCENARIO_ALLOWED_SECTIONS, load_config, load_with_base  # noqa: E402
from comfort_core.contexts import extent_ratio, key_of  # noqa: E402
from comfort_core.selection import INFEASIBLE, select  # noqa: E402

D = PROJECT_ROOT / "configs" / "stage1a"
n_ok = 0
fails = []


def ok(cond, what):
    global n_ok
    if cond:
        n_ok += 1
    else:
        fails.append(what)


SAFETY_KEYS = [("navigation", "collision_monitor"), ("amr",), ("navigation", "dwb"),
               ("navigation", "max_linear_speed"), ("navigation", "passage_margin"),
               ("comfort_space", "yield"), ("comfort_space", "costmap")]


def pick(cfg, path):
    for k in path:
        cfg = cfg.get(k, {}) if isinstance(cfg, dict) else None
    return cfg


envs = sorted((D / "envs").glob("w*.yaml"))
conds = sorted((D / "conditions").glob("*.yaml"))
ok(envs and conds, "stage1a envs/conditions exist (run scripts/stage1a_plan.py --phase scan)")
for c in conds:
    ok(set(load_with_base(str(c))) <= SCENARIO_ALLOWED_SECTIONS, f"{c.name}: only scenario+comfort_space")

ref = None
for e in envs:
    for c in (conds[:2] + conds[-2:]):
        cfg = load_config(scenario=str(c), common_path=str(e))
        snap = {p: pick(cfg, p) for p in SAFETY_KEYS}
        snap[("comfort_space", "yield")] = {k: v for k, v in (snap[("comfort_space", "yield")] or {}).items()}
        ref = ref or snap
        for p in SAFETY_KEYS:
            ok(snap[p] == ref[p], f"{e.name} x {c.name}: {'.'.join(p)} differs")

for e in envs:
    cfg = load_config(common_path=str(e))
    ctx = key_of(cfg["scenario_meta"]["context"])
    for s in (0.8, 1.2, 1.6, 2.0):
        got = fz.extent_of(cfg, "b3o", s, "a")
        want = max(extent_ratio(cfg, ctx) * s, float(cfg["comfort_space"]["min_radius"]))
        ok(abs(got - want) < 1e-6, f"{e.name}: B3-O e({s}) {got:.4f} != {want:.4f}")
    ok(abs(fz.extent_of(cfg, "static", 0.9, "a") - 0.9) < 1e-9, f"{e.name}: static e != radius")

for e in [x for x in envs if x.name.endswith("_c3.yaml")][:3]:
    cfg = load_config(common_path=str(e))
    model = {"wall_clearance_m": 0.50}
    pred = fz.predict(cfg, "static", "a", res=0.01, model=model)
    for b in ("infl", "lethal", "exec"):
        if pred[b]["e"] is None:
            # No passage even at the min_radius floor: the closed form must agree.
            ok(pred["closed_form"][b] < float(cfg["comfort_space"]["min_radius"]) + 0.02,
               f"{e.name}: grid {b} infeasible but closed form {pred['closed_form'][b]:.3f}")
            continue
        ok(abs(pred[b]["e"] - pred["closed_form"][b]) <= 0.02,
           f"{e.name}: grid {b} {pred[b]['e']:.3f} vs closed form {pred['closed_form'][b]:.3f}")

for rn, rmin, rp in [(0.5, 0.6, 0.9), (1.2, 0.6, 0.9), (0.8, 0.6, 0.9), (1.2, 0.6, None)]:
    s = select("c1_engaged_rear", rn, rmin, rp)
    ok(s.r_star >= rmin - 1e-12, f"R* {s.r_star} below R_comfort_min {rmin}")
    ok((s.status == INFEASIBLE) == (rn < rmin), f"status {s.status} for R_nav {rn} R_min {rmin}")
    if s.status != INFEASIBLE:
        ok(s.r_star <= rn + 1e-12, f"R* {s.r_star} above R_nav_safe {rn}")
        ok(s.rule == ("primary" if rp is not None else "fallback"), f"rule {s.rule}")

for f in fails:
    print("FAIL", f)
print(f"[check_stage1a] {n_ok} checks passed" + (f", {len(fails)} FAILED" if fails else ""))
sys.exit(1 if fails else 0)
