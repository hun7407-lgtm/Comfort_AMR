#!/usr/bin/env python3
"""Check the Stage 1 size sweep before (and after) it runs. No simulator.

    python3 scripts/check_stage1.py                       configs only
    python3 scripts/check_stage1.py --dir results/batches/261112_stage1_size_sweep
                                                          ... and the results in it

Five groups, each of which exists because the sweep is worth nothing
without it:

  1. config validation        every condition file loads on every seed's
                              campaign config, through comfort_core.config,
                              with only `scenario` and `comfort_space` in it
  2. policy scaling           B3-O at size s has every extent equal to
                              (s / 1.20) x the unscaled extent, in all three
                              states and all three directions, at several
                              walking speeds; static at size s is s
                              everywhere; the min_radius floor holds; the
                              recorded geometry columns say the same
  3. same-seed reproducibility for one seed, the loaded configs of all
                              conditions are identical outside comfort_space
                              and scenario (people, walks, missions, start
                              times, map, Nav2, safety layer), and inside
                              comfort_space differ only in policy /
                              static_radius / aisle_scaling
  4. physical safety unaffected the collision monitor, footprint, speed
                              limits, inflation and the rest of what
                              scripts/generate_nav2_params.py renders come
                              from sections the conditions cannot touch,
                              and the generator itself takes no scenario
  5. result CSV validation    (--dir) header has the new columns; every
                              (condition, seed) appears once; the geometry
                              columns match the condition; the commit,
                              seed and mechanism are filled in; fleet rows
                              exist per trial

Exit 0 means every check passed.
"""

from __future__ import annotations

import argparse
import csv
import inspect
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from comfort_core import load_config                                # noqa: E402
from comfort_core.config import SCENARIO_ALLOWED_SECTIONS, load_with_base  # noqa: E402
from comfort_core.comfort_space import (GEOMETRY_FIELDS, geometry_record,  # noqa: E402
                                        make_policy)
from comfort_core.metrics import FIELDS, FLEET_FIELDS                 # noqa: E402
from comfort_core.size_sweep import (KIND_POLICY, condition_name,      # noqa: E402
                                     parse_condition)
from comfort_core import nav2_tuning                                   # noqa: E402

CAMPAIGN = "configs/campaigns/cross/campaign_h8_a3_cross_s{seed}.yaml"
REFERENCE_SIZE = 1.20
STATES = ("station_engaged", "stationary", "moving")
SPEEDS = (0.0, 0.6, 1.0, 1.4)

failures: list[str] = []
checks = 0


def ok(label: str, cond: bool) -> None:
    global checks
    checks += 1
    if not cond:
        failures.append(label)


def near(label: str, got: float, want: float, tol: float = 1e-9) -> None:
    ok(f"{label}: got {got:.6f}, want {want:.6f}", abs(got - want) <= tol)


def group(n: int, title: str) -> None:
    print(f"\n{n}. {title}")


def strip(cfg: dict) -> dict:
    """Everything a condition is not allowed to change."""
    return {k: v for k, v in cfg.items() if k not in SCENARIO_ALLOWED_SECTIONS}


def comfort_minus_factor(cs: dict) -> dict:
    return {k: v for k, v in cs.items()
            if k not in ("policy", "static_radius", "aisle_scaling")}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conditions", default="configs/stage1")
    ap.add_argument("--seeds", default="1-33")
    ap.add_argument("--dir", default=None, help="a results folder to validate too")
    args = ap.parse_args()

    a, b = (args.seeds.split("-") + [args.seeds])[:2]
    seeds = list(range(int(a), int(b) + 1))
    cond_dir = PROJECT_ROOT / args.conditions
    conds = sorted(p for p in cond_dir.glob("*.yaml") if parse_condition(p))
    ok(f"condition files in {args.conditions}", bool(conds))
    if not conds:
        print(f"[FAIL] no condition files in {cond_dir}")
        return 1

    # ---- 1. config validation -------------------------------------
    group(1, f"config validation: {len(conds)} conditions x {len(seeds)} seeds")
    for c in conds:
        overlay = load_with_base(c)
        ok(f"{c.name} has only {sorted(SCENARIO_ALLOWED_SECTIONS)}",
           set(overlay) <= SCENARIO_ALLOWED_SECTIONS)
        kind, size = parse_condition(c)
        ok(f"{c.name} scenario.name matches the file",
           overlay["scenario"]["name"] == condition_name(kind, size))
        ok(f"{c.name} policy is {KIND_POLICY[kind]}",
           overlay["comfort_space"]["policy"] == KIND_POLICY[kind])
        for seed in seeds:
            try:
                cfg = load_config(scenario=str(c), common_path=CAMPAIGN.format(seed=seed))
                make_policy(cfg)
                ok(f"{c.name} loads on seed {seed}", True)
            except Exception as exc:  # noqa: BLE001
                ok(f"{c.name} on seed {seed}: {exc}", False)
    print(f"   {len(conds)} conditions load on seeds {seeds[0]}-{seeds[-1]}")

    # ---- 2. policy scaling ------------------------------------------
    group(2, "policy scaling: ratios kept, floor kept, geometry columns truthful")
    base_cfg = load_config(scenario="b3d", common_path=CAMPAIGN.format(seed=seeds[0]))
    base = make_policy(base_cfg)
    min_r = float(base_cfg["comfort_space"]["min_radius"])
    near("unscaled moving front at the reference speed is 1.20",
         base.extents_for("_", "moving", 1.0).front, REFERENCE_SIZE)
    for c in conds:
        kind, size = parse_condition(c)
        cfg = load_config(scenario=str(c), common_path=CAMPAIGN.format(seed=seeds[0]))
        pol = make_policy(cfg)
        rec = geometry_record(cfg, pol)
        near(f"{c.name} comfort_size_m", float(rec["comfort_size_m"]), size, 1e-4)
        near(f"{c.name} comfort_scale", float(rec["comfort_scale"]), size / REFERENCE_SIZE, 1e-4)
        if kind == "static":
            for st in STATES:
                for v in SPEEDS:
                    e = pol.extents_for("_", st, v)
                    for name, got in zip(("front", "side", "rear"), e):
                        near(f"{c.name} {st} v={v} {name}", got, max(size, min_r))
            continue
        scale = size / REFERENCE_SIZE
        near(f"{c.name} scale_factor", pol.scale_factor, scale, 1e-9)
        for st in STATES:
            for v in SPEEDS:
                want = base.extents_for("_", st, v)
                got = pol.extents_for("_", st, v)
                for name, w, g in zip(("front", "side", "rear"), want, got):
                    near(f"{c.name} {st} v={v} {name} = scale x unscaled (or floor)",
                         g, max(w * scale, min_r), 1e-9)
                # relations survive scaling
                if st == "stationary":
                    ok(f"{c.name} stationary rear > side > front", got.rear > got.side > got.front)
                if st == "station_engaged":
                    ok(f"{c.name} engaged front >= side >= rear",
                       got.front >= got.side >= got.rear)
                if st == "moving" and v >= 0.6:
                    ok(f"{c.name} moving v={v} front > side", got.front > got.side)
        ok(f"{c.name} every extent >= min_radius {min_r}",
           all(x >= min_r - 1e-12 for st in STATES for v in SPEEDS
               for x in pol.extents_for("_", st, v)))
        # the recorded columns are the applied values
        e = pol.extents_for("_", "station_engaged")
        near(f"{c.name} cs_engaged_rear_m column", float(rec["cs_engaged_rear_m"]), e.rear, 1e-4)
        e = pol.extents_for("_", "moving", float(rec["cs_moving_reference_speed"]))
        near(f"{c.name} cs_moving_front_m column", float(rec["cs_moving_front_m"]), e.front, 1e-4)
    # a size that would push the smallest extent under the floor is floored, not negative
    tiny = dict(base_cfg)
    tiny["comfort_space"] = dict(base_cfg["comfort_space"])
    tiny["comfort_space"]["aisle_scaling"] = {"enabled": True, "target_size_m": 0.3,
                                              "floor_m": 0.0}
    t = make_policy(tiny).extents_for("_", "station_engaged")
    near("size 0.30: engaged rear floored at min_radius", t.rear, min_r)
    ok("size 0.30: engaged rear would have been 0.15 without the floor",
       0.6 * 0.3 / REFERENCE_SIZE < min_r)
    print(f"   {len(conds)} conditions: scale = size / {REFERENCE_SIZE:.2f} on every extent")

    # ---- 3. same-seed reproducibility ------------------------------
    group(3, "same-seed reproducibility: only comfort_space differs within a seed")
    for seed in (seeds[0], seeds[len(seeds) // 2], seeds[-1]):
        loaded = {c.name: load_config(scenario=str(c), common_path=CAMPAIGN.format(seed=seed))
                  for c in conds}
        ref_name = next(iter(loaded))
        ref = loaded[ref_name]
        for name, cfg in loaded.items():
            ok(f"seed {seed}: {name} == {ref_name} outside scenario/comfort_space",
               strip(cfg) == strip(ref))
            ok(f"seed {seed}: {name} comfort_space differs from {ref_name} only in "
               f"policy/static_radius/aisle_scaling",
               comfort_minus_factor(cfg["comfort_space"])
               == comfort_minus_factor(ref["comfort_space"]))
            for section in ("humans", "fleet", "navigation", "scenario_meta", "amr"):
                ok(f"seed {seed}: {name} section {section} identical",
                   cfg.get(section) == ref.get(section))
    # and the campaign config itself is the same file for all conditions
    # of a seed, which the manifest guarantees by construction.
    print("   people, walks, missions, start times, map and Nav2 identical per seed")

    # ---- 4. physical safety unaffected -----------------------------
    group(4, "physical safety unaffected by the Comfort Space size")
    src = (PROJECT_ROOT / "scripts/generate_nav2_params.py").read_text()
    ok("generate_nav2_params.py loads the config WITHOUT a scenario",
       "load_config(common_path=args.config)" in src and "scenario=" not in
       src.split("load_config(common_path=args.config)")[0][-200:])
    for seed in (seeds[0],):
        rendered = {}
        for c in conds:
            cfg = load_config(scenario=str(c), common_path=CAMPAIGN.format(seed=seed))
            rendered[c.name] = (
                cfg["navigation"].get("collision_monitor"),
                nav2_tuning.footprint_polygon(cfg),
                nav2_tuning.speed_limits(cfg),
                nav2_tuning.inflation_radius(cfg),
                nav2_tuning.cost_scaling_factor(cfg),
                nav2_tuning.dwb_horizon(cfg),
                nav2_tuning.critic_scales(cfg),
                nav2_tuning.observation_ranges(cfg),
                cfg["comfort_space"]["costmap"],   # mask value / buffer, not size
            )
        first = next(iter(rendered.values()))
        for name, r in rendered.items():
            ok(f"{name}: collision monitor, footprint, speeds, inflation, DWB, "
               f"critics, ranges, mask identical to the first condition", r == first)
        ok("collision_monitor block is present and non-empty", bool(first[0]))
    sig = inspect.signature(nav2_tuning.summary)
    ok("nav2_tuning has no size/scale/comfort parameter in its API",
       not any("comfort" in p or "scale" in p for p in sig.parameters))
    print("   safety layer and Nav2 tuning come from sections no condition can set")

    # ---- 5. result CSV validation ----------------------------------
    if args.dir:
        group(5, f"result CSV validation: {args.dir}")
        d = Path(args.dir)
        trials = d / "trials.csv"
        ok("trials.csv exists", trials.is_file())
        if trials.is_file():
            rows = list(csv.DictReader(trials.open(newline="", encoding="utf-8")))
            header = list(rows[0].keys()) if rows else []
            ok("header is comfort_core.metrics.FIELDS", tuple(header) == FIELDS)
            for col in ("deadlock_mechanism", *GEOMETRY_FIELDS):
                ok(f"column {col} present", col in header)
            seen: dict[tuple, int] = {}
            for r in rows:
                key = (r["scenario"], r["seed"])
                seen[key] = seen.get(key, 0) + 1
                parsed = parse_condition(r["scenario"])
                ok(f"row {key}: scenario is a size condition", parsed is not None)
                if parsed:
                    kind, size = parsed
                    ok(f"row {key}: policy {r['policy']} matches {kind}",
                       r["policy"] == KIND_POLICY[kind])
                    if r["comfort_size_m"]:
                        near(f"row {key}: comfort_size_m", float(r["comfort_size_m"]), size, 1e-3)
                    else:
                        ok(f"row {key}: comfort_size_m empty", False)
                ok(f"row {key}: seed recorded", bool(r["seed"]))
                ok(f"row {key}: commit recorded", bool(r["commit"]))
                ok(f"row {key}: result recorded", r["result"] in
                   ("SUCCESS", "DEADLOCK", "TIMEOUT", "INVALID", "PLANNING_FAILURE"))
                if r["result"] != "SUCCESS":
                    ok(f"row {key}: mechanism recorded for a {r['result']}",
                       bool(r["deadlock_mechanism"]))
                else:
                    ok(f"row {key}: no mechanism on a SUCCESS", not r["deadlock_mechanism"])
                if r["valid"] == "True":
                    ok(f"row {key}: travel_time > 0", float(r["travel_time"] or 0) > 0)
            dups = {k: n for k, n in seen.items() if n > 1}
            ok(f"no (condition, seed) run twice: {sorted(dups)[:5]}", not dups)
            print(f"   {len(rows)} rows, {len(seen)} distinct (condition, seed)")
            fleet = d / "fleet_trials.csv"
            if fleet.is_file():
                frows = list(csv.DictReader(fleet.open(newline="", encoding="utf-8")))
                ok("fleet header is FLEET_FIELDS", tuple(frows[0].keys()) == FLEET_FIELDS if frows else False)
                per = {}
                for fr in frows:
                    per.setdefault((fr["scenario"], fr["config"]), set()).add(fr["robot"])
                missing = [k for k in seen if (k[0], CAMPAIGN.format(seed=k[1]).replace(
                    "configs/campaigns/cross/", "configs/")) not in per and
                    not any(pk[0] == k[0] and pk[1].endswith(f"_s{k[1]}.yaml") for pk in per)]
                ok(f"every trial has fleet rows (missing {len(missing)}: {missing[:3]})",
                   not missing)
                ok("fleet scenario column is the condition name, not a path",
                   all(parse_condition(fr["scenario"]) and "/" not in fr["scenario"]
                       for fr in frows))
                print(f"   {len(frows)} fleet rows")
            else:
                failures.append("fleet_trials.csv missing")

    print()
    if failures:
        for f in failures[:40]:
            print(f"   FAIL  {f}")
        if len(failures) > 40:
            print(f"   ... and {len(failures) - 40} more")
        print(f"[check_stage1] {len(failures)} of {checks} checks FAILED")
        return 1
    print(f"[check_stage1] {checks} checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
