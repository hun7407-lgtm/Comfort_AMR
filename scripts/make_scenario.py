#!/usr/bin/env python3
"""Write one paper scenario, and say everything about it before it runs.

    # look at a scenario without writing anything
    python3 scripts/make_scenario.py --seed 1 --humans 8 --amrs 1 --dry-run

    # write it, so Isaac Sim and the ROS 2 nodes can read it
    python3 scripts/make_scenario.py --seed 1 --humans 8 --amrs 1

    # the whole mission pool, to choose a minimum distance from
    python3 scripts/make_scenario.py --humans 8 --pool

    # the same seed under all four policies has to be one scenario
    python3 scripts/make_scenario.py --seed 1 --humans 8 --check

Plain Python 3. No ROS, no omni, and it starts nothing: this is the
step before the simulator, not a short run of it.

What it writes
--------------
    configs/scenarios/paper_<label>.yaml    what a trial reads
    results/scenarios/paper_<label>.json    the resolved values, for
                                            the record

The policy is NOT in either. One scenario is run under B1, B2, B3-R and
B3, which is the whole design, so it reaches a trial through
--scenario and never through the generated file. See
comfort_core/scenario.py.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core import load_config, resolve_path  # noqa: E402
from comfort_core.comfort_space import make_policy, robot_grow  # noqa: E402
from comfort_core.scenario import (BASE_CONFIG, DENSITY,  # noqa: E402
                                   ExperimentFactors, ScenarioGenerator,
                                   generate, mission_pool, role_positions,
                                   same_draw_as, validate_scenario,
                                   walk_timeline)

POLICIES = ("b1", "b2", "b3r", "b3")


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--humans", type=int, default=8, choices=sorted(DENSITY))
    p.add_argument("--amrs", type=int, default=1, choices=(1, 2, 3))
    p.add_argument("--provider", default="scripted",
                   choices=("scripted", "sensor"),
                   help="scripted = Layer 1 (policy evaluation), "
                        "sensor = Layer 2 (perception-integrated)")
    p.add_argument("--policy", default="b3", choices=POLICIES,
                   help="only used to print the run command; it is never "
                        "written into the scenario")
    p.add_argument("--base", default=BASE_CONFIG)
    p.add_argument("--out", default="configs/scenarios")
    p.add_argument("--dry-run", action="store_true",
                   help="report and validate, write nothing")
    p.add_argument("--pool", action="store_true",
                   help="print every candidate mission and stop")
    p.add_argument("--check", action="store_true",
                   help="prove the four policies get the same scenario, "
                        "and that the seed reproduces")
    p.add_argument("--quiet", action="store_true")
    return p.parse_args()


# ---------------------------------------------------------------
# reporting
# ---------------------------------------------------------------
def print_pool(cfg, humans: int) -> None:
    roles = role_positions(cfg)
    people = {r: roles[r] for r in DENSITY[humans]}
    gen = ScenarioGenerator(cfg)
    pool = mission_pool(cfg, people, 0.0, gen.standoff, gen.mouth)
    print(f"\n  every pair of empty cart slots, {humans} people in the bay")
    print(f"  (slot poses are {gen.mouth:.3f} m toward the slot mouth, and "
          f"no end is within {gen.standoff:.2f} m of anybody)\n")
    print(f"    {'start':16s} {'goal':16s} {'start xy':20s} {'goal xy':20s}"
          f" {'route m':>8s}  side")
    for m in pool:
        print(f"    {m.start_slot:16s} {m.goal_slot:16s} "
              f"({m.start[0]:+6.2f}, {m.start[1]:+6.2f})     "
              f"({m.goal[0]:+6.2f}, {m.goal[1]:+6.2f})     "
              f"{m.distance:8.2f}  {'cross' if m.cross_side else 'same'}")
    for limit in (8.0, 12.0, 16.0, 20.0):
        kept = [m for m in pool if m.distance >= limit]
        cross = sum(1 for m in kept if m.cross_side)
        print(f"  at a {limit:4.1f} m minimum: {len(kept):3d} pairs "
              f"({cross} crossing sides, {len(kept) - cross} same side)")
    print(f"\n  scenario_pool.min_mission_distance is "
          f"{gen.min_distance:.1f} m in configs/common.yaml. It is a "
          f"starting value, not a decision.")


def print_report(resolved, cfg, problems) -> None:
    f = resolved.factors
    gen = ScenarioGenerator(load_config(common_path=f_base(resolved)))
    print(f"\n=== {resolved.scenario_id} ===")
    print(f"  seed               : {f.seed}")
    print(f"  humans             : {f.human_count}")
    print(f"  AMRs               : {f.amr_count}")
    print(f"  state provider     : {f.provider}  "
          f"({'Layer 1, policy evaluation' if f.provider == 'scripted' else 'Layer 2, perception-integrated'})")
    print(f"  policies to run    : {', '.join(POLICIES)}  "
          f"(one scenario, four runs)")

    print("\n  people")
    print(f"    {'id':4s} {'position':18s} {'state':16s} role")
    for role, person in sorted(resolved.people.items()):
        job = "belt operator" if role[0] in "NS" else "storage side"
        print(f"    {role:4s} ({person.position[0]:+6.2f}, "
              f"{person.position[1]:+6.2f})   {person.state:16s} {job}")

    print("\n  exchanges")
    for ex in resolved.exchanges:
        print(f"    {ex.side:6s} {ex.belt_role} <-> {ex.edge_role}")
        print(f"      {ex.belt_role}: station_engaged -> moving -> "
              f"{ex.belt_ends_as}, leaves at t+{ex.belt_start_at:.1f} s, "
              f"{ex.belt_speed:.2f} m/s")
        print(f"      {ex.edge_role}: stationary -> moving -> "
              f"{ex.edge_ends_as}, leaves at t+{ex.edge_start_at:.1f} s, "
              f"{ex.edge_speed:.2f} m/s")
    print(f"\n    {'who':4s} {'departs':>9s} {'arrives':>9s} {'metres':>8s}")
    for role, departs, arrives, metres in walk_timeline(resolved):
        print(f"    {role:4s} {departs:8.1f} s {arrives:8.1f} s "
              f"{metres:7.2f}")
    print("    (t is navigation.trial_start_sim_time, "
          f"{cfg['navigation'].get('trial_start_sim_time', 0.0):.0f} "
          "simulated s after the scene plays)")

    print("\n  walk routes")
    for ex in resolved.exchanges:
        for role, route in ((ex.belt_role, ex.belt_route),
                            (ex.edge_role, ex.edge_route)):
            legs = " -> ".join(f"({x:+.2f},{y:+.2f})" for x, y in route)
            print(f"    {role:4s} {legs}")

    m = resolved.mission
    print("\n  AMR mission")
    print(f"    start  {m.start_slot:16s} ({m.start[0]:+6.2f}, "
          f"{m.start[1]:+6.2f})  facing "
          f"{cfg['amr']['spawn']['yaw_deg']:+.0f} deg")
    print(f"    goal   {m.goal_slot:16s} ({m.goal[0]:+6.2f}, "
          f"{m.goal[1]:+6.2f})  facing "
          f"{cfg['navigation']['goal_yaw_deg']:+.0f} deg")
    print(f"    route  {m.distance:.2f} m along the lanes, "
          f"{'crossing to the other side' if m.cross_side else 'one side only'}")
    speed = float(cfg["navigation"]["max_linear_speed"])
    print(f"    {m.distance / speed:.0f} s of driving at "
          f"{speed:.2f} m/s, against a "
          f"{cfg['navigation']['max_trial_time']:.0f} s trial limit")

    if resolved.fleet_plan:
        print("\n  other AMRs")
        for name, plan in sorted(resolved.fleet_plan.items()):
            print(f"    {name}  sets off t+{plan['start_at']:.1f} s from "
                  f"{plan['start']}, working "
                  f"{' -> '.join(plan['jobs'])}")

    print("\n  how tight the aisle is, per person")
    _print_clearances(resolved, cfg)

    for note in resolved.notes:
        print(f"\n  note: {note}")

    if problems:
        print(f"\n  NOT USABLE -- {len(problems)} problem(s):")
        for problem in problems:
            print(f"    - {problem}")
    else:
        print("\n  checks: layout, declared states, mission standoff, "
              "walk routes clear of\n          every machine, column and "
              "parked cart, waypoints inside the bay,\n          and the two "
              "halves of each exchange never within 0.70 m -- all pass")


def _print_clearances(resolved, cfg) -> None:
    """What each policy leaves of the aisle at each person's station.

    Geometry, printed before anything runs, so the preview says what the
    scenario ASKS of the robot rather than what it did.

    The number is the widest channel left for the robot's CENTRE across
    the aisle at that person's x: the navigable lane, pulled in by the
    inflation radius at each edge, minus every Comfort Space that
    reaches into it from either side. Zero means the planner has no
    route past that station at that policy.

    People on the storage side are included even though they stand
    outside the lane -- at 1.20 m plus the robot's inscribed radius a
    Comfort Space centred 0.64 m off the lane edge reaches well into
    it, and that is exactly the case B2 and B3 differ over.
    """
    from comfort_core.factory_layout import build_layout
    layout = build_layout(cfg)
    bands = {b.name: b for b in layout.bands}
    margin = float(cfg["factory"]["map"].get("lane_margin", 0.0))
    # Derived exactly as scripts/generate_nav2_params.py derives the
    # costmap's own inflation_radius, so this table cannot drift away
    # from the planner it is describing.
    inflation = (float(cfg["amr"]["footprint_width"]) / 2.0
                 + float(cfg["navigation"]["passage_margin"]))
    grow = robot_grow(cfg)
    policies = {name: make_policy(load_config(scenario=name,
                                              common_path=f_base(resolved)))
                for name in POLICIES}
    # A section is as long as the robot, so somebody 0.5 m along the
    # aisle still counts as being in the way.
    reach = float(cfg["amr"]["footprint_length"])

    print(f"    {'at':4s} {'aisle':6s} " +
          "".join(f"{p.upper():>9s}" for p in POLICIES))
    for role, person in sorted(resolved.people.items()):
        north = person.position[1] > 0
        aisle = bands["aisle_north" if north else "aisle_south"]
        lo = aisle.y_min - margin + inflation
        hi = aisle.y_max + margin - inflation
        widths = []
        for name in POLICIES:
            blocked = []
            for other in resolved.people.values():
                if abs(other.position[0] - person.position[0]) > reach:
                    continue
                r = policies[name].radius_for(other.role, other.state, 0.0)
                if r <= 0.0:
                    continue
                r += grow
                blocked.append((other.position[1] - r, other.position[1] + r))
            widths.append(_widest_gap(lo, hi, blocked))
        print(f"    {role:4s} {'north' if north else 'south':6s} " +
              "".join(f"{w:9.2f}" for w in widths))
    print(f"    m of lane left for the robot's centre at that station "
          f"(the lane itself is\n    {hi - lo:.2f} m wide for a centre). "
          f"0.00 means no channel past it at that\n    policy. A person "
          f"WALKING is wider again: `moving` is "
          f"{policies['b3'].radius_for('x', 'moving', 1.0):.2f} m and in B3 "
          f"stretched\n    along the heading.")


def _widest_gap(lo: float, hi: float, blocked) -> float:
    """Longest stretch of [lo, hi] no interval in `blocked` covers."""
    edges = sorted((max(lo, a), min(hi, b)) for a, b in blocked
                   if b > lo and a < hi)
    best = 0.0
    cursor = lo
    for a, b in edges:
        if a > cursor:
            best = max(best, a - cursor)
        cursor = max(cursor, b)
    return round(max(best, hi - cursor), 2)


_BASE = {}


def f_base(resolved) -> str:
    return _BASE.get("path", BASE_CONFIG)


# ---------------------------------------------------------------
def main() -> int:
    args = parse_args()
    _BASE["path"] = args.base
    base_cfg = load_config(common_path=args.base)

    if args.pool:
        print_pool(base_cfg, args.humans)
        return 0

    factors = ExperimentFactors(seed=args.seed, human_count=args.humans,
                                amr_count=args.amrs, policy=args.policy,
                                provider=args.provider)
    resolved = generate(factors, args.base)
    document = resolved.to_config(args.base)

    # Validate against the config a run would actually read, which means
    # writing it somewhere first. A dry run uses a scratch file.
    out_dir = resolve_path(args.out)
    name = f"{resolved.scenario_id}.yaml"
    header = (f"# {resolved.scenario_id}\n"
              f"# Written by scripts/make_scenario.py. Edit the seed and the\n"
              f"# factors, never this file: a hand edit makes it something "
              f"no\n# seed reproduces.\n"
              f"#\n"
              f"#   python3 scripts/make_scenario.py --seed {args.seed} "
              f"--humans {args.humans} --amrs {args.amrs} "
              f"--provider {args.provider}\n")
    text = header + yaml.safe_dump(document, sort_keys=False,
                                   default_flow_style=None)

    target = out_dir / name
    if args.dry_run:
        target = resolve_path("results/scenarios/.preview.yaml")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")

    cfg = load_config(common_path=str(target))
    problems = validate_scenario(resolved, cfg)

    if args.check:
        return _check(args, resolved, problems)

    if not args.quiet:
        print_report(resolved, cfg, problems)

    if problems:
        if args.dry_run:
            target.unlink(missing_ok=True)
        else:
            target.unlink(missing_ok=True)
            print(f"\n[ERROR] nothing written: this scenario is not usable",
                  file=sys.stderr)
        return 1

    if args.dry_run:
        target.unlink(missing_ok=True)
        print("\n  --dry-run: nothing written")
        return 0

    sidecar = resolve_path("results/scenarios") / f"{resolved.scenario_id}.json"
    sidecar.parent.mkdir(parents=True, exist_ok=True)
    sidecar.write_text(json.dumps(resolved.metadata(), indent=2),
                       encoding="utf-8")

    rel = target.relative_to(PROJECT_ROOT)
    print(f"\n  written: {rel}")
    print(f"           {sidecar.relative_to(PROJECT_ROOT)}")
    print(f"\n  look at it   : ./scripts/preview_scenario.sh --config {rel}")
    print(f"  one trial    : ./scripts/run_scenario.sh {args.policy} "
          f"--config {rel}")
    print(f"  all four     : ./scripts/run_batch.sh --scenarios "
          f"{','.join(POLICIES)} --configs {rel} --repeats 1 --dry-run")
    return 0


def _check(args, resolved, problems) -> int:
    """Non-experimental validation: determinism and policy independence."""
    ok = True
    if problems:
        ok = False
        print("  scenario is not usable:")
        for problem in problems:
            print(f"    - {problem}")

    again = generate(resolved.factors, args.base)
    if same_draw_as(resolved, again) or again.metadata() != resolved.metadata():
        ok = False
        print("  FAIL the same factors produced a different scenario twice")
    else:
        print("  pass  the same seed reproduces exactly")

    across = []
    for policy in POLICIES:
        for provider in ("scripted", "sensor"):
            other = generate(ExperimentFactors(
                seed=args.seed, human_count=args.humans, amr_count=args.amrs,
                policy=policy, provider=provider), args.base)
            across.extend(same_draw_as(resolved, other))
    if across:
        ok = False
        print("  FAIL the policy or the provider changed the scenario:")
        for problem in sorted(set(across)):
            print(f"    - {problem}")
    else:
        print(f"  pass  all {len(POLICIES)} policies x 2 providers get the "
              f"same people, walks and mission")

    first = generate(ExperimentFactors(seed=args.seed,
                                       human_count=args.humans, amr_count=1),
                     args.base)
    for count in (2, 3):
        other = generate(ExperimentFactors(seed=args.seed,
                                           human_count=args.humans,
                                           amr_count=count), args.base)
        if first.people != other.people or first.mission != other.mission:
            ok = False
            print(f"  FAIL amr_count {count} moved the people or the mission")
            break
    else:
        print("  pass  amr_count 1/2/3 get the same people and the same "
              "mission")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
