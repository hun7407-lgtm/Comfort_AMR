#!/usr/bin/env python3
"""Check comfort_core/nav2_tuning.py, the arithmetic both robots share.

Pure Python: no ROS, no Isaac, no Nav2. Run it after touching the tuning
block in configs/common.yaml or either parameter generator.

    ./scripts/check_nav2_tuning.py
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import load_config                      # noqa: E402
from comfort_core import nav2_tuning as nt                 # noqa: E402

CHECKS = 0
FAILURES = []


def check(label, got, want, tol=1e-9):
    global CHECKS
    CHECKS += 1
    ok = (got == want if isinstance(want, (bool, str, int))
          else abs(float(got) - float(want)) <= tol)
    if not ok:
        FAILURES.append(f"{label}: got {got!r}, want {want!r}")


def check_true(label, value):
    global CHECKS
    CHECKS += 1
    if not value:
        FAILURES.append(f"{label}: expected true")


def group(title):
    print(f"\n{title}")


def main() -> int:
    sim = load_config()
    real = load_config(common_path="configs/real_amr.yaml")

    group(" 1. the two robots are the shapes the configs say they are")
    check("sim half extents", nt.half_extents(sim)[1], 0.24)
    check("real half extents", nt.half_extents(real)[1], 0.285)
    check_true("sim footprint polygon",
               nt.footprint_polygon(sim).startswith("[[0.350, 0.240]"))
    check_true("real footprint polygon",
               nt.footprint_polygon(real).startswith("[[0.432, 0.285]"))

    group(" 2. clearance is half the width plus the passage margin")
    margin = float(sim["navigation"]["passage_margin"])
    check("sim clearance", nt.clearance_radius(sim), 0.24 + margin)
    check("real clearance", nt.clearance_radius(real), 0.285 + margin)

    group(" 3. inflation follows the clearance unless the corridor pins it")
    check("sim inflation derived", nt.inflation_radius(sim),
          nt.clearance_radius(sim))
    check("real inflation pinned", nt.inflation_radius(real), 0.35)
    floored = copy.deepcopy(real)
    floored["navigation"]["dwb"]["inflation_radius"] = 0.10
    check("pin below the inscribed radius is floored there",
          nt.inflation_radius(floored), 0.285)

    group(" 4. the horizon is a duration, so it is speed-dependent")
    check("sim horizon poses", nt.horizon_poses(sim), 1.7 * 0.8 / 0.05)
    check("real horizon poses", nt.horizon_poses(real), 1.7 * 0.2 / 0.05)
    check_true("the simulated robot covers more ground per horizon",
               nt.horizon_poses(sim) > nt.horizon_poses(real))
    stopped = copy.deepcopy(sim)
    stopped["navigation"]["max_linear_speed"] = 0.0
    check("a standing robot still has one pose", nt.horizon_poses(stopped), 1.0)

    group(" 5. path pressure is PathDist plus PathAlign")
    check("sim path pressure", nt.path_pressure_scale(sim), 64.0)

    group(" 6. the mask value comes from the Comfort Space costmap config")
    check("sim mask value", nt.mask_value(sim), 98)
    check_true("the mask is not lethal", nt.mask_value(sim) < 100)

    group(" 7. summing is on, because the final pose alone cannot grade")
    check("sim sum_scores", nt.base_obstacle_sum_scores(sim), True)
    check("real sum_scores", nt.base_obstacle_sum_scores(real), True)

    group(" 8. the scale solves traversal = avoid_margin x sidestep")
    for name, cfg in (("sim", sim), ("real", real)):
        wanted = float(cfg["navigation"]["dwb"]["base_obstacle"]["avoid_margin"])
        traverse = (nt.horizon_poses(cfg) * nt.mask_value(cfg)
                    * nt.base_obstacle_scale(cfg))
        sidestep = (nt.clearance_radius(cfg) * 0.5
                    * nt.path_pressure_scale(cfg))
        check(f"{name} traversal/sidestep ratio", traverse / sidestep, wanted,
              tol=1e-6)

    group(" 9. one formula, two different numbers -- the whole point")
    check_true("the real robot needs a larger scale",
               nt.base_obstacle_scale(real) > nt.base_obstacle_scale(sim))
    ratio = nt.base_obstacle_scale(real) / nt.base_obstacle_scale(sim)
    check_true(f"and by roughly the speed ratio (got {ratio:.2f}x)",
               3.0 < ratio < 6.0)

    group("10. a pinned scale skips the derivation")
    pinned = copy.deepcopy(sim)
    pinned["navigation"]["dwb"]["base_obstacle"]["scale"] = 0.2
    check("pinned scale honoured", nt.base_obstacle_scale(pinned), 0.2)

    group("11. sum_scores off falls back to scoring one pose")
    single = copy.deepcopy(sim)
    single["navigation"]["dwb"]["base_obstacle"]["sum_scores"] = False
    check("one-pose scale is horizon times larger",
          nt.base_obstacle_scale(single),
          nt.base_obstacle_scale(sim) * nt.horizon_poses(sim), tol=1e-6)

    group("12. observation ranges default to the simulated LiDAR")
    check("sim mark range", nt.observation_ranges(sim)[0], 6.0)
    check("sim raytrace range", nt.observation_ranges(sim)[1], 8.0)
    check("real mark range", nt.observation_ranges(real)[0], 10.0)
    check("real raytrace range", nt.observation_ranges(real)[1], 12.0)
    check_true("raytrace reaches past marking on both",
               all(nt.observation_ranges(c)[1] > nt.observation_ranges(c)[0]
                   for c in (sim, real)))

    group("13. speeds and critic weights come out whole")
    check("sim speed", nt.speed_limits(sim)[0], 0.8)
    check("real speed", nt.speed_limits(real)[0], 0.2)
    for name, cfg in (("sim", sim), ("real", real)):
        scales = nt.critic_scales(cfg)
        check(f"{name} critic count", len(scales), 6)
        check_true(f"{name} every weight is positive",
                   all(v > 0.0 for v in scales.values()))

    group("14. the summary prints without blowing up")
    for name, cfg in (("sim", sim), ("real", real)):
        text = nt.summary(cfg, name)
        check_true(f"{name} summary mentions the scale",
                   "BaseObstacle" in text)
        check_true(f"{name} summary is multi-line", text.count("\n") >= 8)

    print()
    for line in FAILURES:
        print(f"  FAIL {line}")
    if FAILURES:
        print(f"\n[check_nav2_tuning] {len(FAILURES)} of {CHECKS} checks FAILED")
        return 1
    print(f"[check_nav2_tuning] {CHECKS} checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
