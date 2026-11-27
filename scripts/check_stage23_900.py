#!/usr/bin/env python3
"""Offline checks for the stage23_900 campaign code. No ROS / Isaac.

    python3 scripts/check_stage23_900.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import yaml  # noqa: E402

from comfort_core import final_metrics as fm  # noqa: E402

CFG = PROJECT_ROOT / "configs" / "final_nav2only_stage23_900" / "stage23_900.yaml"


def plan(phase):
    return subprocess.run([sys.executable, "scripts/final_stage23_900_plan.py", "--phase", phase, "--check"],
                          cwd=PROJECT_ROOT, capture_output=True, text=True)


def test_counts_and_refusals():
    S = yaml.safe_load(CFG.read_text())
    assert [float(a) for a in S["alphas"]] == [0.5, 0.6, 0.7, 0.8, 0.9]
    assert len(S["stage2"]["contexts"]) == 5 and S["stage2"]["main_seeds"] == [1, 30]
    assert len(S["stage3"]["policies"]) == 5 and S["stage3"]["main_seeds"] == [1001, 1030]
    r = plan("s2pilot")
    assert r.returncode == 0 and "60 trials" in r.stdout, r.stdout + r.stderr
    r = plan("s3pilot")
    assert r.returncode == 0 and "15 trials" in r.stdout, r.stdout + r.stderr
    r = plan("s3dup")
    assert r.returncode == 0 and "6 trials" in r.stdout, r.stdout + r.stderr
    r = plan("s3main")
    assert r.returncode == 0 and "150 trials" in r.stdout, r.stdout + r.stderr
    r = plan("s2main")
    if S.get("frozen"):
        assert r.returncode == 0 and "750 trials" in r.stdout, r.stdout + r.stderr
    else:
        assert r.returncode != 0 and "no frozen geometry" in r.stdout + r.stderr


def test_no_b1_b2_in_stage2_and_exact_grid():
    from final_stage23_900_plan import cname
    assert [cname(a) for a in (0.5, 0.6, 0.7, 0.8, 0.9)] == ["S3_a0.50", "S3_a0.60", "S3_a0.70", "S3_a0.80", "S3_a0.90"]


def test_jitter_is_deterministic_and_policy_free():
    from final_stage23_900_plan import static_env, moving_env
    S = yaml.safe_load(CFG.read_text())
    c = S["stage2"]["contexts"]["EN-R"]
    g = S["stage2"]["geometry"]
    a = static_env("EN-R", c, g["static"], 2.2, 7, S["stage2"]["jitter"]["static"])
    b = static_env("EN-R", c, g["static"], 2.2, 7, S["stage2"]["jitter"]["static"])
    assert a == b
    assert a[0]["stations"]["a"]["position"][1] == round(1.1 - 0.225, 4)      # back on the +y wall
    assert abs(a[0]["humans"]["a"]["position"][1] - (1.1 - 0.45 - 0.30)) < 1e-9
    m1 = moving_env("MV-P", g["moving"], 3.0, 0.40, 3, S["stage2"]["jitter"]["moving"])
    m2 = moving_env("MV-P", g["moving"], 3.0, 0.40, 4, S["stage2"]["jitter"]["moving"])
    assert m1 != m2 and m1[1]["human_lane_y"] > m1[1]["amr_lane_y"]


def test_nav2_only_success_labels():
    assert fm.success_class(0.5, True) == "SUCCESS" and fm.success_class(5, True) == "SUCCESS_IDLE"
    assert fm.success_class(20, True) == "SUCCESS_LONG_IDLE"
    assert fm.success_class(5, False) == "NORMAL_YIELD" and fm.success_class(20, False) == "TEMPORARY_HOLD"
    assert {"SUCCESS_IDLE", "SUCCESS_LONG_IDLE", "NORMAL_YIELD", "TEMPORARY_HOLD"} <= set(fm.SUCCESS_CLASSES)


def test_paired_stats():
    from final_stage23_900_report3 import holm, mcnemar_exact
    assert mcnemar_exact(0, 0) == 1.0
    assert abs(mcnemar_exact(0, 6) - 0.03125) < 1e-12
    assert holm([0.01, 0.04, 0.03]) == [0.03, 0.06, 0.06]


def test_eta_needs_samples():
    import final_stage23_900_eta as eta
    assert eta.pct([1, 2, 3, 4], 0.5) == 2.5 and eta.pct([], 0.5) is None


if __name__ == "__main__":
    n = 0
    bad = 0
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            try:
                f()
                print("PASS", k)
            except AssertionError as e:
                bad += 1
                print("FAIL", k, e)
            n += 1
    print("ALL PASS" if not bad else f"{bad} FAILED")
    sys.exit(1 if bad else 0)
