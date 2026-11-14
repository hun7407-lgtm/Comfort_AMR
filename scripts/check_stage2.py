#!/usr/bin/env python3
"""Offline checks of the Stage 2 human-study tooling. No ROS, no robot, no people.

    python3 scripts/check_stage2.py

Every fixture here (distance levels, ratings, trial rows) is TEST DATA,
built in a temporary directory that is deleted at the end. Nothing is
written to configs/ or results/, and no value here is a study result.

Checks:
  config     distance_levels_m null -> refused (generator, recorder placeholders);
             contexts enable/disable from config; anonymous participant IDs
  orders     Williams balance (position and first-order carry-over), repeats,
             every (context, d) exactly `repeats` times per participant
  schema     trial row fields (status, times, distances, speed), questionnaire
             fields; item wording/scale only in the config
  logic      trial status, distance summary, usable ratings, candidates
  analysis   scripts/stage2_analysis.py end to end on a fixture
  frozen     configs/frozen/comfort_requirements.yaml still all null
"""

from __future__ import annotations

import copy
import csv
import math
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core import stage2 as s2  # noqa: E402
from comfort_core.config import load_with_base  # noqa: E402

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


def run(*args):
    return subprocess.run([sys.executable, *args], cwd=PROJECT_ROOT, capture_output=True, text=True)


def main() -> int:
    cfg = load_with_base("configs/stage2/stage2_real.yaml")

    # ---- config
    check(cfg["stage2"]["distance_levels_m"] is None, "distance_levels_m ships null")
    raises(lambda: s2.distance_levels(cfg), "distance_levels raises while null")
    check(any("distance_levels_m" in t for t in s2.missing_placeholders(cfg)),
          "recorder placeholder list names distance_levels_m")
    r = run("scripts/stage2_make_conditions.py", "--orders", "2")
    check(r.returncode == 2 and "distance_levels_m is null" in r.stderr,
          "stage2_make_conditions refuses while null")
    check(s2.study_contexts(cfg) == ["c1", "c2", "c3", "c4"], "all four contexts enabled by config")
    c = copy.deepcopy(cfg)
    c["stage2"]["contexts"]["c4_stationary_side"] = False
    check(s2.study_contexts(c) == ["c1", "c2", "c3"], "C4 disabled from config")
    c["stage2"]["contexts"] = ["c2_stationary_rear", "c4_stationary_side"]
    check(s2.study_contexts(c) == ["c2", "c4"], "list form still accepted")
    c["stage2"]["contexts"] = {k: False for k in cfg["stage2"]["contexts"]}
    raises(lambda: s2.study_contexts(c), "no context enabled -> refused")
    check(s2.participant_ids(cfg, ["P01", "P002"]) == ["P01", "P002"], "anonymous IDs accepted")
    for bad in ("Hong Gildong", "kim@example.com", "P1", "01", ""):
        raises(lambda b=bad: s2.participant_ids(cfg, [b]), f"non-anonymous ID refused: {bad!r}")
    raises(lambda: s2.participant_ids(cfg, ["P01", "P01"]), "duplicate ID refused")

    # ---- Williams / orders
    for n in range(2, 7):
        rows = s2.williams(n)
        check(len(rows) == (n if n % 2 == 0 else 2 * n), f"williams({n}) row count")
        for pos in range(n):
            cnt = Counter(r[pos] for r in rows)
            check(set(cnt) == set(range(n)) and len(set(cnt.values())) == 1,
                  f"williams({n}) every item equally often at position {pos}")
        pairs = Counter((r[i], r[i + 1]) for r in rows for i in range(n - 1))
        check(len(pairs) == n * (n - 1) and len(set(pairs.values())) == 1,
              f"williams({n}) first-order carry-over balanced")
    ctx = s2.study_contexts(cfg)
    levels = [0.5, 0.75, 1.0]           # TEST FIXTURE, not the study's levels
    for reps in (1, 2):
        seq = s2.participant_orders(["P01", "P02", "P03", "P04"], ctx, levels, reps)
        check(len(seq) == 4 * len(ctx) * len(levels) * reps, f"orders length, repeats {reps}")
        for pid in ("P01", "P02", "P03", "P04"):
            mine = [x for x in seq if x["participant_id"] == pid]
            check([x["order_index"] for x in mine] == list(range(1, len(mine) + 1)),
                  f"{pid} order_index 1..N (repeats {reps})")
            cnt = Counter((x["context"], x["passing_distance_m"]) for x in mine)
            check(set(cnt.values()) == {reps} and len(cnt) == len(ctx) * len(levels),
                  f"{pid} every (context, d) exactly {reps}x")
            blocks = [x["context"] for x in mine]
            changes = sum(1 for a, b in zip(blocks, blocks[1:]) if a != b)
            check(changes == len(ctx) - 1, f"{pid} contexts in contiguous blocks")
            check(len({x["trial_id"] for x in mine}) == len(mine), f"{pid} trial ids unique")
            check({x["repeat"] for x in mine} == set(range(1, reps + 1)), f"{pid} repeat numbers")
        firsts = Counter(next(x["context"] for x in seq if x["participant_id"] == p)
                         for p in ("P01", "P02", "P03", "P04"))
        check(len(firsts) == 4, "four participants start with four different contexts")

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        r = run("scripts/stage2_make_conditions.py", "--levels", "0.5", "0.75", "1.0",
                "--participants", "P01", "P02", "--out", str(tmp / "cond"))
        check(r.returncode == 0, "generator runs with fixture levels")
        rows = list(csv.DictReader((tmp / "cond" / "orders.csv").open()))
        check(list(rows[0]) == ["participant_id", "trial_id", "block", "order_index", "context",
                                "passing_distance_m", "repeat", "target_size_m", "condition"],
              "orders.csv columns")
        check(len(rows) == 2 * 4 * 3, "orders.csv rows")
        check(all((tmp / "cond" / f"{x['condition']}.yaml").exists() for x in rows),
              "every planned condition file written")
        r = run("scripts/stage2_make_conditions.py", "--levels", "0.5", "--participants",
                "Hong Gildong", "--out", str(tmp / "cond2"))
        check(r.returncode == 2, "generator refuses a name as participant id")

        # ---- schema
        for f in ("participant_id", "trial_id", "context", "planned_distance_m",
                  "actual_min_distance_m", "min_center_distance_m", "amr_speed_mean_mps",
                  "amr_speed_max_mps", "amr_speed_at_min_distance_mps", "trial_start_time",
                  "trial_end_time", "trial_status", "stop_reason", "operator_note", "repeat"):
            check(f in s2.TRIAL_FIELDS, f"trial row has {f}")
        for f in ("comfort_score", "perceived_safety", "acceptability", "optional_comment",
                  "questionnaire_version", "repeat", "context", "passing_distance_m"):
            check(f in s2.QUESTIONNAIRE_FIELDS, f"questionnaire row has {f}")
        check(set(s2.TRIAL_STATUSES) == {"NORMAL", "OPERATOR_ABORT", "PARTICIPANT_STOP", "E_STOP",
                                         "NAVIGATION_FAILURE", "INVALID_TRIAL"}, "six trial statuses")
        items = s2.questionnaire_items(cfg)
        check([i["key"] for i in items] == list(s2.RATING_KEYS), "config items = rating keys")
        for name in ("comfort_core/stage2.py", "scripts/stage2_analysis.py",
                     "scripts/stage2_questionnaire.py"):
            text = (PROJECT_ROOT / name).read_text(encoding="utf-8")
            check(not any(i["text"]["ko"] in text or i["text"]["en"] in text for i in items),
                  f"no item wording hard-coded in {name}")
        c = copy.deepcopy(cfg)
        c["stage2"]["questionnaire"]["items"][0]["scale_max"] = 5
        it5 = s2.questionnaire_items(c)
        check(not s2.validate_response({"participant_id": "P01", "trial_id": "t", "condition": "x",
                                        "comfort_size": 1, "trial_order": 1, "comfort_score": 5}, it5),
              "scale changed in config: 5 valid on 1..5")
        check(s2.validate_response({"participant_id": "P01", "trial_id": "t", "condition": "x",
                                    "comfort_size": 1, "trial_order": 1, "comfort_score": 6}, it5),
              "scale changed in config: 6 invalid on 1..5")
        c["stage2"]["questionnaire"]["items"][0]["primary"] = False
        raises(lambda: s2.questionnaire_items(c), "no primary item -> refused")
        row = s2.make_response(cfg, "P01", "S_P01_01", "b3o_r1.00", 1.0, 1,
                               {"comfort_score": 5, "perceived_safety": None, "acceptability": 4},
                               "", context="c2_stationary_rear", passing_distance_m=0.75)
        check(row["questionnaire_version"] == cfg["stage2"]["questionnaire"]["version"],
              "questionnaire version recorded")
        raises(lambda: s2.make_response(cfg, "P01", "t", "b3o_r1.00", 1.0, 1, {"comfort_score": 9}),
               "out-of-scale rating refused")
        raises(lambda: s2.make_response(cfg, "P01", "t", "b3o_r1.00", 1.0, 1, {}),
               "missing primary rating refused")
        raises(lambda: s2.make_response(cfg, "Kim", "t", "b3o_r1.00", 1.0, 1, {"comfort_score": 4}),
               "questionnaire refuses a name")

        # ---- logic
        check(s2.trial_status("SUCCESS") == "NORMAL", "SUCCESS -> NORMAL")
        check(s2.trial_status("ABORTED") == "NAVIGATION_FAILURE", "ABORTED -> NAVIGATION_FAILURE")
        check(s2.trial_status("TIMEOUT") == "NAVIGATION_FAILURE", "TIMEOUT -> NAVIGATION_FAILURE")
        check(s2.trial_status("NO_POSE") == "INVALID_TRIAL", "NO_POSE -> INVALID_TRIAL")
        check(s2.trial_status("INTERRUPTED") == "OPERATOR_ABORT", "Ctrl+C default -> OPERATOR_ABORT")
        check(s2.trial_status("SUCCESS", "PARTICIPANT_STOP") == "PARTICIPANT_STOP",
              "operator report wins over the nav result")
        check(s2.trial_status("E_STOP", "E_STOP") == "E_STOP", "e-stop status")
        raises(lambda: s2.trial_status("SUCCESS", "STOPPED"), "unknown status refused")
        samples = [{"t": 0.0, "clearance": 1.0, "center_distance": 1.6, "v": 0.0},
                   {"t": 0.1, "clearance": 0.40, "center_distance": 0.95, "v": 0.5},
                   {"t": 0.2, "clearance": 0.55, "center_distance": 1.05, "v": 0.7},
                   {"t": 0.3, "clearance": math.nan, "center_distance": math.nan, "v": 0.6}]
        d = s2.distance_summary(samples, 0.25)
        check(abs(d["actual_min_distance_m"] - 0.65) < 1e-9, "actual min distance = clearance + body radius")
        check(abs(d["min_center_distance_m"] - 0.95) < 1e-9, "raw centre distance")
        check(abs(d["amr_speed_at_min_distance_mps"] - 0.5) < 1e-9, "speed at the closest point")
        check(abs(d["amr_speed_mean_mps"] - 0.6) < 1e-9 and d["amr_speed_max_mps"] == 0.7, "AMR speeds")
        check(s2.distance_summary([], 0.25)["actual_min_distance_m"] is None, "no samples -> None")

        # ---- analysis on a fixture (TEST DATA, temporary)
        sess = tmp / "fixture_session"
        sess.mkdir()
        q_rows, t_rows = [], []
        n = 0
        for pid in ("P01", "P02", "P03"):
            for cx in ("c2_stationary_rear", "c3_stationary_front"):
                for dd, score in ((0.5, 2), (0.75, 4), (1.0, 6)):
                    n += 1
                    tid = f"fx_{pid}_{n:02d}"
                    status = "PARTICIPANT_STOP" if (pid == "P03" and dd == 0.5) else "NORMAL"
                    t_rows.append({"trial_id": tid, "participant_id": pid, "context": cx,
                                   "planned_distance_m": dd, "actual_min_distance_m": dd + 0.02,
                                   "trial_status": status})
                    q_rows.append(s2.make_response(cfg, pid, tid, "b3o_r1.00", 1.0, n,
                                                   {"comfort_score": score, "acceptability": 3 + (dd > 0.6) + (dd > 0.9)},
                                                   context=cx, passing_distance_m=dd))
        for rr in t_rows:
            s2.append_row(sess / "trials.csv", rr, s2.TRIAL_FIELDS)
        for rr in q_rows:
            s2.append_row(sess / "questionnaire.csv", rr, s2.QUESTIONNAIRE_FIELDS)
        s2.append_row(sess / "status_amendments.csv",
                      {"trial_id": t_rows[1]["trial_id"], "trial_status": "INVALID_TRIAL",
                       "stop_reason": "test", "operator_note": "", "timestamp": ""},
                      s2.STATUS_AMENDMENT_FIELDS)
        kept, excl = s2.usable_ratings(q_rows, s2.read_rows(sess / "trials.csv"),
                                       s2.read_rows(sess / "status_amendments.csv"))
        check(len(excl.get("PARTICIPANT_STOP", [])) == 2, "PARTICIPANT_STOP ratings kept out of the curve")
        check(len(excl.get("INVALID_TRIAL", [])) == 1, "amendment applied")
        check(len(kept) == len(q_rows) - 3, "only NORMAL ratings kept")
        curves = s2.by_context_distance(kept, "comfort_score", (1, 7))
        cand = s2.comfort_candidates(curves["c2_stationary_rear"], (1, 7), "median", None)
        check(cand["r_comfort_min_by_cutoff"]["4"]["distance_m"] == 0.75, "candidate at cutoff 4")
        check(cand["r_comfort_min_by_cutoff"]["7"]["distance_m"] is None, "no distance reaches 7")
        check(cand["r_pref_argmax"] == 1.0, "argmax candidate")
        acc = s2.by_context_distance(kept, "acceptability", (1, 7))
        mid = s2.comfort_candidates(acc["c3_stationary_front"], (1, 7), "median", 4)
        check(mid["r_pref_midpoint"]["at_midpoint_m"] == [0.75], "midpoint candidate")
        r = run("scripts/stage2_analysis.py", "--session-dir", str(sess))
        check(r.returncode == 0 and (sess / "ANALYSIS.md").exists(), "analysis runs on the fixture")
        md = (sess / "ANALYSIS.md").read_text() if (sess / "ANALYSIS.md").exists() else ""
        check("NOT decided here" in md and "PARTICIPANT_STOP" in md, "analysis states no final value, lists stops")
        for fig in ("S1_comfort_curves.png", "S2_participant_curves.png", "S3_planned_vs_actual.png"):
            check((sess / "figures" / fig).exists(), f"figure {fig}")
        r = run("scripts/stage2_analysis.py", "--session-dir", str(tmp / "empty"))
        check(r.returncode == 0 and "nothing to analyse" in r.stdout, "no data -> nothing produced")
        r = run("scripts/stage2_mark_trial.py", "--session-dir", str(sess), "--trial-id", "nope",
                "--status", "INVALID_TRIAL", "--reason", "x")
        check(r.returncode == 2, "mark_trial refuses an unknown trial")
        r = run("scripts/stage2_questionnaire.py", "--session-dir", str(sess), "--participant", "P01",
                "--trial-id", "fx_q", "--condition", "b3o_r1.00", "--size", "1.0", "--order", "1",
                "--rating", "comfort_score=5", "--comment", "")
        check(r.returncode == 0, "questionnaire CLI non-interactive")

    # ---- frozen comfort file stays null
    fr = yaml.safe_load((PROJECT_ROOT / "configs/frozen/comfort_requirements.yaml").read_text())
    vals = [v for c in fr["contexts"].values() for v in c.values()]
    check(all(v is None for v in vals), "comfort_requirements.yaml: every R_comfort_min / R_pref null")
    check(fr.get("stage2_source") is None, "comfort_requirements.yaml: no Stage 2 source yet")

    print(f"[check_stage2] {N - len(FAILED)}/{N} checks passed")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
