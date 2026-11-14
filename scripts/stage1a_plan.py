#!/usr/bin/env python3
"""Stage 1 controlled experiment: environments, conditions, PREDICTIONS and
manifests, always written before the runs they predict.

    python3 scripts/stage1a_plan.py --phase scan            # step0 + 1A-1 scan
    python3 scripts/stage1a_plan.py --phase predict \\
        --model configs/stage1a/model_exec.json             # 1A-2 + 1-B, pre-registered
    python3 scripts/stage1a_plan.py --phase scan --check    # print, write nothing

Plain Python 3 (numpy/scipy). No ROS, no omni.

One AMR, one person, a straight corridor, fixed start/goal, no fleet, no
walkers. Factors: corridor width W, the person's distance from the wall
y_h, the Human Context (comfort_core/contexts.py) and the effective
corridor-side extent e. Seeds are not a factor: a repeat is the same
trial again, for the run-to-run spread of Isaac Sim / ROS 2 / Nav2.

Flow (docs/STAGES.md, revised 2026-11-14 after the smoke runs showed the
path bounds miss the planner's buffer and the protective field):

  scan      step0  B1 (no Comfort Space), every W, 2 repeats
            1a1    Static, W 1.5 / 1.8 / 2.2 / 2.6, e from R_path^infl down to
                   the 0.30 m floor in 0.10 steps, plus R_path^lethal + 0.10;
                   1 run each. Model-free: it locates the observed boundary.
            predictions.json keeps the PATH bounds (R_path^infl/lethal),
            pre-registered, so how far they miss is itself a result.
  refine    scripts/stage1a_calibrate.py --refine: 3 runs at the last pass,
            the first failure and 0.05 m points between, per width.
  calibrate scripts/stage1a_calibrate.py: one parameter, the wall clearance
            of the execution model (comfort_core/feasibility.py), fitted on
            1A-1 only -> configs/stage1a/model_exec.json.
  predict   this script, --phase predict: R_exec_model for every 1A-2 and
            1-B cell from the frozen model, written to
            predictions_exec.json, THEN the runs.
            1a2   B3-O at W 2.2: C1 engaged-rear, C2 stationary-rear,
                  C3 stationary-front
            1b    hold-out: W 2.0 (Static, C1, C3); y_h 0.60 at W 2.2
                  (Static, C2); C4 stationary-side at W 2.2
            points 1a2: model-0.30 x1, -0.10 x1, model x3, +0.10 x3, +0.30 x1
                   1b : -0.10 x1, model x2, +0.10 x2, +0.20 x1

> Engineering bounds of the navigation stack. Not comfort distances.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core.config import load_config  # noqa: E402
from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.contexts import CONTEXTS  # noqa: E402
from comfort_core.size_sweep import condition_name, write_condition  # noqa: E402

OUT = PROJECT_ROOT / "configs" / "stage1a"
HUMAN = "a"
MIN_E = 0.30          # comfort_space.min_radius: smaller sizes are floored
# Person on the +y wall, passage toward -y: bearing person -> passage is -90.
PASSAGE_BEARING = -90.0

SCAN_WIDTHS = (1.5, 1.8, 2.2, 2.6)
STEP0_WIDTHS = (1.5, 1.8, 2.0, 2.2, 2.6)
STEP0_REPEATS = 2

PREDICT_CELLS = (
    [("1a2", 2.2, 0.35, c, "b3o") for c in ("c1", "c2", "c3")]
    + [("1b", 2.0, 0.35, "c3", "static"), ("1b", 2.0, 0.35, "c1", "b3o"),
       ("1b", 2.0, 0.35, "c3", "b3o"), ("1b", 2.2, 0.60, "c3", "static"),
       ("1b", 2.2, 0.60, "c2", "b3o"), ("1b", 2.2, 0.35, "c4", "b3o")]
)
POINTS = {
    "1a2": ((-0.30, 1), (-0.10, 1), (0.0, 3), (0.10, 3), (0.30, 1)),
    "1b": ((-0.10, 1), (0.0, 2), (0.10, 2), (0.20, 1)),
}


def env_name(w: float, yh: float, ctx: str) -> str:
    return f"w{w:.2f}_y{yh:.2f}_{ctx}"


def env_overlay(w: float, yh: float, ctx: str) -> dict:
    c = CONTEXTS[ctx]
    facing = ((PASSAGE_BEARING + c["facing_offset_deg"] + 180.0) % 360.0) - 180.0
    return {
        "base": "configs/common.yaml",
        "scenario_meta": {"scenario_id": f"stage1a_{env_name(w, yh, ctx)}",
                          "stage": "Stage 1 controlled", "aisle_width_m": w,
                          "human_wall_offset_m": yh, "context": c["name"],
                          "context_label": c["label"], "human_count": 1, "amr_count": 1},
        "corridor": {"width": w, "length": 12.0, "center": [0.0, 0.0]},
        # No station boxes: a box between the person and the wall would
        # narrow the aisle by more than the person does.
        "stations": {"a": {"enabled": False}, "b": {"enabled": False}},
        "humans": {
            "a": {"position": [0.0, round(w / 2.0 - yh, 4)], "state": c["state"],
                  "facing_deg": facing},
            "b": {"enabled": False},
            "c": {"enabled": False},
        },
        "navigation": {"start": [-5.0, 0.0], "goal": [5.0, 0.0]},
        "fleet": {"enabled": False},
    }


_HEADER = """\
# Stage 1 controlled environment {name}: corridor {w:.2f} m wide, one
# person {yh:.2f} m from the +y wall at x = 0, {label}.
# AMR 1 drives (-5, 0) -> (5, 0); no fleet, no walkers.
# GENERATED by scripts/stage1a_plan.py -- regenerate, do not hand-edit.
"""


def env_cfg(w: float, yh: float, ctx: str, write: bool):
    d = OUT / "envs"
    d.mkdir(parents=True, exist_ok=True)
    p = d / f"{env_name(w, yh, ctx)}.yaml"
    text = (_HEADER.format(name=env_name(w, yh, ctx), w=w, yh=yh, label=CONTEXTS[ctx]["label"])
            + yaml.safe_dump(env_overlay(w, yh, ctx), sort_keys=False))
    target = p if write else d / f".check_{p.name}"
    target.write_text(text, encoding="utf-8")
    cfg = load_config(common_path=str(target))
    if not write:
        target.unlink()
    return cfg


def git_commit() -> str:
    try:
        sha = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT,
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=PROJECT_ROOT,
                               capture_output=True, text=True).stdout.strip()
        return sha + ("+" if dirty else "")
    except Exception:
        return "unknown"


# Variants: the same experiment under a different Comfort-layer setting.
#   default  the study configuration
#   nobuf    CANDIDATE: comfort_space.costmap.buffer_width 0 (the planner bias
#            band off; gate boundary and safety layer unchanged). Written to
#            its own conditions/manifests/predictions/model files so it never
#            mixes with the default. Adopting it is the user's decision.
#   buf10    CANDIDATE: buffer_width 0.10 } fixed in advance (2026-11-14) as the
#   buf15    CANDIDATE: buffer_width 0.15 } only two candidates; selection rule
#            in configs/stage1a/buffer_selection.yaml, applied by
#            scripts/stage1a_select_buffer.py. No other value is added.
VARIANTS = {"default": {"suffix": "", "cond_dir": "conditions", "buffer": None},
            "nobuf": {"suffix": "_nobuf", "cond_dir": "conditions_nobuf", "buffer": 0.0},
            "buf10": {"suffix": "_buf10", "cond_dir": "conditions_buf10", "buffer": 0.10},
            "buf15": {"suffix": "_buf15", "cond_dir": "conditions_buf15", "buffer": 0.15}}
# Scan grid of the buffer candidates around R_path^infl (the boundary moves
# down by about the buffer width) and the low-e replication: the points
# where buffer 0 grazed the gate (261114_stage1a_nobuf/lowe), 3 runs each.
CANDIDATE_GRID = (-0.50, -0.40, -0.30, -0.20, -0.10, 0.0)
LOWE = ([(w, 0.35, "c3", "static", 0.30) for w in SCAN_WIDTHS]
        + [(2.0, 0.35, "c3", "static", 0.81), (2.2, 0.35, "c1", "b3o", 0.81)])
LOWE_REPEATS = 3
VARIANT = VARIANTS["default"]


def apply_variant(cfg):
    if VARIANT["buffer"] is not None:
        cfg["comfort_space"].setdefault("costmap", {})["buffer_width"] = VARIANT["buffer"]
    return cfg


def write_cond(kind: str, value: float):
    p = write_condition(OUT / VARIANT["cond_dir"], kind, value, stage="Stage 1 controlled")
    if VARIANT["buffer"] is not None:
        text = p.read_text(encoding="utf-8")
        p.write_text(f"# VARIANT {VARIANT['suffix'][1:]} (candidate): Comfort costmap buffer_width "
                     f"{VARIANT['buffer']}.\n" + text
                     + f"  costmap:\n    buffer_width: {VARIANT['buffer']}\n", encoding="utf-8")
    return p


def trial(cfg, kind: str, e: float, label: str, reps: int, bounds: dict):
    e = round(max(e, MIN_E), 2)
    value = round(fz.value_for_extent(cfg, kind, e, HUMAN), 2)
    e_actual = fz.extent_of(cfg, kind, value, HUMAN)
    return {"point": label, "e_target": e, "value": value, "e": round(e_actual, 4),
            "condition": condition_name(kind, value), "repeats": reps,
            **{f"feasible_{b}": e_actual <= v + 1e-9 for b, v in bounds.items()}}


def manifest_lines(env: str, trials):
    lines = []
    for r in range(max(t["repeats"] for t in trials)):
        for t in trials:
            if r < t["repeats"]:
                lines.append(f"configs/stage1a/{VARIANT['cond_dir']}/{t['condition']}.yaml "
                             f"configs/stage1a/envs/{env}.yaml")
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--phase", choices=("scan", "predict"), required=True)
    ap.add_argument("--model", help="fitted execution model (phase predict)")
    ap.add_argument("--check", action="store_true", help="print, write nothing")
    ap.add_argument("--res", type=float, default=0.01, help="grid resolution, m")
    ap.add_argument("--variant", choices=sorted(VARIANTS), default="default")
    args = ap.parse_args()
    global VARIANT
    VARIANT = VARIANTS[args.variant]
    sfx = VARIANT["suffix"]
    write = not args.check
    cells_out, manifests, conds = [], {}, set()

    if args.phase == "scan":
        manifests = {"step0": [], "1a1_scan": []} if not sfx else {"1a1_scan": []}
        for w in SCAN_WIDTHS:
            cfg = apply_variant(env_cfg(w, 0.35, "c3", write))
            pred = fz.predict(cfg, "static", HUMAN, res=args.res)
            eb = {b: pred[b]["e"] for b in fz.BOUNDS}
            trials, e = [], eb["infl"]
            grid = []
            if sfx:
                # The diagnostic already put this variant's boundary near
                # R_path^infl: a dense grid there instead of the full descent.
                offs = (-0.30, -0.20, -0.10, -0.05, 0.0) if sfx == "_nobuf" else CANDIDATE_GRID
                grid = [round(eb["infl"] + d, 2) for d in offs
                        if eb["infl"] + d >= MIN_E - 1e-9]
            while not sfx and e >= MIN_E - 1e-9:
                grid.append(round(e, 2))
                e -= 0.10
            if all(abs(g - MIN_E) > 0.05 for g in grid):
                grid.append(MIN_E)
            for i, v in enumerate(sorted(set(grid))):
                trials.append(trial(cfg, "static", v, f"scan{i}", 1, eb))
            trials.append(trial(cfg, "static", eb["lethal"] + 0.10, "above_lethal", 1, eb))
            env = env_name(w, 0.35, "c3")
            for t in trials:
                conds.add(("static", t["value"]))
            cells_out.append({"part": "1a1", "env": env, "aisle_width_m": w,
                              "human_wall_offset_m": 0.35, "context": "c3",
                              "context_label": CONTEXTS["c3"]["label"], "kind": "static",
                              "R_path_infl": {"value": pred["infl"]["value"], "e": round(eb["infl"], 4)},
                              "R_path_lethal": {"value": pred["lethal"]["value"], "e": round(eb["lethal"], 4)},
                              "closed_form_e": {k: round(v, 4) for k, v in pred["closed_form"].items()},
                              "trials": trials})
            # Descending order inside the scan: the far-feasible end first.
            manifests["1a1_scan"] += manifest_lines(env, sorted(trials, key=lambda t: t["e"]))
            print(f"  1a1 {env:20s} R_path^infl {eb['infl']:.3f}  R_path^lethal {eb['lethal']:.3f}  "
                  f"scan e {[t['e_target'] for t in trials]}")
        for r in range(STEP0_REPEATS if not sfx else 0):
            for w in STEP0_WIDTHS:
                env_cfg(w, 0.35, "c3", write)
                manifests["step0"].append(f"b1 configs/stage1a/envs/{env_name(w, 0.35, 'c3')}.yaml")
        if sfx and sfx != "_nobuf":
            manifests["lowe"] = []
            lowe = []
            for w, yh, ctx, kind, e in LOWE:
                t = trial(apply_variant(env_cfg(w, yh, ctx, write)), kind, e, "lowe", LOWE_REPEATS, {})
                conds.add((kind, t["value"]))
                lowe.append((env_name(w, yh, ctx), t))
            for r in range(LOWE_REPEATS):
                manifests["lowe"] += [manifest_lines(env, [t])[0] for env, t in lowe]
        pred_file = OUT / f"predictions{sfx}.json"
        model = None
    else:
        if not args.model:
            print("[stage1a_plan] --phase predict needs --model (scripts/stage1a_calibrate.py)")
            return 2
        model = json.loads(Path(args.model).read_text())
        manifests = {"1a2": [], "1b": []}
        for part, w, yh, ctx, kind in PREDICT_CELLS:
            cfg = apply_variant(env_cfg(w, yh, ctx, write))
            pred = fz.predict(cfg, kind, HUMAN, res=args.res, model=model)
            eb = {b: pred[b]["e"] for b in (*fz.BOUNDS, "exec")}
            if eb["exec"] is None:
                print(f"  {part} {env_name(w, yh, ctx)} {kind}: model says no passage at any "
                      f"size ({pred['exec']['note']}); cell skipped")
                continue
            trials = [trial(cfg, kind, eb["exec"] + d, f"exec{d:+.2f}", reps, eb)
                      for d, reps in POINTS[part]]
            env = env_name(w, yh, ctx)
            for t in trials:
                conds.add((kind, t["value"]))
            cells_out.append({"part": part, "env": env, "aisle_width_m": w,
                              "human_wall_offset_m": yh, "context": ctx,
                              "context_label": CONTEXTS[ctx]["label"], "kind": kind,
                              "R_path_infl": {"value": pred["infl"]["value"], "e": round(eb["infl"], 4)},
                              "R_path_lethal": {"value": pred["lethal"]["value"], "e": round(eb["lethal"], 4)},
                              "R_exec_model": {"value": pred["exec"]["value"], "e": round(eb["exec"], 4)},
                              # Pre-registered scoring: every e <= R_exec_model must pass
                              # (majority); every e > R_exec_model + 0.10 must not. The
                              # 0.10 m above the model is the calibration resolution.
                              "predicted_fail_above_e": round(eb["exec"] + 0.10, 4),
                              "closed_form_e": {k: round(v, 4) for k, v in pred["closed_form"].items()},
                              "trials": trials})
            manifests[part] += manifest_lines(env, trials)
            print(f"  {part:4s} {env:20s} {kind:6s} R_exec_model e={eb['exec']:.3f} "
                  f"(param {pred['exec']['value']:.3f})  R_path^infl {eb['infl']:.3f}")
        pred_file = OUT / f"predictions_exec{sfx}.json"

    counts = {k: len(v) for k, v in manifests.items()}
    print(f"[stage1a_plan] {args.phase}: trials {counts}  total {sum(counts.values())}")
    if not write:
        return 0
    if pred_file.exists() and args.phase == "predict":
        print(f"[stage1a_plan] {pred_file.name} exists; predictions are written once. "
              f"Move it aside deliberately to re-register.")
        return 1
    for kind, value in sorted(conds):
        write_cond(kind, value)
    (OUT / "manifests").mkdir(parents=True, exist_ok=True)
    for part, lines in manifests.items():
        (OUT / "manifests" / f"{part}{sfx}.txt").write_text(
            f"# Stage 1 controlled, part {part}. GENERATED by scripts/stage1a_plan.py\n"
            + "\n".join(lines) + "\n", encoding="utf-8")
    if pred_file.exists() and args.phase == "predict":
        print(f"[stage1a_plan] {pred_file.name} exists; predictions are written once. "
              f"Move it aside deliberately to re-register.")
        return 1
    pred_file.write_text(json.dumps({
        "written_by": "scripts/stage1a_plan.py", "phase": args.phase, "commit": git_commit(),
        "grid_resolution_m": args.res, "model": model, "variant": args.variant,
        "bounds": ("R_path^infl / R_path^lethal: robot centre kept (inscribed radius + gate "
                   "margin) outside the Comfort Space, (humans.radius + clearance) from the "
                   "body, clearance from walls = inflation radius / inscribed radius. "
                   "R_exec_model: Comfort Space grown by inscribed radius + costmap buffer, "
                   "walls and body at the fitted wall_clearance_m."),
        "cells": cells_out}, indent=2), encoding="utf-8")
    print(f"[stage1a_plan] wrote envs, conditions, manifests, {pred_file.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
