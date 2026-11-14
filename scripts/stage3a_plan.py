#!/usr/bin/env python3
"""Stage 3-A controlled validation: B2 Static Calibrated vs B3-O Dynamic
Selection in the Stage 1 corridors, with the outcome PREDICTED per cell
before anything runs.

    python3 scripts/stage3a_plan.py                    # needs model_exec.json and the frozen file
    python3 scripts/stage3a_plan.py --check            # print what is missing, write nothing

Plain Python 3. No ROS, no omni.

Cells: W 1.5 / 1.8 / 2.0 (hold-out) / 2.2 x contexts c1 / c2 / c3 (+ c4 if
the frozen file has it), 2 policies, 3 repeats each.

Per cell, from the frozen inputs only:
    R_nav_safe     Stage 1 execution model (configs/stage1a/model_exec.json)
    R_comfort_min  configs/frozen/comfort_requirements.yaml
    R_B2           max over contexts of R_comfort_min
    B3-O  R* = select(...)          -> predicted PASS if FEASIBLE, else
                                       PASSAGE_INFEASIBLE (waits; no shrink)
    B2    predicted PASS iff R_B2 <= R_nav_safe, else STATIC_OVER_RESERVATION
          when R_comfort_min <= R_nav_safe (a space meeting both existed)
          or PASSAGE_INFEASIBLE otherwise

Writes configs/stage3a/{b2_static_calibrated.yaml, b3o_selection_<env>.yaml},
manifest.txt and predictions.json (refuses to overwrite predictions).
Stage 3-B is not started unless every cell matches its prediction in at
least 2 of 3 repeats, with zero contacts and zero comfort-requirement
violations (docs/STAGES.md).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.config import load_config  # noqa: E402
from comfort_core.contexts import CONTEXTS  # noqa: E402
from comfort_core.selection import FROZEN, INFEASIBLE, select, static_calibrated  # noqa: E402

OUT = PROJECT_ROOT / "configs" / "stage3a"
MODEL = PROJECT_ROOT / "configs" / "stage1a" / "model_exec.json"
WIDTHS = (1.5, 1.8, 2.0, 2.2)
REPEATS = 3


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    missing = []
    if not MODEL.exists():
        missing.append(f"{MODEL.relative_to(PROJECT_ROOT)} (Stage 1 fit)")
    req = (yaml.safe_load(FROZEN.read_text()) or {}).get("contexts") or {}
    measured = {c: v for c, v in req.items() if (v or {}).get("r_comfort_min_m") is not None}
    if not measured:
        missing.append(f"{FROZEN.relative_to(PROJECT_ROOT)}: r_comfort_min_m (Stage 2)")
    if missing:
        print("[stage3a_plan] not ready; missing:\n   - " + "\n   - ".join(missing))
        return 2
    model = json.loads(MODEL.read_text())
    r_b2 = static_calibrated(measured)
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
    from stage1a_plan import env_cfg, env_name  # noqa: E402
    cells, lines = [], []
    OUT.mkdir(parents=True, exist_ok=True)
    b2 = OUT / "b2_static_calibrated.yaml"
    if not args.check:
        b2.write_text("# Stage 3-A B2: one radius = max over contexts of R_comfort_min (frozen).\n"
                      + yaml.safe_dump({"scenario": {"name": "b2_static_calibrated"},
                                        "comfort_space": {"policy": "static",
                                                          "static_radius": round(r_b2, 4)}}),
                      encoding="utf-8")
    for w in WIDTHS:
        for ck, c in CONTEXTS.items():
            if c["name"] not in measured:
                continue
            cfg = env_cfg(w, 0.35, ck, not args.check)
            env = env_name(w, 0.35, ck)
            r_nav = fz.predict(cfg, "b3o", "a", model=model)["exec"]["e"] or 0.0
            r_nav_static = fz.predict(cfg, "static", "a", model=model)["exec"]["e"] or 0.0
            r_min = float(measured[c["name"]]["r_comfort_min_m"])
            s = select(c["name"], r_nav, r_min, measured[c["name"]].get("r_pref_m"))
            b3o_pred = "SUCCESS" if s.status != INFEASIBLE else INFEASIBLE
            if r_b2 <= r_nav_static:
                b2_pred = "SUCCESS"
            else:
                b2_pred = "STATIC_OVER_RESERVATION" if r_min <= r_nav_static else INFEASIBLE
            spot = {"name": "a", "x": 0.0, "y": cfg["humans"]["a"]["position"][1],
                    "states": {c["state"]: {"context": c["name"], "r_nav_safe_m": round(r_nav, 3)}}}
            sel = OUT / f"b3o_selection_{env}.yaml"
            if not args.check:
                sel.write_text("# Stage 3-A B3-O Dynamic Selection for one corridor cell.\n"
                               + yaml.safe_dump({"scenario": {"name": "b3o_selection"},
                                                 "comfort_space": {
                                                     "policy": "b3o_selection",
                                                     "aisle_scaling": {"enabled": False},
                                                     "selection": {
                                                         "requirements_file": str(FROZEN.relative_to(PROJECT_ROOT)),
                                                         "spot_radius_m": 0.6, "spots": [spot]}}},
                                                sort_keys=False), encoding="utf-8")
            cells.append({"env": env, "aisle_width_m": w, "context": c["name"],
                          "r_nav_safe": round(r_nav, 3), "r_comfort_min": r_min,
                          "r_b2": r_b2, "selection": s.as_row(),
                          "predicted": {"b2_static_calibrated": b2_pred, "b3o_selection": b3o_pred}})
            for _ in range(REPEATS):
                lines.append(f"configs/stage3a/b2_static_calibrated.yaml configs/stage1a/envs/{env}.yaml")
                lines.append(f"configs/stage3a/{sel.name} configs/stage1a/envs/{env}.yaml")
            print(f"  {env:20s} R_nav_safe {r_nav:.2f}  R_comfort_min {r_min:.2f}  R* {s.r_star:.2f} "
                  f"({s.status})  B2 {r_b2:.2f} -> predicted B2 {b2_pred}, B3-O {b3o_pred}")
    print(f"[stage3a_plan] {len(cells)} cells, {len(lines)} trials")
    if args.check:
        return 0
    pred = OUT / "predictions.json"
    if pred.exists():
        print(f"[stage3a_plan] {pred} exists; predictions are written once")
        return 1
    (OUT / "manifest.txt").write_text("# Stage 3-A controlled validation\n" + "\n".join(lines) + "\n",
                                      encoding="utf-8")
    pred.write_text(json.dumps({"model": model, "frozen": str(FROZEN), "cells": cells}, indent=2),
                    encoding="utf-8")
    print(f"[stage3a_plan] wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
