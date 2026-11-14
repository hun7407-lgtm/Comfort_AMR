#!/usr/bin/env python3
"""Stage 1A-1: observed boundary per width -> refine manifest -> execution model.

    python3 scripts/stage1a_calibrate.py --dir <scan batch> --refine OUT.txt
    python3 scripts/stage1a_calibrate.py --dir <scan batch> --dir <refine batch> --fit

Plain Python 3. No ROS, no omni.

Reads the Static calibration widths of 1A-1 (configs/stage1a/predictions.json,
part 1a1) and their trials. Per width:

    R_exec_lo   largest tested e that the majority of its runs passed, with
                every smaller tested e passed as well
    R_exec_hi   smallest tested e above it that the majority did NOT pass

--refine writes a manifest: R_exec_lo, R_exec_hi and the 0.05 m points
between them brought to 3 runs each.

--fit turns each width's boundary into the one parameter of the execution
model (comfort_core/feasibility.py, bound 'exec'):

    wall_clearance = (person centre -> far wall) - e - inscribed radius
                     - costmap buffer

evaluated at R_exec_lo (the conservative end). The model takes the LARGEST
over the widths (the most demanding), so R_exec_model never exceeds what
any calibration width actually passed. The spread across widths is
reported: a constant wall clearance is the model's claim, and the hold-outs
(1-B) test it. Writes configs/stage1a/model_exec.json once; it refuses to
overwrite an existing model (move it aside deliberately).

Hold-out cells (1-B) and the 1A-2 context cells are never read here.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.config import load_config  # noqa: E402
from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.comfort_space import robot_grow  # noqa: E402
from comfort_core.layout import corridor_bounds  # noqa: E402
from comfort_core.size_sweep import condition_name, parse_condition, write_condition  # noqa: E402

sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
from stage1a_plan import VARIANTS  # noqa: E402

PRED = PROJECT_ROOT / "configs" / "stage1a" / "predictions.json"
MODEL = PROJECT_ROOT / "configs" / "stage1a" / "model_exec.json"
REPEATS = 3


def rows_of(dirs):
    out = []
    for d in dirs:
        p = Path(d) / "trials.csv"
        if p.exists():
            out += list(csv.DictReader(p.open(newline="")))
    return out


def boundary(points):
    """points: e -> (passes, n). (lo, hi, low_failures).

    lo = the largest tested e passed by the majority of its runs; hi = the
    smallest tested e above lo. Failures BELOW lo are not the upper
    boundary -- with the costmap buffer off, a small Comfort Space lets
    the planner graze the gate boundary and the gate holds (seen at
    W 1.8, e 0.30) -- so they are returned separately, never dropped.
    """
    tested = sorted(e for e, (p, n) in points.items() if n)
    passed = [e for e in tested if points[e][0] * 2 > points[e][1]]
    if not passed:
        return None, (tested[0] if tested else None), []
    lo = max(passed)
    above = [e for e in tested if e > lo]
    low = [e for e in tested if e < lo and e not in passed]
    return lo, (min(above) if above else None), low


def collect(cells, rows):
    by = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    for r in rows:
        env = Path(r.get("config", "")).stem
        parsed = parse_condition(r.get("scenario", ""))
        if not parsed or parsed[0] != "static":
            continue
        if r.get("result") == "INVALID" or r.get("valid", "True") not in ("True", "true", "1"):
            continue
        e = round(parsed[1], 2)
        by[env][e][1] += 1
        by[env][e][0] += r.get("result") == "SUCCESS"
    return {c["env"]: {e: tuple(v) for e, v in by.get(c["env"], {}).items()} for c in cells}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", required=True)
    ap.add_argument("--refine", help="write the refine manifest here")
    ap.add_argument("--fit", action="store_true", help="write configs/stage1a/model_exec.json")
    ap.add_argument("--variant", choices=sorted(VARIANTS), default="default",
                    help="a Comfort-layer candidate (scripts/stage1a_plan.py VARIANTS)")
    args = ap.parse_args()
    global MODEL
    var = VARIANTS[args.variant]
    sfx = var["suffix"]
    cond_dir = "configs/stage1a/" + var["cond_dir"]
    MODEL = MODEL.with_name(f"model_exec{sfx}.json")
    pred = json.loads(PRED.with_name(f"predictions{sfx}.json").read_text())
    cells = [c for c in pred["cells"] if c["part"] == "1a1"]
    obs = collect(cells, rows_of(args.dir))
    refine, per_width = [], []
    for c in cells:
        pts = obs[c["env"]]
        lo, hi, low_fail = boundary(pts)
        cfg = load_config(common_path=f"configs/stage1a/envs/{c['env']}.yaml")
        b = corridor_bounds(cfg)
        far = float(cfg["humans"]["a"]["position"][1]) - b.y_min
        if var["buffer"] is not None:
            cfg["comfort_space"]["costmap"]["buffer_width"] = var["buffer"]
        extra = robot_grow(cfg) + fz.comfort_buffer(cfg)
        fit = None if lo is None else round(far - lo - extra, 4)
        bound_only = None
        if lo is None and pts:
            # Nothing passed: the clearance is at least what the smallest
            # tested e would have needed.
            bound_only = round(far - min(pts) - extra, 4)
        per_width.append({"env": c["env"], "aisle_width_m": c["aisle_width_m"],
                          "R_exec_lo": lo, "R_exec_hi": hi, "failures_below_lo": low_fail,
                          "points": {f"{e:.2f}": f"{p}/{n}" for e, (p, n) in sorted(pts.items())},
                          "wall_clearance_at_lo": fit, "wall_clearance_lower_bound": bound_only})
        print(f"  {c['env']:20s} R_exec bracket {lo} - {hi}"
              + (f" (failures below: {low_fail})" if low_fail else "") + "   wall clearance "
              f"{fit if fit is not None else ('>= ' + str(bound_only) if bound_only else '-')}   "
              f"points {per_width[-1]['points']}")
        if args.refine and lo is not None and hi is not None:
            targets = [lo]
            e = lo + 0.05
            while e < hi - 1e-6:
                targets.append(round(e, 2))
                e += 0.05
            targets.append(hi)
            for e in targets:
                have = pts.get(round(e, 2), (0, 0))[1]
                need = max(REPEATS - have, 0)
                if need:
                    p = write_condition(cond_dir, "static", e, stage="Stage 1 controlled")
                    if var["buffer"] is not None and "buffer_width" not in p.read_text():
                        p.write_text(f"# VARIANT {args.variant} (candidate): Comfort costmap "
                                     f"buffer_width {var['buffer']}.\n" + p.read_text()
                                     + f"  costmap:\n    buffer_width: {var['buffer']}\n")
                    refine += [f"{cond_dir}/{condition_name('static', e)}.yaml "
                               f"configs/stage1a/envs/{c['env']}.yaml"] * need
    if args.refine:
        # Interleave so repeats of one point are not back to back.
        order = sorted(range(len(refine)), key=lambda i: (refine[:i].count(refine[i]), i))
        Path(args.refine).write_text("# Stage 1A-1 refine. GENERATED by scripts/stage1a_calibrate.py\n"
                                     + "\n".join(refine[i] for i in order) + "\n", encoding="utf-8")
        print(f"[stage1a_calibrate] refine manifest: {len(refine)} trials -> {args.refine}")
    if args.fit:
        fits = [w["wall_clearance_at_lo"] for w in per_width if w["wall_clearance_at_lo"] is not None]
        if not fits:
            print("[stage1a_calibrate] no width has an observed boundary; nothing to fit")
            return 1
        if MODEL.exists():
            print(f"[stage1a_calibrate] {MODEL} exists; the model is fitted once. Move it aside "
                  "deliberately to refit.")
            return 1
        try:
            commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT,
                                    capture_output=True, text=True).stdout.strip()
        except Exception:
            commit = "unknown"
        model = {"variant": args.variant, "wall_clearance_m": max(fits),
                 "spread_m": round(max(fits) - min(fits), 4),
                 "fitted_on": [w["env"] for w in per_width if w["wall_clearance_at_lo"] is not None],
                 "per_width": per_width,
                 "comfort_extra_m": round(extra, 4),
                 "method": ("wall_clearance = (person centre to far wall) - R_exec_lo - inscribed "
                            "radius - costmap buffer, per calibration width; model takes the max"),
                 "batches": [Path(d).name for d in args.dir], "commit": commit}
        MODEL.write_text(json.dumps(model, indent=2), encoding="utf-8")
        print(f"[stage1a_calibrate] wall_clearance_m = {model['wall_clearance_m']:.3f} "
              f"(spread {model['spread_m']:.3f} over {len(fits)} widths) -> {MODEL}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
