#!/usr/bin/env python3
"""Stage 1 final: apply the pre-registered buffer selection rule, then freeze.

    python3 scripts/stage1a_finalize.py                 evaluate every candidate,
                                                        write results/batches/261114_stage1a_final/
    python3 scripts/stage1a_finalize.py --freeze        ... and, if one candidate passed,
                                                        write configs/frozen/navigation_requirements.yaml

Plain Python 3. No ROS, no omni.

Reads configs/stage1a/buffer_selection.yaml (written before any candidate
ran) and, per candidate, its batch (scan, refine, 1a2, 1b, lowe), its
fitted model (model_exec_<v>.json) and its pre-registered predictions
(predictions_exec_<v>.json). Criteria A-D are applied exactly as written
there; nothing is tuned here. If both pass, the smaller buffer wins; if
none passes, the result is a blocker and nothing is frozen.

Outputs (the final folder holds no new trials; the selected candidate's
batch IS the final Stage 1 data):

    <final>/buffer_selection.json   per candidate: every criterion, the evidence
    <final>/SELECTION.md            the same, readable
    <final>/r_nav_safe.json         per Stage 1 cell: R_path^infl (= R_nav_safe),
                                    R_path^lethal, R_exec_model, observed R_exec

--freeze writes configs/frozen/navigation_requirements.yaml once (refuses
to overwrite): buffer, model parameter, footprint, gate margin, inflation,
the collision monitor block, the R_nav_safe definition, source batch and
commit. Stage 3 reads it; nothing after Stage 1 may change it.

> Engineering bounds of this navigation stack. Not comfort distances.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import yaml  # noqa: E402

from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.comfort_space import robot_grow  # noqa: E402
from comfort_core.config import load_config  # noqa: E402
from comfort_core.nav2_tuning import inflation_radius  # noqa: E402
from comfort_core.size_sweep import parse_condition  # noqa: E402
from stage1a_plan import LOWE, VARIANTS  # noqa: E402

CRITERIA = PROJECT_ROOT / "configs" / "stage1a" / "buffer_selection.yaml"
FROZEN_NAV = PROJECT_ROOT / "configs" / "frozen" / "navigation_requirements.yaml"
STAGE1A = PROJECT_ROOT / "configs" / "stage1a"
TOL = 1e-6

_EXT = {}


def extent(env: str, kind: str, value: float) -> float:
    k = (env, kind, round(value, 4))
    if k not in _EXT:
        cfg = load_config(common_path=f"configs/stage1a/envs/{env}.yaml")
        _EXT[k] = round(fz.extent_of(cfg, kind, value, "a"), 4)
    return _EXT[k]


def rows_in(d: Path):
    p = d / "trials.csv"
    if not p.exists():
        return []
    with p.open(newline="") as f:
        return list(csv.DictReader(f))


def outcome(r) -> str:
    if r.get("result") == "SUCCESS":
        return "pass"
    m = r.get("deadlock_mechanism") or r.get("result") or "?"
    if m == "stall" and "ProtectiveStop" in (r.get("deadlock_cause") or ""):
        return "protective_stop"
    return m


def valid(r) -> bool:
    return r.get("result") != "INVALID" and r.get("valid", "True") in ("True", "true", "1")


def points(rows):
    """(env, kind, e) -> list of outcomes."""
    out = defaultdict(list)
    for r in rows:
        if not valid(r):
            continue
        c = parse_condition(r.get("scenario", ""))
        if not c:
            continue
        kind = "static" if c[0] == "static" else "b3o"
        env = Path(r.get("config", "")).stem
        out[(env, kind, extent(env, kind, c[1]))].append(outcome(r))
    return out


def majority_pass(outs) -> bool:
    return sum(o == "pass" for o in outs) * 2 > len(outs)


def evaluate(variant: str, batch: Path) -> dict:
    sfx = VARIANTS[variant]["suffix"]
    model_p = STAGE1A / f"model_exec{sfx}.json"
    pred_p = STAGE1A / f"predictions_exec{sfx}.json"
    res = {"variant": variant, "buffer_width_m": VARIANTS[variant]["buffer"],
           "batch": str(batch.relative_to(PROJECT_ROOT)), "criteria": {}, "passed": False}
    if not model_p.exists() or not pred_p.exists():
        res["blocker"] = f"missing {model_p.name if not model_p.exists() else pred_p.name}"
        return res
    model = json.loads(model_p.read_text())
    pred = json.loads(pred_p.read_text())
    res["model"] = {"file": str(model_p.relative_to(PROJECT_ROOT)),
                    "wall_clearance_m": model["wall_clearance_m"], "spread_m": model["spread_m"]}
    res["predictions_file"] = str(pred_p.relative_to(PROJECT_ROOT))
    res["predictions_commit"] = pred.get("commit")
    cal = points(rows_in(batch / "scan") + rows_in(batch / "refine"))
    val = points(rows_in(batch / "1a2") + rows_in(batch / "1b"))
    low = points(rows_in(batch / "lowe"))
    res["trials"] = {d: len(rows_in(batch / d)) for d in ("scan", "refine", "1a2", "1b", "lowe")}

    # A: environment boundary, R_exec_lo strictly increasing in W
    widths = sorted(model["per_width"], key=lambda w: w["aisle_width_m"])
    los = [(w["aisle_width_m"], w["R_exec_lo"], w["R_exec_hi"]) for w in widths]
    a_ok = all(lo is not None for _w, lo, _h in los) and all(
        b[1] > a[1] + TOL for a, b in zip(los, los[1:]))
    res["criteria"]["A_environment_boundary"] = {
        "pass": a_ok, "brackets": [{"W": w, "pass_e": lo, "fail_e": hi} for w, lo, hi in los],
        "wall_clearance_spread_m": model["spread_m"]}

    # B, C, D over the validation cells
    cells = [c for c in pred["cells"] if c["part"] in ("1a2", "1b")]
    b_ok, c_bad, d_bad, cell_rows = len(cells) == 9, [], [], []
    for c in cells:
        m = c["R_exec_model"]["e"]
        hi = c["predicted_fail_above_e"]
        pts = {e: o for (env, kind, e), o in val.items() if env == c["env"] and kind == c["kind"]}
        below = {e: o for e, o in pts.items() if e <= m + TOL}
        above = {e: o for e, o in pts.items() if e > hi + TOL}
        upper = (bool(below) and all(majority_pass(o) for o in below.values())
                 and all(not majority_pass(o) for o in above.values()))
        b_ok = b_ok and upper
        # Informational: the boundary itself (model point passes, beyond
        # model + 0.10 fails), apart from any failure lower down (C).
        at = {e: o for e, o in below.items() if e >= m - 0.05}
        boundary_only = (bool(at) and all(majority_pass(o) for o in at.values())
                         and all(not majority_pass(o) for o in above.values()))
        for e, o in below.items():
            if "comfort_gate_hold" in o:
                c_bad.append(f"{c['part']} {c['env']} {c['kind']} e {e}: {o}")
            if "protective_stop" in o:
                d_bad.append(f"{c['part']} {c['env']} {c['kind']} e {e}: {o}")
        cell_rows.append({"part": c["part"], "env": c["env"], "kind": c["kind"],
                          "R_path_infl_e": c["R_path_infl"]["e"], "R_path_lethal_e": c["R_path_lethal"]["e"],
                          "R_exec_model_e": m, "upper_ok": upper,
                          "boundary_match": boundary_only,
                          "points": {f"{e:.2f}": f"{sum(x == 'pass' for x in o)}/{len(o)}"
                                     + ("" if majority_pass(o) else f" ({', '.join(sorted(set(o) - {'pass'}))})")
                                     for e, o in sorted(pts.items())}})
    res["criteria"]["B_upper_boundary"] = {
        "pass": b_ok, "cells": cell_rows, "n_cells": len(cells),
        "boundary_match": sum(c["boundary_match"] for c in cell_rows)}

    # 1A-1 at or below each width's observed pass point
    lo_of = {w["env"]: w["R_exec_lo"] for w in model["per_width"]}
    for (env, kind, e), o in cal.items():
        lo = lo_of.get(env)
        if lo is not None and e <= lo + TOL:
            if "comfort_gate_hold" in o:
                c_bad.append(f"1a1 {env} e {e}: {o}")
            if "protective_stop" in o:
                d_bad.append(f"1a1 {env} e {e}: {o}")
    # low-e replication
    lowe_rows = []
    lowe_expected = len(LOWE)
    for (env, kind, e), o in sorted(low.items()):
        ok = majority_pass(o)
        lowe_rows.append({"env": env, "kind": kind, "e": e,
                          "pass": f"{sum(x == 'pass' for x in o)}/{len(o)}", "outcomes": o})
        if "comfort_gate_hold" in o or not ok:
            c_bad.append(f"lowe {env} {kind} e {e}: {o}")
        if "protective_stop" in o:
            d_bad.append(f"lowe {env} {kind} e {e}: {o}")
    if len(lowe_rows) < lowe_expected:
        c_bad.append(f"lowe: {len(lowe_rows)} of {lowe_expected} points have data")
    res["criteria"]["C_low_e"] = {"pass": not c_bad, "violations": c_bad, "lowe": lowe_rows}
    res["criteria"]["D_protective_stop"] = {"pass": not d_bad, "violations": d_bad}
    res["passed"] = all(v["pass"] for v in res["criteria"].values())
    return res


def r_nav_safe_table(variant: str, batch: Path, model: dict) -> list:
    """Every Stage 1 cell of the selected variant: path bounds and the model."""
    sfx = VARIANTS[variant]["suffix"]
    out = []
    for name in (f"predictions{sfx}.json", f"predictions_exec{sfx}.json"):
        for c in json.loads((STAGE1A / name).read_text())["cells"]:
            cfg = load_config(common_path=f"configs/stage1a/envs/{c['env']}.yaml")
            cfg["comfort_space"].setdefault("costmap", {})["buffer_width"] = VARIANTS[variant]["buffer"]
            ex = c.get("R_exec_model") or fz.predict(cfg, c["kind"], "a", res=0.01, model=model)["exec"]
            out.append({"part": c["part"], "env": c["env"], "kind": c["kind"],
                        "aisle_width_m": c["aisle_width_m"],
                        "human_wall_offset_m": c["human_wall_offset_m"], "context": c["context"],
                        "R_nav_safe_e (= R_path^infl)": c["R_path_infl"]["e"],
                        "R_path_lethal_e": c["R_path_lethal"]["e"],
                        "R_exec_model_e": round(ex["e"], 4) if ex.get("e") is not None else None})
    return out


def git(*a) -> str:
    try:
        return subprocess.run(["git", *a], cwd=PROJECT_ROOT, capture_output=True,
                              text=True).stdout.strip()
    except Exception:
        return ""


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write_md(final: Path, results: list, selected) -> None:
    L = ["# Stage 1 final: buffer selection", "",
         "Rule and candidates fixed before any candidate ran: `configs/stage1a/buffer_selection.yaml`.",
         "Applied by `scripts/stage1a_finalize.py`. Nothing tuned after the runs.", "",
         "| candidate | buffer (m) | A environment | B upper boundary | C low-e | D protective stop | passed |",
         "|---|---|---|---|---|---|---|"]
    for r in results:
        cr = r["criteria"]
        f = lambda k: ("-" if k not in cr else ("pass" if cr[k]["pass"] else "FAIL"))  # noqa: E731
        L.append(f"| {r['variant']} | {r['buffer_width_m']} | {f('A_environment_boundary')} | "
                 f"{f('B_upper_boundary')} | {f('C_low_e')} | {f('D_protective_stop')} | "
                 f"{'yes' if r['passed'] else 'no'} |")
    L += ["", f"**Selected:** {selected['variant'] + ' (buffer ' + str(selected['buffer_width_m']) + ' m)' if selected else 'none -- blocker, nothing frozen'}", ""]
    for r in results:
        L += [f"## {r['variant']} (buffer {r['buffer_width_m']} m)", "", f"Batch `{r['batch']}`; trials {r.get('trials')}.", ""]
        if "blocker" in r:
            L += [f"Blocker: {r['blocker']}", ""]
            continue
        L += [f"Model: wall clearance {r['model']['wall_clearance_m']} m (spread {r['model']['spread_m']} m), "
              f"predictions `{r['predictions_file']}` (commit {r['predictions_commit']}).", "",
              "A. 1A-1 brackets (largest e passed by majority | next e tested):", ""]
        for b in r["criteria"]["A_environment_boundary"]["brackets"]:
            L.append(f"- W {b['W']:.1f}: {b['pass_e']} | {b['fail_e']}")
        L += ["", "B. Pre-registered cells:", "",
              f"Boundary itself (model point passes, > model + 0.10 fails): "
              f"{r['criteria']['B_upper_boundary']['boundary_match']}/{r['criteria']['B_upper_boundary']['n_cells']}. "
              "Criterion B also needs every lower tested e to pass.", "",
              "| part | env | kind | R_path^infl | R_exec_model | B ok | boundary match | points e: pass/n |",
              "|---|---|---|---|---|---|---|---|"]
        for c in r["criteria"]["B_upper_boundary"]["cells"]:
            L.append(f"| {c['part']} | {c['env']} | {c['kind']} | {c['R_path_infl_e']} | {c['R_exec_model_e']} | "
                     f"{'yes' if c['upper_ok'] else 'NO'} | {'yes' if c['boundary_match'] else 'NO'} | " + "; ".join(f"{e}: {v}" for e, v in c["points"].items()) + " |")
        L += ["", "C. Low-e:", ""]
        for x in r["criteria"]["C_low_e"]["lowe"]:
            L.append(f"- {x['env']} {x['kind']} e {x['e']}: {x['pass']}")
        L += [f"- violations: {r['criteria']['C_low_e']['violations'] or 'none'}", "",
              f"D. Protective stops at or below the boundary: {r['criteria']['D_protective_stop']['violations'] or 'none'}", ""]
    L += ["> Engineering bounds of this navigation stack. Not comfort distances.", ""]
    (final / "SELECTION.md").write_text("\n".join(L), encoding="utf-8")


def freeze(sel: dict, final: Path) -> int:
    if FROZEN_NAV.exists():
        print(f"[stage1a_finalize] {FROZEN_NAV.relative_to(PROJECT_ROOT)} exists; frozen once. "
              "Move it aside deliberately (and say why) to refreeze.")
        return 1
    v = sel["variant"]
    model_p = PROJECT_ROOT / sel["model"]["file"]
    cfg = load_config(common_path="configs/stage1a/envs/w2.20_y0.35_c3.yaml")
    cm = cfg["navigation"].get("collision_monitor", {})
    doc = {
        "frozen_at": datetime.now().isoformat(timespec="seconds"),
        "frozen_at_commit": git("rev-parse", "HEAD"),
        "stage1_final_model_version": f"stage1a_{v}_v1",
        "source": {"selection": str((final / "buffer_selection.json").relative_to(PROJECT_ROOT)),
                   "batch": sel["batch"], "criteria": str(CRITERIA.relative_to(PROJECT_ROOT)),
                   "predictions": sel["predictions_file"],
                   "predictions_commit": sel["predictions_commit"]},
        "comfort_buffer_m": sel["buffer_width_m"],
        "model": {"file": sel["model"]["file"], "sha256": sha256(model_p),
                  "wall_clearance_m": sel["model"]["wall_clearance_m"],
                  "spread_m": sel["model"]["spread_m"]},
        "robot": {"footprint_length_m": float(cfg["amr"]["footprint_length"]),
                  "footprint_width_m": float(cfg["amr"]["footprint_width"]),
                  "inscribed_radius_m (robot_grow)": robot_grow(cfg),
                  "inflation_radius_m": round(float(inflation_radius(cfg)), 4)},
        "gate": {"margin_m": float(cfg["comfort_space"]["yield"]["margin"]),
                 "lookahead_s": float(cfg["comfort_space"]["yield"]["lookahead_seconds"])},
        "physical_safety_layer": {"collision_monitor": cm,
                                  "note": "identical for every policy; never scaled with comfort"},
        "r_nav_safe": {
            "definition": "R_nav_safe := R_path^infl (Conservative Navigation Feasibility Bound)",
            "bound": "infl",
            "computed_by": "comfort_core.feasibility.r_path(cfg, kind, human, 'infl')",
            "execution_model_bound": "exec (comfort_core.feasibility, wall clearance above)",
            "table": str((final / "r_nav_safe.json").relative_to(PROJECT_ROOT))},
        "applies_to": ["b2_static_calibrated", "b3o_selection"],
        "common_yaml_sha256": sha256(PROJECT_ROOT / "configs" / "common.yaml"),
    }
    FROZEN_NAV.parent.mkdir(parents=True, exist_ok=True)
    FROZEN_NAV.write_text(
        "# Frozen navigation requirements from Stage 1 (scripts/stage1a_finalize.py --freeze).\n"
        "# Written once; Stage 3 reads it and may not change it. B2 and B3-O use the same\n"
        "# buffer and the same physical safety layer. Engineering bounds, not comfort distances.\n"
        + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"[stage1a_finalize] froze {FROZEN_NAV.relative_to(PROJECT_ROOT)} (buffer {sel['buffer_width_m']} m)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--final", default="results/batches/261114_stage1a_final")
    ap.add_argument("--batch-pattern", default="results/batches/261114_stage1a_{variant}")
    ap.add_argument("--freeze", action="store_true")
    args = ap.parse_args()
    crit = yaml.safe_load(CRITERIA.read_text())
    final = PROJECT_ROOT / args.final
    final.mkdir(parents=True, exist_ok=True)
    results = [evaluate(c["variant"], PROJECT_ROOT / args.batch_pattern.format(variant=c["variant"]))
               for c in crit["candidates"]]
    passing = [r for r in results if r["passed"]]
    selected = min(passing, key=lambda r: r["buffer_width_m"]) if passing else None
    (final / "buffer_selection.json").write_text(json.dumps(
        {"criteria_file": str(CRITERIA.relative_to(PROJECT_ROOT)), "rule": crit["rule"],
         "selected": None if not selected else {"variant": selected["variant"],
                                                "buffer_width_m": selected["buffer_width_m"]},
         "candidates": results, "evaluated_at_commit": git("rev-parse", "--short", "HEAD")},
        indent=2, default=str), encoding="utf-8")
    write_md(final, results, selected)
    for r in results:
        print(f"  {r['variant']}: " + ("blocker " + r["blocker"] if "blocker" in r else
              "  ".join(f"{k[0]} {'pass' if v['pass'] else 'FAIL'}" for k, v in r["criteria"].items()))
              + f"  -> {'PASS' if r['passed'] else 'fail'}")
    if not selected:
        print("[stage1a_finalize] no candidate passed: BLOCKER, nothing frozen")
        return 3
    model = json.loads((PROJECT_ROOT / selected["model"]["file"]).read_text())
    batch = PROJECT_ROOT / selected["batch"]
    (final / "r_nav_safe.json").write_text(json.dumps(
        {"variant": selected["variant"], "buffer_width_m": selected["buffer_width_m"],
         "definition": "R_nav_safe := R_path^infl; R_exec_model from the fitted execution model",
         "cells": r_nav_safe_table(selected["variant"], batch, model)}, indent=2), encoding="utf-8")
    print(f"[stage1a_finalize] selected {selected['variant']} (buffer {selected['buffer_width_m']} m) "
          f"-> {final.relative_to(PROJECT_ROOT)}")
    return freeze(selected, final) if args.freeze else 0


if __name__ == "__main__":
    sys.exit(main())
