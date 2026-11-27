#!/usr/bin/env python3
"""Stage 2 uniform-scale sweep: score launches, rates, selection, report.

    python3 scripts/stage2scale_report.py --dir results/batches/261126_stage2scale_v1_smoke

Plain Python 3 (numpy, matplotlib optional). Reads <dir>/manifest.tsv,
plan.json, trials/<id>/. Writes, under <dir>/analysis/:
    trials/<id>.json            final_metrics.analyse_trial + passage score
    scale_trials.csv            one row per launch
    scale_summary.json          per cell (scenario x geometry x condition) per target
    selected_state_scales.json  per context and per state, with identification status
    B3O_SCALE_CANDIDATE.yaml    only when a common alpha exists for a state; provenance per cell
    figures/*.png, STAGE2_SCALE_REPORT.md

The comfort number is comfort_reference.comfort() of the ACTUAL closest
centre distance and bearing; alpha, the mask and the configured extents
never enter it. A launch that does not complete the intended passage is a
joint failure (J = 0). Design: docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core import comfort_reference as cref  # noqa: E402
from comfort_core import final_metrics as fm  # noqa: E402
from comfort_core import stage2scale as s2  # noqa: E402

SUCCESS = set(fm.SUCCESS_CLASSES)


def read_manifest(d: Path):
    rows = []
    for line in (d / "manifest.tsv").read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            tid, rep, cond, cfg = line.split("\t")
            rows.append({"trial_id": tid, "repeat": int(rep), "condition": Path(cond).stem, "env": Path(cfg).stem})
    return rows


def score_launch(tdir: Path, plan: dict, geom: dict, scn: dict):
    """Everything about one launch that does not depend on the target."""
    a = fm.analyse_trial(tdir, spatial=False)
    D = fm.load(tdir)
    ticks = D["ticks"]
    cfg = fm.config(D["done"]["condition"], D["done"]["config"])
    heads = fm.headings(ticks, cfg) if ticks else []
    A = a["amrs"]["AMR1"]
    t0, t1 = A["t_begin"], A["t_verdict"]
    hl, hw = plan["amr_half_length_m"], plan["amr_half_width_m"]
    ref = {k: plan["reference"][k] for k in ("stationary", "station_engaged", "moving")}
    sc = cref.score_passage(ticks, "AMR1", "a", hw, ref, heading_of=lambda i, k: heads[i].get(k),
                            t_from=t0, t_to=t1, contact_m=plan["human_radius_m"] + hw) or {}
    # closest tick, footprint clearance, side, crossing, logged shape
    best, edge_min, rel_x, shapes, obs_at = None, None, [], [], None
    for i, tk in enumerate(ticks):
        t = tk.get("t")
        pose, p = (tk.get("amr") or {}).get("AMR1"), (tk.get("people") or {}).get("a")
        if t is None or not (t0 <= t <= t1) or not pose or not p:
            continue
        e = fm.rect_clearance(pose, hl, hw, p["x"], p["y"])
        edge_min = e if edge_min is None else min(edge_min, e)
        d = math.hypot(pose[0] - p["x"], pose[1] - p["y"])
        if best is None or d < best[0]:
            best = (d, pose, p)
        rel_x.append(pose[0] - p["x"])
        if p.get("F") is not None:
            shapes.append((p["F"], p["S"], p["R"]))
    side_ok, crossed = None, bool(rel_x) and min(rel_x) < 0 < max(rel_x)
    if best:
        _d, pose, p = best
        obs_at = p.get("o")
        if scn["state"] == "moving":
            want = math.copysign(1.0, geom["amr_lane_y"] - geom["human_lane_y"])
            side_ok = math.copysign(1.0, pose[1] - p["y"]) == want and crossed
        else:   # person on the +y wall, aisle at -y
            side_ok = pose[1] < p["y"] and crossed
    state_ok = None
    if sc.get("state") is not None:
        state_ok = sc["state"] == scn["state"] and (scn["state"] != "moving" or obs_at == "moving")
    completed = A["class"] in SUCCESS and A["result"] == "SUCCESS"
    out = {
        "nav_class": A["class"], "class_detail": A.get("class_detail"), "result": A["result"],
        "completed": completed, "contact": (A.get("contact_s") or 0.0) > 0.0,
        "contact_s": A.get("contact_s"), "protective_stop_s": A.get("protective_stop_s"),
        # who closed the last centimetre of the human-AMR touch (secondary sensitivity only;
        # `contact` above is ALL human-AMR contact and is the primary figure)
        "contact_robot_closed": (A.get("contact_robot_closed_s") or 0.0) > 0.0,
        "contact_robot_closed_s": A.get("contact_robot_closed_s"),
        "terminal_protective_stop": A["class"] == "PROTECTIVE_STOP",
        "comfort_deadlock": A["class"] in fm.COMFORT_CLASSES,
        "hold_s": A["hold_s"], "travel_time_s": A["travel_time"],
        "path_length_m": A.get("path_length"), "replans": A.get("replans"), "idle_s": A.get("idle_s"),
        "collision_monitor_stop": (A.get("protective_stop_s") or 0.0) > 0.0,
        "safe_nav_complete": completed and not ((A.get("contact_s") or 0.0) > 0.0),
        "control_mode": a.get("control_mode"),
        "state_declared_at_closest": sc.get("state"), "state_observed_at_closest": obs_at, "state_ok": state_ok,
        "side_ok": side_ok, "crossed": crossed,
        "sector": sc.get("sector"), "intended_sector": scn["intended_sector"], "rel_deg": sc.get("rel_deg"),
        "min_cc_m": sc.get("min_cc_m"),
        "min_footprint_edge_m": None if edge_min is None else round(edge_min, 4),
        "predicted_comfort_at_closest": sc.get("predicted_comfort_at_closest"),
        "predicted_comfort_min": sc.get("predicted_comfort_min"),
        "inside_tested_range": sc.get("inside_tested_range"),
        "amr_speed_at_closest_mps": sc.get("amr_speed_at_closest_mps"),
        "human_speed_at_closest_mps": sc.get("human_speed_at_closest_mps"),
        "relative_speed_at_closest_mps": sc.get("relative_speed_at_closest_mps"),
        "amr_speed_vs_reference_mps": sc.get("amr_speed_vs_reference"),
        "min_ttc_s": (sc.get("min_ttc") or {}).get("ttc_s"),
        "logged_F_med": s2.median(x[0] for x in shapes), "logged_S_med": s2.median(x[1] for x in shapes),
        "logged_R_med": s2.median(x[2] for x in shapes),
    }
    return out, a, sc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    args = ap.parse_args()
    d = PROJECT_ROOT / args.dir if not Path(args.dir).is_absolute() else Path(args.dir)
    plan = json.loads((d / "plan.json").read_text())
    crit = plan["criteria"]
    targets = [crit["primary_target"]] + list(crit["sensitivity_targets"])
    out = d / "analysis"
    (out / "trials").mkdir(parents=True, exist_ok=True)

    rows = []
    for m in read_manifest(d):
        t = d / "trials" / m["trial_id"]
        if not (t / "DONE").exists():
            continue
        done = json.loads((t / "DONE").read_text())
        geom = plan["geometries"][m["env"]]
        scn_name = geom["scenario"]
        scn = plan["scenarios"][scn_name]
        alpha = plan["conditions"][m["condition"]]["alpha"]
        base = {"trial_id": m["trial_id"], "scenario": scn_name, "state": scn["state"], "geometry": geom["geometry"],
                "env": m["env"], "width_m": geom["width_m"], "lane_gap_m": geom.get("lane_gap_m"),
                "condition": m["condition"], "alpha": alpha, "repeat": m["repeat"],
                "physical_signature": geom["physical_signature"]}
        if done.get("infra_failure") or done.get("result") in (None, "INVALID", "NO_ROW"):
            rows.append({**base, "excluded_infra": True, "excluded_reason": done.get("result"),
                         "attempts": done.get("attempts")})
            continue
        r, a, sc = score_launch(t, plan, geom, scn)
        exp = plan["extents_by_alpha"].get(f"{alpha:.2f}", {}).get(scn["state"]) if alpha is not None else None
        r["expected_F"], r["expected_S"], r["expected_R"] = (
            (round(exp["front"], 4), round(exp["side"], 4), round(exp["rear"], 4)) if exp else (None, None, None))
        for tg in targets:
            o = s2.passage_outcome({**r, "comfort": r["predicted_comfort_at_closest"]}, tg)
            r[f"J@{tg}"] = o["J"]
            r[f"reasons@{tg}"] = ";".join(o["reasons"])
            if tg == crit["primary_target"]:
                r["nav_ok"], r["encounter_ok"], r["comfort_ok"] = o["nav_ok"], o["encounter_ok"], o["comfort_ok"]
        domain = []
        if scn["state"] == "station_engaged":
            domain.append("engaged_transfer")
        if r["inside_tested_range"] is False:
            domain.append("distance_outside_tested")
        if (r["amr_speed_vs_reference_mps"] or 0.0) > crit["reference_speed_tolerance_mps"]:
            domain.append("amr_faster_than_reference")
        r["out_of_reference_domain"] = ";".join(domain)
        row = {**base, "excluded_infra": False, **r}
        rows.append(row)
        (out / "trials" / f"{m['trial_id']}.json").write_text(
            json.dumps({"row": row, "reference_score": sc, "analysis": a}, indent=1, default=str))
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with (out / "scale_trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    valid = [r for r in rows if not r["excluded_infra"]]
    cells = defaultdict(list)
    for r in valid:
        cells[(r["scenario"], r["geometry"], r["condition"])].append(r)
    cell_tab = []
    for (scn, gid, cn), rs in sorted(cells.items()):
        e = {"scenario": scn, "geometry": gid, "condition": cn, "alpha": rs[0]["alpha"], "n": len(rs),
             "width_m": rs[0]["width_m"], "lane_gap_m": rs[0]["lane_gap_m"],
             "median_min_cc_completed_m": s2.median(x["min_cc_m"] for x in rs if x["completed"]),
             "median_min_cc_all_m": s2.median(x["min_cc_m"] for x in rs),
             "median_footprint_edge_m": s2.median(x["min_footprint_edge_m"] for x in rs),
             "median_comfort_completed": s2.median(x["predicted_comfort_at_closest"] for x in rs if x["encounter_ok"]),
             "terminal_protective_stop_k": sum(x["terminal_protective_stop"] for x in rs),
             "transient_protective_stop_k": sum((x["protective_stop_s"] or 0) > 0 and not x["terminal_protective_stop"]
                                                for x in rs),
             "comfort_deadlock_k": sum(x["comfort_deadlock"] for x in rs),
             "contact_k": sum(x["contact"] for x in rs),
             "median_hold_s": s2.median(x["hold_s"] for x in rs),
             "sectors": sorted({str(x["sector"]) for x in rs}),
             "classes": sorted({x["nav_class"] for x in rs}),
             "by_target": {}}
        for tg in targets:
            outs = [s2.passage_outcome({**x, "comfort": x["predicted_comfort_at_closest"]}, tg) for x in rs]
            e["by_target"][str(tg)] = s2.rates(outs)
        cell_tab.append(e)

    # selection per context (scenario x geometry) and per state, per target
    sel = {"criteria": crit, "contexts": {}, "states": {}}
    for tg in targets:
        T = str(tg)
        per_state = defaultdict(lambda: defaultdict(dict))   # state -> geometry -> ctx -> by_alpha
        for scn, ent in plan["scenarios"].items():
            if ent.get("status") != "RUN":
                sel["contexts"].setdefault(scn, {"status": ent.get("status"), "feasibility": ent.get("feasibility")})
                continue
            for gid in sorted({c["geometry"] for c in cell_tab if c["scenario"] == scn}):
                cs = [c for c in cell_tab if c["scenario"] == scn and c["geometry"] == gid and c["alpha"] is not None]
                by_alpha = {c["alpha"]: c["by_target"][T] for c in cs}
                dmed = {c["alpha"]: c["median_min_cc_completed_m"] for c in cs}
                res = s2.select_context(by_alpha, crit["joint_min"], 0 + crit["nav_min"], dmed,
                                        crit["min_distance_response_m"])
                res_sens = s2.select_context(by_alpha, crit["joint_min_sensitivity"], crit["nav_min"], dmed,
                                             crit["min_distance_response_m"])
                res["sensitivity_joint_0.90"] = {k: res_sens.get(k) for k in ("status", "alpha_star")}
                sel["contexts"].setdefault(scn, {}).setdefault(gid, {})[T] = res
                if ent.get("primary"):
                    per_state[ent["state"]][gid][scn] = by_alpha
        for st, by_geom in per_state.items():
            for gid, ctxs in by_geom.items():
                stat = {c: sel["contexts"][c][gid][T]["status"] for c in ctxs}
                # a primary context of this state that the plan already recorded as unidentifiable
                # (e.g. no feasible + sensitive pilot geometry) is inherited by the state
                for c, e in plan["scenarios"].items():
                    if e.get("primary") and e.get("state") == st and e.get("status") in (s2.UNIDENTIFIABLE,
                                                                                         s2.INSUFFICIENT):
                        stat[c] = e["status"]
                r = s2.select_state(ctxs, crit["joint_min"], crit["nav_min"], stat)
                r["context_status"] = stat
                if r["status"] == s2.COMMON_OPERATIONAL:
                    r["claim"] = ("operational candidate only: not every context shows an identified "
                                  "comfort bracket; NOT a comfort lower bound")
                sel["states"].setdefault(st, {}).setdefault(gid, {})[T] = r
    summary = {"plan_phase": plan["phase"], "n_launches": len(rows), "n_infra_excluded": len(rows) - len(valid),
               "targets": targets, "cells": cell_tab}
    (out / "scale_summary.json").write_text(json.dumps(summary, indent=1))
    (out / "selected_state_scales.json").write_text(json.dumps(sel, indent=1))
    candidate(out, plan, sel, crit)
    figures(out, cell_tab, plan, crit)
    report(out, plan, summary, sel, rows)
    print(f"{len(rows)} launches ({len(rows) - len(valid)} infra-excluded) -> {out.relative_to(PROJECT_ROOT)}")
    return 0


def candidate(out, plan, sel, crit):
    """Write B3O_SCALE_CANDIDATE.yaml only from a COMMON feasible alpha per
    state; otherwise write the reason and no numbers for that state."""
    T = str(crit["primary_target"])
    lines = ["# GENERATED by scripts/stage2scale_report.py -- Stage 2 uniform-scale candidate.",
             f"# Phase {plan['phase']}. Literature-predicted comfort target {T}/7 (ASSUMPTION).",
             "# NOT an experimentally validated comfort distance; simulation-derived operational scale."]
    if plan["phase"] in ("smoke", "pilot", "confirm"):
        lines.append(f"# DIAGNOSTIC ({plan['phase']} phase): do NOT use as a Stage 3 input (confirm: see CONFIRM_VERDICT.md; candidate values come from the coarse batch).")
    body = {"states": {}}
    for st in s2.STATES:
        g = sel["states"].get(st) or {}
        ent = {}
        for gid, byt in g.items():
            r = byt.get(T, {})
            a = r.get("alpha_star")
            e = {"status": r.get("status"), "alpha_star": a, "context_status": r.get("context_status"),
                 "claim": r.get("claim", "identified bracket in every primary context"
                                      if r.get("status") == s2.COMMON_IDENTIFIED else None)}
            if a is not None:
                ex = plan["extents_by_alpha"][f"{a:.2f}"][st]
                e["extents_m"] = {k: round(v, 4) for k, v in ex.items()}
            ent[gid] = e
        body["states"][st] = ent or {"status": "NOT_TESTED"}
    body["untested_cells"] = ("directions not tested per state keep the template ratio at the selected alpha: "
                              "a TRANSFERRED hypothesis, not tested evidence")
    (out / "B3O_SCALE_CANDIDATE.yaml").write_text("\n".join(lines) + "\n" + yaml.safe_dump(body, sort_keys=False))


def figures(out, cell_tab, plan, crit):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    T = str(crit["primary_target"])
    (out / "figures").mkdir(exist_ok=True)
    for (scn, gid) in sorted({(c["scenario"], c["geometry"]) for c in cell_tab}):
        cs = [c for c in cell_tab if c["scenario"] == scn and c["geometry"] == gid]
        sc = sorted([c for c in cs if c["alpha"] is not None], key=lambda c: c["alpha"])
        fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
        x = [c["alpha"] for c in sc]
        ax[0].plot(x, [c["median_min_cc_completed_m"] for c in sc], "o-", label="median d_cc (completed)")
        ax[0].plot(x, [c["median_min_cc_all_m"] for c in sc], "s:", alpha=0.5, label="median d_cc (all)")
        ax[1].plot(x, [c["median_comfort_completed"] for c in sc], "o-", label="predicted comfort")
        ax[1].axhline(crit["primary_target"], color="red", lw=1.2)
        ax[2].plot(x, [c["by_target"][T]["nav_rate"] for c in sc], "o-", label="nav completion")
        ax[2].plot(x, [c["by_target"][T]["joint_rate"] for c in sc], "s--", label=f"joint @ {T}")
        for c in cs:
            if c["alpha"] is None:
                lab = "B1" if c["condition"] == "B1" else f"B2 {plan['anchor_m']:.2f}"
                col = "grey" if lab == "B1" else "black"
                if c["median_min_cc_all_m"] is not None:
                    ax[0].axhline(c["median_min_cc_all_m"], color=col, ls=":", lw=0.8, label=lab)
                if c["median_comfort_completed"] is not None:
                    ax[1].axhline(c["median_comfort_completed"], color=col, ls=":", lw=0.8, label=lab)
        ax[0].set(xlabel="alpha", ylabel="actual min centre distance [m]", title=f"{scn} {gid}")
        ax[1].set(xlabel="alpha", ylabel="literature-predicted comfort (1-7)", ylim=(1, 7.2))
        ax[2].set(xlabel="alpha", ylabel="rate", ylim=(-0.05, 1.05))
        for a in ax:
            if a.get_legend_handles_labels()[0]:
                a.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(out / "figures" / f"S2S_{scn}_{gid}.png", dpi=110)
        plt.close(fig)


def report(out, plan, summary, sel, rows):
    crit = plan["criteria"]
    T = str(crit["primary_target"])
    L = ["# Stage 2 uniform-scale sweep — report", "",
         f"Phase **{plan['phase']}**. Launches {summary['n_launches']} (infra-excluded {summary['n_infra_excluded']}).",
         "", "> Predicted comfort = published comfort–distance curves (comfort_reference.py) evaluated at the ACTUAL "
         "closest centre distance and bearing. Not a measurement of anyone's comfort, not an experimentally "
         "validated comfort distance. A smallest passing alpha without an observed comfort violation is NOT a "
         "comfort lower bound.", "",
         "## Shapes (alpha = 1, normalised to the R_ref anchor)", "",
         "| state | raw F/S/R | alpha=1 F/S/R |", "|---|---|---|"]
    for st in s2.STATES:
        r, n = plan["templates_raw"][st], plan["templates_normalised"][st]
        L.append(f"| {st} | {r['front']:.3f}/{r['side']:.3f}/{r['rear']:.3f} | "
                 f"{n['front']:.4f}/{n['side']:.4f}/{n['rear']:.4f} |")
    L += ["", f"## Cells (target {T})", "",
          "| scenario | geometry | condition | n | nav k/n | joint k/n (95% CI) | median d_cc m (completed) | "
          "median edge m | median comfort | term. PS | comfort DL | contact | classes |",
          "|---|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for c in summary["cells"]:
        b = c["by_target"][T]
        L.append(f"| {c['scenario']} | {c['geometry']} | {c['condition']} | {c['n']} | {b['nav_k']}/{b['n']} | "
                 f"{b['joint_k']}/{b['n']} {b['joint_ci']} | {c['median_min_cc_completed_m']} | "
                 f"{c['median_footprint_edge_m']} | {c['median_comfort_completed']} | "
                 f"{c['terminal_protective_stop_k']} | {c['comfort_deadlock_k']} | {c['contact_k']} | "
                 f"{', '.join(c['classes'])} |")
    L += ["", f"## Identification and selection (target {T})", ""]
    if plan["phase"] == "confirm":
        L += ["> **Confirm phase:** this batch holds only B1, B2, alpha* and the next smaller alpha, so the "
              "identification status below (which needs alpha 1.00 and the smallest alpha) is NOT meaningful here. "
              "Use `scripts/stage2scale_confirm.py` -> `CONFIRM_VERDICT.md` (status from coarse, cells from this batch).", ""]
    L += [
          "| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |", "|---|---|---|---|---|---|"]
    for scn, g in sel["contexts"].items():
        if "status" in g:
            L.append(f"| {scn} | — | {g['status']} | — | — | {json.dumps(g.get('feasibility'))} |")
            continue
        for gid, byt in g.items():
            r = byt.get(T, {})
            L.append(f"| {scn} | {gid} | {r.get('status')} | {r.get('alpha_star')} | {r.get('distance_response_m')} | "
                     f"{r.get('note', '')} |")
    L += ["", "| state | geometry | status | alpha* | per-context status |", "|---|---|---|---|---|"]
    for st, g in sel["states"].items():
        for gid, byt in g.items():
            r = byt.get(T, {})
            L.append(f"| {st} | {gid} | {r.get('status')} | {r.get('alpha_star')} | {r.get('context_status')} |")
    L += ["", "## Limits", "",
          "- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. "
          "station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.",
          "- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).",
          "- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.",
          "- Overtaking: not run (walker faster than the AMR's limit).", ""]
    (out / "STAGE2_SCALE_REPORT.md").write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    sys.exit(main())
