#!/usr/bin/env python3
"""Stage 2 (simulation): score every trial with the literature reference
model, pick the smallest size per scenario x version x target, write the
derived Adaptive Comfort Space config and the report.

    python3 scripts/stage2sim_report.py                       # the batch in stage2sim.yaml
    python3 scripts/stage2sim_report.py --dir results/batches/261126_stage2sim_smoke

Plain Python 3 (numpy, matplotlib). Reads <dir>/manifest.tsv, plan.json
and trials/<id>/ (DONE, trials.csv, trial_log.jsonl). Writes
    <dir>/analysis/trials/<id>.json     final_metrics.analyse_trial + reference scores
    <dir>/analysis/stage2sim_trials.csv one row per trial
    <dir>/analysis/stage2sim_summary.json per condition, per target; selection
    <dir>/analysis/figures/*.png
    <dir>/analysis/STAGE2SIM_REPORT.md
    <dir>/analysis/B3L_<target>.yaml    derived state_directional config (Stage 3 input)

"Predicted comfort" is what the published curves (comfort_reference.py)
give for the passage the AMR actually made. It is not a measurement of
anyone's comfort.
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

CFG = PROJECT_ROOT / "configs" / "stage2sim" / "stage2sim.yaml"
SUCCESS = {"SUCCESS", "NORMAL_YIELD", "TEMPORARY_HOLD"}
NEAR_CC = 2.5


def manifest(d: Path):
    rows = []
    for line in (d / "manifest.tsv").read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            tid, rep, cond, cfg = line.split("\t")
            rows.append({"trial_id": tid, "repeat": int(rep), "condition": cond, "config": cfg,
                         "env": Path(cfg).stem})
    return rows


def out_of_view_s(ticks, heads, person, half_deg, t0, t1):
    tot, prev = 0.0, None
    for i, tk in enumerate(ticks):
        t = tk["t"]
        pose, p = (tk.get("amr") or {}).get("AMR1"), (tk.get("people") or {}).get(person)
        h = heads[i].get(person) if heads else None
        if prev is not None and pose and p and h is not None and t0 <= t <= t1:
            dx, dy = pose[0] - p["x"], pose[1] - p["y"]
            rel = math.degrees((math.atan2(dy, dx) - h + math.pi) % (2 * math.pi) - math.pi)
            if math.hypot(dx, dy) < NEAR_CC and abs(rel) > half_deg:
                tot += t - prev
        prev = t
    return round(tot, 2)


def score_trial(tdir: Path, S: dict, hw: float):
    a = fm.analyse_trial(tdir, spatial=False)
    D = fm.load(tdir)
    ticks = D["ticks"]
    cfg = fm.config(D["done"]["condition"], D["done"]["config"])
    heads = fm.headings(ticks, cfg) if ticks else []
    people = sorted({k for tk in ticks for k in (tk.get("people") or {})})
    A = a["amrs"]["AMR1"]
    ref = {k: S["reference"][k] for k in ("stationary", "station_engaged", "moving")}
    sc = None
    person = people[0] if people else None
    if person:
        sc = cref.score_passage(ticks, "AMR1", person, hw, ref,
                                heading_of=lambda i, k: heads[i].get(k),
                                t_from=A["t_begin"], t_to=A["t_verdict"],
                                contact_m=float(cfg["humans"]["radius"]) + hw)
    inter = [e for e in a["interactions"] if e["amr"] == "AMR1"]
    return {
        "analysis": a, "reference_score": sc, "person": person,
        "min_edge_m": min((e["min_clearance"] for e in inter), default=None),
        "intrusion_s": round(sum(e.get("intrusion_s") or 0.0 for e in inter), 2),
        "penetration_max_m": max((e.get("penetration") or 0.0 for e in inter), default=0.0),
        "out_of_view_s": out_of_view_s(ticks, heads, person, S["criteria"]["out_of_view_half_angle_deg"],
                                       A["t_begin"], A["t_verdict"]) if person else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=None)
    args = ap.parse_args()
    S = yaml.safe_load(CFG.read_text())
    d = PROJECT_ROOT / (args.dir or S["batch"])
    plan = json.loads((d / "plan.json").read_text())
    hw = plan["amr_half_width_m"]
    crit = S["criteria"]
    out = d / "analysis"
    (out / "trials").mkdir(parents=True, exist_ok=True)
    cond_meta = {}
    for name, e in plan["scenarios"].items():
        for c in e.get("conditions", []):
            cond_meta[(c["env"], c["condition"])] = {**c, "scenario": name}

    rows = []
    for m in manifest(d):
        t = d / "trials" / m["trial_id"]
        if not (t / "DONE").exists():
            continue
        done = json.loads((t / "DONE").read_text())
        cname = Path(m["condition"]).stem
        meta = cond_meta[(m["env"], cname)]
        base = {"trial_id": m["trial_id"], "scenario": meta["scenario"], "variant": meta["variant"],
                "env": m["env"], "version": meta["version"],
                "radius_m": meta["radius_m"], "repeat": m["repeat"], "condition": cname}
        if done.get("infra_failure") or done.get("result") in (None, "INVALID", "NO_ROW"):
            rows.append({**base, "excluded": True, "failure_reason": done.get("result")})
            continue
        r = score_trial(t, S, hw)
        (out / "trials" / f"{m['trial_id']}.json").write_text(json.dumps(r, indent=1, default=str))
        A = r["analysis"]["amrs"]["AMR1"]
        sc = r["reference_score"] or {}
        nav_ok = A["class"] in SUCCESS and (A.get("contact_s") or 0.0) == 0.0
        rows.append({**base, "excluded": False,
                     "state_at_closest": sc.get("state"), "sector_at_closest": sc.get("sector"),
                     "rel_deg": sc.get("rel_deg"),
                     "min_cc_m": sc.get("min_cc_m"), "min_edge_m": r["min_edge_m"],
                     "predicted_comfort_at_closest": sc.get("predicted_comfort_at_closest"),
                     "predicted_comfort_min": sc.get("predicted_comfort_min"),
                     "predicted_comfort_clearance_matched": sc.get("predicted_comfort_clearance_matched"),
                     "inside_tested_range": sc.get("inside_tested_range"),
                     "amr_speed_at_closest_mps": sc.get("amr_speed_at_closest_mps"),
                     "relative_speed_at_closest_mps": sc.get("relative_speed_at_closest_mps"),
                     "min_ttc_s": (sc.get("min_ttc") or {}).get("ttc_s"),
                     "jafari_dmin_ok": sc.get("jafari_dmin_ok"),
                     "intrusion_s": r["intrusion_s"], "penetration_max_m": r["penetration_max_m"],
                     "out_of_view_s": r["out_of_view_s"],
                     "nav_class": A["class"], "nav_ok": nav_ok, "result": A["result"],
                     "travel_time_s": A["travel_time"], "hold_s": A["hold_s"],
                     "contact_s": A.get("contact_s"), "protective_stop_s": A.get("protective_stop_s"),
                     "comfort_deadlock": r["analysis"]["comfort_deadlock"],
                     "failure_reason": None if nav_ok else A.get("class_detail") or A["class"]})
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with (out / "stage2sim_trials.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    # ---- per condition x target
    metric = crit["comfort_metric"]
    groups = defaultdict(list)
    for r in rows:
        if not r["excluded"]:
            groups[(r["scenario"], r["variant"], r["version"], r["radius_m"])].append(r)
    cond_tab = []
    for (scn, var, ver, rad), rs in sorted(groups.items(), key=lambda x: (x[0][0], x[0][1], x[0][2], -(x[0][3] or 9))):
        vals = [x[metric] for x in rs if x[metric] is not None]
        e = {"scenario": scn, "variant": var, "env": rs[0]["env"], "version": ver, "radius_m": rad, "n": len(rs),
             "nav_ok_rate": round(sum(x["nav_ok"] for x in rs) / len(rs), 3),
             "comfort_median": round(sorted(vals)[len(vals) // 2], 3) if vals else None,
             "comfort_min": round(min(vals), 3) if vals else None,
             "min_cc_median": _med([x["min_cc_m"] for x in rs]),
             "min_edge_median": _med([x["min_edge_m"] for x in rs]),
             "hold_median": _med([x["hold_s"] for x in rs]),
             "travel_median": _med([x["travel_time_s"] for x in rs]),
             "sectors": sorted({x["sector_at_closest"] for x in rs if x["sector_at_closest"]}),
             "states": sorted({x["state_at_closest"] for x in rs if x["state_at_closest"]}),
             "violation_rate": {}}
        for tg in crit["targets"]:
            n_ok = sum(1 for v in vals if v >= tg - 1e-9)
            e["violation_rate"][str(tg)] = round(1 - n_ok / len(vals), 3) if vals else None
        cond_tab.append(e)

    # ---- selection
    sel = {}
    for scn, ent in plan["scenarios"].items():
        if ent["status"] != "RUN":
            sel[scn] = {"status": ent["status"], "feasibility": ent.get("feasibility")}
            continue
        sel[scn] = {}
        for var in S["variants"]:
            for ver in ent["versions"]:
                lad = sorted([c for c in cond_tab if c["scenario"] == scn and c["variant"] == var
                              and c["version"] == ver], key=lambda c: -c["radius_m"])
                sel[scn][f"{var}/{ver}"] = {str(tg): select(lad, tg, crit) for tg in crit["targets"]}
    summary = {"metric": metric, "criteria": crit, "conditions": cond_tab, "selection": sel,
               "n_trials": len(rows), "n_excluded": sum(r["excluded"] for r in rows)}
    (out / "stage2sim_summary.json").write_text(json.dumps(summary, indent=1))

    # ---- derived configs
    for tg in crit["targets"]:
        write_derived(out, plan, sel, tg, S, hw)
    figures(out, cond_tab, crit, plan)
    report(out, plan, summary, rows, S)
    print(f"{len(rows)} trials scored -> {out.relative_to(PROJECT_ROOT)}")
    return 0


def _med(xs):
    xs = sorted(x for x in xs if x is not None)
    return round(xs[len(xs) // 2], 3) if xs else None


def select(lad, tg, crit):
    """lad: conditions of one scenario x version, largest radius first."""
    if not lad:
        return {"status": "NO_DATA"}
    passes = []
    for c in lad:
        vr = c["violation_rate"][str(tg)]
        cok = vr is not None and vr <= crit["max_violation_rate"] + 1e-9
        nok = c["nav_ok_rate"] >= crit["min_nav_ok_rate"] - 1e-9
        passes.append((c["radius_m"], cok, nok))
    if not passes[0][1]:
        return {"status": "INFEASIBLE_AT_START", "detail":
                f"predicted comfort below {tg} even at {passes[0][0]} m: needs slow-down / wait / yield / detour",
                "ladder": passes}
    best = None
    for r, cok, nok in passes:
        if cok and nok:
            best = r
        elif crit.get("monotone", True):
            break
    if best is None:
        return {"status": "NEEDS_POLICY", "detail": "comfort met but navigation fails at every comfortable size",
                "ladder": passes}
    lowest = passes[-1][0]
    status = "OK" if best > lowest else "OK_AT_LADDER_END"
    return {"status": status, "R_min_m": best, "ladder": passes}


def write_derived(out, plan, sel, tg, S, hw):
    """state_directional config: tested cells from the simulation, the rest
    from the literature shape (marked), cap = ladder start."""
    cap = plan["ladder_m"][0]
    prov, ext = {}, {}
    cells = {("station_engaged", "rear"): ("SE_rear", "adaptive"), ("stationary", "front"): ("ST_front", "adaptive"),
             ("stationary", "side"): ("ST_side", "adaptive"), ("moving", "side"): ("MV_pass", "static")}
    ref = {k: S["reference"][k] for k in ("stationary", "station_engaged", "moving")}
    for st in ("station_engaged", "stationary", "moving"):
        lit = cref.required_extents(tg, st, hw, ref, S["reference"]["distance_basis"], S["reference"]["engaged_extra_m"])
        for dr in ("front", "side", "rear"):
            key = cells.get((st, dr))
            s = sel.get(key[0], {}).get(f"{S['criteria']['variant']}/{key[1]}", {}).get(str(tg)) if key else None
            if s and s.get("status") in ("OK", "OK_AT_LADDER_END"):
                ext[(st, dr)] = s["R_min_m"]
                prov[f"{st}|{dr}"] = f"simulation ({key[0]}, {key[1]}), {s['status']}"
            elif s:
                ext[(st, dr)] = cap
                prov[f"{st}|{dr}"] = f"simulation {s.get('status')}: kept at {cap} m, needs a behaviour policy"
            elif lit[dr] is not None:
                ext[(st, dr)] = round(min(max(lit[dr], 0.30), cap), 3)
                prov[f"{st}|{dr}"] = "literature only (not tested in simulation)"
            else:
                ext[(st, dr)] = cap
                prov[f"{st}|{dr}"] = f"no reference reaches {tg}: kept at {cap} m"
    mv = {"heading_source": "velocity", "heading_min_speed_mps": 0.1, "speed_mapping": {
        dr: {"intercept_m": ext[("moving", dr)], "gain_s": 0.0, "min_m": ext[("moving", dr)],
             "max_m": ext[("moving", dr)]} for dr in ("front", "side", "rear")}}
    cs = {"policy": "state_directional", "directional": {
        st: {"front_m": ext[(st, "front")], "side_m": ext[(st, "side")], "rear_m": ext[(st, "rear")],
             "heading_source": "body"} for st in ("station_engaged", "stationary")},
        "aisle_scaling": {"enabled": False}}
    cs["directional"]["moving"] = mv
    name = f"B3L_c{str(tg).replace('.', '')}"
    text = ("# GENERATED by scripts/stage2sim_report.py -- Stage 2 (simulation) derived Adaptive Comfort Space.\n"
            f"# Literature-predicted comfort target {tg} (7-point). NOT an experimentally validated comfort distance.\n"
            + "".join(f"#   {k}: {v}\n" for k, v in prov.items())
            + yaml.safe_dump({"scenario": {"name": name, "label": f"B3-L Adaptive (Stage 2 sim, target {tg})"},
                              "comfort_space": cs}, sort_keys=False))
    (out / f"{name}.yaml").write_text(text)


def figures(out, cond_tab, crit, plan):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    (out / "figures").mkdir(exist_ok=True)
    for scn, var in [(a, b) for a in plan["scenarios"] for b in plan["variants"]]:
        cs = [c for c in cond_tab if c["scenario"] == scn and c["variant"] == var]
        if not cs:
            continue
        fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
        for ver, mk in (("static", "o-"), ("adaptive", "s--")):
            v = sorted([c for c in cs if c["version"] == ver], key=lambda c: c["radius_m"])
            if not v:
                continue
            x = [c["radius_m"] for c in v]
            ax[0].plot(x, [c["comfort_median"] for c in v], mk, label=ver)
            ax[0].plot(x, [c["comfort_min"] for c in v], mk, alpha=0.35)
            ax[1].plot(x, [c["min_edge_median"] for c in v], mk, label=f"{ver} edge")
            ax[1].plot(x, [c["min_cc_median"] for c in v], mk, alpha=0.35, label=f"{ver} centre")
            ax[2].plot(x, [c["nav_ok_rate"] for c in v], mk, label=ver)
        b1 = [c for c in cs if c["version"] == "B1"]
        if b1:
            ax[0].axhline(b1[0]["comfort_median"] or 0, color="grey", ls=":", label="B1")
        for tg in crit["targets"]:
            ax[0].axhline(tg, color="red", lw=0.6 if tg != crit["primary_target"] else 1.4, alpha=0.5)
        ax[1].plot([0.3, 0.95], [0.3, 0.95], color="grey", lw=0.6)
        ax[0].set(xlabel="Comfort Space size in tested direction [m]", ylabel="literature-predicted comfort (1-7)",
                  title=f"{scn} ({var}): predicted comfort")
        ax[1].set(xlabel="size [m]", ylabel="actual min distance [m]", title="set size vs actual distance")
        ax[2].set(xlabel="size [m]", ylabel="navigation OK rate", ylim=(-0.05, 1.05), title="navigation")
        for a in ax:
            if a.get_legend_handles_labels()[0]:
                a.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(out / "figures" / f"S2_{scn}_{var}.png", dpi=110)
        plt.close(fig)


def report(out, plan, summary, rows, S):
    crit = S["criteria"]
    L = ["# Stage 2 (simulation) — minimum Comfort Space by literature-predicted comfort", "",
         "> **Predicted comfort** = what published human-rating curves give for the passage the AMR actually made "
         "(`comfort_core/comfort_reference.py`). It is not a measurement of comfort and not an experimentally "
         "validated comfort distance. Same reference model for every policy.", "",
         f"Trials scored: {summary['n_trials']} (excluded infra: {summary['n_excluded']}). Ladder {plan['ladder_m']} m; "
         f"corridors {json.dumps(plan['variants'])}, selection on `{crit['variant']}`. Metric `{summary['metric']}`; pass = violation rate ≤ {crit['max_violation_rate']} "
         f"and navigation OK rate ≥ {crit['min_nav_ok_rate']} (ASSUMPTIONS), monotone over the ladder. "
         f"Primary target {crit['primary_target']} (ASSUMPTION).", "",
         "## References used", ""] + [f"- {x}" for x in plan["references"]] + [
         "", "## Literature-required extent (person centre → AMR edge, m) per target", "",
         "| scenario | " + " | ".join(str(t) for t in crit["targets"]) + " |",
         "|---|" + "---|" * len(crit["targets"])]
    for scn, e in plan["scenarios"].items():
        if e["status"] != "RUN":
            continue
        tst = e["tested"]
        L.append(f"| {scn} ({tst}) | " + " | ".join(
            str((e["required_extent_by_target"][str(t)] or {}).get(tst)) for t in crit["targets"]) + " |")
    L += ["", "## Per condition", "",
          "| scenario | corridor | version | size m | n | nav OK | pred. comfort median (min) | min centre m | min edge m | hold s | "
          + " | ".join(f"viol@{t}" for t in crit["targets"]) + " |",
          "|---|---|---|---:|---:|---:|---|---:|---:|---:|" + "---:|" * len(crit["targets"])]
    for c in summary["conditions"]:
        L.append(f"| {c['scenario']} | {c['variant']} | {c['version']} | {c['radius_m']} | {c['n']} | {c['nav_ok_rate']} | "
                 f"{c['comfort_median']} ({c['comfort_min']}) | {c['min_cc_median']} | {c['min_edge_median']} | "
                 f"{c['hold_median']} | " + " | ".join(str(c["violation_rate"][str(t)]) for t in crit["targets"]) + " |")
    L += ["", "## Selection (smallest size that passes)", "",
          "| scenario | corridor/version | " + " | ".join(str(t) for t in crit["targets"]) + " |",
          "|---|---|" + "---|" * len(crit["targets"])]
    for scn, s in summary["selection"].items():
        if "status" in s:
            L.append(f"| {scn} | — | " + " | ".join(s["status"] for _ in crit["targets"]) + " |")
            continue
        for ver, byt in s.items():
            L.append(f"| {scn} | {ver} | " + " | ".join(
                (f"{byt[str(t)].get('R_min_m')} ({byt[str(t)]['status']})" if byt[str(t)].get("R_min_m")
                 else byt[str(t)]["status"]) for t in crit["targets"]) + " |")
    L += ["", "## Not run / limits", ""]
    for scn, e in plan["scenarios"].items():
        if e["status"] != "RUN":
            L.append(f"- **{scn}: {e['status']}** — {json.dumps(e.get('feasibility'))}")
    L += ["- Reference curves were measured with robots at 0.35 m/s; this AMR passes at the speed in the trial table "
          "(column `amr_speed_at_closest_mps`). Speed is not in the model (Neggers 2022 Front. gives per-speed "
          "parameters only in figures), so predicted comfort is likely optimistic at higher AMR speed.",
          "- station_engaged uses the standing-participant curve; task engagement itself is not in any reference.",
          "- Moving: head-on passing only, one curve for all sides; overtaking has no numeric reference.",
          "- Out-of-view time, min TTC (constant velocity, contact = person radius + AMR half width), relative speed: kinematic indicators, not comfort.",
          "- Run-to-run variance: identical trials are repeated; see the min/median spread per condition.",
          "", "Files: `stage2sim_trials.csv`, `stage2sim_summary.json`, `B3L_c*.yaml`, `figures/`."]
    (out / "STAGE2SIM_REPORT.md").write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    sys.exit(main())
