#!/usr/bin/env python3
"""Stage 3 (stage23_900) results: 8 people / 3 Nav2 AMRs, five policies on matched seeds.

    python3 scripts/final_stage23_900_report3.py --dir results/batches/261127_stage23_900_s3main \
        [--dup results/batches/261127_stage23_900_s3dup --pilot results/batches/261127_stage23_900_s3pilot]

Per launch (comfort_core/final_metrics.analyse_trial + the trial logs):
  nav (system)   all three AMRs reached their missions (SUCCESS / SUCCESS_IDLE / SUCCESS_LONG_IDLE)
  contact        any human-AMR PHYSICAL contact (trials.csv contact_person_seconds for AMR1,
                 fleet_trials.csv contact_person_s for a1 / a2) PLUS the robot-closed share
                 (contact_robot_seconds / contact_robot_s). The simulator splits one human-AMR
                 touch by who closed the last centimetre; both halves are human-AMR contact
                 (protocol: worker-induced contact is never hidden). The split is reported.
  encounters     every (AMR, person) pair with min centre distance < encounter.radius_m while that AMR
                 is on its mission; scored with comfort_reference.score_passage at the closest point
                 (ACTUAL distance, bearing, state); C = None -> NOT_SCORABLE (never a pass)
  primary joint  nav AND no human contact AND >= 1 encounter AND every encounter scorable AND
                 every encounter C >= 5.0 (4.5 / 5.5 sensitivity)
  system time    max over AMRs of mission time (successes); capped = failures set to the cap
                 (trial_start..navigation.max_trial_time for AMR1, i.e. max_trial_time)
Winner rule: plan.json `winner_rule` (from the yaml, fixed before the data). Paired exact McNemar
(Holm), paired bootstrap of capped system time (2000 resamples, seed 7). Exploratory at n = 30.
Writes <dir>/analysis/: per_trial.csv, per_encounter.csv, STAGE3_RESULTS.csv, STAGE3_RESULTS.md,
stage3_selection.json, figures/S3_*.png.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import comfort_reference as cref  # noqa: E402
from comfort_core import final_metrics as fm  # noqa: E402
from comfort_core import stage2scale as s2  # noqa: E402

TARGETS = (5.0, 4.5, 5.5)


def F(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def pctl(xs, q):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return round(xs[lo] + (xs[hi] - xs[lo]) * (k - lo), 4)


def rate(k, n):
    lo, hi = s2.wilson(k, n)
    return {"k": k, "n": n, "rate": round(k / n, 4) if n else None, "ci": [lo, hi]}


def fr(r):
    return "—" if not r["n"] else f"{r['k']}/{r['n']} ({r['ci'][0]:.2f}–{r['ci'][1]:.2f})"


def mcnemar_exact(b, c):
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = sum(math.comb(n, i) for i in range(0, k + 1)) / 2 ** n
    return min(1.0, 2 * p)


def holm(ps):
    order = sorted(range(len(ps)), key=lambda i: ps[i])
    adj = [None] * len(ps)
    run = 0.0
    for r, i in enumerate(order):
        run = max(run, min(1.0, (len(ps) - r) * ps[i]))
        adj[i] = run
    return adj


def read_manifest(d):
    out = []
    for ln in (d / "manifest.tsv").read_text().splitlines():
        if ln and not ln.startswith("#"):
            tid, rep, cond, cfg = ln.split("\t")
            out.append({"trial_id": tid, "seed": int(rep), "condition": Path(cond).stem, "config": cfg})
    return out


def score_trial(tdir, plan):
    a = fm.analyse_trial(tdir, spatial=False)
    D = fm.load(tdir)
    ticks = D["ticks"]
    cfg = fm.config(D["done"]["condition"], D["done"]["config"])
    heads = fm.headings(ticks, cfg) if ticks else []
    hl, hw = plan["amr_half_length_m"], plan["amr_half_width_m"]
    R = float(plan["encounter"]["radius_m"])
    ref = plan["reference"]
    cap = float(cfg["navigation"].get("max_trial_time", 120.0))
    # person_closed / robot_closed: who closed the last centimetre of a HUMAN-AMR touch.
    human_contact = {"AMR1": F((D["amr1"] or {}).get("contact_person_seconds")) or 0.0}
    robot_contact = {"AMR1": F((D["amr1"] or {}).get("contact_robot_seconds")) or 0.0}
    for f in D["fleet"]:
        human_contact[f["robot"]] = F(f.get("contact_person_s")) or 0.0
        robot_contact[f["robot"]] = F(f.get("contact_robot_s")) or 0.0
    people = sorted({k for tk in ticks for k in (tk.get("people") or {})})
    encs = []
    for n, A in a["amrs"].items():
        t0, t1 = A["t_begin"], A["t_verdict"]
        for k in people:
            dmin, emin = None, None
            for tk in ticks:
                t = tk.get("t")
                pose, p = (tk.get("amr") or {}).get(n), (tk.get("people") or {}).get(k)
                if t is None or not (t0 <= t <= t1) or not pose or not p:
                    continue
                dd = math.hypot(pose[0] - p["x"], pose[1] - p["y"])
                dmin = dd if dmin is None else min(dmin, dd)
                if dd < R + hl:
                    e = fm.rect_clearance(pose, hl, hw, p["x"], p["y"])
                    emin = e if emin is None else min(emin, e)
            if dmin is None or dmin >= R:
                continue
            sc = cref.score_passage(ticks, n, k, hw, ref, heading_of=lambda i, kk: heads[i].get(kk),
                                    t_from=t0, t_to=t1, contact_m=plan["human_radius_m"] + hw) or {}
            c = sc.get("predicted_comfort_at_closest")
            encs.append({"amr": n, "human": k, "state": sc.get("state"), "sector": sc.get("sector"),
                         "min_cc_m": sc.get("min_cc_m"), "min_footprint_edge_m": None if emin is None else round(emin, 4),
                         "geometric_overlap": emin is not None and emin < plan["human_radius_m"],
                         "predicted_comfort": c, "predicted_comfort_min": sc.get("predicted_comfort_min"),
                         "scorable": c is not None, "rel_deg": sc.get("rel_deg"),
                         "amr_speed_mps": sc.get("amr_speed_at_closest_mps"),
                         "human_speed_mps": sc.get("human_speed_at_closest_mps"),
                         "inside_tested_range": sc.get("inside_tested_range"), "t_closest": sc.get("t_closest"),
                         "clearance_matched_comfort": sc.get("predicted_comfort_clearance_matched")})
    names = list(a["amrs"])
    nav = all(a["amrs"][n]["class"] in fm.SUCCESS_CLASSES for n in names)
    contact = any(v > 0 for v in human_contact.values()) or any(v > 0 for v in robot_contact.values())
    times = [a["amrs"][n]["travel_time"] for n in names]
    sys_time = max(times) if nav and all(t is not None for t in times) else None
    row = {"nav": nav, "contact": contact,
           "human_contact_s": round(sum(human_contact.values()) + sum(robot_contact.values()), 2),
           "contact_person_closed_s": round(sum(human_contact.values()), 2),
           "contact_robot_closed_s": round(sum(robot_contact.values()), 2),
           "amr_success_k": sum(a["amrs"][n]["class"] in fm.SUCCESS_CLASSES for n in names), "amr_n": len(names),
           "classes": ";".join(f"{n}:{a['amrs'][n]['class']}" for n in names),
           "comfort_deadlock": a["comfort_deadlock"],
           "robot_robot_block": any(a["amrs"][n]["class"] == "ROBOT_ROBOT_BLOCK" for n in names),
           "nav2_abort": any(a["amrs"][n]["class"] == "NAV2_ABORT" for n in names),
           "protective_stop_trial": any((a["amrs"][n].get("protective_stop_s") or 0) > 0 for n in names),
           "protective_stop_s": round(sum(a["amrs"][n].get("protective_stop_s") or 0 for n in names), 2),
           "terminal_protective_stop": any(a["amrs"][n]["class"] == "PROTECTIVE_STOP" for n in names),
           "idle_s": round(sum(a["amrs"][n].get("idle_s") or 0 for n in names), 2),
           "replans": sum(a["amrs"][n].get("replans") or 0 for n in names),
           "path_length_m": round(sum(a["amrs"][n].get("path_length") or 0 for n in names), 3),
           "amr1_path_length_m": a["amrs"]["AMR1"].get("path_length"),
           "system_time_s": sys_time, "system_time_capped_s": sys_time if sys_time is not None else cap,
           "min_cc_m": min((e["min_cc_m"] for e in encs if e["min_cc_m"] is not None), default=None),
           "min_edge_m": min((e["min_footprint_edge_m"] for e in encs if e["min_footprint_edge_m"] is not None),
                             default=None),
           "n_encounters": len(encs), "n_scorable": sum(e["scorable"] for e in encs),
           "control_mode": a.get("control_mode")}
    for tg in TARGETS:
        allok = bool(encs) and all(e["scorable"] for e in encs) and all(e["predicted_comfort"] >= tg - 1e-9
                                                                          for e in encs)
        row[f"joint@{tg}"] = nav and not contact and allok
        if not nav:
            cat = "FAIL_NAV"
        elif contact:
            cat = "FAIL_CONTACT"
        elif not encs or not all(e["scorable"] for e in encs):
            cat = "NOT_SCORABLE"
        elif not allok:
            cat = "FAIL_COMFORT"
        else:
            cat = "JOINT_PASS"
        row[f"category@{tg}"] = cat
    # SECONDARY sensitivity (declared before the main data): contact counted only when the AMR
    # closed the last centimetre. Scripted people never avoid (configs/common.yaml avoid_robot).
    row["contact_rc"] = any(v > 0 for v in robot_contact.values())
    allok5 = bool(encs) and all(e["scorable"] for e in encs) and all(e["predicted_comfort"] >= 5.0 - 1e-9 for e in encs)
    row["joint_rc@5.0"] = nav and not row["contact_rc"] and allok5
    return row, encs, a


def boot_median_diff(x, y, B=2000, seed=7):
    pairs = [(a, b) for a, b in zip(x, y) if a is not None and b is not None]
    if len(pairs) < 3:
        return None
    rng = random.Random(seed)
    obs = s2.median(b - a for a, b in pairs)
    bs = sorted(s2.median(b - a for a, b in (rng.choice(pairs) for _ in pairs)) for _ in range(B))
    return {"median_diff": round(obs, 2), "ci95": [round(bs[int(0.025 * B)], 2), round(bs[int(0.975 * B) - 1], 2)]}


def analyse_batch(d, plan):
    rows, encs_all = [], []
    out = d / "analysis"
    (out / "trials").mkdir(parents=True, exist_ok=True)
    fam = {int(s): m["family"] for s, m in plan["seeds"].items()}
    for m in read_manifest(d):
        t = d / "trials" / m["trial_id"]
        if not (t / "DONE").exists():
            continue
        done = json.loads((t / "DONE").read_text())
        base = {"trial_id": m["trial_id"], "seed": m["seed"], "family": fam.get(m["seed"]), "condition": m["condition"],
                "alpha": plan["conditions"][m["condition"]]["alpha"], "wall_s": done.get("wall_s"),
                "attempts": len(done.get("attempts") or [])}
        if done.get("infra_failure") or done.get("result") in (None, "INVALID", "NO_ROW"):
            rows.append({**base, "excluded_infra": True, "excluded_reason": done.get("result") or done.get("infra_failure")})
            continue
        r, encs, a = score_trial(t, plan)
        rows.append({**base, "excluded_infra": False, **r})
        for e in encs:
            encs_all.append({**base, **e})
        (out / "trials" / f"{m['trial_id']}.json").write_text(json.dumps({"row": rows[-1], "encounters": encs,
                                                                           "analysis": a}, default=str))
    return rows, encs_all


def write_csv(p, rows):
    if not rows:
        p.write_text("")
        return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with p.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def select(tab, by, plan, jk, ck):
    """Winner rule (yaml stage3.winner_rule) on joint key jk and contact key ck."""
    elig = [e for e in tab if e["n"] and e[ck]["k"] <= 1 and e["nav"]["rate"] >= 0.80 - 1e-9
            and e[jk]["rate"] >= 0.80 - 1e-9 and (e["coverage"] or 0) >= 0.90 - 1e-9]
    selection = {"rule": plan["winner_rule"], "joint_key": jk, "contact_key": ck,
                 "eligible": [e["policy"] for e in elig],
                 "screen": {e["policy"]: {"contact_k": e[ck]["k"], "nav": e["nav"]["rate"],
                                          "joint": e[jk]["rate"], "coverage": e["coverage"]} for e in tab}}
    pairs = {}
    if not elig:
        selection["decision"] = "NO_ACCEPTABLE_SCALE"
    else:
        best = max(elig, key=lambda e: (e[jk]["rate"], e["nav"]["rate"]))
        others = [e for e in elig if e is not best]
        ps = []
        for e in others:
            seeds = sorted(set(by[best["policy"]]) & set(by[e["policy"]]))
            b = sum(by[best["policy"]][s][jk] and not by[e["policy"]][s][jk] for s in seeds)
            c = sum(by[e["policy"]][s][jk] and not by[best["policy"]][s][jk] for s in seeds)
            p = mcnemar_exact(b, c)
            ps.append(p)
            pairs[e["policy"]] = {"vs": best["policy"], "best_only": b, "other_only": c, "n_pairs": len(seeds), "p": round(p, 4)}
        for (k, v), pa in zip(pairs.items(), holm(ps)):
            v["p_holm"] = round(pa, 4)
        top = [best] + [e for e in others if pairs[e["policy"]]["p_holm"] >= 0.05]

        def dom(x, y):   # x dominates y
            ge = (x[jk]["rate"] >= y[jk]["rate"] and x["nav"]["rate"] >= y["nav"]["rate"]
                  and x["median_time_capped_s"] <= y["median_time_capped_s"])
            gt = (x[jk]["rate"] > y[jk]["rate"] or x["nav"]["rate"] > y["nav"]["rate"]
                  or x["median_time_capped_s"] < y["median_time_capped_s"])
            return ge and gt
        pareto = [e for e in top if not any(dom(o, e) for o in top if o is not e)]
        pick = min(pareto, key=lambda e: e["median_time_capped_s"])
        selection.update({"reference_best_joint": best["policy"], "top_set": [e["policy"] for e in top],
                          "pareto": [e["policy"] for e in pareto], "decision": pick["policy"],
                          "tie_break": "smallest median capped system time within the Pareto front",
                          "statistically_distinguishable_from_best": [k for k, v in pairs.items() if v["p_holm"] < 0.05]})
    selection["mcnemar_vs_best"] = pairs
    return selection, pairs, elig


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--dup", default="")
    ap.add_argument("--pilot", default="")
    a = ap.parse_args()
    d = PROJECT_ROOT / a.dir
    plan = json.loads((d / "plan.json").read_text())
    out = d / "analysis"
    rows, encs = analyse_batch(d, plan)
    write_csv(out / "per_trial.csv", rows)
    write_csv(out / "per_encounter.csv", encs)
    valid = [r for r in rows if not r["excluded_infra"]]
    pols = list(plan["policies"].items())          # [(C1, B2_r0.950), ...]
    by = {cn: {r["seed"]: r for r in valid if r["condition"] == cn} for _c, cn in pols}
    tab = []
    for case, cn in pols:
        rs = list(by[cn].values())
        n = len(rs)
        es = [e for e in encs if e["condition"] == cn]
        e = {"case": case, "policy": cn, "alpha": plan["conditions"][cn]["alpha"], "n": n,
             "nav": rate(sum(r["nav"] for r in rs), n),
             "safe_nav": rate(sum(r["nav"] and not r["contact"] for r in rs), n),
             "contact": rate(sum(r["contact"] for r in rs), n),
             "contact_rc": rate(sum(r["contact_rc"] for r in rs), n),
             "joint_rc@5.0": rate(sum(r["joint_rc@5.0"] for r in rs), n),
             "amr_completion": rate(sum(r["amr_success_k"] for r in rs), sum(r["amr_n"] for r in rs)),
             "comfort_deadlock": rate(sum(r["comfort_deadlock"] for r in rs), n),
             "robot_robot_block": rate(sum(r["robot_robot_block"] for r in rs), n),
             "nav2_abort": rate(sum(r["nav2_abort"] for r in rs), n),
             "protective_stop_trial": rate(sum(r["protective_stop_trial"] for r in rs), n),
             "terminal_protective_stop": rate(sum(r["terminal_protective_stop"] for r in rs), n),
             "median_contact_s": pctl([r["human_contact_s"] for r in rs if r["contact"]], 0.5),
             "median_stop_s": pctl([r["protective_stop_s"] for r in rs], 0.5),
             "median_idle_s": pctl([r["idle_s"] for r in rs], 0.5),
             "median_replans": pctl([r["replans"] for r in rs], 0.5),
             "median_time_success_s": pctl([r["system_time_s"] for r in rs], 0.5),
             "median_time_capped_s": pctl([r["system_time_capped_s"] for r in rs], 0.5),
             "median_path_m": pctl([r["path_length_m"] for r in rs if r["nav"]], 0.5),
             "median_min_cc_m": pctl([r["min_cc_m"] for r in rs], 0.5),
             "median_min_edge_m": pctl([r["min_edge_m"] for r in rs], 0.5),
             "encounters": len(es), "scorable": sum(x["scorable"] for x in es),
             "coverage": round(sum(x["scorable"] for x in es) / len(es), 4) if es else None,
             "median_comfort": pctl([x["predicted_comfort"] for x in es], 0.5),
             "p10_comfort": pctl([x["predicted_comfort"] for x in es], 0.1),
             "encounter_ge_5": rate(sum((x["predicted_comfort"] or -1) >= 5.0 for x in es if x["scorable"]),
                                    sum(x["scorable"] for x in es)),
             "per_state": {}, "per_family_joint": {}}
        for st in ("stationary", "station_engaged", "moving"):
            xs = [x for x in es if x["state"] == st and x["scorable"]]
            e["per_state"][st] = {"n": len(xs), "median_C": pctl([x["predicted_comfort"] for x in xs], 0.5),
                                  "p10_C": pctl([x["predicted_comfort"] for x in xs], 0.1),
                                  "ge_5": rate(sum(x["predicted_comfort"] >= 5.0 for x in xs), len(xs)),
                                  "median_d_cc": pctl([x["min_cc_m"] for x in xs], 0.5)}
        for fam in ("NOMINAL", "BOTTLENECK", "CROSSING"):
            xs = [r for r in rs if r["family"] == fam]
            e["per_family_joint"][fam] = rate(sum(r["joint@5.0"] for r in xs), len(xs))
        for tg in TARGETS:
            e[f"joint@{tg}"] = rate(sum(r[f"joint@{tg}"] for r in rs), n)
            e[f"categories@{tg}"] = {c: sum(r[f"category@{tg}"] == c for r in rs)
                                     for c in ("JOINT_PASS", "FAIL_NAV", "FAIL_CONTACT", "NOT_SCORABLE", "FAIL_COMFORT")}
        tab.append(e)
    # winner rule (fixed before the data); primary = ALL human-AMR contact
    selection, pairs, elig = select(tab, by, plan, "joint@5.0", "contact")
    # SECONDARY sensitivity, declared in the yaml before the main data: robot-closed contact only
    sel_rc, _p, _e = select(tab, by, plan, "joint_rc@5.0", "contact_rc")
    selection["secondary_robot_closed_contact"] = sel_rc
    selection["mcnemar_vs_best"] = pairs
    # paired descriptive comparisons vs B2 (all policies)
    b2 = pols[0][1]
    vs_b2 = {}
    for _c, cn in pols[1:]:
        seeds = sorted(set(by[b2]) & set(by[cn]))
        b = sum(by[b2][s]["joint@5.0"] and not by[cn][s]["joint@5.0"] for s in seeds)
        c = sum(by[cn][s]["joint@5.0"] and not by[b2][s]["joint@5.0"] for s in seeds)
        nb = sum(by[b2][s]["nav"] and not by[cn][s]["nav"] for s in seeds)
        nc = sum(by[cn][s]["nav"] and not by[b2][s]["nav"] for s in seeds)
        vs_b2[cn] = {"n_pairs": len(seeds), "joint_B2_only": b, "joint_policy_only": c,
                     "joint_p": round(mcnemar_exact(b, c), 4), "nav_B2_only": nb, "nav_policy_only": nc,
                     "nav_p": round(mcnemar_exact(nb, nc), 4),
                     "capped_time_policy_minus_B2": boot_median_diff([by[b2][s]["system_time_capped_s"] for s in seeds],
                                                                     [by[cn][s]["system_time_capped_s"] for s in seeds])}
    ph = holm([v["joint_p"] for v in vs_b2.values()])
    for v, p in zip(vs_b2.values(), ph):
        v["joint_p_holm"] = round(p, 4)
    selection["paired_vs_B2"] = vs_b2
    # duplicate-control diagnostic
    dup = None
    if a.dup:
        dd = PROJECT_ROOT / a.dup
        dplan = json.loads((dd / "plan.json").read_text())
        drows, _ = analyse_batch(dd, dplan)
        runs = defaultdict(list)
        for r in drows:
            if not r["excluded_infra"]:
                runs[r["seed"]].append(r)
        if a.pilot:
            pd = PROJECT_ROOT / a.pilot
            prows = list(csv.DictReader((pd / "analysis" / "per_trial.csv").open())) if (pd / "analysis" / "per_trial.csv").exists() else []
            for r in prows:
                if r.get("condition") == dplan["conditions"] and False:
                    pass
        dup = {}
        for s, rs in sorted(runs.items()):
            dup[s] = {"n": len(rs), "nav": [r["nav"] for r in rs], "joint@5.0": [r["joint@5.0"] for r in rs],
                      "system_time_capped_s": [r["system_time_capped_s"] for r in rs],
                      "nav_disagree": len({r["nav"] for r in rs}) > 1,
                      "joint_disagree": len({r["joint@5.0"] for r in rs}) > 1}
        write_csv(out / "duplicate_control.csv", drows)
    selection["duplicate_control"] = dup
    (out / "stage3_selection.json").write_text(json.dumps({"policies": tab, "selection": selection}, indent=1, default=str))
    flat = []
    for e in tab:
        row = {}
        for k, v in e.items():
            if isinstance(v, dict) and "k" in v:
                row[f"{k}_k"], row[f"{k}_n"], row[f"{k}_rate"] = v["k"], v["n"], v["rate"]
                row[f"{k}_ci_lo"], row[f"{k}_ci_hi"] = v["ci"]
            elif isinstance(v, dict):
                row[k] = json.dumps(v, default=str)
            else:
                row[k] = v
        flat.append(row)
    write_csv(out / "STAGE3_RESULTS.csv", flat)
    # figures
    figs = out / "figures"
    figs.mkdir(exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        lab = [e["policy"].replace("S3_a", "α ").replace("B2_r0.950", "B2 0.95") for e in tab]
        fig, ax = plt.subplots(1, 4, figsize=(18, 4))
        x = range(len(tab))
        for i, (k, t) in enumerate((("nav", "nav (all 3 AMRs)"), ("safe_nav", "safe nav"), ("joint@5.0", "primary joint @5.0"),
                                    ("contact", "human contact trials"))):
            v = [e[k]["rate"] or 0 for e in tab]
            lo = [max(0.0, (e[k]["rate"] or 0) - (e[k]["ci"][0] or 0)) for e in tab]
            hi = [max(0.0, (e[k]["ci"][1] or 0) - (e[k]["rate"] or 0)) for e in tab]
            ax[i].bar(x, v, yerr=[lo, hi], capsize=3)
            ax[i].set_xticks(list(x))
            ax[i].set_xticklabels(lab, rotation=30)
            ax[i].set_ylim(0, 1.05)
            ax[i].set_title(t + " (Wilson 95 %)", fontsize=10)
        fig.tight_layout()
        fig.savefig(figs / "S3_rates.png", dpi=130)
        plt.close(fig)
        fig, ax = plt.subplots(1, 3, figsize=(15, 4))
        ax[0].boxplot([[r["system_time_capped_s"] for r in by[e["policy"]].values()] for e in tab], labels=lab)
        ax[0].set_title("system time, failures capped (s)")
        ax[1].boxplot([[r["path_length_m"] for r in by[e["policy"]].values() if r["nav"]] or [0] for e in tab], labels=lab)
        ax[1].set_title("total path length, successes (m)")
        ax[2].boxplot([[x_["predicted_comfort"] for x_ in encs if x_["condition"] == e["policy"] and x_["scorable"]] or [0]
                       for e in tab], labels=lab)
        ax[2].axhline(5.0, color="red", lw=1)
        ax[2].set_title("predicted comfort per encounter")
        for x_ in ax:
            x_.tick_params(axis="x", rotation=30)
        fig.tight_layout()
        fig.savefig(figs / "S3_time_path_comfort.png", dpi=130)
        plt.close(fig)
        fig, ax = plt.subplots(figsize=(6, 5))
        for e, l in zip(tab, lab):
            ax.scatter(e["median_time_capped_s"], e["joint@5.0"]["rate"], s=60)
            ax.annotate(l, (e["median_time_capped_s"], e["joint@5.0"]["rate"]), textcoords="offset points", xytext=(5, 5))
        ax.set_xlabel("median capped system time (s)")
        ax.set_ylabel("primary joint rate @5.0")
        ax.set_title("Comfort–Navigation trade-off (Pareto view)")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(figs / "S3_pareto.png", dpi=130)
        plt.close(fig)
    except Exception as ex:        # noqa: BLE001
        print("figures skipped:", ex)
    # markdown
    excl = [r for r in rows if r["excluded_infra"]]
    L = [f"# STAGE 3 RESULTS — stage23_900 {plan['phase']}", "",
         "> 8 people / 3 Nav2 AMRs, five policies on the same seeds. Predicted comfort = literature curve at the "
         "ACTUAL closest distance / bearing / state per encounter (not a subjective rating). One global alpha "
         "across states with state-specific template ratios -- this does NOT test separately optimised "
         "state-wise alphas.", "",
         f"Launches {len(rows)}; valid {len(valid)}; infra-excluded {len(excl)}"
         + (f" ({', '.join(r['trial_id'] for r in excl)})" if excl else "")
         + f". Encounter = (AMR, person) with min d_cc < {plan['encounter']['radius_m']} m during that AMR's mission.", "",
         "| case | policy | n | nav | safe nav | joint@5.0 (95 % CI) | joint@4.5 | joint@5.5 | contact | AMR completion | "
         "comfort DL | robot block | Nav2 abort | PS trials | med time succ s | med time capped s | med path m | "
         "med replans | med idle s | coverage | med C | P10 C |",
         "|---|---|---:|---|---|---|---|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for e in tab:
        L.append(f"| {e['case']} | {e['policy']} | {e['n']} | {fr(e['nav'])} | {fr(e['safe_nav'])} | {fr(e['joint@5.0'])} | "
                 f"{e['joint@4.5']['k']}/{e['n']} | {e['joint@5.5']['k']}/{e['n']} | {fr(e['contact'])} | "
                 f"{e['amr_completion']['k']}/{e['amr_completion']['n']} | {e['comfort_deadlock']['k']} | "
                 f"{e['robot_robot_block']['k']} | {e['nav2_abort']['k']} | {e['protective_stop_trial']['k']} | "
                 f"{e['median_time_success_s']} | {e['median_time_capped_s']} | {e['median_path_m']} | "
                 f"{e['median_replans']} | {e['median_idle_s']} | {e['coverage']} | {e['median_comfort']} | {e['p10_comfort']} |")
    L += ["", "Primary categories @5.0:", "", "| policy | JOINT_PASS | FAIL_NAV | FAIL_CONTACT | NOT_SCORABLE | FAIL_COMFORT |",
          "|---|---:|---:|---:|---:|---:|"]
    for e in tab:
        c = e["categories@5.0"]
        L.append(f"| {e['policy']} | {c['JOINT_PASS']} | {c['FAIL_NAV']} | {c['FAIL_CONTACT']} | {c['NOT_SCORABLE']} | {c['FAIL_COMFORT']} |")
    L += ["", "Per state (scorable encounters):", "", "| policy | state | n | med d_cc | med C | P10 C | C >= 5.0 |",
          "|---|---|---:|---:|---:|---:|---|"]
    for e in tab:
        for st, v in e["per_state"].items():
            L.append(f"| {e['policy']} | {st} | {v['n']} | {v['median_d_cc']} | {v['median_C']} | {v['p10_C']} | {fr(v['ge_5'])} |")
    L += ["", "Joint @5.0 per family:", "", "| policy | NOMINAL | BOTTLENECK | CROSSING |", "|---|---|---|---|"]
    for e in tab:
        f_ = e["per_family_joint"]
        L.append(f"| {e['policy']} | {fr(f_['NOMINAL'])} | {fr(f_['BOTTLENECK'])} | {fr(f_['CROSSING'])} |")
    L += ["", "## Selection (rule fixed before the data)", "", "```", plan["winner_rule"].strip(), "```", "",
          f"Eligible: {selection['eligible'] or 'none'}. **Decision: {selection['decision']}**"
          + (f"; top set {selection.get('top_set')}, Pareto {selection.get('pareto')}" if elig else ""), "",
          "| vs best | best only | other only | pairs | p | p (Holm) |", "|---|---:|---:|---:|---:|---:|"]
    for k, v in pairs.items():
        L.append(f"| {k} vs {v['vs']} | {v['best_only']} | {v['other_only']} | {v['n_pairs']} | {v['p']} | {v['p_holm']} |")
    sr = selection["secondary_robot_closed_contact"]
    L += ["", "### SECONDARY sensitivity (declared before the main data): robot-closed contact only", "",
          "Scripted people never avoid the AMR (configs/common.yaml `avoid_robot: false`), so a person can walk "
          "into an AMR that has already stopped. The PRIMARY endpoint counts every human-AMR contact; this "
          "secondary analysis repeats the same rule counting only contacts the AMR closed. It does not replace "
          "the primary decision.", "",
          "| policy | contact (all) | contact (robot-closed) | joint@5.0 primary | joint@5.0 robot-closed |",
          "|---|---|---|---|---|"]
    for e in tab:
        L.append(f"| {e['policy']} | {fr(e['contact'])} | {fr(e['contact_rc'])} | {fr(e['joint@5.0'])} | {fr(e['joint_rc@5.0'])} |")
    L += ["", f"Secondary eligible: {sr['eligible'] or 'none'}. Secondary decision: {sr['decision']}"
          + (f"; top set {sr.get('top_set')}, Pareto {sr.get('pareto')}" if sr.get('top_set') else ""), ""]
    L += ["", "Paired vs B2 (exploratory): joint McNemar, nav McNemar, bootstrap median capped-time difference:", "",
          "| policy | pairs | joint B2-only / policy-only | p (Holm) | nav B2-only / policy-only | p | Δ capped time s (95 % CI) |",
          "|---|---:|---|---:|---|---:|---|"]
    for k, v in vs_b2.items():
        t_ = v["capped_time_policy_minus_B2"]
        L.append(f"| {k} | {v['n_pairs']} | {v['joint_B2_only']} / {v['joint_policy_only']} | {v['joint_p_holm']} | "
                 f"{v['nav_B2_only']} / {v['nav_policy_only']} | {v['nav_p']} | "
                 f"{(str(t_['median_diff']) + ' ' + str(t_['ci95'])) if t_ else '—'} |")
    if dup:
        L += ["", "## Duplicate-control diagnostic (outside the 150; same seed + same B2 config repeated)", "",
              "| seed | runs | nav | joint@5.0 | capped time s | disagree (nav / joint) |", "|---|---:|---|---|---|---|"]
        for s, v in dup.items():
            L.append(f"| {s} | {v['n']} | {v['nav']} | {v['joint@5.0']} | {v['system_time_capped_s']} | "
                     f"{v['nav_disagree']} / {v['joint_disagree']} |")
    L += ["", "Figures: `figures/S3_rates.png`, `figures/S3_time_path_comfort.png`, `figures/S3_pareto.png`.", ""]
    (out / "STAGE3_RESULTS.md").write_text("\n".join(L) + "\n")
    print(f"{len(rows)} launches, decision {selection['decision']} -> {out.relative_to(PROJECT_ROOT)}/STAGE3_RESULTS.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
