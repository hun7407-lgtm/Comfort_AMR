#!/usr/bin/env python3
"""Stage 2 (stage23_900) results: per-trial / per-encounter CSV, cell table, figures.

    python3 scripts/final_stage23_900_report2.py --dir results/batches/261127_stage23_900_s2main

Scores every launch with scripts/stage2scale_report.py (predicted comfort from the ACTUAL
closest centre distance + bearing + state, comfort_core/comfort_reference.py), then writes
<dir>/analysis/: per_trial.csv, per_encounter.csv, STAGE2_RESULTS.csv, STAGE2_RESULTS.md,
figures/S2_<context>.png, figures/S2_overview.png. Identification per context comes from
comfort_core.stage2scale.select_context over the tested alphas 0.50-0.90 (IDENTIFIED_BRACKET
only when the next lower tested alpha fails on comfort while navigation succeeds;
FLOOR_LIMITED when 0.50 passes; UNIDENTIFIABLE_GEOMETRY when the 0.90 -> 0.50 distance
response is < 0.10 m). Rates: k/n with Wilson 95 % CI. Infra-excluded launches are counted
and listed, never silently dropped.
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

from comfort_core import stage2scale as s2  # noqa: E402


def T(v):
    return str(v).strip().lower() in ("true", "1")


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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--skip-score", action="store_true")
    a = ap.parse_args()
    d = PROJECT_ROOT / a.dir
    if not a.skip_score:
        r = subprocess.run([sys.executable, "scripts/stage2scale_report.py", "--dir", a.dir], cwd=PROJECT_ROOT)
        if r.returncode != 0:
            return r.returncode
    plan = json.loads((d / "plan.json").read_text())
    out = d / "analysis"
    rows = list(csv.DictReader((out / "scale_trials.csv").open()))
    sel = json.loads((out / "selected_state_scales.json").read_text())
    crit = plan["criteria"]
    targets = [crit["primary_target"]] + list(crit["sensitivity_targets"])
    # per_trial.csv = scale_trials.csv (every launch, incl. infra-excluded rows)
    (out / "per_trial.csv").write_text((out / "scale_trials.csv").read_text())
    enc_keys = ["trial_id", "scenario", "state", "geometry", "env", "condition", "alpha", "repeat", "excluded_infra",
                "state_declared_at_closest", "state_observed_at_closest", "state_ok", "side_ok", "crossed", "sector",
                "intended_sector", "rel_deg", "min_cc_m", "min_footprint_edge_m", "predicted_comfort_at_closest",
                "predicted_comfort_min", "inside_tested_range", "amr_speed_at_closest_mps",
                "human_speed_at_closest_mps", "relative_speed_at_closest_mps", "min_ttc_s", "out_of_reference_domain"]
    with (out / "per_encounter.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=enc_keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    valid = [r for r in rows if not T(r["excluded_infra"])]
    excl = [r for r in rows if T(r["excluded_infra"])]
    cells = defaultdict(list)
    for r in valid:
        cells[(r["scenario"], r["geometry"], float(r["alpha"]))].append(r)
    tab = []
    for (scn, gid, al), rs in sorted(cells.items()):
        n = len(rs)
        comp = [r for r in rs if T(r["completed"])]
        e = {"context": scn, "state": rs[0]["state"], "geometry": gid, "alpha": al, "n": n,
             "nav": rate(sum(T(r["completed"]) for r in rs), n),
             "safe_nav": rate(sum(T(r["completed"]) and not T(r["contact"]) for r in rs), n),
             "contact": rate(sum(T(r["contact"]) for r in rs), n),
             "contact_robot_closed": rate(sum(T(r.get("contact_robot_closed")) for r in rs), n),
             "deadlock_terminal_stop": rate(sum(T(r["terminal_protective_stop"]) for r in rs), n),
             "comfort_deadlock": rate(sum(T(r["comfort_deadlock"]) for r in rs), n),
             "collision_monitor_stop": rate(sum(T(r.get("collision_monitor_stop")) for r in rs), n),
             "median_d_cc_m": pctl([F(r["min_cc_m"]) for r in comp], 0.5),
             "p10_d_cc_m": pctl([F(r["min_cc_m"]) for r in comp], 0.1),
             "median_edge_m": pctl([F(r["min_footprint_edge_m"]) for r in comp], 0.5),
             "median_comfort": pctl([F(r["predicted_comfort_at_closest"]) for r in comp], 0.5),
             "p10_comfort": pctl([F(r["predicted_comfort_at_closest"]) for r in comp], 0.1),
             "comfort_missing": sum(F(r["predicted_comfort_at_closest"]) is None for r in comp),
             "median_travel_s": pctl([F(r["travel_time_s"]) for r in comp], 0.5),
             "median_path_m": pctl([F(r.get("path_length_m")) for r in comp], 0.5),
             "median_replans": pctl([F(r.get("replans")) for r in comp], 0.5),
             "median_idle_s": pctl([F(r.get("idle_s")) for r in rs], 0.5),
             "median_stop_s": pctl([F(r["protective_stop_s"]) for r in rs], 0.5)}
        for tg in targets:
            e[f"joint@{tg}"] = rate(sum(int(F(r[f"J@{tg}"]) or 0) for r in rs), n)
        tab.append(e)
    flat = []
    for e in tab:
        row = {}
        for k, v in e.items():
            if isinstance(v, dict):
                row[f"{k}_k"], row[f"{k}_n"], row[f"{k}_rate"] = v["k"], v["n"], v["rate"]
                row[f"{k}_ci_lo"], row[f"{k}_ci_hi"] = v["ci"]
            else:
                row[k] = v
        flat.append(row)
    if flat:
        with (out / "STAGE2_RESULTS.csv").open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(flat[0]))
            w.writeheader()
            w.writerows(flat)
    # figures
    figs = out / "figures"
    figs.mkdir(exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        ctxs = sorted({e["context"] for e in tab})
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
        for i, c in enumerate(ctxs):
            col = f"C{i}"    # one fixed colour per context in all three panels
            es = sorted([e for e in tab if e["context"] == c], key=lambda e: e["alpha"])
            x = [e["alpha"] for e in es]
            j = 0.006 * (i - (len(ctxs) - 1) / 2)    # small vertical offset so overlapping rates stay visible
            ax[0].plot(x, [e["median_d_cc_m"] for e in es], "o-", color=col, label=c)
            ax[1].plot(x, [e["median_comfort"] for e in es], "o-", color=col, label=c)
            ax[2].plot(x, [e["joint@5.0"]["rate"] + j for e in es], "o-", color=col, label=c)
            ax[2].plot(x, [e["nav"]["rate"] + j for e in es], "x:", color=col, alpha=0.6)
        ax[1].axhline(crit["primary_target"], color="red", lw=1)
        for i, t in enumerate(("median actual closest centre distance (m)",
                               "median predicted comfort (literature curve)", "joint @5.0 (solid), nav (dotted); offset ±0.01 for visibility")):
            ax[i].set_xlabel("alpha (uniform scale of R_ref 0.95 m template)")
            ax[i].set_title(t, fontsize=10)
            ax[i].grid(alpha=0.3)
        ax[0].legend(fontsize=8)
        fig.suptitle(f"Stage 2 stage23_900 ({plan['phase']}): n per cell = "
                     f"{min((e['n'] for e in tab), default=0)}–{max((e['n'] for e in tab), default=0)}")
        fig.tight_layout()
        fig.savefig(figs / "S2_overview.png", dpi=130)
        plt.close(fig)
        for c in ctxs:
            es = sorted([e for e in tab if e["context"] == c], key=lambda e: e["alpha"])
            rs = [r for r in valid if r["scenario"] == c and T(r["completed"])]
            fig, ax = plt.subplots(1, 2, figsize=(10, 4))
            ax[0].scatter([float(r["alpha"]) for r in rs], [F(r["min_cc_m"]) for r in rs], s=10, alpha=0.5)
            ax[0].plot([e["alpha"] for e in es], [e["median_d_cc_m"] for e in es], "k-")
            ax[1].scatter([float(r["alpha"]) for r in rs], [F(r["predicted_comfort_at_closest"]) for r in rs],
                          s=10, alpha=0.5)
            ax[1].axhline(crit["primary_target"], color="red", lw=1)
            ax[0].set_ylabel("actual closest centre distance (m)")
            ax[1].set_ylabel("predicted comfort at closest point")
            for x_ in ax:
                x_.set_xlabel("alpha")
                x_.grid(alpha=0.3)
            fig.suptitle(f"{c}: every completed launch (dots), median (line)")
            fig.tight_layout()
            fig.savefig(figs / f"S2_{c}.png", dpi=120)
            plt.close(fig)
    except Exception as ex:        # noqa: BLE001
        print("figures skipped:", ex)
    # markdown
    L = [f"# STAGE 2 RESULTS — stage23_900 {plan['phase']}", "",
         "> Predicted comfort = published comfort–distance curves (Neggers 2022) evaluated at the ACTUAL closest "
         "centre distance and bearing; not a measurement of anyone's comfort, not an experimentally validated "
         "comfort distance. station_engaged uses the standing-person curve (transfer). Curves: robot ~0.35 m/s.", "",
         f"Launches {len(rows)}; valid {len(valid)}; infra-excluded {len(excl)}"
         + (f" ({', '.join(r['trial_id'] for r in excl)})" if excl else "") + ".",
         f"Commit at plan {plan['commit_at_plan']}; manifest sha {plan.get('checksums', {}).get('manifest_sha256_16')}.",
         "", "## Cells", "",
         "| context | geom | alpha | n | nav | safe nav | joint@5.0 (95 % CI) | joint@4.5 | joint@5.5 | contact (robot-closed) | "
         "term. stop | CM stop | med d_cc | P10 d_cc | med C | P10 C | med time s | med path m | med replans |",
         "|---|---|---:|---:|---|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for e in tab:
        L.append(f"| {e['context']} | {e['geometry']} | {e['alpha']:.2f} | {e['n']} | {fr(e['nav'])} | {fr(e['safe_nav'])} | "
                 f"{fr(e['joint@5.0'])} | {e['joint@4.5']['k']}/{e['n']} | {e['joint@5.5']['k']}/{e['n']} | "
                 f"{e['contact']['k']} ({e['contact_robot_closed']['k']}) | {e['deadlock_terminal_stop']['k']} | {e['collision_monitor_stop']['k']} | "
                 f"{e['median_d_cc_m']} | {e['p10_d_cc_m']} | {e['median_comfort']} | {e['p10_comfort']} | "
                 f"{e['median_travel_s']} | {e['median_path_m']} | {e['median_replans']} |")
    L += ["", "## Identification per context (tested alpha 0.50–0.90)", "",
          "| context | geom | status @5.0 | alpha* @5.0 | @4.5 | @5.5 | d_cc response 0.90→0.50 m | pilot status |",
          "|---|---|---|---|---|---|---|---|"]
    for c, g in sel["contexts"].items():
        if "status" in g:
            L.append(f"| {c} | — | {g['status']} | — | — | — | — | — |")
            continue
        for gid, by in g.items():
            p = by["5.0"]
            L.append(f"| {c} | {gid} | {p['status']} | {p.get('alpha_star')} | {by['4.5']['status']} "
                     f"({by['4.5'].get('alpha_star')}) | {by['5.5']['status']} ({by['5.5'].get('alpha_star')}) | "
                     f"{p.get('distance_response_m')} | {plan['scenarios'][c].get('identification_at_pilot', '—')} |")
    L += ["", "Rules: IDENTIFIED_BRACKET needs the next lower tested alpha to fail on comfort while navigation succeeds; "
          "FLOOR_LIMITED_BOUND_NOT_OBSERVED = 0.50 passes (not a comfort lower bound); UNIDENTIFIABLE_GEOMETRY = "
          "distance response < 0.10 m. 'Operationally passing' = nav and joint >= 0.80 in the TESTED range only.", "",
          "Figures: `figures/S2_overview.png`, `figures/S2_<context>.png`. Raw: `per_trial.csv`, `per_encounter.csv`, "
          "`trials/*.json`.", ""]
    (out / "STAGE2_RESULTS.md").write_text("\n".join(L) + "\n")
    print(f"{len(rows)} launches, {len(tab)} cells -> {out.relative_to(PROJECT_ROOT)}/STAGE2_RESULTS.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
