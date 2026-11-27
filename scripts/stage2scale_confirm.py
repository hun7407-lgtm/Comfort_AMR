#!/usr/bin/env python3
"""Confirmation verdict for the Stage 2 uniform-scale sweep (Nav2-only v2).

    python3 scripts/stage2scale_confirm.py \
        --coarse results/batches/261126_nav2only_stage2_v2_coarse \
        --confirm results/batches/261126_nav2only_stage2_v2_confirm

Why a separate step: the confirmation batch only re-runs B1, B2, alpha* and the
next smaller tested alpha, so it cannot re-derive the identification status
(the distance response needs alpha 1.00 and the smallest alpha). The status
comes from the coarse sweep; the confirmation only checks whether the
decisive cells reproduce on new launches.

Rule (the coarse selection criteria applied to the confirmation cells):
    alpha* REPRODUCED   iff joint rate >= joint_min and nav rate >= nav_min at alpha*
    bracket REPRODUCED  iff (IDENTIFIED_BRACKET only) the next smaller alpha fails
                        (joint rate < joint_min) and that failure is a comfort failure
                        (nav rate >= nav_min), so it is not a navigation-only bound
    CONFIRMED  = both reproduced (bracket contexts) / alpha* reproduced (floor-limited:
                 still NOT a comfort lower bound; the status stays FLOOR_LIMITED)
    NOT_CONFIRMED otherwise. Sensitivity: joint >= 0.90 and C* 4.5 / 5.5 reported.
Contexts frozen or recorded as unidentifiable keep that status.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def cell(summary, scn, cond):
    for c in summary["cells"]:
        if c["scenario"] == scn and c["condition"] == cond:
            return c
    return None


def atag(a):
    return f"S3_a{a:.2f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--coarse", required=True)
    ap.add_argument("--confirm", required=True)
    a = ap.parse_args()
    co = json.loads((PROJECT_ROOT / a.coarse / "analysis" / "selected_state_scales.json").read_text())
    cs = json.loads((PROJECT_ROOT / a.confirm / "analysis" / "scale_summary.json").read_text())
    plan = json.loads((PROJECT_ROOT / a.confirm / "plan.json").read_text())
    crit = co["criteria"]
    jmin, nmin = crit["joint_min"], crit["nav_min"]
    out = {"rule": __doc__.split("Rule (")[1].split("Contexts frozen")[0].strip(), "contexts": {}}
    L = ["# Stage 2 confirmation verdict (Nav2-only v2, ABBREVIATED)", "",
         "Status from the coarse sweep; confirmation = new launches of the decisive cells. Predicted comfort "
         "is a published-curve prediction at the actual closest distance, not measured comfort.", "",
         "| context | coarse status @5.0 | alpha* | target | alpha* joint (95 % CI) | alpha* nav | next smaller | "
         "its joint | its nav | median d_cc alpha* / next (m) | median C alpha* / next | verdict |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for scn, ctx in co["contexts"].items():
        if "status" in ctx:
            out["contexts"][scn] = {"status": ctx["status"], "verdict": "NOT_RUN"}
            L.append(f"| {scn} | {ctx['status']} | — | — | — | — | — | — | — | — | — | not run |")
            continue
        if plan["scenarios"].get(scn, {}).get("status") != "RUN":
            for g, by_t in ctx.items():
                st = by_t["5.0"]["status"]
                out["contexts"][scn] = {"geometry": g, "status": st, "verdict": "NOT_RUN"}
                L.append(f"| {scn} | {st} | — | — | — | — | — | — | — | — | — | not run |")
            continue
        for g, by_t in ctx.items():
            res = {"geometry": g, "by_target": {}}
            for T in ("5.0", "4.5", "5.5"):
                sel = by_t[T]
                ast = sel.get("alpha_star")
                tested = sel["tested_alpha"]
                smaller = [x for x in tested if ast is not None and x < ast - 1e-9]
                nxt = max(smaller) if smaller else None
                ca = cell(cs, scn, atag(ast)) if ast is not None else None
                cn = cell(cs, scn, atag(nxt)) if nxt is not None else None
                r = {"coarse_status": sel["status"], "alpha_star": ast, "next_smaller": nxt}
                if ca is None:
                    r["verdict"] = "NOT_IN_CONFIRM_BATCH"
                else:
                    bt = ca["by_target"][T]
                    r["alpha_star_cell"] = {"n": bt["n"], "joint_k": bt["joint_k"], "joint_ci": bt["joint_ci"],
                                            "nav_k": bt["nav_k"], "median_cc": ca["median_min_cc_completed_m"],
                                            "median_C": ca["median_comfort_completed"], "contact_k": ca["contact_k"]}
                    ok_a = bt["joint_rate"] >= jmin - 1e-9 and bt["nav_rate"] >= nmin - 1e-9
                    ok_a90 = bt["joint_rate"] >= crit["joint_min_sensitivity"] - 1e-9
                    if sel["status"] == "IDENTIFIED_BRACKET":
                        if cn is None:
                            r["verdict"] = "NEXT_ALPHA_NOT_IN_CONFIRM_BATCH" if ok_a else "NOT_CONFIRMED"
                        else:
                            bn = cn["by_target"][T]
                            r["next_cell"] = {"n": bn["n"], "joint_k": bn["joint_k"], "joint_ci": bn["joint_ci"],
                                              "nav_k": bn["nav_k"], "median_cc": cn["median_min_cc_completed_m"],
                                              "median_C": cn["median_comfort_completed"]}
                            ok_b = bn["joint_rate"] < jmin - 1e-9 and bn["nav_rate"] >= nmin - 1e-9
                            r["verdict"] = "CONFIRMED" if (ok_a and ok_b) else "NOT_CONFIRMED"
                    elif sel["status"] == "FLOOR_LIMITED_BOUND_NOT_OBSERVED":
                        r["verdict"] = "OPERATIONAL_ALPHA_REPRODUCED" if ok_a else "NOT_CONFIRMED"
                    else:
                        r["verdict"] = "NOT_CONFIRMED" if not ok_a else "REPRODUCED_" + sel["status"]
                    r["joint_0.90_sensitivity"] = ok_a90
                res["by_target"][T] = r
                if T == "5.0" or (ca is not None):
                    A = r.get("alpha_star_cell") or {}
                    N = r.get("next_cell") or {}
                    f = lambda c: f"{c['joint_k']}/{c['n']} ({c['joint_ci'][0]:.2f}–{c['joint_ci'][1]:.2f})" if c else "—"
                    L.append(f"| {scn} | {sel['status']} | {ast} | {T} | {f(A)} | "
                             f"{(str(A['nav_k']) + '/' + str(A['n'])) if A else '—'} | {nxt} | {f(N)} | "
                             f"{(str(N['nav_k']) + '/' + str(N['n'])) if N else '—'} | "
                             f"{A.get('median_cc')} / {N.get('median_cc')} | {A.get('median_C')} / {N.get('median_C')} | "
                             f"{r['verdict']} |")
            out["contexts"][scn] = res
    L += ["", "Rule:", "", "```", out["rule"], "```", ""]
    d = PROJECT_ROOT / a.confirm / "analysis"
    (d / "confirm_verdict.json").write_text(json.dumps(out, indent=1))
    (d / "CONFIRM_VERDICT.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[:len(L) - 6]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
