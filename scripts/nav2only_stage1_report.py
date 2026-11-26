#!/usr/bin/env python3
"""Stage 1 Nav2-only revalidation (2026-11-26 night): R_nav_upper under the
Nav2-only chain (no comfort_gate). ABBREVIATED: seeds 1-10.

    python3 scripts/nav2only_stage1_report.py --dir results/batches/261126_nav2only_stage1

Rule, FIXED BEFORE THE DATA (same thresholds as the legacy rule of
docs/FINAL_261125_PLAN.md §4, with the gate-based quantities replaced):
    comfort-mask no-progress = an AMR's class is a comfort class of
        final_metrics (a person's Comfort Space, grown by the AMR half width,
        blocks its plan / position for >= 50 % of the 12 s before the verdict).
        Without a gate this can only come from the costmap mask; the gate-only
        class PERSISTENT_COMFORT_HOLD cannot occur.
    size r PASSES iff
        comfort-mask no-progress rate <= 5 %                 (0 of 10 at n = 10)
        system success >= same-seed B1 system success - 5 %p  (>= B1 count at n = 10)
        human-AMR contact trials <= same-seed B1 contact trials
    R_nav_upper(nav2-only) = largest tested size that passes (legacy rule, not monotone);
    transition = smallest tested size that fails above it.
    Extension: if the largest tested size passes -> upper NOT identified in the tested range;
    if no size passes -> upper not justified (stop, report).
Every rate is count / n with a Wilson 95 % CI. n = 10 cannot resolve 5 % rates;
the result is a PROVISIONAL, abbreviated revalidation.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.final_metrics import COMFORT_CLASSES, analyse_trial, wilson  # noqa: E402


def size_of(cond: str):
    return None if cond == "B1" else int(cond.split("_r")[1]) / 100.0


def rate(k, n):
    lo, hi = wilson(k, n) if n else (None, None)
    return {"k": k, "n": n, "rate": (k / n) if n else None, "ci95": [lo, hi]}


def fmt(r):
    if not r["n"]:
        return "—"
    return f"{r['k']}/{r['n']} ({100 * r['rate']:.0f} %, {100 * r['ci95'][0]:.0f}–{100 * r['ci95'][1]:.0f})"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    a = ap.parse_args()
    d = PROJECT_ROOT / a.dir
    out = d / "analysis"
    (out / "trials").mkdir(parents=True, exist_ok=True)
    recs = []
    for t in sorted((d / "trials").glob("*/DONE")):
        done = json.loads(t.read_text())
        if done.get("infra_failure") or done.get("result") in (None, "INVALID", "NO_ROW"):
            recs.append({"trial": t.parent.name, "excluded": done.get("result") or "infra"})
            continue
        r = analyse_trial(t.parent)
        r["trial"] = t.parent.name
        ctrl = {}
        for j in t.parent.glob("*.json"):
            try:
                ctrl = json.loads(j.read_text()).get("control") or ctrl
            except Exception:       # noqa: BLE001
                pass
        r["control"] = {"mode": ctrl.get("mode"), "problems": ctrl.get("problems")}
        (out / "trials" / f"{t.parent.name}.json").write_text(json.dumps(r, default=str))
        recs.append(r)
    valid = [r for r in recs if "excluded" not in r]
    by = defaultdict(dict)
    for r in valid:
        by[r["condition"]][r["seed"]] = r
    b1 = by.get("B1", {})
    rows = []
    for cond in sorted(by, key=lambda c: (size_of(c) or 0)):
        rs = list(by[cond].values())
        paired = [r for r in rs if r["seed"] in b1]
        cm = sum(any(x["class"] in COMFORT_CLASSES for x in r["amrs"].values()) for r in rs)
        contact = sum(any((x.get("contact_s") or 0) > 0 for x in r["amrs"].values()) for r in rs)
        rows.append({
            "condition": cond, "size_m": size_of(cond), "n": len(rs),
            "system_success": rate(sum(r["system_success"] for r in rs), len(rs)),
            "b1_success_same_seeds": rate(sum(b1[r["seed"]]["system_success"] for r in paired), len(paired)),
            "comfort_mask_no_progress": rate(cm, len(rs)),
            "contact_trials": rate(contact, len(rs)),
            "b1_contact_same_seeds": rate(sum(any((x.get("contact_s") or 0) > 0 for x in b1[r["seed"]]["amrs"].values())
                                              for r in paired), len(paired)),
            "protective_stop_trials": rate(sum(any((x.get("protective_stop_s") or 0) > 0 for x in r["amrs"].values())
                                               for r in rs), len(rs)),
            "classes": sorted({f"{n}:{x['class']}" for r in rs for n, x in r["amrs"].items()
                               if x["class"] not in ("SUCCESS", "NORMAL_YIELD", "TEMPORARY_HOLD")}),
            "gate_free": all((r["control"]["mode"] == "nav2_only" and not r["control"]["problems"]) for r in rs),
        })
    for r in rows:
        if r["size_m"] is None:
            continue
        s, b = r["system_success"], r["b1_success_same_seeds"]
        r["pass"] = (r["comfort_mask_no_progress"]["rate"] <= 0.05 + 1e-9
                     and s["n"] > 0 and b["n"] > 0 and s["rate"] >= b["rate"] - 0.05 - 1e-9
                     and r["contact_trials"]["k"] <= r["b1_contact_same_seeds"]["k"])
    sizes = [r for r in rows if r["size_m"] is not None]
    ok = [r["size_m"] for r in sizes if r["pass"]]
    if not sizes:
        dec = {"status": "NO_DATA"}
    elif not ok:
        dec = {"status": "NOT_JUSTIFIED", "R_nav_upper": None,
               "note": "no tested size passes: Nav2-only upper bound cannot be justified in the tested range"}
    elif max(ok) == max(r["size_m"] for r in sizes):
        dec = {"status": "NOT_IDENTIFIED_IN_RANGE", "R_nav_upper": None, "largest_passing": max(ok),
               "note": "the largest tested size passes: no upper transition inside the tested range"}
    else:
        up = max(ok)
        tr = min(r["size_m"] for r in sizes if r["size_m"] > up)
        dec = {"status": "IDENTIFIED", "R_nav_upper": up, "transition": tr}
    dec["rule"] = __doc__.split("Rule, FIXED BEFORE THE DATA")[1].split("Every rate")[0].strip()
    dec["n_launches"] = len(recs)
    dec["n_excluded"] = len(recs) - len(valid)
    (out / "nav2only_stage1_upper.json").write_text(json.dumps({"decision": dec, "rows": rows}, indent=1))
    L = ["# Stage 1 Nav2-only revalidation (ABBREVIATED, provisional)", "",
         "Chain: Nav2 controller → collision monitor for every AMR; no comfort_gate (yield / evade / backoff / "
         "off-lane); buffer 0.10 m. Seeds 1–10 of the legacy Stage 1 scene. Legacy 0.90 m came from the gate "
         "chain and is not reused as a result.", "",
         f"Launches {dec['n_launches']} (excluded {dec['n_excluded']}). **Decision: {dec['status']}** "
         f"{json.dumps({k: v for k, v in dec.items() if k not in ('rule', 'status')})}", "",
         "| condition | n | system success | B1 same seeds | comfort-mask no-progress | contact trials | "
         "protective-stop trials | pass | gate-free | failure classes |",
         "|---|---:|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['condition']} | {r['n']} | {fmt(r['system_success'])} | {fmt(r['b1_success_same_seeds'])} | "
                 f"{fmt(r['comfort_mask_no_progress'])} | {fmt(r['contact_trials'])} | {fmt(r['protective_stop_trials'])} | "
                 f"{r.get('pass', '—')} | {r['gate_free']} | {', '.join(r['classes'])} |")
    L += ["", "Rule (fixed before the data):", "", "```", dec["rule"], "```", ""]
    (out / "NAV2ONLY_STAGE1_REPORT.md").write_text("\n".join(L) + "\n")
    print(json.dumps(dec, indent=1)[:600])
    return 0


if __name__ == "__main__":
    sys.exit(main())
