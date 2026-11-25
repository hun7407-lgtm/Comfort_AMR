#!/usr/bin/env python3
"""Stage 2 (final, rev. 2026-11-25): Simulation-derived Provisional Lower Bound.

    python3 scripts/final_stage2_proxy.py

Plain Python 3. Reads the Stage 1 per-trial analysis
(results/batches/261125_final_stage1/analysis/trials/*.json, made by
scripts/final_stage1_report.py) and stage1_upper_bound.json; writes
results/batches/261125_final_stage2_proxy/{stage2_provisional_lower_bound.json, STAGE2_PROXY.md}.

source = SIMULATION_DERIVED_PROVISIONAL. NOT a human study result and NOT a
human comfort distance: it is how close the simulated navigation stack came
to people in clean, successful Stage 1 interactions.

Clean interaction: that AMR's mission succeeded (SUCCESS / NORMAL_YIELD /
TEMPORARY_HOLD), it had no contact, no protective stop, no Nav2 abort; the
interaction has a state and a direction.

Per (state, direction) and per source (B1; B2 at each size; B2 pooled over
sizes <= R_nav_upper): N, min, Q10, Q25, median, Q75 of the physical
clearance (person centre -> AMR footprint), penetration (median / max),
intrusion seconds, share of interactions with the AMR standing, AMR speed at
the closest point.

R_lower_proxy(H, d) (docs/FINAL_261125_PLAN.md §5, fixed before the data):
max(Q10 of B1, Q10 of B2 pooled <= R_nav_upper), each only with N >= 20;
one source with N >= 20 -> that one; none -> INSUFFICIENT SAMPLE.
"""

from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.final_metrics import DIRECTIONS, STATES, SUCCESS_CLASSES  # noqa: E402

S1 = PROJECT_ROOT / "results" / "batches" / "261125_final_stage1" / "analysis"
OUT = PROJECT_ROOT / "results" / "batches" / "261125_final_stage2_proxy"
MIN_N = 20


def q(xs, p):
    return round(float(np.quantile(xs, p)), 3) if xs else None


def stats(items):
    c = [e["min_clearance"] for e in items]
    pen = [e["penetration"] for e in items]
    return {"N": len(items), "min": round(min(c), 3) if c else None, "q10": q(c, 0.10), "q25": q(c, 0.25),
            "median": q(c, 0.5), "q75": q(c, 0.75),
            "penetration_median": q(pen, 0.5), "penetration_max": round(max(pen), 3) if pen else None,
            "intrusion_s_median": q([e["intrusion_s"] for e in items], 0.5),
            "standing_share": round(sum(1 for e in items if e["hold_s"] > 0) / len(items), 3) if items else None,
            "amr_speed_at_min_median": q([e["amr_speed"] for e in items if e["amr_speed"] is not None], 0.5)}


def main() -> int:
    ub = json.loads((S1 / "stage1_upper_bound.json").read_text())
    upper = ub["decision"]["R_nav_upper_m"]
    groups = defaultdict(lambda: defaultdict(list))
    n_trials = defaultdict(int)
    for f in sorted((S1 / "trials").glob("*.json")):
        r = json.loads(f.read_text())
        if r.get("infra_failure"):
            continue
        cond = r["condition"]
        size = None if cond == "B1" else int(cond.split("_r")[1]) / 100.0
        n_trials[cond] += 1
        for e in r["interactions"]:
            a = r["amrs"].get(e["amr"], {})
            if (a.get("class") not in SUCCESS_CLASSES or (a.get("contact_s") or 0) > 0
                    or (a.get("protective_stop_s") or 0) > 0 or a.get("nav2_abort")):
                continue
            if e["state"] not in STATES or e["direction"] not in DIRECTIONS:
                continue
            key = (e["state"], e["direction"])
            groups[cond][key].append(e)
            if size is not None and upper is not None and size <= upper + 1e-9:
                groups["B2_pooled_le_upper"][key].append(e)
    tables = {src: {f"{s}|{d}": stats(groups[src][(s, d)]) for s in STATES for d in DIRECTIONS}
              for src in sorted(groups)}
    proxy = {}
    for s in STATES:
        for d in DIRECTIONS:
            k = f"{s}|{d}"
            cands = {src: tables[src][k] for src in ("B1", "B2_pooled_le_upper") if src in tables}
            ok = {src: v["q10"] for src, v in cands.items() if v["N"] >= MIN_N}
            proxy[k] = {"state": s, "direction": d,
                        "R_lower_proxy_m": round(max(ok.values()), 3) if ok else None,
                        "status": "OK" if ok else "INSUFFICIENT SAMPLE",
                        "from": max(ok, key=ok.get) if ok else None,
                        "N": {src: v["N"] for src, v in cands.items()},
                        "q10": {src: v["q10"] for src, v in cands.items()}}
    OUT.mkdir(parents=True, exist_ok=True)
    doc = {"source": "SIMULATION_DERIVED_PROVISIONAL",
           "note": "Not a human study result and not a human comfort distance. Physical clearance = person centre "
                   "-> AMR footprint, from clean successful Stage 1 interactions of the simulated navigation stack.",
           "rule": "R_lower_proxy(H,d) = max(Q10 B1, Q10 B2 pooled <= R_nav_upper), each only with N >= 20",
           "R_nav_upper_m": upper, "stage1_partial": ub.get("partial"), "min_n": MIN_N,
           "trials_per_condition": dict(n_trials), "R_lower_proxy": proxy, "tables": tables,
           "units": "Comfort parameter and clearance are the same quantity (person centre -> nearest AMR footprint "
                    "point); comfort_space.robot_grow adds the AMR half-width only inside the planner mask."}
    (OUT / "stage2_provisional_lower_bound.json").write_text(json.dumps(doc, indent=1))
    L = ["# Stage 2 proxy — Simulation-derived Provisional Lower Bound", "",
         "> source = **SIMULATION_DERIVED_PROVISIONAL**. Not a human study result, not a human comfort distance. "
         "It is how close the simulated navigation stack came to people in clean successful Stage 1 interactions.",
         "", f"R_nav_upper (Stage 1) = {upper} m. Rule: {doc['rule']}.", "",
         "## R_lower_proxy(H, d)", "",
         "| State | Direction | N (B1 / B2 pooled) | Q10 B1 | Q10 B2 pooled | R_lower_proxy | status |",
         "|---|---|---:|---:|---:|---:|---|"]
    for k, v in proxy.items():
        L.append(f"| {v['state']} | {v['direction']} | {v['N'].get('B1', 0)} / {v['N'].get('B2_pooled_le_upper', 0)}"
                 f" | {v['q10'].get('B1')} | {v['q10'].get('B2_pooled_le_upper')} | {v['R_lower_proxy_m']} | "
                 f"{v['status']} |")
    for src, tab in tables.items():
        L += ["", f"## Source {src}", "",
              "| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) "
              "| standing share | AMR speed at min |", "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
        for k, v in tab.items():
            s, d = k.split("|")
            L.append(f"| {s} | {d} | {v['N']}{' (INSUFFICIENT)' if v['N'] < MIN_N else ''} | {v['q10']} | "
                     f"{v['median']} | {v['penetration_median']} / {v['penetration_max']} | {v['intrusion_s_median']} "
                     f"| {v['standing_share']} | {v['amr_speed_at_min_median']} |")
    (OUT / "STAGE2_PROXY.md").write_text("\n".join(L) + "\n")
    print(json.dumps(proxy, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
