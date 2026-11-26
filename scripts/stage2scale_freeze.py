#!/usr/bin/env python3
"""Apply the pilot freeze rule (declared in stage2scale_v2_nav2only.yaml before the pilot).

    python3 scripts/stage2scale_freeze.py --dir results/batches/261126_nav2only_stage2_v2_pilot [--write]

Reads <dir>/analysis/scale_trials.csv (scripts/stage2scale_report.py). Per
geometry: TECHNICALLY INFEASIBLE if B1 did not complete every launch or had a
contact; SENSITIVE if median d_cc(largest alpha >= 0.70 with a completed
passage) - median d_cc(smallest alpha) >= min_distance_response_m (completed
passages only). Per context the first feasible and sensitive geometry in the
declared order is frozen (static: narrowest width first; moving: gap 0.80,
0.65, 1.00). Writes <dir>/analysis/pilot_freeze.json (every geometry with its
reason, accepted or not); --write also fills `frozen:` in the config yaml.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import yaml  # noqa: E402

from comfort_core import stage2scale as s2  # noqa: E402


def truthy(v):
    return str(v).strip().lower() in ("true", "1")


def judge(rows, min_resp):
    b1 = [r for r in rows if r["condition"] == "B1"]
    reasons = []
    if not b1:
        reasons.append("no B1 launch")
    if b1 and not all(truthy(r["completed"]) for r in b1):
        reasons.append("B1 did not complete every launch")
    if any(truthy(r["contact"]) for r in b1):
        reasons.append("B1 contact")
    feasible = not reasons
    by_a = defaultdict(list)
    for r in rows:
        if r["alpha"] not in ("", None) and truthy(r["completed"]) and r["min_cc_m"] not in ("", None):
            by_a[float(r["alpha"])].append(float(r["min_cc_m"]))
    alphas_all = sorted({float(r["alpha"]) for r in rows if r["alpha"] not in ("", None)})
    lo = min(alphas_all) if alphas_all else None
    hi_c = [a for a in sorted(by_a, reverse=True) if a >= 0.70 - 1e-9]
    resp = None
    if lo is not None and hi_c and by_a.get(lo):
        resp = round(s2.median(by_a[hi_c[0]]) - s2.median(by_a[lo]), 4)
    sensitive = resp is not None and resp >= min_resp - 1e-9
    if resp is None:
        reasons.append("distance response unknown (no completed passage at alpha >= 0.70 or at the smallest alpha)")
    elif not sensitive:
        reasons.append(f"distance response {resp} m < {min_resp} m")
    return {"feasible": feasible, "sensitive": sensitive, "response_m": resp,
            "compared_alpha": [hi_c[0] if hi_c else None, lo], "reasons": reasons,
            "n": len(rows), "b1_completed": f"{sum(truthy(r['completed']) for r in b1)}/{len(b1)}"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--config", default="configs/stage2scale/stage2scale_v2_nav2only.yaml")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    d = PROJECT_ROOT / a.dir
    plan = json.loads((d / "plan.json").read_text())
    S = yaml.safe_load((PROJECT_ROOT / a.config).read_text())
    min_resp = float(S["criteria"]["min_distance_response_m"])
    rows = [r for r in csv.DictReader((d / "analysis" / "scale_trials.csv").open()) if not truthy(r["excluded_infra"])]
    by_geo = defaultdict(list)
    for r in rows:
        by_geo[(r["scenario"], r["env"])].append(r)
    order_gap = [float(g) for g in S["phases"]["pilot"]["moving_lane_gaps"]]
    out = {"rule": __doc__.strip(), "contexts": {}}
    frozen = {}
    for scn in plan["scenarios"]:
        geos = [(env, g) for env, g in plan["geometries"].items() if g["scenario"] == scn]
        if not geos:
            continue
        if plan["scenarios"][scn]["state"] == "moving":
            geos.sort(key=lambda x: (x[1]["width_m"], order_gap.index(float(x[1]["lane_gap_m"]))))
        else:
            geos.sort(key=lambda x: x[1]["width_m"])
        res, chosen = [], None
        for env, g in geos:
            j = judge(by_geo.get((scn, env), []), min_resp)
            j["env"] = env
            j["width_m"] = g["width_m"]
            j["lane_gap_m"] = g.get("lane_gap_m")
            if chosen is None and j["feasible"] and j["sensitive"]:
                chosen = j
                j["decision"] = "FROZEN"
            else:
                j["decision"] = "not chosen" if (j["feasible"] and j["sensitive"]) else "REJECTED"
            res.append(j)
        out["contexts"][scn] = {"geometries": res, "frozen": chosen["env"] if chosen else None,
                                "status": "FROZEN" if chosen else s2.UNIDENTIFIABLE}
        if chosen:
            frozen[scn] = {"width_m": chosen["width_m"]}
            if chosen["lane_gap_m"] is not None:
                frozen[scn]["lane_gap_m"] = chosen["lane_gap_m"]
    (d / "analysis" / "pilot_freeze.json").write_text(json.dumps(out, indent=1))
    for scn, c in out["contexts"].items():
        print(f"{scn:5s} {c['status']:24s} {c['frozen']}")
        for g in c["geometries"]:
            print(f"      {g['env']:22s} {g['decision']:10s} resp {g['response_m']} B1 {g['b1_completed']} {g['reasons']}")
    if a.write:
        p = PROJECT_ROOT / a.config
        txt = p.read_text()
        assert "\nfrozen: {}" in txt, "frozen block already filled; edit by hand with a decision-log entry"
        txt = txt.replace("\nfrozen: {}", "\nfrozen:  # from " + a.dir + "/analysis/pilot_freeze.json\n"
                          + "".join(f"  {k}: {json.dumps(v)}\n" for k, v in frozen.items()).rstrip("\n"))
        p.write_text(txt)
        print("frozen block written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
