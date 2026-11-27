#!/usr/bin/env python3
"""Stage 2 pilot freeze (stage23_900): apply the policy-neutral rule fixed in
configs/final_nav2only_stage23_900/stage23_900.yaml (`stage2.freeze_rule`).

    python3 scripts/final_stage23_900_freeze.py --dir results/batches/261127_stage23_900_s2pilot [--write]

Reads <dir>/analysis/scale_trials.csv (scripts/stage2scale_report.py). Writes
<dir>/analysis/pilot_freeze.json with every candidate and its reasons; --write fills
`frozen:` in the yaml (once; a filled block is never overwritten).
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

CFG = "configs/final_nav2only_stage23_900/stage23_900.yaml"


def T(v):
    return str(v).strip().lower() in ("true", "1")


def judge(rows, min_resp):
    n = len(rows)
    comp = [r for r in rows if T(r["completed"])]
    contact = sum(T(r["contact"]) for r in rows)
    bad_enc = [r["trial_id"] for r in comp if not (T(r["side_ok"]) and r["sector"] == r["intended_sector"])]
    reasons = []
    if not n:
        reasons.append("no launch")
    if contact:
        reasons.append(f"contact in {contact}/{n}")
    if n and len(comp) / n < 0.5:
        reasons.append(f"pooled completion {len(comp)}/{n} < 0.50")
    if bad_enc:
        reasons.append(f"wrong side / sector in {bad_enc}")
    valid = not reasons
    by_a = defaultdict(list)
    for r in comp:
        if r["min_cc_m"] not in ("", None):
            by_a[float(r["alpha"])].append(float(r["min_cc_m"]))
    hi = 0.90 if by_a.get(0.90) else next((x for x in sorted(by_a, reverse=True) if x >= 0.70 - 1e-9), None)
    resp = None
    if hi is not None and by_a.get(0.50):
        resp = round(s2.median(by_a[hi]) - s2.median(by_a[0.50]), 4)
    sensitive = resp is not None and resp >= min_resp - 1e-9
    if resp is None:
        reasons.append("distance response unknown (no completed launch at 0.50 or at >= 0.70)")
    elif not sensitive:
        reasons.append(f"distance response {resp} m < {min_resp} m")
    return {"valid": valid, "sensitive": sensitive, "response_m": resp, "compared_alpha": [hi, 0.50],
            "n": n, "completed": len(comp), "contact": contact,
            "median_d_cc_by_alpha": {str(k): s2.median(v) for k, v in sorted(by_a.items())}, "reasons": reasons}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    d = PROJECT_ROOT / a.dir
    S = yaml.safe_load((PROJECT_ROOT / CFG).read_text())
    plan = json.loads((d / "plan.json").read_text())
    min_resp = float(S["criteria"]["min_distance_response_m"])
    rows = [r for r in csv.DictReader((d / "analysis" / "scale_trials.csv").open()) if not T(r["excluded_infra"])]
    by = defaultdict(list)
    for r in rows:
        by[(r["scenario"], r["geometry"])].append(r)
    P = S["stage2"]["pilot"]
    out = {"rule": S["stage2"]["freeze_rule"], "contexts": {}}
    frozen = {}
    for ctx, c in S["stage2"]["contexts"].items():
        if c["state"] == "moving":
            cands = [(f"w{float(P['moving_width']):.2f}_g{float(g):.2f}",
                      {"width_m": float(P["moving_width"]), "lane_gap_m": float(g)}) for g in P["moving_lane_gaps"]]
        else:
            cands = [(f"w{float(w):.2f}", {"width_m": float(w)}) for w in P["static_widths"]]
        res, chosen, first_valid = [], None, None
        for gid, geo in cands:
            j = judge(by.get((ctx, gid), []), min_resp)
            j.update({"geometry": gid, **geo})
            if j["valid"] and first_valid is None:
                first_valid = j
            if chosen is None and j["valid"] and j["sensitive"]:
                chosen = j
                j["decision"] = "FROZEN"
            else:
                j["decision"] = "not chosen" if (j["valid"] and j["sensitive"]) else "REJECTED"
            res.append(j)
        if chosen:
            st, pick = "FROZEN", chosen
        else:
            st, pick = s2.UNIDENTIFIABLE, (first_valid or res[0])
            pick["decision"] = pick["decision"] + " (run in MAIN, labelled UNIDENTIFIABLE)"
        out["contexts"][ctx] = {"status": st, "geometry": pick["geometry"], "candidates": res}
        frozen[ctx] = {k: pick[k] for k in ("width_m", "lane_gap_m") if k in pick}
        if st != "FROZEN":
            frozen[ctx]["status"] = st
    (d / "analysis" / "pilot_freeze.json").write_text(json.dumps(out, indent=1))
    for ctx, c in out["contexts"].items():
        print(f"{ctx:5s} {c['status']:24s} {c['geometry']}")
        for g in c["candidates"]:
            print(f"      {g['geometry']:12s} {g['decision']:12s} resp {g['response_m']} comp {g['completed']}/{g['n']} "
                  f"contact {g['contact']} {g['median_d_cc_by_alpha']} {g['reasons']}")
    if a.write:
        p = PROJECT_ROOT / CFG
        txt = p.read_text()
        key = "\nfrozen: {}"
        assert key in txt, "frozen block already filled; edit by hand with a decision-log entry"
        block = ("\nfrozen:  # from " + a.dir + "/analysis/pilot_freeze.json\n"
                 + "".join(f"  {k}: {json.dumps(v)}\n" for k, v in frozen.items()).rstrip("\n"))
        p.write_text(txt.replace(key, block, 1))
        print("frozen block written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
