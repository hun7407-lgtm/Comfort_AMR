#!/usr/bin/env python3
"""Retrospective check: the Stage 1 execution model on the preliminary bay.

    python3 scripts/pilot_model_check.py --model configs/stage1a/model_exec.json \\
        --dir results/batches/261112_stage1_size_sweep

NOT a pre-registered test -- the preliminary results were seen before the
model existed. It asks only whether the model, fitted on the straight
corridor, lands in the same place as the preliminary Static transition.

For every spot AMR 1's cross mission passes (people at the north and south
belt posts, the edge/storage spots, and where the walkers end up), the
model's Static R_nav_safe is computed on the bay map with the same local
search as Stage 3 (scripts/stage3_make_configs.spot_table, kind static).
The smallest of them is the predicted Static limit for the mission; it is
printed beside the preliminary comfort-gate-hold counts by size.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.config import load_config  # noqa: E402
from comfort_core.layout import human_positions  # noqa: E402
from comfort_core.size_sweep import parse_condition  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--campaign", default="configs/campaigns/cross/campaign_h8_a3_cross_s1.yaml")
    ap.add_argument("--out", default=None, help="write the result as JSON here")
    args = ap.parse_args()
    model = json.loads(Path(args.model).read_text())
    cfg = load_config(common_path=args.campaign)
    lanes = cfg["fleet"]["lanes"]
    centre_ys = [float(lanes["north_y"]), float(lanes["south_y"])]
    humans = cfg["humans"]
    spots = {}
    for key, x, y in human_positions(cfg):
        h = humans[key]
        for px, py in [(x, y)] + [tuple(p) for p in (h.get("walk") or {}).get("route", [])[-1:]]:
            name = f"{px:+.2f},{py:+.2f}"
            if name in spots:
                continue
            cy = min(centre_ys, key=lambda c: abs(c - py))
            c = copy.deepcopy(cfg)
            c["humans"] = {k: v for k, v in c["humans"].items()
                           if not (isinstance(v, dict) and "position" in v)}
            c["humans"]["a"] = {"position": [px, py], "state": "stationary",
                                "facing_deg": h.get("facing_deg")}
            c["humans"]["state_source"] = "ground_truth"
            c["navigation"] = dict(c["navigation"], start=[px - 4.0, cy], goal=[px + 4.0, cy])
            c["_feasibility_window"] = [px - 5.0, px + 5.0]
            r = fz.r_path(c, "static", "a", "exec", res=0.02, model=model)
            spots[name] = r["e"]
    counts = defaultdict(lambda: [0, 0])
    for row in csv.DictReader((Path(args.dir) / "trials.csv").open(newline="")):
        p = parse_condition(row["scenario"])
        if p and p[0] == "static" and row.get("result") != "INVALID":
            counts[p[1]][1] += 1
            counts[p[1]][0] += row.get("deadlock_mechanism") == "comfort_gate_hold"
    # Spots on AMR 1's route: the cross mission runs east along the north
    # aisle from its start, crosses at fleet.lanes.east_x and comes back west
    # along the south aisle to its goal (configs/campaigns/cross/*.yaml).
    sx, sy = cfg["navigation"]["start"]
    gx, gy = cfg["navigation"]["goal"]
    ex = float(lanes["east_x"])
    def on_route(name):
        px, py = (float(v) for v in name.split(","))
        same = (py > 0) == (sy > 0)
        return (sx - 1.0 <= px <= ex) if same else (gx - 1.0 <= px <= ex)
    route = {k: v for k, v in spots.items() if on_route(k) and v is not None}
    limit = min(route.values())
    print("Static R_nav_safe per spot (execution model, wall clearance "
          f"{model['wall_clearance_m']:.3f} m):")
    for name, v in sorted(spots.items(), key=lambda kv: (kv[1] is None, kv[1])):
        print(f"  {name:14s} {('-' if v is None else f'{v:.2f}')} m"
              + ("   on AMR 1's route" if name in route else ""))
    print(f"predicted Static limit for the mission (smallest spot on the route): {limit:.2f} m")
    print("preliminary Static comfort gate holds by size: "
          + ", ".join(f"{s:.2f}: {g}/{n}" for s, (g, n) in sorted(counts.items())))
    if args.out:
        Path(args.out).write_text(json.dumps({
            "note": "retrospective, not pre-registered", "model": model,
            "static_r_nav_safe_by_spot": spots, "on_route": sorted(route),
            "predicted_static_limit": limit,
            "preliminary_static_gate_holds": {f"{s:.2f}": v for s, v in sorted(counts.items())}},
            indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
