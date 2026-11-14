#!/usr/bin/env python3
"""Build the Stage 2 conditions (context x passing distance) and participant orders.

    python3 scripts/stage2_make_conditions.py --config configs/stage2/stage2_real.yaml
    python3 scripts/stage2_make_conditions.py --config ... --orders 8
    python3 scripts/stage2_make_conditions.py --config ... --levels 0.5 0.75 1.0 1.25 1.5
            (pipeline check only, NOT recorded as the study's levels)

Reads stage2.contexts and stage2.distance_levels_m. For every (context, d)
the B3-O target size that gives effective extent d in that context is
target = d / ratio(context) (comfort_core/contexts.py: c1 rear 0.50,
c2 rear 0.875, c3 front 0.667, c4 side 0.75 of the target size), and one
condition overlay per distinct target is written to configs/stage2/conditions/
with comfort_core.size_sweep, exactly as Stage 1 writes its own.

conditions.csv lists context, d, target size and condition file.
--orders N writes orders.csv: P01..PNN, context blocks in a Williams order,
distance levels in a Williams order inside each block
(comfort_core.stage2.participant_orders).

Refuses while stage2.distance_levels_m is null: the levels wait for the
real AMR's protective field and the safety briefing.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.config import load_with_base                      # noqa: E402
from comfort_core.contexts import CONTEXTS, target_for_extent       # noqa: E402
from comfort_core.size_sweep import condition_name, write_condition  # noqa: E402
from comfort_core.stage2 import (distance_levels, participant_orders,  # noqa: E402
                                 study_contexts)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/stage2/stage2_real.yaml")
    ap.add_argument("--out", default="configs/stage2/conditions")
    ap.add_argument("--orders", type=int, default=0)
    ap.add_argument("--levels", type=float, nargs="+", default=None,
                    help="override stage2.distance_levels_m (pipeline check only)")
    args = ap.parse_args()
    cfg = load_with_base(args.config)
    if args.levels:
        print("[WARN] distance levels overridden on the command line; pipeline check, "
              "not the study's levels", file=sys.stderr)
        cfg.setdefault("stage2", {})["distance_levels_m"] = args.levels
    try:
        levels = distance_levels(cfg)
        contexts = study_contexts(cfg)
    except ValueError as exc:
        print(f"[stage2_make_conditions] {exc}", file=sys.stderr)
        return 2
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for c in contexts:
        for d in levels:
            target = round(target_for_extent(cfg, c, d), 2)
            write_condition(out, "b3o", target, stage="Stage 2")
            rows.append({"context": CONTEXTS[c]["name"], "passing_distance_m": d,
                         "target_size_m": target, "condition": condition_name("b3o", target)})
    with (out / "conditions.csv").open("w", newline="", encoding="utf-8") as h:
        w = csv.DictWriter(h, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"{len(contexts)} contexts x {len(levels)} distances -> {out}/conditions.csv")
    for r in rows:
        print(f"  {r['context']:22s} d {r['passing_distance_m']:.2f} m -> target {r['target_size_m']:.2f} "
              f"({r['condition']})")
    if args.orders:
        lookup = {(r["context"], r["passing_distance_m"]): r for r in rows}
        seq = participant_orders(args.orders, contexts, levels)
        with (out / "orders.csv").open("w", newline="", encoding="utf-8") as h:
            w = csv.writer(h)
            w.writerow(["participant_id", "block", "trial_order", "context",
                        "passing_distance_m", "target_size_m", "condition"])
            for s in seq:
                r = lookup[(CONTEXTS[s["context"]]["name"], s["passing_distance_m"])]
                w.writerow([s["participant_id"], s["block"], s["trial_order"], r["context"],
                            f"{s['passing_distance_m']:.2f}", f"{r['target_size_m']:.2f}",
                            r["condition"]])
        print(f"  wrote {out / 'orders.csv'} ({args.orders} participants x "
              f"{len(contexts) * len(levels)} trials, Williams-balanced)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
