#!/usr/bin/env python3
"""Build the Stage 2 size ladder from R_nav_max, and the participant orders.

    python3 scripts/stage2_make_conditions.py --config configs/stage2/stage2_real.yaml
    python3 scripts/stage2_make_conditions.py --config ... --orders 12      sequences for 12 participants
    python3 scripts/stage2_make_conditions.py --config ... --r-nav-max 1.2  override for a dry run (NOT recorded as a result)

Reads stage2.r_nav_max_m, comfort_step_m, n_conditions, size_floor_m and
writes one B3-O condition overlay per rung to configs/stage2/conditions/:

    b3o_r1.20.yaml, b3o_r1.10.yaml, b3o_r1.00.yaml, ...

exactly the files Stage 1 uses (comfort_core.size_sweep), so "B3-O at
1.00 m" means the same geometry on the real robot as in the simulator.

Refuses to run while stage2.r_nav_max_m is null: the ladder's top is a
Stage 1 measurement, not a guess. --r-nav-max exists to exercise the
pipeline and prints a warning when used.

--orders N writes orders.csv: participant P01..PNN, each a permutation
of the rungs drawn from stage2.order_seed, so the sequence a participant
sees is reproducible and not monotone in size.
"""

from __future__ import annotations

import argparse
import csv
import random
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.config import load_with_base       # noqa: E402
from comfort_core.size_sweep import write_conditions, condition_name  # noqa: E402
from comfort_core.stage2 import size_ladder, stage2_block  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/stage2/stage2_real.yaml")
    ap.add_argument("--out", default="configs/stage2/conditions")
    ap.add_argument("--orders", type=int, default=0,
                    help="also write orders.csv for this many participants")
    ap.add_argument("--r-nav-max", type=float, default=None,
                    help="override stage2.r_nav_max_m (pipeline check only)")
    args = ap.parse_args()

    cfg = load_with_base(args.config)
    if args.r_nav_max is not None:
        print(f"[WARN] stage2.r_nav_max_m overridden to {args.r_nav_max} on the command "
              f"line; this is a pipeline check, not the Stage 1 value", file=sys.stderr)
        cfg.setdefault("stage2", {})["r_nav_max_m"] = args.r_nav_max
    try:
        sizes = size_ladder(cfg)
    except ValueError as exc:
        print(f"[stage2_make_conditions] cannot build the ladder: {exc}", file=sys.stderr)
        return 2
    paths = write_conditions(args.out, sizes, kinds=("b3o",), stage="Stage 2")
    s2 = stage2_block(cfg)
    print(f"ladder from R_nav_max {sizes[0]:.2f} m, step {float(s2['comfort_step_m']):.2f} m: "
          + ", ".join(f"{s:.2f}" for s in sizes))
    for p in paths:
        print(f"  wrote {p.relative_to(PROJECT_ROOT) if p.is_relative_to(PROJECT_ROOT) else p}")

    if args.orders:
        rng = random.Random(int(s2.get("order_seed", 0)))
        out = Path(args.out) / "orders.csv"
        with out.open("w", newline="", encoding="utf-8") as h:
            w = csv.writer(h)
            w.writerow(["participant_id", "trial_order", "condition", "comfort_size"])
            for i in range(1, args.orders + 1):
                seq = list(sizes)
                rng.shuffle(seq)
                for order, size in enumerate(seq, 1):
                    w.writerow([f"P{i:02d}", order, condition_name("b3o", size), f"{size:.2f}"])
        print(f"  wrote {out} ({args.orders} participants x {len(sizes)} rungs, "
              f"seed {s2.get('order_seed')})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
