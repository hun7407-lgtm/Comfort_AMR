#!/usr/bin/env python3
"""Write the Comfort Space size conditions as scenario overlays.

    python3 scripts/make_size_conditions.py                       0.8 1.0 1.2 1.4 1.6 -> configs/stage1/
    python3 scripts/make_size_conditions.py --sizes 1.25 1.3 1.35 refinement around a threshold
    python3 scripts/make_size_conditions.py --kinds b3o --sizes 1.1 1.0 0.9 \
        --out configs/stage2/conditions --stage "Stage 2"
    python3 scripts/make_size_conditions.py --table            print the applied F/S/R per condition

One file per (policy, size): static_r1.00.yaml, b3o_r1.00.yaml. Each
holds only `scenario` and `comfort_space`, so comfort_core.config will
overlay it on any campaign config without touching the people, the
missions or Nav2. The table shows what each condition comes out as in
metres on the cross scenario, which is what the trial rows record.

Pure Python; no ROS, no simulator.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.comfort_space import GEOMETRY_FIELDS       # noqa: E402
from comfort_core.size_sweep import (DEFAULT_SIZES, KIND_LABEL,  # noqa: E402
                                     applied_geometry, write_conditions)

REFERENCE_CAMPAIGN = "configs/campaigns/cross/campaign_h8_a3_cross_s1.yaml"


def table(paths, campaign: str) -> str:
    cols = ["condition", "policy"] + [f for f in GEOMETRY_FIELDS
                                      if f != "cs_moving_reference_speed"]
    short = {c: c.replace("cs_", "").replace("_m", "") for c in cols}
    rows = [applied_geometry(p, campaign) for p in paths]
    width = {c: max(len(short[c]), *(len(f"{r[c]}") for r in rows)) for c in cols}
    out = ["  ".join(short[c].ljust(width[c]) for c in cols)]
    for r in rows:
        out.append("  ".join(f"{r[c]}".ljust(width[c]) for c in cols))
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sizes", nargs="+", type=float, default=list(DEFAULT_SIZES))
    ap.add_argument("--kinds", nargs="+", choices=sorted(KIND_LABEL),
                    default=["static", "b3o"])
    ap.add_argument("--out", default="configs/stage1")
    ap.add_argument("--stage", default="Stage 1")
    ap.add_argument("--campaign", default=REFERENCE_CAMPAIGN,
                    help="config the --table extents are evaluated on")
    ap.add_argument("--table", action="store_true",
                    help="also print the applied extents per condition")
    args = ap.parse_args()

    paths = write_conditions(args.out, args.sizes, args.kinds, args.stage)
    for p in paths:
        print(f"wrote {p.relative_to(PROJECT_ROOT) if p.is_relative_to(PROJECT_ROOT) else p}")
    if args.table:
        print()
        print(f"applied extents on {args.campaign} (metres, after scaling and the "
              f"min_radius floor):")
        print(table(paths, args.campaign))
    return 0


if __name__ == "__main__":
    sys.exit(main())
