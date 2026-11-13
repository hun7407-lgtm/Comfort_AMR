#!/usr/bin/env python3
"""Write the Stage 1 trial list for scripts/run_campaign.sh.

    python3 scripts/stage1_manifest.py --out results/batches/261112_stage1_size_sweep/manifest.txt
    python3 scripts/stage1_manifest.py --seeds 5 --sizes 0.8 1.2 1.6 --out ...      smoke test
    python3 scripts/stage1_manifest.py --seeds 1-33 --sizes 1.25 1.3 1.35 --out ... refinement

One line per trial:

    configs/stage1/<kind>_r<size>.yaml configs/campaigns/cross/campaign_h8_a3_cross_s<seed>.yaml --fleet-tail 150

Seed-major order: every condition of seed 1, then every condition of
seed 2, ... so that a sweep stopped part way has complete seeds rather
than complete sizes, and whatever has run so far compares like with
like. Within a seed the conditions alternate static / B3-O and climb
in size.

The condition files must already exist (scripts/make_size_conditions.py);
a missing one is an error here, not forty minutes into the run.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.size_sweep import DEFAULT_SIZES, condition_name  # noqa: E402

CAMPAIGN = "configs/campaigns/cross/campaign_h8_a3_cross_s{seed}.yaml"
CONDITIONS = "configs/stage1"
FLEET_TAIL = 150


def parse_seeds(spec: list[str]) -> list[int]:
    """"1-33" or "1 2 5" or "1-8 12" -> sorted unique ints."""
    out: set[int] = set()
    for item in spec:
        for part in str(item).split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                a, b = part.split("-", 1)
                out.update(range(int(a), int(b) + 1))
            else:
                out.add(int(part))
    return sorted(out)


def lines(seeds, sizes, kinds, conditions_dir=CONDITIONS, campaign=CAMPAIGN,
          fleet_tail=FLEET_TAIL) -> list[str]:
    out = []
    for seed in seeds:
        cfg = campaign.format(seed=seed)
        if not (PROJECT_ROOT / cfg).is_file():
            raise FileNotFoundError(f"no campaign config for seed {seed}: {cfg}")
        for size in sizes:
            for kind in kinds:
                cond = f"{conditions_dir}/{condition_name(kind, size)}.yaml"
                if not (PROJECT_ROOT / cond).is_file():
                    raise FileNotFoundError(
                        f"no condition file {cond}; run scripts/make_size_conditions.py")
                out.append(f"{cond} {cfg} --fleet-tail {fleet_tail}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", nargs="+", default=["1-33"])
    ap.add_argument("--sizes", nargs="+", type=float, default=list(DEFAULT_SIZES))
    ap.add_argument("--kinds", nargs="+", default=["static", "b3o"])
    ap.add_argument("--conditions", default=CONDITIONS)
    ap.add_argument("--fleet-tail", type=int, default=FLEET_TAIL)
    ap.add_argument("--title", default="Stage 1 Comfort Space size sweep")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    seeds = parse_seeds(args.seeds)
    rows = lines(seeds, args.sizes, args.kinds, args.conditions,
                 fleet_tail=args.fleet_tail)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    header = (f"# {args.title}: {len(args.kinds)} policies x {len(args.sizes)} sizes "
              f"x {len(seeds)} seeds = {len(rows)} trials\n"
              f"# sizes {' '.join(f'{s:.2f}' for s in args.sizes)} m; "
              f"kinds {' '.join(args.kinds)}; seeds {seeds[0]}-{seeds[-1]}\n"
              "# cross scenario; same campaign config for every condition of a seed\n")
    out.write_text(header + "\n".join(rows) + "\n", encoding="utf-8")
    print(f"wrote {out}: {len(rows)} trials "
          f"({len(args.kinds)} x {len(args.sizes)} x {len(seeds)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
