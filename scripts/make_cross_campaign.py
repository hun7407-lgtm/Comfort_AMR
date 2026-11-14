#!/usr/bin/env python3
"""Write cross-scenario campaign configs for any seed (Stage 3-B).

    python3 scripts/make_cross_campaign.py --verify            # s1-33 byte-identical?
    python3 scripts/make_cross_campaign.py --seeds 34-100      # write new seeds
    python3 scripts/make_cross_campaign.py --seeds 34-100 --dry-run

Plain Python 3. No ROS, no omni.

The 33 cross configs of 2026-11-05 were written without a generator in
the repository. Read back from the files, everything except the seed is
fixed and the two fleet start times follow one rule:

    AMR 2 (a1)  start_at = 1.0 + 0.3 * (seed mod 2)          1.0 / 1.3 s
    AMR 3 (a2)  start_at = 9.0 + 0.3 * ((5 * seed) mod 7)    9.0 ... 10.8 s

--verify regenerates seeds 1-33 with this rule from the seed-1 file and
compares byte for byte, so seeds 34-100 are made by the SAME procedure,
not an approximation of it.

What the seed decides (comfort_core/scenario.py::ScenarioGenerator.generate
via the base configs/scenarios/paper_h8_a3_scr_s<NNNN>.yaml):
    which north (N1-N3) and south (S1-S3) worker exchanges places with the
    edge worker, the four walkers' speeds, the north/south exchange start
    times, and (through this file) the fleet start times above.
What it does not: where the eight people stand and their states (fixed
roles of the bay), AMR 1's mission (fixed here: n1_r2c3 -> s3_r2c2), the
fleet's start slots and jobs (fixed), map, Nav2, safety layer.

A missing base scenario is generated with scripts/make_scenario.py
(--seed N --humans 8 --amrs 3), its own validation included.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CROSS = PROJECT_ROOT / "configs" / "campaigns" / "cross"
SCEN = PROJECT_ROOT / "configs" / "scenarios"
TEMPLATE_SEED = 1


def offsets(seed: int):
    return round(1.0 + 0.3 * (seed % 2), 1), round(9.0 + 0.3 * ((5 * seed) % 7), 1)


def fmt(v: float) -> str:
    return f"{v:.1f}"


def render(seed: int) -> str:
    text = (CROSS / f"campaign_h8_a3_cross_s{TEMPLATE_SEED}.yaml").read_text(encoding="utf-8")
    a1_0, a2_0 = offsets(TEMPLATE_SEED)
    a1, a2 = offsets(seed)
    subs = [
        (f"people from seed {TEMPLATE_SEED}.", f"people from seed {seed}."),
        (f"sets off at {fmt(a1_0)} s,", f"sets off at {fmt(a1)} s,"),
        (f"sets off at {fmt(a2_0)} s so", f"sets off at {fmt(a2)} s so"),
        (f"paper_h8_a3_scr_s{TEMPLATE_SEED:04d}.yaml", f"paper_h8_a3_scr_s{seed:04d}.yaml"),
        (f"campaign_h8_a3_cross_s{TEMPLATE_SEED}\n", f"campaign_h8_a3_cross_s{seed}\n"),
        (f"amr_offsets: {{a1: {fmt(a1_0)}, a2: {fmt(a2_0)}}}",
         f"amr_offsets: {{a1: {fmt(a1)}, a2: {fmt(a2)}}}"),
        (f"a1: {{enabled: true, start_at: {fmt(a1_0)},", f"a1: {{enabled: true, start_at: {fmt(a1)},"),
        (f"a2: {{enabled: true, start_at: {fmt(a2_0)},", f"a2: {{enabled: true, start_at: {fmt(a2)},"),
    ]
    for old, new in subs:
        if text.count(old) != 1:
            raise RuntimeError(f"template line not found exactly once: {old!r}")
        text = text.replace(old, new)
    return text


def parse_seeds(spec: str):
    out = []
    for part in spec.split(","):
        m = re.fullmatch(r"\s*(\d+)\s*(?:-\s*(\d+))?\s*", part)
        if not m:
            raise ValueError(f"bad seed spec {part!r}")
        a, b = int(m.group(1)), int(m.group(2) or m.group(1))
        out += list(range(a, b + 1))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", default="")
    ap.add_argument("--verify", action="store_true", help="regenerate 1-33 and compare")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if args.verify:
        bad = []
        for s in range(1, 34):
            p = CROSS / f"campaign_h8_a3_cross_s{s}.yaml"
            if p.read_text(encoding="utf-8") != render(s):
                bad.append(s)
        print(f"[make_cross_campaign] verify 1-33: "
              f"{'all byte-identical' if not bad else f'DIFFER at seeds {bad}'}")
        if bad:
            return 1
    for s in parse_seeds(args.seeds) if args.seeds else []:
        base = SCEN / f"paper_h8_a3_scr_s{s:04d}.yaml"
        out = CROSS / f"campaign_h8_a3_cross_s{s}.yaml"
        if out.exists():
            if out.read_text(encoding="utf-8") != render(s):
                print(f"[make_cross_campaign] {out.name} exists and differs; not overwritten")
                return 1
            continue
        if args.dry_run:
            print(f"  would write {out.name} (offsets {offsets(s)}), base "
                  f"{'exists' if base.exists() else 'to be generated'}")
            continue
        if not base.exists():
            r = subprocess.run([sys.executable, "scripts/make_scenario.py", "--seed", str(s),
                                "--humans", "8", "--amrs", "3", "--quiet"],
                               cwd=PROJECT_ROOT, capture_output=True, text=True)
            if r.returncode != 0 or not base.exists():
                print(f"[make_cross_campaign] seed {s}: make_scenario failed\n{r.stdout}{r.stderr}")
                return 1
        out.write_text(render(s), encoding="utf-8")
        print(f"  wrote {out.relative_to(PROJECT_ROOT)}  offsets AMR 2 {offsets(s)[0]} s, "
              f"AMR 3 {offsets(s)[1]} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
