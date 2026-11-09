#!/usr/bin/env python3
"""Merge several campaign result folders into one, for a combined report.

    python3 scripts/merge_campaigns.py --out results/batches/261107_MAIN_cross_seeds1-33_merged \
        results/batches/261105_cross_seeds1-8 results/batches/261106_cross_seeds9-33

Concatenates trials.csv and fleet_trials.csv (one header), copies the
manifests and campaign logs under their folder's name, and symlinks every
trial log into <out>/logs/ with the folder's name as a prefix so the
report scripts' globs (logs/*_<stem>_<policy>.log) still match and two
campaigns' "005_..." files cannot collide. Nothing in the sources is
touched. Plain Python 3.
"""
from __future__ import annotations

import argparse
import csv
import shutil
import sys
from pathlib import Path


def concat(paths, out: Path) -> int:
    rows, header = [], None
    for p in paths:
        if not p.exists():
            continue
        with p.open(newline="") as f:
            r = csv.DictReader(f)
            if header is None:
                header = r.fieldnames
            elif r.fieldnames != header:
                # same tooling, same columns -- anything else is a mistake
                sys.exit(f"column mismatch between campaigns: {p}")
            rows.extend(r)
    if header is None:
        return 0
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sources", nargs="+")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    out = Path(args.out)
    (out / "logs").mkdir(parents=True, exist_ok=True)
    srcs = [Path(s) for s in args.sources]
    n = concat([s / "trials.csv" for s in srcs], out / "trials.csv")
    m = concat([s / "fleet_trials.csv" for s in srcs], out / "fleet_trials.csv")
    linked = 0
    for s in srcs:
        for name in ("manifest.txt", "campaign.log"):
            if (s / name).exists():
                shutil.copy(s / name, out / f"{s.name}__{name}")
        for log in sorted((s / "logs").glob("*.log")):
            dst = out / "logs" / f"{s.name}__{log.name}"
            if dst.is_symlink() or dst.exists():
                dst.unlink()
            dst.symlink_to(log.resolve())
            linked += 1
    (out / "SOURCES.md").write_text(
        "# merged from\n" + "".join(f"- {s}\n" for s in srcs)
        + f"\n{n} AMR1 rows, {m} fleet rows, {linked} log links\n")
    print(f"{out}: {n} AMR1 rows, {m} fleet rows, {linked} log links from {len(srcs)} campaign(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
