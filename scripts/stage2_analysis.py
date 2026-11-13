#!/usr/bin/env python3
"""Stage 2 analysis: the comfort-size curve and R_comfort_min.

    python3 scripts/stage2_analysis.py --session-dir results/stage2/261201_pilot
    python3 scripts/stage2_analysis.py --session-dir ... --config configs/stage2/stage2_real.yaml

Reads <session-dir>/questionnaire.csv (and trials.csv when present, for
the measured passing distance per trial) and writes

    <session-dir>/ANALYSIS.md
    <session-dir>/comfort_by_size.json      per size: n, mean, median, sd, distribution
    <session-dir>/figures/S1_comfort_vs_size.png     per-size median/IQR + every rating
    <session-dir>/figures/S2_participant_curves.png  one line per participant (paired view)
    <session-dir>/figures/S3_distribution.png        1..7 histogram per size

R_comfort_min is derived ONLY if stage2.comfort_acceptance_score is set
in the config. It ships as null, so with the shipped config this script
reports the curve and says that no cutoff has been decided. A two-segment
breakpoint fit is reported as exploratory beside it. A repeated-measures
test (Friedman) is run when scipy is installed and every participant has
every size; otherwise it says so.

Plain Python 3 + matplotlib (+ scipy if present). With no rows it
prints the schema and exits 0: nothing is produced from nothing.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.config import load_with_base  # noqa: E402
from comfort_core.stage2 import (LIKERT_MAX, LIKERT_MIN, QUESTIONNAIRE_FIELDS,  # noqa: E402
                                 breakpoint_estimate, by_size, paired_table,
                                 r_comfort_min, read_rows, validate_response)

BLUE = "#0072b2"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session-dir", required=True)
    ap.add_argument("--config", default="configs/stage2/stage2_real.yaml")
    ap.add_argument("--item", default="comfort_score",
                    choices=("comfort_score", "perceived_safety", "distance_acceptability"))
    args = ap.parse_args()
    d = Path(args.session_dir)
    cfg = load_with_base(args.config)
    s2 = cfg.get("stage2", {}) or {}
    cutoff = s2.get("comfort_acceptance_score")
    statistic = str(s2.get("acceptance_statistic") or "median")

    rows = read_rows(d / "questionnaire.csv")
    if not rows:
        print(f"[stage2_analysis] no questionnaire.csv rows in {d}; nothing to analyse.")
        print("  schema: " + ", ".join(QUESTIONNAIRE_FIELDS))
        return 0
    bad = [(r.get("trial_id"), validate_response(r)) for r in rows]
    bad = [(t, p) for t, p in bad if p]
    if bad:
        print(f"[stage2_analysis] {len(bad)} invalid row(s), excluded:")
        for t, p in bad[:10]:
            print(f"   {t}: {'; '.join(p)}")
        rows = [r for r in rows if not validate_response(r)]

    stats = by_size(rows, args.item)
    paired = paired_table(rows, args.item)
    verdict = r_comfort_min(stats, None if cutoff is None else float(cutoff), statistic)
    knee = breakpoint_estimate(stats, "mean")
    sizes = sorted(stats)
    participants = sorted(paired)

    # Friedman, if possible
    friedman = None
    complete = [p for p in participants if all(s in paired[p] for s in sizes)]
    if len(sizes) >= 3 and len(complete) >= 3:
        try:
            from scipy.stats import friedmanchisquare
            cols = [[paired[p][s] for p in complete] for s in sizes]
            stat, pval = friedmanchisquare(*cols)
            friedman = {"n_participants": len(complete), "k_sizes": len(sizes),
                        "chi2": float(stat), "p": float(pval)}
        except ImportError:
            friedman = {"note": "scipy not installed; Friedman test skipped"}
    else:
        friedman = {"note": f"needs >= 3 sizes and >= 3 participants with every size "
                            f"(have {len(sizes)} sizes, {len(complete)} complete participants)"}

    # trials.csv join: measured passing distance per size
    trials = read_rows(d / "trials.csv")
    clr_by_size = {}
    for t in trials:
        try:
            clr_by_size.setdefault(round(float(t["comfort_size_m"]), 4), []).append(float(t["min_clearance"]))
        except (KeyError, ValueError):
            pass

    figs = d / "figures"
    figs.mkdir(exist_ok=True)
    # S1
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for s in sizes:
        vals = [int(r[args.item]) for r in rows if round(float(r["comfort_size"]), 4) == s
                and r.get(args.item) not in ("", None)]
        ax.scatter([s] * len(vals), vals, s=18, color=BLUE, alpha=0.35, zorder=2)
    med = [stats[s]["median"] for s in sizes]
    ax.plot(sizes, med, color=BLUE, marker="o", lw=2, ms=7, label=f"median ({args.item})", zorder=3)
    ax.plot(sizes, [stats[s]["mean"] for s in sizes], color=BLUE, ls="--", lw=1, label="mean", zorder=3)
    if cutoff is not None:
        ax.axhline(float(cutoff), color="#b2182b", ls=":", lw=1.2, label=f"acceptance cutoff {cutoff}")
        if verdict["R_comfort_min_m"] is not None:
            ax.axvline(verdict["R_comfort_min_m"], color="#b2182b", lw=1,
                       label=f"R_comfort_min {verdict['R_comfort_min_m']:.2f} m")
    ax.set_ylim(LIKERT_MIN - 0.3, LIKERT_MAX + 0.3)
    ax.set_yticks(range(LIKERT_MIN, LIKERT_MAX + 1))
    ax.set_xlabel("B3-O Comfort Space size (m)")
    ax.set_ylabel(f"{args.item} (1..7)")
    ax.set_xticks(sizes)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
    fig.suptitle(f"{d.name}\nS1  perceived comfort vs Comfort Space size "
                 f"(n={len(rows)} ratings, {len(participants)} participants)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(figs / "S1_comfort_vs_size.png", dpi=150)
    plt.close(fig)
    # S2
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for p in participants:
        xs = sorted(paired[p])
        ax.plot(xs, [paired[p][x] for x in xs], marker="o", ms=4, lw=1, alpha=0.7, label=p)
    ax.set_ylim(LIKERT_MIN - 0.3, LIKERT_MAX + 0.3)
    ax.set_xlabel("B3-O Comfort Space size (m)")
    ax.set_ylabel(f"{args.item} (1..7)")
    ax.set_xticks(sizes)
    ax.grid(alpha=0.25)
    if len(participants) <= 12:
        ax.legend(fontsize=7, ncol=2)
    fig.suptitle(f"{d.name}\nS2  one line per participant (paired comparison)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(figs / "S2_participant_curves.png", dpi=150)
    plt.close(fig)
    # S3
    fig, axes = plt.subplots(1, len(sizes), figsize=(2.2 * len(sizes) + 1, 3.2), sharey=True)
    axes = list(axes) if len(sizes) > 1 else [axes]
    for ax, s in zip(axes, sizes):
        dist = stats[s]["distribution"]
        ax.bar(list(dist), list(dist.values()), color=BLUE, edgecolor="white")
        ax.set_title(f"{s:.2f} m (n={stats[s]['n']})", fontsize=9)
        ax.set_xticks(range(LIKERT_MIN, LIKERT_MAX + 1))
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("ratings")
    fig.suptitle(f"{d.name}\nS3  rating distribution per size", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(figs / "S3_distribution.png", dpi=150)
    plt.close(fig)

    (d / "comfort_by_size.json").write_text(json.dumps(
        {"item": args.item, "by_size": {f"{s:.2f}": stats[s] for s in sizes},
         "r_comfort_min": verdict, "breakpoint": knee, "friedman": friedman},
        indent=2, default=str), encoding="utf-8")

    lines = [f"# {d.name}: Stage 2 analysis ({args.item})", "",
             f"{len(rows)} valid ratings from {len(participants)} participants over "
             f"{len(sizes)} sizes.", "",
             "## Per size", "",
             "| size (m) | n | mean | median | sd | min | max | 1 | 2 | 3 | 4 | 5 | 6 | 7 | "
             "measured min clearance (m, mean) |", "|" + "---|" * 15]
    for s in sizes:
        st = stats[s]
        clr = clr_by_size.get(s)
        lines.append(f"| {s:.2f} | {st['n']} | {st['mean']:.2f} | {st['median']:.1f} | "
                     f"{'-' if math.isnan(st['sd']) else f'{st['sd']:.2f}'} | {st['min']} | {st['max']} | "
                     + " | ".join(str(st["distribution"][k]) for k in range(1, 8))
                     + f" | {'-' if not clr else f'{sum(clr) / len(clr):.2f}'} |")
    lines += ["", "## R_comfort_min", "",
              f"- cutoff (`stage2.comfort_acceptance_score`): **{cutoff}**; statistic: {statistic}",
              f"- R_comfort_min: **{verdict['R_comfort_min_m']}**",
              f"- {verdict['note']}", "",
              "## Exploratory breakpoint (two-segment fit on per-size means)", ""]
    if knee:
        lines += [f"- knee at {knee['knee_m']:.2f} m; slope below {knee['slope_below']:.2f}, "
                  f"above {knee['slope_above']:.2f} per m; residual {knee['residual']:.3f} vs "
                  f"single line {knee['single_segment_residual']:.3f}"]
    else:
        lines += ["- needs >= 4 sizes"]
    lines += ["", "## Repeated measures", "", f"- Friedman: {friedman}", "",
              "## Participant x size (mean rating)", "",
              "| participant | " + " | ".join(f"{s:.2f}" for s in sizes) + " |",
              "|" + "---|" * (len(sizes) + 1)]
    for p in participants:
        lines.append(f"| {p} | " + " | ".join(
            f"{paired[p][s]:.1f}" if s in paired[p] else "-" for s in sizes) + " |")
    lines += ["", "Figures: figures/S1_comfort_vs_size.png, S2_participant_curves.png, "
              "S3_distribution.png. Raw data: questionnaire.csv (never modified by this script).", ""]
    (d / "ANALYSIS.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"[stage2_analysis] {len(rows)} ratings, {len(sizes)} sizes -> {d / 'ANALYSIS.md'}")
    print(f"  R_comfort_min: {verdict['R_comfort_min_m']}  ({verdict['note']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
