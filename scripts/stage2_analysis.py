#!/usr/bin/env python3
"""Stage 2 analysis: comfort curves per Human Context and the CANDIDATES for
R_comfort_min and R_pref.

    python3 scripts/stage2_analysis.py --session-dir results/stage2/261201_pilot
    python3 scripts/stage2_analysis.py --session-dir ... --item perceived_safety
    python3 scripts/stage2_analysis.py --session-dir A --session-dir B --out results/stage2/pooled

Reads <session-dir>/questionnaire.csv, trials.csv and status_amendments.csv
(any may be missing) and writes to --out (default: the first session dir)

    ANALYSIS.md
    comfort_by_context.json     per context x distance: n, mean, median, sd,
                                distribution; candidates; status counts
    figures/S1_comfort_curves.png       median (solid) / mean (dashed) per context, every rating
    figures/S2_participant_curves.png   one line per participant, a panel per context
    figures/S3_planned_vs_actual.png    planned d vs measured person centre -> AMR surface

Only ratings of NORMAL trials enter the curves (comfort_core.stage2.usable_ratings).
PARTICIPANT_STOP and every other status are counted per (context, d) on
their own: a stop is not a low comfort score.

Nothing here fixes R_comfort_min or R_pref. The rules are
stage2.analysis.r_comfort_min_rule / r_pref_rule and
stage2.comfort_acceptance_score, null until the pilot; until then the
report lists what each possible rule would give (every cutoff on the
scale, argmax, the midpoint crossing of a bipolar item). Item wording and
scales come from the config, never from this file.

Plain Python 3 + matplotlib (+ scipy for Friedman if present). With no rows
it prints the schema and exits 0: nothing is produced from nothing.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.config import load_with_base  # noqa: E402
from comfort_core.contexts import key_of  # noqa: E402
from comfort_core.stage2 import (QUESTIONNAIRE_FIELDS, RATING_KEYS, by_context_distance,  # noqa: E402
                                 comfort_candidates, questionnaire_items, r_comfort_min,
                                 read_rows, stage2_block, usable_ratings, validate_response)

# Fixed context order and colour (Okabe-Ito), so a context keeps its colour
# whichever subset is analysed.
COLOURS = {"c1": "#0072B2", "c2": "#E69F00", "c3": "#009E73", "c4": "#CC79A7"}


def ctx_key(name):
    try:
        return key_of(name)
    except ValueError:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session-dir", action="append", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--config", default="configs/stage2/stage2_real.yaml")
    ap.add_argument("--item", default="comfort_score", choices=RATING_KEYS)
    args = ap.parse_args()
    dirs = [Path(d) for d in args.session_dir]
    out = Path(args.out) if args.out else dirs[0]
    cfg = load_with_base(args.config)
    s2 = stage2_block(cfg)
    items = {it["key"]: it for it in questionnaire_items(cfg)}
    if args.item not in items:
        print(f"[stage2_analysis] item {args.item} is not in stage2.questionnaire.items")
        return 2
    item = items[args.item]
    scale = (item["scale_min"], item["scale_max"])
    cutoff = s2.get("comfort_acceptance_score")
    statistic = str(s2.get("acceptance_statistic") or "median")
    rules = s2.get("analysis") or {}

    q, trials, amend = [], [], []
    for d in dirs:
        q += read_rows(d / "questionnaire.csv")
        trials += read_rows(d / "trials.csv")
        amend += read_rows(d / "status_amendments.csv")
    if not q and not trials:
        print(f"[stage2_analysis] no questionnaire.csv / trials.csv rows in {dirs}; nothing to analyse.")
        print("  questionnaire schema: " + ", ".join(QUESTIONNAIRE_FIELDS))
        return 0
    bad = [(r.get("trial_id"), validate_response(r, list(items.values()))) for r in q]
    bad = [(t, p) for t, p in bad if p]
    q = [r for r in q if not validate_response(r, list(items.values()))]
    rows, excluded = usable_ratings(q, trials, amend)

    # status per (context, d), from trials.csv + amendments
    status_of = {t.get("trial_id"): t.get("trial_status") or "" for t in trials}
    for a in amend:
        status_of[a.get("trial_id")] = a.get("trial_status") or ""
    status_counts = defaultdict(Counter)
    for t in trials:
        status_counts[(t.get("context"), t.get("planned_distance_m"))][status_of[t.get("trial_id")]] += 1

    curves = by_context_distance(rows, args.item, scale)
    cands, verdicts = {}, {}
    for c, stats in curves.items():
        cands[c] = comfort_candidates(stats, scale, statistic, item.get("midpoint"))
        verdicts[c] = r_comfort_min(stats, None if cutoff is None else float(cutoff), statistic)
    contexts = sorted(curves, key=lambda c: (ctx_key(c) or "z", c))
    participants = sorted({r["participant_id"] for r in rows})

    # Friedman per context: distances as repeated conditions
    friedman = {}
    for c in contexts:
        per = defaultdict(lambda: defaultdict(list))
        for r in rows:
            if r.get("context") == c and r.get(args.item) not in ("", None):
                per[r["participant_id"]][float(r["passing_distance_m"])].append(int(r[args.item]))
        ds = sorted(curves[c])
        complete = [p for p in per if all(d in per[p] for d in ds)]
        if len(ds) >= 3 and len(complete) >= 3:
            try:
                from scipy.stats import friedmanchisquare
                cols = [[sum(per[p][d]) / len(per[p][d]) for p in complete] for d in ds]
                st, pv = friedmanchisquare(*cols)
                friedman[c] = {"n": len(complete), "k": len(ds), "chi2": float(st), "p": float(pv)}
            except ImportError:
                friedman[c] = {"note": "scipy not installed"}
        else:
            friedman[c] = {"note": f"needs >= 3 distances and >= 3 complete participants "
                                   f"(have {len(ds)}, {len(complete)})"}

    figs = out / "figures"
    figs.mkdir(parents=True, exist_ok=True)
    # S1 comfort curves
    fig, ax = plt.subplots(figsize=(7.5, 5.2))
    for i, c in enumerate(contexts):
        k = ctx_key(c)
        col = COLOURS.get(k, "#555555")
        ds = sorted(curves[c])
        jitter = (i - (len(contexts) - 1) / 2) * 0.012
        for d in ds:
            vals = [int(r[args.item]) for r in rows if r.get("context") == c
                    and float(r["passing_distance_m"]) == d and r.get(args.item) not in ("", None)]
            ax.scatter([d + jitter] * len(vals), vals, s=14, color=col, alpha=0.3, zorder=2)
        ax.plot(ds, [curves[c][d]["median"] for d in ds], color=col, marker="o", lw=2, ms=6,
                label=f"{c} median", zorder=3)
        ax.plot(ds, [curves[c][d]["mean"] for d in ds], color=col, ls="--", lw=1, zorder=3)
    if cutoff is not None:
        ax.axhline(float(cutoff), color="#444444", ls=":", lw=1.2, label=f"acceptance cutoff {cutoff}")
    ax.set_ylim(scale[0] - 0.3, scale[1] + 0.3)
    ax.set_yticks(range(scale[0], scale[1] + 1))
    ax.set_xlabel("planned passing distance d (m, person centre -> AMR surface)")
    ax.set_ylabel(f"{args.item} ({scale[0]}..{scale[1]})")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=2, frameon=False)
    fig.suptitle(f"S1  {args.item} by passing distance, per Human Context\n"
                 f"{len(rows)} ratings, {len(participants)} participants; solid = median, dashed = mean",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(figs / "S1_comfort_curves.png", dpi=150)
    plt.close(fig)
    # S2 participant curves, a panel per context
    fig, axes = plt.subplots(1, max(len(contexts), 1), figsize=(3.2 * max(len(contexts), 1) + 0.6, 3.4),
                             sharey=True, squeeze=False)
    for ax, c in zip(axes[0], contexts):
        for p in participants:
            pts = defaultdict(list)
            for r in rows:
                if r["participant_id"] == p and r.get("context") == c and r.get(args.item) not in ("", None):
                    pts[float(r["passing_distance_m"])].append(int(r[args.item]))
            xs = sorted(pts)
            ax.plot(xs, [sum(pts[x]) / len(pts[x]) for x in xs], marker="o", ms=3, lw=1, alpha=0.7,
                    color="#555555")
        ax.set_title(c, fontsize=9)
        ax.set_ylim(scale[0] - 0.3, scale[1] + 0.3)
        ax.grid(alpha=0.25)
        ax.set_xlabel("d (m)")
    axes[0][0].set_ylabel(args.item)
    fig.suptitle("S2  one line per participant (repeats averaged)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(figs / "S2_participant_curves.png", dpi=150)
    plt.close(fig)
    # S3 planned vs actual distance
    pa = [(float(t["planned_distance_m"]), float(t["actual_min_distance_m"]), ctx_key(t.get("context", "")))
          for t in trials if t.get("actual_min_distance_m") not in ("", None)
          and t.get("planned_distance_m") not in ("", None)]
    fig, ax = plt.subplots(figsize=(4.8, 4.4))
    if pa:
        for k in sorted({x[2] for x in pa}, key=str):
            sel = [x for x in pa if x[2] == k]
            ax.scatter([x[0] for x in sel], [x[1] for x in sel], s=18, color=COLOURS.get(k, "#555555"),
                       label=str(k))
        lo = min(min(x[0], x[1]) for x in pa) - 0.1
        hi = max(max(x[0], x[1]) for x in pa) + 0.1
        ax.plot([lo, hi], [lo, hi], color="#999999", lw=1, ls=":")
        ax.legend(fontsize=8)
    ax.set_xlabel("planned d (m)")
    ax.set_ylabel("measured min distance (m, person centre -> AMR surface)")
    ax.grid(alpha=0.25)
    fig.suptitle("S3  planned vs measured passing distance (all trials)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(figs / "S3_planned_vs_actual.png", dpi=150)
    plt.close(fig)

    measured = defaultdict(list)
    for t in trials:
        if status_of[t.get("trial_id")] == "NORMAL" and t.get("actual_min_distance_m") not in ("", None):
            measured[(t.get("context"), float(t["planned_distance_m"]))].append(float(t["actual_min_distance_m"]))

    (out / "comfort_by_context.json").write_text(json.dumps({
        "item": args.item, "scale": scale, "questionnaire_versions": sorted({r.get("questionnaire_version", "") for r in rows}),
        "curves": {c: {f"{d:.2f}": curves[c][d] for d in sorted(curves[c])} for c in contexts},
        "candidates": cands, "r_comfort_min_with_configured_cutoff": verdicts,
        "rules": {"comfort_acceptance_score": cutoff, "statistic": statistic, **rules},
        "excluded_ratings": excluded, "invalid_rows": bad,
        "status_counts": {f"{c}@{d}": dict(v) for (c, d), v in status_counts.items()},
        "friedman": friedman}, indent=2, default=str), encoding="utf-8")

    L = [f"# Stage 2 analysis: {args.item}", "",
         f"Sessions: {', '.join(str(d) for d in dirs)}. {len(rows)} usable ratings (NORMAL trials) "
         f"from {len(participants)} participants. Scale {scale[0]}..{scale[1]}; statistic {statistic}.", "",
         "R_comfort_min and R_pref are NOT decided here: "
         f"`comfort_acceptance_score` = {cutoff}, `r_comfort_min_rule` = {rules.get('r_comfort_min_rule')}, "
         f"`r_pref_rule` = {rules.get('r_pref_rule')}. The tables list candidates only.", ""]
    L += ["## Trial status per (context, d)", "", "| context | d (m) | " + " | ".join(
        ("NORMAL", "PARTICIPANT_STOP", "OPERATOR_ABORT", "E_STOP", "NAVIGATION_FAILURE", "INVALID_TRIAL")) + " |",
          "|" + "---|" * 8]
    for (c, d), cnt in sorted(status_counts.items(), key=lambda kv: (str(kv[0][0]), str(kv[0][1]))):
        L.append(f"| {c} | {d} | " + " | ".join(str(cnt.get(s, 0)) for s in (
            "NORMAL", "PARTICIPANT_STOP", "OPERATOR_ABORT", "E_STOP", "NAVIGATION_FAILURE", "INVALID_TRIAL")) + " |")
    if excluded:
        L += ["", "Ratings excluded from the curves: " + "; ".join(
            f"{k}: {len(v)}" for k, v in sorted(excluded.items()) if k != "no_trial_row")
            + ("" if "no_trial_row" not in excluded else
               f"; kept without a trial row: {len(excluded['no_trial_row'])}")]
    if bad:
        L += ["", f"Invalid questionnaire rows (excluded): {len(bad)}"]
    for c in contexts:
        st = curves[c]
        L += ["", f"## {c}", "", f"| d (m) | n | mean | median | sd | measured d (mean, m) | "
              + " | ".join(str(k) for k in range(scale[0], scale[1] + 1)) + " |",
              "|" + "---|" * (6 + scale[1] - scale[0] + 1)]
        for d in sorted(st):
            m = measured.get((c, d))
            sd = st[d]["sd"]
            L.append(f"| {d:.2f} | {st[d]['n']} | {st[d]['mean']:.2f} | {st[d]['median']:.1f} | "
                     f"{'-' if math.isnan(sd) else f'{sd:.2f}'} | "
                     f"{'-' if not m else f'{sum(m) / len(m):.3f}'} | "
                     + " | ".join(str(st[d]["distribution"][k]) for k in range(scale[0], scale[1] + 1)) + " |")
        cd = cands[c]
        L += ["", f"Candidates ({statistic}):", "",
              "| cutoff | smallest d reaching it | larger d falling below again |", "|---|---|---|"]
        for cut, v in cd["r_comfort_min_by_cutoff"].items():
            L.append(f"| >= {cut} | {v['distance_m'] if v['distance_m'] is not None else '-'} | "
                     f"{v['non_monotonic_above'] or '-'} |")
        L += ["", f"- R_pref candidate, argmax of {statistic}: {cd['r_pref_argmax']}"]
        if "r_pref_midpoint" in cd:
            mp = cd["r_pref_midpoint"]
            L.append(f"- R_pref candidate, midpoint {mp['midpoint']}: at {mp['at_midpoint_m'] or '-'}, "
                     f"linear crossing {mp['linear_crossing_m']}")
        L += [f"- with the configured cutoff: {verdicts[c]['R_comfort_min_m']} ({verdicts[c]['note']})",
              f"- Friedman over d: {friedman[c]}"]
    L += ["", "Figures: figures/S1_comfort_curves.png, S2_participant_curves.png, S3_planned_vs_actual.png.",
          "Raw files are never modified by this script.", ""]
    (out / "ANALYSIS.md").write_text("\n".join(L), encoding="utf-8")
    print(f"[stage2_analysis] {len(rows)} usable ratings, {len(contexts)} contexts -> {out / 'ANALYSIS.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
