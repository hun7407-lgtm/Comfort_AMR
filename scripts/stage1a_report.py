#!/usr/bin/env python3
"""Stage 1 controlled experiment: predicted bounds vs observed passage.

    python3 scripts/stage1a_report.py --dir results/batches/<stage1a batch> [--dir ...]

Reads trials.csv from one or more batch folders and the PRE-REGISTERED
predictions in configs/stage1a/predictions.json (written by
scripts/stage1a_plan.py before anything ran). Never recomputes the
predictions from the results.

Per cell (environment x policy/context) and per tested extent e:
passes, comfort gate holds, other failures by mechanism. Then

    R_exec bracket  = (largest e passed by the majority with every smaller
                       tested e also passed, smallest e above it that was
                       not) -- observed passage boundary
    agreement       = every e <= R_path^infl passed (majority) AND every
                      e > R_path^lethal did NOT pass (majority, invalid runs
                      excluded); points between the two bounds are the
                      model's uncertainty band and are reported, not scored.
                      How a run failed is listed beside it: in this corridor
                      a blocked passage often shows as a controller stall
                      in front of the Comfort Space (soft mask cost) rather
                      than as a gate hold, so both are kept, never merged.

Writes REPORT.md, cells.json and figures:
    F1_feasibility_map.png   every trial on the e axis, one row per cell,
                             predicted band shaded
    F2_pred_vs_exec.png      R_path^infl / R_path^lethal vs R_exec bracket
    F3_context_mapping.png   W 2.2 contexts on the target_size axis and on
                             the effective-extent axis
    F0_step0.png             B1 runs (no Comfort Space), every width

Display names: AMR 1 (the only robot here).

> Engineering bounds of the navigation stack. Not comfort distances.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from comfort_core.size_sweep import parse_condition  # noqa: E402

PRED = PROJECT_ROOT / "configs" / "stage1a" / "predictions.json"
COLOUR = {"pass": "#0072b2", "gate": "#d55e00", "other": "#7f7f7f"}
MARK = {"pass": "o", "gate": "x", "other": "^"}
LABEL = {"pass": "SUCCESS", "gate": "comfort gate hold", "other": "other failure"}
CTX_COLOUR = {"c1": "#0072b2", "c2": "#e69f00", "c3": "#009e73", "c4": "#cc79a7"}


def outcome(r: Dict[str, str]) -> str:
    if r.get("result") == "SUCCESS":
        return "pass"
    if (r.get("deadlock_mechanism") or "") == "comfort_gate_hold":
        return "gate"
    return "other"


def mech(r: Dict[str, str]) -> str:
    """Mechanism, with a stall split by what the safety layer was doing at
    the verdict (deadlock_cause): ProtectiveStop / Slowdown / none."""
    if r.get("result") == "SUCCESS":
        return "SUCCESS"
    m = r.get("deadlock_mechanism") or r.get("result") or "?"
    cause = r.get("deadlock_cause") or ""
    if m == "stall":
        if "ProtectiveStop" in cause:
            return "stall/protective-stop"
        if "Slowdown" in cause:
            return "stall/slowdown"
    return m


def load_rows(dirs: List[Path]) -> List[Dict[str, str]]:
    rows = []
    for d in dirs:
        p = d / "trials.csv"
        if p.exists():
            with p.open(newline="") as f:
                for r in csv.DictReader(f):
                    r["_batch"] = d.name
                    rows.append(r)
    return rows


def majority(c: Counter, key: str) -> bool:
    n = sum(c.values())
    return n > 0 and c[key] * 2 > n


_CFG_CACHE: Dict[str, Any] = {}


def _extent(cell: Dict[str, Any], value: float) -> float:
    """Effective extent of a policy value in this cell's environment."""
    from comfort_core.config import load_config
    from comfort_core import feasibility as fz
    env = cell["env"]
    if env not in _CFG_CACHE:
        _CFG_CACHE[env] = load_config(common_path=str(PROJECT_ROOT / "configs" / "stage1a"
                                                      / "envs" / f"{env}.yaml"))
    return round(fz.extent_of(_CFG_CACHE[env], cell["kind"], value, "a"), 4)


def analyse(cells: List[Dict[str, Any]], rows: List[Dict[str, str]]):
    by_cell = defaultdict(list)
    step0 = defaultdict(list)
    invalid = Counter()
    for r in rows:
        env = Path(r.get("config", "")).stem
        if r.get("scenario") == "b1" or r.get("policy") == "none":
            step0[env].append(r)
            continue
        parsed = parse_condition(r.get("scenario", ""))
        if not parsed:
            continue
        if r.get("result") == "INVALID" or str(r.get("valid", "True")) not in ("True", "true", "1"):
            invalid[Path(r.get("config", "")).stem] += 1
            continue
        kind, value = parsed
        by_cell[(env, kind, round(value, 2))].append(r)
    out = []
    for c in cells:
        pts = []
        seen = set()
        # Every value that was run in this cell counts, predicted point or
        # not (scan, refine and exploration runs land here too).
        extra = [{"point": "observed", "value": v, "e": _extent(c, v),
                  "condition": f"{c['kind']}_r{v:.2f}", "repeats": 0}
                 for (env, kind, v) in by_cell if env == c["env"] and kind == c["kind"]]
        for t in list(c["trials"]) + extra:
            key = (c["env"], c["kind"], round(t["value"], 2))
            if key in seen:
                continue
            seen.add(key)
            rs = by_cell.get(key, [])
            cnt = Counter(outcome(r) for r in rs)
            pts.append({**t, "n": len(rs), "pass": cnt["pass"], "gate": cnt["gate"],
                        "other": cnt["other"], "mechanisms": dict(Counter(mech(r) for r in rs)),
                        "outcomes": [outcome(r) for r in rs]})
        pts.sort(key=lambda p: p["e"])
        tested = [p for p in pts if p["n"]]
        # R_exec bracket: the largest e the majority passed, and the next
        # tested e above it (scripts/stage1a_calibrate.boundary). Failures
        # below it are listed, not hidden.
        passed = [p["e"] for p in tested if p["pass"] * 2 > p["n"]]
        lo = max(passed) if passed else None
        above = [p["e"] for p in tested if lo is None or p["e"] > lo]
        hi = min(above) if above else None
        low_fail = [p["e"] for p in tested if lo is not None and p["e"] < lo
                    and p["pass"] * 2 <= p["n"]]
        e_infl, e_leth = c["R_path_infl"]["e"], c["R_path_lethal"]["e"]
        if "R_exec_model" in c:
            ok_e, fail_e = c["R_exec_model"]["e"], c["predicted_fail_above_e"]
            basis = "R_exec_model"
        else:
            ok_e, fail_e = e_infl, e_leth
            basis = "R_path (infl/lethal)"
        must_pass = [p for p in tested if p["e"] <= ok_e + 1e-9]
        must_fail = [p for p in tested if p["e"] > fail_e + 1e-9]
        band = [p for p in tested if ok_e + 1e-9 < p["e"] <= fail_e + 1e-9]
        ok_pass = all(p["pass"] * 2 > p["n"] for p in must_pass)
        ok_fail = all((p["n"] - p["pass"]) * 2 > p["n"] for p in must_fail)
        complete = len(tested) == len(pts)
        verdict = ("not run" if not tested else
                   ("consistent" if ok_pass and ok_fail else "INCONSISTENT")
                   + ("" if complete else " (partial)"))
        nonmono = low_fail
        out.append({**{k: c[k] for k in ("part", "env", "aisle_width_m", "human_wall_offset_m",
                                          "context", "context_label", "kind")},
                    "R_path_infl": c["R_path_infl"], "R_path_lethal": c["R_path_lethal"],
                    "R_exec_model": c.get("R_exec_model"), "scored_against": basis,
                    "path_miss_m": (None if lo is None else round(e_infl - lo, 3)),
                    "R_exec_bracket": [lo, hi], "verdict": verdict,
                    "pass_below_infl": ok_pass, "gate_above_lethal": ok_fail,
                    "band_points": [(p["e"], p["pass"], p["n"]) for p in band],
                    "non_monotonic_e": nonmono, "invalid_runs": invalid.get(c["env"], 0),
                    "points": pts})
    return out, step0


def cell_label(c) -> str:
    who = "Static" if c["kind"] == "static" else f"B3-O {c['context'].upper()}"
    return f"{c['part']}  W {c['aisle_width_m']:.2f}  y_h {c['human_wall_offset_m']:.2f}  {who}"


def fig_map(res, out: Path):
    run = [c for c in res if any(p["n"] for p in c["points"])]
    if not run:
        return False
    fig, ax = plt.subplots(figsize=(8.5, 0.55 * len(run) + 2.2))
    for i, c in enumerate(run):
        y = len(run) - 1 - i
        ax.fill_betweenx([y - 0.38, y + 0.38], c["R_path_infl"]["e"], c["R_path_lethal"]["e"],
                         color="#bbbbbb", alpha=0.6, lw=0)
        if c.get("R_exec_model"):
            ax.plot([c["R_exec_model"]["e"]] * 2, [y - 0.4, y + 0.4], color="#009e73", lw=2)
        for p in c["points"]:
            for j, o in enumerate(p["outcomes"]):
                ax.scatter(p["e"], y + (j - (len(p["outcomes"]) - 1) / 2) * 0.17,
                           marker=MARK[o], color=COLOUR[o], s=30, zorder=3)
    ax.set_yticks(range(len(run)))
    ax.set_yticklabels([cell_label(c) for c in reversed(run)], fontsize=7)
    ax.set_xlabel("effective corridor-side Comfort extent e (m)")
    for o in ("pass", "gate", "other"):
        ax.scatter([], [], marker=MARK[o], color=COLOUR[o], label=LABEL[o])
    ax.fill_between([], [], color="#bbbbbb", alpha=0.6, label="path bounds [R_path^infl, R_path^lethal]")
    if any(c.get("R_exec_model") for c in run):
        ax.plot([], [], color="#009e73", lw=2, label="R_exec_model (fitted on 1A-1, pre-registered)")
    ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, frameon=False)
    ax.grid(axis="x", alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_title("F1  Stage 1: every AMR 1 trial on the effective-extent axis", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return True


def fig_pred_exec(res, out: Path):
    run = [c for c in res if c["R_exec_bracket"][0] is not None or c["R_exec_bracket"][1] is not None]
    if not run:
        return False
    fig, ax = plt.subplots(figsize=(5.4, 5.0))
    allv = []
    for c in run:
        if c.get("R_exec_model"):
            x0 = x1 = c["R_exec_model"]["e"]
        else:
            x0, x1 = c["R_path_infl"]["e"], c["R_path_lethal"]["e"]
        lo, hi = c["R_exec_bracket"]
        lo_v = lo if lo is not None else min(p["e"] for p in c["points"]) - 0.05
        hi_v = hi if hi is not None else max(p["e"] for p in c["points"]) + 0.05
        col = "#0072b2" if c["kind"] == "b3o" else "#d55e00"
        mk = "o" if c.get("R_exec_model") else "s"
        ax.plot([x0, x1], [(lo_v + hi_v) / 2] * 2, color=col, lw=2, marker=mk, ms=4)
        ax.plot([(x0 + x1) / 2] * 2, [lo_v, hi_v], color=col, lw=1)
        allv += [x0, x1, lo_v, hi_v]
    lim = (min(allv) - 0.1, max(allv) + 0.1)
    ax.plot(lim, lim, color="#555555", ls="--", lw=1, label="R_exec = R_path")
    ax.plot([], [], color="#d55e00", lw=2, label="Static cells")
    ax.plot([], [], color="#0072b2", lw=2, label="B3-O cells")
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.plot([], [], color="#555555", marker="s", ls="", label="1A-1 (x = path bounds, before calibration)")
    ax.plot([], [], color="#555555", marker="o", ls="", label="1A-2 / 1-B (x = R_exec_model, pre-registered)")
    ax.set_xlabel("predicted boundary e (m)")
    ax.set_ylabel("observed R_exec bracket (m, vertical bar)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.25)
    ax.set_title("F2  predicted vs observed passage boundary", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return True


def fig_context(res, out: Path):
    cs = [c for c in res if c["kind"] == "b3o" and abs(c["aisle_width_m"] - 2.2) < 1e-6
          and abs(c["human_wall_offset_m"] - 0.35) < 1e-6 and any(p["n"] for p in c["points"])]
    if not cs:
        return False
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    for k, ax, xlab in ((("value"), axes[0], "B3-O target_size_m (implementation parameter)"),
                        (("e"), axes[1], "effective corridor-side extent e (m)")):
        for i, c in enumerate(cs):
            pts = [p for p in c["points"] if p["n"]]
            xs = [p[k] for p in pts]
            ys = [p["pass"] / p["n"] for p in pts]
            ax.plot(xs, ys, marker="o", color=CTX_COLOUR.get(c["context"], "#333"),
                    label=f"{c['context'].upper()} {c['context_label']}")
        if k == "e":
            c0 = cs[0]
            ax.axvspan(c0["R_path_infl"]["e"], c0["R_path_lethal"]["e"], color="#bbbbbb",
                       alpha=0.5, label="predicted band")
        ax.set_xlabel(xlab)
        ax.grid(alpha=0.25)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[0].set_ylabel("AMR 1 pass fraction")
    axes[1].legend(fontsize=7, loc="lower left")
    fig.suptitle("F3  W = 2.2 m: Human Context on the parameter axis vs the effective-extent axis",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return True


def fig_step0(step0, out: Path):
    if not step0:
        return False
    envs = sorted(step0)
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    for i, env in enumerate(envs):
        rs = step0[env]
        for j, r in enumerate(rs):
            o = outcome(r)
            ax.scatter(i + (j - (len(rs) - 1) / 2) * 0.12, float(r.get("min_clearance") or 0) + 0.25,
                       marker=MARK[o], color=COLOUR[o], s=40)
    ax.set_xticks(range(len(envs)))
    ax.set_xticklabels([e.replace("_c3", "") for e in envs], fontsize=8)
    ax.set_ylabel("min distance, person centre\nto AMR 1 footprint (m)")
    for o in ("pass", "gate", "other"):
        ax.scatter([], [], marker=MARK[o], color=COLOUR[o], label=LABEL[o])
    ax.legend(fontsize=7)
    ax.grid(alpha=0.25)
    ax.set_title("F0  Step 0: no Comfort Space (B1), safety layer only", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return True


def fmt(v, n=2):
    return "-" if v is None else f"{v:.{n}f}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", action="append", required=True, help="batch folder (repeatable)")
    ap.add_argument("--out", help="where to write the report (default: the first --dir)")
    ap.add_argument("--predictions", action="append", default=None,
                    help="predictions file(s); default predictions.json and, if it exists, "
                         "predictions_exec.json")
    args = ap.parse_args()
    dirs = [Path(d) for d in args.dir]
    out = Path(args.out or dirs[0])
    (out / "figures").mkdir(parents=True, exist_ok=True)
    pfiles = [Path(p) for p in (args.predictions or [PRED, PRED.with_name("predictions_exec.json")])
              if Path(p).exists()]
    preds = [json.loads(p.read_text()) for p in pfiles]
    cells = [c for pr in preds for c in pr["cells"]]
    rows = load_rows(dirs)
    # Cells that were run model-free (no prediction file entry): add them
    # with their path bounds computed now, labelled as such -- they are
    # observations, never scored as a pre-registered test.
    known = {(c["env"], c["kind"]) for c in cells}
    from comfort_core.config import load_config
    from comfort_core import feasibility as fz
    for r in rows:
        env = Path(r.get("config", "")).stem
        p = parse_condition(r.get("scenario", ""))
        if not p or not env.startswith("w") or (env, p[0]) in known:
            continue
        known.add((env, p[0]))
        cfg = load_config(common_path=str(PROJECT_ROOT / "configs/stage1a/envs" / f"{env}.yaml"))
        pr = fz.predict(cfg, p[0], "a")
        meta = cfg.get("scenario_meta", {})
        cells.append({"part": "scan", "env": env, "aisle_width_m": meta.get("aisle_width_m"),
                      "human_wall_offset_m": meta.get("human_wall_offset_m"),
                      "context": str(meta.get("context", ""))[:2], "context_label": meta.get("context_label", ""),
                      "kind": p[0], "R_path_infl": {"value": pr["infl"]["value"], "e": pr["infl"]["e"]},
                      "R_path_lethal": {"value": pr["lethal"]["value"], "e": pr["lethal"]["e"]},
                      "trials": []})
    res, step0 = analyse(cells, rows)
    (out / "cells.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    figs = {"F0": fig_step0(step0, out / "figures" / "F0_step0.png"),
            "F1": fig_map(res, out / "figures" / "F1_feasibility_map.png"),
            "F2": fig_pred_exec(res, out / "figures" / "F2_pred_vs_exec.png"),
            "F3": fig_context(res, out / "figures" / "F3_context_mapping.png")}

    r = [f"# Stage 1 controlled: predicted vs observed navigation boundary", "",
         f"Batches: {', '.join(d.name for d in dirs)}. {len(rows)} AMR 1 trials. "
         "Predictions: " + ", ".join(f"`{p.resolve().relative_to(PROJECT_ROOT)}` (commit "
                                     f"{pr.get('commit')})" for p, pr in zip(pfiles, preds))
         + ", each written before the runs it predicts.", "",
         "One AMR, one person, straight corridor, start (-5, 0) -> goal (5, 0), no fleet, "
         "no walkers. e = how far the person's Comfort Space reaches toward the passage, "
         "from the person's centre (the quantity the gate keeps the AMR's nearest surface out of).", "",
         "- R_path^infl: largest e with a robot-centre path when walls/body are kept at the "
         "inflation radius (conservative; R_nav_safe).",
         "- R_path^lethal: the same with the inscribed radius (optimistic upper bound).",
         "- R_exec_model: execution model fitted on the 1A-1 widths only (Comfort Space grown "
         "by inscribed radius + costmap buffer; walls at the fitted wall clearance), "
         "pre-registered for 1A-2 and 1-B. Scored: e <= R_exec_model must pass, "
         "e > R_exec_model + 0.10 must not.",
         "- R_exec: observed boundary, bracket (last e passed by the majority, first e not).", "",
         "## Step 0 (B1, no Comfort Space)", ""]
    if step0:
        r += ["| environment | trials | outcomes | min distance person centre - AMR 1 footprint (m) |",
              "|---|---|---|---|"]
        for env in sorted(step0):
            rs = step0[env]
            r.append(f"| {env} | {len(rs)} | {dict(Counter(mech(x) for x in rs))} | "
                     + ", ".join(fmt(float(x.get('min_clearance') or 'nan') + 0.25) for x in rs) + " |")
    else:
        r.append("not run")
    r += ["", "## Cells", "",
          "| cell | R_path^infl e (param) | R_path^lethal e (param) | R_exec_model e | R_exec bracket e | R_path^infl - R_exec | verdict (scored against) | points e: pass/n (mechanisms) |",
          "|---|---|---|---|---|---|---|---|"]
    for c in res:
        pts = "; ".join(f"{p['e']:.2f}: {p['pass']}/{p['n']}"
                        + (f" ({', '.join(f'{k} {v}' for k, v in p['mechanisms'].items() if k != 'SUCCESS')})"
                           if p["n"] - p["pass"] else "")
                        for p in c["points"] if p["n"])
        r.append(f"| {cell_label(c)} | {c['R_path_infl']['e']:.3f} ({c['R_path_infl']['value']:.2f}) | "
                 f"{c['R_path_lethal']['e']:.3f} ({c['R_path_lethal']['value']:.2f}) | "
                 f"{fmt((c['R_exec_model'] or {}).get('e'), 3)} | "
                 f"{fmt(c['R_exec_bracket'][0])} - {fmt(c['R_exec_bracket'][1])} | "
                 f"{fmt(c['path_miss_m'])} | {c['verdict']} ({c['scored_against']})"
                 + (f"; fails below the bracket at {c['non_monotonic_e']}" if c["non_monotonic_e"] else "")
                 + f" | {pts or '-'} |")
    r += ["", "## Figures", ""]
    for k, f in (("F0", "F0_step0.png"), ("F1", "F1_feasibility_map.png"),
                 ("F2", "F2_pred_vs_exec.png"), ("F3", "F3_context_mapping.png")):
        r.append(f"- {k}: `figures/{f}`" + ("" if figs[k] else " (no data yet)"))
    r += ["", "> Engineering bounds of this navigation stack (footprint, inflation, gate margin). "
          "Not comfort distances. The physical safety layer is identical in every trial."]
    (out / "REPORT.md").write_text("\n".join(r) + "\n", encoding="utf-8")
    print(f"[stage1a_report] {len(rows)} trials -> {out / 'REPORT.md'}")
    for c in res:
        if c["verdict"] != "not run":
            print(f"  {cell_label(c):42s} pred [{c['R_path_infl']['e']:.2f}, {c['R_path_lethal']['e']:.2f}]"
                  f"  exec {fmt(c['R_exec_bracket'][0])}-{fmt(c['R_exec_bracket'][1])}  {c['verdict']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
