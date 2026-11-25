#!/usr/bin/env python3
"""Final Stage 1-3 videos: rule-based selection, overlays, comparisons, indexes.

    python3 scripts/final_videos.py stage1            # per-size sets + comparisons
    python3 scripts/final_videos.py stage3            # per-policy matched seeds + side-by-sides
    python3 scripts/final_videos.py smoke             # pipeline check on the smoke batch

Plain Python 3 + Pillow + the bundled ffmpeg. Every clip is cut from the
recording of THAT trial (results/batches/.../trials/<trial_id>/<stamp>_camera.mp4,
fixed orthographic top view CAM01_TOP); nothing is re-simulated, so the
verdict on screen is the verdict in the trial's CSV/DONE. Overlays are drawn
from the trial's own trial_log.jsonl (5 Hz) and pose log.

Selection per condition (fixed rules, in this order, no duplicates):
    1 typical_success   success trial whose AMR 1 travel time is closest to the
                        condition's median among successes
    2 low_yield_success success trial with the least standing time (3 AMRs)
    3 high_yield        success trial with the most standing time (TEMPORARY_HOLD first)
    4 comfort_failure   comfort-induced deadlock (lowest seed); else any failure;
                        else none and the index says NO DEADLOCK OBSERVED IN THIS CONDITION
    5 overlap           trial with the largest Comfort Space overlap on the floor
    then filled to 5 with further successes by seed.

Outputs (per batch): videos/<group>/<label>/*.mp4 (overlay + clean), VIDEO_INDEX.md
per folder, video_metadata.json (experiment_id, trial_id, seed, policy,
comfort_size, camera, human_shift_pattern, system_success, amr1/a1/a2 result,
deadlock_type, video_path, trial_log_path, selection reason).
"""

from __future__ import annotations

import argparse
import glob
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import paper_annotate as ann  # noqa: E402
from paper_capture_post import duration, ffmpeg_path  # noqa: E402
from comfort_core.final_metrics import COMFORT_CLASSES, SUCCESS_CLASSES, analyse_trial, config  # noqa: E402

FF = ffmpeg_path()
B = PROJECT_ROOT / "results" / "batches"
W, H = 1920, 1080
TOP = {"centre": (0.0, 0.0), "width_m": 30.0}
K = W / TOP["width_m"]
AMR_COL = {"AMR1": (0, 114, 178), "a1": (213, 94, 0), "a2": (0, 158, 115)}
STATE_TXT = {"moving": "Moving", "stationary": "Stationary", "station_engaged": "Station Engaged"}
FPS_IN = 10


def px(x, y):
    return W / 2 + (x - TOP["centre"][0]) * K, H / 2 - (y - TOP["centre"][1]) * K


def find_video(tdir: Path):
    v = sorted(tdir.glob("*_camera.mp4"))
    f = sorted(tdir.glob("*_frames.jsonl"))
    return (v[-1] if v else None), (f[-1] if f else None)


def load_log(tdir: Path):
    ticks, plans = [], []
    for line in (tdir / "trial_log.jsonl").read_text().splitlines():
        try:
            o = json.loads(line)
        except ValueError:
            continue
        if o.get("ev") == "tick":
            ticks.append(o)
        elif o.get("ev") == "plan" and o.get("t") is not None:
            plans.append(o)
    return ticks, plans


def nearest(ticks, t):
    lo, hi = 0, len(ticks) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if ticks[mid]["t"] < t:
            lo = mid + 1
        else:
            hi = mid
    return ticks[lo] if ticks else None


def last_plan(plans, name, t):
    best = None
    for p in plans:
        if p["amr"] == name and p["t"] <= t:
            best = p
    return best


def caption(d, title, sub, y0=0):
    f1, f2 = ann.font(30, True), ann.font(21)
    while d.textlength(title, font=f1) > W - 40:
        f1 = ann.font(f1.size - 1, True)
    while d.textlength(sub, font=f2) > W - 40:
        f2 = ann.font(f2.size - 1)
    d.rectangle((0, y0, W, y0 + 84), fill=(0, 0, 0, 220))
    d.text((18, y0 + 8), title, font=f1, fill=(255, 255, 255))
    d.text((18, y0 + 50), sub, font=f2, fill=(225, 225, 225))


def render(tdir: Path, rec, out: Path, title: str, sub: str, start=None, end=None):
    """Overlay video of one trial from its own recording and logs."""
    video, poses = find_video(tdir)
    ticks, plans = load_log(tdir)
    frames = [json.loads(l) for l in poses.read_text().splitlines() if l.strip()]
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix="fv_", dir=str(out.parent)))
    cmd = [FF, "-loglevel", "error", "-y"]
    if start is not None:
        cmd += ["-ss", f"{start:.2f}"]
    cmd += ["-i", str(video)]
    if end is not None:
        cmd += ["-t", f"{end - (start or 0):.2f}"]
    subprocess.run(cmd + ["-q:v", "2", str(tmp / "%05d.jpg")], check=True)
    files = sorted(tmp.glob("*.jpg"))
    i0 = int(round((start or 0) * FPS_IN))
    verdicts = {n: a["class"] for n, a in rec["amrs"].items()}
    tverd = {n: a["t_verdict"] for n, a in rec["amrs"].items()}
    trails = {n: [] for n in rec["amrs"]}
    for fi, tk in enumerate(frames[:i0]):
        for n, a in (tk.get("amr") or {}).items():
            if n in trails:
                trails[n].append(px(a["x"], a["y"]))
    for k, f in enumerate(files):
        fr = frames[min(i0 + k, len(frames) - 1)]
        t = fr["sim_time"]
        tk = nearest(ticks, t)
        im = Image.open(f).convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        for n, a in (fr.get("amr") or {}).items():
            if n in trails:
                trails[n].append(px(a["x"], a["y"]))
        for n in rec["amrs"]:
            col = AMR_COL.get(n, (90, 90, 90))
            pl = last_plan(plans, n, t)
            if pl and t <= tverd[n] + 0.5 and len(pl["pts"]) > 1:
                d.line([px(x, y) for x, y in pl["pts"]], fill=col + (150,), width=3)
                gx, gy = px(*pl["pts"][-1])
                d.ellipse((gx - 13, gy - 13, gx + 13, gy + 13), outline=col + (255,), width=4)
                d.text((gx + 16, gy - 12), f"{n} goal", font=ann.font(18, True), fill=col + (255,),
                       stroke_width=2, stroke_fill=(255, 255, 255, 255))
            if len(trails[n]) > 1:
                d.line(trails[n], fill=col + (235,), width=5)
        people = (tk or {}).get("people") or {}
        for key, p in people.items():
            x, y = px(p["x"], p["y"])
            st = STATE_TXT.get(p.get("o") or p.get("s"), p.get("o") or "")
            size = f"  F{p['F']:.2f}/S{p['S']:.2f}/R{p['R']:.2f}" if "F" in p else ""
            d.text((x + 14, y + 8), f"{key} {st}{size}", font=ann.font(17, True), fill=(20, 20, 20, 255),
                   stroke_width=3, stroke_fill=(255, 255, 255, 255))
        for n, a in ((tk or {}).get("amr") or {}).items():
            if n not in rec["amrs"]:
                continue
            x, y = px(a[0], a[1])
            if t > tverd[n]:
                status = verdicts[n]
            elif n == "AMR1":
                status = (tk.get("gate") or "")
            else:
                status = "STOP (safety)" if (tk.get("cm") or {}).get(n) == 1 else ""
            label = f"{n}" + (f"  {status}" if status and status != "DRIVING" else "")
            f_ = ann.font(20, True)
            l_, t_, r_, b_ = d.textbbox((x, y - 34), label, font=f_, anchor="mm")
            d.rounded_rectangle((l_ - 7, t_ - 4, r_ + 7, b_ + 4), radius=6, fill=AMR_COL.get(n, (90, 90, 90)) + (235,))
            d.text((x, y - 34), label, font=f_, fill=(255, 255, 255), anchor="mm")
        caption(d, title, f"{sub}   |   sim t = {t:6.1f} s")
        d.rectangle((0, H - 46, W, H), fill=(0, 0, 0, 200))
        d.text((18, H - 38), "verdict: " + "   ".join(f"{n} {verdicts[n]}" for n in rec["amrs"]),
               font=ann.font(22, True), fill=(255, 255, 255))
        Image.alpha_composite(im, ov).convert("RGB").save(f, quality=92)
    subprocess.run([FF, "-loglevel", "error", "-y", "-framerate", str(FPS_IN), "-i", str(tmp / "%05d.jpg"),
                    "-vf", "fps=30", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(out)], check=True)
    shutil.rmtree(tmp, ignore_errors=True)


def clean(tdir: Path, out: Path):
    video, _ = find_video(tdir)
    out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(video, out)


def grid(srcs, labels, out: Path, cols=3, tile_w=960):
    """Tiles in a grid, each scaled to tile_w; a shorter clip holds its last frame."""
    out.parent.mkdir(parents=True, exist_ok=True)
    tile_h = tile_w * 9 // 16
    durs = [duration(FF, s) or 0.0 for s in srcs]
    cmd = [FF, "-loglevel", "error", "-y"]
    for s in srcs:
        cmd += ["-i", str(s)]
    parts = []
    for i in range(len(srcs)):
        pad = max(durs) - durs[i]
        parts.append(f"[{i}:v]fps=30,scale={tile_w}:{tile_h},setsar=1"
                     + (f",tpad=stop_mode=clone:stop_duration={pad:.2f}" if pad > 0.05 else "") + f"[v{i}]")
    rows = (len(srcs) + cols - 1) // cols
    while len(parts) < rows * cols:                       # blank tiles
        j = len(parts)
        parts.append(f"color=c=black:s={tile_w}x{tile_h}:d={max(durs):.2f}:r=30[v{j}]")
    layout = "|".join(f"{(i % cols) * tile_w}_{(i // cols) * tile_h}" for i in range(rows * cols))
    filt = ";".join(parts) + ";" + "".join(f"[v{i}]" for i in range(rows * cols)) + \
        f"xstack=inputs={rows * cols}:layout={layout}[o]"
    subprocess.run(cmd + ["-filter_complex", filt, "-map", "[o]", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                          "-crf", "21", str(out)], check=True)


# ------------------------------------------------------------------ selection
def records(dirs):
    out = []
    for d in dirs:
        for t in sorted((d / "trials").glob("*/DONE")):
            cache = d.parent / "analysis" / "trials" / f"{t.parent.name}.json"
            if cache.exists() and cache.stat().st_mtime >= t.stat().st_mtime:
                r = json.loads(cache.read_text())
            else:
                r = analyse_trial(t.parent)
            r["tdir"] = t.parent
            if not r.get("infra_failure") and find_video(t.parent)[0]:
                out.append(r)
    return out


def hold(r):
    return sum((a.get("hold_s") or 0) for a in r["amrs"].values())


def select(rs):
    picked, why = [], {}

    def take(r, reason):
        if r is not None and r["trial_id"] not in why:
            picked.append(r)
            why[r["trial_id"]] = reason

    succ = [r for r in rs if r["system_success"]]
    if succ:
        tt = sorted(r["amrs"]["AMR1"]["travel_time"] or 0 for r in succ)
        medv = tt[len(tt) // 2]
        take(min(succ, key=lambda r: (abs((r["amrs"]["AMR1"]["travel_time"] or 0) - medv), r["seed"])),
             "typical success (AMR 1 travel time nearest the median)")
        take(min(succ, key=lambda r: (hold(r), r["seed"])), "success with the least standing time")
        th = [r for r in succ if any(a["class"] == "TEMPORARY_HOLD" for a in r["amrs"].values())] or succ
        take(max(th, key=lambda r: (hold(r), -r["seed"])), "success with the most standing time (high yield / hold)")
    comfort = sorted([r for r in rs if r["comfort_deadlock"]], key=lambda r: r["seed"])
    fail = sorted([r for r in rs if not r["system_success"]], key=lambda r: r["seed"])
    note = None
    if comfort:
        take(comfort[0], "comfort-induced deadlock")
    elif fail:
        take(fail[0], "failure (not comfort-induced)")
        note = "NO COMFORT-INDUCED DEADLOCK OBSERVED IN THIS CONDITION"
    else:
        note = "NO DEADLOCK OBSERVED IN THIS CONDITION"
    ov = [r for r in rs if (r["spatial"].get("overlap_max_m2") or 0) > 0]
    if ov:
        take(max(ov, key=lambda r: (r["spatial"]["overlap_max_m2"], -r["seed"])),
             "largest Comfort Space overlap (multi-human)")
    for r in sorted(succ, key=lambda r: r["seed"]):
        if len(picked) >= 5:
            break
        take(r, "further success (fill to 5, lowest seed)")
    return picked, why, note


def meta_row(r, exp, video, reason, size, policy):
    cfg = config(*_cond_cfg(r))
    shifts = (cfg.get("scenario_meta") or {}).get("exchanges") or cfg.get("exchanges")
    return {"experiment_id": exp, "trial_id": r["trial_id"], "seed": r["seed"], "policy": policy,
            "comfort_size": size, "camera": "CAM01_TOP (orthographic, 30 m)", "human_shift_pattern": shifts,
            "system_success": r["system_success"],
            "amr1_result": r["amrs"]["AMR1"]["class"], "a1_result": r["amrs"].get("a1", {}).get("class"),
            "a2_result": r["amrs"].get("a2", {}).get("class"),
            "deadlock_type": sorted({a["class"] for a in r["amrs"].values() if a["class"] not in SUCCESS_CLASSES}),
            "video_path": str(video.relative_to(PROJECT_ROOT)),
            "trial_log_path": str((r["tdir"] / "trial_log.jsonl").relative_to(PROJECT_ROOT)),
            "trial_result_path": str((r["tdir"] / "DONE").relative_to(PROJECT_ROOT)), "selection_reason": reason}


def _cond_cfg(r):
    done = json.loads((r["tdir"] / "DONE").read_text())
    return done["condition"], done["config"]


def write_index(folder: Path, rows, title, note=None):
    folder.mkdir(parents=True, exist_ok=True)
    L = [f"# {title}", ""]
    if note:
        L += [f"**{note}**", ""]
    L += ["| file | seed | condition | policy | verdict (AMR1 / a1 / a2) | system | selection reason | trial result |",
          "|---|---|---|---|---|---|---|---|"]
    for m in rows:
        L.append(f"| {Path(m['video_path']).name} | {m['seed']} | {m.get('condition', '')} | {m['policy']} | "
                 f"{m['amr1_result']} / {m['a1_result']} / {m['a2_result']} | "
                 f"{'SUCCESS' if m['system_success'] else 'FAIL'} | {m['selection_reason']} | "
                 f"`{m['trial_result_path']}` |")
    (folder / "VIDEO_INDEX.md").write_text("\n".join(L) + "\n")


def policy_of(cond):
    if cond == "B1":
        return "B1", None
    if cond.startswith("B2"):
        return "B2", int(cond.split("_r")[1]) / 100.0
    return "B3-O", cond.split("_", 1)[1]


def condition_sets(rs, vroot: Path, exp: str, group_of):
    meta = []
    by = {}
    for r in rs:
        by.setdefault(r["condition"], []).append(r)
    for cond, group in sorted(by.items()):
        picked, why, note = select(group)
        pol, size = policy_of(cond)
        folder = vroot / group_of(cond)
        rows = []
        for r in picked:
            stem = f"{r['trial_id']}"
            sz = f"r {size:.2f} m" if isinstance(size, float) else (size or "no Comfort Space")
            title = f"{exp}  seed {r['seed']}  {pol} {sz}  ->  system {'SUCCESS' if r['system_success'] else 'FAIL'}"
            o = folder / f"{stem}_overlay.mp4"
            if not o.exists():
                render(r["tdir"], r, o, title, f"trial {r['trial_id']}  |  {why[r['trial_id']]}")
            c = folder / f"{stem}_clean.mp4"
            if not c.exists():
                clean(r["tdir"], c)
            m = meta_row(r, exp, o, why[r["trial_id"]], size, pol)
            m["condition"] = cond
            m["clean_video_path"] = str(c.relative_to(PROJECT_ROOT))
            rows.append(m)
        write_index(folder, rows, f"{exp} — {cond}", note)
        meta += rows
    return meta


def stage1():
    S1 = B / "261125_final_stage1"
    rs = records([d for d in (S1 / "coarse", S1 / "fine") if (d / "trials").exists()])
    vroot = S1 / "videos"
    phase_of = {r["condition"]: r["tdir"].parent.parent.name for r in rs}

    def group(cond):
        if cond == "B1":
            return "coarse/B1"
        return f"{phase_of[cond]}/size_{cond.split('_r')[1]}"
    meta = condition_sets(rs, vroot, "Stage 1", group)
    # same-seed size comparison: seeds whose class sequence changes across sizes
    seq = {}
    for r in rs:
        seq.setdefault(r["seed"], {})[r["condition"]] = r
    coarse = ["B1"] + [f"B2_r{s}" for s in ("080", "100", "120", "140", "160")]
    changing = [s for s, v in sorted(seq.items()) if all(c in v for c in coarse)
                and len({v[c]["system_success"] for c in coarse}) > 1]
    comp = []
    for s in changing[:3]:
        srcs, labels = [], []
        for c in coarse:
            r = seq[s][c]
            o = vroot / "comparisons" / "tiles" / f"{r['trial_id']}_overlay.mp4"
            if not o.exists():
                pol, size = policy_of(c)
                render(r["tdir"], r, o, f"seed {s}  {pol}{'' if size is None else f' r {size:.2f} m'}",
                       f"system {'SUCCESS' if r['system_success'] else 'FAIL'}")
            srcs.append(o)
            labels.append(c)
        out = vroot / "comparisons" / f"seed{s:03d}_size_comparison_B1_080_100_120_140_160.mp4"
        if not out.exists():
            grid(srcs, labels, out)
        comp.append({"file": str(out.relative_to(PROJECT_ROOT)), "seed": s,
                     "conditions": coarse, "system": [seq[s][c]["system_success"] for c in coarse]})
    # upper-bound comparison: one seed, the sizes just below / at / just above R_nav_upper
    extra = []
    ubf = S1 / "analysis" / "stage1_upper_bound.json"
    if ubf.exists():
        up = json.loads(ubf.read_text())["decision"]["R_nav_upper_m"]
        sizes = sorted({int(c.split("_r")[1]) for c in phase_of if c.startswith("B2")})
        if up is not None:
            u = int(round(up * 100))
            below = max([x for x in sizes if x < u], default=None)
            above = min([x for x in sizes if x > u], default=None)
            trio = [f"B2_r{x:03d}" for x in (below, u, above) if x is not None]
            # a seed where the trio's outcome changes, else the lowest seed that has all three
            cands = [sd for sd, v in sorted(seq.items()) if all(c in v for c in trio)]
            pick = next((sd for sd in cands if len({seq[sd][c]["system_success"] for c in trio}) > 1),
                        cands[0] if cands else None)
            if pick is not None:
                srcs = []
                for c in trio:
                    r = seq[pick][c]
                    o = vroot / "comparisons" / "tiles" / f"{r['trial_id']}_overlay.mp4"
                    if not o.exists():
                        render(r["tdir"], r, o, f"seed {pick}  B2 r {int(c.split('_r')[1]) / 100:.2f} m"
                               f"{'  (R_nav_upper)' if int(c.split('_r')[1]) == u else ''}",
                               f"system {'SUCCESS' if r['system_success'] else 'FAIL'}")
                    srcs.append(o)
                out = vroot / "comparisons" / f"seed{pick:03d}_upper_bound_{'_'.join(c.split('_r')[1] for c in trio)}.mp4"
                if not out.exists():
                    grid(srcs, trio, out, cols=len(srcs))
                extra.append({"file": str(out.relative_to(PROJECT_ROOT)), "kind": "upper_bound_comparison",
                              "seed": pick, "conditions": trio, "R_nav_upper_m": up,
                              "system": [seq[pick][c]["system_success"] for c in trio]})
    # comfort-overlap deadlock examples (as recorded)
    ovl = sorted([r for r in rs if any(a["class"] == "COMFORT_OVERLAP_BLOCK" for a in r["amrs"].values())],
                 key=lambda r: (-(r["spatial"].get("overlap_max_m2") or 0), r["seed"]))
    for r in ovl[:3]:
        o = vroot / "overlap_cases" / f"{r['trial_id']}_overlap_deadlock.mp4"
        if not o.exists():
            pol, size = policy_of(r["condition"])
            who = "; ".join(f"{n}: {a['class_detail']}" for n, a in r["amrs"].items()
                            if a["class"] == "COMFORT_OVERLAP_BLOCK")
            render(r["tdir"], r, o, f"Comfort overlap deadlock  seed {r['seed']}  {pol} r {size:.2f} m", who)
        m = meta_row(r, "Stage 1", o, "COMFORT_OVERLAP_BLOCK example (largest overlap first)", *policy_of(r["condition"])[::-1])
        m["condition"] = r["condition"]
        extra.append(m)
    if not ovl:
        extra.append({"kind": "overlap_cases", "note": "NO COMFORT_OVERLAP_BLOCK OBSERVED"})
    write_index(vroot / "overlap_cases", [m for m in extra if "trial_id" in m], "Stage 1 — Comfort overlap deadlock",
                None if ovl else "NO COMFORT_OVERLAP_BLOCK OBSERVED")
    (vroot / "video_metadata.json").write_text(json.dumps({"condition_videos": meta, "comparisons": comp,
                                                           "extra": extra}, indent=1, default=str))
    print(f"stage1: {len(meta)} condition videos, {len(comp)} comparisons, {len(extra)} extra")


def stage3():
    S3 = B / "261125_final_stage3"
    rs = records([d for d in (S3 / "main", S3 / "sensitivity") if (d / "trials").exists()])
    vroot = S3 / "videos"
    meta = condition_sets(rs, vroot, "Stage 3",
                          lambda c: {"B1": "B1"}.get(c, "B2" if c.startswith("B2") else c.replace("B3O_MID", "B3O")))
    seq = {}
    for r in rs:
        seq.setdefault(r["seed"], {})[policy_of(r["condition"])[0] if not r["condition"].startswith("B3O_")
                                      or r["condition"] == "B3O_MID" else r["condition"]] = r
    full = {sd: v for sd, v in seq.items() if all(k in v for k in ("B1", "B2", "B3-O"))}

    def moving_close(r):
        xs = [e["min_clearance"] for e in r["interactions"] if e["state"] == "moving"]
        return min(xs) if xs else 9.0
    cases = []
    A = [sd for sd, v in sorted(full.items()) if v["B2"]["comfort_deadlock"] and v["B3-O"]["system_success"]]
    if A:
        cases.append(("A", A[0], "B2 comfort-induced deadlock, B3-O success"))
    Bc = [(hold(v["B2"]) - hold(v["B3-O"]), sd) for sd, v in full.items()
          if v["B2"]["system_success"] and v["B3-O"]["system_success"] and hold(v["B2"]) > hold(v["B3-O"])]
    if Bc:
        cases.append(("B", max(Bc)[1], "B2 long standing / yield, B3-O success with less"))
    C = [sd for sd, v in sorted(full.items()) if v["B2"]["system_success"] and v["B3-O"]["system_success"]]
    if C:
        cases.append(("C", C[len(C) // 2], "both succeed"))
    D = [(v["B2"]["spatial"].get("overlap_max_m2") or 0, sd) for sd, v in full.items()]
    if D and max(D)[0] > 0:
        cases.append(("D", max(D)[1], "largest multi-human Comfort Space overlap (B2)"))
    E = [(moving_close(v["B3-O"]), sd) for sd, v in full.items()]
    if E and min(E)[0] < 9.0:
        cases.append(("E", min(E)[1], "closest moving-human crossing (B3-O)"))
    used, chosen = set(), []
    for c, sd, why in cases:
        if sd not in used:
            chosen.append((c, sd, why))
            used.add(sd)
    for sd in sorted(full):
        if len(chosen) >= 5:
            break
        if sd not in used:
            chosen.append(("fill", sd, "matched seed (fill to 5, lowest seed)"))
            used.add(sd)
    side = []
    for c, sd, why in chosen:
        srcs = []
        for pol in ("B1", "B2", "B3-O"):
            r = full[sd][pol]
            o = vroot / "matched_seed" / f"seed{sd:03d}_{pol.replace('-', '')}_overlay.mp4"
            if not o.exists():
                render(r["tdir"], r, o, f"Stage 3  seed {sd}  {pol}  ->  system "
                       f"{'SUCCESS' if r['system_success'] else 'FAIL'}", f"case {c}: {why}")
            srcs.append(o)
            m = meta_row(r, "Stage 3", o, f"matched seed, case {c}: {why}", None, pol)
            m["condition"] = r["condition"]
            side.append(m)
        out3 = vroot / "matched_seed" / f"seed{sd:03d}_B1_B2_B3O_grid.mp4"
        if not out3.exists():
            grid(srcs, ["B1", "B2", "B3-O"], out3, cols=3)
        out2 = vroot / "side_by_side" / f"seed{sd:03d}_case{c}_B2_vs_B3O.mp4"
        if not out2.exists():
            grid(srcs[1:], ["B2", "B3-O"], out2, cols=2, tile_w=1280)
        side.append({"file": str(out2.relative_to(PROJECT_ROOT)), "kind": "B2_vs_B3O", "case": c, "seed": sd,
                     "why": why, "B2": full[sd]["B2"]["system_success"], "B3-O": full[sd]["B3-O"]["system_success"]})
    write_index(vroot / "matched_seed", [m for m in side if "trial_id" in m], "Stage 3 — matched seeds (B1 / B2 / B3-O)")
    found = {c for c, _, _ in cases}
    missing = [c for c in "ABCDE" if c not in found]
    (vroot / "side_by_side").mkdir(parents=True, exist_ok=True)
    (vroot / "side_by_side" / "VIDEO_INDEX.md").write_text(
        "# Stage 3 — B2 vs B3-O side by side (same seed)\n\n"
        + "".join(f"- `{Path(m['file']).name}`: case {m['case']} — {m['why']} (B2 "
                  f"{'SUCCESS' if m['B2'] else 'FAIL'}, B3-O {'SUCCESS' if m['B3-O'] else 'FAIL'})\n"
                  for m in side if m.get("kind") == "B2_vs_B3O")
        + (f"\nCases not present in the results (not made): {', '.join(missing)}\n" if missing else ""))
    (vroot / "video_metadata.json").write_text(json.dumps({"condition_videos": meta, "matched": side,
                                                           "cases_missing": missing}, indent=1, default=str))
    print(f"stage3: {len(meta)} condition videos, {len(chosen)} matched seeds, missing cases {missing}")


def smoke():
    S = B / "261125_final_smoke"
    rs = records([S])
    meta = condition_sets(rs, S / "videos", "Smoke", lambda c: c)
    (S / "videos" / "video_metadata.json").write_text(json.dumps(meta, indent=1, default=str))
    print(f"smoke: {len(meta)} videos")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=["stage1", "stage3", "smoke"])
    args = ap.parse_args()
    {"stage1": stage1, "smoke": smoke, "stage3": stage3}[args.what]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
