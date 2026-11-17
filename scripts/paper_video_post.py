#!/usr/bin/env python3
"""Paper/presentation videos, step 3: cut, overlay, compose, VIDEO_INDEX.md.

    python3 scripts/paper_video_plan.py
    ./scripts/paper_captures.sh --root results/videos/paper_261114/demo_runs --only runs --limit 1500
    python3 scripts/paper_video_post.py

Plain Python 3 + Pillow + the bundled ffmpeg. Demonstration only: the
recorded demo runs apply the FROZEN Stage 1 navigation setting (buffer
0.10 m, identical safety layer and Nav2 parameters for B2 and B3-O) to
the existing cross scenario. They are not Stage 3 and not a performance
result; R_comfort_min / R_pref do not exist yet.

Inputs (read only)
    results/videos/paper_261114/demo_runs/       runs.txt, captures.log, capture_trials/
    results/videos/paper_261114/<cond>/<cfg>/    <STAMP>_camera[_CAMxx].mp4, _costmap.mp4,
                                                 <STAMP>_frames.jsonl (pose log)
    earlier capture runs of today (results/captures/paper_261114) for the
    corridor and buffer-0 diagnostic clips.

Overlays are drawn from the pose log written by the simulation at every
recorded frame (isaac_sim/utils/recorder.py, recording.pose_log): exact
on the orthographic cameras, by pinhole projection of the configured
camera on the perspective ones. Frames are taken at 15 fps on the
simulation clock and resampled (frame repetition) to 30 fps; playback is
simulated real time.

Every output name carries scenario, seed, policy, camera and clean/overlay.
"""

from __future__ import annotations

import glob
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import paper_annotate as ann  # noqa: E402
from paper_capture_post import duration, ffmpeg_path, nearest_after, rel, shown  # noqa: E402
from comfort_core import load_config  # noqa: E402
from comfort_core.labels import robot_label  # noqa: E402

VERSION = "paper_261114"
VROOT = PROJECT_ROOT / "results" / "videos" / VERSION
DEMO = VROOT / "demo_runs"
CAPS = PROJECT_ROOT / "results" / "captures" / VERSION
OUT_FPS = 30
APERTURE_MM = 20.955          # replicator camera default horizontal aperture
AMR_COL = {"AMR 1": (0, 114, 178), "AMR 2": (213, 94, 0), "AMR 3": (0, 158, 115)}
DEMO_NOTE = "System demo: frozen Stage 1 navigation (buffer 0.10 m) on the cross scenario -- not Stage 3"
DIAG_NOTE = "Diagnostic demonstration -- same seed, not a Stage 3 main result"
STATE_TEXT = {"station_engaged": "engaged", "stationary": "standing", "moving": "walking"}

FF = ffmpeg_path()

TOP10 = [
    ("hero/HERO_CROSS_seed002_B3O_CAM05.mp4", "people, Comfort Spaces and AMRs in perspective, no UI",
     "slides: opening; supplementary video"),
    ("overview/CROSS_seed002_B3O_CAM02_overlay.mp4", "the whole system: 8 people, AMR 1/2/3, Comfort Spaces",
     "slides: system overview"),
    ("static_vs_b3o/CROSS_seed002_B2_vs_B3O_CAM03_sidebyside.mp4", "same seed at buffer 0.10: B2 Static held at "
     "the gate, B3-O passed (diagnostic)", "discussion / slides (diagnostic example)"),
    ("multi_amr/CROSS_seed002_B3O_CAM01_overlay.mp4", "three AMRs on their missions: trajectories and targets",
     "slides: multi-AMR operation in a human-shared workspace"),
    ("b3o_closeup/CROSS_seed002_B3O_CAM04_overlay.mp4", "directional Comfort Space; AMR 1 approaches, passes, "
     "leaves", "slides: B3-O concept"),
    ("debug/CROSS_seed002_B3O_CAM03_debug.mp4", "first plan vs executed path around Comfort Spaces and the "
     "0.10 m buffer band", "paper: method; Q&A"),
    ("top/CROSS_seed002_B3O_CAM01_overlay.mp4", "fixed top view with trajectories and Comfort Spaces",
     "paper: scenario figure source; slides"),
    ("comfort_geometry/CROSS_seed002_B3O_CAM03_overlay.mp4", "Comfort geometry per person state",
     "paper: method (Comfort Space)"),
    ("failures/VIDEO_FAIL_comfort_gate_hold_CROSS_seed002_B2_CAM03_overlay.mp4", "comfort gate hold in the cross "
     "scene (B2 Static, buffer 0.10)", "paper/slides: failure mechanism"),
    ("failures/VIDEO_FAIL_protective_stop_static_r1.07_rec_W22.mp4", "ProtectiveStop just above the Stage 1 "
     "boundary (corridor, buffer 0.10)", "paper/slides: failure mechanism"),
]


# ------------------------------------------------------------------ geometry
class View:
    """metres -> pixels for one recording camera."""

    def __init__(self, spec, size=(1920, 1080)):
        self.w, self.h = size
        self.spec = spec
        if spec.get("ortho"):
            o = spec["ortho"]
            self.k = self.w / float(o["width_m"])
            cx, cy = o.get("centre", [0.0, 0.0])
            self.origin = (self.w / 2 - cx * self.k, self.h / 2 + cy * self.k)
        else:
            p = [float(v) for v in spec["position"]]
            la = [float(v) for v in spec["look_at"]]
            f = _norm([la[i] - p[i] for i in range(3)])
            r = _norm(_cross(f, [0.0, 0.0, 1.0]))
            u = _cross(r, f)
            self.p, self.f, self.r, self.u = p, f, r, u
            self.fx = float(spec.get("focal_length", 24.0)) / APERTURE_MM * self.w

    def __call__(self, x, y, z=0.0):
        if self.spec.get("ortho"):
            return self.origin[0] + x * self.k, self.origin[1] - y * self.k
        d = [x - self.p[0], y - self.p[1], z - self.p[2]]
        zc = sum(a * b for a, b in zip(d, self.f))
        if zc <= 0.1:
            return None
        return (self.w / 2 + self.fx * sum(a * b for a, b in zip(d, self.r)) / zc,
                self.h / 2 - self.fx * sum(a * b for a, b in zip(d, self.u)) / zc)


def _norm(v):
    n = math.sqrt(sum(a * a for a in v))
    return [a / n for a in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


# ------------------------------------------------------------------ ffmpeg
def frames_of(src, start=None, end=None):
    d = Path(tempfile.mkdtemp(prefix="pv_", dir=str(PROJECT_ROOT / "results" / "videos" / VERSION)))
    cmd = [FF, "-loglevel", "error", "-y"]
    if start is not None:
        cmd += ["-ss", f"{start:.3f}"]
    cmd += ["-i", str(src)]
    if end is not None:
        cmd += ["-t", f"{end - (start or 0):.3f}"]
    subprocess.run(cmd + ["-q:v", "2", str(d / "%05d.jpg")], check=True)
    return d, sorted(d.glob("*.jpg"))


def encode(frame_dir, out, in_fps=15):
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([FF, "-loglevel", "error", "-y", "-framerate", str(in_fps), "-i", str(frame_dir / "%05d.jpg"),
                    "-vf", f"fps={OUT_FPS},scale=trunc(iw/2)*2:trunc(ih/2)*2", "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "18", str(out)], check=True)


def reencode(src, out, start=None, end=None, vf=None):
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [FF, "-loglevel", "error", "-y"]
    if start is not None:
        cmd += ["-ss", f"{start:.3f}"]
    cmd += ["-i", str(src)]
    if end is not None:
        cmd += ["-t", f"{end - (start or 0):.3f}"]
    filt = f"fps={OUT_FPS}" + (f",{vf}" if vf else "") + ",scale=trunc(iw/2)*2:trunc(ih/2)*2"
    subprocess.run(cmd + ["-vf", filt, "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(out)],
                   check=True)


def stack(srcs, out, titles=None, height=1080):
    """Side by side, each scaled to `height`. A clip that ends earlier (a run
    that stopped on a verdict) holds its last frame until the longest ends."""
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [FF, "-loglevel", "error", "-y"]
    for s in srcs:
        cmd += ["-i", str(s)]
    durs = [duration(FF, s) or 0.0 for s in srcs]
    parts = []
    for i, _ in enumerate(srcs):
        pad = max(durs) - durs[i]
        parts.append(f"[{i}:v]fps={OUT_FPS},scale=-2:{height},setsar=1"
                     + (f",tpad=stop_mode=clone:stop_duration={pad:.2f}" if pad > 0.05 else "") + f"[v{i}]")
    filt = ";".join(parts) + ";" + "".join(f"[v{i}]" for i in range(len(srcs))) + \
        f"hstack=inputs={len(srcs)}:shortest=0[o]"
    subprocess.run(cmd + ["-filter_complex", filt, "-map", "[o]", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                          "-crf", "18", str(out)], check=True)


# ------------------------------------------------------------------ drawing
def caption(d, size, title, sub="", y0=0):
    w = size[0]
    k = w / 1920.0                       # same look on smaller frames (costmap view)
    s1, s2 = int(34 * k), int(22 * k)
    f1, f2 = ann.font(s1, True), ann.font(s2)
    while d.textlength(title, font=f1) > w - 36 and s1 > 12:    # never run off the frame
        s1 -= 1
        f1 = ann.font(s1, True)
    while sub and d.textlength(sub, font=f2) > w - 36 and s2 > 10:
        s2 -= 1
        f2 = ann.font(s2)
    h = int(56 * k) + (int(32 * k) if sub else 0)
    d.rectangle((0, y0, w, y0 + h), fill=(0, 0, 0, 225))
    d.text((int(18 * k), y0 + int(10 * k)), title, font=f1, fill=(255, 255, 255))
    if sub:
        d.text((int(18 * k), y0 + int(54 * k)), sub, font=f2, fill=(225, 225, 225))


def tag(d, xy, text, colour):
    if xy is None:
        return
    f = ann.font(26, True)
    l, t, r, b = d.textbbox((xy[0], xy[1] - 34), text, font=f, anchor="mm")
    d.rounded_rectangle((l - 8, t - 4, r + 8, b + 4), radius=6, fill=colour + (235,))
    d.text((xy[0], xy[1] - 34), text, font=f, fill=(255, 255, 255), anchor="mm")


def overlay_video(src, log, view, out, opts, start=None, end=None):
    """Draw labels/trails/targets/plan from the pose log onto each frame."""
    rows = [json.loads(line) for line in Path(log).read_text().splitlines() if line.strip()]
    d, files = frames_of(src)
    i0 = int(round((start or 0) * 15))
    i1 = len(files) if end is None else min(len(files), int(round(end * 15)))
    keep = files[i0:i1]
    trails = {}
    tmp = Path(tempfile.mkdtemp(prefix="pvo_", dir=str(VROOT)))
    for n, f in enumerate(files[:i1]):
        r = rows[min(n, len(rows) - 1)]
        for name, a in (r.get("amr") or {}).items():
            trails.setdefault(robot_label(name), []).append((a["x"], a["y"]))
        if f not in keep:
            continue
        im = Image.open(f).convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        dr = ImageDraw.Draw(ov)
        if opts.get("plan"):
            pts = [view(x, y) for x, y in opts["plan"]]
            pts = [p for p in pts if p]
            if len(pts) > 1:
                dr.line(pts, fill=(0, 114, 178, 200), width=4)
        if opts.get("trails"):
            for lab, tr in trails.items():
                if opts.get("only") and lab not in opts["only"]:
                    continue
                pts = [p for p in (view(x, y) for x, y in tr[-int(opts["trails"] * 15):]) if p]
                if len(pts) > 1:
                    dr.line(pts, fill=AMR_COL.get(lab, (90, 90, 90)) + (230,), width=5)
        if opts.get("targets"):
            for name, a in (r.get("amr") or {}).items():
                if a.get("target"):
                    lab = robot_label(name)
                    p = view(*a["target"])
                    if p:
                        dr.ellipse((p[0] - 14, p[1] - 14, p[0] + 14, p[1] + 14),
                                   outline=AMR_COL.get(lab, (90, 90, 90)) + (255,), width=5)
                        dr.text((p[0] + 18, p[1] - 12), f"{lab} target", font=ann.font(20, True),
                                fill=AMR_COL.get(lab, (90, 90, 90)) + (255,))
        if opts.get("people"):
            for key, (x, y) in (r.get("people") or {}).items():
                p = view(x, y)
                if p:
                    st = opts["people"].get(key, "")
                    dr.text((p[0] + 16, p[1] + 10), f"{key} {st}".strip(), font=ann.font(20, True),
                            fill=(20, 20, 20, 255), stroke_width=3, stroke_fill=(255, 255, 255, 255))
        if opts.get("labels"):
            for name, a in (r.get("amr") or {}).items():
                lab = robot_label(name)
                if opts.get("only") and lab not in opts["only"]:
                    continue
                tag(dr, view(a["x"], a["y"], 0.6), lab, AMR_COL.get(lab, (90, 90, 90)))
        if opts.get("caption"):
            caption(dr, im.size, opts["caption"][0], opts["caption"][1], opts.get("caption_y", 0))
        Image.alpha_composite(im, ov).convert("RGB").save(tmp / f"{len(list(tmp.iterdir())) + 1:05d}.jpg",
                                                          quality=93)
    encode(tmp, out)
    shutil.rmtree(d, ignore_errors=True)
    shutil.rmtree(tmp, ignore_errors=True)


def caption_video(src, out, title, sub="", start=None, end=None):
    d, files = frames_of(src, start, end)
    for f in files:
        im = Image.open(f).convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        caption(ImageDraw.Draw(ov), im.size, title, sub)
        Image.alpha_composite(im, ov).convert("RGB").save(f, quality=93)
    in_fps = round(len(files) / max(0.1, (end - (start or 0)) if end else duration(FF, src)))
    encode(d, out, in_fps=max(1, in_fps))
    shutil.rmtree(d, ignore_errors=True)


# ------------------------------------------------------------------ runs
def demo_runs(root=DEMO):
    log = root / "captures.log"
    starts = {}
    for line in log.read_text().splitlines() if log.exists() else []:
        m = re.match(r"(\d\d:\d\d:\d\d)\s+run (\S+)", line)
        if m:
            starts[m.group(2)] = m.group(1)
    day = datetime.fromtimestamp(log.stat().st_mtime).strftime("%Y%m%d") if log.exists() else ""
    runs = {}
    for line in (root / "runs.txt").read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        cond, cfgp, tg = line.split()
        if tg not in starts:
            continue
        t0 = datetime.strptime(day + starts[tg], "%Y%m%d%H:%M:%S")
        trial = nearest_after(glob.glob(str(root / "capture_trials" / f"*_{Path(cfgp).stem}.json")), t0,
                              r"(\d{8}T\d{6})", "%Y%m%dT%H%M%S")
        main = nearest_after(glob.glob(str(VROOT / "**" / Path(cfgp).stem / "*_camera.mp4"), recursive=True),
                             t0.replace(second=0), r"(\d{8}_\d{4})_camera", "%Y%m%d_%H%M",
                             must=load_config(scenario=cond)["scenario"]["name"])
        if not (trial and main):
            print(f"  [skip] {tg}: trial {trial} video {main}")
            continue
        stamp = main.name.split("_camera")[0]
        cfg = load_config(scenario=cond, common_path=cfgp)
        vids = {"CAM01_TOP": main}
        for c in cfg["recording"].get("extra_cameras") or []:
            p = main.parent / f"{stamp}_camera_{c['name']}.mp4"
            if p.exists():
                vids[c["name"]] = p
        cams = {"CAM01_TOP": cfg["recording"]["camera"],
                **{c["name"]: c for c in cfg["recording"].get("extra_cameras") or []}}
        if tg.startswith("FAILREPLAY"):
            cams = {"FAILCAM": cfg["recording"]["camera"], "CAM01_TOP": cfg["recording"]["extra_cameras"][0]}
            vids = {"FAILCAM": main, "CAM01_TOP": main.parent / f"{stamp}_camera_CAM01_TOP.mp4"}
        d0 = json.loads(Path(trial).read_text())
        runs[tg] = {"cond": cond, "cfg": cfgp, "config": cfg, "trial": trial, "row": d0["row"],
                    "plan": d0.get("planned_path") or [], "verdict": shown(d0["row"]), "videos": vids,
                    "cams": cams, "log": main.parent / f"{stamp}_frames.jsonl",
                    "costmap": main.parent / f"{stamp}_costmap.mp4"}
        print(f"  {tg}: {runs[tg]['verdict']}  ({rel(trial)})")
    return runs


class Index:
    def __init__(self):
        self.rows = []

    def add(self, path, scenario, seed, policy, camera, buffer, shows, look, kind, use, start=None, end=None):
        path = Path(path)
        if not path.exists():
            print(f"  [skip] {rel(path)}")
            return
        self.rows.append({"file": str(path.relative_to(VROOT)), "scenario": scenario, "seed": seed,
                          "policy": policy, "camera": camera, "duration": f"{duration(FF, path):.1f} s",
                          "buffer": buffer, "shows": shows, "look": look, "kind": kind, "use": use})
        print(f"  wrote {rel(path)}")


def people_states(cfg):
    """Display text per person: the configured state, and that they walk later."""
    out = {}
    for k, v in cfg["humans"].items():
        if isinstance(v, dict) and "position" in v and v.get("enabled", True):
            st = STATE_TEXT.get(str(v.get("state")), str(v.get("state")))
            out[k] = st + (" -> walks" if v.get("walk") else "")
    return out


def interaction_window(log, person_xy=None, before=10.0, after=8.0, total=None):
    """Seconds (video time) around AMR 1's closest approach to a person."""
    rows = [json.loads(line) for line in Path(log).read_text().splitlines() if line.strip()]
    best = None
    for n, r in enumerate(rows):
        a = (r.get("amr") or {}).get("AMR1")
        if not a:
            continue
        for key, (x, y) in (r.get("people") or {}).items():
            if person_xy and math.hypot(x - person_xy[0], y - person_xy[1]) > 1.0:
                continue
            dist = math.hypot(a["x"] - x, a["y"] - y)
            if best is None or dist < best[0]:
                best = (dist, n / 15.0, key)
    if best is None:
        return 0.0, total, None
    t = best[1]
    s = max(0.0, t - before)
    e = t + after if total is None else min(total, t + after)
    if total is not None and e - s < before + after:   # clipped at an end: keep the length
        s = max(0.0, min(s, total - (before + after)))
        e = min(total, s + before + after)
    return s, e, best[2]


# ------------------------------------------------------------------ main
def main() -> int:
    runs = demo_runs()
    idx = Index()
    buf = "0.10"

    def name(seed, pol, cam, look):
        return f"CROSS_seed{seed:03d}_{pol}_{cam}_{look}.mp4"

    # 1/2/4/6/7/8/10 from seed 2 B3-O (overlay run) and its clean twin
    ov, cl = runs.get("CROSS_seed002_B3O_overlay"), runs.get("CROSS_seed002_B3O_clean")
    if ov:
        v = ov["verdict"]
        cap = (f"Cross scenario, seed 2, B3-O (AMR 1: {v})", DEMO_NOTE)
        states = people_states(ov["config"])
        # 1 overview
        if cl:
            reencode(cl["videos"]["CAM02_ISOMETRIC"], VROOT / "overview" / name(2, "B3O", "CAM02", "clean"))
            idx.add(VROOT / "overview" / name(2, "B3O", "CAM02", "clean"), "cross h8 a3", 2, "B3-O", "CAM02_ISOMETRIC",
                    buf, "whole system: 8 people, AMR 1/2/3 missions at once, no overlay", "clean", "system demo",
                    "Slides: system overview; supplementary video")
        overlay_video(ov["videos"]["CAM02_ISOMETRIC"], ov["log"], View(ov["cams"]["CAM02_ISOMETRIC"]),
                      VROOT / "overview" / name(2, "B3O", "CAM02", "overlay"),
                      {"labels": True, "caption": cap})
        idx.add(VROOT / "overview" / name(2, "B3O", "CAM02", "overlay"), "cross h8 a3", 2, "B3-O", "CAM02_ISOMETRIC",
                buf, "whole system with Comfort Spaces and AMR 1/2/3 labels", "overlay", "system demo",
                "Slides: system overview; supplementary video")
        # 2 top view
        overlay_video(ov["videos"]["CAM01_TOP"], ov["log"], View(ov["cams"]["CAM01_TOP"]),
                      VROOT / "top" / name(2, "B3O", "CAM01", "overlay"),
                      {"labels": True, "trails": 120, "caption": cap})
        idx.add(VROOT / "top" / name(2, "B3O", "CAM01", "overlay"), "cross h8 a3", 2, "B3-O", "CAM01_TOP (ortho)",
                buf, "fixed top view: AMR trajectories, people, Comfort Spaces, crossing traffic", "overlay",
                "system demo", "Paper: method figure source; slides: scenario explanation")
        if cl:
            reencode(cl["videos"]["CAM01_TOP"], VROOT / "top" / name(2, "B3O", "CAM01", "clean"))
            idx.add(VROOT / "top" / name(2, "B3O", "CAM01", "clean"), "cross h8 a3", 2, "B3-O", "CAM01_TOP (ortho)",
                    buf, "fixed top view, no Comfort Space, no labels", "clean", "system demo",
                    "Slides: scenario explanation")
        # 4 multi-AMR
        src = (cl or ov)["videos"]["CAM01_TOP"]
        overlay_video(src, (cl or ov)["log"], View((cl or ov)["cams"]["CAM01_TOP"]),
                      VROOT / "multi_amr" / name(2, "B3O", "CAM01", "labels"),
                      {"labels": True})
        idx.add(VROOT / "multi_amr" / name(2, "B3O", "CAM01", "labels"), "cross h8 a3", 2, "B3-O", "CAM01_TOP (ortho)",
                buf, "AMR 1/2/3 on their missions at the same time, small labels only", "clean + labels",
                "system demo", "Slides: multi-AMR operation in a human-shared workspace")
        overlay_video(src, (cl or ov)["log"], View((cl or ov)["cams"]["CAM01_TOP"]),
                      VROOT / "multi_amr" / name(2, "B3O", "CAM01", "overlay"),
                      {"labels": True, "trails": 120, "targets": True,
                       "caption": ("AMR 1 / AMR 2 / AMR 3: trajectories and current mission targets", DEMO_NOTE)})
        idx.add(VROOT / "multi_amr" / name(2, "B3O", "CAM01", "overlay"), "cross h8 a3", 2, "B3-O",
                "CAM01_TOP (ortho)", buf, "trajectories and mission targets of all three AMRs", "overlay",
                "system demo", "Paper/slides: multi-AMR operation")
        # 3 B3-O close-up (around the closest approach to N3, the station in CAM04)
        n3 = ov["config"]["humans"]["N3"]["position"]
        total = duration(FF, ov["videos"]["CAM04_HUMAN_AMR_CLOSEUP"])
        s, e, who = interaction_window(ov["log"], n3, 12.0, 10.0, total)
        overlay_video(ov["videos"]["CAM04_HUMAN_AMR_CLOSEUP"], ov["log"], View(ov["cams"]["CAM04_HUMAN_AMR_CLOSEUP"]),
                      VROOT / "b3o_closeup" / name(2, "B3O", "CAM04", "overlay"),
                      {"labels": True, "only": ["AMR 1"],
                       "caption": (f"B3-O close-up: person {who} ({states.get(who, '')}), AMR 1 approach, pass, leave",
                                   DEMO_NOTE)}, s, e)
        idx.add(VROOT / "b3o_closeup" / name(2, "B3O", "CAM04", "overlay"), "cross h8 a3", 2, "B3-O",
                "CAM04_HUMAN_AMR_CLOSEUP", buf, f"directional Comfort Space of {who} ({states.get(who, '')}); "
                "AMR 1 approaches, passes and leaves", "overlay", "system demo", "Slides: B3-O concept close-up")
        reencode(ov["videos"]["CAM04_HUMAN_AMR_CLOSEUP"], VROOT / "b3o_closeup" / name(2, "B3O", "CAM04", "raw"), s, e)
        idx.add(VROOT / "b3o_closeup" / name(2, "B3O", "CAM04", "raw"), "cross h8 a3", 2, "B3-O",
                "CAM04_HUMAN_AMR_CLOSEUP", buf, "same clip without labels (Comfort Space drawn in the scene)",
                "overlay (in-scene Comfort Space only)", "system demo", "Paper supplementary")
        # 6 Comfort geometry
        overlay_video(ov["videos"]["CAM03_CROSS_CLOSEUP"], ov["log"], View(ov["cams"]["CAM03_CROSS_CLOSEUP"]),
                      VROOT / "comfort_geometry" / name(2, "B3O", "CAM03", "overlay"),
                      {"people": states, "labels": True,
                       "caption": ("B3-O Comfort Space geometry by person state (top, orthographic)", DEMO_NOTE)})
        idx.add(VROOT / "comfort_geometry" / name(2, "B3O", "CAM03", "overlay"), "cross h8 a3", 2, "B3-O",
                "CAM03_CROSS_CLOSEUP (ortho)", buf, "semi-transparent directional Comfort Spaces with each "
                "person's state (engaged / standing / walking)", "overlay", "system demo",
                "Paper: method (Comfort geometry); slides")
        reencode(ov["videos"]["CAM05_LOW_PERSPECTIVE"], VROOT / "comfort_geometry" / name(2, "B3O", "CAM05", "raw"))
        idx.add(VROOT / "comfort_geometry" / name(2, "B3O", "CAM05", "raw"), "cross h8 a3", 2, "B3-O",
                "CAM05_LOW_PERSPECTIVE", buf, "Comfort Spaces in perspective along the north aisle",
                "overlay (in-scene Comfort Space only)", "system demo", "Slides")
        # 7 debug
        overlay_video(ov["videos"]["CAM03_CROSS_CLOSEUP"], ov["log"], View(ov["cams"]["CAM03_CROSS_CLOSEUP"]),
                      VROOT / "debug" / name(2, "B3O", "CAM03", "debug"),
                      {"plan": ov["plan"], "trails": 600, "labels": True, "only": ["AMR 1"],
                       "caption": ("Debug: AMR 1 first global plan (blue) and executed trajectory",
                                   "Comfort Space drawn grown by the AMR half width; pale outer band = + buffer "
                                   "0.10 m | " + DEMO_NOTE)})
        idx.add(VROOT / "debug" / name(2, "B3O", "CAM03", "debug"), "cross h8 a3", 2, "B3-O",
                "CAM03_CROSS_CLOSEUP (ortho)", buf, "AMR 1 only: first planned path, executed trajectory, "
                "Comfort Space and buffer band, footprint in scene", "overlay/debug", "system demo",
                "Paper: method (how Comfort Space shapes the path); Q&A")
        if ov["costmap"].exists():
            caption_video(ov["costmap"], VROOT / "debug" / name(2, "B3O", "COSTMAP", "debug"),
                          "Costmap view (global/local costmap, Comfort layer, path, footprint)",
                          "recorded by the costmap recorder at 5 fps on its own clock; not frame-synchronised "
                          "with the camera views")
            idx.add(VROOT / "debug" / name(2, "B3O", "COSTMAP", "debug"), "cross h8 a3", 2, "B3-O",
                    "costmap recorder", buf, "global/local costmap with the Comfort layer, plan and footprint",
                    "overlay/debug", "system demo", "Q&A; appendix")
        # 8 buffer 0.10 in the cross scene
        overlay_video(ov["videos"]["CAM04_HUMAN_AMR_CLOSEUP"], ov["log"], View(ov["cams"]["CAM04_HUMAN_AMR_CLOSEUP"]),
                      VROOT / "buffer" / name(2, "B3O", "CAM04", "buffer010_overlay"),
                      {"labels": True, "only": ["AMR 1"],
                       "caption": ("Final buffer 0.10 m: pale outer band of each Comfort Space = buffer region "
                                   "(soft cost)", DEMO_NOTE)}, s, e)
        idx.add(VROOT / "buffer" / name(2, "B3O", "CAM04", "buffer010_overlay"), "cross h8 a3", 2, "B3-O",
                "CAM04_HUMAN_AMR_CLOSEUP", buf, "normal passing with the 0.10 m buffer band visible", "overlay",
                "system demo", "Slides: final buffer")
        # 10 hero (clean UI, in-scene Comfort Space)
        total5 = duration(FF, ov["videos"]["CAM05_LOW_PERSPECTIVE"])
        hs, he, _ = interaction_window(ov["log"], None, 14.0, 12.0, total5)  # 26 s
        reencode(ov["videos"]["CAM05_LOW_PERSPECTIVE"], VROOT / "hero" / "HERO_CROSS_seed002_B3O_CAM05.mp4", hs, he)
        idx.add(VROOT / "hero" / "HERO_CROSS_seed002_B3O_CAM05.mp4", "cross h8 a3", 2, "B3-O", "CAM05_LOW_PERSPECTIVE",
                buf, "presentation shot: people, Comfort Spaces, AMR movement, no UI", "clean UI (in-scene Comfort "
                "Space)", "system demo", "Slides: opening; supplementary video")
        reencode(ov["videos"]["CAM02_ISOMETRIC"], VROOT / "hero" / "HERO_CROSS_seed002_B3O_CAM02.mp4", hs, he)
        idx.add(VROOT / "hero" / "HERO_CROSS_seed002_B3O_CAM02.mp4", "cross h8 a3", 2, "B3-O", "CAM02_ISOMETRIC",
                buf, "presentation shot, whole bay, no UI", "clean UI (in-scene Comfort Space)", "system demo",
                "Slides: opening")

    # 5 same seed, Static vs B3-O, frozen buffer 0.10
    for seed in (2, 3):
        b2, b3 = runs.get(f"CROSS_seed{seed:03d}_B2_overlay"), runs.get(f"CROSS_seed{seed:03d}_B3O_overlay")
        if not (b2 and b3):
            continue
        for cam in ("CAM01_TOP", "CAM03_CROSS_CLOSEUP"):
            short = cam.split("_")[0]
            outs = []
            for pol, r in (("B2", b2), ("B3O", b3)):
                o = VROOT / "static_vs_b3o" / name(seed, pol, short, "overlay")
                overlay_video(r["videos"][cam], r["log"], View(r["cams"][cam]), o,
                              {"labels": True, "trails": 600, "only": ["AMR 1"],
                               "caption": (f"seed {seed}  {'B2 Static r 1.20' if pol == 'B2' else 'B3-O target 1.20'}"
                                           f"  -> AMR 1: {r['verdict']}", DIAG_NOTE + " | buffer 0.10")})
                idx.add(o, "cross h8 a3", seed, "B2 Static" if pol == "B2" else "B3-O", cam, buf,
                        f"AMR 1 outcome {r['verdict']} (same seed, scene, start time, people, missions)", "overlay",
                        "diagnostic", "Discussion / slides (diagnostic example)")
                outs.append(o)
            sbs = VROOT / "static_vs_b3o" / f"CROSS_seed{seed:03d}_B2_vs_B3O_{short}_sidebyside.mp4"
            stack(outs, sbs)
            idx.add(sbs, "cross h8 a3", seed, "B2 Static | B3-O", cam, buf,
                    f"left B2 ({b2['verdict']}), right B3-O ({b3['verdict']}); same seed and start", "overlay",
                    "diagnostic", "Discussion / slides (diagnostic example)")

    # 5b the recorded diagnostic pair at buffer 0 (today's capture runs, same camera)
    for seed in (2, 3):
        pair = []
        for pol in ("static", "b3o"):
            vids = sorted(glob.glob(str(VROOT / f"diag_nobuffer_{pol}_r1.20_cross" / f"rec_cross_s{seed}" /
                                        "*_camera.mp4")))
            trials = sorted(glob.glob(str(CAPS / "capture_trials" /
                                          f"*_diag_nobuffer_{pol}_r1.20_rec_cross_s{seed}.json")))
            if vids and trials:
                pair.append((pol, Path(vids[-1]), shown(json.loads(Path(trials[-1]).read_text())["row"])))
        if len(pair) == 2:
            outs = []
            for pol, v, verdict in pair:
                o = VROOT / "static_vs_b3o" / f"CROSS_seed{seed:03d}_{'B2' if pol == 'static' else 'B3O'}_RECDIAG_buffer000.mp4"
                caption_video(v, o, f"seed {seed}  {'B2 Static r 1.20' if pol == 'static' else 'B3-O target 1.20'}"
                              f"  -> AMR 1: {verdict}", "Diagnostic demonstration (costmap buffer 0, the recorded "
                              "diagnostic setting) -- not a Stage 3 main result")
                outs.append(o)
                idx.add(o, "cross h8 a3", seed, "B2 Static" if pol == "static" else "B3-O", "earlier recording camera",
                        "0.00 (diagnostic)", f"AMR 1 outcome {verdict}", "overlay", "diagnostic",
                        "Discussion (the recorded diagnostic case)")
            sbs = VROOT / "static_vs_b3o" / f"CROSS_seed{seed:03d}_B2_vs_B3O_RECDIAG_buffer000_sidebyside.mp4"
            stack(outs, sbs)
            idx.add(sbs, "cross h8 a3", seed, "B2 Static | B3-O", "earlier recording camera", "0.00 (diagnostic)",
                    f"left B2 ({pair[0][2]}), right B3-O ({pair[1][2]}) at buffer 0", "overlay", "diagnostic",
                    "Discussion (the recorded diagnostic case)")

    # 8b buffer comparison in the Stage 1 corridor: the last three W 2.0 capture
    # runs of today, re-recorded in run order buffer 0 / 0.10 / 0.35 with one camera
    vids = sorted(glob.glob(str(VROOT / "static_r0.81" / "rec_W20" / "*_camera.mp4")))[-3:]
    trials = sorted(glob.glob(str(CAPS / "capture_trials" / "*_static_r0.81_rec_W20.json")))[-3:]
    if len(vids) == 3 and len(trials) == 3:
        outs = []
        for v, t, b in zip(vids, trials, ("0.00", "0.10", "0.35")):
            verdict = shown(json.loads(Path(t).read_text())["row"])
            o = VROOT / "buffer" / f"S1_corridor_W20_e0.81_static_buffer{b.replace('.', '')}_REC.mp4"
            caption_video(Path(v), o, f"Stage 1 corridor W 2.0, Static e 0.81, buffer {b} m -> AMR 1: {verdict}",
                          "Capture run (not a measurement); same camera for the three buffers")
            outs.append(o)
            idx.add(o, "Stage 1 corridor", "-", "Static", "corridor recording camera", b,
                    f"buffer {b}: AMR 1 {verdict}", "overlay", "capture run", "Slides: buffer selection")
        sbs = VROOT / "buffer" / "S1_corridor_W20_e0.81_buffer000_010_035_sidebyside.mp4"
        stack(outs, sbs, height=720)
        idx.add(sbs, "Stage 1 corridor", "-", "Static", "corridor recording camera", "0 | 0.10 | 0.35",
                "the three buffers on one condition, side by side", "overlay", "capture run",
                "Slides: buffer comparison on one condition")

    # 9 failures (only verdicts recorded in the trial files)
    fails = []
    for t in sorted(glob.glob(str(CAPS / "capture_trials" / "*.json"))):
        row = json.loads(Path(t).read_text())["row"]
        fails.append((Path(t), row, shown(row)))

    def cap_video_for(trial: Path):
        """The capture video of a capture trial: same capture config, latest
        recording that started before the trial record was written."""
        stamp, rest = trial.stem.split("_", 1)
        scen, cfg_stem = rest.split("_rec_")
        tt = datetime.strptime(stamp[:13], "%Y%m%dT%H%M")
        best = None
        for c in glob.glob(str(VROOT / f"{scen}*" / f"rec_{cfg_stem}" / "*_camera.mp4")):
            tv = datetime.strptime(re.search(r"(\d{8}_\d{4})_camera", c).group(1), "%Y%m%d_%H%M")
            if tv <= tt and (best is None or tv > best[0]):
                best = (tv, Path(c))
        return best[1] if best else None

    # latest first: the 21:10-21:17 runs overlapped with a duplicate capture loop
    picked = {}
    for t, row, v in reversed(fails):
        if v in ("COMFORT_GATE_HOLD", "PROTECTIVE_STOP") and v not in picked:
            vid = cap_video_for(t)
            if vid:
                picked[v] = (t, row, vid)
    for v, (t, row, vid) in picked.items():
        dur = duration(FF, vid)
        b = "0.00 (diagnostic)" if "diag_nobuffer" in t.name else ("0.10" if "rec_W22" in t.name else "see trial")
        o = VROOT / "failures" / f"VIDEO_FAIL_{v.lower()}_{t.stem.split('_', 1)[1]}.mp4"
        caption_video(vid, o, f"Failure mechanism: {v}", f"recorded verdict ({rel(t)}); capture run, not a "
                      "measurement", max(0.0, dur - 14.0), dur)
        idx.add(o, row["scenario"], row.get("seed") or "-", row["policy"], "recording camera", b,
                f"{v}: last ~14 s before the verdict", "overlay", "capture run", "Paper/slides: failure mechanism")
    # a demo run (frozen buffer 0.10) that itself ended in a failure verdict
    for tg, r in runs.items():
        if tg.startswith("FAILREPLAY") or r["verdict"] == "SUCCESS" or "_overlay" not in tg:
            continue
        seed = int(tg.split("seed")[1][:3])
        cam = "CAM03_CROSS_CLOSEUP"
        dur = duration(FF, r["videos"][cam])
        o = VROOT / "failures" / f"VIDEO_FAIL_{r['verdict'].lower()}_CROSS_seed{seed:03d}_B2_CAM03_overlay.mp4"
        overlay_video(r["videos"][cam], r["log"], View(r["cams"][cam]), o,
                      {"labels": True, "trails": 600, "only": ["AMR 1"],
                       "caption": (f"Failure mechanism: {r['verdict']} (cross, seed {seed}, B2 Static r 1.20)",
                                   DIAG_NOTE + " | buffer 0.10")}, max(0.0, dur - 15.0), dur)
        idx.add(o, "cross h8 a3", seed, "B2 Static", cam, "0.10", f"{r['verdict']} in the cross scene: AMR 1 "
                f"held at ({r['row'].get('verdict_x')}, {r['row'].get('verdict_y')}) yielding to "
                f"{r['row'].get('verdict_person')}", "overlay", "diagnostic", "Paper/slides: failure mechanism")
    for tg, r in runs.items():
        if not tg.startswith("FAILREPLAY"):
            continue
        want = tg.split("_", 2)[2]
        if r["verdict"] != want:
            print(f"  [not used] {tg}: re-run gave {r['verdict']}, not {want}")
            continue
        dur = duration(FF, r["videos"]["FAILCAM"])
        o = VROOT / "failures" / f"VIDEO_FAIL_{want.lower()}_{tg.split('_')[1]}_{r['row']['scenario']}.mp4"
        overlay_video(r["videos"]["FAILCAM"], r["log"], View(r["cams"]["FAILCAM"]), o,
                      {"labels": True, "trails": 300, "only": ["AMR 1"],
                       "caption": (f"Failure mechanism: {want}", f"re-recording of size-sweep trial "
                                   f"({r['row']['scenario']}, {tg.split('_')[1]}, buffer 0.35 of that study); "
                                   "verdict reproduced")}, max(0.0, dur - 15.0), dur)
        idx.add(o, "cross h8 a3", int(tg.split("seed")[1][:3]), "B3-O (size sweep)", "failure camera",
                "0.35 (size sweep)", f"{want}, reproduced in the re-recording", "overlay", "capture run",
                "Paper/slides: failure mechanism")

    top_failure_from_demo(idx, runs)
    if (TOP / "runs.txt").exists():
        top_views(idx)
    write_index(idx, runs)
    for d in VROOT.glob("pv*_*"):
        shutil.rmtree(d, ignore_errors=True)
    return 0


TOP = VROOT / "top_runs"
TOP_NOTE = "Capture run, fixed orthographic top view -- not a measurement and not a Stage 3 result"
CORRIDOR_BAND = (180, 820)    # rows of the 13 m-wide corridor top view kept in the stacks
CORRIDOR_CAPTION_Y = 720       # caption below the corridor, clear of the Comfort Space


def vstack(srcs, out, band):
    """Stacked vertically, each cropped to the corridor rows `band`; a shorter
    clip holds its last frame."""
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [FF, "-loglevel", "error", "-y"]
    for s in srcs:
        cmd += ["-i", str(s)]
    durs = [duration(FF, s) or 0.0 for s in srcs]
    y0, y1 = band
    parts = [f"[{i}:v]fps={OUT_FPS},crop=iw:{y1 - y0}:0:{y0},setsar=1"
             + (f",tpad=stop_mode=clone:stop_duration={max(durs) - durs[i]:.2f}" if max(durs) - durs[i] > 0.05
                else "") + f"[v{i}]" for i in range(len(srcs))]
    filt = ";".join(parts) + ";" + "".join(f"[v{i}]" for i in range(len(srcs))) + \
        f"vstack=inputs={len(srcs)}:shortest=0[o]"
    subprocess.run(cmd + ["-filter_complex", filt, "-map", "[o]", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                          "-crf", "18", str(out)], check=True)


def top_views(idx):
    """A fixed top view of every capture-run scenario (top_runs/runs.txt):
    clean + overlay per run, the corridor series stacked, the cross pairs
    side by side, and the top view of each recorded failure."""
    runs = demo_runs(TOP)
    for tg, r in sorted(runs.items()):
        cfg, cam = r["config"], "CAM01_TOP"
        cs = cfg["comfort_space"]
        b = f"{float(cs['costmap']['buffer_width']):.2f}"
        corridor = tg.startswith("TOP_S1_")
        if corridor:
            e = float(cs["static_radius"])
            scen, seed, pol = f"Stage 1 corridor W {float(cfg['corridor']['width']):.1f}", "-", "Static"
            title = f"{scen}, Static e {e:.2f}, buffer {b} m -> AMR 1: {r['verdict']}"
            stem = f"S1_corridor_W{float(cfg['corridor']['width']) * 10:.0f}_static_e{e:.2f}_buffer{b.replace('.', '')}"
        else:
            seed = int(tg.split("_s")[1][:3])
            pol = "B2 Static" if "_B2_" in tg else "B3-O"
            scen = "cross h8 a3"
            title = (f"Cross, seed {seed}, {'B2 Static r 1.20' if pol == 'B2 Static' else 'B3-O target 1.20'}, "
                     f"buffer {b} m -> AMR 1: {r['verdict']}")
            stem = f"CROSS_seed{seed:03d}_{'B2' if pol == 'B2 Static' else 'B3O'}_buffer{b.replace('.', '')}"
        note = TOP_NOTE if corridor else "Diagnostic demonstration (buffer 0, the recorded diagnostic setting) " \
                                         "-- top view, not a Stage 3 main result"
        o = VROOT / "top" / f"{stem}_CAM01_overlay.mp4"
        overlay_video(r["videos"][cam], r["log"], View(r["cams"][cam]), o,
                      {"labels": True, "trails": 600, "people": people_states(cfg),
                       "caption": (title, note), "caption_y": CORRIDOR_CAPTION_Y if corridor else 0})
        idx.add(o, scen, seed, pol, "CAM01_TOP (ortho)", b, f"fixed top view: AMR trajectories, people, Comfort "
                f"Spaces; AMR 1 {r['verdict']}", "overlay", "capture run" if corridor else "diagnostic",
                "Paper: scenario figure source; slides: scenario explanation")
        c = VROOT / "top" / f"{stem}_CAM01_clean.mp4"
        reencode(r["videos"][cam], c)
        idx.add(c, scen, seed, pol, "CAM01_TOP (ortho)", b, "fixed top view, no labels (Comfort Space drawn in "
                "the scene)", "clean", "capture run" if corridor else "diagnostic", "Slides: scenario explanation")
        r["top_overlay"] = o
        if r["verdict"] in ("COMFORT_GATE_HOLD", "PROTECTIVE_STOP"):
            dur = duration(FF, r["videos"][cam])
            f = VROOT / "failures" / f"VIDEO_FAIL_{r['verdict'].lower()}_{stem}_CAM01_TOP.mp4"
            overlay_video(r["videos"][cam], r["log"], View(r["cams"][cam]), f,
                          {"labels": True, "trails": 600, "people": people_states(cfg),
                           "caption": (f"Failure mechanism: {r['verdict']} (top view)", f"{title.split(' -> ')[0]}; "
                                       "capture run, not a measurement"),
                           "caption_y": CORRIDOR_CAPTION_Y if corridor else 0}, max(0.0, dur - 15.0), dur)
            idx.add(f, scen, seed, pol, "CAM01_TOP (ortho)", b, f"{r['verdict']}: last ~15 s before the verdict, "
                    "from above", "overlay", "capture run" if corridor else "diagnostic",
                    "Paper/slides: failure mechanism")
    # corridor series, one above the other
    for group, tags, what in (
            ("S1_corridor_W22_static_e042_102_107_buffer010", ["TOP_S1_E_small_W22_e0.42", "TOP_S1_E_lastpass_W22_e1.02",
                                                               "TOP_S1_E_firstfail_W22_e1.07"],
             "Static e 0.42 / 1.02 / 1.07 at buffer 0.10 (around the Stage 1 boundary)"),
            ("S1_corridor_W20_e0.81_buffer000_010_035", ["TOP_S1_buffer_000_W20_e0.81", "TOP_S1_buffer_010_W20_e0.81",
                                                         "TOP_S1_buffer_035_W20_e0.81"],
             "one condition, buffer 0 / 0.10 / 0.35")):
        if all(t in runs for t in tags):
            o = VROOT / "top" / f"{group}_CAM01_stack.mp4"
            vstack([runs[t]["top_overlay"] for t in tags], o, CORRIDOR_BAND)
            idx.add(o, "Stage 1 corridor", "-", "Static", "CAM01_TOP (ortho)", "see rows",
                    f"{what}, top to bottom: " + " / ".join(runs[t]["verdict"] for t in tags), "overlay",
                    "capture run", "Slides: Stage 1 boundary / buffer comparison")
    for seed in (2, 3):
        pair = [runs.get(f"TOP_CROSS_s{seed:03d}_{p}_diagnostic") for p in ("B2", "B3O")]
        if all(pair):
            o = VROOT / "top" / f"CROSS_seed{seed:03d}_B2_vs_B3O_buffer000_CAM01_sidebyside.mp4"
            stack([p["top_overlay"] for p in pair], o)
            idx.add(o, "cross h8 a3", seed, "B2 Static | B3-O", "CAM01_TOP (ortho)", "0.00 (diagnostic)",
                    f"left B2 ({pair[0]['verdict']}), right B3-O ({pair[1]['verdict']}); same seed and start",
                    "overlay", "diagnostic", "Discussion (the recorded diagnostic case)")
    return runs


def top_failure_from_demo(idx, runs):
    """The top view of the demo-run failure (B2, frozen buffer 0.10)."""
    for tg, r in runs.items():
        if tg.startswith("FAILREPLAY") or r["verdict"] == "SUCCESS" or "_overlay" not in tg:
            continue
        seed = int(tg.split("seed")[1][:3])
        dur = duration(FF, r["videos"]["CAM01_TOP"])
        o = VROOT / "failures" / f"VIDEO_FAIL_{r['verdict'].lower()}_CROSS_seed{seed:03d}_B2_CAM01_TOP.mp4"
        overlay_video(r["videos"]["CAM01_TOP"], r["log"], View(r["cams"]["CAM01_TOP"]), o,
                      {"labels": True, "trails": 600, "people": people_states(r["config"]),
                       "caption": (f"Failure mechanism: {r['verdict']} (cross, seed {seed}, B2 Static r 1.20, top view)",
                                   DIAG_NOTE + " | buffer 0.10")}, max(0.0, dur - 15.0), dur)
        idx.add(o, "cross h8 a3", seed, "B2 Static", "CAM01_TOP (ortho)", "0.10", f"{r['verdict']} from above",
                "overlay", "diagnostic", "Paper/slides: failure mechanism")


def top_count():
    log = TOP / "captures.log"
    return sum(1 for line in log.read_text().splitlines() if re.match(r"\d\d:\d\d:\d\d\s+run ", line)) \
        if log.exists() else 0


def write_index(idx, runs):
    rows = sorted(idx.rows, key=lambda r: r["file"])
    n_clean = sum(1 for r in rows if r["look"].startswith("clean"))
    log = DEMO / "captures.log"
    executed = sum(1 for line in log.read_text().splitlines() if re.match(r"\d\d:\d\d:\d\d\s+run ", line)) \
        if log.exists() else len(runs)
    lines = [
        f"# Video index ({VERSION})", "",
        "> These videos are a demonstration of the navigation-side configuration frozen in Stage 1 "
        "(Comfort costmap buffer 0.10 m; identical safety layer, collision monitor and Nav2 parameters for "
        "B2 and B3-O) applied to the existing complex cross scenario (8 people, AMR 1/2/3). They are **not** "
        "results of the Stage 3 main final performance experiment, which needs the Stage 2 human study "
        "(R_comfort_min, R_pref are still null). Comfort sizes are the ones of the existing diagnostic pair "
        "(Static r 1.20 / B3-O target 1.20), not Dynamic Selection.", "",
        "Frames are taken at 15 fps on the simulation clock and resampled to 30 fps (simulated real time), "
        "1920x1080 (side-by-sides wider). Overlays are drawn from the per-frame pose log of the same run.", "",
        f"Total {len(rows)} videos: clean {n_clean}, overlay {len(rows) - n_clean}. "
        f"Demo/capture runs executed: {executed} in results/videos/{VERSION}/demo_runs (not a results batch); "
        f"{len(runs)} used (a re-recorded tag supersedes its earlier run). Also reused: today's capture runs "
        "under results/captures/paper_261114/capture_trials (corridor buffer comparison, corridor ProtectiveStop, "
        "cross buffer-0 diagnostic pairs).", "",
        "Top views: every scenario has a fixed orthographic top view (CAM01_TOP; corridor 13 m wide). The "
        "corridor and buffer-0 cross capture runs, first recorded with one perspective camera, were re-recorded "
        f"with the top camera in results/videos/{VERSION}/top_runs ({top_count()} runs; same condition and base "
        "files, capture runs, not a results batch). A re-recording's outcome is its own and can differ from the "
        "earlier capture run; the caption of each clip shows the verdict of the run it shows.", "",
        "Demo runs:", ""] + [f"- `{t}`: AMR 1 {r['verdict']} (`{rel(Path(r['trial']))}`)" for t, r in sorted(runs.items())] + [
        "", "Camera poses: `configs/captures/paper_261114/video/cameras.yaml`. Regenerate: "
        "`python3 scripts/paper_video_plan.py`, `./scripts/paper_captures.sh --root "
        f"results/videos/{VERSION}/demo_runs --only runs --limit 1500`, `python3 scripts/paper_video_post.py`.", "",
        "## Recommended (top 10)", ""] + [f"{i}. `results/videos/{VERSION}/{f}` -- {what} ({use})"
                                          for i, (f, what, use) in enumerate(TOP10, 1) if (VROOT / f).exists()] + [
        "", "## All videos", "",
        "| file | scenario | seed | policy | camera | duration | buffer (m) | what it demonstrates | clean/overlay "
        "| kind | recommended use |", "|" + "---|" * 11]
    for r in rows:
        lines.append("| " + " | ".join(str(r[k]).replace("|", "/") for k in
                                        ("file", "scenario", "seed", "policy", "camera", "duration", "buffer", "shows",
                                         "look", "kind", "use")) + " |")
    (VROOT / "VIDEO_INDEX.md").write_text("\n".join(lines) + "\n")
    (VROOT / "video_index.json").write_text(json.dumps(rows, indent=1))
    print(f"[paper_video_post] {len(rows)} videos (clean {n_clean})")


if __name__ == "__main__":
    if "--index-only" in sys.argv:          # rewrite VIDEO_INDEX.md from video_index.json
        ix = Index()
        ix.rows = json.loads((VROOT / "video_index.json").read_text())
        write_index(ix, demo_runs())
        sys.exit(0)
    if "--top-only" in sys.argv:            # add the top views to the existing index
        ix = Index()
        tops = {r["file"] for r in json.loads((VROOT / "video_index.json").read_text())
                if r["file"].startswith("top/") or r["file"].endswith("_CAM01_TOP.mp4")}
        ix.rows = [r for r in json.loads((VROOT / "video_index.json").read_text()) if r["file"] not in tops
                   or r["file"].startswith("top/CROSS_seed002_B3O_CAM01")]
        dr = demo_runs()
        top_failure_from_demo(ix, dr)
        top_views(ix)
        write_index(ix, dr)
        for d in VROOT.glob("pv*_*"):
            shutil.rmtree(d, ignore_errors=True)
        sys.exit(0)
    sys.exit(main())
