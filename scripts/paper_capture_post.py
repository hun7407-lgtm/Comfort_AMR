#!/usr/bin/env python3
"""Paper captures, step 3: frames, annotations, composites, CAPTURE_INDEX.md.

    python3 scripts/paper_capture_plan.py --final-variant buf10   # 1. jobs
    ./scripts/paper_captures.sh                                   # 2. Isaac stills + recorded runs
    python3 scripts/paper_capture_post.py                         # 3. this

Plain Python 3 (Pillow, matplotlib, the bundled ffmpeg). Visual
documentation only: nothing here is a measurement and nothing is written
outside results/captures/<version>/. Every number drawn is read from a
config, a sidecar JSON written by isaac_sim/scripts/paper_shots.py, a
trial record or a Stage 1 model file -- never typed.

    raw/             Isaac stills (paper_shots.py), untouched
    frames/          frames cut from the capture-run videos (camera, costmap)
    annotated/       raw image + labels (W, e, AMR names, verdict banners)
    trial_overlays/  top views drawn from trial records (paper_trial_overlay.py)
    composite/       panels of images that share camera, zoom and resolution
    gallery/         failure-mechanism gallery
    hero/            copies of the stills proposed for title slides
    CAPTURE_INDEX.md one row per image

Existing results are read, never written. Capture-run trials live in
<root>/capture_trials and are not a results batch.
"""

from __future__ import annotations

import argparse
import glob
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

import paper_annotate as ann  # noqa: E402
from comfort_core import feasibility as fz  # noqa: E402
from comfort_core.config import load_config  # noqa: E402
from comfort_core.failure_class import verdict  # noqa: E402
from comfort_core.size_sweep import parse_condition  # noqa: E402

VERSION = "paper_261114"
CONTEXTS = {"c1": "C1 engaged-rear", "c2": "C2 stationary-rear", "c3": "C3 stationary-front",
            "c4": "C4 stationary-side"}
VERDICT_TEXT = {"SUCCESS": "SUCCESS", "COMFORT_GATE_HOLD": "Comfort Gate Hold",
                "PROTECTIVE_STOP": "ProtectiveStop (collision monitor)", "POST_EVADE_STALL": "Post-evade stall",
                "NAV2_FAILURE": "Nav2 failure (abort)", "ROBOT_ROBOT_DEADLOCK": "Robot-robot deadlock",
                "PASSAGE_INFEASIBLE": "Passage infeasible", "INVALID": "Invalid trial"}
DIAG = "DIAGNOSTIC pilot (costmap buffer 0) -- not a Stage 3 main result"

# Existing trial records used as examples (read only; verdicts re-checked on load).
EXISTING = {
    "protective_stop_035": "results/batches/261114_stage1a/refine/20261114T022226_static_r0.82_w2.20_y0.35_c3.json",
    "success_000": "results/batches/261114_stage1a/diag_nobuffer/"
                   "20261114T030725_diag_nobuffer_static_r0.82_w2.20_y0.35_c3.json",
    "gate_hold_000": "results/batches/261114_stage1a_nobuf/lowe/20261114T055337_static_r0.81_w2.00_y0.35_c3.json",
    "post_evade_stall": "results/batches/261112_stage1_size_sweep/"
                        "20261112T050609_b3o_r1.40_campaign_h8_a3_cross_s5.json",
    "nav2_failure": "results/batches/261112_stage1_size_sweep/20261112T030059_b3o_r0.80_campaign_h8_a3_cross_s3.json",
}
# An earlier capture run kept as the gate-hold example (G): buffer 0 at W 2.0,
# e 0.81 held at the gate in this run and passed in the re-recorded one
# (buffer 0 grazes the gate boundary; it varies run to run). Recorded with
# the earlier, lower corridor camera.
EXTRA_RUNS = {
    "S1_G_gatehold_buffer_000_W20_e0.81_earliercam": {
        "scenario": "configs/stage1a/conditions_nobuf/static_r0.81.yaml",
        "config": "configs/captures/paper_261114/rec_W20.yaml",
        "trial": "results/captures/paper_261114/capture_trials/20261114T212347_static_r0.81_rec_W20.json",
        "camera": "results/videos/paper_261114/static_r0.81/rec_W20/20261114_2122_camera.mp4",
        "costmap": "results/videos/paper_261114/static_r0.81/rec_W20/20261114_2122_costmap.mp4"},
}
EXISTING_FRAME_PES = ("results/notion/261109/raw/frames/"
                      "b3d_state_directional__capture_cross_s5_zoom__20261109_2236_camera_036.0.png")


def shown(row) -> str:
    """The verdict a capture is labelled with. comfort_core.failure_class.verdict
    first; a Stage 1 row has no selection fields, so a recorded
    comfort_gate_hold comes back UNCLASSIFIED there -- for a caption the
    recorded mechanism is used (the classifier is not changed)."""
    v = verdict(row)
    if v == "UNCLASSIFIED" and row.get("deadlock_mechanism") == "comfort_gate_hold":
        return "COMFORT_GATE_HOLD"
    return v


TOP10 = [
    ("composite/S1_contexts_4panel_b3o_topzoom.png", "B3-O C1-C4, same target size, same camera (method)"),
    ("composite/S1_contexts_4panel_e_annotated.png", "the e axis: one target size, four corridor-side extents"),
    ("annotated/S1_e_definition_static.png", "definition of e on the Static Comfort Space"),
    ("trial_overlays/S1_Rpath_Rexec_explanation.png", "R_path^infl / R_path^lethal / R_exec model vs observed pass-fail"),
    ("trial_overlays/S1_buffer_000_vs_035_W22_e0.82_existing.png", "why the buffer was re-selected: 0.35 pushes AMR 1 to the wall"),
    ("composite/S1_PASS_FAIL_3panel_trajectories.png", "pass far below / just below / fail just above the boundary"),
    ("composite/S1_environment_Wseries_top_annotated.png", "Stage 1 environment factor W"),
    ("composite/CROSS_seed002_B2_vs_B3O_4panel.png", "same seed, B2 hold vs B3-O passage (diagnostic)"),
    ("annotated/CROSS_seed002_missions.png", "cross scene: 8 people, AMR 1/2/3 missions"),
    ("hero/HERO_S1_person_comfort_amr_C1.png", "title image: person, Comfort Space, AMR 1"),
]


class Post:
    def __init__(self, root: Path):
        self.root = root
        self.d = {k: root / k for k in ("raw", "frames", "annotated", "trial_overlays", "composite",
                                        "gallery", "hero")}
        for p in self.d.values():
            p.mkdir(parents=True, exist_ok=True)
        self.index = []
        self.plan = json.loads((root / "plan.json").read_text())
        self.fbuf = float(self.plan["final_buffer_m"])
        self.ffmpeg = ffmpeg_path()

    # ------------------------------------------------------------ bookkeeping
    def rec(self, path, kind, stage, scenario, policy="-", seed="-", buffer="-", context="-", e="-",
            view="-", shows="", use=""):
        path = Path(path)
        if not path.exists():
            print(f"  [skip] {rel(path)} not written")
            return
        self.index.append({"file": rel(path), "kind": kind, "stage": stage, "scenario": scenario,
                           "policy": policy, "seed": seed, "buffer": buffer, "context": context,
                           "e": e, "view": view, "shows": shows, "use": use})

    def raw_meta(self, png: Path) -> dict:
        m = json.loads(png.with_suffix(".json").read_text())
        info = {"policy": m.get("policy", "-"), "buffer": f"{m.get('buffer_width', 0):.2f}",
                "view": m["view"]["name"] + (" (ortho)" if m["view"].get("ortho") else " (persp)")}
        env = Path(m["config"]).stem
        info["scenario"] = env
        mc = re.search(r"_(c[1-4])$", env)
        info["context"] = CONTEXTS.get(mc.group(1), "-") if mc else "-"
        kv = parse_condition(m.get("scenario") or "")
        if kv and m.get("comfort_drawn") and env.startswith("w"):
            cfg = load_config(scenario=m["scenario"], common_path=m["config"])
            info["e"] = f"{fz.extent_of(cfg, 'static' if kv[0] == 'static' else 'b3o', kv[1], 'a'):.2f}"
            info["scenario"] += f" + {kv[0]}_r{kv[1]:.2f}"
        else:
            info["e"] = "-"
        if not m.get("comfort_drawn"):
            info["policy"] = "none drawn"
            info["buffer"] = "-"
        ms = re.search(r"cross_s(\d+)", m["config"])
        info["seed"] = str(int(ms.group(1))) if ms else "-"
        return info

    # ---------------------------------------------------------------- stills
    def stills(self):
        raw = self.d["raw"]
        for png in sorted(raw.glob("*.png")):
            if not png.with_suffix(".json").exists():
                continue
            i = self.raw_meta(png)
            name = png.stem
            stage = "Cross scene (pilot illustration)" if name.startswith("CROSS") else "Stage 1 (controlled)"
            shows, use = describe_still(name)
            self.rec(png, "clean" if "clean" in name or "human_only" in name else "overlay", stage,
                     i["scenario"], i["policy"], i["seed"], i["buffer"], i["context"], i["e"], i["view"],
                     shows, use)

        A = self.d["annotated"]
        C = self.d["composite"]
        # A. corridor width on every environment plan; W series
        ws = ("15", "18", "22", "26")
        for w in ws:
            src = raw / f"S1_env_W{w}_clean_top.png"
            if src.exists():
                out = A / f"S1_environment_W{w}_annotated.png"
                ann.width(src, out)
                self.rec(out, "annotated", "Stage 1 (controlled)", f"w{int(w) / 10:.2f}_y0.35_c3", "-",
                         view="top (ortho)", shows=f"corridor width W = {int(w) / 10:.1f} m marked",
                         use="Method: Stage 1 environment / width factor")
        for view in ("top", "iso", "close"):
            srcs = [raw / f"S1_env_W{w}_clean_{view}.png" for w in ws]
            self.grid(C / f"S1_environment_Wseries_{view}_clean.png", srcs, 2,
                      [f"W {int(w) / 10:.1f} m" for w in ws], "Stage 1 (controlled)",
                      "W 1.5/1.8/2.2/2.6, person 0.35 m from wall", view,
                      "same camera for the four widths", "Method: environment factor W")
        self.grid(C / "S1_environment_Wseries_top_annotated.png",
                  [A / f"S1_environment_W{w}_annotated.png" for w in ws], 2,
                  [f"W {int(w) / 10:.1f} m" for w in ws], "Stage 1 (controlled)", "W series", "top (ortho)",
                  "four widths, width marked", "Method: environment factor W", kind="annotated")

        # B. Static Comfort Space
        tags = ("human_only", "static", "static_amr")
        for view in ("topzoom", "hero", "iso", "close", "top"):
            self.grid(C / f"S1_static_W22_sequence_{view}.png", [raw / f"S1_static_W22_{t}_{view}.png" for t in tags],
                      3, ["person only", "+ Static Comfort Space", "+ AMR 1"], "Stage 1 (controlled)",
                      "w2.20_y0.35_c3 + static", view, "human only / + Comfort Space / + AMR",
                      "Method: what the Comfort Space is")
        # D. e definition on the Static and every context
        for src, sc in ([(raw / "S1_static_W22_static_topzoom.png", "static")]
                        + [(raw / f"S1_context_{c}_b3o_noamr_topzoom.png", c) for c in ("C1", "C2", "C3", "C4")]):
            if src.exists():
                out = A / f"S1_e_definition_{sc}.png"
                r = ann.extent(src, out)
                i = self.raw_meta(src)
                self.rec(out, "annotated", "Stage 1 (controlled)", i["scenario"], i["policy"], buffer=i["buffer"],
                         context=i["context"], e=f"{r['e']:.2f}", view="topzoom (ortho)",
                         shows="person centre, Comfort Space boundary, corridor-side effective extent e "
                               f"vs {'target_size_m' if r['kind'] == 'b3o' else 'radius'} {r['target_or_radius']:.2f}",
                         use="Method: definition of e (Stage 1 x axis)")
        # C. contexts
        cs = ("C1", "C2", "C3", "C4")
        for view in ("top", "topzoom", "iso", "close", "hero"):
            for tag in ("b3o", "b3o_noamr"):
                self.grid(C / f"S1_contexts_4panel_{tag}_{view}.png",
                          [raw / f"S1_context_{c}_{tag}_{view}.png" for c in cs], 2,
                          [CONTEXTS[c.lower()] for c in cs], "Stage 1 (controlled)",
                          "w2.20_y0.35_c1..c4 + b3o_r1.20", view,
                          "B3-O C1-C4, same target_size 1.20 m, person, AMR and camera",
                          "Method: B3-O context-dependent shapes")
        self.grid(C / "S1_contexts_4panel_e_annotated.png", [A / f"S1_e_definition_{c}.png" for c in cs], 2,
                  [CONTEXTS[c.lower()] for c in cs], "Stage 1 (controlled)", "contexts, e marked",
                  "topzoom (ortho)", "C1-C4 with corridor-side e", "Method: one target size, four e",
                  kind="annotated")
        # F (still). three buffers on one scene
        b = int(round(self.fbuf * 100))
        btags = ("buf000", f"buf{b:03d}", "buf035")
        btit = ["buffer 0", f"buffer {self.fbuf:.2f} (final)", "buffer 0.35 (earlier default)"]
        for view in ("top", "topzoom", "iso"):
            self.grid(C / f"S1_buffer_3panel_still_{view}.png",
                      [raw / f"S1_buffer_still_W20_{t}_{view}.png" for t in btags], 3, btit,
                      "Stage 1 (controlled)", "w2.00_y0.35_c3 + static e 0.81", view,
                      "Comfort Space drawn with three costmap buffers (outer band)",
                      "Results: buffer selection")
        # K / L. cross
        for src in (raw / "CROSS_s002_overview_b3o_top.png", raw / "CROSS_s002_overview_clean_top.png",
                    raw / "CROSS_s002_overview_b3o_crossing.png"):
            if src.exists():
                out = A / (src.stem + "_robots.png")
                ann.robots(src, out)
                self.rec(out, "annotated", "Cross scene (pilot illustration)", "campaign_h8_a3_cross_s2",
                         seed="2", view=src.stem.rsplit("_", 1)[-1], shows="AMR 1/2/3 labelled",
                         use="Method: cross scene overview")
        bg = raw / "CROSS_s002_overview_clean_top.png"
        if bg.exists():
            out = A / "CROSS_seed002_missions.png"
            subprocess.run([sys.executable, "scripts/scenario_figure.py", "--config",
                            "configs/campaigns/cross/campaign_h8_a3_cross_s2.yaml", "--background", str(bg),
                            "--out", str(out)], cwd=PROJECT_ROOT, check=False)
            self.rec(out, "annotated", "Cross scene (pilot illustration)", "campaign_h8_a3_cross_s2", seed="2",
                     view="top (ortho)", shows="8 people (roles) and AMR 1/2/3 missions as designed",
                     use="Method: cross scenario")
        for view in ("top", "crossing", "iso_se", "north_aisle"):
            out = C / f"CROSS_seed002_Static_vs_B3O_{view}.png"
            self.grid(out, [raw / f"CROSS_s002_static_b2static_{view}.png",
                            raw / f"CROSS_s002_overview_b3o_{view}.png"], 2,
                      ["Static r 1.20 (illustration)", "B3-O target 1.20 (illustration)"],
                      "Cross scene (pilot illustration)", "campaign_h8_a3_cross_s2", view,
                      "same scene and camera, the two Comfort Space policies drawn (initial configs, "
                      f"buffer {self.fbuf:.2f}); pilot illustration, no outcome",
                      "Discussion: policy difference in the shared scene", banner="pilot illustration")

    # ------------------------------------------------------------- capture runs
    def runs(self):
        log = (self.root / "captures.log").read_text().splitlines() if (self.root / "captures.log").exists() else []
        starts = {}
        for line in log:
            m = re.match(r"(\d\d:\d\d:\d\d)\s+run (\S+)", line)
            if m:
                starts[m.group(2)] = m.group(1)
        out = {}
        for line in (self.root / "runs.txt").read_text().splitlines():
            if not line.strip() or line.startswith("#"):
                continue
            scen, cfgp, tag = line.split()
            if tag not in starts:
                print(f"  [skip] run {tag}: not in captures.log")
                continue
            t0 = datetime.strptime(self.plan_date() + starts[tag], "%Y%m%d%H:%M:%S")
            # runs are sequential: the first record of this capture config after
            # the run started is its own (the scenario name in the file may be
            # shortened, e.g. a trailing "_cross" is dropped)
            trial = nearest_after(glob.glob(str(self.root / "capture_trials" / f"*_{Path(cfgp).stem}.json")),
                                  t0, r"(\d{8}T\d{6})", "%Y%m%dT%H%M%S")
            cam = nearest_after(glob.glob(str(PROJECT_ROOT / "results/videos" / VERSION / "**" / Path(cfgp).stem
                                               / "*_camera.mp4"), recursive=True),
                                t0.replace(second=0), r"(\d{8}_\d{4})_camera", "%Y%m%d_%H%M",
                                must=Path(scen).stem)
            cmap = Path(str(cam).replace("_camera.mp4", "_costmap.mp4")) if cam else None
            out[tag] = {"scenario": scen, "config": cfgp, "trial": trial, "camera": cam,
                        "costmap": cmap if cmap and cmap.exists() else None}
            print(f"  run {tag}: trial {rel(trial) if trial else None}  video {rel(cam) if cam else None}")
        for tag, r in EXTRA_RUNS.items():
            r = {k: (PROJECT_ROOT / v if k in ("trial", "camera", "costmap") else v) for k, v in r.items()}
            if all(Path(r[k]).exists() for k in ("trial", "camera", "costmap")):
                out[tag] = r
        return out

    def plan_date(self):
        return datetime.fromtimestamp((self.root / "captures.log").stat().st_mtime).strftime("%Y%m%d")

    def frames(self, tag, r, fracs=(0.15, 0.45, 0.7)):
        F = self.d["frames"]
        got = {}
        for src, lab in ((r["camera"], "cam"), (r["costmap"], "costmap")):
            if not src:
                continue
            dur = duration(self.ffmpeg, src)
            if not dur:
                continue
            ts = [round(dur * f, 1) for f in fracs] + [max(0.0, round(dur - 1.0, 1))]
            for t, name in zip(ts, [f"t{t:05.1f}" for t in ts[:-1]] + ["final"]):
                out = F / f"{tag}_{lab}_{name}.png"
                cut(self.ffmpeg, src, t, out)
                got[(lab, name)] = (out, t)
        return got

    def run_products(self, runs):
        F, A, C, D, G = (self.d[k] for k in ("frames", "annotated", "composite", "trial_overlays", "gallery"))
        info = {}
        for tag, r in runs.items():
            row = json.loads(Path(r["trial"]).read_text())["row"] if r["trial"] else None
            v = shown(row) if row else "?"
            cross = tag.startswith("CROSS")
            seed = re.search(r"s(\d+)", tag).group(1).lstrip("0") if cross else "-"
            pol = row["policy"] if row else "-"
            cond = r["scenario"]
            cfg = load_config(scenario=cond, common_path=r["config"])
            buf = float((cfg["comfort_space"].get("costmap") or {}).get("buffer_width", 0) or 0)
            kv = parse_condition(cond)
            e = (f"{fz.extent_of(cfg, 'static' if kv[0] == 'static' else 'b3o', kv[1], 'a'):.2f}"
                 if kv and not cross else "-")
            ctx = "-" if cross else CONTEXTS.get(re.search(r"_(c[1-4])", load_config(common_path=r["config"])
                                                          ["scenario_meta"]["scenario_id"]).group(1), "-")
            stage = "Cross scene (diagnostic pilot, buffer 0)" if cross else "Stage 1 capture run (final buffer)"
            if "buffer_0" in tag:
                stage = "Stage 1 capture run (buffer comparison)"
            scen = f"{Path(r['config']).stem} + {Path(cond).stem}"
            got = self.frames(tag, r)
            info[tag] = {"verdict": v, "frames": got, "row": row, "buf": buf, "e": e, "cond": cond, "seed": seed}
            for (lab, name), (p, t) in got.items():
                self.rec(p, "clean" if lab == "cam" else "debug", stage, scen, pol, seed, f"{buf:.2f}", ctx, e,
                         f"{'recording camera' if lab == 'cam' else 'costmap view (RViz-style)'} t={t:.1f}s",
                         f"{'frame' if lab == 'cam' else 'costmap frame'} of the capture run; trial verdict "
                         f"{VERDICT_TEXT.get(v, v)}", "Results/slides: " + ("failure mechanism" if v != "SUCCESS"
                                                                            else "successful passage"))
            fin = got.get(("cam", "final"))
            if fin:
                out = A / f"{tag}_{v}.png"
                ann.banner(fin[0], out, f"AMR 1: {VERDICT_TEXT.get(v, v)}",
                           (DIAG + f" | seed {seed} | {pol}") if cross else
                           f"W {cfg['corridor']['width']:.1f} m  |  {pol}  |  e {e} m  |  costmap buffer {buf:.2f} m")
                self.rec(out, "annotated", stage, scen, pol, seed, f"{buf:.2f}", ctx, e, "recording camera, final",
                         f"final frame with the trial verdict ({VERDICT_TEXT.get(v, v)})",
                         "Results: outcome example")
            if not cross and r["trial"]:
                out = D / f"{tag}_trial_overlay.png"
                overlay(r["trial"], cond, out, f"capture run {tag}")
                self.rec(out, "overlay", stage, scen, pol, "-", f"{buf:.2f}", ctx, e, "top (data)",
                         "walls, inflation band, Comfort Space, gate boundary, buffer region, first plan, "
                         "trajectory, final footprint (from the trial record)", "Results: mechanism explanation")
        return info

    # -------------------------------------------------- run-based composites
    def run_composites(self, info, runs):
        C, A = self.d["composite"], self.d["annotated"]
        b = int(round(self.fbuf * 100))
        # E. pass far below / just below / fail just above (final buffer)
        e_tags = [t for t in info if t.startswith("S1_E_")]
        e_tags.sort(key=lambda t: ("small" not in t, "lastpass" not in t))
        if len(e_tags) == 3:
            titles = [f"{lab}: e {info[t]['e']} m -> {VERDICT_TEXT.get(info[t]['verdict'])}"
                      for lab, t in zip(("far below", "just below", "just above"), e_tags)]
            self.grid(C / "S1_PASS_FAIL_3panel_camera.png", [info[t]["frames"][("cam", "final")][0]
                                                               for t in e_tags], 3, titles,
                      "Stage 1 capture run (final buffer)", "w2.20_y0.35_c3 + static", "recording camera, final",
                      "pass far below / pass just below / fail just above the observed boundary",
                      "Results: Stage 1 boundary", buffer=f"{self.fbuf:.2f}")
            out = C / "S1_PASS_FAIL_3panel_trajectories.png"
            overlay_many([runs[t]["trial"] for t in e_tags], [runs[t]["scenario"] for t in e_tags], out, titles,
                         "W 2.2 m, Static, final buffer: three e values (capture runs)")
            self.rec(out, "composite", "Stage 1 capture run (final buffer)", "w2.20_y0.35_c3 + static",
                     "static", buffer=f"{self.fbuf:.2f}", view="top (data)",
                     shows="three trajectories over walls, Comfort Space, gate and buffer regions",
                     use="Results: Stage 1 boundary")
        # F / G / H / I. three buffers on one condition
        f_tags = ["S1_buffer_000_W20_e0.81", f"S1_buffer_{b:03d}_W20_e0.81", "S1_buffer_035_W20_e0.81"]
        if all(t in info for t in f_tags):
            titles = [f"buffer {info[t]['buf']:.2f} m: {VERDICT_TEXT.get(info[t]['verdict'])}" for t in f_tags]
            for lab in ("cam", "costmap"):
                for tagn in ("final", "mid"):
                    # runs differ in length, so "mid" is each run's own middle frame
                    keys = [sorted(k[1] for k in info[t]["frames"] if k[0] == lab and k[1] != "final")
                            for t in f_tags]
                    if tagn == "mid" and not all(len(k) > 1 for k in keys):
                        continue
                    ims = [info[t]["frames"].get((lab, "final" if tagn == "final" else k[1]))
                           for t, k in zip(f_tags, keys)]
                    if not all(ims):
                        continue
                    self.grid(C / f"S1_buffer_3panel_{lab}_{tagn}.png", [p for p, _ in ims], 3, titles,
                              "Stage 1 capture run (buffer comparison)", "w2.00_y0.35_c3 + static e 0.81",
                              ("recording camera" if lab == "cam" else "costmap view") + f", {tagn}",
                              "same condition, buffer 0 / final / 0.35", "Results: buffer selection",
                              kind="composite" if lab == "cam" else "debug")
            out = C / "S1_buffer_3panel_trajectories.png"
            overlay_many([runs[t]["trial"] for t in f_tags], [runs[t]["scenario"] for t in f_tags], out, titles,
                         "W 2.0 m, Static e 0.81: costmap buffer 0 / final / 0.35 (capture runs)")
            self.rec(out, "composite", "Stage 1 capture run (buffer comparison)", "w2.00_y0.35_c3 + static e 0.81",
                     "static", view="top (data)", shows="Comfort Space, buffer region, plan, wall, footprint for "
                     "the three buffers", use="Results: buffer selection")
        # M. same seed, B2 hold vs B3-O (diagnostic)
        for seed in ("002", "003"):
            b2, b3 = f"CROSS_s{seed}_B2_diagnostic", f"CROSS_s{seed}_B3O_diagnostic"
            if b2 not in info or b3 not in info:
                continue
            fb2, fb3 = info[b2]["frames"], info[b3]["frames"]
            mids = sorted(k[1] for k in fb2 if k[0] == "cam" and k[1] != "final")
            if not mids:
                continue
            before = mids[-1]
            # B3-O at the very simulated time of the B2 "before" frame (both
            # recordings start at the agreed start, so video times line up)
            t_before = fb2[("cam", before)][1]
            same = self.d["frames"] / f"{b3}_cam_t{t_before:05.1f}.png"
            cut(self.ffmpeg, runs[b3]["camera"], t_before, same)
            self.rec(same, "clean", "Cross scene (diagnostic pilot, buffer 0)", f"campaign_h8_a3_cross_s{int(seed)}",
                     "b3o", str(int(seed)), "0.00", view=f"recording camera t={t_before:.1f}s",
                     shows="B3-O at the time of the B2 'before' frame", use="Discussion (diagnostic)")
            pick = [fb2[("cam", before)], fb2[("cam", "final")], (same, t_before), fb3[("cam", "final")]]
            names = ["B2_before", "B2_failure", "B3O_same_time", "B3O_passage"]
            for (p, t), n in zip(pick, names):
                dst = self.d["annotated"] / f"CROSS_seed{seed}_{n}.png"
                pol = "B2 Static r1.20" if n.startswith("B2") else "B3-O target 1.20"
                ann.banner(p, dst, f"seed {int(seed)}  {pol}  t = {t:.1f} s",
                           DIAG + f" | final verdict {VERDICT_TEXT.get(info[b2 if n.startswith('B2') else b3]['verdict'])}")
                self.rec(dst, "annotated", "Cross scene (diagnostic pilot, buffer 0)",
                         f"campaign_h8_a3_cross_s{int(seed)}", pol, str(int(seed)), "0.00", view="recording camera",
                         shows=n.replace("_", " "), use="Discussion: same-seed B2 hold vs B3-O passage (diagnostic)")
            self.grid(C / f"CROSS_seed{seed}_B2_vs_B3O_4panel.png",
                      [self.d["annotated"] / f"CROSS_seed{seed}_{n}.png" for n in names], 2,
                      ["B2 Static: before the hold", "B2 Static: final (held)", "B3-O: same time",
                       "B3-O: final"], "Cross scene (diagnostic pilot, buffer 0)", f"cross seed {int(seed)}",
                      "recording camera", "same seed, B2 hold vs B3-O passage", "Discussion (diagnostic)",
                      seed=str(int(seed)), buffer="0.00")
            bgp = self.d["raw"] / f"CROSS_s{seed}_gallery_clean_top.png"
            if seed == "002":
                bgp = self.d["raw"] / "CROSS_s002_overview_clean_top.png"
            if bgp.exists():
                for t in (b2, b3):
                    out = A / f"{t}_trajectory.png"
                    ann.trajectory(bgp, out, Path(runs[t]["trial"]),
                                   f"{t.replace('_', ' ')}: {VERDICT_TEXT.get(info[t]['verdict'])} ({DIAG})")
                    self.rec(out, "overlay", "Cross scene (diagnostic pilot, buffer 0)",
                             f"campaign_h8_a3_cross_s{int(seed)}", info[t]["row"]["policy"], str(int(seed)), "0.00",
                             view="top (ortho) + trial", shows="first plan, AMR 1 trajectory, final footprint "
                             "(background: scene at t = 0)", use="Discussion (diagnostic)")

    # ------------------------------------------------- existing-trial products
    def existing(self):
        D = self.d["trial_overlays"]
        ok = {k: PROJECT_ROOT / v for k, v in EXISTING.items() if (PROJECT_ROOT / v).exists()}
        for k, p in ok.items():
            print(f"  existing {k}: {shown(json.loads(p.read_text())['row'])}")
        if "success_000" in ok and "protective_stop_035" in ok:
            out = D / "S1_buffer_000_vs_035_W22_e0.82_existing.png"
            overlay_many([ok["success_000"], ok["protective_stop_035"]], [None, None], out,
                         ["diagnostic, buffer 0", "Stage 1A (earlier default), buffer 0.35"],
                         "W 2.2 m, Static e 0.82: same condition, two buffers (existing trials)")
            self.rec(out, "composite", "Stage 1 (existing trials)", "w2.20_y0.35_c3 + static_r0.82", "static",
                     buffer="0 / 0.35", context=CONTEXTS["c3"], e="0.82", view="top (data)",
                     shows="buffer 0 passes, buffer 0.35 pushes AMR 1 to the wall -> ProtectiveStop",
                     use="Results: why the buffer was re-selected")
        if "protective_stop_035" in ok:
            out = D / "S1_ProtectiveStop_buffer035_existing.png"
            overlay_many([ok["protective_stop_035"]], [None], out, ["buffer 0.35: wall-side push"],
                         "ProtectiveStop at buffer 0.35 (existing Stage 1A trial)")
            self.rec(out, "overlay", "Stage 1 (existing trials)", "w2.20_y0.35_c3 + static_r0.82", "static",
                     buffer="0.35", context=CONTEXTS["c3"], e="0.82", view="top (data)",
                     shows="trajectory pressed toward the far wall, collision monitor stop",
                     use="Results: buffer side effect (H)")
        if "gate_hold_000" in ok:
            out = D / "S1_comfort_gate_hold_W20_e0.81_existing.png"
            overlay_many([ok["gate_hold_000"]], [None], out, ["buffer 0: gate hold"],
                         "Comfort gate hold at a low e (existing no-buffer trial)")
            self.rec(out, "overlay", "Stage 1 (existing trials)", "w2.00_y0.35_c3 + static_r0.81", "static",
                     buffer="0.00", context=CONTEXTS["c3"], e="0.81", view="top (data)",
                     shows="AMR 1 holds at the gate boundary although the passage exists",
                     use="Results: gate hold mechanism (G)")
        return ok

    # ------------------------------------------------------------- J figure
    def model_figure(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fv = self.plan["final_variant"]
        model = json.loads((PROJECT_ROOT / f"configs/stage1a/model_exec_{fv}.json").read_text())
        nav = json.loads((PROJECT_ROOT / "results/batches/261114_stage1a_final/r_nav_safe.json").read_text())
        cells = {c["env"]: c for c in nav["cells"] if c["part"] == "1a1"}
        pws = model["per_width"]
        fig, axes = plt.subplots(len(pws), 1, figsize=(19.2, 10.8), sharex=True)
        for ax, pw in zip(axes, pws):
            c = cells[pw["env"]]
            for e, frac in pw["points"].items():
                k, n = map(int, frac.split("/"))
                ax.scatter([float(e)], [0], s=140, c=["#009E73" if k * 2 > n else "#D55E00"], zorder=4,
                           edgecolors="black", linewidths=0.6)
                ax.annotate(frac, (float(e), 0), textcoords="offset points", xytext=(0, 12), ha="center",
                            fontsize=10, color="#333333")
            ax.axvspan(pw["R_exec_lo"], pw["R_exec_hi"], color="#999999", alpha=0.25, lw=0)
            marks = [("R_exec (observed bracket)", (pw["R_exec_lo"] + pw["R_exec_hi"]) / 2, "#555555", "-"),
                     ("R_path^infl = R_nav_safe", c["R_nav_safe_e (= R_path^infl)"], "#0072B2", "--"),
                     ("R_path^lethal (optimistic)", c["R_path_lethal_e"], "#CC79A7", ":")]
            if c.get("R_exec_model_e") is not None:
                marks.insert(1, ("R_exec model", c["R_exec_model_e"], "#222222", "-."))
            for lab, x, col, ls in marks:
                ax.axvline(x, color=col, ls=ls, lw=2)
            ax.set_yticks([])
            ax.set_ylim(-0.6, 0.8)
            ax.set_ylabel(f"W {pw['aisle_width_m']:.1f} m", rotation=0, ha="right", va="center", fontsize=12)
            for s in ("top", "right", "left"):
                ax.spines[s].set_visible(False)
        axes[-1].set_xlabel("corridor-side effective extent e (m)", fontsize=12)
        from matplotlib.lines import Line2D
        h = [Line2D([], [], color="#0072B2", ls="--", lw=2, label="R_path^infl = R_nav_safe (Conservative "
                                                                   "Navigation Feasibility Bound)"),
             Line2D([], [], color="#CC79A7", ls=":", lw=2, label="R_path^lethal (optimistic path bound)"),
             Line2D([], [], color="#222222", ls="-.", lw=2, label=f"R_exec model ({fv}, fitted)"),
             Line2D([], [], color="#999999", lw=8, alpha=0.4, label="R_exec observed bracket (last pass .. first fail)"),
             Line2D([], [], marker="o", ls="", color="#009E73", markeredgecolor="black", label="tested e: majority pass"),
             Line2D([], [], marker="o", ls="", color="#D55E00", markeredgecolor="black", label="tested e: majority fail")]
        fig.legend(handles=h, loc="lower center", ncol=3, fontsize=11, frameon=False)
        fig.suptitle(f"Stage 1A-1 ({fv}, buffer {self.fbuf:.2f} m): observed pass/fail vs the path bounds and "
                     "the execution model\nvalues read from configs/stage1a/model_exec_" + fv +
                     ".json and results/batches/261114_stage1a_final/r_nav_safe.json; engineering bounds, "
                     "not comfort distances", fontsize=13)
        fig.tight_layout(rect=(0, 0.1, 1, 0.93))
        out = self.d["trial_overlays"] / "S1_Rpath_Rexec_explanation.png"
        fig.savefig(out, dpi=100)
        plt.close(fig)
        self.rec(out, "overlay", "Stage 1 final (data figure)", "1A-1 W 1.5/1.8/2.2/2.6", "static",
                 buffer=f"{self.fbuf:.2f}", context=CONTEXTS["c3"], view="data figure",
                 shows="tested e with pass counts, R_exec bracket, R_exec model, R_path^infl (= R_nav_safe), "
                       "R_path^lethal", use="Results: R_path vs R_exec (J)")

    # -------------------------------------------------------------- gallery
    def gallery(self, info, runs, ok):
        G = self.d["gallery"]
        panels = []

        def add(v, src, note, scen, pol, seed, buf, e="-"):
            dst = G / f"GALLERY_{v}.png"
            im = Image.open(src).convert("RGB")
            if im.size != (1920, 1080):
                im = im.resize((1920, 1080))
            im.save(dst)
            self.rec(dst, "overlay", "failure gallery", scen, pol, seed, buf, e=e, view="see source",
                     shows=f"{VERDICT_TEXT[v]}: {note}", use="Results/appendix: failure mechanism gallery")
            panels.append((dst, VERDICT_TEXT[v]))

        A, D = self.d["annotated"], self.d["trial_overlays"]
        succ = [t for t in info if info[t]["verdict"] == "SUCCESS" and t.startswith("S1_E_small")]
        if succ:
            add("SUCCESS", D / f"{succ[0]}_trial_overlay.png", f"capture run {succ[0]} (final buffer)",
                "w2.20_y0.35_c3", "static", "-", f"{self.fbuf:.2f}", info[succ[0]]["e"])
        if (D / "S1_comfort_gate_hold_W20_e0.81_existing.png").exists():
            add("COMFORT_GATE_HOLD", D / "S1_comfort_gate_hold_W20_e0.81_existing.png",
                rel(Path(EXISTING["gate_hold_000"])), "w2.00_y0.35_c3", "static", "-", "0.00", "0.81")
        if (D / "S1_ProtectiveStop_buffer035_existing.png").exists():
            add("PROTECTIVE_STOP", D / "S1_ProtectiveStop_buffer035_existing.png",
                EXISTING["protective_stop_035"], "w2.20_y0.35_c3", "static", "-", "0.35", "0.82")
        for v, key, seed in (("POST_EVADE_STALL", "post_evade_stall", "005"), ("NAV2_FAILURE", "nav2_failure", "003")):
            bgp = self.d["raw"] / f"CROSS_s{seed}_gallery_clean_top.png"
            if key in ok and bgp.exists():
                out = A / f"CROSS_seed{seed}_{v}_existing_trajectory.png"
                row = json.loads(ok[key].read_text())["row"]
                ann.trajectory(bgp, out, ok[key], f"size-sweep preliminary trial ({row['scenario']}, seed "
                               f"{int(seed)}): {VERDICT_TEXT[v]}  (background: scene at t = 0)")
                add(v, out, EXISTING[key], f"campaign_h8_a3_cross_s{int(seed)} + {row['scenario']}",
                    row["policy"], str(int(seed)), "0.35 (size sweep)")
        if (PROJECT_ROOT / EXISTING_FRAME_PES).exists():
            dst = G / "GALLERY_POST_EVADE_STALL_existing_frame_261109.png"
            Image.open(PROJECT_ROOT / EXISTING_FRAME_PES).convert("RGB").save(dst)
            self.rec(dst, "clean", "failure gallery (existing 261109 capture)", "capture_cross_s5_zoom",
                     "b3d_state_directional (earlier)", "5", "-", view="recording camera t=36.0s",
                     shows="post-evade stall, existing frame copied unchanged", use="Appendix: failure gallery")
        # ROBOT_ROBOT_DEADLOCK: no trial record in results/ has this verdict
        tile = G / "GALLERY_ROBOT_ROBOT_DEADLOCK_NO_EXISTING_EXAMPLE.png"
        im = Image.new("RGB", (1920, 1080), (245, 245, 245))
        d = ImageDraw.Draw(im)
        d.text((960, 500), "NO EXISTING EXAMPLE", font=ann.font(90, True), fill=(120, 120, 120), anchor="mm")
        d.text((960, 620), "no trial record under results/ is classified ROBOT_ROBOT_DEADLOCK",
               font=ann.font(36), fill=(120, 120, 120), anchor="mm")
        im.save(tile)
        self.rec(tile, "placeholder", "failure gallery", "-", shows="NO EXISTING EXAMPLE "
                 "(placeholder tile, not a capture)", use="do not use as a result")
        panels.append((tile, VERDICT_TEXT["ROBOT_ROBOT_DEADLOCK"]))
        if panels:
            self.grid(self.d["composite"] / "GALLERY_failure_mechanisms_6panel.png", [p for p, _ in panels], 3,
                      [t for _, t in panels], "failure gallery", "mixed (see panels)", "mixed",
                      "one example per verdict where one exists", "Appendix: failure mechanism gallery")

    # ----------------------------------------------------------------- hero
    def hero(self):
        picks = [("S1_context_C1_b3o_hero.png", "HERO_S1_person_comfort_amr_C1.png"),
                 ("S1_static_W22_static_amr_hero.png", "HERO_S1_person_static_amr.png"),
                 ("CROSS_s002_overview_b3o_iso_se.png", "HERO_CROSS_overview_iso.png"),
                 ("CROSS_s002_overview_b3o_crowd.png", "HERO_CROSS_crowd.png"),
                 ("CROSS_s002_overview_b3o_north_aisle.png", "HERO_CROSS_north_aisle.png")]
        for src, dst in picks:
            s = self.d["raw"] / src
            if s.exists():
                shutil.copyfile(s, self.d["hero"] / dst)
                i = self.raw_meta(s)
                self.rec(self.d["hero"] / dst, "clean", "hero (copy of raw/" + src + ")", i["scenario"],
                         i["policy"], i["seed"], i["buffer"], i["context"], i["e"], i["view"],
                         "title / opening slide image", "Slides: title, overview")

    # ---------------------------------------------------------------- utils
    def grid(self, out, srcs, cols, titles, stage, scen, view, shows, use, kind="composite", banner=None,
             seed="-", buffer="-"):
        srcs = [Path(s) for s in srcs]
        if not srcs or not all(s.exists() for s in srcs):
            missing = [rel(s) for s in srcs if not s.exists()]
            print(f"  [skip] {rel(Path(out))}: missing {missing[:2]}")
            return
        ann.grid(Path(out), srcs, cols, titles)
        if banner:
            ann.banner(Path(out), Path(out), banner, "same camera; Comfort Space drawn from the initial configs")
        self.rec(out, kind, stage, scen, seed=seed, buffer=buffer, view=view, shows=shows, use=use)

    def write_index(self):
        rows = sorted(self.index, key=lambda r: r["file"])
        kinds = {}
        for r in rows:
            kinds[r["kind"]] = kinds.get(r["kind"], 0) + 1
        lines = [f"# Capture index ({VERSION})", "",
                 "Visual documentation of the existing implementation and results; not new measurements.",
                 f"Final costmap buffer {self.fbuf:.2f} m ({self.plan['final_variant']}). Display names AMR 1/2/3.",
                 "Cross-scene B2-vs-B3-O captures are **diagnostic pilot** runs (costmap buffer 0), "
                 "not Stage 3 main results. Every image is 1920x1080 or larger (composites).",
                 "", "Regenerate: `python3 scripts/paper_capture_plan.py --final-variant "
                 f"{self.plan['final_variant']}`, `./scripts/paper_captures.sh`, "
                 "`python3 scripts/paper_capture_post.py`.", "",
                 "| kind | count |", "|---|---|"] + [f"| {k} | {v} |" for k, v in sorted(kinds.items())] + [
                 f"| **total** | **{len(rows)}** |", "",
                 "## Recommended (top 10)", ""] + [f"{i}. `{self.root.relative_to(PROJECT_ROOT)}/{f}` — {why}"
                                                    for i, (f, why) in enumerate(TOP10, 1)
                                                    if (self.root / f).exists()] + [
                 "", "## All images", "",
                 "| file | kind | stage | scenario | policy | seed | buffer | human context | e (m) | camera/view "
                 "| what it shows | paper use |", "|" + "---|" * 12]
        for r in rows:
            lines.append("| " + " | ".join(str(r[k]).replace("|", "/") for k in
                                            ("file", "kind", "stage", "scenario", "policy", "seed", "buffer",
                                             "context", "e", "view", "shows", "use")) + " |")
        (self.root / "CAPTURE_INDEX.md").write_text("\n".join(lines) + "\n")
        (self.root / "capture_index.json").write_text(json.dumps(rows, indent=1))
        print(f"[paper_capture_post] {len(rows)} images indexed: {kinds}")


def describe_still(name):
    if name.startswith("S1_env_W22_human_amr"):
        return "person and AMR 1 in the W 2.2 corridor, no Comfort Space", "Method: Stage 1 setup"
    if name.startswith("S1_env"):
        return "Stage 1 corridor, one person 0.35 m from the wall, AMR 1 at the start", "Method: environment"
    if name.startswith("S1_static"):
        t = name.split("_W22_")[1]
        return {"human_only": "person only", "static": "person + Static Comfort Space (floor disc = robot-centre "
                "exclusion, outer band = + buffer)", "static_amr": "person + Static Comfort Space + AMR 1"}.get(
                    t.rsplit("_", 1)[0], t), "Method: Comfort Space"
    if name.startswith("S1_context"):
        return ("B3-O Comfort Space for " + name.split("_")[2] + ", target_size 1.20 m"
                + (" (AMR hidden)" if "noamr" in name else " with AMR 1"), "Method: context-dependent B3-O shapes")
    if name.startswith("S1_buffer_still"):
        return "same Comfort Space drawn with costmap buffer " + name.split("_")[4], "Results: buffer selection"
    if name.startswith("CROSS_s002_overview_clean"):
        return "cross scene, 8 people and AMR 1/2/3 at t = 0, no Comfort Space", "Method: cross scene"
    if name.startswith("CROSS_s002_overview"):
        return "cross scene with B3-O Comfort Spaces (initial config, illustration)", "Method/Discussion: cross"
    if name.startswith("CROSS_s002_static"):
        return "cross scene with Static Comfort Spaces (illustration)", "Discussion: Static vs B3-O"
    if "gallery" in name:
        return "cross scene plan, background for trajectory overlays", "Appendix"
    return "", ""


def overlay(trial, cond, out, title):
    subprocess.run([sys.executable, "scripts/paper_trial_overlay.py", "--trial", str(trial), "--condition",
                    str(cond), "--out", str(out), "--title", title, "--json", str(Path(out).with_suffix(".json"))],
                   cwd=PROJECT_ROOT, check=True, stdout=subprocess.DEVNULL)


def overlay_many(trials, conds, out, titles, title):
    cmd = [sys.executable, "scripts/paper_trial_overlay.py", "--out", str(out), "--title", title,
           "--json", str(Path(out).with_suffix(".json")), "--panel-titles", *titles]
    for t, c in zip(trials, conds):
        cmd += ["--trial", str(t)]
    if all(conds):
        for c in conds:
            cmd += ["--condition", str(c)]
    subprocess.run(cmd, cwd=PROJECT_ROOT, check=True, stdout=subprocess.DEVNULL)


def nearest_after(paths, t0, pat, fmt, must=None):
    best = None
    for p in paths:
        if must and must not in p:
            continue
        m = re.search(pat, Path(p).name)
        if not m:
            continue
        t = datetime.strptime(m.group(1), fmt)
        if t >= t0 and (best is None or t < best[0]):
            best = (t, Path(p))
    return best[1] if best else None


def ffmpeg_path():
    c = sorted(glob.glob(str(Path.home() / "isaacsim/kit/python/lib/python3.*/site-packages/imageio_ffmpeg/"
                                           "binaries/ffmpeg-linux-x86_64-*")))
    return c[-1] if c else shutil.which("ffmpeg")


def duration(ff, src):
    r = subprocess.run([ff, "-i", str(src)], capture_output=True, text=True)
    m = re.search(r"Duration: (\d+):(\d+):([\d.]+)", r.stderr)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None


def cut(ff, src, t, out):
    subprocess.run([ff, "-y", "-loglevel", "error", "-ss", f"{t:.2f}", "-i", str(src), "-frames:v", "1",
                    str(out)], check=False)


def rel(p):
    p = Path(p)
    try:
        return str(p.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(p)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=f"results/captures/{VERSION}")
    a = ap.parse_args()
    post = Post(PROJECT_ROOT / a.root)
    post.stills()
    runs = post.runs()
    info = post.run_products(runs)
    post.run_composites(info, runs)
    ok = post.existing()
    post.model_figure()
    post.gallery(info, runs, ok)
    post.hero()
    post.write_index()
    return 0


if __name__ == "__main__":
    sys.exit(main())
