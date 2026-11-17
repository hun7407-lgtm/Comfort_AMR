#!/usr/bin/env python3
"""Paper/presentation videos of the complex cross scenario: configs and run list.

    python3 scripts/paper_video_plan.py             # write configs + runs.txt
    ./scripts/paper_captures.sh --root results/videos/paper_261114/demo_runs --only runs
    python3 scripts/paper_video_post.py             # cut, overlay, index

Demonstration only. The navigation side is the frozen Stage 1 setting
(configs/frozen/navigation_requirements.yaml: Comfort costmap buffer
0.10 m, same safety layer and Nav2 parameters for every policy). The
Comfort Space sizes are the ones of the existing diagnostic cross pair
(Static r 1.20 / B3-O target 1.20): Stage 2 has not produced
R_comfort_min / R_pref, so nothing here is Stage 3 Dynamic Selection or
a final performance result.

Writes
    configs/captures/paper_261114/video/cameras.yaml        CAM01..CAM05 (re-shootable)
    configs/captures/paper_261114/video/demo_*.yaml         condition overlays (buffer 0.10)
    configs/captures/paper_261114/video/rec_s<seed>_<clean|overlay>.yaml
    results/videos/paper_261114/demo_runs/runs.txt
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from comfort_core.selection import frozen_navigation  # noqa: E402

VERSION = "paper_261114"
CFG_DIR = PROJECT_ROOT / "configs" / "captures" / VERSION / "video"
RUN_ROOT = PROJECT_ROOT / "results" / "videos" / VERSION / "demo_runs"
FPS = 15          # taken on the simulation clock; resampled to 30 fps in post
SIZE = [1920, 1080]

# Fixed camera set. CAM01 is orthographic (metres -> pixels exact, for overlays).
CAMERAS = {
    "CAM01_TOP": {"ortho": {"centre": [0.0, 0.0], "width_m": 30.0}},
    "CAM02_ISOMETRIC": {"position": [16.0, -14.0, 11.0], "look_at": [0.0, 0.0, 1.0], "focal_length": 20.0},
    "CAM03_CROSS_CLOSEUP": {"ortho": {"centre": [4.0, 2.0], "width_m": 14.0}},
    "CAM04_HUMAN_AMR_CLOSEUP": {"position": [2.6, 1.0, 3.4], "look_at": [5.3, 3.8, 0.5], "focal_length": 18.0},
    "CAM05_LOW_PERSPECTIVE": {"position": [11.5, 4.21, 1.9], "look_at": [-10.0, 4.21, 0.9], "focal_length": 22.0},
}
POLICIES = {
    "B2": ("demo_b2_static_r1.20_buf010", "B2 Static Comfort Space r 1.20 m (demo, not calibrated)",
           {"policy": "static", "static_radius": 1.2}),
    "B3O": ("demo_b3o_r1.20_buf010", "B3-O Dynamic Comfort Space, target 1.20 m (demo, not Dynamic Selection)",
            {"policy": "state_directional",
             "aisle_scaling": {"enabled": True, "target_size_m": 1.2, "floor_m": 0.0}}),
}
# seed -> runs. Seeds 2 and 3: the existing diagnostic pair (buffer 0)
# recorded B2 comfort_gate_hold and B3-O SUCCESS on both
# (results/batches/261114_diag_nobuffer_cross); here the same seeds run
# with the frozen buffer 0.10, and the outcome is whatever it is.
RUNS = [
    (2, "B3O", "overlay"), (2, "B3O", "clean"), (2, "B2", "overlay"),
    (3, "B3O", "overlay"), (3, "B2", "overlay"),
]

# Failure clips. Re-recordings of size-sweep trials that recorded the
# mechanism (results/batches/261112_stage1_size_sweep), with their own
# condition files unchanged (that study's buffer 0.35). A clip is used
# only if the re-run records the same verdict. Both stopped near (1.8, 5.3).
FAILURE_RUNS = [
    ("configs/stage1/b3o_r1.40.yaml", 5, "POST_EVADE_STALL"),   # 20261112T050609_b3o_r1.40_..._cross_s5
    ("configs/stage1/b3o_r0.80.yaml", 3, "NAV2_FAILURE"),       # 20261112T030059_b3o_r0.80_..._cross_s3
]
FAIL_CAM = {"position": [0.2, 0.6, 6.5], "look_at": [1.9, 4.8, 0.0], "focal_length": 18.0}

# Top views of the capture-run scenarios that were recorded with one
# perspective camera only (results/captures/paper_261114/runs.txt): same
# condition and base files, only the camera becomes an orthographic top
# view (+ pose log). Root: results/videos/paper_261114/top_runs.
TOP_ROOT = PROJECT_ROOT / "results" / "videos" / VERSION / "top_runs"
TOP_SCENES = {      # capture config stem -> (base, top camera)
    "top_W22": ("configs/stage1a/envs/w2.20_y0.35_c3.yaml", {"ortho": {"centre": [0.0, 0.0], "width_m": 13.0}}),
    "top_W20": ("configs/stage1a/envs/w2.00_y0.35_c3.yaml", {"ortho": {"centre": [0.0, 0.0], "width_m": 13.0}}),
    "top_cross_s2": ("configs/campaigns/cross/campaign_h8_a3_cross_s2.yaml", CAMERAS["CAM01_TOP"]),
    "top_cross_s3": ("configs/campaigns/cross/campaign_h8_a3_cross_s3.yaml", CAMERAS["CAM01_TOP"]),
}
TOP_RUNS = [        # (condition, scene, tag) -- the conditions of the capture runs, unchanged
    ("configs/stage1a/conditions_buf10/static_r0.42.yaml", "top_W22", "TOP_S1_E_small_W22_e0.42"),
    ("configs/stage1a/conditions_buf10/static_r1.02.yaml", "top_W22", "TOP_S1_E_lastpass_W22_e1.02"),
    ("configs/stage1a/conditions_buf10/static_r1.07.yaml", "top_W22", "TOP_S1_E_firstfail_W22_e1.07"),
    ("configs/stage1a/conditions_nobuf/static_r0.81.yaml", "top_W20", "TOP_S1_buffer_000_W20_e0.81"),
    ("configs/stage1a/conditions_buf10/static_r0.81.yaml", "top_W20", "TOP_S1_buffer_010_W20_e0.81"),
    ("results/captures/paper_261114/overlays/static_r0.81.yaml", "top_W20", "TOP_S1_buffer_035_W20_e0.81"),
    ("configs/stage1a/diagnostic/diag_nobuffer_static_r1.20_cross.yaml", "top_cross_s2", "TOP_CROSS_s002_B2_diagnostic"),
    ("configs/stage1a/diagnostic/diag_nobuffer_b3o_r1.20_cross.yaml", "top_cross_s2", "TOP_CROSS_s002_B3O_diagnostic"),
    ("configs/stage1a/diagnostic/diag_nobuffer_static_r1.20_cross.yaml", "top_cross_s3", "TOP_CROSS_s003_B2_diagnostic"),
    ("configs/stage1a/diagnostic/diag_nobuffer_b3o_r1.20_cross.yaml", "top_cross_s3", "TOP_CROSS_s003_B3O_diagnostic"),
]


def main() -> int:
    nav = frozen_navigation()
    buf = float(nav["comfort_buffer_m"])
    CFG_DIR.mkdir(parents=True, exist_ok=True)
    RUN_ROOT.mkdir(parents=True, exist_ok=True)
    head = ("# DEMO / CAPTURE ONLY -- generated by scripts/paper_video_plan.py. Not a study\n"
            "# condition and not Stage 3: frozen navigation (buffer from\n"
            "# configs/frozen/navigation_requirements.yaml), Comfort size of the\n"
            "# existing diagnostic cross pair.\n")
    (CFG_DIR / "cameras.yaml").write_text(head + yaml.safe_dump({"resolution": SIZE, "fps": FPS,
                                                                 "cameras": CAMERAS}, sort_keys=False))
    conds = {}
    for key, (name, label, cs) in POLICIES.items():
        p = CFG_DIR / f"{name}.yaml"
        p.write_text(head + yaml.safe_dump({
            "scenario": {"name": name, "label": label},
            "comfort_space": {**cs, "costmap": {"buffer_width": buf}}}, sort_keys=False))
        conds[key] = str(p.relative_to(PROJECT_ROOT))
    lines = ["# scenario-overlay capture-config tag  (demo runs, not measurements)"]
    for seed, pol, look in RUNS:
        main_cam, *extra = CAMERAS.items()
        rec = {"version": VERSION, "fps": FPS, "resolution": SIZE, "label": False, "pose_log": True,
               "camera": main_cam[1],
               "extra_cameras": [{"name": n, **c} for n, c in extra]}
        cfgp = CFG_DIR / f"rec_s{seed}_{look}.yaml"
        cfgp.write_text(head + yaml.safe_dump({
            "base": f"configs/campaigns/cross/campaign_h8_a3_cross_s{seed}.yaml",
            "recording": rec,
            "comfort_space": {"visualization": {"enabled": look == "overlay"}}}, sort_keys=False))
        lines.append(f"{conds[pol]} {cfgp.relative_to(PROJECT_ROOT)} "
                     f"CROSS_seed{seed:03d}_{pol}_{look}")
    for cond, seed, mech in FAILURE_RUNS:
        cfgp = CFG_DIR / f"rec_fail_s{seed}.yaml"
        cfgp.write_text("# FAILURE RE-RECORDING ONLY -- generated by scripts/paper_video_plan.py.\n"
                        "# Re-runs a size-sweep trial with ITS condition file (buffer 0.35 of that\n"
                        "# study), NOT the frozen navigation setting; only the camera is added.\n"
                        + yaml.safe_dump({
                            "base": f"configs/campaigns/cross/campaign_h8_a3_cross_s{seed}.yaml",
                            "recording": {"version": VERSION, "fps": FPS, "resolution": SIZE, "label": False,
                                          "pose_log": True, "camera": FAIL_CAM,
                                          "extra_cameras": [{"name": "CAM01_TOP", **CAMERAS["CAM01_TOP"]}]}},
                            sort_keys=False))
        lines.append(f"{cond} {cfgp.relative_to(PROJECT_ROOT)} FAILREPLAY_seed{seed:03d}_{mech}")
    (RUN_ROOT / "runs.txt").write_text("\n".join(lines) + "\n")
    TOP_ROOT.mkdir(parents=True, exist_ok=True)
    for stem, (base, cam) in TOP_SCENES.items():
        (CFG_DIR / f"{stem}.yaml").write_text(
            "# TOP-VIEW CAPTURE ONLY -- generated by scripts/paper_video_plan.py. Same base\n"
            "# and condition as the capture run; only the camera (orthographic top) and\n"
            "# the pose log are added.\n"
            + yaml.safe_dump({"base": base,
                              "recording": {"version": VERSION, "fps": FPS, "resolution": SIZE, "label": False,
                                            "pose_log": True, "camera": cam}}, sort_keys=False))
    (TOP_ROOT / "runs.txt").write_text(
        "# scenario-overlay capture-config tag  (top-view capture runs, not measurements)\n"
        + "".join(f"{c} {(CFG_DIR / f'{s}.yaml').relative_to(PROJECT_ROOT)} {t}\n" for c, s, t in TOP_RUNS))
    print(f"{len(TOP_RUNS)} top-view capture runs -> {TOP_ROOT / 'runs.txt'}")
    print(f"buffer {buf} m (frozen); {len(RUNS)} demo runs + {len(FAILURE_RUNS)} failure re-recordings -> {RUN_ROOT / 'runs.txt'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
