# stage23_900 — run log (final Stage 2 750 + Stage 3 150, Nav2-only)

Authority: [FINAL_STAGE2_STAGE3_NAV2_ONLY_PROTOCOL_20261127.md](FINAL_STAGE2_STAGE3_NAV2_ONLY_PROTOCOL_20261127.md).
Config: `configs/final_nav2only_stage23_900/stage23_900.yaml` (every number, freeze rule, winner rule — written before data).
Code: `scripts/final_stage23_900_{plan,freeze,report2,report3,eta}.py`, `scripts/final_stage23_900_main.sh`, tests `scripts/check_stage23_900.py`.
Batches: `results/batches/261127_stage23_900_{s2pilot,s3pilot,s2main,s3main,s3dup}`; ETA `results/batches/261127_stage23_900_status/ETA_REPORT.md`.
Stage 1: NOT rerun; R_ref = 0.95 m (abbreviated 2026-11-26 Nav2-only revalidation) as a fixed engineering reference.

start_time: 2026-11-27 21:53:33 KST (first pilot launch). Hardware: NVIDIA GeForce RTX 5080 (driver 580.173.02),
Intel i9-12900 (24 threads), 62 GB RAM; Isaac Sim 5.1 + ROS 2 Jazzy + Nav2; ONE trial at a time (one Isaac Sim worker).

## Code fixes required by the protocol (§6)
- `comfort_core/final_metrics.py`: under `control.mode nav2_only` a standstill is not a yield — classes
  SUCCESS / SUCCESS_IDLE (1–10 s) / SUCCESS_LONG_IDLE (≥ 10 s), `gate_yield_s = None`, `idle_s` = speed < 0.05 m/s;
  legacy runs keep NORMAL_YIELD / TEMPORARY_HOLD and gate seconds.
- `comfort_core/comfort_reference.py`: secondary `predicted_comfort_clearance_matched` used d_cc + (hw_this − hw_ref);
  fixed to the documented d_equiv = d_cc − hw_this + hw_ref (+ unit test). Primary metric unchanged.
- Stage 3 human contact: CORRECTED 11-28 00:2x (before main data). The simulator splits ONE human–AMR touch by who
  closed the last centimetre: `contact_person_*` = the person walked in, `contact_robot_*` = the AMR drove in. Both are
  human–AMR contact; the first draft counted only `contact_person_*` and mislabelled the other as robot–robot.
  Primary contact = both halves; the split is reported (`contact_person_closed_s`, `contact_robot_closed_s`).

## Log
| time (KST) | event |
|---|---|
| 11-27 21:53 | commit 062a7b3 (plan code + pilots' manifests); pilots started: 3 Stage 2 + 3 Stage 3 launches first as timing samples |
| 11-27 22:11 | timing samples: Stage 2 76 / 78 / 77 s (all SUCCESS); Stage 3 248 / 357 / 244 s (all 3 AMRs SUCCESS, no retry; the 357 s launch was one slow simulator start/run). Initial ETA (ETA_REPORT.md): final analysis optimistic 2026-11-29 04:15, central 06:06, conservative 10:23 KST |
| 11-28 00:19 | pilots done: Stage 2 60/60, Stage 3 15/15, 0 infra retries / INVALID |
| 11-28 00:2x | **Stage 2 freeze** (rule in yaml `stage2.freeze_rule`, `analysis/pilot_freeze.json`): ST-F / ST-S / ST-R / EN-R FROZEN at width 2.60 (first VALID+SENSITIVE candidate; responses 0.28 / 0.36 / 0.36 / 0.30 m, 0 contact, 6/6 completed). MV-P: both lane gaps REJECTED (contact 5/6 at g0.40, 6/6 at g0.25; response 0.137 / −0.021 m) → run in MAIN at the first candidate w3.00_g0.40 labelled UNIDENTIFIABLE_GEOMETRY, as the rule prescribes. No geometry was added or tuned after seeing the pilot |
| 11-28 00:2x | **Pilot contact finding**: every pilot contact (Stage 2: 12 MV-P launches; Stage 3: 9/15 trials, all five policies including B2) was closed by the scripted person (robot-closed share 0.00 s in all 75 pilot launches). People never avoid the AMR (`configs/common.yaml avoid_robot: false`, a study premise kept for comparability). Stage 3 NOMINAL s2001: walker E2 walks into a2 standing at 0.01 m/s in all five policies; CROSSING s2003: N2 walks into AMR1 in all four S3 policies, not in B2 |
| 11-28 00:2x | **Declared before main data** (yaml `stage3.secondary_contact_sensitivity`): primary endpoints and the winner rule are unchanged (ALL human–AMR contact). A SECONDARY sensitivity repeats the winner rule counting only robot-closed contact; Stage 2 tables add the robot-closed count. Scenarios are NOT altered to remove contacts |
| 11-28 00:2x | **Stage 3 pilot check** (policy-neutral, no timing change needed): 15/15 valid; CROSSING yields moving-state encounters (6), BOTTLENECK none moving (stationary 5 + engaged 11), NOMINAL has all three states. Pilot decision by the frozen rule: NO_ACCEPTABLE_SCALE (n = 3, descriptive only) |
| 11-28 00:24 | manifests generated: s2main 750 (manifest sha 76eeb062b9be4298), s3main 150 (82c1e3bba664ba49), s3dup 6 (3d9ef1007f467238), config sha cd654c19b78810dc; offline checks ALL PASS |
| 11-28 00:24:45 | commit 1f0f36a (freeze + manifests + secondary declaration), pushed. Note: the 00:30–00:45 times first written in this log / the yaml comment were estimates; the true order is fixed by the commit time (00:24:45) vs the first MAIN launch (00:24:53) |
| 11-28 00:24:53 | MAIN started: `nohup ./scripts/final_stage23_900_main.sh` (s2main 750 → s3main 150 → s3dup 6); ETA watcher rewrites `results/batches/261127_stage23_900_status/ETA_REPORT.md` every 30 MAIN trials |
