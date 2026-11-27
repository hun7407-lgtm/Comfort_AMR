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
- Stage 3 human contact = `contact_person_*` columns (robot–robot contact reported apart).

## Log
| time (KST) | event |
|---|---|
| 11-27 21:53 | commit 062a7b3 (plan code + pilots' manifests); pilots started: 3 Stage 2 + 3 Stage 3 launches first as timing samples |
| 11-27 22:11 | timing samples: Stage 2 76 / 78 / 77 s (all SUCCESS); Stage 3 248 / 357 / 244 s (all 3 AMRs SUCCESS, no retry; the 357 s launch was one slow simulator start/run). Initial ETA (ETA_REPORT.md): final analysis optimistic 2026-11-29 04:15, central 06:06, conservative 10:23 KST |
