# Nav2-only campaign — run log and pre-registered rules (2026-11-26 night → 2026-11-27)

Authority: [NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md](NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md).
**ABBREVIATED by user request** ("내일 오후 8시에는 마무리되도록 약식으로 진행", 2026-11-26 22:20):
fewer seeds / repeats than the spec; every result here is provisional and labelled so.
Old batches (`261125_*`, `261126_stage2sim*`, `261126_stage2scale_v1_smoke`) are untouched and are gate-chain history.

## 1. What changed in the code (new runs only; default keeps old behaviour)

| file | change |
|---|---|
| `comfort_core/control_mode.py` (new) | `control.mode` = `legacy_gate` (default) / `nav2_only`; merged-config guard; ROS-graph guard |
| `scripts/launch_nodes.py` | nav2_only: AMR1 controller / behaviors publish `cmd_vel_safe` directly (like the fleet) |
| `scripts/nav2_bringup.launch.py` | nav2_only: `comfort_gate.py` is not launched (so no yield / evade / backoff / off-lane) |
| `scripts/nav2_goal.py` | nav2_only: instead of requiring the gate heartbeat, checks the live graph (no `comfort_gate` node; `/cmd_vel_safe` and `/<ns>/cmd_vel_safe` published only by controller_server / behavior_server and read by collision_monitor; `/cmd_vel` only from collision_monitor; nothing on `cmd_vel_nav`) and the merged config; any violation → INVALID |
| `comfort_core/metrics.py` | trial JSON keeps `control` (mode, graph, problems); CSV columns unchanged |
| `configs/nav2only/` (new) | `control_overlay.yaml`; `seeds/s1..s30.yaml` = legacy Stage 1 scene + overlay; `smoke/safety_headon.yaml` |
| `configs/stage2scale/stage2scale_v2_nav2only.yaml`, `scripts/stage2scale_plan.py` | v2: versioned out_dir, control overlay, R_ref with provenance, alpha grid from R_ref, frozen geometry, confirm phase |
| `scripts/nav2only_stage1_report.py`, `scripts/stage2scale_freeze.py` (new) | Stage 1 rule, pilot freeze rule |

Unchanged: Nav2 planner / controller / BT / native recoveries, collision monitor and its fields, footprint,
mask value 98, min_radius 0.30, `perception.source: tf`, `humans.state_source: ground_truth`.

## 2. Pre-registered rules (written before the corresponding data)

- Stage 1 (revalidation): `scripts/nav2only_stage1_report.py` docstring. Seeds 1–10 (ABBREVIATED), B1 + B2 0.85 /
  0.90 / 0.95 / 1.00. Pass = comfort-mask no-progress ≤ 5 %, system success ≥ same-seed B1 − 5 %p, contact trials ≤
  B1's. R_nav_upper = largest passing size; largest tested passes → NOT_IDENTIFIED_IN_RANGE (R_ref stays
  `legacy_0.90m` as a labelled design anchor); none passes → stop and report.
- R_ref: IDENTIFIED → R_ref = new upper (`new_nav2_only_upper`), alpha grid recomputed from min_radius;
  otherwise 0.90 with label `legacy_0.90m_design_anchor`.
- Stage 2 pilot freeze: `scripts/stage2scale_freeze.py` docstring (B1 must complete without contact; distance
  response ≥ 0.10 m; first acceptable geometry in declared order).
- Stage 2 selection: `comfort_core/stage2scale.py` (J, non-monotone, identification statuses), C* = 5.0,
  joint ≥ 0.80 and nav ≥ 0.80; sensitivity 4.5 / 5.5 and joint ≥ 0.90.

## 3. Schedule (ABBREVIATED)

| step | trials | batch |
|---|---|---|
| runtime + safety smoke | 2 | `results/batches/261126_nav2only_runtime_smoke` |
| Stage 1 revalidation | 50 | `results/batches/261126_nav2only_stage1` |
| Stage 2 pilot (first repeat = smoke) | 110 | `results/batches/261126_nav2only_stage2_v2_pilot` |
| Stage 2 coarse (5 repeats) | ≤ 225 | `..._coarse` |
| Stage 2 confirm (10 repeats, tight only) | ≤ 200 | `..._confirm` |

## 4. Log

| time (KST) | event |
|---|---|
| 11-26 22:33 | runtime smoke started (commit 58c7bb9 + uncommitted Nav2-only changes) |
| 11-26 22:38 | runtime smoke 2/2 done. Stage 1 scene s1 B2 0.90: graph guard OK (no comfort_gate; /cmd_vel_safe, /a1/…, /a2/… only controller_server + behavior_server; /cmd_vel only collision_monitor), AMR1 / a1 / a2 all reached their goals. Safety head-on (B1, walker in the AMR's lane, walker ignores the robot): collision monitor STOP 2.07 s, AMR 0.79 → 0.06 m/s before the bodies met; the scripted walker then walked on and pushed the stopped AMR back ~1.5 m (contact 1.95 s). Safety layer works; the contact comes from the non-avoiding walker. |
| 11-26 22:50 | rules + code committed before any Stage 1 / Stage 2 Nav2-only data; Stage 1 revalidation started |
| 11-27 02:24 | Stage 1 revalidation 50/50 done (0 excluded, all gate-free by graph guard). **IDENTIFIED: R_nav_upper(nav2-only) = 0.95 m, transition 1.00 m** (ABBREVIATED n = 10/size, provisional). System success B1 8/10, 0.85 / 0.90 / 0.95 10/10, 1.00 8/10 (ROBOT_ROBOT_BLOCK s004, HUMAN_MOTION_BLOCK s003); comfort-mask no-progress 0/10 at every size. 1.00 fails on contact trials 10/10 > B1 9/10 (success alone would pass: 8/10 = B1) — a thin margin at n = 10. Contact note: human–AMR contact occurs in 9–10/10 trials at every size incl. B1, almost all on fleet AMR a2 (which never had a gate); legacy gate-chain Stage 1 seeds 1–10 had the same pattern (a2 contact 8–10/10 per condition), so it is a property of the scene's non-avoiding walkers, not of the Nav2-only chain. AMR1 contact: B1 4/10 (legacy B1 4/10). |
| 11-27 02:26 | R_ref := 0.95 (`anchor_provenance: new_nav2_only_upper`); alpha grid [1.0 … 0.5, 0.45] (no clip: min 0.30/(0.75·0.95) = 0.421). Report figure label now uses R_ref. Pilot generated (11 geometries, 110 trials). |
| 11-27 04:59 | Stage 2 pilot 110/110 done (0 excluded, 0 INVALID). Freeze rule applied unchanged: ST-S w2.60, ST-R w2.60, EN-R w2.20, MV-P w3.00 gap 0.65 (gap 0.80 / 1.00: distance response 0.046 / 0.015 m < 0.10 → rejected). **ST-F: both widths rejected** (B1 incomplete or contact in one of 2 launches) → UNIDENTIFIABLE_GEOMETRY, not run further. Every pilot contact (6) is in B1 (no Comfort Space: AMR passes at d_cc 0.48–0.49 m, footprint edge 0.20–0.23 m < human radius 0.25, then collision monitor STOP next to the person); 0 contacts in B2 / S3. Failures in Comfort conditions are terminal PROTECTIVE_STOP at w2.20 (B2 0.95 in all four static contexts, α 1.00 in ST-R) — wall inside the collision-monitor stop field while detouring. Known limitation of the rule: it uses B1 as the feasibility probe, so a B1 that grazes the person removes the geometry. |
| 11-27 05:05 | `stage2scale_plan.py`: a context frozen as UNIDENTIFIABLE_GEOMETRY is recorded and skipped (instead of aborting the plan). Coarse generated: 4 contexts × 9 conditions × 5 = 180 trials. |
