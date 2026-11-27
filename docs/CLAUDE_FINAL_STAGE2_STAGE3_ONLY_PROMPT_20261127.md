# Claude FINAL HANDOFF — Stage2 (750) + Stage3 (150) Nav2-only, plus KST finish-time estimate

Repository: `hun7407-lgtm/Comfort_AMR`
Branch: `feature/b1-baseline-and-yield`

## Mandatory READ FIRST and authority

1. `docs/FINAL_STAGE2_STAGE3_NAV2_ONLY_PROTOCOL_20261127.md` (**current authoritative final 900-MAIN protocol**)
2. `AGENTS.md`, `docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md`, `docs/NAV2ONLY_STAGE2_RESULTS.md`, `docs/NAV2ONLY_RUN_LOG.md`.
3. `configs/common.yaml`, `configs/nav2only/control_overlay.yaml`, `comfort_core/control_mode.py`, `comfort_core/stage2scale.py`, `comfort_core/comfort_reference.py`, `comfort_core/final_metrics.py`, `scripts/stage2scale_plan.py`, `scripts/stage2scale_report.py`, `scripts/nav2_goal.py`, `scripts/launch_nodes.py`, `scripts/nav2_bringup.launch.py`, `scripts/final_plan.py`, `scripts/final_run.py`, `scripts/make_cross_campaign.py`, and prior `results/batches/261125_final_stage3/analysis/STAGE3_REPORT.md` as legacy caution.

**User's revised decision supersedes all old plans that require Stage 1 rerun, Stage 2 B1/B2, or 1,350/1,500/1,650 new runs.** Do not follow those outdated counts. Stage1 NEW=0, final Stage2=750, final Stage3=150, MAIN TOTAL=900.

## Research objective

Implement and run a thesis-grade controlled experiment that tests the hypothesized **middle Scale Comfort–Navigation tradeoff**, not a scripted demonstration of desired findings. The old thesis draft's example values such as alpha0.80 joint 86.7% are NOT observed/target values; no fabricated or tuned outcomes. The winner can be any tested policy or none; publish null, unfavorable and unidentifiable findings.

Use existing abbreviated Nav2-only Stage 1 `R_ref=0.95 m` **as a preliminary engineering reference ONLY**; do not rerun or modify Stage 1 or call the 0.95m a proven upper safety limit.

## Stage 2: Exactly 750 final main runs, with B3 Scale only

- Isaac Sim + ROS2 Nav2, **1 human/1 AMR**.
- 5 contexts: `ST-F` stationary front, `ST-S` stationary side, `ST-R` stationary rear, `EN-R` station_engaged rear, `MV-P` moving counterwalking encounter.
- **5 B3 asymmetric policies only: alpha = 0.50, 0.60, 0.70, 0.80, 0.90**. **Do not include B1, B2, 0.45, or 1.00.**
- **5 contexts × 5 alphas × 30 launches = 750 MAIN trials**. Pilot/smoke/diagnostic not pooled.
- Freeze F/S/R state-dependent reference geometry from previous Stage2 code, scaling all three axes by the SAME alpha; no independent axis tuning or shape replacement. Check no `min_radius=0.30m` clipping and actual costmap geometry across conditions.
- Real physical EN-R station fixture, true crossing encounters for MV-P, valid ST-F front pass. Existing ST-F and MV-P unidentifiable cases require **policy-neutral pilot review** before main; log limitations faithfully rather than choose pilot geometry to favor .70/.80.
- Every cell logs completed navigation/safe completion/joint predicted comfort>=5.0, actual minimum person-AMR center distance + footprint clearance, predictor output by actual logged distance & bearing (not alpha), contact, protective-stop, Travel Time, Path Length, idle/replans, per-trial/per-encounter observations and 95% CI. `C*=5.0` primary, sensitivity 4.5 & 5.5. Report floor-limited/unidentifiable scale where applicable; do not infer comfort lower bound from smallest tested value.

## Stage 3: Exactly 150 final main runs, FIVE comparison policies

- **8 humans (moving/stationary/station_engaged) / 3 fully Nav2-controlled AMRs.**
- 5 cases × 30 matched NEW holdout seeds (`1001–1030`):
  1. `B2_circle_0.95m`
  2. `B3_uniform_alpha_0.60`
  3. `B3_uniform_alpha_0.70`
  4. `B3_uniform_alpha_0.80`
  5. `B3_uniform_alpha_0.90`
- Plan **10 nominal factory + 10 station-aisle bottleneck + 10 moving crossing** scenarios BEFORE looking at results, physically feasible and policy-neutral. Within a matched seed, keep all worker positions/states/paths/timings and all robot missions, map, safety and control invariant; change only Comfort Space policy/mask. Counterbalance trial order. Prior Gate-enabled Stage3 seeds `101–130` and results cannot be pooled.
- **Primary endpoint:** 3 AMRs all complete missions AND ZERO Human–AMR physical contact AND all required, scorable human encounters have predicted Comfort >=5.0. NO encounter/missing comfort score is not automatic pass; report coverage and missingness. Secondary: system success, safe completion, actual distance, Comfort median/P10/distributions, Travel Time (success-only + all-trial capped), Path Length, robot/comfort deadlock, protective stops, replans, idle time, contact and 95% CIs.
- Predetermined safety + joint/nav feasibility screening, then maximal observed joint rate; ties/uncertainty -> Pareto feasible group, time tie-break predeclared. Matched outcome comparisons with uncertainty; historical repeated identical masks disagreed 11/30 seeds so run prespecified duplicate-control diagnostic **outside 150 MAIN** to quantify nondeterminism. Never force alpha .70/.80 to win.
- Global alpha applied to different state-specific F/S/R ratios does NOT prove distinct learned state-specific alphas. Additional sixth policy is optional only if separately preregistered, not silently added to main count.

## Nonnegotiable runtime integrity

Every experimental robot: `Nav2 controller or native behavior_server → cmd_vel_safe → Collision Monitor → cmd_vel`; `control.mode:nav2_only`, buffer 0.10m, finite cost 98, safety/recovery preserved, no custom comfort_gate, custom yielding, evade, backoff or off_lane. Validate actual merged YAML and ROS graph per trial. Controlled human state `ground_truth` and position from `tf`; do not claim sensor state recognition. DO NOT weaken collision monitor or hide worker-induced contacts.

Audit `comfort_core/final_metrics.py` so Nav2 stop time isn't mislabeled `Gate Yield`; check secondary comfort-reference robot-width sign; test all 3 robot mask consumption and contact logging, retry/error classification, condition physical signature identity, all 30 cell denominators, and reproducibility. Version all new code/configs/results; never overwrite previous batches.

## **IMPORTANT: User specifically requests ESTIMATED COMPLETION TIME in KOREAN STANDARD TIME**

This is a real experiment duration estimate, not a guaranteed promise. **Implement and produce a realistic ETA based on ACTUAL observed per-run wall-clock elapsed time** (including Isaac Sim startup and teardown), not simulated-time or arbitrary guessing:

1. At beginning record system `start_time` as full **YYYY-MM-DD HH:mm:ss Asia/Seoul**, GPU/CPU/parallel workers, current valid launch count and remaining Stage2=750, Stage3=150.
2. During early Smoke/Pilot, MEASURE at least 3 representative Stage2 launches AND 3 representative Stage3 launches, with `wall_start`, `wall_end`, `elapsed_seconds`, outcome and whether it hit timeout. Then calculate separate Stage2 / Stage3 measured throughput in **runs/hour**, session setup, analysis & likely retries. If Stage3 samples unavailable, label its provisional estimate explicitly.
3. Compute remaining `n2=750 - valid_final_stage2`, `n3=150 - valid_final_stage3`. Predict `n2 × real Stage2 seconds/trial + n3 × real Stage3 seconds/trial + remaining pilot + overhead + retries + analysis`. Do not double count completed runs; don't assume 2 parallel runs merely because CPU has cores. If actual parallelism exists use **measured total throughput**.
4. Show **Stage2 completion expected YYYY-MM-DD HH:mm KST, Stage3 expected completion, entire results and plots finish expected**, together with **remaining days/hours**, measured sample count, optimistic/central/conservative ranges, current completion %, and warnings about ROS/Isaac scheduling variance/timeouts. Base optimistic/central/conservative on actual P25/median-or-mean/P90 observed run duration (plus explicit overhead).
5. Create/update `ETA_REPORT.md` and `progress_<timestamp>.json` at early timing sample then **every 30 actual completed main trials**, recalculating estimate from live cumulative throughput; state completed/remaining, success/fail/invalid, trials/hour, elapsed time, expected completion date/time. If slower/faster than first estimate, show revised ETA and rationale.
6. If simulator is not accessible, **do not invent ETA based on imaginary measurements**. State that reliable estimated KST deadline requires real benchmark, provide exact diagnostic command and how to obtain ETA. Historical Gate-based ~220–395 wall seconds per multi-AMR run is just rough prior context, not calibrated for new Stage2/Stage3.

## Workflow / output

- FIRST read authoritative protocol & inspect existing code; adapt into new **Stage23-900** config generators/manifest, controller & logging audits, offline tests, pilot geometry freeze, and full 30-run pipelines.
- Verify the 750 + 150 planned unique main runs; get short real timing samples **as early as safe**, and provide the user with an initial **measured KST ETA before running all 900**. Proceed with actual runs if execution environment permits, updating progress/ETA.
- Commit all frozen config/manifest hashes before main data, then save full per-trial/per-encounter result CSV, metrics/95% CI, STAGE2/STAGE3 reports and manuscript graphs. Record git SHA and reproducible commands, preserve old files.
- Korean thesis-ready 실험 설계/실험 결과/결론 each **two paragraphs, exactly 3 sentences each** AFTER real results (never copy invented illustration values as data). Clearly distinguish observed findings, surrogate assumptions, run noise and unresolved contexts.
- If Isaac/ROS cannot run on this machine, implement/offline verify, report MAIN executed count accurately and the precise workstation command to benchmark/run. No falsely claimed finish time.

**Completion checklist: Stage1=0 NEW; Stage2=5×5×30=750 MAIN; Stage3=5×30=150 MAIN; GRAND MAIN TOTAL=900. Separate pilot/smoke/duplicate diagnostics. ETA based on measured real wall-clock time, reported as a concrete Korea-local date and time with bounds.**
