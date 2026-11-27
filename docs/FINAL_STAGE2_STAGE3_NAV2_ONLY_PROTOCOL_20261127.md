# FINAL STAGE 2–3 ONLY — Nav2-only Comfort–Navigation 900-trial protocol (2026-11-27)

**CURRENT AUTHORITATIVE PROTOCOL. STATUS: DESIGN FROZEN FOR IMPLEMENTATION; EXPERIMENTS NOT YET EXECUTED.**
**User decision:** Stage 1 is NOT rerun. Stage 2 has no B1 or B2. Stage 3 has B2 and four B3 scale conditions. Pilot, smoke, runtime replicates are separate from MAIN counts.

This document **supersedes all incompatible directions** in:
- `docs/FINAL_NAV2_ONLY_3STAGE_PROTOCOL_20261127.md` (old 1650-trial three-stage plan);
- `docs/CLAUDE_FINAL_NAV2_ONLY_3STAGE_PROMPT_20261127.md`;
- earlier versions of `docs/CLAUDE_FINAL_STAGE2_STAGE3_ONLY_PROMPT_20261127.md` (old 1500-trial plan);
- earlier abbreviated Stage 2 and 2026-11-25 Gate-enabled Stage 3 protocols **for the new run only**.
Do not alter or delete existing raw result batches. Git SHA, frozen geometry, and source provenance must be stored.

## 1. Goal, hypothesis, and no-result-steering rule

Research contribution: a Digital Twin framework for reference asymmetrical state-dependent Comfort Space sizing and evaluation in **Nav2 multi-AMR** systems. The F/S/R geometry is literature-based, **not newly invented**, and no human-subject comfort scores were collected. `predicted_comfort` is a model prediction from observed simulated proximity and published curves, **not a subjective rating**.

Falsifiable hypotheses: Smaller scales may improve navigability but reduce predicted comfort through closer encounters; larger scales may maintain distance but increase path length, protective stops or failure; intermediate `alpha=0.70/0.80` *may* balance both. `0.70/0.80` must **not** be baked into maps, scoring, selection, figures, or runtime stop conditions as the required winner. Valid outcomes include 0.60/0.90/B2 winning, no safe policy, or statistically indistinguishable policies.

Five Stage 3 policies use **one global alpha across states**, but state-dependent template ratios are different. This design does NOT isolate individually optimized *state-wise alpha*; such a claim needs a separate preregistered sixth ablation. Keep this distinction in the paper.

## 2. Stage 1 is historical reference only

**NO new Stage 1 config generation, no Stage 1 simulation, no Stage 1 report rerun or results mixed with new samples.**
Use `R_ref=0.95 m` from abbreviated 2026-11-26 Nav2-only Stage 1 as a **fixed preliminary engineering normalization reference** for B2 and B3 in both final stages. Do NOT call 0.95m a statistically confirmed population Navigation Upper Bound or a comfort validation. Do not silently change `R_ref` after viewing final data. Legacy 0.90m came from Gate-enabled run and is excluded.

## 3. Fixed control, safety, reference template, and score

Same on all AMRs (`AMR1/a1/a2`):
human pose, heading and state (controlled simulator `perception.source=tf`, `humans.state_source=ground_truth`) → policy-driven Comfort costmap mask (when present) → Nav2 global/local costmap → NavFn planner + DWB controller / native Nav2 recovery → `cmd_vel_safe` → Collision Monitor → `cmd_vel` → simulator.

- `control.mode=nav2_only`, `comfort_space.costmap.buffer_width=0.10`, finite mask value 98; physical collision monitor, obstacle layer, footprint, maximum velocity, default Nav2 recovery unchanged between all cases. `comfort_gate` not launched; no custom `yield/evade/backoff/off_lane/traffic-rule`; Nav2 builtin behavior-server recovery stays ON.
- Use the **same source/station/map/worker motion/timing/3-AMR mission/NAV2/safety settings** for each matched seed and every policy. Only policy and its mask F/S/R radii change. Validate merged config and actual ROS graph at EACH launch, including `cmd_vel_safe` publisher Nav2 only and `cmd_vel` publisher Collision Monitor only. Runtime violations → INVALID with evidence, never success.
- B2: fixed **circular** mask r=0.95m. B3: three reference F/S/R axes scaled uniformly: `E_state(1) = 0.95 * E_raw_state / max(E_raw_state)`; `E_state(alpha) = alpha * E_state(1)`, with `station_engaged raw(F,S,R)=(0.80,0.70,0.60)`, `stationary=(0.80,0.90,1.05)`, `moving=(1.20,0.90,0.90)` at 1.0 m/s. Moving heading follows velocity; static states body heading. `min_radius=0.30m` policy must not clip actual sizes; assert exact shapes across all three AMRs.
- Compute predicted Comfort from **ACTUAL logged closest human–AMR center distance, actual bearing and state** with `comfort_core/comfort_reference.py`; do not use radius or alpha itself as a fake measured Comfort. Primary target `C*=5.0`; report `4.5` and `5.5` sensitivity. Note curves refer to different robot-speed conditions (~0.35m/s) vs simulator and station-engaged is a transfer from a standing-person reference; label `out_of_reference_domain`. Preserve unavailable/invalid scores as missing rather than auto-pass.

## 4. STAGE 2 — only five B3 Scale conditions, n=750

**Purpose:** For each Human State/approach, examine actual clearance, predicted Comfort and Nav2 feasibility across chosen scale range; verify the earlier 0.50–0.60 engaged-rear comfort transition where observed. Stage 2 is **not** a comprehensive search for a universal human-comfort minimum (0.45 and 1.00 are NOT included).

| Context | Human State | Intended pass | Policies | MAIN n |
|---|---|---|---|---:|
| ST-F | Stationary | Front | B3 alpha .50/.60/.70/.80/.90 | 5 × 30=150 |
| ST-S | Stationary | Side | same | 150 |
| ST-R | Stationary | Rear | same | 150 |
| EN-R | Station Engaged | Rear of active work fixture | same | 150 |
| MV-P | Moving | physically relevant counterwalking passing | same | 150 |
| **Stage 2 total** | | | **5 contexts × 5 alpha × 30 launches** | **750** |

- **No B1 or B2 for final Stage 2.** They remain historical pilot references and B2 returns as final Stage 3 comparator. `alpha` grid = **0.50,0.60,0.70,0.80,0.90 exactly**, no 0.45 or 1.00.
- For valid main execution, first conduct a **policy-neutral geometry pilot**, freeze each physical geometry and required encounter direction, committing configs/hashes BEFORE 30-launch main. It is methodologically OK to test candidate corridor widths, realistic lane gaps, station fixture clearance. Pilot must not choose layouts specifically to make alpha .70/.80 best. ST-F previously unidentifiable, MV-P actual distance response only .074m; make physically meaningful scenes and record `UNIDENTIFIABLE` honestly if still not sensitive.
- EN-R previous config had `stations.enabled=false` despite `station_engaged` state. Add a real geometrically and semantically meaningful workstation object/zone, and keep identical fixture in all five policies; state tag alone does not establish worker operation realism.
- Stage 2 trial must reach goal, be contact-free, have valid declared Human State + pass side/direction, and predicted Comfort >=5.0 to be a **joint pass**. Raw `navigation_complete`, `safe_nav_complete`, `joint`, `contact`, `collision_monitor_stop`, `closest_center_m`, `closest_footprint_clearance_m`, `predicted_comfort`, travel time/path length/replans must be logged separately. Report n=30 per cell and Wilson 95% CI; matched config/replicate IDs and passage-event logs.
- If nav/joint empirical rate >= .80, can describe **operationally passing scale in the TESTED range**; a comfort lower bound `IDENTIFIED_BRACKET` requires adjacent lower tested alpha whose **comfort fails while navigation succeeds**. If minimum tested .50 passes, `FLOOR_LIMITED_BOUND_NOT_OBSERVED`; if geometry inadequate/response<0.10m, `UNIDENTIFIABLE_GEOMETRY`. Do not relabel .50 as proven minimal comfort distance or .60 as statewise optimum.
- 30 matched parameterized scenario seeds per context (e.g., predeclared path/heading/speed jitter if valid); if identical physical geometry repeated 30 times instead, call them 30 runtime repeats, not 30 independent human environments. Vary nothing across policies for a matched context/seed. Record all observations including failures.

## 5. STAGE 3 — five total policies, n=150

**Purpose:** 8 Human workers with stationary/moving/station_engaged states, 3 fully independent Nav2 AMRs. Five policies across **fresh holdout seeds 1001–1030**, same seed shared across policies, 30 launches per policy.

| Case | Policy | MAIN |
|---|---|---:|
| C1 | B2 circle r=.95m | 30 |
| C2 | B3 uniform alpha=.60 | 30 |
| C3 | B3 uniform alpha=.70 | 30 |
| C4 | B3 uniform alpha=.80 | 30 |
| C5 | B3 uniform alpha=.90 | 30 |
| **Stage 3 total** | **5 policies × 30 matched seeds** | **150** |

- Predefine 10 nominal factory scenarios, 10 workplace bottleneck encounters, 10 moving-crossing scenarios, balanced across 30 holdouts; include all three Human States where applicable. Scenario/collision-contact safety must be physically credible. Pilot/freeze geometry/timing **before** any main result, not adjusted to favor a policy. Legacy seeds101–130 & Gate-based `261125_final_stage3` are **not** reused.
- Counterbalance/interleave policy order per matched seed; old Stage 3 had 11/30 identical-mask seeds disagree because of asynchronous Isaac/ROS simulation, so separately run duplicate-control diagnosis on prespecified seeds/configs (NOT counted toward 150) to quantify runtime noise. Record hardware, driver, run wall time, sim-time, ROS graph, randomized execution order.
- **Primary joint system success:** all 3 AMRs finish assigned missions AND **no human–AMR physical contact** AND all required predeclared scorable Human–AMR encounters satisfy predicted Comfort >=5.0. Encounter denominator/coverage must be explicit; no encounter or unscorable curve is `NOT_SCORABLE`, not automatically passing. Report coverage and sensitivity; do not exclude difficult encounters.
- Secondary results: all-robot mission completion (nav-only), safe mission completion, per-robot completion, contact fraction/duration, Collision Monitor STOP, Nav2 abort & comfort-mask no-progress and robot–robot blocking, actual closest person center and footprint clearance, per-state Comfort median/P10/distribution and fraction >=C*, Travel Time (successes only **AND** capped all-trial, including failures), Path Length, idle duration (not `yield`), replans, n/30 and Wilson CI. Contact cannot be hidden because nav goal succeeded. Mask-relative intrusion is not a fair across-policy Comfort effect.
- Precommit winner-selection rule: safety and reliable scorable comfort first (empirical no-contact <=1/30 for each policy, nav completion >=.80, joint >=.80 and min coverage threshold set before run); if none qualify, `NO_ACCEPTABLE_SCALE`. For qualifying policies compare observed joint first; ties/inconclusive -> Pareto set and predeclared capped mission-time tie-break, with paired McNemar/CI and paired numeric bootstrap; multiple-testing correction/clearly exploratory labels. **Intermediate alpha is a falsifiable hypothesis.**
- Cases use **common alpha** across states; independent optimized state-wise alpha claims require optional preregistered sixth condition (NOT part of these 150).

## 6. Mandatory fixes, integrity tests, and reporting

1. In `comfort_core/final_metrics.py`, previous `gate_yield_s=hold_s` and `NORMAL_YIELD` can mislabel Nav2 normal stopping as a live custom yield. Correct new `nav2_only` semantics, preserve legacy backward compatibility.
2. Audit `comfort_core/comfort_reference.py` clearance-matched secondary halfwidth sign vs documented `d_equiv=d_cc-hw_current+hw_reference`, unit tests. Do not silently change primary old comfort metric.
3. Every Stage 2/3 policy must have the correct filter mask consumed by each AMR, and real human–AMR contact and protective stop events logged independently of Nav2 action status. Do not disable safety or physics to achieve higher nominal completion.
4. All fixed seeds/configs must be reproducible (except known runtime scheduling noise); config/signature/commit hashes recorded, complete manifest, idempotent resuming. Invalid infrastructure launches and retries recorded with objective classification; mission failures/contact retained in denominator. `n=30` for **every** main cell, no 5/10-run shortcut.
5. Historical `results/batches/261125_*`, `261126_*` immutable; write versioned new code/configs/results `configs/final_nav2only_stage23_900/`, `scripts/final_stage23_900_*.py`, `results/batches/*stage23_900*` or equivalent.
6. Deliver full `per_trial.csv`, `per_encounter.csv`, `STAGE2_RESULTS.md/csv`, `STAGE3_RESULTS.md/csv`, figures (Scale vs observed distance/predicted comfort; five policies nav/safe/joint/time/path/contact/Pareto), CI, raw logs, controls/config hash, exact reproduce/resume commands, missing/invalid counts and limitations; then Korean thesis-ready design/results/conclusion **each exactly 2 paragraphs × 3 sentences** based only on ACTUAL data (previous mock result tables are placeholders).

## 7. Expected time-to-completion (ETA) REQUIRED IN CLAUDE'S REPORT

The user explicitly wants **a calendar completion time**, not an unsupported generic "it will take some hours." Claude MUST use the actual simulator machine and **Asia/Seoul timezone** and report:
- At the start: `started_at_kst`, hardware/GPU, simulation mode and number of concurrently executable trials (usually ONE actual Isaac Sim worker/GPU, do not assume parallelism).
- After a prespecified **real timing sample**: at least **3 representative Stage 2 smoke/pilot launches + 3 Stage 3 smoke/pilot launches** with full simulator init/teardown, ROS startup, successful and time-out behavior if encountered. Log `wall_seconds` per trial, startup overhead, succeeded vs timeout; don't substitute simulated-time seconds as wall time.
- Compute `N_stage2_remaining=750 - completed_valid_main_stage2`, `N_stage3_remaining=150-completed_valid_main_stage3`. Each stage ETA = remaining × measured **per-stage wall-clock throughput including startup/shutdown, failures and timeout** + fixed pilot/smoke/analysis time + explicit retries/infra margin. If measured parallelism >1, use measured completed-trials/hour throughput, not naive division by workers.
- Report **three scenarios**: optimistic / central / conservative using observed e.g. P25 / mean-or-median / P90 wall times, with realistic re-run and analysis overhead. `ETA_kst = now_KST + estimated_remaining_wall_duration`, specify concrete **YYYY-MM-DD HH:mm KST**, time remaining in h/min, remaining trials by stage, sample count, assumptions, and variability. Display Stage 2 end, Stage 3 end, and overall final analysis end. If insufficient Stage 3 timing samples, show provisional assumption and clearly label uncertainty until measured.
- Update ETA **after each completed batch (e.g. every 30 main trials)** using actual cumulative throughput and committed DONE counts; report any significant drift and root cause. `progress_{timestamp}.json` + `ETA_REPORT.md` include `progress_percent`, `completed/invalid/retry`, rate/trials per hour, run start, elapsed, expected end in KST. No fabricated timestamps or promises of unattended future delivery; only report measured run status and projections. Clearly distinguish **estimate** from scheduled guarantee.
- Optionally historical Gate-enabled multi-AMR trials took ~220–395s wall time, but this is NOT a reliable prediction for NEW Stage 2 single-AMR or Nav2-only Stage 3. Historical values may be used only as explicitly rough initial context before real timing samples.

## 8. Execution order and completion criteria

1. READ this protocol + current `docs/CLAUDE_FINAL_STAGE2_STAGE3_ONLY_PROMPT_20261127.md` (after latest update), `AGENTS.md`, existing Nav2-only code and reports, then build **NEW 900-trial manifests & pipelines** without changing old data.
2. Offline regression tests, ROS graph/safety smoke; independent Stage 2 geometry pilot/freeze and Stage 3 factory pilot/freeze, commit policy-neutral geometry + all 900 launch manifest/checksums BEFORE gathering main results.
3. Execute Stage 2 750 main and Stage 3 150 main with `control.mode:nav2_only`. Every trial records wall time/actual runtime status. Pilot, smoke and duplicate diagnostics are additional, never subtracted from 900.
4. Generate `ETA_REPORT.md` **as soon as real timing samples exist**, then regularly update ETA/progress in KST; execute final analysis with actual data only.
5. If local Isaac Sim/ROS2 cannot launch, implement + test offline, report `MAIN_EXECUTED=0` and "ETA not measurable here", plus exact workstation commands and how to get measured ETA. Do not claim a predicted completion timestamp from no observed throughput.
6. DO NOT rerun Stage 1 and DO NOT include B1/B2 in Stage 2. **Final nominal MAIN = Stage2 750 + Stage3 150 = 900.**

**This is a reproducible, falsifiable experiment design, not a recipe to make alpha=.80 win.**
