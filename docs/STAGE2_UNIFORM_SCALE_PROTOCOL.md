# Stage 2 — State-wise Uniform Scale Sweep and Comfort–Navigation Trade-off

**Status (2026-11-26): DESIGN SPECIFICATION ONLY — not executed, not validated by human subjects.** This protocol supersedes the previously proposed *independent front/side/rear sweep* in [STAGE2_LOWER_BOUND_REDESIGN.md](STAGE2_LOWER_BOUND_REDESIGN.md) and the old simulator generator plan in [STAGE2_SIM_PLAN.md](STAGE2_SIM_PLAN.md) for upcoming **new experiments**. Historic data and Stage 1/3 results remain untouched.

**Research question:** Given a state-dependent comfort shape, how far can its **overall size** be reduced while preserving a prespecified literature-predicted human comfort requirement **and** completing navigation reliably? If the two requirements do not trade off within tested conditions, the study **must NOT claim to have identified the comfort lower bound**.

## 1. Three distinct quantities; scope of claims

1. \`R_nav_upper=0.90 m\`: Stage 1 **static circular B2** operational upper bound in a specific multi-AMR factory scenario. It is the design normalization anchor for Stage 2; not a proven universal directional AMR or human safety upper limit.
2. \`D_req(H,theta;C*)\`: inverted Neggers et al. (2022a/b) predicted **human–robot centre-to-centre distance** meeting an assumed comfort target \`C*\`. Literature-extrapolated when applied outside the source study's tested robot, speed, state or direction. In particular, \`station_engaged\` has no task-specific rating curve.
3. \`alpha^*_H\`: smallest feasible **uniform shape-scale factor** for state \`H\`, estimated from ACTUAL simulated completed passages, NOT from the configured comfort mask or from size-only scoring. Can be declared bounded, unresolved or unidentifiable, as described below.

Safety is outside the objective: Nav2 collision monitor, footprint, obstacle map and protective fields MUST remain unchanged and enabled.

## 2. Shape normalization: fixed ratios within each human state

Start with \`configs/common.yaml\` engineering shapes. For \`moving\`, use the existing mapping **evaluated at the predeclared reference walking speed \`v_ref=1.0 m/s\`** for Stage 2 main sweep. Keep velocity heading; freeze the template magnitudes during this primary experiment. This prevents changing direction ratios with unknown realized animation speed. Speed-dependent extension is a **later sensitivity / Stage 3 policy issue**, not established by the 2022 Neggers fit.

Define \`E^raw_H=(F,S,R)\`, \`k_H=0.90/max(E^raw_H)\`, \`E_H(1)=k_H E^raw_H\`, \`E_H(alpha)=alpha E_H(1)\`. Apply alpha identically to ALL three axes, not individually. Retain the offset ellipse construction \`a=(F+R)/2\`, \`b=S\`, \`offset=(F-R)/2\`, and heading by human state.

| State | Raw template F/S/R, m | Normalized alpha=1 F/S/R, m | Evidence |
|---|---|---|---|
| \`station_engaged\` | 0.80 / 0.70 / 0.60 | 0.900 / 0.7875 / 0.675 | Existing engineering hypothesis; **rear contraction not empirically verified** |
| \`stationary\` | 0.80 / 0.90 / 1.05 | 0.685714 / 0.771429 / 0.900 | Directional ordering supported by Neggers standing study; exact template lengths not empirically determined |
| \`moving\` at 1.0 m/s | 1.20 / 0.90 / 0.90 | 0.900 / 0.675 / 0.675 | Forward elongation motivated by Gorrini locomotion and Neggers passing evidence; exact ratios assumed |

**Check actual \`ComfortShape.bounding_radius <= 0.900001\` for each normalized template.** The theoretical reference is the human-centred circle; policy masking grows shapes for robot-centre path planning by the robot half width, so do not compare the grown mask against 0.90 m.

**Floor/clipping warning.** \`StateDirectionalComfortPolicy._squeeze\` currently computes \`floor + scale*(raw-floor)\` when \`aisle_scaling\` is enabled, and \`max(value, min_radius)\` after that. This does **not** preserve shape ratios if floor is nonzero or any extent clips. Generate direct per-alpha directional extents, leave \`aisle_scaling.enabled: false\`, and verify \`E_H(alpha)/E_H(1) == alpha\` per axis. Keep \`comfort_space.min_radius=0.30 m\` unchanged. For these templates, \`alpha >= max_H[0.30/min(E_H(1))] = 0.44444...\` avoids clipping, so the last **prespecified** candidate is 0.45. Do not go below 0.45 without explicit study redesign and an integrity check; never silently clip and call it ratio-preserving.

Primary coarse scale ladder: **[1.00, 0.90, 0.80, 0.70, 0.60, 0.50, 0.45]**; add 0.05 steps *within* 0.45–1.00 around a transition in a separately identified fine stage. Finer spacing is conditional, not prespecified proof of a threshold. Static B2 at \`r=0.90\` and B1 (none) are separate baselines and share EXACTLY the same scenarios and geometry.

## 3. Factors, scenarios and identification limits

Primary independent variable \`alpha\` (one scalar multiplies the entire shape). Within each trial only the **target person's** known state is present; maintain the same three-state policy definition so the Stage 3 interface uses separate \`alpha^*_station_engaged\`, \`alpha^*_stationary\`, and \`alpha^*_moving\`. Use a scripted/declarative state when possible; log the actual observed state so state-transition contamination can be diagnosed.

Core calibration contexts, one AMR + one person:

| ID | State | Encounter | What is inferable |
|---|---|---|---|
| ST-F | stationary | pass in front of the person's body direction | Directional standing reference |
| ST-S | stationary | pass at side, test left/right in balanced pilots | Directional standing reference |
| ST-R | stationary | pass behind person | Directional standing reference |
| EN-R | station_engaged | pass behind person attending to a station | **Stationary-rating transfer**, task-specific comfort UNKNOWN |
| MV-P | moving | head-on approach with a lateral passing event | Moving passing reference, only within documented measurement scope |

### Moving head-on passing MUST start in separated lanes (2026-11-26 user revision)

**Supersedes the old same-centreline MV_pass geometry.** The old `scripts/stage2sim_plan.py:env_overlay()` assigns the SAME `y` to `humans.a.position/route` and `navigation.start/goal`, so a moving person and AMR approach directly along one line. The researcher explicitly rejects this smoke-test layout.

- Keep an **oncoming lateral pass** (opposite x travel directions) but use **two distinct parallel lanes**. In the **top-view recording**, arrange the pedestrian on the visual **left** lane and the AMR on the visual **right** lane, not nose-to-nose on one centreline. Since the camera image orientation is not guaranteed by the world coordinate convention, verify left/right in the rendered top view and swap lane coordinates if necessary.
- Use separate config keys `human_lane_y` and `amr_lane_y`. For example only, person `(+7.0,+0.40) -> (-7.0,+0.40)`, AMR `(-5.5,-0.40) -> (+5.5,-0.40)`: initial/pass-by lateral centreline separation = 0.80 m. **The y-sign/image-left correspondence must be validated, not presumed.** The actual lane gap is chosen in a pilot, not fixed by this example.
- Pilot centreline separations `{0.65, 0.80, 1.00}` m (engineering candidate set, NOT observed comfort thresholds). Keep within the corridor and check wall/protective-field feasibility. Note the human radius is currently 0.25 m and AMR half width 0.24 m; very close lanes may cause protective stop even before comfort is assessed. Do not change safety settings to force passage.
- **Freeze the selected two lane coordinates, starts/goals, and corridor geometry across alpha/B1/B2.** Vary only the Comfort Space policy. Ensure the walker and robot actually pass side-by-side in the encounter region, with a noncontact minimum gap, rather than only approaching opposite along the same line. Reject wrong-way passages or noncompletion as joint failures.
- Smoke/video acceptance: screenshot or short top-view clip must show (1) the walker and AMR originating in different lateral lanes, (2) pedestrian visually left and AMR visually right, (3) oncoming movement and lateral passing without initial same-line collision, (4) a measured actual closest centre/footprint distance and its response to alpha. Document camera orientation, actual lane coordinates, nominal lane gap and achieved minimum clearance.
- **Identification caveat:** if the fixed lane geometry is so separated that all alpha values generate essentially the same actual passing distance/comfort score, label `UNIDENTIFIABLE_GEOMETRY`. Do not select the smallest alpha just because the two lanes never interact; re-pilot **fixed** lane separation and width under a documented new version instead of secretly changing geometry between scale conditions.

Optional preregistered transfer/robustness contexts: EN-F and EN-S (task geometry/cost, **not new human comfort evidence**); opposing multi-AMR stress block after calibration (must not be used to retune alpha). Actual overtaking is currently **INFEASIBLE** with the walker (~1.04 m/s) faster than AMR maximum (1.0 m/s); log as unsupported, do not fake or silently label it passing.

**Do not independently shrink F/S/R.** A scenario's encounter direction is a *test context*, not a parameter-change axis. \`alpha_H\` must be selected across all preregistered relevant contexts for that state. For stationary, \`alpha^*_{stationary}\` must pass ST-F/S/R. For engaged and moving, the result's claim must be explicitly restricted to EN-R and MV-P respectively unless additional contexts are run; unsupported cells remain transferred hypotheses.

For engaged, include an optional separate hypothesis ablation \`engaged uses stationary ratios\` at the matched alpha to test geometric effects. Do not assert the current compact rear shape is validated by task literature. Keep this ablation OUT of primary alpha calibration.

## 4. Pilot: ensure an identifiable comfort–navigation trade-off

**Critical issue with old generator:** \`scripts/stage2sim_plan.py:tight_width\` changes corridor width as a function of radius. This is confounded; replace with FIXED geometry per scenario × corridor variant across ALL alpha/B1/B2 comparisons.

Two fixed corridor variants per scenario: \`tight\` (primary, pilot chosen) and \`wide\` (secondary transfer). Select widths in a **separate pilot** from a preregistered candidate set, for example static scenarios 2.0/2.2/2.4/2.6 m and moving 2.6/2.8/3.0 m. These are **pilot candidate dimensions, not confirmed feasible values**. Freeze corridor width, human-to-wall offset, AMR start/goal, human route and speed, facing, wall, station layout, Nav2, costmap buffer (0.10 m), and all protective controls inside each condition comparison. Set the wide variant independent of alpha.

Pilot conditions at **B1, B2(0.90), alpha=1.00,0.70,0.45**, 2–3 independent launches per candidate width. Pilot **exclusively** determines (i) whether the AMR can physically traverse at some alpha without wall-related protective-stop artifacts and (ii) whether measured actual human–AMR distance responds to alpha (distance/clearance sensitivity). **Predeclare a minimum practical observed response**, e.g. >=0.10 m change in median achieved centre distance between alpha=1.00 and 0.45 among completed comparable passages; this is an engineering detectability rule, not a finding or a statistical significance test. Publish the whole pilot-width matrix, including widths rejected. Do not choose width because it gives the desired comfort-threshold crossing.

If every feasible width gives flat \`d_actual(alpha)\`, classify **UNIDENTIFIABLE_GEOMETRY** and revise the controlled encounter geometry (e.g. bottleneck/entry constraints) in a NEW, documented pilot version. Do not simply accept the smallest alpha as a human comfort lower bound. A wall-only collision monitor deadlock is a geometry artifact, not comfort evidence.

## 5. Metrics and scoring, including incomplete passes

Compute human–AMR centre distance from logged poses and predicted comfort from **published** reference parameters only; do NOT feed the configured alpha, mask, radius or predicted cost into the comfort reference formula. Keep scoring identical for B1, B2 and adaptive at a given human state and encounter.

- **Passage validity/completion:** actual AMR traversal of intended side of human and reaching goal; actual closest point belongs to intended relative sector; no collision/contact; state valid. Nonpassage, stalled robot, wrong-side detour, invalid state, etc. count as **joint failure**, not as a very comfortable passage.
- **Primary human-reference proxy:** \`predicted_comfort_at_closest\` for COMPLETED intended encounters, scored on the original **centre-to-centre** distance and actual relative bearing. Predeclare \`C*=5.0\` on a seven-point scale as a **study assumption**, not a universal comfort threshold. Sensitivity targets 4.5 and 5.5. Flag whether closest distance lies in the reference's sampled interval and whether AMR/relative speed is outside the study's ~0.35 m/s robot-speed context.
- **Secondary:** minimum predicted score during the encounter, time below target, exact human-centre to nearest AMR-footprint-edge clearance (not \`d_cc-0.24\` for all relative robot headings), min TTC (identified kinematic estimate only), elapsed time, hold, protective stop, nav class, Comfort-induced deadlock, actual min centre distance, speed at closest, relative sector and shape extents recorded on each tick.
- **Separate rates** with denominators: navigation completion rate; conditional (completed encounters only) comfort pass rate; and **joint rate across all valid launches**.
- Treat simulator/ROS infrastructure failure per existing retry policy: exclude infrastructure-invalid attempts only; human-contact, deadlock, wrong-side passage, abort and protective-stop terminal failure stay in denominator. Missing or unscorable observations must NOT be silently classified comfort-compliant.

Define per-launch \`J_i(H,W,alpha,C*) = 1\` iff (AMR finishes intended human passage/mission, no contact, valid encounter direction/state, and literature-predicted \`C>=C*\` at closest point). All other outcome launches: \`J_i=0\`. Keep protective-stop transient vs terminal separate; report both, and require no terminal protective stop for \`J=1\`. This is a predicted-comfort-based *operational* indicator, **not subjective comfort**.

## 6. Nonmonotone feasibility and selection (predeclare)

Proposed main operational acceptability: \`joint_success_rate >= 0.80\` AND \`nav_passage_rate >= 0.80\` for every PRIMARY preregistered scenario of that state (both are engineering thresholds). Report count/n and Wilson 95% CI; 0.80 observed at n=30 is NOT high-confidence proof true success probability >=0.80. Add 0.90 joint-rate threshold as sensitivity. Use **all** launched valid trials in the joint denominator.

Selection per (H, W, context): among **actually tested** alpha, find smallest alpha satisfying both rates, **without assuming bigger alpha must pass**; bigger comfort spaces may cause deadlock, so overall joint feasibility can be nonmonotonic. Validate adjacent smaller size and selected size using independent final launches. Aggregate \`alpha^*_H\` as the smallest tested alpha that passes EVERY primary context for H; if no single alpha satisfies all contexts, report **NO_COMMON_FEASIBLE_ALPHA**, not a fabricated state bound.

**Crucial identification status, separate from numerical policy choice**:
- \`IDENTIFIED_BRACKET\`: measurable change in achieved clearance with alpha, comfort-limit violation appears at a smaller tested alpha (or above a feasible one) and a feasible candidate above it is confirmed, with navigation performance separately assessed. Report tested bracket, do not claim sub-grid accuracy.
- \`FLOOR_LIMITED\` or \`BOUND_NOT_OBSERVED\`: alpha=0.45 still meets predicted comfort and joint criterion; you only found a *smallest admissible tested policy*, **not the comfort threshold**.
- \`UNIDENTIFIABLE_GEOMETRY\`: measured achieved clearance does not respond materially to scale in pilot; no lower-bound claim.
- \`NO_FEASIBLE_ALPHA\`: every candidate fails navigation and/or predicted comfort jointly; consider behavioral mitigation (slow/wait/yield/replan), not a size-only solution.
- \`OUT_OF_REFERENCE_DOMAIN\` (qualifier): scorer uses distance, speed, direction or task context not validated by the cited paper. Do not use it as an unqualified human-comfort finding.

If a selected alpha satisfies joint success but there is NO comfort threshold crossing, report **operationally feasible minimum tested alpha** separately from a comfort-based lower bound. Stage 3 can use an operational policy candidate labeled as such if the protocol says so, but cannot claim a measured human comfort threshold.

## 7. Repeats, comparisons and costs

Recommended staged design to limit unnecessary simulation:
1. **Pilot (diagnostic only):** candidate widths × B1, B2, alpha=[1.0,0.7,0.45] × 2–3 launches; do not pool with final evidence.
2. **Coarse screening:** primary five contexts × each frozen \`tight\` width × 7 alpha values + B1/B2, **10 independent launches per cell**; approximately 450 trials (5×9×10), excluding pilot. Use predeclared 0.05 step fine search ONLY where conditions change.
3. **Confirmatory:** at least **30 new launches** for selected alpha and immediately smaller candidate for each context, plus matched B1/B2 baselines where relevant; include wide corridor as held-out geometry robustness, not as an alpha-tuned dataset. Same scenario/seed inputs across policies, interleaved launch order; asynchronous Isaac Sim/ROS outputs can differ even for same seed.
4. **Optional Stage 2B:** opposed multi-AMR narrow corridor with frozen state-wise candidate scales, used to test transfer to Stage 3-like interactions, not recalibrate on the same seeds.

For fixed alpha, trace all three realized F/S/R values, geometry IDs, matched seed, launch ID, build/version/config hash, person state/heading, actual speeds, and scoring-reference flags. Never overwrite \`results/batches/261125_*\`, \`261126_stage2sim_*\`, or old \`configs/stage2sim/\` generated assets. Use a new versioned prefix, e.g. \`configs/stage2scale/\` and \`results/batches/261126_stage2scale_v1/\`.

## 8. Plots and final artifact contract

- \`alpha\` → median/quantiles of actual achieved **centre distance** (by context; scatter actual per-run).
- \`alpha\` → median and individual-run **literature-predicted comfort** on completed passages only, horizontal \`C*=5\`.
- \`alpha\` → navigation passage success, joint pass success, Comfort-induced deadlock, Wilson 95% CIs, denominators.
- Comfort–navigation feasible interval / Pareto display with distinguishable \`IDENTIFIED\` vs \`FLOOR_LIMITED\`.
- Static B2 and no-Comfort B1 reference marks, same geometry and scoring.
- Report the actual distribution of closest-point sectors and speeds, and annotate unsupported model transfers.

Expected files: \`manifest.tsv\`, \`plan.json\`, \`analysis/scale_trials.csv\`, \`analysis/scale_summary.json\`, \`analysis/STAGE2_SCALE_REPORT.md\`, \`analysis/figures/\`, \`analysis/selected_state_scales.json\`, \`analysis/B3O_SCALE_CANDIDATE.yaml\`. The config should embed provenance and unresolved/unsupported flags. **Do not silently put invented per-direction lower bounds in all nine cells.**

## 9. Implementation acceptance tests

1. At alpha=1.00 the normalized shape max (including \`ComfortShape.bounding_radius\`) is <=0.900001 m; raw ratios preserved; moving reference shape matches 1.0 m/s template; at alpha=0.50 each F/S/R is exactly half of alpha=1 within numerical tolerance; no axis clips at alpha=0.45.
2. Same scenario×variant at alpha=1.0, 0.7 and 0.45, B1 and B2: identical physical geometry, human scripts and Nav2/safety parameters; only policy/extents differ.
2a. MV-P specifically: distinct pedestrian/AMR lateral lanes for both start and goal, opposite travel directions, required visual left/right arrangement in a real top-view smoke capture; lane separation and geometry identical across alpha, B1 and B2; measured passing clearance must be reported.
3. Artificial log: deadlocked AMR far from human -> **joint failure** even if predicted comfort=7; passing near human with C below target -> **joint failure**; successful completed passage with C above target -> success.
4. Synthetic nonmonotone ladder: alpha=1.0 navigation fail, alpha=0.7 success/comfort pass, alpha=0.45 comfort fail -> **select 0.7** (no all-larger pass requirement).
5. Synthetic flat-distance response at all alpha -> \`UNIDENTIFIABLE_GEOMETRY\` or \`BOUND_NOT_OBSERVED\`, never "comfort lower bound 0.45".
6. If selected scale works for ST-F and ST-S but fails ST-R, there must be **no** reported common stationary alpha at that scale.
7. Reference score unchanged between two policy labels with identical logged centre-distance/angle; scores are not computed from configured radius.
8. Infra-only failed startup is retried/excluded with audit trail; real motion failure and contact remain in analysis.
9. No modification to Stage 1/previous Stage 3 experimental results, physical safety layer or custom Nav2 controller code. \`--check\` and unit tests run before any expensive Isaac Sim experiments.

## 10. Claim discipline and literature

Terms: **literature-predicted comfort**, **simulation-derived minimum operational scale**; no claim of measured human acceptance. The reference human–robot curves from Neggers et al. (2022a/b) were measured at robot ~0.35 m/s; \`station_engaged\` uses stationary transfer. Gorrini et al. (2014, 2019) motivate walking state and frontal reach but do not quantitatively set industrial AMR front ratios; Kinoe (2015, 2018) and Cui (2023) motivate task-context sensitivity but do not prove rear shrinkage for a lone worker and unrelated AMR. See [PAPER_RELATED_WORK_MODELING_DRAFT.md](PAPER_RELATED_WORK_MODELING_DRAFT.md).

## 11. Do not start the full study before these decisions are recorded

- [ ] \`C*=5/7\` and sensitivity targets 4.5/5.5 approved as **operational choices**.
- [ ] 80% primary joint + nav rate and 90% sensitivity documented with uncertainty approach.
- [ ] Pilot corridor-width set and identification criteria frozen **before** reading pilot outcomes; pilot outcomes published, no favorable outcome cherry-picking.
- [ ] Confirm moving animation character \`a\`, actual speed and achievable robot reference-speed variation; use speed-domain qualifiers.
- [ ] Confirm \`station_engaged\` shape explicitly treated as engineering hypothesis.
- [ ] All unit and smoke tests pass, config files and prior experiments preserved.
