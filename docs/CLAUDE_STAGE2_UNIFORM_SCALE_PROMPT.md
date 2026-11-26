# Claude implementation handoff — Stage 2 uniform-scale experiment (copy/paste prompt)

You are implementing the next Stage 2 experiment in the \`hun7407-lgtm/Comfort_AMR\` repository, active branch \`feature/b1-baseline-and-yield\`. The experiment framework is Isaac Sim 5.1 + ROS 2 Jazzy + Nav2. **Implement and test code; do NOT run the expensive full simulation campaign without explicit user approval.**

**Read these files before writing any code:**
1. \`AGENTS.md\` (nonnegotiable constraints and frozen Stage 1).
2. **Current authority:** \`docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md\`. The older \`docs/STAGE2_LOWER_BOUND_REDESIGN.md\` proposed independent axis sweeps, which the researcher **rejected**; \`docs/STAGE2_SIM_PLAN.md\` is a previous prototype. Do not follow contradictory instructions in those older experimental plans.
3. \`docs/PAPER_RELATED_WORK_MODELING_DRAFT.md\`, \`docs/FINAL_261125_PLAN.md\` (historical Stage 1/3; don't overwrite).
4. \`configs/common.yaml\`, \`comfort_core/comfort_space.py\`, \`comfort_core/shapes.py\`, \`comfort_core/comfort_reference.py\`, \`comfort_core/human_state.py\`.
5. \`scripts/stage2sim_plan.py\`, \`scripts/stage2sim_report.py\`, \`configs/stage2sim/stage2sim.yaml\`, \`scripts/final_run.py\`, \`comfort_core/final_metrics.py\`, \`scripts/trial_logger.py\`.

## Researcher's decided approach

- Stage 1 B2 Static Circle \`R_nav_upper=0.90m\` is our design normalization anchor, **not** an empirically proven human comfort threshold.
- Have **three separately fixed shape templates**: \`station_engaged\` raw F/S/R=(0.80,0.70,0.60), \`stationary\`=(0.80,0.90,1.05), \`moving\` at reference 1.0 m/s=(1.20,0.90,0.90).
- Normalize EACH template separately so its maximum directional extent is 0.90m:
  - engaged \`(0.900, 0.7875, 0.675)\`
  - stationary \`(0.685714..., 0.771428..., 0.900)\`
  - moving \`(0.900, 0.675, 0.675)\`
- Introduce **one and only one swept parameter alpha**, multiplying **all three** F/S/R axes together, preserving every state's ratio and offset-ellipse shape family. Do NOT implement an independent front/side/rear sweep. Keep orientation: moving follows velocity; engaged/stationary follow body heading.
- Primary moving template must not quietly change with sampled walker animation speed; the reference \`v=1.0m/s\` provides fixed F/S/R for primary Stage 2. Log real speed and leave speed-dependent behavior for separate sensitivity only.
- Candidate alpha values: \`[1.00,0.90,0.80,0.70,0.60,0.50,0.45]\`, and 0.05 fine steps near a **predeclared** transition. Alpha below 0.45 makes the existing 0.30m \`min_radius\` clamp spoil ratio preservation; **do not lower min_radius or alter safety**.
- For *shape normalization* verify the true \`ComfortShape.bounding_radius\` is <=0.90 m at alpha=1. Nav2's robot-grown costmap mask is not the reference shape.
- The existing \`StateDirectionalComfortPolicy._squeeze\` default behavior is \`floor + scale*(raw-floor)\`; do not use this with a nonzero floor as an allegedly pure multiplier. Prefer generating **direct per-condition extents** and \`aisle_scaling.enabled: false\`. Unit-test exact ratios at every alpha and absence of clipping.
- Existing task-engaged rear compression is an **unverified engineering hypothesis**. The literature does not establish its human comfort threshold. Never mislabel it as experimentally validated.

## NEW RESEARCHER REQUIREMENT — moving smoke scenario must have two separated lanes (PRIORITY)

The researcher observed that the existing moving smoke-test video places the oncoming pedestrian and AMR **directly in line**, which is not desired. In `scripts/stage2sim_plan.py:env_overlay()`, the existing moving case assigns the SAME `y` to the walker route and AMR start/goal via `lane_y`. **Do not reproduce that geometry** in the new Stage 2 module.

- Make an **oncoming, side-by-side passing** scenario, not a face-to-face, same-centreline potential collision. In the top-view video, the **pedestrian should be in the visually left lane; AMR in the visually right lane**. Use explicit `human_lane_y` and `amr_lane_y` (and preserve different y-values along the intended straight trajectories). Reverse world-y signs if camera orientation requires it; verify actual rendered camera orientation instead of assuming how world y maps to screen left.
- Example initial geometry **for pilot only**: walker begins `(+7,+0.40)`, walks toward `(-7,+0.40)`; AMR starts `(-5.5,-0.40)`, navigates toward `(+5.5,-0.40)`. This yields 0.80 m lateral separation between nominal lane centre lines. The user cares about the observed left/right relationship, not these precise coordinates; do not silently assert they are guaranteed visually correct.
- Treat lateral separations `0.65/0.80/1.00m` as **pilot candidate offsets** and select via physical feasibility and **alpha-to-actual-clearance sensitivity**; do not tune a geometry solely because it forces the desired Comfort score. Keep the chosen separation, corridor width, human speed/route, headings, starts/goals and safety settings **identical for B1/B2/all alpha**. Do not disable/relax protective stop or alter safety to make tight lanes pass.
- Important: too much fixed lateral separation can yield the SAME actual clearance for every alpha. In that case return `UNIDENTIFIABLE_GEOMETRY`, not a Lower Bound; re-pilot a fixed geometry under a new version.
- Add unit/integration checks that `abs(human_lane_y-amr_lane_y)>0`, opposing route x direction and same world geometry across policy/scale variants. Add smoke capture/top-view inspection criteria: two distinct lateral lanes, pedestrian visually left and AMR visually right, no direct same-line encounter, AMR finishes intended pass, measured min centre-to-centre/footprint-edge clearance, and reporter confirms whether distance varies with scale.
- Do **not** edit old `configs/stage2sim/` or overwrite prior smoke results. Implement only in the new `configs/stage2scale/` generator and tests. Include a before/after figure or top-view screenshot from the actual scene **if available**; if not, mark camera verification outstanding and do not claim visual acceptance.

## Required implementation scope

1. Create a **new independent experiment module**: \`configs/stage2scale/\` with new \`stage2scale.yaml\` and generated configs, \`scripts/stage2scale_plan.py\`, \`scripts/stage2scale_report.py\` (or equivalent clear naming); keep old Stage 2 pilot and all Stage 1/3 results untouched. Reuse stable existing runner/logger/reference code rather than rewriting Nav2.
2. The generator must create B1 no-Comfort, B2 static r=0.90m, and B3-state-shape at alpha candidates **in the SAME physical scenario**. Explicitly verify by hashing/comparing loaded configs that for each scenario×corridor group the walls, aisle width, human pose/route/facing/state, AMR start/goal, timing, Nav2, collision monitor and costmap buffer are identical. Only comfort policy/shape settings should change.
3. **Never copy** the radius-dependent \`tight_width(g,r)\` design; it changes the corridor width with alpha. Implement fixed tight and wide widths per scenario. Include a separate pilot over preregistered candidate widths to find a physically navigable, clearance-sensitive condition; publish rejected widths too and freeze each selected width before the confirmatory sweep.
4. Main controlled scenarios: \`ST-F\`, \`ST-S\`, \`ST-R\`, \`EN-R\` and \`MV-P\` (head-on moving-person pass). Add optional EN-F/EN-S as explicit secondary scenarios; **do not label moving overtaking as feasible** without making its human speed slower than AMR and obtaining an adequate comfort-reference model.
5. The **only** scientific comfort score comes from \`comfort_core/comfort_reference.py\` using ACTUAL logged human–AMR centre-to-centre closest distance and relative angle, identical scorer for all policies. The model \`C(d,theta)\` must NEVER receive alpha, configured R, mask or cost as its distance. Primary \`C*=5.0/7\` (researcher's provisional operational target, not a validated universal threshold), sensitivities 4.5 and 5.5. Mark data outside source study's 0.35 m/s robot-speed conditions / tested distance range; \`station_engaged\` is a stationary-reference transfer, not task-subject evidence.
6. Log actual min \`d_cc\`, closest-point direction, footprint-edge distance using the actual oriented AMR footprint (do not use fixed half-width subtraction as exact geometry), actual robot and human speed, passage completion, nav success/deadlock and protective stop, hold and time, per-tick shape F/S/R, relative bearing, comfort at closest and minimum comfort during valid encounter.
7. If AMR **does not complete the intended human pass**, even if it stays very far from the human and the predicted comfort score is high, count it as **joint failure**, NOT as a comfortable success. Wrong-sector detour is a failure/mismatch for that calibration cell. Missing logs or invalid states are not silently counted comfortable. Keep only genuine simulator startup failures separate per existing infra-retry rules.
8. Define \`J=1\` iff completed intended passage + no human contact + correct state/direction + model-predicted C>=5 at closest point. Use all valid launches as denominator. Report \`joint_rate\`, \`nav_completion_rate\`, and \`comfort_pass_rate_given_completion\` separately. Predeclared **provisional** acceptance is joint >=0.80 and nav >=0.80; report n, counts, Wilson 95% CI and 0.90 sensitivity.
9. **Allow nonmonotone feasibility.** Existing \`scripts/stage2sim_report.py:select()\` disallows a smaller feasible scale after larger scale fails. The new selector must NOT use "all larger alpha must pass". Inspect each tested alpha independently; choose smallest jointly feasible alpha for each context and a common feasible alpha for each state across ALL predeclared primary contexts. Never choose an untested interpolated value.
10. **Identifiability status required**: \`IDENTIFIED_BRACKET\` only if achieved clearance responds to alpha and an observed lower-alpha comfort violation brackets a feasible alpha; \`FLOOR_LIMITED / BOUND_NOT_OBSERVED\` if smallest tested alpha still passes; \`UNIDENTIFIABLE_GEOMETRY\` if distance is flat regardless of scaling; \`NO_COMMON_FEASIBLE_ALPHA\` if no alpha works across a state's tested contexts. A smallest operational alpha without observed Comfort degradation is **NOT a discovered human comfort lower bound**. Extrapolated reference models need flags.
11. Pilot uses B1, B2, alpha 1.0 / 0.70 / 0.45 and 2–3 repeats across fixed candidate widths (see protocol). Main screening: 5 contexts × 9 conditions including baselines × 10 repeats = ~450 runs, AFTER pilot width freeze; final ≥30 NEW repeats for selected and adjacent smaller alpha plus matched baselines, wide corridor for robustness. Ensure same seeds across policies, independent launches, meaningful interleaved run order, detailed manifests and versioned configuration provenance.
12. Produce transparent report \`analysis/STAGE2_SCALE_REPORT.md\`, \`scale_trials.csv\`, \`scale_summary.json\`, \`selected_state_scales.json\`, annotated plots and a \`B3O_SCALE_CANDIDATE.yaml\` for later Stage 3. The candidate must identify assumption/transfer status of untested state×direction cells. Do NOT run Stage 3 until the Stage 2 selection is confirmed.

## Critical regression and synthetic unit tests

- normalized templates' F/S/R, \`bounding_radius <=0.90\`, exact \`alpha\` ratio preservation at alpha .50 and .45, \`min_radius=0.30\` unmodified and no clipping;
- B1/B2/B3 configurations have **bitwise/semantic identical physical scenario and Nav2/safety parameters** for the same scenario and seed; only the comfort policy/extents differ;
- deadlock far from human with predicted Comfort=7 gives \`J=0\`;
- completed passage too near with C<5 gives \`J=0\`;
- intended, collision-free completed pass with C>=5 gives \`J=1\`;
- nonmonotone synthetic: alpha 1.0 nav FAIL, 0.7 joint PASS, 0.45 comfort FAIL ⇒ selected alpha 0.7;
- MV-P moving-person and AMR routes have distinct parallel lanes, opposite travel directions, and consistent fixed geometry across B1/B2/all alpha; top-view must confirm pedestrian-left/AMR-right rather than same-y head-on;
- flat achieved distance across all alpha ⇒ flag \`UNIDENTIFIABLE_GEOMETRY\` / \`BOUND_NOT_OBSERVED\` without assigning a validated comfort lower bound;
- ST-F/S pass but ST-R fail ⇒ no common stationary alpha at that candidate;
- identical actual trajectories under different policy labels ⇒ identical predicted comfort score;
- publish all errors and pilot failures, do not edit/rewrite old result CSVs or logs.

## Working method / expected delivery

1. Audit the repository first and briefly describe which existing functions will be reused, which will be added, and the expected number of run configs/trials. State any actual blockers (e.g. walker speed not controllable) with evidence.
2. Implement code **only in the new Stage 2 scale experiment paths**, plus minimal shared helper fixes with tests if strictly needed. Do not change collision-monitor, safety rules, Nav2 controller/planner, common frozen Stage 1 parameters or older experimental results. For a cross-cutting fix that could change earlier results, stop and explain before committing.
3. Execute static validation, shape/unit/integration tests and generator \`--check\`. If Isaac Sim is accessible, run **short smoke tests** only (no unapproved 450-trial campaign). Never claim any result not actually observed. If simulation is inaccessible, clearly state which tests ran and provide exact commands to run on the experiment PC.
4. Show changed files, all commands/tests and outputs, a brief scientific limitations section, and a concise researcher-facing summary in Korean. Commit on the working branch or provide a reviewable diff; do not overwrite history. Document the pilot width decision log before progressing.

Do not report \`alpha^*\` or new Stage 3 performance as measured if the only completed work is code or smoke testing. This is a **research protocol implementation**, not permission to manufacture validation data.
