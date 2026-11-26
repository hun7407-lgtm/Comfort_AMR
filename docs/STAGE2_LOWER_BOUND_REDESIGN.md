# Stage 2 Lower Bound — proposed experimental redesign (2026-11-26)

**STATUS: PROPOSAL ONLY / NOT IMPLEMENTED OR RUN.** This document responds to the latest evidence review and supersedes the *design rationale* in `docs/STAGE2_SIM_PLAN.md` only after protocol approval and code changes. Its values are not experimental outcomes. Preserve all Stage 1 results and prior Stage 2/3 batches.

Related manuscript paragraphs: [PAPER_RELATED_WORK_MODELING_DRAFT.md](PAPER_RELATED_WORK_MODELING_DRAFT.md).

## A. What Stage 2 can and cannot identify

**Primary purpose:** find the smallest **configured** policy extent allowing a controlled AMR to complete an encounter with a human while meeting a prespecified **literature-predicted comfort** criterion, for the supported human-state × approach-direction contexts.

Distinguish:

- `D_req(H, phi, C*, v_ref)`: human–robot **centre-to-centre** distance at which a published comfort–distance model reaches target `C*`, for a defined empirical context. This is a model-inverted *literature reference*, not a simulator observation.
- `D_edge_req`: person centre to nearest AMR footprint edge; convert using the correct AMR geometry for the actual passage orientation. The existing approximate lateral conversion subtracts 0.24 m; do not apply this half-width blindly to front/end-on robot orientation.
- `R_policy_min(H, phi)`: minimum **configured** directional extent such that repeated, completed, contact-free AMR passes in a **fixed geometry** meet the predicted comfort and navigation acceptance criteria. It need not equal `D_edge_req`.

An inverse Gaussian curve has no intrinsic discontinuous "comfortable/not comfortable" point. The study's operational target `C*=5.0/7` is explicitly a **choice**; run sensitivity at 4.5 and 5.5. Only human-subject evaluation could support a claim about actual comfort acceptance. The existing `R_nav_upper=0.90 m` is a Stage 1, multi-human/multi-AMR *static-circle* operational reference under a specific factory geometry, **not** a universal cap on every directional extent.

## B. What is empirically supported

| Cell group | Evidence source | Stage 2 interpretation |
|---|---|---|
| Stationary × front/side/rear | Neggers 2022 stationary human, AGV study B, 0.35 m/s | Can score literature-predicted comfort with side-specific curve |
| Moving × head-on passing | Neggers 2022 walking human, Pepper, 0.35 m/s | Can score head-on pass; no distinct directional front/rear curves |
| Moving × forward extension vs speed | Gorrini 2014, 2019: **human–human**, frontal locomotion, shared original data | Supports hypothesis/sensitivity; cannot set numeric robot comfort threshold |
| Moving × overtaking | Neggers 2022 experiment 2: reported effects; exact speed-specific fitted numeric parameters not available in current code | Mark unsupported for quantitative comfort scoring until reference parameters or new human data |
| Station engaged × any direction | Kinoe 2015/2018; Cui 2023 support task/social context dependence in **human–human** interaction | Transfer stationary reference as primary conservative hypothesis; no human-validated reduction due to machine work |

Gorrini (2019) revisits the 2013 Tokyo experiment also reported in Gorrini (2014); do not double-count as independent evidence. Cui (2023) found *no significant reduction in the solo-task control*, so `station_engaged` is **not automatically smaller**.

## C. Proposed two-part protocol

### Part 1 — literature requirement (no navigation experiments)

1. Lock the reference and original measurement basis before seeing new simulator outcomes: `N22a_B` for stationary, `N22b_E1` for moving head-on passing, stationary transfer to engaged explicitly labeled. The curves were fitted at a robot speed of 0.35 m/s; report the transfer limitation for the operational AMR.
2. Set primary `C*=5.0` (assumed 7-point criterion), sensitivity 4.5/5.5. Do **not** call this a population-validated psychological threshold.
3. For supported context and azimuth `phi`, invert `C=1+a0(phi)*(1-exp(-d_cc^2/(2*sigma(phi)^2)))`:
   `D_req = sigma(phi)*sqrt(-2*ln(1-(C*-1)/a0(phi)))`.
   Return `UNREACHABLE` when `C*-1 >= a0(phi)`; return `UNSUPPORTED` for an unmeasured context, rather than filling it with a fabricated observation.
4. Express required centre-to-centre distance and, separately, the exact footprint-edge separation. Report empirical fit-range and AMR speed mismatch flags.
5. Illustrative current model at 5.0/7 (metres, person centre to AMR lateral edge): stationary front 0.456; stationary side 0.537; stationary rear 0.555; moving head-on passing side 0.446. For engaged these are **stationary-transfer assumptions**. Recompute after any model changes.

### Part 2 — operational calibration in simulation

**Scenario factors** (initial identification; one AMR/one person):

| Scenario | State label | Human facing / relative AMR passage | Comfort reference status |
|---|---|---|---|
| ST-F | stationary | front | direct stationary reference |
| ST-S | stationary | side (left/right counterbalance) | direct stationary reference |
| ST-R | stationary | rear | direct stationary reference |
| EN-F/S/R | station_engaged | same pose/facing as matched stationary trials; only state/task label differs | stationary-transfer; *hypothesis comparison only* |
| MV-PASS | moving | head-on approach, lateral passage (walking speed as actually realized) | walking-person passing reference, within speed-domain caution |
| MV-FRONT / MV-OVERTAKE | moving | forward and from behind | exploratory geometry/feasibility only; cannot claim validated subjective comfort |

**Freeze environment per scenario.** Keep corridor width, person-wall gap, person pose, wall geometry, AMR start/goal, AMR speed limit, navigation parameters, costmap filter buffer, physical collision monitor, human route/speed script, and timing identical across all tested radii. Change **only** the specified comfort-space policy extents. Use a `wide` geometry and a fixed `tight` geometry, chosen in a separate pilot to avoid protective-stop failures caused only by wall proximity. Existing `tight_width = 0.50 + r + lane_free_m` must not be used for causal comparisons: it changes the corridor width with the independent variable. Illustrative widths (not validated): static-human tight 2.4 m, wide 2.6 m; moving scenarios may need other *fixed* widths after pilot.

**Policy alternatives.** B1 (no Comfort), B2 (static circle), and B3 directional ellipse. For calibration, vary one targeted extent at a time while fixing the other two at the literature baseline (or in a defined, disclosed shape family), to identify direction-specific contributions. A scaled whole ellipse changes all three extents simultaneously and cannot independently identify the selected sector. Control the actual closest-point sector and discard/move encounters that did not pass on the intended side from that cell's calibration analysis.

**Search.** Pilot B1 and a few representative sizes for physical feasibility. Coarse values 0.30 to 0.90 m in 0.10 m steps (include sizes near the model-derived requirement); once the feasible interval is located, add 0.05 m steps around the boundary. The 0.90 m Stage 1 static reference is a *comparison*, not a hard cap for all adapted extents. If an empirically required extent exceeds 0.90 m, keep the unmet requirement visible and classify feasibility rather than clipping it to 0.90 m.

**Repetition.** Five pilot repeats are for debugging only, not statistical identification. Use ~10 matched replicates per coarse setting, then **30 or more predeclared independent launches** for final candidate, the immediately smaller candidate, and the relevant B1/B2 references. Hold human trajectories and map fixed by replicate/seed; reorder/interleave policy execution to reduce time/run-order bias. Identical seeds do not guarantee deterministic ROS/Isaac Sim outcomes: preserve launch-level replicate IDs, log startup/timing variation and use uncertainty intervals.

**Speed domain.** The published Neggers curves used robot speed 0.35 m/s. Record actual robot and closing speeds per encounter. If physically and procedurally feasible, add a reference-speed condition at 0.35 m/s in a separate Stage 2 calibration block (not a change to Stage 1/3 frozen Nav2 comparisons). At actual operational speeds (~0.7–0.8 m/s in previous logs), report scores only as out-of-domain **predictions/sensitivity**, not verified comfort. Obtain or digitize published per-speed fitted parameters with provenance, or use new human ratings, before making speed-specific comfort claims. The existing walker animation is around 1.04 m/s, so do not advertise a pedestrian-speed sweep until the motion system can realize controlled speeds.

## D. Response variables and correct decision rule

Log every encounter's declared/observed human state, facing and velocity heading, actual pass sector, centre-to-centre minimum distance, exact footprint-edge minimum clearance, predicted comfort at closest point, trajectory minimum predicted comfort (secondary; specify window), actual AMR speed/closing speed, crossing completion, contacts, protective-stop, hold duration, deadlock class, travel time, and actual geometry.

**Critical:** an AMR that waits or deadlocks **without completing the passage** must not be counted as a "comfortable passage" just because its distance remained high. Compute comfort among completed passes and also define a per-replicate **joint** success indicator:

`J_j(r) = 1{AMR completed intended passage AND no human contact AND predicted comfort >= C* AND no prohibited protective-stop/deadlock condition}`.

Primary feasibility, proposed and to be preregistered before the full run:

- `Pr(J=1 | r) >= 0.80` **observed** (engineering acceptance choice), and separately `Pr(nav success | r) >= 0.80`.
- Report numerators/denominators and Wilson 95% confidence intervals. With 30 replicates, a point estimate of 80% is uncertain; do not interpret it as proof the population probability exceeds 80%. Sensitivity: 0.90 pass-rate threshold.
- `R_policy_min = min{ r in tested candidate set : J-rate(r)>=0.80 and nav-rate(r)>=0.80 }`. Do not interpolate to untested radii without a checked, defensible model.
- **Do not impose the existing "every larger radius must also pass" rule.** Larger radii can cause Comfort-induced deadlock, while smaller radii can navigate successfully and still meet the target. Joint feasibility may be nonmonotonic. Inspect and report all intervals/isolated feasible points; confirm the selected radius with dedicated replicates.
- If none meets both, mark `NO_FEASIBLE_SIZE` and propose a **behavioral** change (slow/wait/detour/coordinate), not an unsupported radius shrink.

Include an independent **geometric** metric at each context: `max(0, D_req - measured_achieved_distance)` and its positive-trial frequency. Do not score intrusion only relative to each method's own radius (shrinking would mechanically improve the number).

## E. Main research hypotheses (do not pre-announce findings)

- **H1 (supported):** stationary human–robot comfort requirement is direction dependent; compare front/side/rear literature-derived threshold and achieved clearance.
- **H2 (partial transfer):** walking encounter warrants a motion-aware space; compare forward-elongated vs symmetric policies as a **navigation and geometry ablation**, without assigning a measured comfort rating to unsupported moving front/rear sectors.
- **H3 (exploratory):** task state may modulate spatial demand. Compare `engaged = stationary` (primary) with `engaged rear = stationary rear + delta` for disclosed `delta in {-0.10, 0.00, +0.10} m`, **engineering sensitivity only**; neither sign is a human validated effect. Avoid automatically allowing smaller rear separation at a machine.
- **H4 (operational):** the smallest feasible configured extent differs from the literature's required physical clearance because Nav2, geometry, and other agents influence realized trajectories.

## F. Audit of current code — changes required *before* the next Stage 2 full run

1. `configs/stage2sim/stage2sim.yaml` and `scripts/stage2sim_plan.py`: remove per-radius corridor width changes; freeze fixed geometry for all size conditions.
2. `scripts/stage2sim_report.py:select()`: remove the universal monotone-all-larger criterion; choose the minimum independently feasible radius (the current SE_rear smoke: 0.90 navigation failure, 0.54 success with predicted comfort >5, yet selector currently returns NEEDS_POLICY).
3. `scripts/stage2sim_report.py`: compute joint success only for **completed** target passes; maintain separate nav-rate, conditional completed-pass comfort, and failure classes. Never allow a non-traversal to count as comfortable.
4. `scripts/stage2sim_plan.py` and derived config writer: ensure independent directional extent variation; do not cap all F/S/R at Stage 1 static `R_nav_upper`; attach `direct evidence / transfer / engineering assumption / unsupported` provenance to every one of nine cells.
5. Enforce evaluated encounter sector and speed-domain checks. Calculate actual polygon clearance for the physical edge metric; keep centre distance for literature model input.
6. Run new pilot, lock assumptions and conditions, then execute full run. Do not relabel 2026-11-26 smoke samples as statistically confirmed final results.
7. Keep physical safety layer, frozen Stage 1 setup, old batches, and existing Stage 3 results untouched. Generate new Stage 2 outputs under a new versioned batch; regenerate Stage 3 on independent holdout seeds only after the new Stage 2 config is frozen.

## G. Stage 3 interface and claim boundary

Export a provenance table `state | relative direction | literature model | D_req | confidence/transfer flag | calibrated R_policy_min | pilot/final n | limits` and versioned B3-O configuration. **Do not force all nine cells to have a "measured lower bound".** Unsupported cells remain assumptions/ablations. Stage 3 tests operational outcomes against B1/B2 and B3-O on independent seeds: system success, Comfort-induced deadlocks, wait/hold, travel time, protective stops, physical clearance and literature-predicted comfort exposure. Report run-to-run variability; do not infer subjective comfort from simulation.

## H. Preflight decision log (still open)

- [ ] Confirm the primary operational comfort cutoff `C*=5.0/7` and alternatives 4.5/5.5.
- [ ] Confirm the target joint pass-rate and whether CI lower bound is used for acceptance.
- [ ] Pilot/freeze tight and wide corridor geometries independent of radius.
- [ ] Establish actual AMR speed domain and whether a 0.35 m/s reference-speed block is feasible.
- [ ] Lock how `station_engaged` transfer assumptions and unsupported moving directions will be reported.
- [ ] Patch selectors and passage-completion scoring; rerun smoke tests.
- [ ] Freeze trial matrix, repeats and seed discipline before looking at the full results.

## References

Gorrini et al. 2014 https://doi.org/10.3141/2421-07; Gorrini et al. 2019 https://doi.org/10.3233/WEB-190403; Neggers et al. 2022a https://doi.org/10.1007/s12369-021-00805-6; Neggers et al. 2022b https://doi.org/10.3389/frobt.2022.915972; Kinoe & Mizuno 2015 https://doi.org/10.1007/978-3-319-20618-9_2; Kinoe 2018 https://doi.org/10.1007/978-3-319-96059-3_37; Cui et al. 2023 https://doi.org/10.3389/fpsyg.2023.1148395.
