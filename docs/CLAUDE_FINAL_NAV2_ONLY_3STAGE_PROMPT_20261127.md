# Claude execution prompt — FINAL Nav2-only Stage1–2–3 (30-rep study)

You are the final implementation and experiment execution engineer for the master's thesis repo **`hun7407-lgtm/Comfort_AMR`, branch `feature/b1-baseline-and-yield`**.

**READ THESE FIRST and treat the first file as the NEW STUDY protocol:**
1. `docs/FINAL_NAV2_ONLY_3STAGE_PROTOCOL_20261127.md` — **authoritative locked experimental design**; do not alter hypotheses, policy set, scoring thresholds or scenario selection to favor an outcome.
2. `docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md` and `docs/NAV2ONLY_STAGE2_RESULTS.md` — old implementation/audit and preliminary research only; old numbers are NOT final.
3. `docs/ARCHITECTURE_4FIG_NAV2_ONLY_SPEC.md` — thesis framework claim.
4. `AGENTS.md`, `configs/common.yaml`, `comfort_core/control_mode.py`, `scripts/launch_nodes.py`, `scripts/nav2_bringup.launch.py`, `scripts/nav2_goal.py`, `scripts/final_run.py`, `scripts/final_plan.py`, `scripts/make_cross_campaign.py`, `comfort_core/stage2scale.py`, `comfort_core/comfort_reference.py`, `comfort_core/final_metrics.py`, `scripts/stage2scale_plan.py`, `scripts/stage2scale_report.py`, `configs/nav2only/control_overlay.yaml`, `configs/stage2scale/stage2scale_v2_nav2only.yaml`, `results/batches/261125_final_stage3/analysis/STAGE3_REPORT.md`.

## Research objective

Implement and RUN an auditable, **final** integrated three-stage experiment testing whether an intermediate state-template-wide Comfort Space SCALE can balance predicted Human Comfort against multi-AMR Nav2 feasibility. The prior illustrative thesis table showing alpha0.80 as winner is **invented placeholder content**, not an observed target, reference, or test oracle. The real winner may be **any** condition or none. Never alter physics, paths, conditions, thresholds, metrics, sample selection, or plots to engineer alpha0.70/0.80 success; never fake or impute trials.

## Required phases

**Stage 1 — final radius revalidation:** 8 workers/3 Nav2 AMRs, B1 and circular B2 r=0.85/0.90/0.95/1.00, **30 runs/condition (150)**. Observe all three AMRs, contact, physical collision stop, Nav2 deadlock and mission success; report the operational bound only if the **predeclared** rule is satisfied. New common normalization reference B2=0.95 is fixed as an **engineering anchor** even if final bound is inconclusive; do not silently shift it.

**Stage 2 — final single-AMR scale characterization:** one person/one AMR in ST-F/ST-S/ST-R/EN-R/MV-P, each 9 conditions = B1 + B2 0.95 + alpha {1.00,.90,.80,.70,.60,.50,.45}, **30 runs/cell (1350)**. Carry out an independent geometry pilot/freeze first; keep actual stations for engaged, both-side sensitivity for stationary if achievable, and fixed parallel counterwalking lanes for moving. B1 touching a walker is a B1 outcome, NOT alone reason to drop all alpha conditions. Uniform alpha multiplied across literature-referenced F/S/R; do not separately tune axes. Read predicted comfort from logged real closest distance/bearing, C*=5.0 (4.5/5.5 sensitivity), and keep `FLOOR_LIMITED` or `UNIDENTIFIABLE` where necessary.

**Stage 3 — final five-way multi-AMR comparison:** 8 people/3 AMRs, **5 conditions × 30 fresh matched seeds (150)**:
- C1 B2 fixed circle 0.95 m
- C2 B3 offset state-reference template × alpha0.60
- C3 B3 template × alpha0.70
- C4 B3 template × alpha0.80
- C5 B3 template × alpha0.90

Use 30 **new** holdout factory scenarios seeded 1001–1030, 10 nominal + 10 station bottleneck + 10 crossing stress, predefined/frozen and identical across five policies. Randomize/balance order of all five conditions within each matched seed. Pilot geometries for technical validity/policy sensitivity BEFORE final comparisons without choosing layouts that make a favorite alpha win. For nondeterminism, run a separate prespecified duplicate-control diagnostic with unchanged B2 and B3 .80 configs for 10 seed cases; exclude these duplicates from primary 150.

**ALL stages:** `control.mode:nav2_only`, costmap buffer .10m, no comfort gate/yield/evade/custom backoff/off_lane/traffic_rule; Nav2 native planner/DWB recovery and collision_monitor always ON for AMR1/a1/a2, no loss of physical collision protection. Ground-truth state+tf for controlled experiments, label LiDAR state recognition as separate/unvalidated. Runtime ROS graph must show Nav2 -> cmd_vel_safe -> Collision Monitor -> cmd_vel and no gate writer. Frozen 0.95 anchor and F/S/R templates, no `min_radius=.30` clipping.

## Experimental validity and code fixes

- Revise new-mode `comfort_core/final_metrics.py` so stopping from Nav2/Collision Monitor is **not labeled Gate Yield**; old data remain compatible. `gate_yield_s=0` under nav2_only; separately log idle/no-progress and physical protective stop.
- Check and test `comfort_reference.py` clearance_matched signed half-width conversion; only a secondary metric, primary predicted comfort unchanged.
- Make Stage 2 `station_engaged` include physical/semantic workstation consistently for all conditions, not simply label a person engaged while every station is disabled.
- Main Stage 3 must record **ALL three AMRs'** paths, successes, contacts and human interactions, not just AMR1. Scripted non-yielding pedestrian contact is a failure of safe/joint completion even if a Nav2 action returned SUCCESS. Do NOT ignore a moving person's collision or retune collision monitor.
- Make `trial_joint_success` primary: ALL three AMRs finish, NO human physical contact, and ALL relevant scorable interactions achieve predicted comfort >=5.0. Handle undefined/out-of-domain predicted comfort with coverage and sensitivity, not automatic PASS. Separately report navigation-only completion, safe completion, comfort conditional on completed encounters, min and distributions of actual distances and predicted comfort, travel time with failures included/capped, path length, replans, deadlock/no progress, Collision Monitor interventions and contacts.
- Use a predeclared winner rule (safe empirical contact <=1/30, mission>=.80, joint>=.80, adequate comfort coverage; highest joint among feasible; ties/uncertainty -> Pareto feasible range, capped completion time tie-break). Provide Wilson CI, matched McNemar/bootstrap and sensitivity. **Never claim an interior optimum merely because plots look nice**.
- Five global-alpha cases alone do not isolate **separate per-state alpha selection**. If time/resources and thesis claim require it, implement a **predeclared extra state-wise ablation** as optional sixth *separate* condition after main five are frozen; do not mix it with core five.

## Implementation/execution workflow

1. Inspect code and create **new versioned V3 config generators, reports, tests and manifests**, not edits to old historical results. Prefer adapting existing scripts over a duplicate simulation stack.
2. Audit physical signature, masks for 3 robots, graph, metrics, contact logger, comfort scoring, seed independence and run resumability.
3. Run offline tests and 1–2 actual Nav2/safety/contacts smoke tests. Diagnose policy-neutral physical geometry once, freeze and commit config/manifest/hash/rules BEFORE main n30.
4. Run Stage1 full 150; Stage2 pilot and 1350; Stage3 pilot and 150. No approval pause required when all prerequisites pass; do NOT reduce repeats without a user instruction. A mandatory technical safety/methodology failure is a reason to STOP clearly, not to silently change scientific protocol.
5. Generate and commit **actual** analysis tables/plots + documentation: Stage1 radius vs success/contact/stop, Stage2 alpha vs actual clearance vs predicted comfort by state, Stage3 five-policy success/joint/median travel/path, contact/stop + confidence intervals and Pareto. Keep per-trial and per-encounter CSV, config and graph hashes, seed/attempt id, code SHA, raw n and exclusion reasons. Historical batches `261125_*`, `261126_*` remain intact.
6. Report results in Korean with completed/failed/skipped counts, new branch commits, exact reproduction commands and each key caveat. Create final thesis text for Experiment Design, Results and Conclusion (each **two paragraphs × three sentences**) ONLY after reading actual data; do not retrofit fictional placeholder values as facts.

**If local Isaac Sim or ROS 2 are unavailable:** do all genuine offline checks, write runnable commands for the workstation, and explicitly mark unexecuted work. Do not report synthetic/assumed data as 30-run results.

Please implement this as the *final empirical protocol*, not another 5-run shortcut or a prompt-only document.
