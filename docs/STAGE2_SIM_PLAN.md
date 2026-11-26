# Stage 2 (simulation, rev. 2026-11-26) — literature-referenced minimum Comfort Space

Replaces the Q10 proxy of `docs/FINAL_261125_PLAN.md` §5 as the Stage 2 used for model building. The proxy and its
results stay where they are (`results/batches/261125_final_stage2_proxy/`); Stage 1 results are untouched.

**Wording rule.** Values here are *literature-predicted comfort* and *literature-referenced minimum sizes*. They are
not experimentally validated comfort distances, not ISO values, and the simulation does not measure anybody's comfort.

## 1. Why the proxy had to go

The Q10 proxy was "how close the navigation stack happened to come in clean successes". For stationary / engaged
people it measured the aisle layout (e.g. 2.08 m for engaged | side), and it contains no human judgement at all. A
lower bound needs an external statement about people. In simulation that can only come from published human
studies, so Stage 2 now:

1. takes comfort-vs-distance curves fitted to human ratings in published experiments,
2. drives the AMR past one person in controlled scenarios with the Comfort Space shrinking from 0.90 m,
3. scores the passage the AMR actually made (closest centre distance and side) with those curves,
4. picks the smallest size that still meets a target predicted comfort and still navigates.

## 2. Existing structure (code audit)

| piece | file | note |
|---|---|---|
| Comfort Space shape | `comfort_core/comfort_space.py` | `static` circle; `state_directional`: per state front / side / rear extents, body heading for standing people, velocity heading + speed ramp for walkers. Unchanged. |
| mask, gate | `scripts/comfort_publisher.py`, `scripts/comfort_gate.py` | unchanged; gate holds only, safety layer separate |
| contexts | `comfort_core/contexts.py` | c1 engaged-rear, c3 stationary-front, c4 stationary-side; reused for facing |
| trial log, metrics | `scripts/trial_logger.py`, `comfort_core/final_metrics.py` | 5 Hz poses, states; classification reused unchanged |
| runner | `scripts/final_run.py` | reused unchanged (manifest, retries for infra only) |

What was missing: an external comfort reference, controlled single-person scenarios for the final pipeline, and a
rule that ties a Comfort Space size to a human-referenced criterion. No code of the navigation stack, gate, safety
layer, costmap or Nav2 changed.

## 3. Reference model (`comfort_core/comfort_reference.py`)

Inverted Gaussian, Neggers et al. 2022 (IJSR) Eq. 1: `C(d) = 1 + a0 (1 − exp(−d² / 2σ²))`, 7-point scale, d = centre-to-
centre at the closest point (cm), θ = polar angle of that point (right 0, front π/2, left π, back 3π/2).

| state | reference | what was tested | values |
|---|---|---|---|
| stationary | N22a study B (custom AGV 730×650 mm) | person standing on footprints, robot passes on a straight line at 0.35 m/s | a0 / σ[cm]: right 5.39/46.14, front 5.62/44.13, left 5.39/47.21, back 4.58/39.11 (Table 1) |
| station_engaged | same as stationary | — | **assumption**: no study varied task engagement; `engaged_extra_m` = 0 |
| moving | N22b Exp. 1 (Pepper) | walking participant, robot head-on at 0.35 m/s | a0 5.71, σ 44.2 (Table 2, η = 0); no side dependence |

Between the four sides: the 4-point trigonometric interpolant (the paper's discrete Fourier interpolation).
Alternative study A (humanoid) is in the code; its back curve never exceeds 4.92.

Checked and NOT used:
- N22b Exp. 2 (AGV, 0.35–1.7 m/s, passing and overtaking): per-speed a0/σ only in figures → no speed model, no
  overtaking curve (TODO: ask the authors for the numbers).
- Jafari et al. 2026 (ICRA): walking pedestrians, robot at 1.4 / 2.8 m/s; binary estimators only, weights set by
  visual inspection, PTTC formula not given. Only `D_min ≥ 1.0 m` is reported, as a secondary flag outside its speed
  range.
- Sisbot et al. 2007 (T-RO): a planner with designed safety/visibility costs, not fitted to ratings → not a comfort
  reference; cited as the origin of the "visibility" idea only.

Known inconsistency: N22b states comfort 6 ↔ 80 cm, but Table 2 with η = 0 gives 6 at ~90 cm (80 cm gives 5.60). The
wider η = 0 curve is used; the exact Eq. 2 form is to be confirmed.

Literature-required extent (person centre → AMR edge = d_cc(target) − AMR half width 0.24 m):

| target | stationary front | side | rear | moving |
|---|---|---|---|---|
| 4.0 | 0.31 | 0.36 | 0.33 | 0.30 |
| 4.5 | 0.38 | 0.44 | 0.42 | 0.37 |
| 5.0 | 0.46 | 0.54 | 0.56 | 0.45 |
| 5.5 | 0.55 | 0.66 | 0.87 | 0.54 |
| 6.0 | 0.69 | 0.84 | never | 0.66 |

This table is a *prediction* for a straight pass at 0.35 m/s. Stage 2 checks what the real navigation stack does.

## 4. Experiment (`configs/stage2sim/stage2sim.yaml`, `scripts/stage2sim_plan.py`)

One AMR, one person, straight corridor, no fleet. The AMR's start–goal line passes 0.30 m from the person (centre to
centre), so the Comfort Space and not the corridor sets the clearance. The person stands 0.50 m from the +y wall so
the AMR cannot pass behind them (except where that is the scenario).

| scenario | person | passage | versions |
|---|---|---|---|
| SE_rear | station_engaged, back to the aisle | behind | static, adaptive |
| ST_front | stationary, facing the aisle | in front | static, adaptive |
| ST_side | stationary, side to the aisle | beside | static, adaptive |
| MV_pass | walking head-on (ignores the AMR) | beside | static (no directional reference for walkers) |
| MV_overtake | walking the same way | from behind | **INFEASIBLE**: walker 1.04 m/s (animation speed, cannot be lowered) vs AMR limit 1.0 m/s; and no numeric overtaking curve |

- Ladder: 0.90 → 0.81 → 0.72 → 0.63 → 0.54 → 0.45 → 0.36 m (step = 10 % of 0.90, configurable: `ladder.mode`,
  `fraction`, `ratio`, `min_m`). 0.90 is the Stage 1 static R_nav_upper, used as the start, not as a ceiling for
  adaptive shapes.
- static = circle r. adaptive = the literature iso-comfort shape at the primary target, scaled so the tested
  direction = r (e.g. SE_rear at r 0.54: front 0.44, side 0.52, rear 0.54).
- B1 (no Comfort Space) per scenario.
- Ladder also ends at the floor 0.30 m (`comfort_space.min_radius`).
- Two corridors: **tight** (primary, 5 repeats): width = person_wall_gap 0.50 + r + `lane_free_m`, so the AMR
  must pass close to the Comfort Space edge; **wide** (secondary, 3 repeats): fixed 2.6 m (3.0 m for MV).
- B1 (no Comfort Space) per scenario and corridor. Full plan: 480 trials, ~75 s each.

Per trial (`<batch>/analysis/stage2sim_trials.csv`): state and side at the closest point, min centre and edge
distance, predicted comfort (at closest; minimum over the encounter; clearance-matched), whether the distance is
inside the reference's tested range, AMR and relative speed, min TTC (constant velocity, contact = person radius +
AMR half width), out-of-view time (AMR within 2.5 m and > 90° from the person's heading), Comfort Space intrusion
time and depth, navigation class, success, deadlock, contact, protective stop, hold, travel time, failure reason.
Set size and achieved distance are separate columns.

## 5. Decision rule (`scripts/stage2sim_report.py`)

For each scenario × version × target (4.0 / 4.5 / 5.0 / 5.5 / 6.0; primary 5.0 = ASSUMPTION, user to confirm):

- a size passes when ≤ 20 % of its repeats are below the target (ASSUMPTION) and ≥ 80 % navigate OK (ASSUMPTION),
- R_min = the smallest size such that it and every larger ladder size pass (monotone),
- 0.90 already below target → `INFEASIBLE_AT_START`: shrinking is not allowed; needs slow-down / wait / yield /
  detour behaviour,
- comfortable but not navigable → `NEEDS_POLICY`.

The derived Adaptive config `B3L_c<target>.yaml` takes the tested cells from the simulation and the untested cells
(e.g. engaged front/side, stationary rear, moving front/rear) from the literature table, each line labelled with its
source. That is the Stage 3 input.

## 6. Status

- Code: reference model + 7 unit checks (`scripts/check_comfort_reference.py`, pass), planner, report.
- Smoke, wide corridor: `results/batches/261126_stage2sim_smoke_wide` (15 trials). The planner centres the AMR in
  the free space, so the clearance follows the corridor (r 0.54 → edge 0.95 m, r 0.90 → 1.36 m), not r → tight
  variant added. First MV trials used human c, who has no character model and never walked; kept in
  `invalid_walker_no_character/`, re-run with human a.
- Smoke, tight corridor (`lane_free_m` 1.0): `results/batches/261126_stage2sim_smoke` (15 trials). Achieved edge
  distance ≈ r + 0.36 m (Nav2 passes in the middle of the free lane). B1: edge 0.25–0.27 m, predicted comfort
  3.5–4.5 (MV_pass: min TTC 0.19 s). Several PROTECTIVE_STOP / deadlock outcomes (SE_rear and ST_front at 0.90,
  ST_side at 0.90 and 0.54, ST_front adaptive 0.54): with a 1.0 m lane only ~0.26 m is left between AMR and wall,
  about the collision monitor's footprint padding (0.25 m). The safety layer stays unchanged; the proposal is to
  widen `lane_free_m` to 1.2 m and re-smoke (not done yet).
- Full run: not started (waits for the primary target, the two rates and `lane_free_m` to be confirmed).

## 7. Text for the paper (draft)

*Stage 2 determines, for each human state and passing side, the smallest Comfort Space that keeps the passage the
robot actually makes above a target level of comfort predicted from published human-subject data. We use the
inverted-Gaussian comfort–distance relations of Neggers et al. (2022a, 2022b), fitted to 7-point comfort ratings of
participants passed by a robot, with side-specific parameters for standing participants and a single curve for
walking participants. In controlled single-person scenarios the Comfort Space is reduced stepwise from the Stage 1
navigation search cap (0.90 m); every passage is scored with the same external reference, irrespective of the policy
that produced it. A size is accepted when the predicted comfort meets the target in a prescribed share of repeated
runs and navigation remains successful. Sizes are not reduced below the comfort requirement for the sake of
navigation; where both cannot be met, the case is classified as requiring a behavioural policy (slowing, waiting,
yielding or detouring).*

*Limitations: the reference curves come from robots at 0.35 m/s and from standing or walking participants without a
task; no reference exists for task-engaged people, for overtaking, or for speed dependence at the AMR's operating
speed. Predicted comfort is a model output, not a measurement, and the resulting sizes are simulation-derived
requirements to be validated with people.*
