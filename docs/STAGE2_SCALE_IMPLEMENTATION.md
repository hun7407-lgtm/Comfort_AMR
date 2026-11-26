# Stage 2 uniform-scale sweep — implementation notes (v1, 2026-11-26)

Implements [STAGE2_UNIFORM_SCALE_PROTOCOL.md](STAGE2_UNIFORM_SCALE_PROTOCOL.md) (authority) and
[CLAUDE_STAGE2_UNIFORM_SCALE_PROMPT.md](CLAUDE_STAGE2_UNIFORM_SCALE_PROMPT.md). The older
`configs/stage2sim/` prototype and its batches (`results/batches/261126_stage2sim*`) are not read or written.
No change to Stage 1 / Stage 3 results, the physical safety layer, Nav2, the gate or `comfort_space.py`.

## 1. Code audit — reused vs new

| reused unchanged | why |
|---|---|
| `comfort_core/comfort_space.py` `StateDirectionalComfortPolicy` | offset ellipse a=(F+R)/2, b=S, offset=(F−R)/2; body heading for standing / engaged, velocity for walkers. The per-alpha extents are written directly into the condition (`aisle_scaling.enabled: false`), so its `_squeeze` (floor + s·(x − floor)) is never used. |
| `comfort_core/shapes.py` `ComfortShape.bounding_radius` | normalisation check (plus a dense numeric check of the offset ellipse) |
| `comfort_core/comfort_reference.py` `score_passage` | the only comfort score: actual closest centre distance + bearing |
| `comfort_core/final_metrics.py` `analyse_trial`, `headings`, `rect_clearance` | nav class (SUCCESS / NORMAL_YIELD / TEMPORARY_HOLD / PROTECTIVE_STOP / comfort classes ...), oriented-footprint clearance |
| `comfort_core/contexts.py` `facing_deg` | c1 engaged-rear, c2 stationary-rear, c3 front, c4 side |
| `scripts/final_run.py`, `run_scenario.sh`, `trial_logger.py` | runner (infra-only retries), 5 Hz log incl. per-tick F/S/R |

| new | content |
|---|---|
| `comfort_core/stage2scale.py` | templates, normalisation, E(alpha) = alpha·E(1), condition block (moving template frozen: gain 0), fixed geometry (static + two-lane moving), physical signature, J per launch, rates + Wilson CI, non-monotone selection, identification status, common state alpha |
| `configs/stage2scale/stage2scale.yaml` | all settings; ASSUMPTION / TODO marked |
| `scripts/stage2scale_plan.py` | `--phase smoke|pilot|coarse`, `--check`; refuses on shape / geometry / lane errors; coarse refuses until widths and lane gap are frozen |
| `scripts/stage2scale_report.py` | `scale_trials.csv`, `scale_summary.json`, `selected_state_scales.json`, `B3O_SCALE_CANDIDATE.yaml`, figures, `STAGE2_SCALE_REPORT.md` |
| `scripts/check_stage2scale.py` | 18 checks (protocol §9 items 1–7 offline), all pass |

The old `scripts/stage2sim_plan.py:tight_width` (corridor width a function of r) and the old same-centreline
moving layout are **not** reused.

## 2. Shapes

| state | raw F/S/R | alpha = 1 F/S/R | bounding radius |
|---|---|---|---|
| station_engaged | 0.80 / 0.70 / 0.60 | 0.900 / 0.7875 / 0.675 | 0.900 |
| stationary | 0.80 / 0.90 / 1.05 | 0.6857 / 0.7714 / 0.900 | 0.900 |
| moving (1.0 m/s, frozen) | 1.20 / 0.90 / 0.90 | 0.900 / 0.675 / 0.675 | 0.900 |

alpha ∈ {1.00, 0.90, 0.80, 0.70, 0.60, 0.50, 0.45}; smallest axis at 0.45 = 0.30375 ≥ min_radius 0.30 (no clip;
min alpha without clip 0.4444). One condition file per alpha covers all three states.

## 3. Geometry (fixed per scenario × geometry id; identical for B1, B2, every alpha)

- Static (ST-F, ST-S, ST-R, EN-R): person 0.50 m from the +y wall, AMR start–goal line 0.30 m below the person,
  corridor 12 m. Pilot width candidates 2.0 / 2.2 / 2.4 / 2.6 m; smoke 2.6 m.
- Moving (MV-P): walker on `human_lane_y` from x = +7 to −7, AMR on `amr_lane_y` from −5.5 to +5.5, lanes
  symmetric about y = 0, gap candidates 0.65 / 0.80 / 1.00 m, corridor 16 m, width candidates 2.6 / 2.8 / 3.0 m
  (smoke 3.0 m). `human_side: +1` puts the walker on +y.
- MV-O (overtaking): INFEASIBLE (walker 1.04 m/s ≥ AMR 1.0 m/s), recorded in plan.json, not run.
- The generator loads every condition on every env and compares a hash of the merged config without
  `comfort_space` / `scenario` (walls, people, start/goal, timing, Nav2, collision monitor): must be identical.

## 4. Per launch and selection

J = 1 iff the AMR completed (class SUCCESS / NORMAL_YIELD / TEMPORARY_HOLD and result SUCCESS), no contact,
passed on the intended side (static: aisle side; moving: its own lane side, and actually crossed the person),
closest point in the intended sector, valid state (declared; walkers also observed moving), and predicted
comfort at closest ≥ C*. Missing evidence never counts in favour. Rates: nav completion, comfort given a valid
encounter, joint over all non-infra launches, Wilson 95 % CI.

Per state: `COMMON_ALPHA_IDENTIFIED` only if every context is an identified bracket; otherwise
`COMMON_ALPHA_OPERATIONAL_ONLY` (not a comfort bound); any context UNIDENTIFIABLE / INSUFFICIENT makes the state
the same with no alpha*. Smoke / pilot candidates are marked DIAGNOSTIC.

Per context: smallest tested alpha with joint ≥ 0.80 and nav ≥ 0.80 (no "larger must pass"). Status:
`IDENTIFIED_BRACKET` (next smaller alpha fails on comfort), `OPERATIONAL_MIN_NOT_COMFORT_BOUND` (next smaller
fails on navigation only), `FLOOR_LIMITED_BOUND_NOT_OBSERVED` (smallest alpha passes), `UNIDENTIFIABLE_GEOMETRY`
(median completed d_cc at alpha 1.00 minus at the smallest alpha < 0.10 m), `NO_FEASIBLE_ALPHA`,
`INSUFFICIENT_DATA`. Per state: smallest alpha passing every primary context, else `NO_COMMON_FEASIBLE_ALPHA`.
Sensitivity: targets 4.5 / 5.5, joint ≥ 0.90.

Not implemented yet: time below target (secondary), left/right balance for ST-S (only the person's right side).

## 5. Trial counts

| phase | content | trials |
|---|---|---|
| smoke | static 4 × 1 width × {B1, B2, 1.00, 0.45} × 1 + MV-P 3 gaps × 4 × 2 | 40 |
| pilot | static 4 × 4 widths × {B1, B2, 1.00, 0.70, 0.45} × 2 + MV-P 3 widths × 3 gaps × 5 × 2 | 250 |
| coarse | 5 contexts × 9 conditions × 10 (frozen tight geometry) | 450 |
| confirmatory | ≥ 30 new launches for selected + next smaller alpha per context, matched B1 / B2, wide geometry | TBD |

## 6. Pilot / geometry decision log

| date | decision | evidence |
|---|---|---|
| 2026-11-26 | smoke geometry: static 2.6 m, moving 3.0 m with gaps 0.65 / 0.80 / 1.00 | protocol candidates; smoke only, not a freeze |
| 2026-11-26 | smoke run, 40 / 40 launches, 0 infra failures | `results/batches/261126_stage2scale_v1_smoke/analysis/` |
| — | tight widths, lane gap: **not frozen** | pilot not run |

## 7. Smoke result (2026-11-26, 1 launch static, 2 launches moving per cell — diagnostic only)

Actual closest centre distance d_cc (m) / literature-predicted comfort at closest (7-point), target 5.0:

| context (geometry) | B1 | B2 0.90 | alpha 1.00 | alpha 0.45 |
|---|---|---|---|---|
| ST-F (2.6 m) | 0.50 / 3.62, contact 0.58 s | 1.58 / 6.61 | 1.37 / 6.58 | 0.94 / 6.05 |
| ST-S (2.6 m) | 0.54 / 3.64 | 1.58 / 6.48 | 1.45 / 6.36 | 1.00 / 5.84 |
| ST-R (2.6 m) | 0.52 / 3.70 | 1.58 / 5.58 | 1.58 / 5.58 | 1.05 / 5.46 |
| EN-R (2.6 m) | 0.53 / 3.71 | 1.58 / 5.59 | 1.36 / 5.57 | 0.94 / 5.32 |
| MV-P gap 0.65 (3.0 m) | 0.85, 0.82 / 5.8, 5.7 | 1.02, 1.04 / 6.3, 6.4 | 0.80, 0.81 / 5.6, 5.6 | 0.67, 0.80 / **4.9**, 5.6 |
| MV-P gap 0.80 | 0.91, 0.91 / 6.0, 6.0 | 0.89, 0.91 / 6.0, 6.0 | 0.75, 0.76 / 5.4, 5.4 | 0.61, 0.78 / **4.5**, 5.5 |
| MV-P gap 1.00 | 1.05, 1.13 / 6.4, 6.5 | 1.20, 1.19 / 6.6, 6.6 | 1.15, 1.17 / 6.5, 6.5 | 1.12, 1.12 / 6.5, 6.5 |

- Every launch reached the goal; no comfort-induced deadlock, no terminal protective stop. The fixed 2.6 m
  corridor removed the wall-side PROTECTIVE_STOP deadlocks of the earlier radius-dependent tight corridor.
- **Static contexts: distance responds to alpha** (d_cc 1.00 → 0.45: −0.42 to −0.53 m), but predicted comfort
  stays ≥ 5.0 down to alpha 0.45 → `FLOOR_LIMITED_BOUND_NOT_OBSERVED` for all four: no comfort lower bound found at
  C* = 5.0. At C* = 5.5, ST-R and EN-R fall below at alpha 0.45 (5.46, 5.32): a bracket appears there (1 launch).
- **MV-P: `UNIDENTIFIABLE_GEOMETRY` at all three lane gaps** (median d_cc change 0.05–0.07 m < 0.10 m). With a
  Comfort Space the AMR mostly stops (comfort gate WAITING / STEPPING ASIDE; AMR speed at closest ≈ 0) and lets
  the walker pass, so the distance is set by where it stopped and by the lane gap, not by alpha. At gap 1.00 the
  lanes are far enough apart that the AMR does not stop at all and every condition gives ~1.1–1.2 m.
- B1 in MV-P passes farther (0.82–0.91 m at gaps 0.65 / 0.80) than alpha 0.45: without a Comfort Space the gate
  does not stop the AMR and Nav2 steers it away from the walker; with one it stops on its lane and the walker
  (who does not avoid the robot and sways ±0.1 m) passes close.
- `MV-P_w3.00_g0.80_S3_a0.45_rep01`: contact 0.23 s (footprint edge 0.21 m < person radius 0.25 m). The gate's
  STEPPING ASIDE turned the AMR (yaw −0.87 rad) while the walker passed, swinging its corner toward the lane
  (`analysis/figures/topview_MV-P_g0.80_a0.45_rep01_close_pass.png`). Kept as a joint failure.
- Top view (`analysis/figures/topview_MV-P_g1.00_a1.00_lanes.png`): walker in the UPPER lane walking right to
  left, AMR in the LOWER lane driving left to right; separate lanes, oncoming lateral pass, no same-line
  approach. The corridor runs left–right in this camera, so "left / right lane" appears as upper / lower; seen in
  each one's direction of travel both keep right. `human_side: -1` swaps them.
- AMR speed at closest 0.76–0.82 m/s when it does not stop: above the references' 0.35 m/s → every such row is
  flagged `amr_faster_than_reference`.

Consequences for the pilot (not decided, needs the researcher):
1. MV-P as designed cannot identify a moving lower bound: the gate's stop-and-wait, not the Comfort Space size,
   decides the passing distance. Options: (a) accept MV-P as UNIDENTIFIABLE and report it; (b) a new documented
   geometry version where the AMR cannot wait out the encounter (e.g. walker overtaken / crossing / bottleneck);
   the gate and safety layer stay unchanged.
2. Static: at C* = 5.0 the predicted comfort does not cross the target within alpha ≥ 0.45, so narrower widths
   in the pilot will mainly show where navigation fails, i.e. an operational minimum, not a comfort bound.

