# 261112_stage1_size_sweep: Stage 1 — Comfort Space size vs navigation feasibility

Cross scenario, 330 AMR1 trials over 5 sizes x 2 policies; seeds per cell: Static 33-33, B3-O 33-33.

Same campaign config for every condition of a seed: people, walks, state transitions, AMR missions and start times, map and Nav2 parameters are identical; only the `comfort_space` overlay differs (`configs/stage1/`). Physical safety layer unchanged (scripts/check_stage1.py group 4).

## Thresholds

Definition: only `comfort_gate_hold` (the yield gate holding the robot out of a Comfort Space when the 12 s no-progress rule fired) is a Comfort-Space-induced deadlock. R_deadlock_onset = smallest size with >= 1 such deadlock(s); R_nav_max = largest size below it with none.

**Size-induced (primary).** Seeds that already end in a gate hold at the smallest swept size (0.80 m) are impassable for the policy regardless of size (scenario-intrinsic) and are excluded; for the other seeds every gate hold is a success -> hold transition caused by growing the space.

| policy | R_nav_max | R_deadlock_onset | size-induced gate holds by size (n) | excluded seeds (hold at smallest size) | non-monotonic seeds | note |
|---|---|---|---|---|---|---|
| Static | 0.80 m | 1.00 m | 0.80: 0/29, 1.00: 4/30, 1.20: 5/30, 1.40: 18/30, 1.60: 27/30 | 5, 25, 30 | seed 2 passes again at 1.20; seed 17 passes again at 1.40; seed 23 passes again at 1.40; seed 26 passes again at 1.20 | onset between 0.80 and 1.00 m; refine with sizes in (0.80, 1.00) at 0.05-0.10 m steps |
| B3-O | 0.80 m | 1.00 m | 0.80: 0/31, 1.00: 3/31, 1.20: 2/30, 1.40: 1/31, 1.60: 2/32 | 11 | seed 12 passes again at 1.40, 1.60; seed 17 passes again at 1.60; seed 23 passes again at 1.60; seed 25 passes again at 1.40; seed 26 passes again at 1.40 | onset between 0.80 and 1.00 m; refine with sizes in (0.80, 1.00) at 0.05-0.10 m steps |

**Any gate hold (literal rule, for reference).** Every comfort gate hold counts, including scenario-intrinsic ones; with a non-zero floor at the smallest size this reading collapses to the smallest size and says nothing about the size effect.

| policy | R_nav_max | R_deadlock_onset | comfort deadlocks by size (n) | note |
|---|---|---|---|---|
| Static | below the sweep | 0.80 m | 0.80: 3/32, 1.00: 5/33, 1.20: 6/33, 1.40: 20/33, 1.60: 30/33 | comfort-induced deadlock already at the smallest swept size 0.80 m; R_nav_max lies below the sweep |
| B3-O | below the sweep | 0.80 m | 0.80: 1/32, 1.00: 4/32, 1.20: 2/31, 1.40: 2/32, 1.60: 2/33 | comfort-induced deadlock already at the smallest swept size 0.80 m; R_nav_max lies below the sweep |

## Summary by size

| size (m) | Static success | B3-O success | Static comfort DL | B3-O comfort DL | Static other fail | B3-O other fail | Static travel s | B3-O travel s | Static min clr m | B3-O min clr m |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 26/32 | 28/32 | 3 | 1 | 3 | 3 | 48.0 | 47.4 | 0.37 | 0.20 |
| 1.00 | 28/33 | 26/32 | 5 | 4 | 0 | 2 | 49.3 | 46.9 | 0.40 | 0.27 |
| 1.20 | 25/33 | 26/31 | 6 | 2 | 2 | 3 | 49.6 | 47.4 | 0.40 | 0.30 |
| 1.40 | 9/33 | 27/32 | 20 | 2 | 4 | 3 | 50.4 | 48.3 | 0.36 | 0.31 |
| 1.60 | 0/33 | 29/33 | 30 | 2 | 3 | 2 | - | 49.2 | 0.56 | 0.36 |

Full per-size, per-mechanism and per-AMR (a1, a2) tables: `tables.md`. Thresholds as data: `thresholds.json`.

## Figures

| | file | shows |
|---|---|---|
| P1 | figures/P1_success_vs_size.png | success rate vs size, both policies, ±1 SE |
| P2 | figures/P2_comfort_deadlock_vs_size.png | comfort-induced deadlock rate vs size; onset (dashed) and R_nav_max (dotted) |
| P3 | figures/P3_travel_time_vs_size.png | travel time of successful trials |
| P4 | figures/P4_travel_distance_vs_size.png | distance driven vs first Nav2 plan |
| P5 | figures/P5_min_clearance_vs_size.png | closest approach to a person |
| P6 | figures/P6_size_x_seed_matrix.png | outcome per size x seed, mechanism lettered |
| P7 | figures/P7_threshold_geometry.png | aisle width left outside the Comfort Spaces, per size |
| P8 | figures/P8_failure_mechanism.png | how AMR1 stopped, per size |
| P9 | figures/P9_fleet_vs_size.png | a1 / a2 success vs size |

## Reading notes

- A trial that did not run has no row; n is printed for every cell.
- Travel time of a trial that did not reach the goal is time-to-stop and is excluded from the travel-time means.
- Post-evade stalls and Nav2 aborts are reported beside the comfort deadlocks and never added into the threshold; they are faults of the evasion module / planner, seen under every policy in the earlier campaigns.
- Static and B3-O share the size axis by the moving-front extent at 1.0 m/s (1.20 m unscaled); B3-O's other extents are smaller at every size (see tables.md, applied extents).
- None of these sizes is an experimentally validated comfort distance; Stage 2 measures that. The physical safety layer is identical in every trial.
