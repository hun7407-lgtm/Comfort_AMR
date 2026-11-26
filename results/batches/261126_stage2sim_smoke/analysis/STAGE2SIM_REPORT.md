# Stage 2 (simulation) — minimum Comfort Space by literature-predicted comfort

> **Predicted comfort** = what published human-rating curves give for the passage the AMR actually made (`comfort_core/comfort_reference.py`). It is not a measurement of comfort and not an experimentally validated comfort distance. Same reference model for every policy.

Trials scored: 15 (excluded infra: 0). Ladder [0.9, 0.81, 0.72, 0.63, 0.54, 0.45, 0.36, 0.3] m; corridors {"tight": {"repeats": 5}, "wide": {"repeats": 3}}, selection on `tight`. Metric `predicted_comfort_at_closest`; pass = violation rate ≤ 0.2 and navigation OK rate ≥ 0.8 (ASSUMPTIONS), monotone over the ladder. Primary target 5.0 (ASSUMPTION).

## References used

- N22a_B: Neggers et al. 2022, Int J Soc Robot 14:561, Table 1, study B (AGV) (standing still, robot 0.35 m/s); right a0 5.39 sigma 46.14 cm, front a0 5.62 sigma 44.13 cm, left a0 5.39 sigma 47.21 cm, back a0 4.58 sigma 39.11 cm
- N22a_A: Neggers et al. 2022, Int J Soc Robot 14:561, Table 1, study A (Pepper) (standing still, robot 0.35 m/s); right a0 6.15 sigma 47.25 cm, front a0 6.15 sigma 41.51 cm, left a0 6.04 sigma 49.52 cm, back a0 3.92 sigma 35.65 cm
- N22b_E1: Neggers et al. 2022, Front Robot AI 9:915972, Exp. 1, Table 2 (eta = 0) (walking, passed head-on, robot 0.35 m/s); right a0 5.71 sigma 44.2 cm, front a0 5.71 sigma 44.2 cm, left a0 5.71 sigma 44.2 cm, back a0 5.71 sigma 44.2 cm

## Literature-required extent (person centre → AMR edge, m) per target

| scenario | 4.0 | 4.5 | 5.0 | 5.5 | 6.0 |
|---|---|---|---|---|---|
| SE_rear (rear) | 0.3306 | 0.4248 | 0.5551 | 0.8727 | None |
| ST_front (front) | 0.3052 | 0.3762 | 0.4561 | 0.5526 | 0.6866 |
| ST_side (side) | 0.3621 | 0.4435 | 0.5372 | 0.656 | 0.842 |
| MV_pass (side) | 0.2996 | 0.369 | 0.4464 | 0.5386 | 0.6625 |

## Per condition

| scenario | corridor | version | size m | n | nav OK | pred. comfort median (min) | min centre m | min edge m | hold s | viol@4.0 | viol@4.5 | viol@5.0 | viol@5.5 | viol@6.0 |
|---|---|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| MV_pass | tight | B1 | None | 1 | 0.0 | 4.512 (4.512) | 0.611 | 0.249 | 0.22 | 0.0 | 0.0 | 1.0 | 1.0 | 1.0 |
| MV_pass | tight | static | 0.9 | 1 | 1.0 | 6.377 (6.377) | 1.054 | 0.667 | 1.05 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| MV_pass | tight | static | 0.54 | 1 | 1.0 | 5.623 (5.623) | 0.805 | 0.484 | 1.23 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| SE_rear | tight | B1 | None | 1 | 1.0 | 3.589 (3.589) | 0.507 | 0.263 | 0.22 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| SE_rear | tight | adaptive | 0.54 | 1 | 1.0 | 5.525 (5.525) | 1.149 | 0.901 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| SE_rear | tight | static | 0.9 | 1 | 0.0 | 5.761 (5.761) | 1.68 | 1.361 | 10.77 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| SE_rear | tight | static | 0.54 | 1 | 1.0 | 5.527 (5.527) | 1.152 | 0.903 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| ST_front | tight | B1 | None | 1 | 1.0 | 3.708 (3.708) | 0.506 | 0.263 | 0.22 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| ST_front | tight | adaptive | 0.54 | 1 | 0.0 | 6.472 (6.472) | 1.198 | 0.918 | 11.78 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_front | tight | static | 0.9 | 1 | 0.0 | 6.613 (6.613) | 1.591 | 1.304 | 10.57 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_front | tight | static | 0.54 | 1 | 1.0 | 6.439 (6.439) | 1.156 | 0.905 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_side | tight | B1 | None | 1 | 1.0 | 3.5 (3.5) | 0.515 | 0.269 | 4.9 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| ST_side | tight | adaptive | 0.54 | 1 | 0.0 | 5.968 (5.968) | 1.248 | 0.949 | 7.12 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| ST_side | tight | static | 0.9 | 1 | 0.0 | 6.197 (6.197) | 1.562 | 1.291 | 8.8 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_side | tight | static | 0.54 | 1 | 0.0 | 6.02 (6.02) | 1.171 | 0.917 | 20.67 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |

## Selection (smallest size that passes)

| scenario | corridor/version | 4.0 | 4.5 | 5.0 | 5.5 | 6.0 |
|---|---|---|---|---|---|---|
| SE_rear | tight/static | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | INFEASIBLE_AT_START |
| SE_rear | tight/adaptive | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | INFEASIBLE_AT_START |
| SE_rear | wide/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| SE_rear | wide/adaptive | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_front | tight/static | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY |
| ST_front | tight/adaptive | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY |
| ST_front | wide/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_front | wide/adaptive | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_side | tight/static | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY |
| ST_side | tight/adaptive | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | NEEDS_POLICY | INFEASIBLE_AT_START |
| ST_side | wide/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_side | wide/adaptive | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| MV_pass | tight/static | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.9 (OK) |
| MV_pass | wide/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| MV_overtake | — | INFEASIBLE | INFEASIBLE | INFEASIBLE | INFEASIBLE | INFEASIBLE |

## Not run / limits

- **MV_overtake: INFEASIBLE** — {"amr_max_speed_mps": 1.0, "walker_speed_mps": 1.04, "amr_can_overtake": false, "distance_needed_m": null, "reference_curve": null, "note": "Overtaking needs the AMR faster than the walker; the walker's animation speed cannot be lowered in this simulation (humans.walk_speed off). No published overtaking comfort curve with numeric parameters either (Neggers 2022 Front. Exp. 2 gives them only in Fig. 10/11)."}
- Reference curves were measured with robots at 0.35 m/s; this AMR passes at the speed in the trial table (column `amr_speed_at_closest_mps`). Speed is not in the model (Neggers 2022 Front. gives per-speed parameters only in figures), so predicted comfort is likely optimistic at higher AMR speed.
- station_engaged uses the standing-participant curve; task engagement itself is not in any reference.
- Moving: head-on passing only, one curve for all sides; overtaking has no numeric reference.
- Out-of-view time, min TTC (constant velocity, contact = person radius + AMR half width), relative speed: kinematic indicators, not comfort.
- Run-to-run variance: identical trials are repeated; see the min/median spread per condition.

Files: `stage2sim_trials.csv`, `stage2sim_summary.json`, `B3L_c*.yaml`, `figures/`.
