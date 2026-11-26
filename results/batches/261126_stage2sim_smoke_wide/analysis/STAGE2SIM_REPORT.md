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
| MV_pass | wide | B1 | None | 1 | 0.0 | 4.243 (4.243) | 0.573 | 0.221 | 0.22 | 0.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| MV_pass | wide | static | 0.9 | 1 | 1.0 | 6.381 (6.381) | 1.056 | 0.81 | 1.4 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| MV_pass | wide | static | 0.54 | 1 | 1.0 | 5.667 (5.667) | 0.815 | 0.574 | 1.5 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| SE_rear | wide | B1 | None | 1 | 1.0 | 3.697 (3.697) | 0.521 | 0.276 | 0.23 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| SE_rear | wide | adaptive | 0.54 | 1 | 1.0 | 5.543 (5.543) | 1.207 | 0.956 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| SE_rear | wide | static | 0.9 | 1 | 1.0 | 5.579 (5.579) | 1.606 | 1.361 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| SE_rear | wide | static | 0.54 | 1 | 1.0 | 5.544 (5.544) | 1.202 | 0.954 | 0.2 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 |
| ST_front | wide | B1 | None | 1 | 0.0 | 3.756 (3.756) | 0.522 | 0.239 | 11.43 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| ST_front | wide | adaptive | 0.54 | 1 | 1.0 | 6.487 (6.487) | 1.207 | 0.957 | 0.2 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_front | wide | static | 0.9 | 1 | 1.0 | 6.611 (6.611) | 1.582 | 1.338 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_front | wide | static | 0.54 | 1 | 1.0 | 6.486 (6.486) | 1.205 | 0.955 | 0.23 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_side | wide | B1 | None | 1 | 1.0 | 3.908 (3.908) | 0.575 | 0.33 | 0.45 | 1.0 | 1.0 | 1.0 | 1.0 | 1.0 |
| ST_side | wide | adaptive | 0.54 | 1 | 1.0 | 6.232 (6.232) | 1.204 | 0.959 | 0.23 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_side | wide | static | 0.9 | 1 | 1.0 | 6.431 (6.431) | 1.584 | 1.333 | 0.22 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| ST_side | wide | static | 0.54 | 1 | 1.0 | 6.227 (6.227) | 1.226 | 0.98 | 0.23 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |

## Selection (smallest size that passes)

| scenario | corridor/version | 4.0 | 4.5 | 5.0 | 5.5 | 6.0 |
|---|---|---|---|---|---|---|
| SE_rear | tight/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| SE_rear | tight/adaptive | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| SE_rear | wide/static | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | INFEASIBLE_AT_START |
| SE_rear | wide/adaptive | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | INFEASIBLE_AT_START |
| ST_front | tight/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_front | tight/adaptive | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_front | wide/static | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) |
| ST_front | wide/adaptive | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) |
| ST_side | tight/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_side | tight/adaptive | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| ST_side | wide/static | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) |
| ST_side | wide/adaptive | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) |
| MV_pass | tight/static | NO_DATA | NO_DATA | NO_DATA | NO_DATA | NO_DATA |
| MV_pass | wide/static | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.54 (OK_AT_LADDER_END) | 0.9 (OK) |
| MV_overtake | — | INFEASIBLE | INFEASIBLE | INFEASIBLE | INFEASIBLE | INFEASIBLE |

## Not run / limits

- **MV_overtake: INFEASIBLE** — {"amr_max_speed_mps": 1.0, "walker_speed_mps": 1.04, "amr_can_overtake": false, "distance_needed_m": null, "reference_curve": null, "note": "Overtaking needs the AMR faster than the walker; the walker's animation speed cannot be lowered in this simulation (humans.walk_speed off). No published overtaking comfort curve with numeric parameters either (Neggers 2022 Front. Exp. 2 gives them only in Fig. 10/11)."}
- Reference curves were measured with robots at 0.35 m/s; this AMR passes at the speed in the trial table (column `amr_speed_at_closest_mps`). Speed is not in the model (Neggers 2022 Front. gives per-speed parameters only in figures), so predicted comfort is likely optimistic at higher AMR speed.
- station_engaged uses the standing-participant curve; task engagement itself is not in any reference.
- Moving: head-on passing only, one curve for all sides; overtaking has no numeric reference.
- Out-of-view time, min TTC (constant velocity, contact = person radius + AMR half width), relative speed: kinematic indicators, not comfort.
- Run-to-run variance: identical trials are repeated; see the min/median spread per condition.

Files: `stage2sim_trials.csv`, `stage2sim_summary.json`, `B3L_c*.yaml`, `figures/`.
