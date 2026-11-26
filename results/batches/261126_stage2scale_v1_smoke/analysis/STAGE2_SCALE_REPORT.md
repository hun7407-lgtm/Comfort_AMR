# Stage 2 uniform-scale sweep — report

Phase **smoke**. Launches 40 (infra-excluded 0).

> Predicted comfort = published comfort–distance curves (comfort_reference.py) evaluated at the ACTUAL closest centre distance and bearing. Not a measurement of anyone's comfort, not an experimentally validated comfort distance. A smallest passing alpha without an observed comfort violation is NOT a comfort lower bound.

## Shapes (alpha = 1, normalised to the 0.90 m anchor)

| state | raw F/S/R | alpha=1 F/S/R |
|---|---|---|
| station_engaged | 0.800/0.700/0.600 | 0.9000/0.7875/0.6750 |
| stationary | 0.800/0.900/1.050 | 0.6857/0.7714/0.9000 |
| moving | 1.200/0.900/0.900 | 0.9000/0.6750/0.6750 |

## Cells (target 5.0)

| scenario | geometry | condition | n | nav k/n | joint k/n (95% CI) | median d_cc m (completed) | median edge m | median comfort | term. PS | comfort DL | contact | classes |
|---|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---|
| EN-R | w2.60 | B1 | 1 | 1/1 | 0/1 (0.0, 0.7935) | 0.5268 | 0.2769 | 3.709 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | B2_r0.900 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.5796 | 1.3346 | 5.587 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.45 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 0.9397 | 0.6951 | 5.324 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a1.00 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.3643 | 1.1169 | 5.574 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | B1 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8358 | 0.5619 | 5.7535 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | B2_r0.900 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0322 | 0.7723 | 6.336 | 0 | 0 | 0 | NORMAL_YIELD |
| MV-P | w3.00_g0.65 | S3_a0.45 | 2 | 2/2 | 1/2 (0.0945, 0.9055) | 0.7359 | 0.484 | 5.2575 | 0 | 0 | 0 | NORMAL_YIELD |
| MV-P | w3.00_g0.65 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.805 | 0.5409 | 5.6225 | 0 | 0 | 0 | NORMAL_YIELD |
| MV-P | w3.00_g0.80 | B1 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9099 | 0.6348 | 6.0235 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.80 | B2_r0.900 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9009 | 0.611 | 5.994 | 0 | 0 | 0 | NORMAL_YIELD |
| MV-P | w3.00_g0.80 | S3_a0.45 | 2 | 1/2 | 1/2 (0.0945, 0.9055) | 0.6944 | 0.3741 | 5.517 | 0 | 0 | 1 | NORMAL_YIELD |
| MV-P | w3.00_g0.80 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7555 | 0.4784 | 5.3855 | 0 | 0 | 0 | NORMAL_YIELD |
| MV-P | w3.00_g1.00 | B1 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0903 | 0.8218 | 6.4315 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | B2_r0.900 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1983 | 0.9014 | 6.565 | 0 | 0 | 0 | NORMAL_YIELD |
| MV-P | w3.00_g1.00 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1167 | 0.8357 | 6.475 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1639 | 0.8594 | 6.5315 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | B1 | 1 | 0/1 | 0/1 (0.0, 0.7935) | 0.4957 | 0.2485 | None | 0 | 0 | 1 | SUCCESS |
| ST-F | w2.60 | B2_r0.900 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.5796 | 1.3326 | 6.611 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.45 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 0.9422 | 0.6998 | 6.046 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a1.00 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.3715 | 1.1278 | 6.575 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | B1 | 1 | 1/1 | 0/1 (0.0, 0.7935) | 0.5209 | 0.2745 | 3.696 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | B2_r0.900 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.577 | 1.3275 | 5.581 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.45 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.0482 | 0.8032 | 5.455 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a1.00 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.5801 | 1.3345 | 5.579 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | B1 | 1 | 1/1 | 0/1 (0.0, 0.7935) | 0.5352 | 0.2896 | 3.638 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | B2_r0.900 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.5765 | 1.3315 | 6.475 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.45 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 0.999 | 0.7532 | 5.842 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a1.00 | 1 | 1/1 | 1/1 (0.2065, 1.0) | 1.4511 | 1.2046 | 6.358 | 0 | 0 | 0 | SUCCESS |

## Identification and selection (target 5.0)

| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |
|---|---|---|---|---|---|
| ST-F | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4293 |  |
| ST-S | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4521 |  |
| ST-R | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.5319 |  |
| EN-R | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4246 |  |
| MV-P | w3.00_g0.65 | UNIDENTIFIABLE_GEOMETRY | None | 0.0691 |  |
| MV-P | w3.00_g0.80 | UNIDENTIFIABLE_GEOMETRY | None | 0.0611 |  |
| MV-P | w3.00_g1.00 | UNIDENTIFIABLE_GEOMETRY | None | 0.0472 |  |
| MV-O | — | INFEASIBLE | — | — | {"walker_speed_mps": 1.04, "amr_max_speed_mps": 1.0, "gain_m": 3.0, "note": "AMR max speed <= walker animation speed: cannot overtake; no numeric overtaking comfort curve either"} |

| state | geometry | status | alpha* | per-context status |
|---|---|---|---|---|
| stationary | w2.60 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.45 | {'ST-F': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| station_engaged | w2.60 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.45 | {'EN-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| moving | w3.00_g0.65 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |
| moving | w3.00_g0.80 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |
| moving | w3.00_g1.00 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |

## Limits

- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.
- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).
- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.
- Overtaking: not run (walker faster than the AMR's limit).

