# Stage 2 uniform-scale sweep — report

Phase **s2pilot**. Launches 60 (infra-excluded 0).

> Predicted comfort = published comfort–distance curves (comfort_reference.py) evaluated at the ACTUAL closest centre distance and bearing. Not a measurement of anyone's comfort, not an experimentally validated comfort distance. A smallest passing alpha without an observed comfort violation is NOT a comfort lower bound.

## Shapes (alpha = 1, normalised to the R_ref anchor)

| state | raw F/S/R | alpha=1 F/S/R |
|---|---|---|
| station_engaged | 0.800/0.700/0.600 | 0.9500/0.8312/0.7125 |
| stationary | 0.800/0.900/1.050 | 0.7238/0.8143/0.9500 |
| moving | 1.200/0.900/0.900 | 0.9500/0.7125/0.7125 |

## Cells (target 5.0)

| scenario | geometry | condition | n | nav k/n | joint k/n (95% CI) | median d_cc m (completed) | median edge m | median comfort | term. PS | comfort DL | contact | classes |
|---|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---|
| EN-R | w2.20 | S3_a0.50 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.7401 | 0.495 | 4.8125 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8911 | 0.6436 | 5.2385 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.046 | 0.7971 | 5.457 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.50 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.7421 | 0.4969 | 4.819 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8785 | 0.6334 | 5.212 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.038 | 0.792 | 5.4455 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.25 | S3_a0.50 | 2 | 0/2 | 0/2 (0.0, 0.6576) | 0.5004 | 0.2044 | None | 0 | 0 | 2 | SUCCESS |
| MV-P | w3.00_g0.25 | S3_a0.70 | 2 | 0/2 | 0/2 (0.0, 0.6576) | 0.491 | 0.2058 | None | 0 | 0 | 2 | SUCCESS |
| MV-P | w3.00_g0.25 | S3_a0.90 | 2 | 0/2 | 0/2 (0.0, 0.6576) | 0.479 | 0.19 | None | 0 | 0 | 2 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.50 | 2 | 0/2 | 0/2 (0.0, 0.6576) | 0.5066 | 0.213 | None | 0 | 0 | 2 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.70 | 2 | 0/2 | 0/2 (0.0, 0.6576) | 0.5199 | 0.215 | None | 0 | 0 | 2 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.90 | 2 | 1/2 | 0/2 (0.0, 0.6576) | 0.644 | 0.2988 | 4.635 | 0 | 0 | 1 | SUCCESS |
| ST-F | w2.20 | S3_a0.50 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7736 | 0.5261 | 5.41 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8901 | 0.6441 | 5.8865 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.20 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0513 | 0.8009 | 6.292 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.50 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7577 | 0.5104 | 5.3305 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8897 | 0.6439 | 5.8835 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0409 | 0.7954 | 6.272 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.20 | S3_a0.50 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8759 | 0.627 | 5.2085 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0833 | 0.8355 | 5.483 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.20 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.2714 | 1.0213 | 5.559 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.50 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.909 | 0.6596 | 5.271 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0719 | 0.8233 | 5.4735 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.2643 | 1.014 | 5.555 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.20 | S3_a0.50 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7938 | 0.5459 | 5.1605 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9695 | 0.7218 | 5.7885 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.20 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1497 | 0.902 | 6.1985 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.50 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7873 | 0.5398 | 5.1245 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9717 | 0.7247 | 5.827 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.90 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1481 | 0.9018 | 6.2125 | 0 | 0 | 0 | SUCCESS |

## Identification and selection (target 5.0)

| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |
|---|---|---|---|---|---|
| ST-F | w2.20 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.2777 |  |
| ST-F | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.2832 |  |
| ST-S | w2.20 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.3559 |  |
| ST-S | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.3608 |  |
| ST-R | w2.20 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.3955 |  |
| ST-R | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.3553 |  |
| EN-R | w2.20 | IDENTIFIED_BRACKET | 0.7 | 0.3059 |  |
| EN-R | w2.60 | IDENTIFIED_BRACKET | 0.7 | 0.2959 |  |
| MV-P | w3.00_g0.25 | UNIDENTIFIABLE_GEOMETRY | None | -0.0214 |  |
| MV-P | w3.00_g0.40 | NO_FEASIBLE_ALPHA | None | 0.1374 |  |

| state | geometry | status | alpha* | per-context status |
|---|---|---|---|---|
| stationary | w2.20 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.5 | {'ST-F': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| stationary | w2.60 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.5 | {'ST-F': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| station_engaged | w2.20 | COMMON_ALPHA_IDENTIFIED | 0.7 | {'EN-R': 'IDENTIFIED_BRACKET'} |
| station_engaged | w2.60 | COMMON_ALPHA_IDENTIFIED | 0.7 | {'EN-R': 'IDENTIFIED_BRACKET'} |
| moving | w3.00_g0.25 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |
| moving | w3.00_g0.40 | NO_COMMON_FEASIBLE_ALPHA | None | {'MV-P': 'NO_FEASIBLE_ALPHA'} |

## Limits

- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.
- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).
- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.
- Overtaking: not run (walker faster than the AMR's limit).

