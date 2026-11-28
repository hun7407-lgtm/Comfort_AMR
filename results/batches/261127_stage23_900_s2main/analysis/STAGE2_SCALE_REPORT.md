# Stage 2 uniform-scale sweep — report

Phase **s2main**. Launches 750 (infra-excluded 0).

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
| EN-R | w2.60 | S3_a0.50 | 30 | 30/30 | 0/30 (0.0, 0.1135) | 0.7461 | 0.5006 | 4.839 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.60 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.8311 | 0.5854 | 5.101 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.70 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.893 | 0.6442 | 5.2465 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.80 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.9773 | 0.7301 | 5.3815 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.90 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.0499 | 0.8022 | 5.4635 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.50 | 30 | 2/30 | 0/30 (0.0, 0.1135) | 0.4939 | 0.2092 | 4.155 | 0 | 0 | 28 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.60 | 30 | 3/30 | 0/30 (0.0, 0.1135) | 0.5164 | 0.2066 | 4.325 | 0 | 0 | 27 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.70 | 30 | 2/30 | 0/30 (0.0, 0.1135) | 0.5151 | 0.2092 | 4.6405 | 0 | 0 | 28 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.80 | 30 | 4/30 | 0/30 (0.0, 0.1135) | 0.5247 | 0.2111 | 4.4445 | 0 | 0 | 26 | SUCCESS |
| MV-P | w3.00_g0.40 | S3_a0.90 | 30 | 7/30 | 0/30 (0.0, 0.1135) | 0.5772 | 0.247 | 4.61 | 0 | 0 | 23 | SUCCESS |
| ST-F | w2.60 | S3_a0.50 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.762 | 0.5158 | 5.352 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.60 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.8299 | 0.5824 | 5.6585 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.70 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.8935 | 0.6455 | 5.8975 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.80 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.9798 | 0.7318 | 6.1415 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.90 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.0486 | 0.8005 | 6.2855 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.50 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.8766 | 0.629 | 5.211 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.60 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.9696 | 0.7212 | 5.3725 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.70 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.0695 | 0.8209 | 5.476 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.80 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.16 | 0.9116 | 5.527 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.90 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.2695 | 1.0214 | 5.559 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.50 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.8006 | 0.5539 | 5.186 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.60 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.8951 | 0.6479 | 5.553 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.70 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 0.9791 | 0.732 | 5.8045 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.80 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.0535 | 0.8048 | 5.9735 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.90 | 30 | 30/30 | 30/30 (0.8865, 1.0) | 1.1513 | 0.9042 | 6.1535 | 0 | 0 | 0 | SUCCESS |

## Identification and selection (target 5.0)

| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |
|---|---|---|---|---|---|
| ST-F | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.2866 |  |
| ST-S | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.3507 |  |
| ST-R | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.5 | 0.3929 |  |
| EN-R | w2.60 | IDENTIFIED_BRACKET | 0.6 | 0.3038 |  |
| MV-P | w3.00_g0.40 | UNIDENTIFIABLE_GEOMETRY | None | 0.0833 |  |

| state | geometry | status | alpha* | per-context status |
|---|---|---|---|---|
| stationary | w2.60 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.5 | {'ST-F': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| station_engaged | w2.60 | COMMON_ALPHA_IDENTIFIED | 0.6 | {'EN-R': 'IDENTIFIED_BRACKET'} |
| moving | w3.00_g0.40 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |

## Limits

- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.
- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).
- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.
- Overtaking: not run (walker faster than the AMR's limit).

