# Stage 2 uniform-scale sweep — report

Phase **pilot**. Launches 110 (infra-excluded 0).

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
| EN-R | w2.20 | B1 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.5054 | 0.2616 | 3.588 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | B2_r0.950 | 2 | 0/2 | 0/2 (0.0, 0.6576) | None | 1.1182 | None | 2 | 0 | 0 | PROTECTIVE_STOP |
| EN-R | w2.20 | S3_a0.45 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.7349 | 0.4895 | 4.7955 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8941 | 0.6438 | 5.248 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1274 | 0.8781 | 5.5115 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | B1 | 2 | 1/2 | 0/2 (0.0, 0.6576) | 0.5074 | 0.2526 | 3.683 | 0 | 0 | 1 | SUCCESS |
| EN-R | w2.60 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.3847 | 1.1353 | 5.5725 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.45 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.7349 | 0.4894 | 4.798 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8865 | 0.6398 | 5.2285 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.60 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1361 | 0.8851 | 5.5185 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | B1 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7959 | 0.5199 | 5.5605 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8832 | 0.5529 | 5.9345 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8126 | 0.5047 | 5.6165 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8594 | 0.5882 | 5.8475 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9261 | 0.6365 | 6.074 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.80 | B1 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8149 | 0.5327 | 5.6125 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.80 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.996 | 0.6739 | 6.2525 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.80 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8456 | 0.5669 | 5.783 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.80 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8221 | 0.5295 | 5.6975 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.80 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8915 | 0.5799 | 5.9575 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | B1 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1193 | 0.8416 | 6.4775 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.198 | 0.9084 | 6.5645 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1162 | 0.8346 | 6.474 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.118 | 0.8344 | 6.4765 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g1.00 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1316 | 0.8469 | 6.4945 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.20 | B1 | 2 | 1/2 | 0/2 (0.0, 0.6576) | 0.5 | 0.2393 | 3.658 | 1 | 0 | 1 | PROTECTIVE_STOP, SUCCESS |
| ST-F | w2.20 | B2_r0.950 | 2 | 0/2 | 0/2 (0.0, 0.6576) | None | 1.0923 | None | 2 | 0 | 0 | PROTECTIVE_STOP |
| ST-F | w2.20 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7467 | 0.4991 | 5.277 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8939 | 0.6484 | 5.8985 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.20 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1593 | 0.9084 | 6.442 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | B1 | 2 | 0/2 | 0/2 (0.0, 0.6576) | 0.4891 | 0.2228 | None | 1 | 0 | 2 | PROTECTIVE_STOP, SUCCESS |
| ST-F | w2.60 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.3769 | 1.1275 | 6.5765 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7324 | 0.4878 | 5.2025 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8959 | 0.6508 | 5.9045 | 0 | 0 | 0 | SUCCESS |
| ST-F | w2.60 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.1391 | 0.89 | 6.419 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.20 | B1 | 2 | 1/2 | 0/2 (0.0, 0.6576) | 0.5321 | 0.2624 | 3.766 | 1 | 0 | 1 | PROTECTIVE_STOP, SUCCESS |
| ST-R | w2.20 | B2_r0.950 | 2 | 0/2 | 0/2 (0.0, 0.6576) | None | 1.1126 | None | 2 | 0 | 0 | PROTECTIVE_STOP |
| ST-R | w2.20 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8279 | 0.5817 | 5.0915 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0831 | 0.8338 | 5.481 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.20 | S3_a1.00 | 2 | 0/2 | 0/2 (0.0, 0.6576) | None | 1.0924 | None | 2 | 0 | 0 | PROTECTIVE_STOP |
| ST-R | w2.60 | B1 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.5208 | 0.273 | 3.694 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.3698 | 1.1219 | 5.573 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.8347 | 0.587 | 5.1095 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.0747 | 0.8251 | 5.4775 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.3784 | 1.1292 | 5.5715 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.20 | B1 | 2 | 1/2 | 0/2 (0.0, 0.6576) | 0.5455 | 0.2673 | 3.709 | 1 | 0 | 1 | PROTECTIVE_STOP, SUCCESS |
| ST-S | w2.20 | B2_r0.950 | 2 | 0/2 | 0/2 (0.0, 0.6576) | None | 1.1058 | None | 2 | 0 | 0 | PROTECTIVE_STOP |
| ST-S | w2.20 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7846 | 0.5385 | 5.1 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.20 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9823 | 0.732 | 5.789 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.20 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.2477 | 1.0017 | 6.2805 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | B1 | 2 | 2/2 | 0/2 (0.0, 0.6576) | 0.5435 | 0.2966 | 3.698 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | B2_r0.950 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.3748 | 1.1275 | 6.3605 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.45 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.7797 | 0.5331 | 5.1005 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.70 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 0.9745 | 0.7301 | 5.786 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a1.00 | 2 | 2/2 | 2/2 (0.3424, 1.0) | 1.2449 | 0.9963 | 6.2315 | 0 | 0 | 0 | SUCCESS |

## Identification and selection (target 5.0)

| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |
|---|---|---|---|---|---|
| ST-F | w2.20 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4126 |  |
| ST-F | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4067 |  |
| ST-S | w2.20 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4631 |  |
| ST-S | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4652 |  |
| ST-R | w2.20 | INSUFFICIENT_DATA | 0.45 | None | no completed passage at the largest or the smallest alpha: distance response unknown |
| ST-R | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.5437 |  |
| EN-R | w2.20 | IDENTIFIED_BRACKET | 0.7 | 0.3925 |  |
| EN-R | w2.60 | IDENTIFIED_BRACKET | 0.7 | 0.4012 |  |
| MV-P | w3.00_g0.65 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.1135 |  |
| MV-P | w3.00_g0.80 | UNIDENTIFIABLE_GEOMETRY | None | 0.0459 |  |
| MV-P | w3.00_g1.00 | UNIDENTIFIABLE_GEOMETRY | None | 0.0154 |  |
| MV-O | — | INFEASIBLE | — | — | {"walker_speed_mps": 1.04, "amr_max_speed_mps": 1.0, "gain_m": 3.0, "note": "AMR max speed <= walker animation speed: cannot overtake; no numeric overtaking comfort curve either"} |

| state | geometry | status | alpha* | per-context status |
|---|---|---|---|---|
| stationary | w2.20 | INSUFFICIENT_DATA | None | {'ST-F': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'INSUFFICIENT_DATA'} |
| stationary | w2.60 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.45 | {'ST-F': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| station_engaged | w2.20 | COMMON_ALPHA_IDENTIFIED | 0.7 | {'EN-R': 'IDENTIFIED_BRACKET'} |
| station_engaged | w2.60 | COMMON_ALPHA_IDENTIFIED | 0.7 | {'EN-R': 'IDENTIFIED_BRACKET'} |
| moving | w3.00_g0.65 | COMMON_ALPHA_OPERATIONAL_ONLY | 0.45 | {'MV-P': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED'} |
| moving | w3.00_g0.80 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |
| moving | w3.00_g1.00 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |

## Limits

- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.
- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).
- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.
- Overtaking: not run (walker faster than the AMR's limit).

