# Stage 2 uniform-scale sweep — report

Phase **coarse**. Launches 180 (infra-excluded 0).

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
| EN-R | w2.20 | B1 | 5 | 4/5 | 0/5 (0.0, 0.4345) | 0.5132 | 0.2679 | 3.6295 | 0 | 0 | 1 | SUCCESS, TEMPORARY_HOLD |
| EN-R | w2.20 | B2_r0.950 | 5 | 1/5 | 1/5 (0.0362, 0.6245) | 1.3247 | 1.0884 | 5.569 | 4 | 0 | 0 | PROTECTIVE_STOP, TEMPORARY_HOLD |
| EN-R | w2.20 | S3_a0.45 | 5 | 5/5 | 0/5 (0.0, 0.4345) | 0.7301 | 0.4853 | 4.777 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.50 | 5 | 5/5 | 0/5 (0.0, 0.4345) | 0.7412 | 0.4943 | 4.815 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.60 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8339 | 0.5846 | 5.107 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.70 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.898 | 0.6522 | 5.252 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.80 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.9781 | 0.7307 | 5.379 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.90 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.0352 | 0.7882 | 5.448 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a1.00 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.1286 | 0.8797 | 5.517 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | B1 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.7229 | 0.4571 | 5.211 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | B2_r0.950 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.9343 | 0.6216 | 6.098 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.45 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8274 | 0.5531 | 5.72 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.50 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8049 | 0.5249 | 5.622 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.60 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.7728 | 0.4996 | 5.472 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.70 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8266 | 0.5518 | 5.716 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.80 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8147 | 0.5332 | 5.666 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a0.90 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8695 | 0.5807 | 5.885 | 0 | 0 | 0 | SUCCESS |
| MV-P | w3.00_g0.65 | S3_a1.00 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.9018 | 0.6082 | 5.998 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | B1 | 5 | 3/5 | 0/5 (0.0, 0.4345) | 0.5164 | 0.2726 | 3.718 | 0 | 0 | 2 | NORMAL_YIELD, SUCCESS |
| ST-R | w2.60 | B2_r0.950 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.3835 | 1.1362 | 5.573 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.45 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8255 | 0.5804 | 5.085 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.50 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.8733 | 0.6307 | 5.202 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.60 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.9689 | 0.7234 | 5.376 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.70 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.0717 | 0.8217 | 5.479 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.80 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.1542 | 0.9041 | 5.524 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.90 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.2821 | 1.0351 | 5.559 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a1.00 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.3746 | 1.1262 | 5.576 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | B1 | 5 | 5/5 | 0/5 (0.0, 0.4345) | 0.5343 | 0.2843 | 3.644 | 0 | 0 | 0 | NORMAL_YIELD, SUCCESS |
| ST-S | w2.60 | B2_r0.950 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.3705 | 1.1238 | 6.296 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.45 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.7828 | 0.5359 | 5.126 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.50 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.7938 | 0.5472 | 5.151 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.60 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.895 | 0.648 | 5.55 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.70 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 0.9785 | 0.7295 | 5.816 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.80 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.0551 | 0.8052 | 5.981 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.90 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.148 | 0.9013 | 6.176 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a1.00 | 5 | 5/5 | 5/5 (0.5655, 1.0) | 1.2416 | 0.9944 | 6.28 | 0 | 0 | 0 | SUCCESS |

## Identification and selection (target 5.0)

| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |
|---|---|---|---|---|---|
| ST-F | — | UNIDENTIFIABLE_GEOMETRY | — | — | null |
| ST-S | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.4588 |  |
| ST-R | w2.60 | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 0.5491 |  |
| EN-R | w2.20 | IDENTIFIED_BRACKET | 0.6 | 0.3985 |  |
| MV-P | w3.00_g0.65 | UNIDENTIFIABLE_GEOMETRY | None | 0.0744 |  |
| MV-O | — | INFEASIBLE | — | — | {"walker_speed_mps": 1.04, "amr_max_speed_mps": 1.0, "gain_m": 3.0, "note": "AMR max speed <= walker animation speed: cannot overtake; no numeric overtaking comfort curve either"} |

| state | geometry | status | alpha* | per-context status |
|---|---|---|---|---|
| stationary | w2.60 | UNIDENTIFIABLE_GEOMETRY | None | {'ST-S': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-R': 'FLOOR_LIMITED_BOUND_NOT_OBSERVED', 'ST-F': 'UNIDENTIFIABLE_GEOMETRY'} |
| station_engaged | w2.20 | COMMON_ALPHA_IDENTIFIED | 0.6 | {'EN-R': 'IDENTIFIED_BRACKET'} |
| moving | w3.00_g0.65 | UNIDENTIFIABLE_GEOMETRY | None | {'MV-P': 'UNIDENTIFIABLE_GEOMETRY'} |

## Limits

- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.
- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).
- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.
- Overtaking: not run (walker faster than the AMR's limit).

