# Stage 2 uniform-scale sweep — report

Phase **confirm**. Launches 100 (infra-excluded 0).

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
| EN-R | w2.20 | B1 | 10 | 4/10 | 0/10 (0.0, 0.2775) | 0.5151 | 0.246 | 3.71 | 4 | 0 | 6 | PROTECTIVE_STOP, SUCCESS |
| EN-R | w2.20 | B2_r0.950 | 10 | 0/10 | 0/10 (0.0, 0.2775) | None | 1.1079 | None | 10 | 0 | 0 | PROTECTIVE_STOP |
| EN-R | w2.20 | S3_a0.50 | 10 | 10/10 | 0/10 (0.0, 0.2775) | 0.7439 | 0.497 | 4.83 | 0 | 0 | 0 | SUCCESS |
| EN-R | w2.20 | S3_a0.60 | 10 | 10/10 | 10/10 (0.7225, 1.0) | 0.8351 | 0.5876 | 5.1115 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | B1 | 10 | 9/10 | 0/10 (0.0, 0.2775) | 0.5225 | 0.2776 | 3.727 | 0 | 0 | 1 | NORMAL_YIELD, SUCCESS |
| ST-R | w2.60 | B2_r0.950 | 10 | 10/10 | 10/10 (0.7225, 1.0) | 1.3718 | 1.1217 | 5.5715 | 0 | 0 | 0 | SUCCESS |
| ST-R | w2.60 | S3_a0.45 | 10 | 10/10 | 10/10 (0.7225, 1.0) | 0.8282 | 0.582 | 5.0925 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | B1 | 10 | 10/10 | 0/10 (0.0, 0.2775) | 0.5508 | 0.3062 | 3.7435 | 0 | 0 | 0 | NORMAL_YIELD, SUCCESS |
| ST-S | w2.60 | B2_r0.950 | 10 | 10/10 | 10/10 (0.7225, 1.0) | 1.373 | 1.1265 | 6.3535 | 0 | 0 | 0 | SUCCESS |
| ST-S | w2.60 | S3_a0.45 | 10 | 10/10 | 10/10 (0.7225, 1.0) | 0.7814 | 0.5349 | 5.093 | 0 | 0 | 0 | SUCCESS |

## Identification and selection (target 5.0)

> **Confirm phase:** this batch holds only B1, B2, alpha* and the next smaller alpha, so the identification status below (which needs alpha 1.00 and the smallest alpha) is NOT meaningful here. Use `scripts/stage2scale_confirm.py` -> `CONFIRM_VERDICT.md` (status from coarse, cells from this batch).

| context | geometry | status | alpha* | d_cc response 1.00→min (m) | note |
|---|---|---|---|---|---|
| ST-F | — | UNIDENTIFIABLE_GEOMETRY | — | — | null |
| ST-S | w2.60 | UNIDENTIFIABLE_GEOMETRY | None | 0.0 |  |
| ST-R | w2.60 | UNIDENTIFIABLE_GEOMETRY | None | 0.0 |  |
| EN-R | w2.20 | UNIDENTIFIABLE_GEOMETRY | None | 0.0912 |  |
| MV-P | — | UNIDENTIFIABLE_GEOMETRY | — | — | null |
| MV-O | — | INFEASIBLE | — | — | {"walker_speed_mps": 1.04, "amr_max_speed_mps": 1.0, "gain_m": 3.0, "note": "AMR max speed <= walker animation speed: cannot overtake; no numeric overtaking comfort curve either"} |

| state | geometry | status | alpha* | per-context status |
|---|---|---|---|---|
| stationary | w2.60 | UNIDENTIFIABLE_GEOMETRY | None | {'ST-S': 'UNIDENTIFIABLE_GEOMETRY', 'ST-R': 'UNIDENTIFIABLE_GEOMETRY', 'ST-F': 'UNIDENTIFIABLE_GEOMETRY'} |
| station_engaged | w2.20 | UNIDENTIFIABLE_GEOMETRY | None | {'EN-R': 'UNIDENTIFIABLE_GEOMETRY'} |

## Limits

- Reference curves: robots at ~0.35 m/s, standing or walking participants without a task. station_engaged is a stationary-reference transfer. Rows flag `out_of_reference_domain`.
- Moving template frozen at 1.0 m/s; realised walker speed logged (`human_speed_at_closest_mps`).
- Smoke / pilot phases are diagnostic and are not pooled with the coarse or confirmatory evidence.
- Overtaking: not run (walker faster than the AMR's limit).

