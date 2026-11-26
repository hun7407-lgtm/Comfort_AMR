# Stage 3 (final) — B1 / B2 / B3-O on holdout seeds 101–130 (30 seeds)

> Same seed = same people, missions, timing, navigation and safety layer; only the Comfort Space differs. B3-O sizes from R_nav_upper (Stage 1) and the Simulation-derived Provisional Lower Bound (Stage 2 proxy, not a human study). Rates: count / n (Wilson 95 % CI).

R_nav_upper = 0.9 m; B2 static = 0.9 m.

| Policy | System Success | Comfort Deadlock | All Failure | Travel Time (AMR 1, median s) | Yield Time (AMR 1 gate, median s) | Min Human Distance (median m) |
|---|---:|---:|---:|---:|---:|---:|
| B1 | 24/30 (80%, 63–90) | 0/30 (0%, 0–11) | 6/30 (20%, 10–37) | 41.93 | 0.0 | 0.231 |
| B2 | 22/30 (73%, 56–86) | 1/30 (3%, 1–17) | 8/30 (27%, 14–44) | 41.3 | 0.0 | 0.235 |
| B3-O | 23/30 (77%, 59–88) | 1/30 (3%, 1–17) | 7/30 (23%, 12–41) | 41.32 | 0.0 | 0.22 |
| B3-O LOWER | 28/30 (93%, 79–98) | 0/30 (0%, 0–11) | 2/30 (7%, 2–21) | 41.185 | 0.0 | 0.228 |
| B3-O UPPER | 25/30 (83%, 66–93) | 2/30 (7%, 2–21) | 5/30 (17%, 7–34) | 41.4 | 0.0 | 0.226 |

## More metrics (medians per trial)

| Policy | AMR1 / a1 / a2 success | robot-robot deadlock | hold s (3 AMRs) | replans | path m (AMR 1) | penetration max m | intrusion s | overlap m² | corridor m² | protective stop trials | contact trials |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| B1 | 27 / 30 / 25 of 30 | 0/30 (0%, 0–11) | 0.83 | 0.0 | 26.61 | 0.0 | 0.0 | None | None | 21/30 (70%, 52–83) | 24/30 (80%, 63–90) |
| B2 | 29 / 30 / 23 of 30 | 0/30 (0%, 0–11) | 2.2 | 1.5 | 26.67 | 0.665 | 5.465 | 1.04 | 6.869 | 8/30 (27%, 14–44) | 23/30 (77%, 59–88) |
| B3-O | 29 / 28 / 25 of 30 | 1/30 (3%, 1–17) | 1.74 | 1.0 | 26.68 | 0.429 | 4.47 | 0.603 | 6.415 | 10/30 (33%, 19–51) | 24/30 (80%, 63–90) |
| B3-O LOWER | 30 / 30 / 28 of 30 | 0/30 (0%, 0–11) | 0.62 | 1.5 | 26.67 | 0.185 | 1.315 | 0.262 | 6.113 | 12/30 (40%, 25–58) | 23/30 (77%, 59–88) |
| B3-O UPPER | 30 / 30 / 25 of 30 | 0/30 (0%, 0–11) | 3.325 | 2.5 | 26.695 | 0.675 | 6.3 | 1.021 | 6.866 | 9/30 (30%, 17–48) | 22/30 (73%, 56–86) |

## Paired tests (same seeds)

- **B2 vs B3-O** (n = 30): system success McNemar only-B2 4 / only-B3-O 5, p = 1.0; travel {'n': 28, 'median_diff': -0.07, 'p': 0.0902}; hold {'n': 30, 'median_diff': -0.11, 'p': 0.7071}; min human {'n': 30, 'median_diff': 0.005, 'p': 0.8394}
- **B1 vs B3-O** (n = 30): system success McNemar only-B1 6 / only-B3-O 5, p = 1.0; travel {'n': 26, 'median_diff': -0.485, 'p': 0.0}; hold {'n': 30, 'median_diff': 0.25, 'p': 0.2188}; min human {'n': 30, 'median_diff': -0.003, 'p': 0.5699}
- **B1 vs B2** (n = 30): system success McNemar only-B1 6 / only-B2 4, p = 0.7539; travel {'n': 26, 'median_diff': -0.465, 'p': 0.0004}; hold {'n': 30, 'median_diff': 0.905, 'p': 0.1765}; min human {'n': 30, 'median_diff': 0.008, 'p': 0.2621}

## Failure classes (per AMR)

- B1: HUMAN_MOTION_BLOCK 3, NAV2_ABORT 3, NORMAL_YIELD 3, PROTECTIVE_STOP 1, SUCCESS 79, UNKNOWN 1
- B2: HUMAN_MOTION_BLOCK 4, NAV2_ABORT 1, NORMAL_YIELD 23, SINGLE_HUMAN_COMFORT_BLOCK 1, SUCCESS 59, UNKNOWN 2
- B3-O: HUMAN_MOTION_BLOCK 4, NORMAL_YIELD 14, PROTECTIVE_STOP 2, ROBOT_ROBOT_BLOCK 1, SINGLE_HUMAN_COMFORT_BLOCK 1, SUCCESS 68
- B3-O LOWER: HUMAN_MOTION_BLOCK 2, NORMAL_YIELD 8, SUCCESS 80
- B3-O UPPER: HUMAN_MOTION_BLOCK 3, NORMAL_YIELD 26, SINGLE_HUMAN_COMFORT_BLOCK 2, SUCCESS 57, TEMPORARY_HOLD 2
