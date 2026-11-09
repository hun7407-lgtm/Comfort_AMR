# 261105_MAIN_pilot_policy_x_amrs_seeds1-8: policy x AMR count

95 AMR1 rows, 96 fleet vehicle rows, AMR counts [1, 2, 3]


## AMR1 outcome by policy and AMR count

| AMRs | policy | n | SUCCESS | DEADLOCK | other | deadlock causes |
|---|---|---|---|---|---|---|
| 1 | B1 none | 8 | 100% | 0% | 0% |  |
| 1 | B2 static | 8 | 38% | 62% | 0% | yielding to a Comfort Space x3; no progress for 12 s x2 |
| 1 | B3 state | 8 | 88% | 12% | 0% | no progress for 12 s while yielding x1 |
| 1 | B3-O directional | 8 | 100% | 0% | 0% |  |
| 2 | B1 none | 8 | 100% | 0% | 0% |  |
| 2 | B2 static | 7 | 43% | 57% | 0% | yielding to a Comfort Space x2; no progress for 12 s x2 |
| 2 | B3 state | 8 | 88% | 12% | 0% | no progress for 12 s while yielding x1 |
| 2 | B3-O directional | 8 | 88% | 12% | 0% | no progress for 12 s while yielding x1 |
| 3 | B1 none | 8 | 100% | 0% | 0% |  |
| 3 | B2 static | 8 | 38% | 50% | 12% | no progress for 12 s while yielding x1; no progress for 12 s x2; yielding to a Comfort Space x1 |
| 3 | B3 state | 8 | 88% | 12% | 0% | no progress for 12 s while yielding x1 |
| 3 | B3-O directional | 8 | 88% | 12% | 0% | no progress for 12 s while yielding x1 |

## Travel time (successes only) and its cost against B1 on the same seed

| AMRs | policy | successes | travel time s (mean) | paired seeds | time - B1 s (mean) | (median) | (max) |
|---|---|---|---|---|---|---|---|
| 1 | B1 none | 8 | 33.3 | 8 | +0.0 | +0.0 | +0.0 |
| 1 | B2 static | 3 | 33.8 | 3 | +1.0 | +0.3 | +2.6 |
| 1 | B3 state | 7 | 34.2 | 7 | +1.3 | +0.1 | +10.3 |
| 1 | B3-O directional | 8 | 36.2 | 8 | +2.9 | -0.0 | +15.4 |
| 2 | B1 none | 8 | 33.2 | 8 | +0.0 | +0.0 | +0.0 |
| 2 | B2 static | 3 | 34.1 | 3 | +1.4 | +0.1 | +4.5 |
| 2 | B3 state | 7 | 34.3 | 7 | +1.6 | +0.1 | +10.6 |
| 2 | B3-O directional | 7 | 34.1 | 7 | +1.3 | +0.1 | +9.1 |
| 3 | B1 none | 8 | 33.3 | 8 | +0.0 | +0.0 | +0.0 |
| 3 | B2 static | 3 | 33.3 | 3 | +0.3 | +0.3 | +0.9 |
| 3 | B3 state | 7 | 34.5 | 7 | +1.5 | +0.5 | +9.4 |
| 3 | B3-O directional | 7 | 34.2 | 7 | +1.3 | +0.1 | +9.0 |

## AMR1: closest approach to people, by whom (m, mean over trials of the per-trial minimum)

| AMRs | policy | n | belt workers (N/S) | storage-side people (E) | any person | contact: robot drove in |
|---|---|---|---|---|---|---|
| 1 | B1 none | 8 | 0.47 | 3.19 | 0.43 | 0% |
| 1 | B2 static | 8 | 0.89 | 4.26 | 0.86 | 0% |
| 1 | B3 state | 8 | 0.59 | 3.38 | 0.59 | 0% |
| 1 | B3-O directional | 8 | 0.59 | 3.19 | 0.59 | 0% |
| 2 | B1 none | 8 | 0.47 | 3.22 | 0.43 | 0% |
| 2 | B2 static | 7 | 1.01 | 4.87 | 1.01 | 0% |
| 2 | B3 state | 8 | 0.59 | 3.39 | 0.59 | 0% |
| 2 | B3-O directional | 8 | 0.60 | 3.39 | 0.60 | 0% |
| 3 | B1 none | 8 | 0.48 | 3.23 | 0.43 | 0% |
| 3 | B2 static | 8 | 0.93 | 4.31 | 0.89 | 0% |
| 3 | B3 state | 8 | 0.58 | 3.39 | 0.58 | 0% |
| 3 | B3-O directional | 8 | 0.58 | 3.38 | 0.58 | 0% |

## Fleet vehicles by policy and AMR count

| AMRs | policy | vehicle rows | SUCCESS | DEADLOCK | ABORTED | legs/min (mean) | min clearance m (mean) | robot drove in | nearest other robot m (min) |
|---|---|---|---|---|---|---|---|---|---|
| 2 | B1 none | 8 | 88% | 12% | 0% | 1.90 | 0.25 | 25% | 0.75 |
| 2 | B2 static | 8 | 75% | 25% | 0% | 1.31 | 0.56 | 0% | 5.97 |
| 2 | B3 state | 8 | 88% | 12% | 0% | 1.76 | 0.45 | 0% | 4.34 |
| 2 | B3-O directional | 8 | 88% | 12% | 0% | 1.78 | 0.45 | 0% | 4.52 |
| 3 | B1 none | 16 | 94% | 6% | 0% | 1.63 | 0.39 | 12% | 0.63 |
| 3 | B2 static | 16 | 69% | 31% | 0% | 0.81 | 0.68 | 0% | 0.55 |
| 3 | B3 state | 16 | 88% | 6% | 6% | 1.40 | 0.62 | 0% | 0.56 |
| 3 | B3-O directional | 16 | 94% | 6% | 0% | 1.54 | 0.57 | 0% | 0.56 |

## Geometry: Comfort Space reach into the aisle (from the configs)

aisle 1.50 m, robot 0.48 m wide, inflation 0.34 m, buffer band outside the Comfort Space 0.35 m; belt worker centre 0.455 m outside the aisle edge, storage-side stander 0.635 m outside the far edge.

| policy | belt worker: reach into aisle m | + buffer | storage-side stander: reach m | + buffer |
|---|---|---|---|---|
| b2 | 0.74 | 1.09 | 0.57 | 0.92 |
| b3 | 0.14 | 0.49 | 0.27 | 0.62 |
| b3d | 0.14 | 0.49 | 0.17 | 0.52 |
| b3d (before 2026-11-05) | 0.49 | 0.84 | 0.17 | 0.52 |
