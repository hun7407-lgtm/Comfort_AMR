# 261105_cross_seeds1-8: policy x AMR count

32 AMR1 rows, 64 fleet vehicle rows, AMR counts [3]


## AMR1 outcome by policy and AMR count

| AMRs | policy | n | SUCCESS | DEADLOCK | other | deadlock causes |
|---|---|---|---|---|---|---|
| 3 | B1 none | 8 | 88% | 12% | 0% | (no cause recorded) x1 |
| 3 | B2 static | 8 | 88% | 12% | 0% | no progress for 12 s while yielding x1 |
| 3 | B3 state | 8 | 100% | 0% | 0% |  |
| 3 | B3-O directional | 8 | 100% | 0% | 0% |  |

## Travel time (successes only) and its cost against B1 on the same seed

| AMRs | policy | successes | travel time s (mean) | paired seeds | time - B1 s (mean) | (median) | (max) |
|---|---|---|---|---|---|---|---|
| 3 | B1 none | 7 | 41.9 | 7 | +0.0 | +0.0 | +0.0 |
| 3 | B2 static | 7 | 48.0 | 6 | +5.4 | +1.7 | +13.7 |
| 3 | B3 state | 8 | 47.6 | 7 | +5.0 | +0.2 | +13.0 |
| 3 | B3-O directional | 8 | 46.9 | 7 | +4.3 | +0.2 | +11.4 |

## AMR1: closest approach to people, by whom (m, mean over trials of the per-trial minimum)

| AMRs | policy | n | belt workers (N/S) | storage-side people (E) | any person | contact: robot drove in |
|---|---|---|---|---|---|---|
| 3 | B1 none | 8 | 0.15 | 0.96 | 0.15 | 0% |
| 3 | B2 static | 8 | 0.70 | 0.55 | 0.54 | 38% |
| 3 | B3 state | 8 | 0.54 | 0.56 | 0.39 | 12% |
| 3 | B3-O directional | 8 | 0.51 | 0.58 | 0.40 | 0% |

## Fleet vehicles by policy and AMR count

| AMRs | policy | vehicle rows | SUCCESS | DEADLOCK | ABORTED | legs/min (mean) | min clearance m (mean) | robot drove in | nearest other robot m (min) |
|---|---|---|---|---|---|---|---|---|---|
| 3 | B1 none | 16 | 81% | 19% | 0% | 0.57 | 0.50 | 12% | 1.55 |
| 3 | B2 static | 16 | 88% | 12% | 0% | 0.59 | 0.51 | 0% | 1.05 |
| 3 | B3 state | 16 | 88% | 12% | 0% | 0.60 | 0.49 | 0% | 1.36 |
| 3 | B3-O directional | 16 | 100% | 0% | 0% | 0.74 | 0.49 | 0% | 1.36 |

## Geometry: Comfort Space reach into the aisle (from the configs)

aisle 1.50 m, robot 0.48 m wide, inflation 0.34 m, buffer band outside the Comfort Space 0.35 m; belt worker centre 0.455 m outside the aisle edge, storage-side stander 0.635 m outside the far edge.

| policy | belt worker: reach into aisle m | + buffer | storage-side stander: reach m | + buffer |
|---|---|---|---|---|
| b2 | 0.74 | 1.09 | 0.57 | 0.92 |
| b3 | 0.14 | 0.49 | 0.27 | 0.62 |
| b3d | 0.14 | 0.49 | 0.17 | 0.52 |
| b3d (before 2026-11-05) | 0.49 | 0.84 | 0.17 | 0.52 |
