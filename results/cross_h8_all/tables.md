# cross_h8_all: policy x AMR count

132 AMR1 rows, 266 fleet vehicle rows, AMR counts [3]


## AMR1 outcome by policy and AMR count

| AMRs | policy | n | SUCCESS | DEADLOCK | other | deadlock causes |
|---|---|---|---|---|---|---|
| 3 | B1 none | 33 | 94% | 6% | -0% | (no cause recorded) x2 |
| 3 | B2 static | 33 | 76% | 24% | 0% | no progress for 12 s while yielding x4; yielding to a Comfort Space x3; no progress for 12 s x1 |
| 3 | B3 state | 33 | 82% | 15% | 3% | (no cause recorded) x1; no progress for 12 s while yielding x4 |
| 3 | B3-O directional | 33 | 82% | 18% | -0% | no progress for 12 s while yielding x5; (no cause recorded) x1 |

## Travel time (successes only) and its cost against B1 on the same seed

| AMRs | policy | successes | travel time s (mean) | paired seeds | time - B1 s (mean) | (median) | (max) |
|---|---|---|---|---|---|---|---|
| 3 | B1 none | 31 | 44.2 | 31 | +0.0 | +0.0 | +0.0 |
| 3 | B2 static | 25 | 49.9 | 23 | +7.4 | +6.6 | +22.8 |
| 3 | B3 state | 27 | 49.0 | 26 | +5.3 | +3.4 | +15.4 |
| 3 | B3-O directional | 27 | 47.9 | 26 | +4.0 | +0.2 | +15.0 |

## AMR1: closest approach to people, by whom (m, mean over trials of the per-trial minimum)

| AMRs | policy | n | belt workers (N/S) | storage-side people (E) | any person | contact: robot drove in |
|---|---|---|---|---|---|---|
| 3 | B1 none | 33 | 0.12 | 0.93 | 0.12 | 0% |
| 3 | B2 static | 33 | 0.69 | 0.41 | 0.40 | 21% |
| 3 | B3 state | 33 | 0.42 | 0.48 | 0.29 | 6% |
| 3 | B3-O directional | 33 | 0.40 | 0.47 | 0.29 | 0% |

## Fleet vehicles by policy and AMR count

| AMRs | policy | vehicle rows | SUCCESS | DEADLOCK | ABORTED | legs/min (mean) | min clearance m (mean) | robot drove in | nearest other robot m (min) |
|---|---|---|---|---|---|---|---|---|---|
| 3 | B1 none | 66 | 88% | 12% | 0% | 0.63 | 0.51 | 8% | 1.33 |
| 3 | B2 static | 68 | 90% | 9% | 1% | 0.61 | 0.60 | 0% | 0.38 |
| 3 | B3 state | 66 | 89% | 9% | 2% | 0.62 | 0.54 | 0% | 1.08 |
| 3 | B3-O directional | 66 | 94% | 6% | 0% | 0.68 | 0.54 | 2% | 0.64 |

## Geometry: Comfort Space reach into the aisle (from the configs)

aisle 1.50 m, robot 0.48 m wide, inflation 0.34 m, buffer band outside the Comfort Space 0.35 m; belt worker centre 0.455 m outside the aisle edge, storage-side stander 0.635 m outside the far edge.

| policy | belt worker: reach into aisle m | + buffer | storage-side stander: reach m | + buffer |
|---|---|---|---|---|
| b2 | 0.74 | 1.09 | 0.57 | 0.92 |
| b3 | 0.14 | 0.49 | 0.27 | 0.62 |
| b3d | 0.14 | 0.49 | 0.17 | 0.52 |
| b3d (before 2026-11-05) | 0.49 | 0.84 | 0.17 | 0.52 |
