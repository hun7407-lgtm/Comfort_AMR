# campaign_261105: policy x AMR count

78 AMR1 rows, 177 fleet vehicle rows, AMR counts [1, 4]

3 AMR1 row(s) relabelled DEADLOCK -> SUCCESS (Nav2 SUCCEEDED, no deadlock detected, base within 0.05 m of the 0.25 m tolerance; GOAL_POSE_SLACK_M in scripts/nav2_goal.py): paper_h8_a1_scr_s0005 b3 (0.253 m), paper_h8_a1_scr_s0005 b3d (0.250 m), campaign_h8_s5_loop_rule b3d (0.264 m)


## AMR1 outcome by policy and AMR count

| AMRs | policy | n | SUCCESS | DEADLOCK | other | deadlock causes |
|---|---|---|---|---|---|---|
| 1 | B1 none | 5 | 100% | 0% | 0% |  |
| 1 | B2 static | 5 | 40% | 60% | 0% | yielding to a Comfort Space x1; no progress for 12 s while yielding x2 |
| 1 | B3 state | 5 | 100% | 0% | 0% |  |
| 1 | B3-O directional | 5 | 80% | 20% | -0% | no progress for 12 s while yielding x1 |
| 4 | B1 none | 4 | 100% | 0% | 0% |  |
| 4 | B2 static | 4 | 25% | 75% | 0% | no progress for 12 s x2; yielding to a Comfort Space x1 |
| 4 | B3 state | 4 | 100% | 0% | 0% |  |
| 4 | B3-O directional | 4 | 50% | 50% | 0% | no progress for 12 s while yielding x1; no progress for 12 s x1 |

## Travel time (successes only) and its cost against B1 on the same seed

| AMRs | policy | successes | travel time s (mean) | paired seeds | time - B1 s (mean) | (median) | (max) |
|---|---|---|---|---|---|---|---|
| 1 | B1 none | 5 | 33.5 | 5 | +0.0 | +0.0 | +0.0 |
| 1 | B2 static | 2 | 36.8 | 2 | +1.7 | +1.7 | +2.9 |
| 1 | B3 state | 5 | 35.4 | 5 | +1.9 | +0.2 | +9.4 |
| 1 | B3-O directional | 4 | 36.8 | 4 | +2.6 | +0.6 | +9.5 |
| 4 | B1 none | 4 | 31.5 | 4 | +0.0 | +0.0 | +0.0 |
| 4 | B2 static | 1 | 27.9 | 1 | +0.3 | +0.3 | +0.3 |
| 4 | B3 state | 4 | 34.5 | 4 | +2.9 | +0.7 | +9.9 |
| 4 | B3-O directional | 2 | 30.9 | 2 | +4.9 | +4.9 | +9.3 |

## AMR1: closest approach to people, by whom (m, mean over trials of the per-trial minimum)

| AMRs | policy | n | belt workers (N/S) | storage-side people (E) | any person | contact: robot drove in |
|---|---|---|---|---|---|---|
| 1 | B1 none | 5 | 0.40 | 2.56 | 0.33 | 0% |
| 1 | B2 static | 5 | 0.89 | 3.54 | 0.89 | 0% |
| 1 | B3 state | 5 | 0.54 | 2.49 | 0.54 | 0% |
| 1 | B3-O directional | 5 | 0.66 | 2.46 | 0.66 | 0% |
| 4 | B1 none | 4 | 0.41 | 2.57 | 0.32 | 0% |
| 4 | B2 static | 4 | 0.98 | 4.24 | 0.98 | 0% |
| 4 | B3 state | 4 | 0.49 | 2.50 | 0.49 | 0% |
| 4 | B3-O directional | 4 | 0.57 | 2.47 | 0.57 | 0% |

## Fleet vehicles by policy and AMR count

| AMRs | policy | vehicle rows | SUCCESS | DEADLOCK | ABORTED | legs/min (mean) | min clearance m (mean) | robot drove in | nearest other robot m (min) |
|---|---|---|---|---|---|---|---|---|---|
| 4 | B1 none | 12 | 100% | 0% | 0% | 1.34 | 0.42 | 8% | 0.76 |
| 4 | B2 static | 12 | 50% | 50% | 0% | 0.67 | 0.75 | 0% | 0.62 |
| 4 | B3 state | 12 | 83% | 17% | 0% | 1.07 | 0.60 | 0% | 0.65 |
| 4 | B3-O directional | 12 | 92% | 8% | 0% | 1.20 | 0.64 | 0% | 0.49 |

## Geometry: Comfort Space reach into the aisle (from the configs)

aisle 1.50 m, robot 0.48 m wide, inflation 0.34 m, buffer band outside the Comfort Space 0.35 m; belt worker centre 0.455 m outside the aisle edge, storage-side stander 0.635 m outside the far edge.

| policy | belt worker: reach into aisle m | + buffer | storage-side stander: reach m | + buffer |
|---|---|---|---|---|
| b2 | 0.74 | 1.09 | 0.57 | 0.92 |
| b3 | 0.14 | 0.49 | 0.27 | 0.62 |
| b3d | 0.14 | 0.49 | 0.17 | 0.52 |
| b3d (before 2026-11-05) | 0.49 | 0.84 | 0.17 | 0.52 |
