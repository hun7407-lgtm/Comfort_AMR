# 261112_stage1_smoke_s5: Stage 1 tables

6 AMR1 rows (0 not size conditions, ignored); sizes 0.80, 1.20, 1.60 m.

Travel time/distance: mean ± sd over SUCCESSFUL trials. Clearance, intrusion, contact, waits: over all valid trials. 'comfort DL' = Comfort gate hold only. 'robot in' = trials where the robot closed the last centimetre of a contact. 'r-r gap' = AMR1's closest centre-to-centre approach to a1/a2 (min over trials). Intrusion is measured against the fixed reference geometry (evaluation.comfort_reference = b3), identical for every size.

## Static — AMR1 by size

| size (m) | n | valid | success | comfort DL | total DL | travel s | dist m | plan m | yield s | wait s | evade s | min clr m | clr min | contact | robot in | intr m | intr n | in CS s | r-r gap m | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 1 | 1 | 0/1 (0%) | 0 (0%) | 1 (100%) | -±- | -±- | - | 14.6 | 20.6 | 11.4 | -0.06 | -0.06 | 1 | 0 | 1.91 | 1 | 6.1 | 4.25 | 0.5 |
| 1.20 | 1 | 1 | 0/1 (0%) | 1 (100%) | 1 (100%) | -±- | -±- | - | 22.4 | 20.6 | 8.4 | 0.09 | 0.09 | 0 | 0 | 1.18 | 1 | 3.6 | 3.06 | 0.5 |
| 1.60 | 1 | 1 | 0/1 (0%) | 1 (100%) | 1 (100%) | -±- | -±- | - | 37.1 | 20.6 | 7.2 | 0.34 | 0.34 | 0 | 0 | 0.42 | 1 | 21.8 | 4.47 | 0.0 |

### Static — failure mechanism by size

| size (m) | Comfort gate hold | Post-evade stall | Nav2 abort | Stall (other) | Planning failure | Timeout | Invalid (physics / measurement) |
|---|---|---|---|---|---|---|---|
| 0.80 | 0 | 1 | 0 | 0 | 0 | 0 | 0 |
| 1.20 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1.60 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |

- Comfort gate hold seeds — 1.20: s5; 1.60: s5
- Post-evade stall seeds — 0.80: s5

### Static — a1 by size

| size (m) | n | success | deadlock | aborted | legs done | dist m | min clr m | clr min | contact | intr m | gap AMR1 m | gap fleet m | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 1 | 100% | 0 | 0 | 100% | 8.22 | 0.36 | 0.36 | 0 | 0.83 | 4.25 | 9.77 | 0.0 |
| 1.20 | 1 | 100% | 0 | 0 | 100% | 8.31 | 0.38 | 0.38 | 0 | 0.82 | 3.06 | 9.81 | 0.0 |
| 1.60 | 1 | 100% | 0 | 0 | 100% | 8.15 | 0.45 | 0.45 | 0 | 0.83 | 4.47 | 9.74 | 0.1 |

### Static — a2 by size

| size (m) | n | success | deadlock | aborted | legs done | dist m | min clr m | clr min | contact | intr m | gap AMR1 m | gap fleet m | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 1 | 100% | 0 | 0 | 100% | 9.80 | -0.07 | -0.07 | 1 | 1.76 | 9.13 | 9.77 | 0.0 |
| 1.20 | 1 | 0% | 1 | 0 | 0% | 6.98 | -0.03 | -0.03 | 1 | 1.93 | 9.50 | 9.81 | 0.0 |
| 1.60 | 1 | 0% | 1 | 0 | 0% | 4.82 | -0.03 | -0.03 | 1 | 1.80 | 9.26 | 9.74 | 0.0 |

## B3-O — AMR1 by size

| size (m) | n | valid | success | comfort DL | total DL | travel s | dist m | plan m | yield s | wait s | evade s | min clr m | clr min | contact | robot in | intr m | intr n | in CS s | r-r gap m | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 1 | 1 | 1/1 (100%) | 0 (0%) | 0 (0%) | 50.8±- | 30.46±- | 25.47 | 7.1 | 20.6 | 7.1 | 0.11 | 0.11 | 0 | 0 | 1.12 | 1 | 3.6 | 1.39 | 0.0 |
| 1.20 | 1 | 1 | 1/1 (100%) | 0 (0%) | 0 (0%) | 53.4±- | 30.30±- | 25.75 | 8.8 | 20.6 | 8.2 | 0.13 | 0.13 | 0 | 0 | 1.09 | 1 | 3.5 | 1.47 | 0.0 |
| 1.60 | 1 | 1 | 1/1 (100%) | 0 (0%) | 0 (0%) | 54.0±- | 30.17±- | 26.42 | 9.3 | 20.6 | 8.6 | 0.15 | 0.15 | 0 | 0 | 1.07 | 1 | 3.5 | 1.26 | 0.0 |

### B3-O — failure mechanism by size

| size (m) | Comfort gate hold | Post-evade stall | Nav2 abort | Stall (other) | Planning failure | Timeout | Invalid (physics / measurement) |
|---|---|---|---|---|---|---|---|
| 0.80 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1.20 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1.60 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |


### B3-O — a1 by size

| size (m) | n | success | deadlock | aborted | legs done | dist m | min clr m | clr min | contact | intr m | gap AMR1 m | gap fleet m | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 1 | 100% | 0 | 0 | 100% | 8.28 | 0.38 | 0.38 | 0 | 0.81 | 1.39 | 9.80 | 0.0 |
| 1.20 | 1 | 100% | 0 | 0 | 100% | 8.25 | 0.39 | 0.39 | 0 | 0.84 | 1.47 | 9.89 | 0.0 |
| 1.60 | 2 | 100% | 0 | 0 | 100% | 8.29 | 0.39 | 0.39 | 0 | 0.78 | 1.26 | 9.87 | 0.0 |

### B3-O — a2 by size

| size (m) | n | success | deadlock | aborted | legs done | dist m | min clr m | clr min | contact | intr m | gap AMR1 m | gap fleet m | stop s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.80 | 1 | 0% | 1 | 0 | 0% | 9.43 | -0.07 | -0.07 | 1 | 1.95 | 9.41 | 9.80 | 0.3 |
| 1.20 | 1 | 100% | 0 | 0 | 100% | 9.38 | -0.06 | -0.06 | 1 | 1.72 | 9.55 | 9.89 | 0.0 |
| 1.60 | 2 | 100% | 0 | 0 | 50% | 7.69 | -0.06 | -0.07 | 2 | 1.62 | 9.55 | 9.87 | 0.0 |

## Applied Comfort Space extents (m)

| condition | size | scale | eng F/S/R | stat F/S/R | mov F/S/R @1.0 m/s |
|---|---|---|---|---|---|
| static_r0.80 | 0.80 | 0.6667 | 0.80/0.80/0.80 | 0.80/0.80/0.80 | 0.80/0.80/0.80 |
| b3o_r0.80 | 0.80 | 0.6667 | 0.53/0.47/0.40 | 0.53/0.60/0.70 | 0.80/0.60/0.60 |
| static_r1.20 | 1.20 | 1.0000 | 1.20/1.20/1.20 | 1.20/1.20/1.20 | 1.20/1.20/1.20 |
| b3o_r1.20 | 1.20 | 1.0000 | 0.80/0.70/0.60 | 0.80/0.90/1.05 | 1.20/0.90/0.90 |
| static_r1.60 | 1.60 | 1.3333 | 1.60/1.60/1.60 | 1.60/1.60/1.60 | 1.60/1.60/1.60 |
| b3o_r1.60 | 1.60 | 1.3333 | 1.07/0.93/0.80 | 1.07/1.20/1.40 | 1.60/1.20/1.20 |
