# 261107_MAIN_cross_seeds1-33_merged: how each failed AMR1 trial stopped

| policy | n | gate hold | post-evade stall | nav2 abort | stall | lost |
|---|---|---|---|---|---|---|
| B1 | 33 | 0 | 0 | 2 | 0 | 0 |
| B2 | 33 | 5 | 3 | 0 | 0 | 0 |
| B3 | 33 | 0 | 4 | 1 | 0 | 1 |
| B3-O | 33 | 1 | 4 | 1 | 0 | 0 |

| seed | north | policy | result | mechanism | where | final pos | travel s | yield s | evade s | closest (who) | person walked in s | runner's cause |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 | N2 | B1 | DEADLOCK | nav2 abort | belt edge | (0.62, 3.52) | 16.7 | 0.0 | 0.0 | -0.05 (N2) | 0.8 |  |
| 5 | N3 | B2 | DEADLOCK | gate hold | aisle | (4.55, 4.28) | 40.0 | 22.9 | 8.4 | 0.08 (E1) | 0.0 | no progress for 12 s while yielding |
| 10 | N2 | B2 | DEADLOCK | gate hold | aisle | (-1.39, 3.92) | 35.4 | 25.0 | 9.5 | -0.06 (N2) | 0.0 | yielding to a Comfort Space |
| 11 | N3 | B2 | DEADLOCK | post-evade stall | refuge | (1.74, 5.25) | 38.8 | 9.1 | 9.1 | -0.07 (E1) | 0.4 | no progress for 12 s while yielding |
| 11 | N3 | B3 | DEADLOCK | nav2 abort | belt edge | (0.82, 2.80) | 32.4 | 15.2 | 15.2 | -0.09 (N3) | 2.4 |  |
| 12 | N3 | B1 | DEADLOCK | nav2 abort | belt edge | (1.75, 3.41) | 33.5 | 0.0 | 0.0 | -0.01 (N3) | 0.2 |  |
| 12 | N3 | B3 | DEADLOCK | post-evade stall | refuge | (1.75, 5.25) | 40.0 | 12.1 | 12.0 | 0.07 (E1) | 0.0 | no progress for 12 s while yielding |
| 12 | N3 | B3-O | DEADLOCK | post-evade stall | refuge | (1.80, 5.30) | 40.0 | 12.3 | 12.2 | 0.06 (E1) | 0.0 | no progress for 12 s while yielding |
| 13 | N3 | B2 | DEADLOCK | post-evade stall | refuge | (1.74, 5.26) | 43.8 | 7.9 | 7.8 | 0.21 (E1) | 0.0 | no progress for 12 s while yielding |
| 13 | N3 | B3-O | DEADLOCK | nav2 abort | refuge | (1.85, 5.14) | 38.0 | 7.8 | 7.6 | 0.02 (E1) | 0.0 |  |
| 14 | N2 | B3-O | DEADLOCK | post-evade stall | belt edge | (1.05, 3.48) | 42.2 | 15.4 | 13.4 | -0.07 (E1) | 3.1 | no progress for 12 s while yielding |
| 15 | N1 | B3 | DEADLOCK | post-evade stall | refuge | (1.77, 5.40) | 36.4 | 6.9 | 6.9 | 0.72 (N2) | 0.0 | no progress for 12 s while yielding |
| 15 | N1 | B3-O | DEADLOCK | post-evade stall | refuge | (1.77, 5.30) | 36.4 | 7.3 | 7.3 | 0.72 (N2) | 0.0 | no progress for 12 s while yielding |
| 17 | N3 | B2 | DEADLOCK | gate hold | aisle | (5.50, 4.25) | 41.0 | 23.9 | 14.4 | -0.08 (E1) | 1.1 | yielding to a Comfort Space |
| 17 | N3 | B3-O | DEADLOCK | post-evade stall | aisle | (6.20, 3.72) | 44.6 | 16.4 | 16.4 | -0.07 (N3) | 0.8 | no progress for 12 s while yielding |
| 19 | N2 | B3 | DEADLOCK | post-evade stall | belt edge | (0.77, 3.53) | 41.6 | 18.3 | 13.9 | -0.08 (E1) | 3.0 | no progress for 12 s while yielding |
| 23 | N2 | B3 | TIMEOUT | lost | aisle | (-1.01, 4.48) | 0.0 | 0.0 | 7.8 | -0.07 (E1) | 1.2 |  |
| 25 | N3 | B2 | DEADLOCK | post-evade stall | aisle | (4.52, 4.41) | 46.8 | 17.3 | 17.3 | 0.36 (E1) | 0.0 | no progress for 12 s, a person came inside the protective field (Prote |
| 25 | N3 | B3-O | DEADLOCK | gate hold | aisle | (4.73, 3.69) | 40.6 | 22.7 | 14.3 | -0.06 (N3) | 0.6 | no progress for 12 s while yielding |
| 28 | N3 | B2 | DEADLOCK | gate hold | aisle | (4.52, 3.84) | 37.8 | 22.7 | 11.5 | -0.06 (E1) | 0.8 | yielding to a Comfort Space |
| 32 | N1 | B3 | DEADLOCK | post-evade stall | crossing | (9.44, -2.18) | 41.8 | 6.6 | 6.6 | 0.73 (N2) | 0.0 | no progress for 12 s while yielding |
| 33 | N3 | B2 | DEADLOCK | gate hold | aisle | (4.38, 4.21) | 37.4 | 21.0 | 8.2 | 0.16 (E1) | 0.0 | no progress for 12 s while yielding |
