# 261105_cross_design_check4: interaction KPIs

## By policy (means over seeds)

| policy | n | AMR1 success | a1 success | a2 success | AMR1 within 2 m of a1 s | ... stopped s | min AMR1-a1 gap m | AMR1 closest person m | a1 closest m | a2 closest m | AMR1 contact s (robot drove in) | a2 contact s (robot drove in) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B1 | 0 | nan% | nan% | nan% | nan | nan | nan | nan | nan | nan | nan (nan) | nan (nan) |
| B2 | 1 | 100% | 100% | 100% | 4.0 | 0.0 | 1.06 | -0.06 | 1.15 | -0.06 | 1.4 (0.0) | 0.9 (0.0) |
| B3 | 1 | 100% | 100% | 100% | 4.0 | 0.0 | 1.37 | -0.08 | 1.09 | -0.07 | 1.4 (0.0) | 1.0 (0.0) |
| B3-O | 0 | nan% | nan% | nan% | nan | nan | nan | nan | nan | nan | nan (nan) | nan (nan) |

## By condition: which north belt worker walks out to swap with E1

| north partner | seeds | policy | n | AMR1 success | deadlocks | time - B1 s (mean, successes) | yield s (mean) | AMR1 closest person m | a2 success |
|---|---|---|---|---|---|---|---|---|---|
| N2 | 2 | B1 | 0 | nan% | 0 | +nan | nan | nan | nan% |
| N2 | 2 | B2 | 1 | 100% | 0 | +nan | 9.3 | -0.06 | 100% |
| N2 | 2 | B3 | 1 | 100% | 0 | +nan | 9.7 | -0.08 | 100% |
| N2 | 2 | B3-O | 0 | nan% | 0 | +nan | nan | nan | nan% |

## Per seed

| seed | policy | AMR1 | travel s | yield s | AMR1 closest (who) | within 2 m of a1 s | stopped s | min gap m | a1 | a1 closest (who) | a2 | a2 closest (who) | a2 contact s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 | B1 | missing | | | | | | | | | | | |
| 2 | B2 | SUCCESS | 54.5 | 9.3 | -0.06 (E1) | 4 | 0 | 1.06 | SUCCESS 1/1 | 1.15 (N2) | SUCCESS 1/1 | -0.06 (E2) | 0.9 |
| 2 | B3 | SUCCESS | 54.2 | 9.7 | -0.08 (E1) | 4 | 0 | 1.37 | SUCCESS 1/1 | 1.09 (N3) | SUCCESS 1/1 | -0.07 (S3) | 1.0 |
| 2 | B3-O | missing | | | | | | | | | | | |
