# cross_h8: interaction KPIs

## By policy (means over seeds)

| policy | n | AMR1 success | a1 success | a2 success | AMR1 within 2 m of a1 s | ... stopped s | min AMR1-a1 gap m | AMR1 closest person m | a1 closest m | a2 closest m | AMR1 contact s (robot drove in) | a2 contact s (robot drove in) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B1 | 8 | 88% | 100% | 62% | 2.2 | 0.0 | 2.09 | 0.15 | 1.01 | -0.01 | 0.1 (0.0) | 1.4 (0.2) |
| B2 | 8 | 88% | 100% | 75% | 3.5 | 0.0 | 1.31 | 0.54 | 1.06 | -0.05 | 0.6 (0.1) | 1.2 (0.0) |
| B3 | 8 | 100% | 88% | 88% | 3.5 | 0.0 | 1.61 | 0.39 | 1.00 | -0.02 | 0.4 (0.0) | 1.1 (0.0) |
| B3-O | 8 | 100% | 100% | 100% | 3.2 | 0.0 | 1.40 | 0.40 | 1.00 | -0.03 | 0.1 (0.0) | 0.7 (0.0) |

## By condition: which north belt worker walks out to swap with E1

| north partner | seeds | policy | n | AMR1 success | deadlocks | time - B1 s (mean, successes) | yield s (mean) | AMR1 closest person m | a2 success |
|---|---|---|---|---|---|---|---|---|---|
| N1 | 1,7,8 | B1 | 3 | 100% | 0 | +0.0 | 0.0 | 0.25 | 100% |
| N1 | 1,7,8 | B2 | 3 | 100% | 0 | +1.1 | 0.0 | 1.10 | 100% |
| N1 | 1,7,8 | B3 | 3 | 100% | 0 | -0.3 | 0.0 | 0.75 | 67% |
| N1 | 1,7,8 | B3-O | 3 | 100% | 0 | -0.3 | 0.0 | 0.72 | 100% |
| N2 | 2,3,4,6 | B1 | 4 | 75% | 1 | +0.0 | 0.0 | 0.12 | 50% |
| N2 | 2,3,4,6 | B2 | 4 | 100% | 0 | +9.7 | 6.6 | 0.24 | 75% |
| N2 | 2,3,4,6 | B3 | 4 | 100% | 0 | +8.6 | 8.4 | 0.19 | 100% |
| N2 | 2,3,4,6 | B3-O | 4 | 100% | 0 | +7.4 | 7.7 | 0.23 | 100% |
| N3 | 5 | B1 | 1 | 100% | 0 | +0.0 | 0.0 | -0.01 | 0% |
| N3 | 5 | B2 | 1 | 0% | 1 | +nan | 22.9 | 0.08 | 0% |
| N3 | 5 | B3 | 1 | 100% | 0 | +10.0 | 10.1 | 0.12 | 100% |
| N3 | 5 | B3-O | 1 | 100% | 0 | +8.9 | 8.8 | 0.10 | 100% |

## Per seed

| seed | policy | AMR1 | travel s | yield s | AMR1 closest (who) | within 2 m of a1 s | stopped s | min gap m | a1 | a1 closest (who) | a2 | a2 closest (who) | a2 contact s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | B1 | SUCCESS | 41.2 | 0.0 | 0.44 (N2) | 4 | 0 | 1.55 | SUCCESS 1/1 | 1.09 (N3) | SUCCESS 1/1 | -0.03 (E2) | 1.6 |
| 1 | B2 | SUCCESS | 42.5 | 0.0 | 1.10 (E1) | 4 | 0 | 1.08 | SUCCESS 1/1 | 1.16 (N2) | SUCCESS 1/1 | -0.05 (E2) | 1.4 |
| 1 | B3 | SUCCESS | 41.3 | 0.0 | 0.75 (N2) | 4 | 0 | 1.41 | SUCCESS 1/1 | 1.12 (N2) | SUCCESS 1/1 | -0.04 (E2) | 0.6 |
| 1 | B3-O | SUCCESS | 41.3 | 0.0 | 0.72 (N2) | 4 | 0 | 1.37 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | -0.05 (E2) | 0.9 |
| 2 | B1 | DEADLOCK | 16.7 | 0.0 | -0.05 (N2) | 0 | 0 | 4.39 | SUCCESS 1/1 | 1.09 (N3) | DEADLOCK 0/1 | -0.11 (S3) | 2.4 |
| 2 | B2 | SUCCESS | 55.3 | 10.3 | -0.06 (N2) | 4 | 0 | 1.07 | SUCCESS 1/1 | 1.17 (N2) | SUCCESS 1/1 | -0.06 (E2) | 0.5 |
| 2 | B3 | SUCCESS | 52.2 | 11.3 | 0.13 (E1) | 4 | 0 | 1.36 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | 0.10 (E2) | 0.0 |
| 2 | B3-O | SUCCESS | 51.8 | 11.3 | -0.00 (E1) | 2 | 0 | 1.40 | SUCCESS 1/1 | 1.12 (N3) | SUCCESS 1/1 | 0.05 (E2) | 0.0 |
| 3 | B1 | SUCCESS | 41.7 | 0.0 | 0.21 (N2) | 2 | 0 | 1.78 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | 0.17 (E2) | 0.0 |
| 3 | B2 | SUCCESS | 55.0 | 7.8 | -0.07 (E1) | 4 | 0 | 1.05 | SUCCESS 1/1 | 1.15 (N2) | SUCCESS 1/1 | -0.04 (S1) | 0.8 |
| 3 | B3 | SUCCESS | 54.4 | 11.9 | -0.07 (E1) | 4 | 0 | 1.36 | SUCCESS 1/1 | 1.09 (N3) | SUCCESS 1/1 | -0.04 (S1) | 0.9 |
| 3 | B3-O | SUCCESS | 53.0 | 9.4 | -0.08 (E1) | 4 | 0 | 1.37 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | -0.08 (S1) | 1.0 |
| 4 | B1 | SUCCESS | 41.0 | 0.0 | 0.27 (N3) | 2 | 0 | 1.73 | SUCCESS 1/1 | 1.13 (N2) | DEADLOCK 0/1 | -0.08 (E2) | 3.0 |
| 4 | B2 | SUCCESS | 43.1 | 0.0 | 1.10 (E1) | 4 | 0 | 1.06 | SUCCESS 1/1 | 1.16 (N2) | DEADLOCK 0/1 | -0.06 (S1) | 3.2 |
| 4 | B3 | SUCCESS | 41.1 | 0.0 | 0.76 (N2) | 4 | 0 | 1.38 | SUCCESS 1/1 | 1.09 (N3) | SUCCESS 1/1 | -0.05 (S1) | 1.7 |
| 4 | B3-O | SUCCESS | 41.2 | 0.0 | 0.75 (N2) | 4 | 0 | 1.41 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | -0.05 (S1) | 1.3 |
| 5 | B1 | SUCCESS | 45.1 | 0.0 | -0.01 (N3) | 4 | 0 | 1.84 | SUCCESS 1/1 | 0.36 (N3) | DEADLOCK 0/1 | -0.07 (E2) | 2.3 |
| 5 | B2 | DEADLOCK | 40.0 | 22.9 | 0.08 (E1) | 0 | 0 | 2.99 | SUCCESS 1/1 | 0.40 (N3) | DEADLOCK 0/1 | -0.06 (S1) | 1.1 |
| 5 | B3 | SUCCESS | 55.1 | 10.1 | 0.12 (E1) | 4 | 0 | 1.39 | SUCCESS 1/1 | 0.29 (N3) | SUCCESS 1/1 | -0.07 (S1) | 1.6 |
| 5 | B3-O | SUCCESS | 54.0 | 8.8 | 0.10 (E1) | 2 | 0 | 1.53 | SUCCESS 1/1 | 0.29 (N3) | SUCCESS 1/1 | -0.04 (E2) | 1.3 |
| 6 | B1 | SUCCESS | 40.9 | 0.0 | 0.06 (N2) | 2 | 0 | 1.78 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | -0.09 (S1) | 1.6 |
| 6 | B2 | SUCCESS | 54.6 | 8.1 | -0.02 (E1) | 4 | 0 | 1.08 | SUCCESS 1/1 | 1.16 (N2) | SUCCESS 1/1 | 0.00 (S1) | 0.0 |
| 6 | B3 | SUCCESS | 53.9 | 10.5 | -0.07 (E1) | 0 | 0 | 3.23 | DEADLOCK 0/1 | 1.13 (N2) | SUCCESS 1/1 | 0.07 (S1) | 0.0 |
| 6 | B3-O | SUCCESS | 51.5 | 10.0 | 0.24 (E1) | 2 | 0 | 1.36 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | 0.08 (S1) | 0.0 |
| 7 | B1 | SUCCESS | 41.8 | 0.0 | 0.14 (N3) | 2 | 0 | 1.83 | SUCCESS 1/1 | 1.11 (N3) | SUCCESS 1/1 | -0.07 (E2) | 0.4 |
| 7 | B2 | SUCCESS | 42.6 | 0.0 | 1.10 (E1) | 4 | 0 | 1.06 | SUCCESS 1/1 | 1.17 (N2) | SUCCESS 1/1 | -0.07 (S1) | 2.1 |
| 7 | B3 | SUCCESS | 41.3 | 0.0 | 0.75 (N2) | 4 | 0 | 1.41 | SUCCESS 1/1 | 1.09 (N3) | DEADLOCK 0/1 | -0.10 (E2) | 3.0 |
| 7 | B3-O | SUCCESS | 41.4 | 0.0 | 0.73 (N3) | 4 | 0 | 1.40 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | -0.08 (E2) | 0.5 |
| 8 | B1 | SUCCESS | 41.8 | 0.0 | 0.18 (N3) | 2 | 0 | 1.83 | SUCCESS 1/1 | 1.12 (N3) | SUCCESS 1/1 | 0.24 (E2) | 0.0 |
| 8 | B2 | SUCCESS | 43.0 | 0.0 | 1.10 (E1) | 4 | 0 | 1.09 | SUCCESS 1/1 | 1.14 (N2) | SUCCESS 1/1 | -0.06 (E2) | 0.8 |
| 8 | B3 | SUCCESS | 41.2 | 0.0 | 0.75 (N2) | 4 | 0 | 1.36 | SUCCESS 1/1 | 1.12 (N3) | SUCCESS 1/1 | -0.05 (E2) | 0.8 |
| 8 | B3-O | SUCCESS | 41.1 | 0.0 | 0.72 (N2) | 4 | 0 | 1.38 | SUCCESS 1/1 | 1.10 (N3) | SUCCESS 1/1 | -0.05 (S2) | 0.7 |
