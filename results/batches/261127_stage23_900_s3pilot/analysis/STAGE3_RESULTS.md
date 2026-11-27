# STAGE 3 RESULTS — stage23_900 s3pilot

> 8 people / 3 Nav2 AMRs, five policies on the same seeds. Predicted comfort = literature curve at the ACTUAL closest distance / bearing / state per encounter (not a subjective rating). One global alpha across states with state-specific template ratios -- this does NOT test separately optimised state-wise alphas.

Launches 15; valid 15; infra-excluded 0. Encounter = (AMR, person) with min d_cc < 1.5 m during that AMR's mission.

| case | policy | n | nav | safe nav | joint@5.0 (95 % CI) | joint@4.5 | joint@5.5 | contact | AMR completion | comfort DL | robot block | Nav2 abort | PS trials | med time succ s | med time capped s | med path m | med replans | med idle s | coverage | med C | P10 C |
|---|---|---:|---|---|---|---|---|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| C1 | B2_r0.950 | 3 | 3/3 (0.44–1.00) | 2/3 (0.21–0.94) | 2/3 (0.21–0.94) | 2/3 | 2/3 | 1/3 (0.06–0.79) | 9/9 | 0 | 0 | 0 | 0 | 41.33 | 41.33 | 43.435 | 0.0 | 0.62 | 1.0 | 5.574 | 5.5174 |
| C2 | S3_a0.60 | 3 | 3/3 (0.44–1.00) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) | 1/3 | 0/3 | 2/3 (0.21–0.94) | 9/9 | 0 | 0 | 0 | 1 | 41.15 | 41.15 | 42.968 | 0.0 | 0.64 | 1.0 | 5.243 | 4.0186 |
| C3 | S3_a0.70 | 3 | 3/3 (0.44–1.00) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) | 1/3 | 0/3 | 2/3 (0.21–0.94) | 9/9 | 0 | 0 | 0 | 1 | 41.33 | 41.33 | 43.015 | 0.0 | 0.62 | 1.0 | 5.264 | 4.041 |
| C4 | S3_a0.80 | 3 | 3/3 (0.44–1.00) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) | 1/3 | 0/3 | 2/3 (0.21–0.94) | 9/9 | 0 | 0 | 0 | 1 | 41.22 | 41.22 | 43.058 | 0.0 | 0.62 | 1.0 | 5.3675 | 4.035 |
| C5 | S3_a0.90 | 3 | 3/3 (0.44–1.00) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) | 1/3 | 0/3 | 2/3 (0.21–0.94) | 9/9 | 0 | 0 | 0 | 1 | 41.15 | 41.15 | 43.205 | 0.0 | 0.64 | 1.0 | 5.4375 | 4.3058 |

Primary categories @5.0:

| policy | JOINT_PASS | FAIL_NAV | FAIL_CONTACT | NOT_SCORABLE | FAIL_COMFORT |
|---|---:|---:|---:|---:|---:|
| B2_r0.950 | 2 | 0 | 1 | 0 | 0 |
| S3_a0.60 | 1 | 0 | 2 | 0 | 0 |
| S3_a0.70 | 1 | 0 | 2 | 0 | 0 |
| S3_a0.80 | 1 | 0 | 2 | 0 | 0 |
| S3_a0.90 | 1 | 0 | 2 | 0 | 0 |

Per state (scorable encounters):

| policy | state | n | med d_cc | med C | P10 C | C >= 5.0 |
|---|---|---:|---:|---:|---:|---|
| B2_r0.950 | stationary | 2 | 1.3077 | 6.5505 | 6.5485 | 2/2 (0.34–1.00) |
| B2_r0.950 | station_engaged | 8 | 1.3485 | 5.5735 | 5.5651 | 8/8 (0.68–1.00) |
| B2_r0.950 | moving | 3 | 0.7798 | 5.506 | 4.3932 | 2/3 (0.21–0.94) |
| S3_a0.60 | stationary | 2 | 0.8985 | 5.8865 | 5.7197 | 2/2 (0.34–1.00) |
| S3_a0.60 | station_engaged | 5 | 0.8731 | 5.2 | 5.057 | 5/5 (0.57–1.00) |
| S3_a0.60 | moving | 3 | 0.5473 | 4.057 | 3.7498 | 1/3 (0.06–0.79) |
| S3_a0.70 | stationary | 2 | 1.0203 | 6.226 | 6.214 | 2/2 (0.34–1.00) |
| S3_a0.70 | station_engaged | 5 | 0.8827 | 5.221 | 5.1924 | 5/5 (0.57–1.00) |
| S3_a0.70 | moving | 4 | 0.8452 | 5.276 | 3.908 | 2/4 (0.15–0.85) |
| S3_a0.80 | stationary | 2 | 0.9667 | 6.108 | 6.0648 | 2/2 (0.34–1.00) |
| S3_a0.80 | station_engaged | 5 | 0.967 | 5.366 | 5.3262 | 5/5 (0.57–1.00) |
| S3_a0.80 | moving | 3 | 0.5483 | 4.065 | 3.825 | 1/3 (0.06–0.79) |
| S3_a0.90 | stationary | 2 | 1.0441 | 6.2765 | 6.2729 | 2/2 (0.34–1.00) |
| S3_a0.90 | station_engaged | 5 | 1.024 | 5.436 | 5.4124 | 5/5 (0.57–1.00) |
| S3_a0.90 | moving | 3 | 0.5936 | 4.392 | 3.7024 | 1/3 (0.06–0.79) |

Joint @5.0 per family:

| policy | NOMINAL | BOTTLENECK | CROSSING |
|---|---|---|---|
| B2_r0.950 | 0/1 (0.00–0.79) | 1/1 (0.21–1.00) | 1/1 (0.21–1.00) |
| S3_a0.60 | 0/1 (0.00–0.79) | 1/1 (0.21–1.00) | 0/1 (0.00–0.79) |
| S3_a0.70 | 0/1 (0.00–0.79) | 1/1 (0.21–1.00) | 0/1 (0.00–0.79) |
| S3_a0.80 | 0/1 (0.00–0.79) | 1/1 (0.21–1.00) | 0/1 (0.00–0.79) |
| S3_a0.90 | 0/1 (0.00–0.79) | 1/1 (0.21–1.00) | 0/1 (0.00–0.79) |

## Selection (rule fixed before the data)

```
Eligible policy: contact trials <= 1/30, nav completion (all 3 AMRs) >= 0.80, primary joint
>= 0.80, encounter scoring coverage >= 0.90. None eligible -> NO_ACCEPTABLE_SCALE.
Among eligible: highest observed joint is the reference; every other eligible policy is
compared with it by exact paired McNemar (Holm over the comparisons). Policies not
significantly worse (adjusted p >= 0.05) form the top set; the top set's Pareto front on
(joint, nav, median capped system time) is reported; tie-break = smallest median capped
system time (failures capped at the run limit). All comparisons are exploratory at n = 30.
```

Eligible: none. **Decision: NO_ACCEPTABLE_SCALE**

| vs best | best only | other only | pairs | p | p (Holm) |
|---|---:|---:|---:|---:|---:|

### SECONDARY sensitivity (declared before the main data): robot-closed contact only

Scripted people never avoid the AMR (configs/common.yaml `avoid_robot: false`), so a person can walk into an AMR that has already stopped. The PRIMARY endpoint counts every human-AMR contact; this secondary analysis repeats the same rule counting only contacts the AMR closed. It does not replace the primary decision.

| policy | contact (all) | contact (robot-closed) | joint@5.0 primary | joint@5.0 robot-closed |
|---|---|---|---|---|
| B2_r0.950 | 1/3 (0.06–0.79) | 0/3 (0.00–0.56) | 2/3 (0.21–0.94) | 2/3 (0.21–0.94) |
| S3_a0.60 | 2/3 (0.21–0.94) | 0/3 (0.00–0.56) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) |
| S3_a0.70 | 2/3 (0.21–0.94) | 0/3 (0.00–0.56) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) |
| S3_a0.80 | 2/3 (0.21–0.94) | 0/3 (0.00–0.56) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) |
| S3_a0.90 | 2/3 (0.21–0.94) | 0/3 (0.00–0.56) | 1/3 (0.06–0.79) | 1/3 (0.06–0.79) |

Secondary eligible: none. Secondary decision: NO_ACCEPTABLE_SCALE


Paired vs B2 (exploratory): joint McNemar, nav McNemar, bootstrap median capped-time difference:

| policy | pairs | joint B2-only / policy-only | p (Holm) | nav B2-only / policy-only | p | Δ capped time s (95 % CI) |
|---|---:|---|---:|---|---:|---|
| S3_a0.60 | 3 | 1 / 0 | 1.0 | 0 / 0 | 1.0 | -0.07 [-0.31, 0.27] |
| S3_a0.70 | 3 | 1 / 0 | 1.0 | 0 / 0 | 1.0 | 0.11 [-0.18, 1.67] |
| S3_a0.80 | 3 | 1 / 0 | 1.0 | 0 / 0 | 1.0 | 0.0 [-0.15, 0.72] |
| S3_a0.90 | 3 | 1 / 0 | 1.0 | 0 / 0 | 1.0 | -0.07 [-0.26, 0.5] |

Figures: `figures/S3_rates.png`, `figures/S3_time_path_comfort.png`, `figures/S3_pareto.png`.

