# Stage 1 Nav2-only revalidation (ABBREVIATED, provisional)

Chain: Nav2 controller → collision monitor for every AMR; no comfort_gate (yield / evade / backoff / off-lane); buffer 0.10 m. Seeds 1–10 of the legacy Stage 1 scene. Legacy 0.90 m came from the gate chain and is not reused as a result.

Launches 50 (excluded 0). **Decision: IDENTIFIED** {"R_nav_upper": 0.95, "transition": 1.0, "n_launches": 50, "n_excluded": 0}

| condition | n | system success | B1 same seeds | comfort-mask no-progress | contact trials | protective-stop trials | pass | gate-free | failure classes |
|---|---:|---|---|---|---|---|---|---|---|
| B1 | 10 | 8/10 (80 %, 49–94) | 8/10 (80 %, 49–94) | 0/10 (0 %, 0–28) | 9/10 (90 %, 60–98) | 8/10 (80 %, 49–94) | — | True | AMR1:NAV2_ABORT, a2:HUMAN_MOTION_BLOCK, a2:NAV2_ABORT |
| B2_r085 | 10 | 10/10 (100 %, 72–100) | 8/10 (80 %, 49–94) | 0/10 (0 %, 0–28) | 9/10 (90 %, 60–98) | 2/10 (20 %, 6–51) | True | True |  |
| B2_r090 | 10 | 10/10 (100 %, 72–100) | 8/10 (80 %, 49–94) | 0/10 (0 %, 0–28) | 9/10 (90 %, 60–98) | 2/10 (20 %, 6–51) | True | True |  |
| B2_r095 | 10 | 10/10 (100 %, 72–100) | 8/10 (80 %, 49–94) | 0/10 (0 %, 0–28) | 9/10 (90 %, 60–98) | 0/10 (0 %, 0–28) | True | True |  |
| B2_r100 | 10 | 8/10 (80 %, 49–94) | 8/10 (80 %, 49–94) | 0/10 (0 %, 0–28) | 10/10 (100 %, 72–100) | 2/10 (20 %, 6–51) | False | True | AMR1:ROBOT_ROBOT_BLOCK, a1:ROBOT_ROBOT_BLOCK, a2:HUMAN_MOTION_BLOCK |

Rule (fixed before the data):

```
(same thresholds as the legacy rule of
docs/FINAL_261125_PLAN.md §4, with the gate-based quantities replaced):
    comfort-mask no-progress = an AMR's class is a comfort class of
        final_metrics (a person's Comfort Space, grown by the AMR half width,
        blocks its plan / position for >= 50 % of the 12 s before the verdict).
        Without a gate this can only come from the costmap mask; the gate-only
        class PERSISTENT_COMFORT_HOLD cannot occur.
    size r PASSES iff
        comfort-mask no-progress rate <= 5 %                 (0 of 10 at n = 10)
        system success >= same-seed B1 system success - 5 %p  (>= B1 count at n = 10)
        human-AMR contact trials <= same-seed B1 contact trials
    R_nav_upper(nav2-only) = largest tested size that passes (legacy rule, not monotone);
    transition = smallest tested size that fails above it.
    Extension: if the largest tested size passes -> upper NOT identified in the tested range;
    if no size passes -> upper not justified (stop, report).
```

