# Stage 2 confirmation verdict (Nav2-only v2, ABBREVIATED)

Status from the coarse sweep; confirmation = new launches of the decisive cells. Predicted comfort is a published-curve prediction at the actual closest distance, not measured comfort.

| context | coarse status @5.0 | alpha* | target | alpha* joint (95 % CI) | alpha* nav | next smaller | its joint | its nav | median d_cc alpha* / next (m) | median C alpha* / next | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ST-F | UNIDENTIFIABLE_GEOMETRY | — | — | — | — | — | — | — | — | — | not run |
| ST-S | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 5.0 | 10/10 (0.72–1.00) | 10/10 | None | — | — | 0.7814 / None | 5.093 / None | OPERATIONAL_ALPHA_REPRODUCED |
| ST-S | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 4.5 | 10/10 (0.72–1.00) | 10/10 | None | — | — | 0.7814 / None | 5.093 / None | OPERATIONAL_ALPHA_REPRODUCED |
| ST-R | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 5.0 | 10/10 (0.72–1.00) | 10/10 | None | — | — | 0.8282 / None | 5.0925 / None | OPERATIONAL_ALPHA_REPRODUCED |
| ST-R | FLOOR_LIMITED_BOUND_NOT_OBSERVED | 0.45 | 4.5 | 10/10 (0.72–1.00) | 10/10 | None | — | — | 0.8282 / None | 5.0925 / None | OPERATIONAL_ALPHA_REPRODUCED |
| EN-R | IDENTIFIED_BRACKET | 0.6 | 5.0 | 10/10 (0.72–1.00) | 10/10 | 0.5 | 0/10 (0.00–0.28) | 10/10 | 0.8351 / 0.7439 | 5.1115 / 4.83 | CONFIRMED |
| MV-P | UNIDENTIFIABLE_GEOMETRY | — | — | — | — | — | — | — | — | — | not run |
| MV-O | INFEASIBLE | — | — | — | — | — | — | — | — | — | not run |

Rule:

```
the coarse selection criteria applied to the confirmation cells):
    alpha* REPRODUCED   iff joint rate >= joint_min and nav rate >= nav_min at alpha*
    bracket REPRODUCED  iff (IDENTIFIED_BRACKET only) the next smaller alpha fails
                        (joint rate < joint_min) and that failure is a comfort failure
                        (nav rate >= nav_min), so it is not a navigation-only bound
    CONFIRMED  = both reproduced (bracket contexts) / alpha* reproduced (floor-limited:
                 still NOT a comfort lower bound; the status stays FLOOR_LIMITED)
    NOT_CONFIRMED otherwise. Sensitivity: joint >= 0.90 and C* 4.5 / 5.5 reported.
```

