# Stage 1 controlled: predicted vs observed navigation boundary

Batches: 261114_stage1a_smoke. 3 AMR 1 trials. Predictions: `configs/stage1a/predictions.json` (commit c44bc0c+, written before the runs).

One AMR, one person, straight corridor, start (-5, 0) -> goal (5, 0), no fleet, no walkers. e = how far the person's Comfort Space reaches toward the passage, from the person's centre (the quantity the gate keeps the AMR's nearest surface out of).

- R_path^infl: largest e with a robot-centre path when walls/body are kept at the inflation radius (conservative; R_nav_safe).
- R_path^lethal: the same with the inscribed radius (optimistic upper bound).
- R_exec: observed boundary, bracket (last e passed by the majority, first e not).

## Step 0 (B1, no Comfort Space)

| environment | trials | outcomes | min distance person centre - AMR 1 footprint (m) |
|---|---|---|---|
| w1.50_y0.35_c3 | 1 | {'SUCCESS': 1} | 0.27 |

## Cells

| cell | R_path^infl e (param) | R_path^lethal e (param) | R_exec bracket e | verdict | points e: pass/n (mechanisms) |
|---|---|---|---|---|---|
| 1a1  W 1.50  y_h 0.35  Static | 0.524 (0.52) | 0.613 (0.61) | - - 0.52 | INCONSISTENT (partial) | 0.52: 0/1 (stall 1); 0.91: 0/1 (stall 1) |
| 1a1  W 1.80  y_h 0.35  Static | 0.823 (0.82) | 0.914 (0.91) | - - - | not run | - |
| 1a1  W 2.20  y_h 0.35  Static | 1.225 (1.22) | 1.313 (1.31) | - - - | not run | - |
| 1a1  W 2.60  y_h 0.35  Static | 1.624 (1.62) | 1.714 (1.71) | - - - | not run | - |
| 1a2  W 2.20  y_h 0.35  B3-O C1 | 1.225 (2.45) | 1.314 (2.63) | - - - | not run | - |
| 1a2  W 2.20  y_h 0.35  B3-O C2 | 1.223 (1.40) | 1.315 (1.50) | - - - | not run | - |
| 1a2  W 2.20  y_h 0.35  B3-O C3 | 1.224 (1.84) | 1.314 (1.97) | - - - | not run | - |
| 1b  W 2.00  y_h 0.35  Static | 1.024 (1.02) | 1.115 (1.11) | - - - | not run | - |
| 1b  W 2.00  y_h 0.35  B3-O C1 | 1.024 (2.05) | 1.115 (2.23) | - - - | not run | - |
| 1b  W 2.00  y_h 0.35  B3-O C3 | 1.025 (1.54) | 1.115 (1.67) | - - - | not run | - |
| 1b  W 2.20  y_h 0.60  Static | 0.974 (0.97) | 1.065 (1.06) | - - - | not run | - |
| 1b  W 2.20  y_h 0.60  B3-O C2 | 0.974 (1.11) | 1.065 (1.22) | - - - | not run | - |
| 1b  W 2.20  y_h 0.35  B3-O C4 | 1.224 (1.63) | 1.315 (1.75) | - - - | not run | - |

## Figures

- F0: `figures/F0_step0.png`
- F1: `figures/F1_feasibility_map.png`
- F2: `figures/F2_pred_vs_exec.png`
- F3: `figures/F3_context_mapping.png` (no data yet)

> Engineering bounds of this navigation stack (footprint, inflation, gate margin). Not comfort distances. The physical safety layer is identical in every trial.
