# Stage 1 controlled: predicted vs observed navigation boundary

Batches: step0, scan, refine, 1a2_scan. 88 AMR 1 trials. Predictions: `configs/stage1a/predictions.json` (commit c44bc0c+), each written before the runs it predicts.

One AMR, one person, straight corridor, start (-5, 0) -> goal (5, 0), no fleet, no walkers. e = how far the person's Comfort Space reaches toward the passage, from the person's centre (the quantity the gate keeps the AMR's nearest surface out of).

- R_path^infl: largest e with a robot-centre path when walls/body are kept at the inflation radius (conservative; R_nav_safe).
- R_path^lethal: the same with the inscribed radius (optimistic upper bound).
- R_exec_model: execution model fitted on the 1A-1 widths only (Comfort Space grown by inscribed radius + costmap buffer; walls at the fitted wall clearance), pre-registered for 1A-2 and 1-B. Scored: e <= R_exec_model must pass, e > R_exec_model + 0.10 must not.
- R_exec: observed boundary, bracket (last e passed by the majority, first e not).

## Step 0 (B1, no Comfort Space)

| environment | trials | outcomes | min distance person centre - AMR 1 footprint (m) |
|---|---|---|---|
| w1.50_y0.35_c3 | 2 | {'SUCCESS': 2} | 0.26, 0.28 |
| w1.80_y0.35_c3 | 2 | {'SUCCESS': 2} | 0.30, 0.30 |
| w2.00_y0.35_c3 | 2 | {'SUCCESS': 2} | 0.35, 0.29 |
| w2.20_y0.35_c3 | 2 | {'SUCCESS': 2} | 0.49, 0.45 |
| w2.60_y0.35_c3 | 2 | {'SUCCESS': 2} | 0.69, 0.73 |

## Cells

| cell | R_path^infl e (param) | R_path^lethal e (param) | R_exec_model e | R_exec bracket e | R_path^infl - R_exec | verdict (scored against) | points e: pass/n (mechanisms) |
|---|---|---|---|---|---|---|---|
| 1a1  W 1.50  y_h 0.35  Static | 0.524 (0.52) | 0.613 (0.61) | - | - - 0.32 | - | INCONSISTENT (R_path (infl/lethal)) | 0.32: 0/1 (stall/protective-stop 1); 0.42: 0/1 (stall/protective-stop 1); 0.52: 0/1 (stall 1); 0.71: 0/1 (stall/slowdown 1) |
| 1a1  W 1.80  y_h 0.35  Static | 0.823 (0.82) | 0.914 (0.91) | - | 0.77 - 0.82 | 0.05 | INCONSISTENT (R_path (infl/lethal)) | 0.32: 1/1; 0.42: 1/1; 0.52: 1/1; 0.62: 1/1; 0.72: 2/3 (stall 1); 0.77: 2/3 (stall/protective-stop 1); 0.82: 0/3 (stall 2, stall/slowdown 1); 1.01: 0/1 (comfort_gate_hold 1) |
| 1a1  W 2.20  y_h 0.35  Static | 1.225 (1.22) | 1.313 (1.31) | - | 0.77 - 0.82 | 0.46 | INCONSISTENT (R_path (infl/lethal)) | 0.32: 1/1; 0.42: 1/1; 0.52: 1/1; 0.62: 1/1; 0.72: 3/3; 0.77: 3/3; 0.82: 1/3 (stall 1, stall/protective-stop 1); 0.92: 0/1 (stall 1); 1.02: 0/1 (stall 1); 1.12: 0/1 (stall 1); 1.22: 0/1 (stall 1); 1.41: 0/1 (stall 1) |
| 1a1  W 2.60  y_h 0.35  Static | 1.624 (1.62) | 1.714 (1.71) | - | 1.22 - 1.27 | 0.40 | INCONSISTENT (R_path (infl/lethal)) | 0.32: 1/1; 0.42: 1/1; 0.52: 1/1; 0.62: 1/1; 0.72: 1/1; 0.82: 1/1; 0.92: 1/1; 1.02: 1/1; 1.12: 1/1; 1.22: 2/3 (stall 1); 1.27: 1/3 (stall 1, stall/protective-stop 1); 1.32: 0/3 (stall 3); 1.42: 1/3 (stall 2); 1.52: 0/1 (stall 1); 1.62: 0/1 (stall/slowdown 1); 1.81: 0/1 (stall/protective-stop 1) |
| scan  W 2.20  y_h 0.35  B3-O C1 | 1.225 (2.45) | 1.314 (2.63) | - | 0.72 - 0.82 | 0.51 | INCONSISTENT (R_path (infl/lethal)) | 0.52: 1/1; 0.62: 1/1; 0.72: 1/1; 0.82: 0/1 (stall 1); 0.92: 0/1 (stall 1); 1.02: 0/1 (stall 1) |
| scan  W 2.20  y_h 0.35  B3-O C2 | 1.223 (1.40) | 1.315 (1.50) | - | 0.72 - 0.82 | 0.51 | INCONSISTENT (R_path (infl/lethal)) | 0.52: 1/1; 0.62: 1/1; 0.72: 1/1; 0.82: 0/1 (stall 1); 0.92: 0/1 (stall/slowdown 1); 1.02: 0/1 (stall 1) |
| scan  W 2.20  y_h 0.35  B3-O C3 | 1.224 (1.84) | 1.314 (1.97) | - | 0.72 - 0.82 | 0.50 | INCONSISTENT (R_path (infl/lethal)) | 0.52: 1/1; 0.62: 1/1; 0.72: 1/1; 0.82: 0/1 (stall 1); 0.92: 0/1 (stall/slowdown 1); 1.02: 0/1 (stall 1) |

## Figures

- F0: `figures/F0_step0.png`
- F1: `figures/F1_feasibility_map.png`
- F2: `figures/F2_pred_vs_exec.png`
- F3: `figures/F3_context_mapping.png`

> Engineering bounds of this navigation stack (footprint, inflation, gate margin). Not comfort distances. The physical safety layer is identical in every trial.
