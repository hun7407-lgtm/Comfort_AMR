# Stage 1 controlled: predicted vs observed navigation boundary

Batches: scan, refine, 1a2, 1b, lowe. 136 AMR 1 trials. Predictions: `configs/stage1a/predictions_buf10.json` (commit 7a6692d+), `configs/stage1a/predictions_exec_buf10.json` (commit 41ca38d+), each written before the runs it predicts.

One AMR, one person, straight corridor, start (-5, 0) -> goal (5, 0), no fleet, no walkers. e = how far the person's Comfort Space reaches toward the passage, from the person's centre (the quantity the gate keeps the AMR's nearest surface out of).

- R_path^infl: largest e with a robot-centre path when walls/body are kept at the inflation radius (conservative; R_nav_safe).
- R_path^lethal: the same with the inscribed radius (optimistic upper bound).
- R_exec_model: execution model fitted on the 1A-1 widths only (Comfort Space grown by inscribed radius + costmap buffer; walls at the fitted wall clearance), pre-registered for 1A-2 and 1-B. Scored: e <= R_exec_model must pass, e > R_exec_model + 0.10 must not.
- R_exec: observed boundary, bracket (last e passed by the majority, first e not).

## Step 0 (B1, no Comfort Space)

not run

## Cells

| cell | R_path^infl e (param) | R_path^lethal e (param) | R_exec_model e | R_exec bracket e | R_path^infl - R_exec | verdict (scored against) | points e: pass/n (mechanisms) |
|---|---|---|---|---|---|---|---|
| 1a1  W 1.50  y_h 0.35  Static | 0.524 (0.52) | 0.613 (0.61) | - | 0.42 - 0.47 | 0.10 | INCONSISTENT (R_path (infl/lethal)) | 0.30: 3/3; 0.32: 1/1; 0.42: 3/3; 0.47: 1/3 (comfort_gate_hold 1, stall 1); 0.52: 0/3 (stall 2, comfort_gate_hold 1); 0.71: 0/1 (stall 1) |
| 1a1  W 1.80  y_h 0.35  Static | 0.823 (0.82) | 0.914 (0.91) | - | 0.72 - 0.77 | 0.10 | INCONSISTENT (R_path (infl/lethal)) | 0.30: 3/3; 0.32: 1/1; 0.42: 1/1; 0.52: 1/1; 0.62: 1/1; 0.72: 3/3; 0.77: 0/3 (stall 2, stall/protective-stop 1); 0.82: 0/3 (stall 1, stall/slowdown 2); 1.01: 0/1 (stall/slowdown 1) |
| 1a1  W 2.20  y_h 0.35  Static | 1.225 (1.22) | 1.313 (1.31) | - | 1.02 - 1.07 | 0.20 | INCONSISTENT (R_path (infl/lethal)) | 0.30: 4/4; 0.72: 1/1; 0.82: 1/1; 0.92: 1/1; 1.02: 3/3; 1.07: 1/3 (stall 2); 1.12: 0/3 (stall 2, stall/protective-stop 1); 1.22: 0/1 (stall 1); 1.41: 0/1 (stall 1) |
| 1a1  W 2.60  y_h 0.35  Static | 1.624 (1.62) | 1.714 (1.71) | - | 1.42 - 1.47 | 0.20 | INCONSISTENT (R_path (infl/lethal)) | 0.30: 4/4; 1.12: 1/1; 1.22: 1/1; 1.32: 1/1; 1.42: 3/3; 1.47: 0/3 (stall 2, stall/protective-stop 1); 1.52: 0/3 (stall/slowdown 1, stall 2); 1.62: 0/1 (stall 1); 1.81: 0/1 (stall/slowdown 1) |
| 1a2  W 2.20  y_h 0.35  B3-O C1 | 1.225 (2.45) | 1.314 (2.63) | 1.014 | 1.01 - 1.11 | 0.21 | consistent (R_exec_model) | 0.71: 1/1; 0.81: 3/3; 0.91: 1/1; 1.01: 3/3; 1.11: 0/3 (stall 1, stall/protective-stop 2); 1.31: 0/1 (stall 1) |
| 1a2  W 2.20  y_h 0.35  B3-O C2 | 1.223 (1.40) | 1.315 (1.50) | 1.014 | 1.01 - 1.11 | 0.22 | consistent (R_exec_model) | 0.71: 1/1; 0.91: 1/1; 1.01: 3/3; 1.11: 0/3 (stall 3); 1.31: 0/1 (stall 1) |
| 1a2  W 2.20  y_h 0.35  B3-O C3 | 1.224 (1.84) | 1.314 (1.97) | 1.014 | 1.01 - 1.11 | 0.22 | consistent (R_exec_model) | 0.71: 1/1; 0.91: 1/1; 1.01: 3/3; 1.11: 0/3 (stall/protective-stop 2, stall 1); 1.31: 0/1 (stall/slowdown 1) |
| 1b  W 2.00  y_h 0.35  Static | 1.024 (1.02) | 1.115 (1.11) | 0.814 | 0.91 - 1.01 | 0.11 | consistent (R_exec_model) | 0.71: 1/1; 0.81: 5/5; 0.91: 2/2; 1.01: 0/1 (stall/protective-stop 1) |
| 1b  W 2.00  y_h 0.35  B3-O C1 | 1.024 (2.05) | 1.115 (2.23) | 0.815 | 0.81 - 0.91 | 0.21 | consistent (R_exec_model) | 0.71: 1/1; 0.81: 2/2; 0.91: 1/2 (stall 1); 1.01: 0/1 (stall 1) |
| 1b  W 2.00  y_h 0.35  B3-O C3 | 1.025 (1.54) | 1.115 (1.67) | 0.814 | 0.91 - 1.01 | 0.12 | consistent (R_exec_model) | 0.71: 1/1; 0.81: 2/2; 0.91: 2/2; 1.01: 0/1 (stall/slowdown 1) |
| 1b  W 2.20  y_h 0.60  Static | 0.974 (0.97) | 1.065 (1.06) | 0.764 | 0.76 - 0.86 | 0.21 | consistent (R_exec_model) | 0.66: 1/1; 0.76: 2/2; 0.86: 0/2 (stall/protective-stop 2); 0.96: 0/1 (stall/protective-stop 1) |
| 1b  W 2.20  y_h 0.60  B3-O C2 | 0.974 (1.11) | 1.065 (1.22) | 0.764 | 0.76 - 0.86 | 0.21 | consistent (R_exec_model) | 0.66: 1/1; 0.76: 2/2; 0.86: 0/2 (stall 1, stall/protective-stop 1); 0.96: 0/1 (stall 1) |
| 1b  W 2.20  y_h 0.35  B3-O C4 | 1.224 (1.63) | 1.315 (1.75) | 1.014 | 1.01 - 1.11 | 0.21 | consistent (R_exec_model) | 0.91: 1/1; 1.01: 2/2; 1.11: 0/2 (stall 2); 1.21: 0/1 (stall/protective-stop 1) |

## Figures

- F0: `figures/F0_step0.png` (no data yet)
- F1: `figures/F1_feasibility_map.png`
- F2: `figures/F2_pred_vs_exec.png`
- F3: `figures/F3_context_mapping.png`

> Engineering bounds of this navigation stack (footprint, inflation, gate margin). Not comfort distances. The physical safety layer is identical in every trial.
