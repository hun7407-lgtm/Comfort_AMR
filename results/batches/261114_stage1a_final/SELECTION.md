# Stage 1 final: buffer selection

Rule and candidates fixed before any candidate ran: `configs/stage1a/buffer_selection.yaml`.
Applied by `scripts/stage1a_finalize.py`. Nothing tuned after the runs.

| candidate | buffer (m) | A environment | B upper boundary | C low-e | D protective stop | passed |
|---|---|---|---|---|---|---|
| buf10 | 0.1 | pass | pass | pass | pass | yes |
| buf15 | 0.15 | pass | FAIL | pass | FAIL | no |

**Selected:** buf10 (buffer 0.1 m)

## buf10 (buffer 0.1 m)

Batch `results/batches/261114_stage1a_buf10`; trials {'scan': 27, 'refine': 28, '1a2': 27, '1b': 36, 'lowe': 18}.

Model: wall clearance 0.49 m (spread 0.1 m), predictions `configs/stage1a/predictions_exec_buf10.json` (commit 41ca38d+).

A. 1A-1 brackets (largest e passed by majority | next e tested):

- W 1.5: 0.42 | 0.47
- W 1.8: 0.72 | 0.77
- W 2.2: 1.02 | 1.07
- W 2.6: 1.42 | 1.47

B. Pre-registered cells:

Boundary itself (model point passes, > model + 0.10 fails): 9/9. Criterion B also needs every lower tested e to pass.

| part | env | kind | R_path^infl | R_exec_model | B ok | boundary match | points e: pass/n |
|---|---|---|---|---|---|---|---|
| 1a2 | w2.20_y0.35_c1 | b3o | 1.2247 | 1.0144 | yes | yes | 0.71: 1/1; 0.91: 1/1; 1.01: 3/3; 1.11: 0/3 (protective_stop, stall); 1.31: 0/1 (stall) |
| 1a2 | w2.20_y0.35_c2 | b3o | 1.2234 | 1.0141 | yes | yes | 0.71: 1/1; 0.91: 1/1; 1.01: 3/3; 1.11: 0/3 (stall); 1.31: 0/1 (stall) |
| 1a2 | w2.20_y0.35_c3 | b3o | 1.224 | 1.0144 | yes | yes | 0.71: 1/1; 0.91: 1/1; 1.01: 3/3; 1.11: 0/3 (protective_stop, stall); 1.31: 0/1 (stall) |
| 1b | w2.00_y0.35_c3 | static | 1.024 | 0.8138 | yes | yes | 0.71: 1/1; 0.81: 2/2; 0.91: 2/2; 1.01: 0/1 (protective_stop) |
| 1b | w2.00_y0.35_c1 | b3o | 1.0241 | 0.8148 | yes | yes | 0.71: 1/1; 0.81: 2/2; 0.91: 1/2 (stall); 1.01: 0/1 (stall) |
| 1b | w2.00_y0.35_c3 | b3o | 1.0247 | 0.8138 | yes | yes | 0.71: 1/1; 0.81: 2/2; 0.91: 2/2; 1.01: 0/1 (stall) |
| 1b | w2.20_y0.60_c3 | static | 0.9739 | 0.7636 | yes | yes | 0.66: 1/1; 0.76: 2/2; 0.86: 0/2 (protective_stop); 0.96: 0/1 (protective_stop) |
| 1b | w2.20_y0.60_c2 | b3o | 0.9736 | 0.7644 | yes | yes | 0.66: 1/1; 0.76: 2/2; 0.86: 0/2 (protective_stop, stall); 0.96: 0/1 (stall) |
| 1b | w2.20_y0.35_c4 | b3o | 1.2237 | 1.0139 | yes | yes | 0.91: 1/1; 1.01: 2/2; 1.11: 0/2 (stall); 1.21: 0/1 (protective_stop) |

C. Low-e:

- w1.50_y0.35_c3 static e 0.3: 3/3
- w1.80_y0.35_c3 static e 0.3: 3/3
- w2.00_y0.35_c3 static e 0.81: 3/3
- w2.20_y0.35_c1 b3o e 0.81: 3/3
- w2.20_y0.35_c3 static e 0.3: 3/3
- w2.60_y0.35_c3 static e 0.3: 3/3
- violations: none

D. Protective stops at or below the boundary: none

## buf15 (buffer 0.15 m)

Batch `results/batches/261114_stage1a_buf15`; trials {'scan': 27, 'refine': 28, '1a2': 27, '1b': 36, 'lowe': 18}.

Model: wall clearance 0.49 m (spread 0.15 m), predictions `configs/stage1a/predictions_exec_buf15.json` (commit 41ca38d+).

A. 1A-1 brackets (largest e passed by majority | next e tested):

- W 1.5: 0.42 | 0.52
- W 1.8: 0.72 | 0.77
- W 2.2: 0.97 | 1.02
- W 2.6: 1.42 | 1.52

B. Pre-registered cells:

Boundary itself (model point passes, > model + 0.10 fails): 8/9. Criterion B also needs every lower tested e to pass.

| part | env | kind | R_path^infl | R_exec_model | B ok | boundary match | points e: pass/n |
|---|---|---|---|---|---|---|---|
| 1a2 | w2.20_y0.35_c1 | b3o | 1.2247 | 0.9643 | yes | yes | 0.66: 1/1; 0.86: 1/1; 0.96: 3/3; 1.06: 0/3 (stall); 1.26: 0/1 (stall) |
| 1a2 | w2.20_y0.35_c2 | b3o | 1.2234 | 0.9635 | yes | yes | 0.66: 1/1; 0.86: 1/1; 0.96: 3/3; 1.06: 0/3 (stall); 1.26: 0/1 (stall) |
| 1a2 | w2.20_y0.35_c3 | b3o | 1.224 | 0.9643 | yes | yes | 0.66: 1/1; 0.86: 1/1; 0.96: 3/3; 1.06: 0/3 (comfort_gate_hold, protective_stop, stall); 1.26: 0/1 (protective_stop) |
| 1b | w2.00_y0.35_c3 | static | 1.024 | 0.7636 | NO | NO | 0.66: 1/1; 0.76: 2/2; 0.86: 2/2; 0.96: 1/1 |
| 1b | w2.00_y0.35_c1 | b3o | 1.0241 | 0.7647 | yes | yes | 0.66: 1/1; 0.76: 2/2; 0.86: 2/2; 0.96: 0/1 (protective_stop) |
| 1b | w2.00_y0.35_c3 | b3o | 1.0247 | 0.765 | yes | yes | 0.66: 1/1; 0.76: 2/2; 0.86: 2/2; 0.96: 0/1 (stall) |
| 1b | w2.20_y0.60_c3 | static | 0.9739 | 0.7135 | yes | yes | 0.61: 1/1; 0.71: 2/2; 0.81: 0/2 (comfort_gate_hold, stall); 0.91: 0/1 (stall) |
| 1b | w2.20_y0.60_c2 | b3o | 0.9736 | 0.7137 | yes | yes | 0.61: 1/1; 0.71: 2/2; 0.81: 0/2 (stall); 0.91: 0/1 (stall) |
| 1b | w2.20_y0.35_c4 | b3o | 1.2237 | 0.9647 | yes | yes | 0.86: 1/1; 0.96: 2/2; 1.06: 0/2 (stall); 1.16: 0/1 (stall) |

C. Low-e:

- w1.50_y0.35_c3 static e 0.3: 3/3
- w1.80_y0.35_c3 static e 0.3: 3/3
- w2.00_y0.35_c3 static e 0.81: 3/3
- w2.20_y0.35_c1 b3o e 0.81: 3/3
- w2.20_y0.35_c3 static e 0.3: 3/3
- w2.60_y0.35_c3 static e 0.3: 3/3
- violations: none

D. Protective stops at or below the boundary: ["1a1 w1.50_y0.35_c3 e 0.42: ['protective_stop', 'pass', 'pass']"]

> Engineering bounds of this navigation stack. Not comfort distances.
