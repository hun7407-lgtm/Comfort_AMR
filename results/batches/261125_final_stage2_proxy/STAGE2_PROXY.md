# Stage 2 proxy — Simulation-derived Provisional Lower Bound

> source = **SIMULATION_DERIVED_PROVISIONAL**. Not a human study result, not a human comfort distance. It is how close the simulated navigation stack came to people in clean successful Stage 1 interactions.

R_nav_upper (Stage 1) = 0.9 m. Rule: R_lower_proxy(H,d) = max(Q10 B1, Q10 B2 pooled <= R_nav_upper), each only with N >= 20.

## R_lower_proxy(H, d)

| State | Direction | N (B1 / B2 pooled) | Q10 B1 | Q10 B2 pooled | R_lower_proxy | status |
|---|---|---:|---:|---:|---:|---|
| moving | front | 6 / 42 | 0.605 | 0.469 | 0.469 | OK |
| moving | side | 12 / 55 | 0.486 | 0.369 | 0.369 | OK |
| moving | rear | 0 / 0 | None | None | None | INSUFFICIENT SAMPLE |
| stationary | front | 3 / 10 | 0.639 | 0.945 | None | INSUFFICIENT SAMPLE |
| stationary | side | 16 / 64 | 1.354 | 1.352 | 1.352 | OK |
| stationary | rear | 0 / 0 | None | None | None | INSUFFICIENT SAMPLE |
| station_engaged | front | 0 / 0 | None | None | None | INSUFFICIENT SAMPLE |
| station_engaged | side | 15 / 62 | 2.159 | 2.083 | 2.083 | OK |
| station_engaged | rear | 108 / 368 | 0.494 | 1.001 | 1.001 | OK |

## Source B1

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 6 (INSUFFICIENT) | 0.605 | 0.851 | 0.0 / 0.0 | 0.0 | 0.0 | 0.78 |
| moving | side | 12 (INSUFFICIENT) | 0.486 | 0.727 | 0.0 / 0.0 | 0.0 | 0.0 | 0.798 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 3 (INSUFFICIENT) | 0.639 | 0.685 | 0.0 / 0.0 | 0.0 | 0.0 | 0.8 |
| stationary | side | 16 (INSUFFICIENT) | 1.354 | 1.356 | 0.0 / 0.0 | 0.0 | 0.0 | 0.108 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 15 (INSUFFICIENT) | 2.159 | 2.168 | 0.0 / 0.0 | 0.0 | 0.0 | 0.695 |
| station_engaged | rear | 108 | 0.494 | 1.375 | 0.0 / 0.0 | 0.0 | 0.0 | 0.778 |

## Source B2_pooled_le_upper

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 42 | 0.469 | 0.827 | 0.016 / 0.505 | 0.2 | 0.048 | 0.788 |
| moving | side | 55 | 0.369 | 0.686 | 0.14 / 0.559 | 0.83 | 0.2 | 0.125 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 10 (INSUFFICIENT) | 0.945 | 1.0 | 0.0 / 0.0 | 0.0 | 0.0 | 0.8 |
| stationary | side | 64 | 1.352 | 1.353 | 0.0 / 0.0 | 0.0 | 0.016 | 0.145 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 62 | 2.083 | 2.104 | 0.0 / 0.0 | 0.0 | 0.0 | 0.726 |
| station_engaged | rear | 368 | 1.001 | 1.373 | 0.0 / 0.0 | 0.0 | 0.003 | 0.767 |

## Source B2_r080

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 12 (INSUFFICIENT) | 0.567 | 0.827 | 0.0 / 0.401 | 0.0 | 0.0 | 0.782 |
| moving | side | 19 (INSUFFICIENT) | 0.388 | 0.686 | 0.114 / 0.468 | 0.6 | 0.105 | 0.796 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 3 (INSUFFICIENT) | 0.934 | 0.946 | 0.0 / 0.0 | 0.0 | 0.0 | 0.799 |
| stationary | side | 21 | 1.353 | 1.354 | 0.0 / 0.0 | 0.0 | 0.0 | 0.163 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 20 | 2.105 | 2.111 | 0.0 / 0.0 | 0.0 | 0.0 | 0.716 |
| station_engaged | rear | 121 | 0.935 | 1.371 | 0.0 / 0.0 | 0.0 | 0.0 | 0.773 |

## Source B2_r085

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 15 (INSUFFICIENT) | 0.488 | 0.819 | 0.031 / 0.439 | 0.2 | 0.067 | 0.79 |
| moving | side | 17 (INSUFFICIENT) | 0.341 | 0.721 | 0.129 / 0.526 | 0.8 | 0.176 | 0.125 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 4 (INSUFFICIENT) | 0.992 | 1.0 | 0.0 / 0.0 | 0.0 | 0.0 | 0.8 |
| stationary | side | 18 (INSUFFICIENT) | 1.352 | 1.353 | 0.0 / 0.0 | 0.0 | 0.0 | 0.134 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 18 (INSUFFICIENT) | 2.087 | 2.103 | 0.0 / 0.0 | 0.0 | 0.0 | 0.724 |
| station_engaged | rear | 117 | 0.994 | 1.371 | 0.0 / 0.0 | 0.0 | 0.009 | 0.768 |

## Source B2_r090

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 15 (INSUFFICIENT) | 0.494 | 0.84 | 0.06 / 0.505 | 0.43 | 0.067 | 0.786 |
| moving | side | 19 (INSUFFICIENT) | 0.373 | 0.656 | 0.244 / 0.559 | 1.2 | 0.316 | 0.106 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 3 (INSUFFICIENT) | 1.012 | 1.025 | 0.0 / 0.0 | 0.0 | 0.0 | 0.8 |
| stationary | side | 25 | 1.352 | 1.353 | 0.0 / 0.0 | 0.0 | 0.04 | 0.145 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 24 | 2.08 | 2.097 | 0.0 / 0.0 | 0.0 | 0.0 | 0.733 |
| station_engaged | rear | 130 | 1.033 | 1.376 | 0.0 / 0.0 | 0.0 | 0.0 | 0.766 |

## Source B2_r095

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 13 (INSUFFICIENT) | 0.601 | 0.851 | 0.099 / 0.47 | 0.6 | 0.077 | 0.79 |
| moving | side | 18 (INSUFFICIENT) | 0.372 | 0.534 | 0.416 / 0.644 | 1.41 | 0.278 | 0.123 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 4 (INSUFFICIENT) | 1.052 | 1.06 | 0.0 / 0.0 | 0.0 | 0.0 | 0.796 |
| stationary | side | 24 | 1.352 | 1.354 | 0.0 / 0.0 | 0.0 | 0.125 | 0.189 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 23 | 2.074 | 2.089 | 0.0 / 0.0 | 0.0 | 0.0 | 0.733 |
| station_engaged | rear | 126 | 1.089 | 1.373 | 0.0 / 0.0 | 0.0 | 0.0 | 0.764 |

## Source B2_r100

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 11 (INSUFFICIENT) | 0.688 | 0.829 | 0.171 / 0.32 | 0.8 | 0.0 | 0.793 |
| moving | side | 17 (INSUFFICIENT) | 0.414 | 0.516 | 0.484 / 0.664 | 2.02 | 0.412 | 0.075 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 4 (INSUFFICIENT) | 1.106 | 1.114 | 0.0 / 0.0 | 0.0 | 0.0 | 0.797 |
| stationary | side | 25 | 1.352 | 1.353 | 0.0 / 0.0 | 0.0 | 0.12 | 0.185 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 24 | 2.07 | 2.083 | 0.0 / 0.0 | 0.0 | 0.0 | 0.733 |
| station_engaged | rear | 128 | 1.137 | 1.373 | 0.0 / 0.0 | 0.0 | 0.0 | 0.761 |

## Source B2_r120

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 15 (INSUFFICIENT) | 0.734 | 0.933 | 0.267 / 0.683 | 1.22 | 0.067 | 0.787 |
| moving | side | 18 (INSUFFICIENT) | 0.559 | 0.711 | 0.49 / 0.716 | 2.22 | 0.667 | 0.002 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 4 (INSUFFICIENT) | 1.283 | 1.292 | 0.0 / 0.0 | 0.0 | 0.0 | 0.78 |
| stationary | side | 27 | 1.352 | 1.352 | 0.0 / 0.0 | 0.0 | 0.148 | 0.17 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 27 | 2.051 | 2.06 | 0.0 / 0.0 | 0.0 | 0.0 | 0.753 |
| station_engaged | rear | 126 | 1.33 | 1.375 | 0.0 / 0.0 | 0.0 | 0.008 | 0.749 |

## Source B2_r140

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 15 (INSUFFICIENT) | 0.832 | 1.128 | 0.272 / 1.001 | 1.2 | 0.0 | 0.762 |
| moving | side | 13 (INSUFFICIENT) | 0.624 | 0.82 | 0.58 / 0.909 | 2.62 | 0.769 | 0.0 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 4 (INSUFFICIENT) | 1.424 | 1.438 | 0.0 / 0.0 | 0.0 | 0.0 | 0.753 |
| stationary | side | 14 (INSUFFICIENT) | 1.349 | 1.351 | 0.05 / 0.052 | 0.42 | 0.071 | 0.236 |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 22 | 1.503 | 2.017 | 0.0 / 0.0 | 0.0 | 0.045 | 0.771 |
| station_engaged | rear | 83 | 1.378 | 1.465 | 0.0 / 0.046 | 0.0 | 0.036 | 0.734 |

## Source B2_r160

| State | Direction | N | Q10 clearance | Median | Penetration (median / max) | Intrusion s (median) | standing share | AMR speed at min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| moving | front | 1 (INSUFFICIENT) | 2.22 | 2.22 | 0.0 / 0.0 | 0.0 | 0.0 | 0.061 |
| moving | side | 8 (INSUFFICIENT) | 0.512 | 0.597 | 1.003 / 1.117 | 4.26 | 1.0 | 0.054 |
| moving | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | side | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| stationary | rear | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | front | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | side | 0 (INSUFFICIENT) | None | None | None / None | None | None | None |
| station_engaged | rear | 50 | 1.414 | 1.497 | 0.103 / 0.296 | 0.81 | 0.44 | 0.72 |
