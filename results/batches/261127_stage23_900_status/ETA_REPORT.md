# ETA_REPORT — stage23_900 (generated 2026-11-28 13:22:02 KST)

**Estimate from measured wall-clock time, not a guarantee.** One Isaac Sim worker, trials in sequence.
Hardware: NVIDIA GeForce RTX 5080, 580.173.02; 12th Gen Intel(R) Core(TM) i9-12900.

**MAIN progress 63.3 %** (570/900 valid)

| stage | target | done | valid | invalid | retried | timeouts | remaining | main started | live trials/h |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|
| Stage 2 | 750 | 570 | 570 | 0 | 0 | 0 | 180 | 2026-11-28 00:24 | 44.03 |
| Stage 3 | 150 | 0 | 0 | 0 | 0 | 0 | 150 | — | — |

Measured wall time per launch (all launches so far, pilot + main, every outcome):

| stage | samples | mean s | median s | P25 s | P90 s | min–max s | runner gap median / P90 s | trials/h (mean) |
|---|---:|---:|---:|---:|---:|---|---|---:|
| Stage 2 | 630 | 81.9 | 77.0 | 77.0 | 82.0 | 75–144 | 0.0 / 1.0 | 44.0 |
| Stage 3 | 15 | 249.8 | 243.0 | 241.0 | 247.2 | 236–357 | 0.0 / 1.0 | 14.4 |

| scenario | Stage 2 MAIN end | Stage 3 MAIN end | all runs (incl. duplicate control) | final analysis |
|---|---|---|---|---|
| optimistic | 2026-11-28 17:48 KST | 2026-11-29 04:10 KST | 2026-11-29 04:34 KST | **2026-11-29 05:04 KST** (15.71 h) |
| central | 2026-11-28 18:02 KST | 2026-11-29 04:46 KST | 2026-11-29 05:11 KST | **2026-11-29 05:41 KST** (16.33 h) |
| conservative | 2026-11-28 18:18 KST | 2026-11-29 05:30 KST | 2026-11-29 05:56 KST | **2026-11-29 06:26 KST** (17.07 h) |

Batches:

| batch | manifest | done | invalid |
|---|---:|---:|---:|
| s2pilot | 60 | 60 | 0 |
| s3pilot | 15 | 15 | 0 |
| s2main | 750 | 570 | 0 |
| s3main | 150 | 0 | 0 |
| s3dup | 6 | 0 | 0 |

Method:

```
wall-clock time is used: every trial's DONE `wall_s` (Isaac Sim start,
ROS 2 bring-up, the run, teardown and data writing, all retries of that trial) and
the gap between one trial's end and the next one's start (runner overhead). One Isaac
Sim worker on one GPU: trials run one after another (no parallelism is assumed).

Per stage, from all of its launches so far (pilot + main, every outcome incl. timeouts):
    optimistic   = P25 wall + median gap        per remaining trial
    central      = mean wall + median gap
    conservative = P90 wall + P90 gap, + 5 % retry / infra margin
Once >= 30 MAIN trials of a stage exist, central also reports the live throughput
(trials/hour over the main run's elapsed wall time). Remaining work is executed in the
campaign order below; a fixed allowance covers freeze / commit / analysis steps.
No sample -> that stage's ETA is NOT computed (stated as such).
```

