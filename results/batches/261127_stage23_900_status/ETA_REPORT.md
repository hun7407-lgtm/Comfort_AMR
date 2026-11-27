# ETA_REPORT — stage23_900 (generated 2026-11-27 22:11:42 KST)

**Estimate from measured wall-clock time, not a guarantee.** One Isaac Sim worker, trials in sequence.
Hardware: NVIDIA GeForce RTX 5080, 580.173.02; 12th Gen Intel(R) Core(TM) i9-12900.

Initial measured ETA: 3 Stage 2 + 3 Stage 3 timing samples (pilot launches).

**MAIN progress 0.0 %** (0/900 valid)

| stage | target | done | valid | invalid | retried | timeouts | remaining | main started | live trials/h |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|
| Stage 2 | 750 | 0 | 0 | 0 | 0 | 0 | 750 | — | — |
| Stage 3 | 150 | 0 | 0 | 0 | 0 | 0 | 150 | — | — |

Measured wall time per launch (all launches so far, pilot + main, every outcome):

| stage | samples | mean s | median s | P25 s | P90 s | min–max s | runner gap median / P90 s | trials/h (mean) |
|---|---:|---:|---:|---:|---:|---|---|---:|
| Stage 2 | 3 | 77.0 | 77.0 | 76.5 | 77.8 | 76–78 | 0.0 / 0.0 | 46.8 |
| Stage 3 | 3 | 283.0 | 248.0 | 246.0 | 335.2 | 244–357 | 0.5 / 0.9 | 12.7 |

| scenario | Stage 2 MAIN end | Stage 3 MAIN end | all runs (incl. duplicate control) | final analysis |
|---|---|---|---|---|
| optimistic | 2026-11-28 16:44 KST | 2026-11-29 03:21 KST | 2026-11-29 03:45 KST | **2026-11-29 04:15 KST** (30.07 h) |
| central | 2026-11-28 16:59 KST | 2026-11-29 05:07 KST | 2026-11-29 05:36 KST | **2026-11-29 06:06 KST** (31.91 h) |
| conservative | 2026-11-28 18:16 KST | 2026-11-29 09:18 KST | 2026-11-29 09:53 KST | **2026-11-29 10:23 KST** (36.2 h) |

Batches:

| batch | manifest | done | invalid |
|---|---:|---:|---:|
| s2pilot | 60 | 3 | 0 |
| s3pilot | 15 | 3 | 0 |
| s2main | — | 0 | 0 |
| s3main | — | 0 | 0 |
| s3dup | — | 0 | 0 |

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

