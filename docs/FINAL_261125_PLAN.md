# Final Stage 1–3 experiments — audit, changes, plan (rev. 2026-11-25)

이 문서는 2026-11-25 요청(Stage 1 Navigation Upper Bound → Stage 2 Simulation-derived Provisional Lower Bound → Stage 3 B1/B2/B3-O holdout 비교)의 **실행 전 기록**이다. 판정 규칙은 해당 Stage 데이터를 보기 전에 여기 적었다. 실행 중 바뀌는 것은 §8에 날짜와 함께 추가한다.

## 1. Repository audit (실제 코드 기준)

| 항목 | 확인 결과 | 근거 |
|---|---|---|
| AMR 1 (`AMR1`) | Nav2: global/local costmap → NavFn planner → DWB controller → `/cmd_vel_nav` → comfort_gate → collision_monitor → `/cmd_vel` | `scripts/nav2_bringup.launch.py` |
| AMR 2/3 (`a1`, `a2`) | 각자 namespace의 Nav2 stack. `fleet.driver: nav2`. fleet_goals.py는 NavigateToPose goal만 보낸다(조향 없음). controller → `/aN/cmd_vel_safe` → collision monitor → `/aN/cmd_vel` | `scripts/fleet_goals.py`, `run_fleet_nav2.sh`, cross campaign `fleet.driver: nav2` |
| Comfort Space in costmap | AMR 1의 comfort_publisher가 하나의 keepout mask(`/costmap_filter_info`)를 내고 3대 모두의 global/local costmap `comfort_filter`가 같은 mask를 읽는다 | `generate_nav2_params.py` (`COSTMAP_FILTERS`), bringup docstring |
| Nav2 우회 | **AMR 1에만** 있었다: comfort_gate의 (1) walker evasion(sidestep, 자체 cmd_vel 타이머), (2) safety-hold 후 후진(backoff), (3) off-lane recovery(직접 회전/전진). a1/a2에는 없음 | `scripts/comfort_gate.py` `_evade_tick`, `_backoff_tick`, `_recover_tick` |
| Hold gate | AMR 1의 gate는 Nav2 명령을 0으로 바꿔 Comfort Space 앞에서 멈춘다(조향하지 않음). a1/a2에는 gate가 없고 costmap의 Comfort mask만 적용된다 | `comfort_gate.py` `_on_cmd` |
| Human randomization | seed → `configs/scenarios/paper_h8_a3_scr_s<NNNN>.yaml`(ScenarioGenerator, `random.Random(draw_key)`): 교대하는 북/남 작업자(N1–N3, S1–S3 중 하나씩), 교대 시작시점, 이동속도, 그에 따른 이동경로. cross campaign이 fleet 출발시각(seed 규칙)을 정한다. AMR 1 mission(n1_r2c3 → s3_r2c2), fleet job, 사람 8명의 기본 위치·상태, map, Nav2, safety layer는 seed와 무관하게 고정 | `comfort_core/scenario.py` L686–767, `scripts/make_cross_campaign.py` |
| Seed 재현성 | 같은 seed = 같은 config 파일(byte 동일, `make_cross_campaign.py --verify`). 실행은 비결정적이다: Isaac Sim/ROS 2 비동기 타이밍 때문에 같은 config 재실행 결과가 달라진 사례가 있다(2026-11-15 top-view 재녹화: W2.2 e1.07 PROTECTIVE_STOP→SUCCESS, cross s2 B3-O SUCCESS→GATE_HOLD). 따라서 영상은 **그 trial 자체의 녹화**만 쓴다(재생 replay 사용 안 함) | `results/videos/paper_261114/top_runs` |
| B1 | `comfort_space.policy: none` (NoComfortPolicy): mask 없음, gate 비활성. safety layer 그대로 | `comfort_core/comfort_space.py` |
| B2 | `policy: static`, `static_radius` r: 모든 사람에게 같은 원 | 〃 |
| B3-O | `policy: state_directional`: state(moving / stationary / station_engaged) × 방향(front/side/rear) extents; moving은 속도 선형식, heading은 velocity, 정지자는 body facing | `configs/common.yaml` `comfort_space.directional` |
| B3 (state-only) | `state_adaptive` 코드가 남아 있으나 **사용하지 않는다** | — |
| Comfort parameter 의미 | 반경/extent = **사람 중심 → AMR footprint 가장 가까운 외곽**까지 비워 둘 거리. planner용 mask는 AMR 반폭(0.24 m)만큼 키워서 칠한다(`robot_grow`). buffer 0.10 m는 그 바깥의 soft-cost 띠. gate는 0.6 s lookahead 위치가 grown shape + margin 0.05 안이면 hold | `comfort_space.robot_grow`, `comfort_gate._intrusion` |
| 기존 결과 재사용 | `261112_stage1_size_sweep`(33 seeds, Static/B3-O 0.8–1.6)은 buffer 0.35, evasion on → 이번 설정과 달라 **수치 재사용 불가**(사전연구로만 인용). `261114_stage1a*`(통제 corridor)의 buffer 0.10 선정·동결은 그대로 쓴다. cross seed 1–100 config는 그대로 사용, 101–200은 같은 generator로 생성 | — |

## 2. 실험 전 수정 (코드)

1. **Nav2 전용 주행.** `comfort_space.yield.evade.enabled: false`, `recovery.backoff.enabled: false`, 신규 `recovery.off_lane.enabled: false` (`comfort_gate.py`에 flag 추가, 기본값 true라 기존 config 동작 불변). AMR 1의 gate는 hold(0 명령)만 한다. Nav2 behavior_server recovery(spin/backup/wait)는 Nav2 자체 기능이라 유지.
2. **Safety layer 유지.** footprint, collision monitor(protective stop), obstacle/inflation layer, Nav2 파라미터는 common.yaml 그대로. 어떤 조건도 이 값을 바꾸지 않는다.
3. **Trial log.** `scripts/trial_logger.py`(수신 전용): 5 Hz로 3대 pose(map), 사람별 declared/observed state·위치·속도·heading·자기 Comfort extent(F/S/R), collision monitor action, AMR 1 gate 상태; 모든 `/plan`(3대) 이벤트; `/fleet/status`. comfort_publisher가 `/comfort_state_log`(JSON)를 추가로 낸다(제어 경로 무관).
4. **녹화.** 모든 trial을 고정 정사영 top view(CAM01_TOP, 30 m, 1920×1080, 10 fps, sim clock) + pose log로 녹화한다. 영상은 그 trial의 증거이고 재녹화 replay는 쓰지 않는다.
5. **Runner.** `scripts/final_run.py`: manifest 순서대로 trial당 새 시뮬레이션, trial 폴더별 결과, `DONE` 표식으로 resume, 기동 실패(시뮬/ Nav2 미기동, row 없음, sim 정지)만 재시도(최대 2회, 실패 시도 폴더 보존).
6. **설정 파일.** `scripts/final_plan.py` → `configs/final/seeds/s<N>.yaml`(seed config + 위 고정 설정), `configs/final/conditions/*.yaml`(policy와 크기만).

## 3. 공통 고정 조건

Navigation = Stage 1(통제 corridor)에서 동결한 설정: Comfort costmap buffer 0.10 m (`configs/frozen/navigation_requirements.yaml`). 같은 seed에서 사람 동작·교대·시작시각, AMR mission·초기조건, 환경, Nav2, safety layer가 모두 같고 **Comfort Space 설정만** 다르다.

## 4. Stage 1 — Navigation Upper Bound (B2 Static size sweep)

Run matrix (seed 순으로 interleave: seed마다 B1, 0.80, 1.00, 1.20, 1.40, 1.60):

| 단계 | 조건 | seeds | trials |
|---|---|---|---:|
| Baseline | B1 | 1–100 | 100 |
| Coarse | B2 r = 0.80 / 1.00 / 1.20 / 1.40 / 1.60 | 1–100 | 500 |
| Fine | transition 구간 0.05 간격, 양 끝은 coarse 재사용 (예: 0.85/0.90/0.95) | 1–100 | 300 |

**실패 분류 (AMR별, trial 종료 시점의 상황으로 판정).** SUCCESS / NORMAL_YIELD(성공, hold 1–10 s) / TEMPORARY_HOLD(성공, hold ≥ 10 s) / PERSISTENT_COMFORT_HOLD / COMFORT_OVERLAP_BLOCK / SINGLE_HUMAN_COMFORT_BLOCK / HUMAN_MOTION_BLOCK / ROBOT_ROBOT_BLOCK / NAV2_ABORT / PROTECTIVE_STOP / TIMEOUT / UNKNOWN. 판정 순서와 기준은 `comfort_core/final_metrics.py`의 docstring에 고정한다(데이터 보기 전).

**Comfort-induced deadlock (trial 단위).** 3대 중 하나라도 PERSISTENT_COMFORT_HOLD / COMFORT_OVERLAP_BLOCK / SINGLE_HUMAN_COMFORT_BLOCK로 끝난 trial. paired 지표로 "같은 seed B1은 system success" 인 경우만 센 값도 함께 낸다. 모든 비율은 `count / denominator`와 Wilson 95% CI로 표시.

**R_nav_upper 판정 규칙 (사전 등록).** 
- transition size = comfort-induced deadlock rate가 처음으로 5/100을 넘는 크기(coarse에서 찾고 그 아래 coarse 크기와의 구간을 0.05 간격 fine search).
- `R_nav_upper` = fine search까지 포함해 comfort-induced deadlock rate ≤ 5/100이면서 system success가 같은 seed의 B1보다 5 %p 이상 낮지 않은 **가장 큰 크기**.
- 이 값은 현재 cross scenario에서의 operational upper bound이며 모든 상황의 수학적 최대값이 아니다.

## 5. Stage 2 — Simulation-derived Provisional Lower Bound

- source = `SIMULATION_DERIVED_PROVISIONAL`. 사람 실험 결과가 아니고 human comfort distance가 아니다.
- 입력: Stage 1의 clean successful interaction(그 AMR mission 성공, contact 0, protective stop 0, Nav2 abort 없음, persistent deadlock 없음). 3대 모두의 human–AMR interaction.
- interaction = 한 사람–한 AMR 쌍에서 사람 중심→AMR footprint 거리가 2.5 m 미만인 연속 구간. 그 구간의 최소 거리(physical clearance), 그 순간의 state(declared, 걸음 중이면 moving), 상대방향(사람 heading 기준 AMR footprint 최근접점의 방위: |θ| ≤ 45° front, ≥ 135° rear, 그 외 side; heading = 서 있으면 config의 facing, 걷는 중이면 속도 방향), AMR 속도, Comfort 침범 깊이·시간, hold 여부.
- `R_lower_proxy(H,d)` = clean interaction의 physical clearance **Q10**. N < 20이면 `INSUFFICIENT SAMPLE`.
- **순환성 주의:** B2 trial의 clearance는 B2 크기에 의해 만들어진다(gate가 r 앞에서 세움). 그래서 표를 source별로(B1 / B2 크기별 / pooled) 모두 내고, 사전 등록한 기본값은 **B1 clean interaction의 Q10과 B2(r ≤ R_nav_upper) pooled Q10 중 큰 값**이다. B1 Q10은 "Comfort 없이 Nav2가 실제로 남기는 거리", B2 pooled는 "운용 가능한 범위의 static 설정이 남긴 거리"이다. 둘 다 사람의 편안함을 측정한 값이 아니다.
- 단위: Comfort parameter와 clearance는 둘 다 사람 중심 → AMR footprint 외곽 거리이므로 직접 대응한다(§1). gate margin 0.05 m와 buffer 0.10 m는 navigation 설정에 포함돼 있고 R(H,d)에 더하지 않는다.

## 6. Stage 3 — holdout seeds 101–200

- B1 × 100, B2 × 100, B3-O × 100 (= 300), 같은 seed 조건.
- B3-O: `state_directional`의 9칸 extent를 `R(H,d) = R_lower(H,d) + α (R_nav_upper − R_lower(H,d))`로 둔다. main = **MID (α = 0.5)**. sensitivity: LOWER(α=0), UPPER(α=1) 각 100 (B3-O만, +200). moving은 속도 의존 없이 같은 R(moving,d)(intercept=R, gain 0). INSUFFICIENT 칸은 같은 state의 가용 칸 최대값, 없으면 전체 최대값(보수적).
- B2: 같은 수준의 static = `max_{H,d} R_MID(H,d)` (모든 칸 요구를 하나의 원으로 만족시키는 가장 작은 static). R_nav_upper를 넘으면 R_nav_upper로 자른다.
- 평가는 §4 지표 + Human Interaction(최소 거리, Comfort penetration, overlap, intrusion duration) + Safety(protective stop, contact). System success = 3대 모두 mission 성공.

## 7. 영상

모든 trial이 그 실행 자체의 top-view 녹화를 가진다. 조건마다 최소 5개를 결과 기반 규칙으로 고른다(typical success, low-yield success, high yield/temporary hold, comfort-induced failure, overlap/multi-human); 해당 유형이 없으면 비우고 `NO DEADLOCK OBSERVED IN THIS CONDITION`. 같은 seed의 크기 비교(stack), upper-bound 비교, overlap deadlock, Stage 3 matched seed(5 seeds × 3 policies), B2 vs B3-O side-by-side(Case A–E 중 실제 존재하는 것). `video_metadata.json`은 trial의 DONE/CSV에서 직접 만든다. overlay 정보는 trial_log에서 그린다.

## 8. 진행 기록

- 2026-11-25 00:20–00:49: audit, 코드 수정(§2), smoke test 6 trials(`results/batches/261125_final_smoke`, seeds 2/5/12 × B1/B2 1.20). 파이프라인 전 구간 동작 확인: seed 5 B2 → AMR 1 SINGLE_HUMAN_COMFORT_BLOCK(E1), seed 5 B1 → a2 PROTECTIVE_STOP(걷는 사람이 와서 닿음), seed 12 B1 → AMR 1 Nav2 abort. trial당 약 240–380 s.
- 2026-11-25 00:49: `scripts/final_pipeline.py` 시작(무인, resume 가능). Stage 1 coarse 600 trials부터. `scripts/final_post.sh`가 끝나면 report와 영상을 만든다(영상 렌더링은 CPU를 많이 써서 trial과 겹치지 않게).
- 주의: 첫 trial `s001_B1`의 기동 구간이 smoke 영상 렌더링 테스트와 겹쳤다(그 trial wall 474 s, host_load 6.5 기록). 주행 구간은 렌더링 종료 후였고 결과는 SUCCESS. 이후 trial 실행 중에는 무거운 작업을 하지 않는다.
- 2026-11-25 04:58: remote desktop disconnected; gnome-remote-desktop no longer uses GPU SM. Trial wall time back to ~220–260 s (success) / ~395 s (deadlock, runs to timeout). Coarse 35/600. Partial (6 seeds) shows B1 system success 3/6 (a2 fails even without Comfort Space) and a comfort deadlock already at 0.80 (1/6) — a single-seed figure, not a decision; the pre-registered rule is unchanged.
- 2026-11-25 11:40 — **Reduced run (user decision):** 30 seeds per condition instead of 100. Stage 1 seeds 1–30 (coarse + fine), Stage 3 holdout seeds 101–130 (main + sensitivity). Manifests keep all 100 seeds, so seeds 31–100 / 131–200 can be added later with the full ranges. Rules unchanged and rate-based: comfort-induced deadlock ≤ 5 % means ≤ 1/30; system success ≥ same-seed B1 − 5 %p; Stage 2 cell minimum N ≥ 20 kept (more INSUFFICIENT cells expected). Implemented as `final_run.py --seeds` and `final_pipeline.py --stage1-seeds 1-30 --stage3-seeds 101-130`; the running coarse batch is NOT restarted: it continues as it is and is cut right after s030_B2_r160 finishes (the s031_B1 trial it starts at that moment is stopped within seconds, left without DONE and excluded); the pipeline then continues with the 30-seed ranges.
- 2026-11-25 17:15 — Coarse cut after seed 30 (180/180 trials of seeds 1–30). Coarse decision (rule as written): transition 1.00 m, R_nav_upper candidate 0.80 m, fine sizes 0.85/0.90/0.95 × seeds 1–30 started 17:14:56. s031_B1 (started at the cut) has no DONE and is excluded.
- **Excluded trial, reported, not hidden:** s022_B2_r080 was INVALID in all 3 attempts ("no odometry for 20 s": the simulation stopped). Every attempt shows the same sequence: walking person S1 closes in on fleet AMR a2 (contact −0.02 m, "person closed in"), a2's pose jumps 2.0 m in 0.1 s, then the simulation stops. This is a reproducible physics divergence at a human–AMR contact, not a random startup failure. It is kept out of the rates (B2 0.80 n = 29), as the runner's infrastructure rule says. It does not change the coarse decision: comfort deadlock at 0.80 is 1/29, or 1/30 if counted as a non-comfort failure; both are ≤ 5 %.
- 2026-11-26 00:38 — **Stage 1 final (seeds 1–30): R_nav_upper = 0.90 m**, transition 0.95 m. Comfort-induced deadlock: 0.80 1/29, 0.85 1/30, 0.90 1/30, 0.95 2/30, 1.00 4/30. System success 0.90 = 28/30 vs same-seed B1 20/30.
- **Stage 2 proxy (as computed by the pre-registered rule):** OK cells moving|front 0.469, moving|side 0.369, stationary|side 1.352, station_engaged|side 2.083, station_engaged|rear 1.001; INSUFFICIENT moving|rear, stationary|front, stationary|rear, station_engaged|front. Three OK cells (stationary|side, station_engaged|side, station_engaged|rear) are **above** R_nav_upper, so they and the INSUFFICIENT cells filled from them are capped at 0.90.
- **Consequence for Stage 3 (reported, not changed):** B2 = 0.90; B3-O MID = 0.90 for every stationary / station_engaged cell and 0.684 / 0.635 / 0.684 (front/side/rear) for moving people; LOWER moving 0.469/0.369/0.469; **UPPER = 0.90 everywhere, i.e. the same Comfort Space as B2** (it now serves as a replicate of B2 on the same seeds). So B3-O differs from B2 only for moving people.
- Interpretation note: the proxy is Q10 of the clearance the navigation stack happened to keep in clean successes. For stationary / station-engaged people it reflects where the aisles put the AMR (e.g. station_engaged|side, AMR never closer than ~2 m), not a distance the AMR needs; that is why it can exceed R_nav_upper. To be discussed in the report; the rule was not changed after seeing the data.
- 2026-11-26 13:21 — **Stage 3 done** (main B1/B2/B3-O MID + sensitivity LOWER/UPPER, seeds 101–130, 150 trials, no exclusions). 14:05 reports and videos done (`scripts/final_post.sh`). Report: `results/batches/261125_final_stage3/analysis/STAGE3_REPORT.md`. System success: B1 24/30, B2 22/30, B3-O MID 23/30, LOWER 28/30, UPPER 25/30. Comfort-induced deadlock: B1 0, B2 1, MID 1, LOWER 0, UPPER 2 (of 30). B2 vs B3-O MID McNemar 4 vs 5, p = 1.0.
- **Run-to-run variance (found after the run, reported, not hidden):** B3-O UPPER has exactly the same Comfort Space as B2 (0.90 everywhere) on the same seeds, so it is a replicate. Their system success disagrees on **11 of 30 seeds** (B2 22/30, UPPER 25/30; B2-only 4, UPPER-only 7). B2 vs B3-O MID disagree on 9 of 30. So the same seed does not give the same outcome twice: timing in the simulation (ROS message timing, GPU-bound render loop) changes the interaction. Per-seed comparisons between policies at n = 30 cannot separate a policy effect of this size from replicate noise. Differences smaller than about ±3–4 successes in 30 should not be read as effects.
- **Penetration / intrusion metrics** are measured against each policy's own Comfort Space extent, so a smaller extent gives smaller penetration by construction (B3-O MID 0.429 m vs B2 0.665 m vs UPPER 0.675 m). They are not evidence that people were kept farther away; min human–AMR distance is the same for all policies (median 0.22–0.24 m, dominated by walking people passing an AMR).
- Video case A (B2 comfort deadlock while B3-O succeeds on the same seed) did not occur in Stage 3, so it was not made (s120: B2 comfort deadlock, B3-O failed for another reason).
