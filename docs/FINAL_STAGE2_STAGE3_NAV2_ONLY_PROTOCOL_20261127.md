# FINAL Stage 2–3 ONLY: Nav2-only Comfort–Navigation 실험 설계 (2026-11-27)

**상태: FINAL EXPERIMENT DESIGN / NOT EXECUTED. 실험 완료를 의미하지 않는다.**

## 0. 우선순위 / 이전 문서 대체

- **사용자 최종 결정: Stage 1은 재실행하지 않는다. Stage 1 신규 코드 수정, manifest 생성, 30회 반복실험, 결과 재산출 작업을 하지 않는다.**
- 이 문서가 **이번 최종 Stage 2·3 실행에 한하여** `docs/FINAL_NAV2_ONLY_3STAGE_PROTOCOL_20261127.md`, `docs/CLAUDE_FINAL_NAV2_ONLY_3STAGE_PROMPT_20261127.md`의 "Stage 1 재실행" 지시를 **대체**한다.
- 과거 Stage 1의 임시 참고값 `R_ref=0.95m`를 **사전 연구로 결정한 engineering reference/normalization anchor**로 고정한다. 이는 10회 반복 잠정치이며 **최종 검증된 Navigation Upper Bound, 보편적 안전거리 또는 인간 실험으로 확인한 Comfort 하한이 아니다.** Stage 1의 0.90m Gate 기반 결과를 새 시스템에 섞지 않는다.
- `docs/NAV2ONLY_STAGE2_RESULTS.md`, `results/batches/261125_final_stage3/`, `results/batches/261126_*`는 과거 약식/레거시 진단자료로만 읽는다. 이전 논문 예시 표의 숫자(예: Stage 3에서 alpha0.80이 최고)는 **만들어진 설명용 초안**이고 실험 목표/정답/실측으로 쓰면 안 된다.
- 이번 연구는 학술적 시스템 프레임워크 연구다. 비대칭 Comfort Space 형상 자체의 신규성, 커스텀 회피제어 신규성, 실측 인간 Comfort의 검증을 주장하지 않는다.

## 1. 검증 가설과 실험 설계의 원칙

- **H1:** 비대칭 Comfort Space가 커지면 로봇 통과 가능성이 감소하거나 정지/교착이 늘 수 있다.
- **H2:** Comfort Space를 작게 하면 Nav2 이동 효율이 좋아질 수 있으나, 실제 인간–AMR 이격거리 감소로 문헌 기반 **예측 Comfort**는 하락할 수 있다.
- **H3:** 중간 Scale `0.70` 또는 `0.80` 근처에서 두 지표가 함께 좋은 운용 절충점이 **존재하는지** 공정하게 검증한다. **반드시 해당 값이 승리하도록 환경/분석을 튜닝하지 않는다.** 극단 Scale이나 고정 원형이 우수하거나 정책 간 차이가 없는 결론도 그대로 보고한다.
- 5가지 Stage 3 정책은 **상태별로 다른 F/S/R 비율을 갖는 참조 형상에 동일한 alpha를 곱한 것**이다. 상태마다 독립적인 alpha를 자동 선정하는 별도 `statewise-adaptive-alpha` 정책 효과를 입증하는 것은 아니다. 해당 주장은 추가 사전등록 6번째 ablation이 있어야 가능하다.

## 2. 전 단계 공통 통제: 실제 Nav2-only 제어/안전 체인

AMR1/a1/a2 모두 `human pose/state → Comfort Costmap Filter Mask (B1은 OFF) → Nav2 global/local costmap → NavFn planner + DWB controller + Nav2 native recovery → cmd_vel_safe → Collision Monitor → cmd_vel → simulator AMR`.

- 새 실험 전부 `control.mode:nav2_only`; `comfort_space.yield.enabled:false`, `yield.evade.enabled:false`, `recovery.backoff.enabled:false`, `recovery.off_lane.enabled:false`, `traffic_rule.enabled:false`, `costmap.buffer_width:0.10`. **custom comfort_gate 노드·Gate HOLD·커스텀 sidestep·후진·off-lane 절대 재도입 금지**.
- Nav2 기본 behavior_server recovery, 물리 안전 Collision Monitor, 원래 obstacle/inflation/footprint, max velocity, LIDAR safety, mask cost value 98, `min_radius:0.30m`는 동일하게 유지; 안전 감시를 낮춰 성능을 좋게 만들면 무효.
- main controlled experiments: `perception.source:tf`, `humans.state_source:ground_truth`. Human state classifying from LIDAR로 검증된 것처럼 주장 금지. Local/global mask가 세 AMR 모두 실제 정책의 size/shape를 읽는지 매 trial 입증.
- 매 launch 병합 YAML 검사 + 실제 ROS graph: `comfort_gate` 부재, `cmd_vel_safe` 발행 Nav2 controller/behavior만, `cmd_vel` 발행 collision_monitor만, `cmd_vel_nav` 발행 없음. 위반 → `INVALID`, 조용히 제외/성공 처리하지 않는다.
- 동일 scenario seed의 정책 간에는 사람 위치·상태·동선·속도·출발시각, 로봇 3대 시작·목표·임무, map/station/obstacles, Nav2·안전파라미터·물리설정이 동일해야 한다. 정책과 F/S/R extent/mask만 다르다. config/geometry/control SHA를 보존한다.

## 3. 정책 정의 및 크기 정규화

- 고정 `R_ref = 0.95m` (기준 반경, Stage 1 재실험 없이 유지).
- **B1:** Comfort Space costmap mask OFF, 그 외 모두 동일.
- **B2:** 반경 `0.95m` 고정 원형, 모든 Human State 동일.
- **B3(alpha):** 문헌에서 참조한 비대칭 F/S/R 형상을 상태별로 적용하고 **하나의 alpha로 F/S/R을 동일 비율 축소**; 축별 임의 튜닝 불가. `E_state(1)=R_ref * E_raw_state / max(E_raw_state)`, `E_state(alpha)=alpha * E_state(1)`. 상태 기준 형상 `station_engaged=(0.80,0.70,0.60)`, `stationary=(0.80,0.90,1.05)`, `moving=(1.20,0.90,0.90)` at 1.0m/s. Moving heading=velocity, stationary/engaged heading=body facing.
- Stage 2 alpha ladder: `[1.00,0.90,0.80,0.70,0.60,0.50,0.45]`; Stage 3 alpha candidates: `[0.60,0.70,0.80,0.90]`.
- actual extent/mask를 log하고 `min_radius=.30m` clipping 없음을 assertion으로 검증한다. `0.45`는 현재 R_ref와 상태 템플릿 기준 clipping 없는 가장 작은 시험값이다.
- 예측 Comfort는 `comfort_core/comfort_reference.py`의 Neggers(2022) 인용 곡선을 **실제 관측된 최소 중심거리, 상대 방위 및 상태**에 적용한 별도 분석 지표다. **Scale·마스크 크기를 Comfort 점수 입력으로 직접 사용하면 안 된다.** 문헌 로봇 속도(~0.35m/s)와 실제 AMR 속도 차이, engaged가 stationary 문헌의 전이 가정이라는 사실을 per-trial flag와 논문 한계에 표시한다.

## 4. Stage 2: Human State별 크기–이격거리–예측 Comfort 분석

**1 human / 1 AMR, 고정된 물리적 상호작용 검증. 총 5 context × 9 policies × 30 independently executed launches = 1,350 MAIN trials.** pilot, smoke는 별도 집계하고 30회를 대신하지 않는다.

| context | state | 상호작용 | 실험정책 |
|---|---|---|---|
| ST-F | Stationary | 사람 정면 통과 | B1, B2, B3 alpha 7종 |
| ST-S | Stationary | 측면 통과 | 동일 |
| ST-R | Stationary | 후방 통과 | 동일 |
| EN-R | Station Engaged | 작업 중인 사람 후방 통과 | 동일 |
| MV-P | Moving | 반대 방향 교행 / 실질적 측면 상호작용 | 동일 |

- **물리적으로 유효한 pilot geometry를 MAIN 시작 전에 확정**: static corridor widths e.g. `2.2,2.4,2.6,2.8m`; moving `2.6,2.8,3.0m` and lane gap `0.55,0.65,0.75,0.85m`, feasible by robot/person footprints. 멋진 결과가 나오는 환경을 사후에 선택하지 말 것. pilot 정의와 채택 이유·실패 이유를 파일에 기록하고 commit/hash freeze 후 main 실행.
- `ST-F`는 옛 pilot에서 B1 접촉으로 탈락했다. **B1 접촉 자체는 B1의 실험 결과**이며 모든 B3 환경을 자동 제외하는 조건이 아니다. 물리 유효성·지정 방향·실제 통과 가능성이 확보되면 모든 정책을 정당하게 비교한다. 확인 불가하면 `UNIDENTIFIABLE_GEOMETRY` 기록하고 실패 시나리오 자체를 삭제·숨김 처리하지 않는다.
- `EN-R`는 `station_engaged`라는 상태 문자만 부여하지 말고 **실제 설비/작업영역 geometric + semantic fixture**를 적절히 구현하여 유지한다(이전 env의 `stations.a/b.enabled:false` 문제 보완). 작업 상황은 시뮬레이션만의 가정으로 표기한다.
- `MV-P`는 이전 차선분리 실험에서 alpha별 실제 최소거리 반응이 0.074m에 불과했다. 사람과 AMR이 실제로 교행하도록 경로/차선/상호작용 범위를 **pilot 단계에서 정책 중립적으로** 설정하고 동결한다. 반응이 다시 <0.10m이면 유효한 결과로 `UNIDENTIFIABLE`; 원하는 결과 맞추려 임의 변경 금지. AMR보다 빠른 보행자를 추월하는 비현실적 상황 추가 금지.
- `C*=5.0/7` 고정 운영 목표, sensitivity `4.5,5.5`; `joint=1`은 (목표 도달 & 사람 접촉 0 & 지정 passage/side/sector/state 성립 & 관측 거리에서 예측 Comfort≥5.0). Navigation 성공만으로 joint 성공 처리하지 않는다.
- 매 조건 관찰 n=30의 nav rate, safe completion, joint rate, contact, protective stop, actual min centre distance + edge clearance, predicted comfort median/distribution, travel time/path/replans/idle. Wilson95% CI.
- 선택 기준은 **관찰 nav≥0.80 및 joint≥0.80**를 만족하는 시험 alpha 중 가장 작은 것. 바로 아래 alpha가 navigation 성공인데 Comfort 실패라면 `IDENTIFIED_BRACKET`; 가장 작은 alpha까지 통과하면 `FLOOR_LIMITED_BOUND_NOT_OBSERVED`; 거리 반응 <0.10m이면 `UNIDENTIFIABLE_GEOMETRY`; 상태별 모든 primary context를 동시 충족하는 공통 alpha가 없으면 `NO_COMMON_FEASIBLE_ALPHA`. 세부 context 점수와 식별불가도 전부 제공.
- Stage 3에 투입할 alpha 0.60/.70/.80/.90는 **사전 계획에서 고정**, Stage 2 winner를 본 뒤 변경하지 않는다.

## 5. Stage 3: B2 vs B3 네 Scale의 다중 AMR 통합 검증

**8 humans / 3 independently Nav2-driven AMRs; 정확히 FIVE cases × 30 newly generated matched scenario seeds = 150 MAIN runs.**

| Case | 정책 | 조건당 |
|---|---|---|
| C1 | B2 static circle radius 0.95m | 30 |
| C2 | B3 uniform alpha 0.60 | 30 |
| C3 | B3 uniform alpha 0.70 | 30 |
| C4 | B3 uniform alpha 0.80 | 30 |
| C5 | B3 uniform alpha 0.90 | 30 |

- 과거 Stage 3 seed 101–130은 이미 본 데이터/레거시 Gate 체인. 새 **holdout 1001–1030** 사용. 미리 10 nominal factory, 10 work-station bottleneck, 10 moving crossing *유형*을 정책 중립으로 정의. 모든 5 정책이 각 seed에서 동일 시작/목표/작업자와 장애물 지도를 사용해야 한다.
- Stage 3의 station/moving/bottleneck 설계는 교착/Comfort trade-off가 **측정 가능하게** 구성하되, 특정 alpha 승리를 유도하는 폭/속도/시작시각/회피 조건 조작 금지. 위험하거나 불가능한 기하를 감지하면 pilot에서 진단하고 **MAIN 실행 전** 고정. 새 본실험 시작 뒤 geometry 변경하면 별도 preregistration과 새 batch가 필요하다.
- 실행 순서는 5정책 × 30개 seed에서 balanced randomization/interleaving. 동일 seed 재현 실행에서도 ROS/Isaac Sim 비결정성이 있을 수 있으므로 동일 정책 10개 seed에 대해 별도 repeated-control diagnostic(본 150회 분모에서 제외), 실패 원인과 session info 보존.
- **Primary Stage 3 joint completion**: 3 AMR의 모든 지정 미션 완료 AND 사람이 접촉한 적 없음 AND 미리 정의한 유효한 Human–AMR encounter에서 문헌 기반 예측 Comfort≥5.0 (각 encounter의 참여자·상태·최소 중심거리·상대방향·평가 가능성 보존). 전혀 encounter가 없거나 scoring 실패/참조영역 바깥이면 자동 joint success 처리하지 말고 `NOT_SCORABLE` + coverage 보고 및 민감도 분석. 사람 이동에 의한 접촉도 최종 안전지표상 접촉으로 기록한다.
- **Other endpoints**: all-robot Nav2 mission completion, safe completion (no contact), per-robot completion, comfort median/P10/per-state, eligible encounters/coverage, human-AMR closest center / footprint edge distance, any contact ratio and duration, physical Collision Monitor protective stop (terminal vs temporary), Nav2 abort/no-progress/robot–robot blocking, travel time per AMR (completed runs only **and** all-run capped time including failures), path length, idle/stopped seconds (NOT custom Yield Gate time), replan count. Report per-policy n, CI and distributions; policy-relative penetration is NOT fair cross-policy distance evidence.
- **Winner rule, frozen before collecting data**: no-contact empirical safety screen <=1/30 cases, 3-AMR mission completion≥0.80, joint≥0.80, adequate scorable encounter coverage (define threshold before launch). If none satisfy: `NO_ACCEPTABLE_SCALE`. Among qualifying policies select highest measured joint rate, and if tied/inconclusive report Pareto-feasible candidates with capped all-run mission time as preregistered tie-break, no fake unique best. Fisher/McNemar for matched binary and suitable paired-bootstrap/CI for continuous, adjust multiple comparisons or label exploratory. Test C*=4.5/5.5 as sensitivity, never change primary C* after results.
- Results may show alpha0.70/0.80, alpha0.60/0.90, circle, or no policy convincingly better. Report all outcomes, not only favorable conditions.
- 5-way **global alpha** is primary scope. Distinct state-wise alpha policy would be optional SIXTH ablation, separately preregistered, 30 extra launches and NOT called one of the required five.

## 6. 기존 코드에서 반드시 확인/수정할 것

1. `comfort_core/final_metrics.py`: old Gate-only `gate_yield_s=hold_s` 및 `NORMAL_YIELD` naming을 새로운 nav2_only 결과에서 Gate Yield로 읽지 않도록 정정, backward-compatibility 유지. Nav2 idle와 Collision Monitor stop을 구분.
2. `comfort_core/comfort_reference.py`: `predicted_comfort_clearance_matched`의 반폭 변환 부호가 문서 식 `d_equiv=d_cc-hw_current+hw_reference`에 일치하는지 확인/테스트. 이는 보조지표이고 이전 실험 주 점수 임의 재계산 금지.
3. All AMR (AMR1/a1/a2) costmap mask/gradient policy, contact/failed/success/predicted Comfort logging correctness. `nav2_only` merged YAML + runtime graph guard always.
4. Report safety failure correctly: Nav2 action SUCCESS + person contact ≠ safe completion. Stop/idle ≠ custom Yield. Collision Monitor settings never relaxed for contact.
5. Stage2 previous station disabled/ST-F pilot failure/MV-P response failure addressed **before** freeze.
6. Stage3 prior Gate-based B2 and "UPPER" same mask nevertheless 11/30 seed outcomes differed; quantify nondeterminism separately rather than overinterpret 1–3 trial difference.
7. Old raw results immutable, new code versioned, deterministic generation and run/resume/idempotent on manifest; full config hash and command per result.
8. Assert **n=30 PER CONDITION** for both stages and final 1,500 main launches only (plus pilot/smoke/diagnostic separately). Infrastructure invalid trials individually tracked and replacement attempts governed by predeclared objective rules; failed missions/contact are outcomes, not dropped data.

## 7. 실행 순서/결과물

1. Read `AGENTS.md`, `docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md`, `docs/NAV2ONLY_STAGE2_RESULTS.md`, old `docs/FINAL_NAV2_ONLY_3STAGE_PROTOCOL_20261127.md` as historical, main code `comfort_core/control_mode.py`, `comfort_core/stage2scale.py`, `scripts/stage2scale_plan.py`, `scripts/final_run.py`, `scripts/make_cross_campaign.py`, `comfort_core/comfort_reference.py`, `comfort_core/final_metrics.py`. For CURRENT execution, THIS document supersedes Stage 1 instructions in all old prompts.
2. Create new `configs/final_nav2only_stage23_v1/`, `scripts/final_stage23_*.py`, `docs/FINAL_STAGE23_*.md`, `results/batches/FINAL_STAGE23_*` (date-based versioned names accepted), no old paths overwritten. Do not create Stage1 generator/runs.
3. Offline tests, complete masked config/graph/safety smoke; if valid, policy-neutral Stage2 geometry pilot and freeze, then Stage2 5×9×30=1,350. Independently pilot/freeze Stage3 physical scenario, then Stage3 5×30=150 with fresh holdouts.
4. Commit protocol, code/config, geometry freeze, manifests and hashes **before** reading main outcome data; then run and commit results. If Isaac Sim/ROS2 unavailable, write/exercise code to extent possible and **explicitly state zero full-run trials**; never invent or backfill results.
5. Outputs: runnable `README` and exact commands; `PROTOCOL_FREEZE.md`, `CONTROL_AUDIT.json`, stage2/3 `manifest.tsv`, per_trial.csv, per_encounter.csv, `STAGE2_REPORT.md/CSV`, `STAGE3_REPORT.md/CSV`, confidence intervals, `figures/` (alpha/comfort/distance; five-case nav/safe/joint, time/path, contacts/pareto), raw logs or verifiable references and code+config SHA.
6. Deliver final Korean thesis-ready **실험 설계·실험 결과·결론, each exactly two paragraphs of three sentences**, with Stage 1 reference only as prior/preliminary methodology; results come from actual Stage 2/3 executions, not placeholder tables. If results inconclusive, write them as inconclusive rather than invent favorable conclusions.

**Bottom line: Stage 1 RUNS = 0; Stage 2 MAIN = 1,350; Stage 3 MAIN = 150; total MAIN = 1,500, plus labeled pilot/smoke/diagnostics.**
