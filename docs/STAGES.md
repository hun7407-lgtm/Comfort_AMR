# 3단계 실험 구조 — Stage 1 / 2 / 3

> **2026-11-14 개정 (현행).** 아래 "개정 설계" 절이 현재 구조다. 그 뒤의
> 2026-11-12 내용(크기 sweep → R_nav_max, Stage 2 사다리)은 기록으로 남긴다.
> `results/batches/261112_stage1_size_sweep`(330 trials)은 **Preliminary
> study**이며 최종 threshold로 쓰지 않는다.

## 개정 설계 (2026-11-14)

```text
Preliminary  기존 cross 330 trials (복잡 환경에서 Static 교착 경향 확인, 원인 분리 불가)
Stage 1      Controlled sim: AMR 1대 + 사람 1명 + 직선 복도. R_nav(E,H) 예측이 실제 통과와 맞는가
Stage 2      Real human study: context별 passing distance -> perceived comfort 곡선
Stage 3-A    Controlled validation: B2 Static Calibrated vs B3-O Dynamic Selection
Stage 3-B    Complex cross, seeds 1-100 paired: 같은 두 정책, failure class별 보고
```

### 공통 정의

| 용어 | 정의 (코드) |
|---|---|
| effective extent e | 사람 중심에서 통로 쪽으로 Comfort Space가 뻗은 길이 (`comfort_core/feasibility.py::effective_extent`). Stage 1 x축이자 Stage 2 passing distance와 같은 양 |
| target_size_m | B3-O 구현 파라미터. e = ratio(context) × target (`comfort_core/contexts.py`: c1 0.50, c2 0.875, c3 0.667, c4 0.75) |
| R_path^infl / R_path^lethal | gate의 배제영역(inscribed 0.24 + gate margin 0.05)과 벽 clearance(inflation 0.34 / inscribed 0.24)로 계산한 경로 존재 한계 |
| R_exec | Isaac Sim + Nav2에서 관측된 통과 경계 (다수결 통과한 최대 e, 다음 실패 e) |
| R_exec_model = R_nav_safe | 실행 모델: Comfort Space를 inscribed + costmap buffer(0.35)만큼 키우고, 벽/몸은 **보정된 wall clearance**로 둔 경로 존재 한계. 1A-1에서 한 파라미터 보정 → 1A-2/1-B는 사전 등록 후 검증 |
| R_comfort_min(H), R_pref(H) | Stage 2 곡선에서 (pilot 후 정의) |
| R* | `comfort_core/selection.py`: R_nav_safe < R_comfort_min이면 PASSAGE_INFEASIBLE(축소 금지, 대기/우회), 아니면 primary min(R_pref, R_nav_safe), fallback R_comfort_min |

### Stage 1 — 왜 실행 모델이 필요한가 (smoke/exploration 관측)

`results/batches/261114_stage1a_smoke`, `261114_stage1a_explore`:
경로 한계(R_path^infl)에서 Comfort Space를 둔 trial은 **gate hold 없이** 멈췄다.
로그상 원인은 두 가지가 겹친 것이다.

1. Comfort costmap의 buffer band(`comfort_space.costmap.buffer_width` 0.35, cost 55) 때문에
   planner가 사람에게서 e + 0.24 + 0.35 이상 떨어진 경로를 고르고, 그만큼 벽 쪽으로 붙는다.
2. collision monitor 보호필드의 측면 폭(footprint 반폭 + scanner tolerance = 0.34,
   `generate_nav2_params.py`)이 inflation 반경(0.34)과 같아서, 벽 쪽 inflation 경계를
   비스듬히 따라가면 필드가 벽에 닿아 **ProtectiveStop** → 후진/재시도 → 12 s 무진전.

안전층은 바꾸지 않는다. 대신 이 효과를 "벽 clearance" 하나로 모델에 넣고 1A-1에서 보정한다.

### Stage 1 실행 순서 (`scripts/stage1a_run.sh --out results/batches/<날짜>_stage1a`)

| phase | 내용 | 파일 |
|---|---|---|
| scan | Step 0 (B1, W 1.5/1.8/2.0/2.2/2.6 ×2) + 1A-1 Static 스캔 (R_path^infl에서 0.10 간격으로 0.30까지, + R_path^lethal+0.10, 각 1회) | `configs/stage1a/manifests/{step0,1a1_scan}.txt`, `predictions.json`(경로 한계 사전등록) |
| refine | 폭마다 관측 경계(마지막 통과, 첫 실패, 그 사이 0.05 간격)를 3회로 | `scripts/stage1a_calibrate.py --refine` |
| fit | wall clearance = (사람→먼 벽) − R_exec_lo − 0.24 − 0.35, 폭별 계산 후 최대값(보수) | `configs/stage1a/model_exec.json` (한 번만 기록) |
| predict | 1A-2(W 2.2, C1/C2/C3), 1-B(W 2.0; y_h 0.60; C4 side) 예측을 **실행 전** 기록 | `configs/stage1a/predictions_exec.json` (덮어쓰기 거부) |
| validate | 1A-2: 모델±(−0.30,−0.10,0×3,+0.10×3,+0.30), 1-B: (−0.10, 0×2, +0.10×2, +0.20) | `manifests/{1a2,1b}.txt` |
| report | 예측 대비 관측, F0–F3 | `scripts/stage1a_report.py` |

채점: e ≤ R_exec_model은 다수결 통과, e > R_exec_model + 0.10은 다수결 실패여야 "consistent".
실패 메커니즘(gate hold / stall / protective stop)은 합치지 않고 나란히 기록한다.

### Stage 1 결과 (2026-11-14 실행)

배치: `results/batches/261114_stage1a` (현행 설정, buffer 0.35), `results/batches/261114_stage1a_nobuf`
(후보 설정: Comfort costmap buffer만 0, gate 경계·안전층 동일), 진단 `261114_stage1a/diag_nobuffer`.

**Step 0 (B1, Comfort 없음)**: 5 폭 × 2 = 10/10 성공, 접촉 0. 단 좁은 복도에서 사람 중심–footprint
최소 0.26–0.30 m (몸 표면 0.01–0.05 m) — 안전층은 slowdown만 걸었다.

**현행 설정 (buffer on), Static 관측 경계 R_exec (3회 반복)**

| W | 통과 | 실패 | R_path^infl |
|---|---|---|---|
| 1.5 | 없음 (0.32도 실패) | ≥0.32 | 0.52 |
| 1.8 | 0.77 | 0.82 | 0.82 |
| 2.2 | 0.77 | 0.82 | 1.22 |
| 2.6 | 1.22 (2/3) | 1.27 | 1.62 |

W 1.8과 2.2가 같은 경계 → 폭으로 설명되지 않는다. 1-파라미터 모델 적합 시 벽 clearance 0.09/0.49/0.44 —
상수가 아님. 원인(궤적 + 진단): buffer band가 넓은 복도에서 planner를 벽 쪽 inflation 경계로 밀고,
보호필드(측면 0.34 m)가 벽을 잡아 ProtectiveStop. **이 설정에서는 환경→경계의 단순 모델이 성립하지 않는다.**
사전등록 예측(1A-2/1-B)은 이 설정으로는 만들지 않았다.

**1A-2 context (현행 설정, model-free scan, W 2.2)**: C1/C2/C3 모두 e ≤ 0.72 통과, e ≥ 0.82 실패 —
target_size 축에서는 1.44 / 0.82 / 1.08로 다르지만 e 축에서 하나로 겹친다
(`261114_stage1a/figures/F3_context_mapping.png`).

**후보 설정 (buffer off)**: 1A-1 경계 W 1.5 0.42|0.47, W 1.8 0.72|0.77, W 2.2 1.12|1.17, W 2.6 1.52|1.57
(각 3회, 경계 안쪽 3/3·바깥 0/3; W 2.6 안쪽 2/3). 벽 clearance **0.490 m, 4개 폭에서 spread 0.000**
(`configs/stage1a/model_exec_nobuf.json`). 이 모델로 1A-2와 1-B를 **실행 전에** 등록
(`predictions_exec_nobuf.json`) → 9개 셀 모두 상한 경계가 예측과 일치 (예측 e에서 전부 통과,
+0.10에서 전부 실패): 1A-2 C1/C2/C3, 1-B W 2.0 (Static, C1, C3), y_h 0.60 (Static, C2), C4 side.
2개 셀은 경계 아래 한 점(e 0.81)이 comfort gate hold로 실패해 INCONSISTENT로 채점됨 — buffer가 없으면
작은 Comfort Space 경계를 planner가 스치고 gate가 붙잡는 현상(buffer가 원래 이것 때문에 도입됨).
반복 측정 `261114_stage1a_nobuf/lowe`: e 0.30은 11/12 통과(W 1.8에서 1회 실패), e 0.81은
W 2.0 Static 0/3, W 2.2 C1 0/3 — **경계 아래에 재현되는 실패 구간**이 있다(경로가 grow 0.24만 피해 gate
margin 0.05 안쪽을 스침). cross 진단(`261114_diag_nobuffer_cross`, seed 1–5): buffer off에서 Static 1.20은
seed 2·3이 성공→gate hold로 악화, B3-O 1.20은 5/5 그대로.

정리: buffer 0.35는 넓은 복도에서 벽 쪽 보호필드 정지를, buffer 0은 gate 경계 스침 정지를 만든다.
둘 다 buffer 폭이 원인이므로 "gate margin + 추종오차보다 크고 벽까지 밀지 않는" 중간 폭이 다음 설계
결정이다(값 선택은 사용자 결정, 아직 실행 안 함).

**결정 필요 (사용자)**: buffer를 유지하면 경계가 환경 기하로 예측되지 않고, 끄면 예측은 되지만 작은 크기에서
gate-grazing 정지가 생긴다. Stage 3의 두 정책 모두 같은 설정을 써야 한다.

### Stage 2 (실행 안 함, interface 갱신)

`configs/stage2/stage2_real.yaml`: `contexts`(c1–c4), `distance_levels_m: null`
(제안 0.50/0.75/1.00/1.25/1.50 — 실기 보호필드·통과속도 확인 전에는 비움).
`scripts/stage2_make_conditions.py`: context × d → B3-O target(=d/ratio) 조건 파일 +
`orders.csv`(context block Williams, block 안 거리 Williams).
기록: `context`, `planned_distance_m`, `measured_distance_m`(사람 중심→footprint = min_clearance + 0.25).
넓은 통제 공간(또는 niche)에서 측정 권장. 실험 전 체크리스트(기관 절차, 동의, 가명, 철회,
안전 브리핑, 비상정지, counterbalancing)는 실행 전에 확인.

### Stage 2 도구 (2026-11-14 갱신, 실행 안 함)

| 항목 | 위치 |
|---|---|
| context on/off | `stage2.contexts` (이름 → true/false; C4는 pilot 후 config에서 끔) |
| 거리 수준 | `stage2.distance_levels_m: null` → 생성기·recorder 모두 거부 |
| 반복·순서 | `stage2.trial_repeats`, context block Williams × 거리 Williams (`orders.csv`: participant_id, trial_id, block, order_index, context, passing_distance_m, repeat, condition) |
| 참가자 ID | `stage2.participant_id_pattern` (`^P[0-9]{2,3}$`), 이름·이메일 거부 |
| 문항·척도 | `stage2.questionnaire` (문구·척도·version은 config, 코드는 key만: comfort_score, perceived_safety, acceptability, optional_comment) |
| trial 기록 | `scripts/stage2_trial.py`: 시작/종료 시각, `actual_min_distance_m`(사람 중심→AMR 표면, primary), `min_center_distance_m`(raw), AMR 속도(평균/최대/최근접 시점), `trial_status`, `stop_reason`, `operator_note` |
| 상태 | NORMAL / OPERATOR_ABORT / PARTICIPANT_STOP / E_STOP / NAVIGATION_FAILURE / INVALID_TRIAL. Ctrl+C → 목표 취소 후 상태 질문; `stage2.safety.estop_topic`(null, 실기 드라이버 확인 후) 설정 시 자동 E_STOP; 사후 수정 `scripts/stage2_mark_trial.py` |
| 분석 | `scripts/stage2_analysis.py`: NORMAL만 곡선에, 그 외 상태는 별도 집계. R_comfort_min / R_pref는 **후보표**만 (`stage2.analysis.*_rule`, `comfort_acceptance_score` 모두 null) |
| 검증 | `scripts/check_stage2.py` |

### Failure verdict 대응표 (`comfort_core/failure_class.py`)

| 보고용 verdict | 세부 class | 판정 |
|---|---|---|
| SUCCESS | SUCCESS | 목표 도달 |
| PASSAGE_INFEASIBLE | PASSAGE_INFEASIBLE | 선택 규칙이 R_nav_safe < R_comfort_min이라 판단한 사람 앞 대기 (의도된 동작) |
| COMFORT_GATE_HOLD | STATIC_OVER_RESERVATION (B2), SELECTION_FAILURE (B3-O) | 통과 가능하다고 본 사람 앞 gate hold / Comfort Space 앞 stall |
| ROBOT_ROBOT_DEADLOCK | ROBOT_ROBOT | 판정 시 fleet AMR ≤ 1.5 m |
| PROTECTIVE_STOP | PROTECTIVE_STOP | collision monitor ProtectiveStop 상태의 stall (Comfort Space·fleet 무관) |
| POST_EVADE_STALL | POST_EVADE_STALL | 회피 후 정체 |
| NAV2_FAILURE | NAV2_FAILURE | nav2 abort / planning failure / timeout |
| INVALID | INVALID | 무효 trial |
| UNCLASSIFIED | UNCLASSIFIED_STALL | 그 외 (따로 보고) |

### Stage 3

- `configs/frozen/comfort_requirements.yaml`: Stage 2 값(전부 null). null이면
  `b3o_selection` 정책과 `stage3_run.sh --main`이 거부한다. Stage 3 결과를 보고 값을 바꾸지 않는다.
- `scripts/stage3_make_configs.py --frozen` (모델·buffer·R_nav_safe bound를 `configs/frozen/navigation_requirements.yaml`에서; 두 정책에 같은 buffer):
  `b3o_selection.yaml`(spot × state별 R_nav_safe 표, 공장 맵 위 국소 경로 탐색) +
  `b2_static_calibrated.yaml`(= max_H R_comfort_min, frozen 전에는 TEMPLATE).
- `comfort_core/selection_policy.py`: 정책 `b3o_selection` (spot에 선 사람만 선택, 보행자는 기존 moving mapping).
- 3-B: `scripts/make_cross_campaign.py`가 seed 1–33 cross config를 byte 단위로 재현(규칙: AMR 2 출발
  1.0+0.3·(s mod 2) s, AMR 3 출발 9.0+0.3·((5s) mod 7) s) → seed 34–100 생성 완료.
  `./scripts/stage3_run.sh --main` (seed마다 정책 순서 교대; 두 frozen 파일 중 하나라도 없거나 null이면 거부).
- 실패 분류 `comfort_core/failure_class.py`: INVALID → PASSAGE_INFEASIBLE → STATIC_OVER_RESERVATION(B2)
  → SELECTION_FAILURE(B3-O; B2에서는 모델 밖 실패) → ROBOT_ROBOT(판정 시 fleet ≤ 1.5 m) →
  POST_EVADE_STALL → NAV2_FAILURE → UNCLASSIFIED_STALL. 입력은 `nav2_goal.py`가 판정 시점에 기록하는
  `verdict_*` 컬럼.
- 보고: `scripts/stage3b_report.py` (exact McNemar, paired Wilcoxon, seed 1–33/34–100 분리, AMR 1/2/3 표기).

표시 이름: AMR 1 / AMR 2 / AMR 3 (`comfort_core/labels.py`). 내부 ID(AMR1/a1/a2)는 ROS namespace·TF·prim·
CSV 값이라 유지한다.

---

## (기록) 2026-11-12 설계

2026-11-12부터의 메인 실험 구조. 이전의 B1/B2/B3/B3-O 4정책 비교
(`results/batches/261107_MAIN_cross_seeds1-33_merged` 등)는 그대로
보존하되 새 실험에서는 쓰지 않는다. 제안 방법은 **B3-O** 하나로
통합한다 (코드 내부 이름은 호환성 때문에 `state_directional` / `b3d`를
유지하고, 새 스크립트와 보고서는 모두 B3-O로 표기한다). 기존 B3
(`state_adaptive`)는 새 runner에서 제외하되 코드·config·결과는 삭제하지
않는다.

```text
Stage 1  Simulation   Comfort Space 크기 sweep  →  R_nav_max  (deadlock이 시작되기 직전의 최대 크기)
Stage 2  Real AMR     R_nav_max에서 Δ씩 줄이며 설문  →  R_comfort_min (acceptable comfort의 최소 크기)
Stage 3  Simulation   B3-O Initial vs B3-O Optimized (+ Static)  →  최종 성능 검증

        R_comfort_min ≤ R* ≤ R_nav_max
```

**물리 안전층(collision monitor, protective field, braking distance)은 어느
단계에서도 변하지 않는다.** 모든 조건 파일은 `scenario`와 `comfort_space`
섹션만 가질 수 있고 (`comfort_core/config.py`가 강제), Nav2 파라미터
생성기는 scenario를 읽지 않는다. `scripts/check_stage1.py` 4번 그룹이
이를 매번 확인한다.

> 어떤 크기도 "experimentally validated comfort distance"가 아니다.
> Stage 1의 크기는 설정 가능한 공학 파라미터의 sweep이고, Stage 2가 사람의
> 판단을 측정한다.

---

## 공통: 크기 조건 = scenario overlay

`comfort_core/size_sweep.py`가 세 단계가 공유하는 정의다.

| 조건 파일 | 내용 |
|---|---|
| `static_r1.00.yaml` | `policy: static`, `static_radius: 1.00` |
| `b3o_r1.00.yaml` | `policy: state_directional`, `aisle_scaling: {enabled: true, target_size_m: 1.00, floor_m: 0.0}` |

B3-O의 "크기"는 **기준 속도(1.0 m/s)에서의 moving front extent**이고,
비스케일 값이 1.20 m이다. `scale = target / 1.20`을 모든 state·방향·
speed mapping·clip에 곱한다 (`comfort_core/comfort_space.py::_scaling`).
front/side/rear 비율, engaged/stationary/moving 관계, heading source,
ellipse offset은 그대로다. `floor_m: 0.0`이면 순수 비례이고
`comfort_space.min_radius`(0.30 m, 사람 몸)만 하한으로 남는다.

적용값 (cross campaign config 기준, `scripts/make_size_conditions.py --table`):

| size | scale | engaged F/S/R | stationary F/S/R | moving@1.0 F/S/R |
|---|---|---|---|---|
| 0.80 | 0.6667 | 0.533/0.467/0.400 | 0.533/0.600/0.700 | 0.800/0.600/0.600 |
| 1.00 | 0.8333 | 0.667/0.583/0.500 | 0.667/0.750/0.875 | 1.000/0.750/0.750 |
| 1.20 | 1.0000 | 0.800/0.700/0.600 | 0.800/0.900/1.050 | 1.200/0.900/0.900 |
| 1.40 | 1.1667 | 0.933/0.817/0.700 | 0.933/1.050/1.225 | 1.400/1.050/1.050 |
| 1.60 | 1.3333 | 1.067/0.933/0.800 | 1.067/1.200/1.400 | 1.600/1.200/1.200 |

모든 trial row는 요청값이 아니라 **적용된 m 값**을 기록한다
(`comfort_size_m, comfort_scale, cs_engaged_*, cs_stationary_*, cs_moving_*`
— `comfort_core/metrics.py` FIELDS 끝, `comfort_core/comfort_space.py::geometry_record`).

### Deadlock 메커니즘 분리

`scripts/nav2_goal.py`가 판정 시점에 `deadlock_mechanism`을 기록한다:

| 값 | 뜻 | 크기 효과? |
|---|---|---|
| `comfort_gate_hold` | yield gate가 Comfort Space 진입을 막고 있는 채로 12 s 무진전 | **예** — 유일하게 크기 때문에 생기는 deadlock |
| `post_evade_stall` | 보행자 회피 후 다시 움직이지 않음 | 아니오 (회피 모듈/지형) |
| `nav2_abort` | Nav2가 goal을 포기 | 아니오 |
| `stall` | 위 어느 것도 아님 | 조사 필요 |
| `planning_failure`, `timeout`, `invalid` | 시작 실패 / 시간 초과 / 물리·측정 결함 | 아니오 |

Stage 1의 threshold는 `comfort_gate_hold`만 센다. 나머지는 옆에 같이
보고하고 절대 합치지 않는다.

---

## Stage 1 — Simulation, Navigation Feasibility Threshold

```bash
./scripts/stage1_sweep.sh --dry-run          # 계획과 적용 F/S/R 표만
./scripts/stage1_sweep.sh --smoke            # seed 5 × {0.8, 1.2, 1.6} × {static, b3o}
./scripts/stage1_sweep.sh                     # 33 seeds × 5 sizes × 2 policies = 330 trials
./scripts/stage1_sweep.sh --sizes "1.25 1.3 1.35" --out results/batches/<...>_stage1_refine
./scripts/stage1_sweep.sh --out <같은 폴더> --skip N   # 중단된 sweep 이어가기
python3 scripts/stage1_report.py --dir results/batches/<...>
python3 scripts/check_stage1.py --dir results/batches/<...>
```

- 시나리오: cross (`configs/campaigns/cross/campaign_h8_a3_cross_s<seed>.yaml`,
  AMR1 + a1 + a2, 작업자 8명). seed당 10개 조건이 **같은 파일**로 돈다.
- 매니페스트는 seed-major (`scripts/stage1_manifest.py`): 중간에 멈춰도
  완료된 seed 단위로 비교 가능.
- 실행은 `scripts/run_campaign.sh` (trial마다 새 시뮬레이션, watchdog,
  시작 실패 1회 재시도).
- 산출: `trials.csv`(AMR1), `fleet_trials.csv`(a1, a2 각각), trial별 JSON
  (궤적 `trajectory`, 첫 Nav2 plan `planned_path` 포함), `REPORT.md`,
  `tables.md`, `thresholds.json`, `figures/P1–P9`.

Threshold 정의 (`scripts/stage1_report.py`, 정책별):

```text
R_deadlock_onset = comfort_gate_hold이 처음 나타나는 가장 작은 size
R_nav_max        = 그 아래에서 comfort_gate_hold이 0인 가장 큰 size
```

모든 size에서 0이면 "R_nav_max ≥ 최대 size (onset은 sweep 밖)", 최소
size에서 이미 나오면 "R_nav_max는 sweep 아래"로 보고한다. size별 개수
표를 항상 함께 출력해 비단조 패턴을 숨기지 않는다. 필요하면
`--sizes`로 threshold 사이를 0.05–0.1 m 간격으로 refine한다.

---

## Stage 2 — Real Environment, Human Comfort Threshold

**아직 실행하지 않는다.** SLAM map과 시나리오가 정해지면 바로 돌 수 있게
interface만 준비되어 있다. 다음이 채워져야 한다
(`comfort_core/stage2.py::missing_placeholders`가 목록을 출력하고,
하나라도 null이면 `stage2_trial.py`는 시작을 거부한다):

| 넣을 곳 | 무엇 |
|---|---|
| `configs/stage2/scenes/<corridor>.yaml` (`scene_template.yaml` 복사) | `nav2.map`(SLAM map yaml), `navigation.start/goal(+yaw)`, `humans.P.position/state/facing_deg`, 복도 폭·통과 측 |
| `configs/stage2/stage2_real.yaml` | `stage2.r_nav_max_m` ← Stage 1 `thresholds.json`(B3-O), `comfort_step_m`(기본 0.1), `n_conditions`, `comfort_acceptance_score`(**null 유지**, pilot 후 결정) |

파일 레이어: `common.yaml → real_amr.yaml → stage2_real.yaml → scenes/<x>.yaml`
+ 조건 overlay `configs/stage2/conditions/b3o_r<size>.yaml`.

```bash
python3 scripts/stage2_make_conditions.py --orders 12        # R_nav_max, -Δ, -2Δ ... + 참가자별 순서
./scripts/stage2_trial.sh --scene configs/stage2/scenes/<x>.yaml \
    --scenario configs/stage2/conditions/b3o_r1.00.yaml \
    --session 2612xx_pilot --participant P01 --order 3 [--check]
python3 scripts/stage2_questionnaire.py --show-items          # 문항·anchor
python3 scripts/stage2_analysis.py --session-dir results/stage2/<session>
```

실기 파이프라인 (기존 노드 재사용):

```text
Real AMR pose (tf map→base) ─┐
Human pose/state/orientation ─┼→ scripts/comfort_publisher.py (B3-O, 조건 overlay)
                              │      → KeepoutFilter mask → Nav2 costmap → Nav2 → comfort_gate → collision monitor → AMR
                              └→ scripts/stage2_trial.py (기록) → results/stage2/<session>/
```

기록 (`comfort_core/stage2.py::TRIAL_FIELDS`): 궤적(`trajectories/<trial>.csv`,
10 Hz, 사람 위치·clearance·yield·safety action 포함), travel time/distance,
planned path length, min human distance·시각, human state/facing/speed,
적용 Comfort Space F/S/R, intrusion, yield s, safety stop/slowdown s,
Nav2 status, commit.

설문 (`QUESTIONNAIRE_FIELDS`): `participant_id, trial_id, condition,
comfort_size, comfort_score, perceived_safety, distance_acceptability,
trial_order, timestamp, notes`. Primary item은 Neggers et al. (2022, IJSR,
doi 10.1007/s12369-021-00805-6)의 7점 perceived comfort; secondary 두
항목의 문구는 사용 전 reference 확인(TODO). 개인정보는 pseudonym 코드만.

분석 (`stage2_analysis.py`): size별 mean/median/sd/분포, 참가자×size 표,
Friedman(scipy 있을 때), 2-segment breakpoint(탐색적), 그리고
`comfort_acceptance_score`가 설정된 경우에만 `R_comfort_min`. 기본은 null
이므로 cutoff 없음을 명시하고 R_comfort_min을 주장하지 않는다. raw
`questionnaire.csv`는 수정하지 않는다.

---

## Stage 3 — Simulation, Optimized B3-O Validation

```bash
python3 scripts/stage3_make_configs.py                 # static_baseline, b3o_initial, b3o_optimized(TEMPLATE)
./scripts/stage3_run.sh --dry-run
./scripts/stage3_run.sh --r-opt <R_opt>                 # b3o_initial vs b3o_optimized, 33 seeds
./scripts/stage3_run.sh --r-opt <R_opt> --with-static   # + static baseline (3 조건)
./scripts/stage3_run.sh --params calibrated.yaml        # 비율까지 바뀐 보정 블록이면
python3 scripts/stage3_report.py --dir results/batches/<...>_stage3_validation
```

- `b3o_initial` = 비스케일 B3-O (Stage 1의 1.20 m 조건과 동일).
- `b3o_optimized` = Stage 2 결과. `--r-opt`(크기)나 `--params`(directional
  블록)로만 채워지며, template(null) 상태에서는 runner가 거부한다.
- 보고: 조건별 success / comfort deadlock / 메커니즘 / travel time·
  distance / 첫 plan 길이와 detour / yield·wait / clearance / contact /
  intrusion / robot-robot gap, seed별 paired Δ(Optimized − Initial),
  궤적+plan overlay (`figures/V4_trajectories_s<seed>.png`).

---

## 검증 (코드만, 시뮬 없이)

```bash
python3 scripts/check_stage1.py                 # 1042 checks: config / scaling / same-seed / safety
python3 scripts/check_stage1.py --dir <결과>     # + CSV 검증
python3 scripts/check_comfort_shapes.py         # 기존 정책 불변
./scripts/stage1_sweep.sh --dry-run
./scripts/stage3_run.sh --dry-run
```
