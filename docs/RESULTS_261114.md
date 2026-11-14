# 결과 요약 — 2026-11-14 (Stage 1 controlled + Stage 2/3 준비)

이 문서는 2026-11-14 작업의 **진입점**이다. 숫자는 아래 링크된 원본(`trials.csv`, `REPORT.md`, JSON)에서
나온 것이며, 여기서 새로 만든 값은 없다. 설계 전체는 [`docs/STAGES.md`](STAGES.md) "개정 설계" 절.

## 0. 한 줄 요약

- Stage 1(시뮬, 1 AMR + 1 사람 + 직선 복도)을 무인으로 247 trial 실행했다.
- **통과 경계를 결정한 것은 사람 위치·복도 폭만이 아니라 Comfort costmap buffer band(폭 0.35 m)였다.**
  - buffer 켬(현행): 넓은 복도에서 planner가 벽 쪽으로 밀려 collision monitor 보호필드가 멈춤 → 경계가 폭으로 설명되지 않음.
  - buffer 끔(후보): 1-파라미터 기하 모델이 4개 폭에서 정확히 맞고, 사전등록한 9개 셀의 상한 경계가 모두 예측과 일치. 대신 경계보다 작은 크기에서 gate 경계를 스치는 정지가 재현됨.
- buffer 폭은 **미결정(사용자 결정)**. Stage 2(실험자 실기)와 Stage 3(B2 vs B3-O)는 **코드만 준비, 실행 안 함**.

## 1. 용어 (읽기 전에)

| 용어 | 뜻 |
|---|---|
| AMR 1 / AMR 2 / AMR 3 | 표시 이름. 내부 ID는 `AMR1` / `a1` / `a2` (ROS ns, TF, CSV 값) 그대로 — `comfort_core/labels.py` |
| e (effective extent) | 사람 중심에서 통로 쪽으로 Comfort Space가 뻗는 거리. Stage 1의 x축. `target_size_m`은 구현 파라미터일 뿐 |
| R_path^infl | 벽·몸을 inflation 반경(0.34)으로 둘 때 경로가 존재하는 최대 e. = R_nav_safe (Conservative Navigation Feasibility Bound) |
| R_path^lethal | inscribed 반경(0.24)만 쓸 때의 낙관 상한 |
| R_exec | 관측 경계: "과반 통과한 마지막 e – 그 다음 실패한 e" bracket |
| R_exec_model | 1A-1에서만 적합한 실행 모델(벽 clearance 1개 파라미터)의 예측 |
| Static / B3-O | Static = 방향 무관 원형 Comfort Space, B3-O = context별 방향성 모양 |
| C1–C4 | c1 engaged-rear, c2 stationary-rear, c3 stationary-front, c4 side |
| comfort_gate_hold | Comfort gate가 진입을 막아 멈춘 경우. stall/protective-stop = 안전층(collision monitor)이 멈춘 경우 |

이 값들은 이 내비게이션 스택의 **공학적 경계**이며 comfort distance가 아니다. 물리 안전층은 모든 trial에서 동일하다.

## 2. Stage 1 결과

### 2.1 Step 0 — B1 (Comfort Space 없음)
5 폭 × 2 = **10/10 성공, 접촉 0**. 좁은 복도에서 사람 중심–footprint 최소 0.26–0.30 m (몸 표면 1–5 cm); 안전층은 감속만.
→ `results/batches/261114_stage1a/step0/`, 그림 `figures/F0_step0.png`

### 2.2 1A-1 Static 경계 — 현행 설정 (buffer 0.35)
→ `results/batches/261114_stage1a/REPORT.md`

| W (m) | 통과한 최대 e | 실패한 최소 e | R_path^infl |
|---|---|---|---|
| 1.5 | 없음 | 0.32 | 0.52 |
| 1.8 | 0.77 | 0.82 | 0.82 |
| 2.2 | 0.77 | 0.82 | 1.22 |
| 2.6 | 1.22 (2/3) | 1.27 | 1.62 |

- W 1.8과 2.2의 경계가 같다 → **폭으로 설명 불가**. 1-파라미터 적합 시 벽 clearance 0.09 / 0.49 / 0.44 (상수 아님).
- 원인: buffer band(cost 55)가 planner를 벽 쪽 inflation 경계로 밀고, 보호필드 측면 반폭(0.34)이 벽을 잡아 ProtectiveStop. gate hold 아님.
- 그래서 이 설정으로는 사전등록 예측을 **만들지 않았다** (`configs/stage1a/model_exec.json` 없음).

### 2.3 1A-2 Human Context 매핑 — 현행 설정, W 2.2, B3-O
- target_size 축 한계: C1 1.44 / C2 0.82 / C3 1.08 (서로 다름)
- e 축: 세 context 모두 **e ≤ 0.72 통과, e ≥ 0.82 실패** → 하나로 겹침.
- 즉 B3-O의 context 효과는 "방향별 모양 → 통로 쪽 실효 크기 e"로 설명된다. → `261114_stage1a/figures/F3_context_mapping.png`

### 2.4 후보 설정 (buffer 0) — 적합 + 사전등록 검증
→ `results/batches/261114_stage1a_nobuf/REPORT.md`, 모델 `configs/stage1a/model_exec_nobuf.json`,
예측 `configs/stage1a/predictions_exec_nobuf.json` (실행 전에 작성, 덮어쓰기 거부)

1A-1 (적합용, 각 경계점 3회):

| W (m) | 통과 | 실패 | 적합된 벽 clearance |
|---|---|---|---|
| 1.5 | 0.42 | 0.47 | 0.490 |
| 1.8 | 0.72 | 0.77 | 0.490 |
| 2.2 | 1.12 | 1.17 | 0.490 |
| 2.6 | 1.52 (2/3) | 1.57 | 0.490 |

spread 0.000 → 1-파라미터 모델 성립.

사전등록 검증 (채점: e ≤ 예측은 통과해야 하고, e > 예측 + 0.10은 실패해야 함):

| 셀 | 조건 | 예측 e | 관측 bracket | 상한 일치 | 판정 |
|---|---|---|---|---|---|
| 1A-2 | W 2.2, C1 | 1.115 | 1.11 – 1.21 | ✓ | INCONSISTENT (e 0.81에서 0/3 gate hold) |
| 1A-2 | W 2.2, C2 | 1.114 | 1.11 – 1.21 | ✓ | consistent |
| 1A-2 | W 2.2, C3 | 1.115 | 1.11 – 1.21 | ✓ | consistent |
| 1-B (새 폭) | W 2.0, Static | 0.914 | 0.91 – 1.01 | ✓ | INCONSISTENT (e 0.81에서 0/3 gate hold) |
| 1-B (새 폭) | W 2.0, C1 | 0.914 | 0.91 – 1.01 | ✓ | consistent |
| 1-B (새 폭) | W 2.0, C3 | 0.914 | 0.91 – 1.01 | ✓ | consistent |
| 1-B (새 위치) | y_h 0.60, Static | 0.864 | 0.86 – 0.96 | ✓ | consistent |
| 1-B (새 위치) | y_h 0.60, C2 | 0.864 | 0.86 – 0.96 | ✓ | consistent |
| 1-B (새 방향) | W 2.2, C4 side | 1.114 | 1.11 – 1.21 | ✓ | consistent |

- **9/9 셀 상한 경계 일치.** 7/9 완전 consistent.
- 경계 아래 실패: buffer가 없으면 planner가 grow(0.24)만 피하고 gate margin(0.05) 안쪽을 스쳐 gate가 붙잡는다 (buffer가 원래 이 때문에 도입됨).
  반복 측정 `261114_stage1a_nobuf/lowe`: e 0.30 → 11/12 통과, e 0.81 → W 2.0 Static 0/3, W 2.2 C1 0/3 (재현됨).

### 2.5 Cross 시나리오 진단 — buffer 0, 크기 1.20, seed 1–5
→ `results/batches/261114_diag_nobuffer_cross/trials.csv`

| seed | Static 1.20 | B3-O 1.20 |
|---|---|---|
| 1 | SUCCESS | SUCCESS |
| 2 | comfort_gate_hold | SUCCESS |
| 3 | comfort_gate_hold | SUCCESS |
| 4 | SUCCESS | SUCCESS |
| 5 | comfort_gate_hold | SUCCESS |

Preliminary(buffer on, `261112_stage1_size_sweep`)과 비교하면 Static seed 2·3이 성공 → gate hold로 바뀌었고, B3-O는 5/5 그대로.

### 2.6 정리 / 결정 필요
- buffer 0.35 → 벽 쪽 보호필드 정지 (폭으로 경계 예측 불가)
- buffer 0 → gate 경계 스침 정지 (상한은 예측되지만 작은 크기에서 실패)
- 제안(미실행): "gate margin + 추종오차보다 크고, 벽까지 밀지 않는" 중간 폭(예: 0.10–0.15 m)으로 Stage 1 재보정(약 2시간). 값 선택 = 파라미터 결정이므로 사용자 결정.
- Stage 3에서는 B2와 B3-O가 **같은 설정**을 써야 한다.
- 미검증: 벽 복도에서 적합한 모델이 공장의 **바닥 페인트 통로**(LiDAR가 못 보는 경계)에도 맞는지 → Stage 3-A에서 확인.

## 3. 준비만 된 것 (실행 안 함)

| 항목 | 파일 | 상태 |
|---|---|---|
| R* 선택 규칙 | `comfort_core/selection.py` | R_nav_safe < R_comfort_min → PASSAGE_INFEASIBLE; else R* = min(R_pref, R_nav_safe), fallback R_comfort_min |
| B3-O 선택 정책 | `comfort_core/selection_policy.py` (`b3o_selection`) | spot별 R_nav_safe 표 사용 |
| 동결 값 | `configs/frozen/comfort_requirements.yaml` | **전부 null** — null이면 정책과 `stage3_run.sh --main`이 실행 거부 |
| Stage 2 조건/순서 | `scripts/stage2_make_conditions.py`, `configs/stage2/stage2_real.yaml` | context × 거리, Williams 순서. `distance_levels_m: null` (실기 보호필드 확인 전 값 없음) |
| Failure class | `comfort_core/failure_class.py`, `scripts/nav2_goal.py` `verdict_*` 컬럼 | smoke로 기록 확인 (`261114_recorder_smoke`) |
| Stage 3-A / 3-B | `scripts/stage3a_plan.py`, `scripts/stage3_run.sh --main`, `scripts/stage3b_report.py` | 3-B: seed 1–100 짝, McNemar + Wilcoxon |
| cross seed 34–100 | `scripts/make_cross_campaign.py`, `configs/campaigns/cross/`, `configs/scenarios/`, `results/scenarios/` | `--verify`로 seed 1–33 byte 동일 재현 확인 |
| Preliminary 그림 | `results/batches/261112_stage1_size_sweep/figures/PRELIM_composite.png` | 330 trial sweep은 pilot 증거만 |

## 4. 배치 폴더 지도

| 폴더 | 내용 | trial |
|---|---|---|
| `results/batches/261114_stage1a_smoke` | 경로 경계에서 첫 smoke (ProtectiveStop 발견) | 소수 |
| `results/batches/261114_stage1a_explore` | 원인 탐색 | 소수 |
| `results/batches/261114_recorder_smoke` | `verdict_*` 기록 확인 | 소수 |
| `results/batches/261114_stage1a` | 현행 설정: step0, scan, refine, 1a2_scan, diag_nobuffer | `REPORT.md` |
| `results/batches/261114_stage1a_nobuf` | 후보 설정: scan, refine, 1a2, 1b, lowe | 121 |
| `results/batches/261114_diag_nobuffer_cross` | cross seed 1–5, buffer 0 | 10 |

각 하위 폴더의 `trials.csv`가 원자료 (한 행 = 한 trial; 컬럼 정의는 `comfort_core/metrics.py` `FIELDS`).
조건 → 크기 파싱은 `comfort_core/size_sweep.py` `parse_condition`, 환경은 `configs/stage1a/envs/`.

## 5. 검증
오프라인 체크 모두 통과: `scripts/check_stage1a.py` 463, `check_stage1` 1042, `check_comfort_shapes` 167, `make_cross_campaign.py --verify`.

## 6. 하지 않은 것 / 주의
- 실제 사람 실험, 실기 AMR 실행 없음. Stage 2 값과 결과를 만들어내지 않았다.
- 기존 결과·config는 수정하지 않았다. 예외: 기존 배치 그림 7장(P10–P12)은 AMR 표기(AMR 1/2/3)만 바꿔 다시 그렸다. 데이터는 동일하다.
- 현행 설정 모델(`model_exec.json`)은 적합하지 않았다. 잘못된 모델을 사전등록하지 않기 위해서다.
