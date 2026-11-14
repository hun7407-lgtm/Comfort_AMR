# 결과 요약 — 2026-11-14 저녁 (Stage 1 최종 + 내비게이션 동결 + Stage 2/3 준비 완료)

이 문서는 [`RESULTS_261114.md`](RESULTS_261114.md)(같은 날 오전 결과, buffer 미결정) **다음** 진입점이다.
오전 문서는 그대로 두었다. 숫자는 모두 아래 링크된 원본(`REPORT.md`, `SELECTION.md`, JSON)에서 나온 것이다.

## 0. 한 줄 요약

- Comfort costmap buffer 후보 **0.10 / 0.15 m**를 실행 전에 사전등록했다(`41ca38d`). 두 후보를 각각 Stage 1 전체(136 trial)로 돌렸다.
- **0.10 m가 기준 A–D를 모두 통과해 선택**됐다. 0.15 m는 B(상한 경계)와 D(경계 이하 ProtectiveStop)에서 탈락했다.
- 내비게이션 요구사항을 [`configs/frozen/navigation_requirements.yaml`](../configs/frozen/navigation_requirements.yaml)에 **동결**했다. B2와 B3-O는 같은 buffer, 같은 물리 안전층을 쓴다.
- Stage 2(실험자 실기) 도구는 준비 완료, **실행 안 함**. Stage 3(B2 vs B3-O 200 trial)은 **잠금 상태**다. comfort 값이 없어서 main run이 거부된다.
- 논문/발표용 캡처: [`results/captures/paper_261114/CAPTURE_INDEX.md`](../results/captures/paper_261114/CAPTURE_INDEX.md).

## 1. 최종 buffer

| 후보 | buffer (m) | A 환경 경계 | B 상한 경계 | C low-e | D ProtectiveStop | 결과 |
|---|---|---|---|---|---|---|
| buf10 | 0.10 | pass | pass (9/9 셀) | pass (18/18) | pass (경계 이하 0건) | **선택** |
| buf15 | 0.15 | pass | FAIL | pass | FAIL | 탈락 |

- 규칙과 기준: [`configs/stage1a/buffer_selection.yaml`](../configs/stage1a/buffer_selection.yaml). 실행 전에 커밋했고 결과를 본 뒤 바꾸지 않았다.
- 판정: [`results/batches/261114_stage1a_final/SELECTION.md`](../results/batches/261114_stage1a_final/SELECTION.md).
- buf15 탈락 이유:
  - B: 1-B `W 2.0 Static` 셀에서 모델 + 0.10 위인 e 0.96이 통과했다. 상한 예측이 틀린 것이다.
  - D: 1A-1 `W 1.5`에서 e 0.42(경계 이하)에 ProtectiveStop이 1건 있었다.
- 사전등록대로, 두 후보가 모두 통과했을 때만 작은 값을 고른다. 이번에는 하나만 통과했으므로 바로 결정됐다. 추가 값은 시험하지 않았다.

## 2. Stage 1 최종 (buffer 0.10, `results/batches/261114_stage1a_buf10`)

- **136 trial**:
  - 1A-1: scan 27 + refine 28
  - 1A-2: 27
  - 1-B: 36
  - low-e: 18
- 결과: SUCCESS 83, DEADLOCK 53. DEADLOCK 내역은 stall 38, stall + ProtectiveStop 13, comfort_gate_hold 2이며, 모두 경계 **위**의 e에서 나왔다.
- **1A-1** (W에 따라 경계가 e 축에서 단조 증가):

| W (m) | 과반 통과 마지막 e | 다음 e (실패) | R_path^infl | R_path^lethal |
|---|---|---|---|---|
| 1.5 | 0.42 | 0.47 | 0.524 | 0.613 |
| 1.8 | 0.72 | 0.77 | 0.823 | 0.914 |
| 2.2 | 1.02 | 1.07 | 1.225 | 1.313 |
| 2.6 | 1.42 | 1.47 | 1.624 | 1.714 |

- **실행 모델:**
  - 벽 clearance 0.49 m. 폭 사이 spread는 0.10 m다.
  - 1A-1에서만 적합했다: [`configs/stage1a/model_exec_buf10.json`](../configs/stage1a/model_exec_buf10.json).
- **1A-2**: 3개 context(C1, C2, C3) 모두 R_exec_model 1.014. 관측 1.01 통과, 1.11 실패로 예측과 일치했다.
- **1-B**: 6개 셀 모두 사전등록 예측과 일치했다.
- **상한 경계**: 사전등록 9셀 중 **9/9** 일치. 모델 지점은 통과하고, 모델 + 0.10 위는 실패했다.
- **low-e**: W 1.5/1.8/2.2/2.6 Static e 0.30, W 2.0 Static e 0.81, W 2.2 B3-O C1 e 0.81 × 3회 = **18/18 통과**. 오전 buffer 0에서 나왔던 gate hold가 재현되지 않았다.
- **ProtectiveStop**:
  - 경계 이하: **0건**.
  - 경계 위: 13건. 경로 한계 위에서 안전층이 멈춘 것으로, 의도된 동작이다.
- 원본: [`REPORT.md`](../results/batches/261114_stage1a_buf10/REPORT.md), `figures/F1–F3`, R_nav_safe 표 [`r_nav_safe.json`](../results/batches/261114_stage1a_final/r_nav_safe.json).

### 주의: R_nav_safe(= R_path^infl)와 실행 경계의 차이

정의대로 **R_nav_safe := R_path^infl**(Conservative Navigation Feasibility Bound)로 동결했다.

그런데 이 스택에서 관측한 R_exec는 모든 셀에서 R_path^infl보다 **0.10–0.22 m 작다**. REPORT.md의 `R_path^infl - R_exec` 열이다. 즉 R_path^infl은 경로 기하로는 보수적이지만, 실제 실행 경계에 대해서는 보수적이지 않다.

동결 파일에는 실행 모델 경계(`execution_model_bound: exec`)도 함께 기록했다. Stage 3 선택에 어느 경계를 쓸지(정의대로 infl, 또는 관측에 맞는 exec)는 **사용자 결정 사항**이다. 현재 설정은 정의대로 `bound: infl`이다.

## 3. 동결된 내비게이션 파라미터

[`configs/frozen/navigation_requirements.yaml`](../configs/frozen/navigation_requirements.yaml) — `scripts/stage1a_finalize.py --freeze`가 한 번만 쓴다. 다시 쓰려고 하면 거부한다.

| 항목 | 값 |
|---|---|
| comfort costmap buffer | 0.10 m |
| 실행 모델 | `stage1a_buf10_v1`, wall clearance 0.49 m, `model_exec_buf10.json` (sha256 기록) |
| footprint | 0.70 × 0.48 m, inscribed (robot_grow) 0.24 m, inflation 0.34 m |
| gate | margin 0.05 m, lookahead 0.6 s |
| 물리 안전층 | collision monitor (stop margin 0.25, slowdown on) — 모든 정책 동일, comfort에 따라 축소하지 않음 |
| R_nav_safe | `R_path^infl`, `comfort_core.feasibility.r_path(cfg, kind, human, 'infl')` |
| 출처 | batch `261114_stage1a_buf10`, 판정 `261114_stage1a_final`, 커밋 `41ca38d` |
| 적용 | `b2_static_calibrated`, `b3o_selection` (둘 다 `costmap.buffer_width: 0.1`) |

Stage 3는 이 파일을 읽기만 한다:
- `stage3_make_configs.py --frozen`은 모델이나 buffer가 다르면 거부한다.
- `stage3_run.sh --main`은 이 파일이 없거나 buffer가 다르면 거부한다.

## 4. Stage 2 준비 상태 (실행 안 함)

| 구성 | 상태 | 파일 |
|---|---|---|
| 조건 생성 | READY | `scripts/stage2_make_conditions.py` (`distance_levels_m` null이면 거부) |
| counterbalancing | READY | `comfort_core.stage2.participant_orders` (Williams, 익명 ID `^P[0-9]{2,3}$`) |
| trial recorder | READY | `scripts/stage2_trial.py --plan orders.csv` |
| 설문 | READY (문항 문구는 TODO) | `stage2_real.yaml` `questionnaire` v0-draft. 보조 문항 문구는 HRI 문헌 대조 TODO |
| 거리 기록 | READY | 사람 중심 → AMR 표면 최소거리. 중심-중심은 raw 추가 열 |
| 중단/안전 기록 | READY (E-stop 토픽 TODO) | 상태 6종. PARTICIPANT_STOP은 실패로 세지 않음. `estop_topic: null` (실기 드라이버 확인 전엔 실험자 수기 입력) |
| 분석 | READY | `scripts/stage2_analysis.py` (context별 곡선, 거리별 평균/중앙값, 참가자별, R_comfort_min/R_pref 후보 — 규칙이 null이면 계산 안 함) |

검증: `scripts/check_stage2.py` 159/159.

**의도적으로 null로 둔 값:**
- `stage2.distance_levels_m`
- `configs/frozen/comfort_requirements.yaml`의 `r_comfort_min_m`, `r_pref_m` 전부

이 값들은 사람 데이터 없이는 채우지 않는다.

**Human Study는 실행하지 않았다.** IRB 결과도 가정하지 않았다.

## 5. Stage 3 준비 상태 (실행 안 함)

| 구성 | 상태 |
|---|---|
| 선택 규칙 단위 테스트 | Case A (R* = R_pref), B (R* = R_nav_safe), C (PASSAGE_INFEASIBLE), fallback (R* = R_comfort_min). fixture 값만 사용 — `check_stage3_interface.py` |
| paired seed | 1–100 config가 `render(s)`와 일치, B2/B3-O는 comfort_space만 다르고 collision monitor 동일. cross 1–33 byte-identical (`make_cross_campaign.py --verify`) |
| 실패 분류 | SUCCESS / PASSAGE_INFEASIBLE / COMFORT_GATE_HOLD / POST_EVADE_STALL / NAV2_FAILURE / ROBOT_ROBOT_DEADLOCK / PROTECTIVE_STOP / INVALID. 코드 이름 대응표는 `docs/STAGES.md` "Failure verdict 대응표" |
| main run 잠금 | `stage3_run.sh --main` → `comfort_requirements.yaml: not frozen yet` 로 거부 (확인함) |

검증: `check_stage3_interface.py` 445/445.

참고: Stage 1 행에는 selection 필드가 없다. 그래서 분류기는 Stage 1의 `comfort_gate_hold`를 UNCLASSIFIED로 둔다. 이는 Stage 3 기준의 동작이며, 분류기는 바꾸지 않았다.

## 6. 남은 사용자 작업

1. 실기 AMR에서 보호필드와 통과 속도를 확인한다. 사용할 수 있는 passing-distance 범위를 정해 `distance_levels_m`을 확정한다.
2. 설문 보조 문항 문구를 확정한다(`questionnaire.version` 올리기). 실기 E-stop 토픽이 있으면 `safety.estop_topic`에 넣는다.
3. IRB와 참가자 모집을 거쳐 Human Study를 진행한다.
4. Stage 2 결과로 `comfort_requirements.yaml`을 채우고 동결한다.
5. Stage 3에서 R_nav_safe를 정의대로 infl로 쓸지, 실행 모델(exec)로 쓸지 결정한다(§2 주의).
6. Stage 3 main run을 실행한다.

## 7. 새 결과 폴더

| 폴더 | 내용 |
|---|---|
| `results/batches/261114_stage1a_buf10` | 후보 0.10: 136 trial, REPORT.md, figures |
| `results/batches/261114_stage1a_buf15` | 후보 0.15: 136 trial, REPORT.md, figures |
| `results/batches/261114_stage1a_final` | 선택 결과(`SELECTION.md`, `buffer_selection.json`), R_nav_safe 표. 새 trial 없음 |
| `results/captures/paper_261114` | 논문/발표용 캡처 (측정 아님), `CAPTURE_INDEX.md` |

## 8. 논문/발표용 캡처 (측정 아님)

[`results/captures/paper_261114/CAPTURE_INDEX.md`](../results/captures/paper_261114/CAPTURE_INDEX.md) — 이미지 283장.

| 종류 | 수 |
|---|---|
| clean | 80 |
| overlay | 88 |
| annotated | 34 |
| debug (costmap) | 46 |
| composite | 34 |
| placeholder | 1 |

- 같은 비교군은 같은 카메라·줌·해상도(1920×1080)로 찍었다. 표시 이름은 AMR 1/2/3이다.
- 캡처 실행은 Stage 1 6회(최종 buffer 0.10, buffer 비교 0/0.10/0.35)와 cross 4회다. trial 기록은 `capture_trials/`에 있고, results batch가 아니다.
- cross seed 2/3의 B2-vs-B3-O 비교는 **diagnostic pilot**(buffer 0)이다. 결과 파일에서 seed를 먼저 확인했다(`261114_diag_nobuffer_cross`: B2 hold, B3-O SUCCESS). Stage 3 main 결과가 아니다.
- 실패 갤러리: SUCCESS, COMFORT_GATE_HOLD, PROTECTIVE_STOP, POST_EVADE_STALL, NAV2_FAILURE는 기존 기록에서 가져왔다. ROBOT_ROBOT_DEADLOCK은 **NO EXISTING EXAMPLE**이다. results/ 아래 어떤 기록도 이 판정이 아니다.
- buffer 0, W 2.0, e 0.81은 첫 캡처 실행에서 gate hold, 재촬영에서 통과였다. buffer 0의 gate 경계 스침이 실행마다 다르기 때문이다. 각 이미지에는 그 실행의 판정을 표기했다.
- PNG와 동영상은 기존 `.gitignore` 규칙(`results/**/*.png`, `*.mp4`)에 따라 로컬에만 있다. 재생성 방법은 인덱스 상단에 있다.

기존 폴더(`261112_stage1_size_sweep`, `261114_stage1a`, `261114_stage1a_nobuf`, `261114_diag_nobuffer_cross`)는 수정하지 않았다.

> 모든 경계값은 이 내비게이션 스택의 공학적 경계이며 comfort distance가 아니다.
