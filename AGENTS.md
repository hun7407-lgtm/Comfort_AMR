# AGENTS.md — AI 에이전트용 진입점

이 저장소를 처음 읽는 AI 에이전트(및 사람)를 위한 안내입니다. **먼저 이 파일을 읽고**, 아래 "읽는 순서"를 따르세요.
작성 기준일: 2026-11-25 (branch `feature/b1-baseline-and-yield`).

---

## 1. 한 줄 요약

Isaac Sim 5.1 + ROS 2 Jazzy + Nav2 디지털 트윈에서, 사람 주변 **Comfort Space**를 Nav2 costmap에 넣었을 때
AMR 교착(deadlock)이 어떻게 생기는지 측정하고, 사람 상태 × 접근 방향에 따라 크기를 바꾸는
**B3-O (State × Direction Adaptive Comfort Space)** 가 고정 크기(B2)보다 나은지 검증하는 연구입니다.

| Policy | 의미 |
|---|---|
| B1 | Comfort Space 없음 (Nav2 + 물리 안전층만) |
| B2 | Static Comfort Space (모든 사람 같은 반경) |
| B3-O | 사람 상태(moving / stationary / station_engaged) × 방향(front / side / rear) 9셀별 반경 — **제안 방법** |

(state-only "B3"는 최종 실험에서 쓰지 않습니다.)

## 2. 현재 상태 (2026-11-25 17:30 기준)

최종 실험 = `docs/FINAL_261125_PLAN.md`. **사용자 결정으로 조건당 100 → 30 seed로 축소** (Stage 1 seed 1–30, Stage 3 holdout seed 101–130).

| 단계 | 상태 | 결과 위치 |
|---|---|---|
| Stage 1 coarse (B1 + B2 0.80–1.60, 180 trial) | **완료** | `results/batches/261125_final_stage1/analysis/STAGE1_REPORT.md` |
| Stage 1 fine (B2 0.85/0.90/0.95, 90 trial) | 진행 중 (11-26 01시경 완료 예상) | `results/batches/261125_final_stage1/fine/` |
| Stage 2 provisional lower bound | 대기 (자동) | `results/batches/261125_final_stage2_proxy/` |
| Stage 3 B1/B2/B3-O + sensitivity (150 trial) | 대기 (자동) | `results/batches/261125_final_stage3/` |
| 리포트 + 영상 | 대기 (자동, `scripts/final_post.sh`) | 각 stage의 `videos/`, `VIDEO_INDEX.md` |

Stage 1 coarse (seed 1–30, 사전등록 규칙 그대로):

| 조건 | n | system success (AMR 3대 모두 성공) | comfort-induced deadlock |
|---|---|---|---|
| B1 | 30 | 20/30 | 0/30 |
| B2 0.80 | 29 | 27/29 | 1/29 |
| B2 1.00 | 30 | 24/30 | 4/30 |
| B2 1.20 | 30 | 21/30 | 6/30 |
| B2 1.40 | 30 | 10/30 | 16/30 |
| B2 1.60 | 30 | 0/30 | 29/30 |

→ transition 1.00 m, R_nav_upper 후보 0.80 m (fine search 후 확정). 규칙: comfort-induced deadlock ≤ 5 % (30 seed에서 ≤ 1/30) **그리고** system success ≥ 같은 seed B1 − 5 %p.

알려진 사항 (보고서에 반영할 것):
- B1이 B2 소형보다 약함 — B1 실패는 대부분 Comfort Space와 무관한 `NAV2_ABORT`.
- `s022_B2_r080`은 3회 모두 INVALID (사람–AMR 접촉 시 물리 발산, 재현됨) → n = 29, `docs/FINAL_261125_PLAN.md` §8에 기록.
- min human–AMR distance(≈0.24 m)는 걸어가는 사람이 AMR 옆을 스치는 경우가 지배 → Comfort Space 비교 지표로 부적합, 상태별로 나눠 보고 예정.
- "처음 5 % 초과" 대신 "급증 지점(1.2→1.4 m)"을 기준으로 할지 논의됨 → **사전등록 규칙 유지**, 로지스틱 R₅%/R₅₀%는 보조 분석으로만 추가 예정.

## 3. 읽는 순서

1. 이 파일
2. `docs/FINAL_261125_PLAN.md` — 최종 실험 설계, 고정 조건, Stage별 판정 규칙, 진행 기록(§8)
3. `results/batches/261125_final_stage1/analysis/STAGE1_REPORT.md` — 현재 결과
4. `comfort_core/final_metrics.py` — trial 하나를 어떻게 판정·측정하는지 (분류 순서, 지표 정의)
5. 배경이 필요하면: `docs/STAGES.md`(개정 설계), `docs/RESULTS_261114_stage1_final.md`(buffer 0.10 동결),
   `configs/frozen/navigation_requirements.yaml`, `docs/NAV2.md`, `docs/SCENARIO.md`, `README.md`

## 4. 코드 지도 (최종 실험 관련)

| 파일 | 역할 |
|---|---|
| `scripts/final_plan.py` | seed별 config(`configs/final/seeds/`), 조건(`configs/final/conditions/`), manifest 생성; `stage3`는 Stage 1/2 결과로 B2·B3-O 크기 계산 |
| `scripts/final_run.py` | manifest를 trial 단위로 실행 (재개 가능, 인프라 실패만 최대 2회 재시도, `--seeds a-b`) |
| `scripts/final_pipeline.py` | Stage 1 coarse → fine → Stage 2 → Stage 3 무인 실행 (`--stage1-seeds 1-30 --stage3-seeds 101-130`) |
| `scripts/final_post.sh` | 파이프라인 종료 후 리포트 + 영상 |
| `scripts/trial_logger.py` | trial 중 5 Hz 기록 (pose, 사람 상태·Comfort Space, plan, 안전층) |
| `comfort_core/final_metrics.py` | trial 분석: 성공/실패 분류, 공간 지표, 상호작용 |
| `scripts/final_stage1_report.py` / `final_stage2_proxy.py` / `final_stage3_report.py` | Stage별 리포트·JSON |
| `scripts/final_videos.py` | trial 원본 녹화 + overlay 영상, `video_metadata.json`, `VIDEO_INDEX.md` |
| `scripts/run_scenario.sh` | trial 하나 실행 (Isaac Sim + Nav2 ×3 + fleet + gate + 녹화) |
| `scripts/comfort_publisher.py` / `comfort_gate.py` | Comfort Space mask 발행 / AMR1 hold gate |

실행 구조: AMR1 = Nav2 controller → comfort_gate(hold만) → collision_monitor → `/cmd_vel`. a1·a2 = 각자 Nav2 스택, 같은 Comfort Space mask를 읽음. 모든 AMR이 costmap → planner → controller를 거침 (Nav2 bypass인 evade / backoff / off-lane recovery는 꺼져 있음).

## 5. 결과 데이터 읽는 법

- trial 폴더: `results/batches/261125_final_stage1/coarse/trials/<trial_id>/` — `DONE`(판정·재시도 기록), `trials.csv`(AMR1), `fleet_trials.csv`(a1, a2), trial JSON.
- 분석값: `results/batches/261125_final_stage1/analysis/trials/<trial_id>.json` — 리포트의 모든 숫자는 여기서 나옵니다.
- **커밋되지 않은 것** (실험 PC에만 있음): `trial_log.jsonl`, `*_frames.jsonl`(원본 스트림, 180 trial에 184 MB), `*.mp4`, `logs/`, `*.log`.
- 이전 배치 목록: `results/batches/README.md`.

## 6. 반드시 지킬 규칙 (사용자 지시)

- **용어:** "experimentally validated comfort distance", "ISO comfort distance"라고 쓰지 않습니다. `R_nav_max`라는 표현을 쓰지 않습니다 (`R_nav_upper`). Stage 2 값은 "Simulation-derived Provisional Lower Bound"이며 human comfort distance가 아닙니다.
- **Physical safety layer**(collision_monitor 등)는 Comfort Space 최적화 때문에 줄이거나 끄지 않습니다.
- Custom planner / controller / MPPI critic 개발, DWB·MPPI·Nav2 내부 로직 수정 금지.
- 실제 장비 정보가 없는 값은 만들지 말고 TODO로 남깁니다. 실험 데이터 없이 결과를 만들지 않습니다.
- 기존 실험 결과를 덮어쓰거나 지우지 않습니다 (`261112_*`, `261114_*` 등).
- 같은 seed 안에서는 Comfort Space만 바뀝니다. 재녹화 영상이 원 trial과 다른 결과면 그 trial의 증거로 쓰지 않습니다.
- `results/notion/261109/`는 커밋하지 않습니다. 커밋 양식은 `docs/COMMIT_STYLE.md`.
