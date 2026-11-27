# AGENTS.md — AI 에이전트용 진입점

이 저장소를 처음 읽는 AI 에이전트(및 사람)를 위한 안내입니다. **먼저 이 파일을 읽고**, 아래 "읽는 순서"를 따르세요.
작성 기준일: 2026-11-26 (branch `feature/b1-baseline-and-yield`).

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

## 2. 현재 상태 (2026-11-26 14:10 기준 — 최종 실험 완료)

최종 실험 = `docs/FINAL_261125_PLAN.md`. **사용자 결정으로 조건당 100 → 30 seed로 축소** (Stage 1 seed 1–30, Stage 3 holdout seed 101–130). 진행 기록과 모든 예외는 plan §8.

| 단계 | 상태 | 결과 위치 |
|---|---|---|
| Stage 1 coarse + fine (B1 + B2 0.80–1.60, 270 trial) | 완료 | `results/batches/261125_final_stage1/analysis/STAGE1_REPORT.md`, `stage1_upper_bound.json` |
| Stage 2 Simulation-derived Provisional Lower Bound | 완료 | `results/batches/261125_final_stage2_proxy/STAGE2_PROXY.md` |
| Stage 3 B1/B2/B3-O MID + sensitivity LOWER/UPPER (150 trial) | 완료 | `results/batches/261125_final_stage3/analysis/STAGE3_REPORT.md`, `stage3_selection.json` |
| 영상 | 완료 (mp4는 실험 PC에만) | 각 stage `videos/**/VIDEO_INDEX.md`, `video_metadata.json` |

**Stage 1 (seed 1–30): R_nav_upper = 0.90 m**, transition 0.95 m. 규칙: comfort-induced deadlock ≤ 5 % (30 seed에서 ≤ 1/30) 그리고 system success ≥ 같은 seed B1 − 5 %p.

| 조건 | n | system success | comfort-induced deadlock |
|---|---|---|---|
| B1 | 30 | 20/30 | 0/30 |
| B2 0.80 | 29 | 27/29 | 1/29 |
| B2 0.85 | 30 | 26/30 | 1/30 |
| B2 0.90 | 30 | 28/30 | 1/30 |
| B2 0.95 | 30 | 26/30 | 2/30 |
| B2 1.00 | 30 | 24/30 | 4/30 |
| B2 1.20 | 30 | 21/30 | 6/30 |
| B2 1.40 | 30 | 10/30 | 16/30 |
| B2 1.60 | 30 | 0/30 | 29/30 |

**Stage 2:** moving|front 0.469, moving|side 0.369 m. stationary|side, station_engaged|side/rear는 R_nav_upper보다 커서 0.90으로 잘림; 나머지 4칸 INSUFFICIENT (N < 20).
→ Stage 3에서 B2 = 0.90, B3-O MID = 정지·작업 중 사람 0.90 / 걷는 사람 0.684·0.635·0.684, LOWER 걷는 사람 0.469·0.369·0.469, **UPPER = B2와 동일**.

**Stage 3 (seed 101–130):**

| Policy | system success | comfort deadlock | protective-stop trial | hold s (median) |
|---|---|---|---|---|
| B1 | 24/30 | 0/30 | 21/30 | 0.83 |
| B2 (0.90) | 22/30 | 1/30 | 8/30 | 2.2 |
| B3-O MID | 23/30 | 1/30 | 10/30 | 1.74 |
| B3-O LOWER | 28/30 | 0/30 | 12/30 | 0.62 |
| B3-O UPPER (= B2 설정) | 25/30 | 2/30 | 9/30 | 3.33 |

B2 vs B3-O MID McNemar 4 vs 5, p = 1.0 → **차이 없음**.

알려진 사항 (결과 해석 시 반드시 고려):
- **재현성:** UPPER와 B2는 같은 seed·같은 설정인데 30 seed 중 11개에서 성공/실패가 다름 → 같은 seed라도 결과가 반복되지 않음 (ROS·렌더 타이밍). n = 30에서 ±3–4 성공 차이는 노이즈 수준. LOWER 28/30도 이 범위를 감안해 읽을 것.
- penetration / intrusion은 각 정책 자신의 반경 기준 → 반경이 작으면 정의상 작게 나옴. 사람을 더 멀리 두었다는 증거가 아님.
- min human–AMR distance(≈0.22–0.24 m)는 걸어가는 사람이 AMR 옆을 스치는 경우가 지배 → 모든 정책 동일, Comfort Space 비교 지표로 부적합.
- Comfort Space가 있으면 protective stop(물리 안전층 정지)이 있는 trial이 크게 줄어듦 (B1 21/30 vs B2 8/30, B3-O 10/30). B1 실패 원인은 `NAV2_ABORT`·`HUMAN_MOTION_BLOCK` 등 Comfort Space와 무관한 것.
- Stage 2 proxy는 "안전하게 성공했을 때 실제로 유지된 거리의 Q10"이라 정지·작업 중 사람은 통로 구조를 반영 (R_nav_upper 초과) → B3-O가 B2와 다른 곳은 걷는 사람뿐. 규칙은 데이터를 본 뒤 바꾸지 않음.
- `s022_B2_r080`은 3회 모두 INVALID (사람–AMR 접촉 시 물리 발산, 재현됨) → n = 29.
- R_nav_upper를 "급증 지점"으로 정할지 논의됨 → 사전등록 규칙 유지. 로지스틱 R₅%/R₅₀% 보조 분석은 아직 안 함.
- Stage 3 영상 case A(B2 comfort deadlock & B3-O 성공)는 발생하지 않아 만들지 않음.

**Stage 2 재설계 (2026-11-26):** 상태별 Comfort Space 모양을 정규화하고 α 하나로 전체 크기를 줄이는 uniform-scale 실험.
설계 `docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md`, v1(gate 체인) smoke `docs/STAGE2_SCALE_IMPLEMENTATION.md`.

**Nav2-only 본실험 (2026-11-26 밤 → 11-27, 약식 완료):** 결과 `docs/NAV2ONLY_STAGE2_RESULTS.md`, 기록·사전등록 규칙 `docs/NAV2ONLY_RUN_LOG.md`.
`control.mode: nav2_only` → comfort_gate 미실행(yield / evade / backoff / off-lane 없음), 모든 AMR controller → collision_monitor, 실행 중 ROS graph 검사.
- Stage 1 재검증(seed 1–10): **R_nav_upper(nav2-only) = 0.95 m**, transition 1.00 → Stage 2 R_ref = 0.95.
- Stage 2(pilot 110 → coarse 180 → confirm 100): station_engaged **α\* = 0.60 식별·재현** (EN-R 2.2 m; F/S/R 0.57 / 0.50 / 0.43 m);
  stationary **UNIDENTIFIABLE** (ST-F 환경 고정 실패, 옆·뒤는 α 0.45까지 통과 = 하한 아님); moving **UNIDENTIFIABLE** (거리 반응 0.074 m). Stage 3 미실행.
- B2·S3 318 launch 접촉 0, 접촉은 B1에서만. 2.2 m에서 B2 0.95는 confirm 10/10 교착.

## 3. 읽는 순서

1. 이 파일
2. `docs/FINAL_261125_PLAN.md` — 최종 실험 설계, 고정 조건, Stage별 판정 규칙, 진행 기록(§8)
3. `results/batches/261125_final_stage1/analysis/STAGE1_REPORT.md`, `results/batches/261125_final_stage2_proxy/STAGE2_PROXY.md`, `results/batches/261125_final_stage3/analysis/STAGE3_REPORT.md` — 결과
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
- **커밋되지 않은 것** (실험 PC에만 있음): `trial_log.jsonl`, `*_frames.jsonl`(원본 스트림, Stage 1 coarse 180 trial만 184 MB), `*.mp4`, `logs/`, `*.log`.
- 이전 배치 목록: `results/batches/README.md`.

## 6. 반드시 지킬 규칙 (사용자 지시)

- **용어:** "experimentally validated comfort distance", "ISO comfort distance"라고 쓰지 않습니다. `R_nav_max`라는 표현을 쓰지 않습니다 (`R_nav_upper`). Stage 2 값은 "Simulation-derived Provisional Lower Bound"이며 human comfort distance가 아닙니다.
- **Physical safety layer**(collision_monitor 등)는 Comfort Space 최적화 때문에 줄이거나 끄지 않습니다.
- Custom planner / controller / MPPI critic 개발, DWB·MPPI·Nav2 내부 로직 수정 금지.
- 실제 장비 정보가 없는 값은 만들지 말고 TODO로 남깁니다. 실험 데이터 없이 결과를 만들지 않습니다.
- 기존 실험 결과를 덮어쓰거나 지우지 않습니다 (`261112_*`, `261114_*` 등).
- 같은 seed 안에서는 Comfort Space만 바뀝니다. 재녹화 영상이 원 trial과 다른 결과면 그 trial의 증거로 쓰지 않습니다.
- `results/notion/261109/`는 커밋하지 않습니다. 커밋 양식은 `docs/COMMIT_STYLE.md`.
