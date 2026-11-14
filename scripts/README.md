# scripts/ — 어떤 스크립트가 무엇을 하는지

파일은 옮기지 않았습니다(문서·셸 스크립트가 서로 경로로 부릅니다). 이름 앞머리가 역할입니다: `run_` 실행, `check_` 점검, `report_`·`*_figures` 결과 정리.

## 한 번 돌리기

| 스크립트 | 하는 일 |
|---|---|
| `run_scenario.sh <b1\|b2\|b3r\|b3\|b3d>` | **trial 한 건 전체** (시뮬 + Nav2 + 판정). `--config`, `--headless`, `--record`, `--fleet-tail` |
| `run_b1.sh`, `run_b2.sh`, `run_b3r.sh`, `run_b3.sh` | 위의 단축 |
| `run_sim.sh`, `run_nav2.sh`, `run_nav2_goal.sh` | 세 조각을 따로 띄울 때 |
| `run_fleet_nav2.sh`, `run_fleet_goals.sh`, `run_traffic_rule.sh` | 다른 AMR의 Nav2 스택, 디스패처, 우측통행 mask |
| `stop_all.sh` | 이 프로젝트가 띄운 것 전부 종료 |
| `watch.sh`, `preview_scenario.sh` | 창으로 에피소드·시나리오 보기 |

## 여러 번 돌리기

| 스크립트 | 하는 일 |
|---|---|
| `run_campaign.sh --manifest … --out …` | **manifest를 무인 실행** (watchdog, 재시도 1회, `--skip`으로 재개) |
| `run_batch.sh` | 같은 trial을 정책·배치별로 반복 (초기 묶음) |
| `stress_wave.sh`, `random_scenario.py` | 무작위 변형 스트레스 |
| `make_scenario.py` | seed에서 논문 시나리오 생성 (`configs/scenarios/`) |

## 결과 정리 (`results/batches/<폴더>`에 대고)

| 스크립트 | 만드는 것 |
|---|---|
| `report_pilot.py --dir` | `tables.md`, 그림 P1–P6 (성공률, 시간 비용, 도착 곡선, 최근접, 플릿, 기하) |
| `pilot_extra_figures.py --dir --trace-seed N` | P7 seed×정책 행렬, P8 궤적 |
| `cross_figures.py --dir` | P9 로봇 간격, P10 세 교차, P11 조건별, `cross_tables.md` |
| `deadlock_mechanism.py --dir` | P12 실패 메커니즘, `deadlock_mechanism.md` |
| `merge_campaigns.py --out … A B` | 묶음 폴더 합치기 |
| `report_campaign.py --dir` | 플릿 패턴 캠페인용 표·그림 F1–F5 |
| `summarise_trials.py`, `triage_trials.py` | CSV 요약, 이상한 trial 골라내기 |
| `scenario_figure.py`, `preview_figure.py` | 시나리오 배치 그림 |

## 개정 실험 (2026-11-14, `docs/STAGES.md` "개정 설계")

| 스크립트 | 역할 |
|---|---|
| `stage1a_plan.py` | Stage 1 controlled: 복도 환경·조건·**사전등록 예측**·매니페스트. `--phase scan` / `--phase predict --model` |
| `stage1a_run.sh` | Stage 1 전체 무인 실행: scan → refine → fit → predict → validate → report (`--from`으로 재개) |
| `stage1a_calibrate.py` | 폭별 관측 경계, refine 매니페스트, 실행 모델 보정(wall clearance) → `configs/stage1a/model_exec.json` |
| `stage1a_report.py` | 예측(경로 한계 / 실행 모델) 대비 관측 경계, F0–F3 |
| `make_cross_campaign.py` | cross campaign config를 seed별로 생성; s1–33 byte 재현 확인(`--verify`) |
| `stage3_make_configs.py --frozen` | `b3o_selection.yaml`(spot별 R_nav_safe) + `b2_static_calibrated.yaml`, 동결된 buffer를 두 정책에 동일 적용 |
| `stage3_run.sh --main` | B2 Static Calibrated vs B3-O Dynamic Selection, seeds 1–100 paired. 두 frozen 파일이 없거나 null이면 거부 |
| `stage3b_report.py` | failure class, exact McNemar, paired Wilcoxon, seed 1–33/34–100 분리 |
| `stage2_make_conditions.py` | (개정) context × passing distance 조건 + `orders.csv`(익명 ID, Williams, `trial_repeats`). `distance_levels_m` null이면 거부 |
| `stage1a_finalize.py [--freeze]` | buffer 후보 판정(사전등록 규칙 `configs/stage1a/buffer_selection.yaml`) → `results/batches/261114_stage1a_final/`, `--freeze`로 `configs/frozen/navigation_requirements.yaml` (한 번만) |
| `stage2_trial.py` | (개정) 실기 통과 1회: 상태(NORMAL/OPERATOR_ABORT/PARTICIPANT_STOP/E_STOP/NAVIGATION_FAILURE/INVALID_TRIAL), 시작·종료 시각, 사람 중심→AMR 표면 최소거리, AMR 속도. `--plan orders.csv` |
| `stage2_questionnaire.py` | (개정) 문항·척도는 config(`stage2.questionnaire`), 코드에 문구 없음 |
| `stage2_mark_trial.py` | trial 상태 사후 수정 → `status_amendments.csv` (trials.csv는 그대로) |
| `stage2_analysis.py` | (개정) context별 comfort 곡선, 거리별 평균/중앙값, 참가자별, R_comfort_min/R_pref **후보표** (확정 안 함) |
| `check_stage2.py` | Stage 2 도구 오프라인 검증 (fixture는 임시 폴더) |
| `check_stage3_interface.py` | 선택 규칙 Case A/B/C, verdict 8종, seed 1–100 짝 무결성, main-run 잠금 |

## 3단계 실험 (2026-11-12부터, `docs/STAGES.md`)

| 스크립트 | 역할 |
|---|---|
| `make_size_conditions.py` | Comfort Space 크기 조건 overlay 생성 (`configs/stage1/static_r*.yaml`, `b3o_r*.yaml`), `--table`로 적용 F/S/R |
| `stage1_sweep.sh` | Stage 1 크기 sweep: 조건 생성 → 검증 → 매니페스트 → `run_campaign.sh` → 리포트. `--dry-run`, `--smoke`, `--sizes`, `--skip` |
| `stage1_manifest.py` | seed-major trial 목록 |
| `stage1_report.py --dir` | R_deadlock_onset / R_nav_max, 표, P1–P9 |
| `check_stage1.py [--dir]` | config / scaling / same-seed / 안전층 불변 / CSV 검증 |
| `stage2_make_conditions.py` | 실기용 context × passing distance 조건 + 참가자별 Williams 순서 (`distance_levels_m`가 null이면 거부) |
| `stage2_trial.sh` / `stage2_trial.py` | 실기 AMR 통과 1회 기록 (placeholder가 남아 있으면 거부) |
| `stage2_questionnaire.py` | 7점 설문 입력 → `questionnaire.csv` |
| `stage2_analysis.py` | comfort–size 곡선, R_comfort_min (cutoff 설정 시에만) |
| `stage3_make_configs.py` | `static_baseline`, `b3o_initial`, `b3o_optimized` (R_opt 없으면 template) |
| `stage3_run.sh` / `stage3_report.py` | Initial vs Optimized (+Static) 33 seeds, paired Δ, 궤적 overlay |

## 점검 (커밋 전, `docs/COMMIT_STYLE.md` 7절)

| 시뮬 없이 | 띄운 채로 |
|---|---|
| `check_nodes.py`, `check_evasion.py`, `check_perception.py`, `check_comfort_shapes.py`, `check_stage1.py`, `check_nav2_tuning.py`, `check_fleet_loop.py`, `check_environment.sh` | `check_fleet_nav2.py`, `check_fleet_stacks.py`, `check_dwb_scores.py`, `check_ros_topics.sh` |

## ROS 2 노드와 생성기 (직접 부르지 않음)

| 스크립트 | 역할 |
|---|---|
| `nav2_goal.py` | trial 실행·판정·기록 (AMR1) |
| `fleet_goals.py` | 플릿 디스패처 + 플릿 측정 |
| `comfort_publisher.py`, `comfort_gate.py`, `traffic_rule.py` | Comfort Space mask, 양보 게이트, 우측통행 mask |
| `human_tracker.py`, `detection_tracker.py`, `people_tracker.py` | 사람 위치: /tf, LiDAR 검출, /scan 추정 |
| `generate_map.py`, `generate_nav2_params.py` | 지도와 Nav2 파라미터 생성 |
| `nav2_bringup.launch.py`, `launch_nodes.py` | Nav2 브링업 |
| `costmap_recorder.py`, `encode_video.sh`, `frames_to_gif.py` | 녹화 |

## 장면·초기 단계

`build_factory_scene.sh`, `view_factory_scene.sh`, `read_factory_scene.sh`, `scene_shots.sh`, `run_mapping.sh`, `run_amr_test.sh`, `run_cmd_vel_test.sh`, `test_cmd_vel.py`, `run_teleop.sh`, `amr_arrow_teleop.py`, `run_people_tracker.sh`, `start_scan.sh`, `start_nav2.sh`(참고용 원본).
