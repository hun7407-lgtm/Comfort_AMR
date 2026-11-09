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

## 점검 (커밋 전, `docs/COMMIT_STYLE.md` 7절)

| 시뮬 없이 | 띄운 채로 |
|---|---|
| `check_nodes.py`, `check_evasion.py`, `check_perception.py`, `check_comfort_shapes.py`, `check_nav2_tuning.py`, `check_fleet_loop.py`, `check_environment.sh` | `check_fleet_nav2.py`, `check_fleet_stacks.py`, `check_dwb_scores.py`, `check_ros_topics.sh` |

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
