# Nav2 Integration

ROS 2 Jazzy, Nav2 1.3.13. Brought up in PHASE D.

## 1. 실행

터미널 세 개입니다.

```bash
# 1. 시뮬레이션
./scripts/run_sim.sh

# 2. Nav2
./scripts/run_nav2.sh

# 3. 한 번의 trial
./scripts/run_nav2_goal.sh
```

전부 멈추려면 `./scripts/stop_all.sh`입니다.

config를 바꾸려면 세 스크립트 모두에 같은 `--config`를 주어야 합니다.

```bash
./scripts/run_sim.sh       --config configs/demo_weave.yaml
./scripts/run_nav2.sh      --config configs/demo_weave.yaml
./scripts/run_nav2_goal.sh --config configs/demo_weave.yaml
```

**trial마다 시뮬레이션을 새로 띄우세요.** 로봇이 장애물에 눌린 상태로 다음
trial을 시작하면 결과가 무의미해집니다. 실제로 그렇게 얻은 DEADLOCK 판정을
한 번 폐기했습니다.

## 2. 구성

```
Isaac Sim ──/lidar/cloud──> pointcloud_to_laserscan ──/scan──┐
          ──/odom, /tf, /clock──────────────────────────────┤
                                                             v
                                         global_costmap / local_costmap
                                                             │
   map_server ──/map──> static_layer ────────────────────────┤
                                                             v
                              planner_server (NavFn) ──> controller_server (DWB)
                                                             │
                                                        /cmd_vel (Twist)
                                                             v
                                                      Isaac Sim ActionGraph
```

`scripts/nav2_bringup.launch.py`가 띄우는 노드는 map_server, planner_server,
controller_server, behavior_server, bt_navigator, lifecycle_manager와
pointcloud_to_laserscan, static_transform_publisher입니다.

nav2_bringup의 `navigation_launch.py`를 쓰지 않았습니다. 그쪽은 smoother,
route, waypoint follower, collision monitor, docking까지 함께 띄우는데,
전부 파라미터를 맞춰줘야 lifecycle이 활성화됩니다. 이 연구가 측정하는 것과
무관한 노드입니다.

## 3. 설계 결정

### AMCL을 쓰지 않습니다

시뮬레이션은 로봇이 어디서 출발했는지 정확히 압니다.
`map`에서 `odom`으로 가는 정적 변환 하나면 충분하고, 값은
`navigation.start`와 `amr.spawn.yaw_deg`에서 나옵니다.

AMCL을 쓰면 위치추정 오차가 결과에 섞입니다.
B2와 B3의 차이가 Comfort Space 정책 하나여야 하는데 교란변수가 생깁니다.

### 지도를 SLAM으로 만들지 않습니다

`scripts/generate_map.py`가 `configs/common.yaml`에서 직접 PGM을 생성합니다.
복도 치수를 이미 정확히 알고 있고, B2와 B3가 바이트 단위로 같은 지도를 쓴다는
것이 보장됩니다.

지도에는 벽과 스테이션만 넣습니다. 사람은 넣지 않습니다.

### MPPI 대신 DWB

Jazzy의 Nav2 기본 controller는 MPPI입니다. MPPI는 매 주기 샘플링 노이즈를
새로 뽑기 때문에 같은 시나리오를 두 번 돌리면 결과가 다릅니다.
B2와 B3를 비교하는 실험에서 확률적 controller는 교란입니다.
DWB는 같은 costmap이면 같은 결과를 냅니다.

### 파라미터를 생성합니다

`configs/nav2/nav2_params.template.yaml`이 원본이고,
`scripts/generate_nav2_params.py`가 `configs/common.yaml`에서
footprint, 속도 상한, goal tolerance, inflation을 채워
`configs/nav2/nav2_params.yaml`을 씁니다. 실행할 때마다 다시 만듭니다.

손으로 두 벌을 유지하면 planner가 믿는 로봇 크기와 통과 판정이 쓰는 로봇
크기가 어긋납니다.

## 4. Humble에서 Jazzy로 오면서 고친 것

reference 저장소의 `amr_nav2_params.yaml`은 Humble 기준입니다.

| 항목 | 조치 |
|---|---|
| `amcl_map_client`, `*_rclcpp_node` 5개 섹션 | 삭제. 구버전 잔재로 아무 노드도 읽지 않음 |
| `bt_navigator.plugin_lib_names` | 삭제. Jazzy 기본값 사용 |
| obstacle layer 2개가 같은 `/scan` 사용 | 1개로 통합. 이중 마킹이 비용을 왜곡 |
| `robot_base_frame` | `base_footprint`로 통일 |
| cmd_vel 메시지 타입 | `enable_stamped_cmd_vel: false`. USD가 `Twist`를 구독 |

## 5. 튜닝 이력

| 파라미터 | 처음 | 지금 | 이유 |
|---|---|---|---|
| `inflation_radius` | 0.44 | 0.34 | `comfort_core.layout`의 통과 여유와 같은 값. 처음엔 margin을 두 번 더해서 통과 판정이 열렸다고 한 틈을 Nav2가 거부했다 |
| `cost_scaling_factor` | 3.0 | 5.0 | 비용이 거리에 따라 빨리 줄어야 좁은 틈 가운데에 저비용 통로가 남는다 |
| `BaseObstacle.scale` | 0.02 | 0.2 | 기본값은 사실상 무시 수준이라 로봇이 경로가 스치는 쪽으로 붙었다가 빠져나오지 못했다 |

## 6. 검증 결과

| 시나리오 | 결과 |
|---|---|
| `configs/common.yaml` 직선 주행 | SUCCESS, 25.5 s, 9.67 m |
| `configs/demo_weave.yaml` S자 회피 | SUCCESS, 27.5 s, 10.6 m (2회 재현) |
| `configs/demo_blocked.yaml` 봉쇄 | TIMEOUT. PLANNING_FAILURE로 나와야 함. 8장 참고 |

## 7. RTX LiDAR가 부분 스캔만 발행하던 문제

**증상.** AMR이 두 번째 사람을 피하지 못하고 그 앞에서 멈췄습니다.
회피가 될 때도 있고 안 될 때도 있었습니다.

**원인.** `ROS2RtxLidarHelper`의 `inputs:fullScan`이 기본값 False입니다.
이 상태에서는 렌더링된 프레임 동안 센서가 훑은 만큼만 발행합니다.
10 Hz로 도는 로터리 LiDAR를 60 Hz로 렌더링하면 한 메시지에
한 바퀴의 6분의 1만 담깁니다.

`pointcloud_to_laserscan`은 그 조각을 온전한 LaserScan으로 바꾸면서
나머지 6분의 5를 무한대로 채웁니다. Nav2의 obstacle layer는 무한대 반환을
"최대 거리까지 아무것도 없음"으로 해석하고 그 광선 위의 셀을 전부 지웁니다.
costmap이 장애물을 찍는 속도보다 지우는 속도가 빨라집니다.

**측정값.** 사람이 정면 1.34 m 앞에 서 있는 상태에서:

```
723빔 중 133빔만 유효, 전부 2.40~2.95 m
그 사람은 global costmap에 아예 없음
Nav2가 사람을 관통하는 경로를 생성
```

**조치.** `isaac_sim/utils/scene_setup.py`의 `patch_lidar_graph()`가
씬 구성 시 `fullScan`을 True로 켭니다. 원본 USD는 그대로 둡니다.

**수정 후.**

```
723빔 중 688빔 유효, 사람 표면에 83빔 명중
S자 회피 2회 연속 SUCCESS, 27.5초로 동일
수정 전에는 39.9초에 도중 정체, 또는 실패
```

## 8. 남은 문제: LiDAR costmap으로는 봉쇄를 판정할 수 없습니다

`demo_blocked.yaml`은 복도에 0.05 m 틈만 남기는 배치입니다.
`comfort_core.layout`의 기하 계산은 BLOCKED로 나옵니다.
그런데 Nav2는 경로를 찾아냅니다.

7장의 fullScan 수정 뒤에도 남아 있습니다.
costmap이 채워질 시간을 6 시뮬레이션 초 주고, 로봇의 현재 위치가 아니라
`navigation.start`에서 goal까지 물어봐도 planner는 여전히 경로를 찾아냅니다.

원인은 두 가지입니다.

**가림(occlusion).** LiDAR는 사람의 앞면만 봅니다. 뒷면과 그 너머는
관측되지 않습니다. static map이 그 자리를 자유 공간으로 선언해 두었으므로
obstacle layer가 찍지 않은 셀은 자유로 남습니다.
`track_unknown_space`를 켜도 소용이 없습니다. static layer가 이미
모든 셀을 known free로 만들어 놓기 때문입니다.

**NavFn은 footprint를 검사하지 않습니다.** 셀 단위로 치명이 아니기만 하면
지나갑니다. 몇 셀짜리 틈은 폭 0.48 m 로봇이 통과할 수 없지만 경로는
만들어집니다.

### 이것이 연구에 주는 결론

**Comfort Space를 LiDAR 관측 경로로 넣으면 안 됩니다.**
사람 뒤쪽 반원이 costmap에 들어가지 않아 반지름이 실효를 잃습니다.

Comfort Space는 사람 위치를 알고 원 전체를 해석적으로 그려 넣어야 합니다.
`nav2_costmap_2d::KeepoutFilter`에 매 주기 마스크를 발행하거나
커스텀 costmap layer를 쓰는 방식입니다. STEP 8에서 그렇게 구현합니다.

LiDAR obstacle layer는 그대로 둡니다. 모델링하지 않은 장애물을 잡아주는
역할은 유효합니다. 다만 Comfort Space의 근거는 되지 못합니다.

### 부수적으로 정해야 할 것

`planner_server`의 NavFn이 footprint를 보지 않으므로,
Comfort Space를 치명 비용으로 칠할 때 반지름에 로봇의 내접 반지름
0.24 m를 더할지 말지 정해야 합니다.
`docs/EXPERIMENT.md`에 반지름의 정의를 명시하세요.
