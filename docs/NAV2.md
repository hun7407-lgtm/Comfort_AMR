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

단, **3D 버전은 SLAM으로 만듭니다**(`nav3d/`). 3D LiDAR를 쓰는 쪽에서는
"센서가 실제로 본 것으로 지도를 만든다"가 요점이기 때문입니다. 그쪽도
지도는 **한 번만** 만들어 B1/B2/B3가 같은 파일을 읽습니다 — 위 문단의
"바이트 단위로 같은 지도"라는 조건은 그대로 지킵니다. 자세한 내용은
[../nav3d/README.md](../nav3d/README.md).

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
| `inflation_radius` | 0.44 | 0.34 (유도) | `comfort_core.layout`의 통과 여유와 같은 값. 처음엔 margin을 두 번 더해서 통과 판정이 열렸다고 한 틈을 Nav2가 거부했다 |
| `cost_scaling_factor` | 3.0 | 5.0 | 비용이 거리에 따라 빨리 줄어야 좁은 틈 가운데에 저비용 통로가 남는다 |
| `BaseObstacle.scale` | 0.02 | 0.2 → 0.006122 (유도) | 아래 참고. sum_scores를 켜면서 다시 풀었다 |
| `BaseObstacle.sum_scores` | false (DWB 기본) | true | 아래 참고 |

### `BaseObstacle.sum_scores`를 켠 이유

Comfort Space costmap을 읽는 critic은 `BaseObstacle` 하나뿐입니다.
나머지는 전부 경로와 목표에 대한 것입니다. 그런데 설치된 헤더
`dwb_critics/base_obstacle.hpp`가 이렇게 적어둡니다.

> it will return either the final pose's cost, or the sum of all poses,
> depending on the `sum_scores` parameter

기본값은 false이고, false는 **궤적의 마지막 pose 하나만** 본다는 뜻입니다.
최대값도 아니고 합도 아닙니다.

시뮬레이션의 궤적 지평은 `sim_time 1.7 s x 0.8 m/s = 1.36 m`입니다.
Comfort Space의 폭보다 넓습니다. 즉 **사람의 Comfort Space를 관통해서
반대쪽으로 빠져나가는 궤적은 비용 0으로 채점됐습니다.** 스치는 궤적과
뚫는 궤적이 제어기에게 구분되지 않았습니다.

costmap은 정상적으로 발행되고 KeepoutFilter로 양쪽 costmap에 들어가고
있었습니다. 전역 planner는 반응했습니다. 반응하지 않은 것은 지역
제어기이고, 이유는 가중치가 아니라 **집계 방식**이었습니다.

켜면 비용이 "얼마나 깊이" 들어가는지와 "얼마나 오래" 머무는지에 비례해
올라갑니다. Comfort Space가 만들어내야 하는 단계적 반응이 그것입니다.

### `BaseObstacle.scale`을 유도하는 이유

sum_scores를 켜면 점수 규모가 1개 pose에서 ~27개 pose의 합으로 바뀌므로
scale을 다시 풀어야 합니다. 설치된 헤더에서 확인한 단위는 이렇습니다.

* `MapGridCritic`(PathDist, PathAlign, GoalDist, GoalAlign)은 점수를 **셀**
  단위로 내고 `getScale()`을 `resolution * 0.5 * scale`로 override합니다
  (`dwb_critics/map_grid.hpp:69`). 따라서 `d` m 벗어나는 비용은
  `(d / resolution) * (resolution * 0.5 * scale) = d * 0.5 * scale`이고,
  **해상도가 약분됩니다.**
* `BaseObstacleCritic`은 `getScale()`을 override하지 않으므로 점수는
  costmap 값 그대로 x scale입니다.

그래서 `n`개 pose가 값 `m`의 mask 안에 있는 궤적의 비용은 `n * m * scale`이고,
이것을 Comfort Space를 실제로 벗어나는 최소 측면 이동 비용의
`avoid_margin`배와 같게 놓고 풉니다.

    n * m * scale = avoid_margin * d_clear * 0.5 * path_pressure

지평 전체를 Comfort Space 안에서 보내는 궤적은 돌아가는 것보다
`avoid_margin`배 비싸고, 가장자리만 스치는 궤적은 그에 비례해서만
비싸집니다. 이진이 아니라 단계적인 반응이 되는 지점이 여기입니다.

유도는 `comfort_core/nav2_tuning.py`에 있고,
`scripts/check_nav2_tuning.py`가 검사합니다.

### 실제로 조종하고 있는지 측정한 값

유도만으로는 "비용이 올라간다"까지만 말할 수 있습니다. DWB가 **고르는
궤적이 바뀌는지**는 별개이고, 그것을 `scripts/check_dwb_scores.py`가
`dwb_msgs/LocalPlanEvaluation`을 받아 직접 셉니다. 선택된 궤적에서
BaseObstacle의 기여를 뺐을 때 승자가 바뀌는 주기를 세는 방식입니다.

게이트 OFF, `configs/nav2_check.yaml`, b2, 1회 주행 (2026-11-04 15:08)
동안 1014개 제어 주기에서:

| | 값 |
|---|---|
| BaseObstacle이 0이 아닌 주기 | 357 / 1014 |
| 선택된 궤적에서의 비용 비중 | 평균 7.57%, 최대 67.48% |
| **승자를 바꾸는 주기 (DECISIVE)** | **857 / 1014 (84.5%)** |

선택된 궤적에서의 평균 기여(`|raw_score x scale|`)는
GoalAlign 11.285, GoalDist 10.906, BaseObstacle 3.648, PathAlign 0.985,
PathDist 0.575, RotateToGoal 0.368, Oscillation 0.000입니다.

비중(7.57%)과 DECISIVE(84.5%)가 크게 다른 것이 요점입니다. Comfort Space는
총 비용을 지배하지 않으면서 — 지배하면 사람을 피하려고 경로를 버립니다 —
후보 궤적들 사이의 순위는 대부분의 주기에서 결정합니다. 의도한 동작이
이것입니다.

측정할 때 주의: 평가 토픽은 `/controller_server/evaluation`이 아니라
전역 `/evaluation`으로 올라옵니다. `publish_evaluation`을 따로 켤 필요는
없었습니다(DWB 기본값으로 발행됨).

## 5b. 시뮬레이터와 실제 로봇이 같은 식을 씁니다

두 로봇은 파라미터 **파일**을 공유할 수 없습니다. 시뮬레이터는 Jazzy이고
실제 AMR은 Humble입니다. plugin 이름 문법이 다르고
(`nav2_navfn_planner/NavfnPlanner` 대 `nav2_navfn_planner::NavfnPlanner`)
Jazzy의 `navigators:` 블록은 Humble에 없습니다.

공유할 수 있는 것은 **산출식**이고, 공유해야 합니다. 튜닝 변경이 한쪽에만
반영되면 디지털 트윈이 서로 무관한 두 로봇이 됩니다.

    configs/common.yaml              공유 입력 (navigation.dwb, navigation.observation)
      └ configs/real_amr.yaml        실제 장비만의 값 (footprint, 속도, LiDAR 거리)
          └ real_corridor.yaml       그 복도와 그 촬영만의 값 (로봇에서 생성)

    comfort_core/nav2_tuning.py      산출식 하나
      ├ scripts/generate_nav2_params.py          (시뮬레이터, Jazzy)
      └ results/.../bin/make_nav2_params.py      (로봇, Humble 기본값 위에)

같은 식에서 **다른 값**이 나오는 것이 정상입니다. 두 로봇은 크기도 속도도
다릅니다.

| 값 | 시뮬레이터 | 실제 AMR |
|---|---|---|
| footprint | 0.700 x 0.480 m | 0.864 x 0.570 m |
| `max_vel_x` | 0.80 m/s | 0.20 m/s |
| `max_vel_theta` | 2.00 rad/s | 0.40 rad/s |
| 통과 여유 반경 | 0.340 m | 0.385 m |
| `inflation_radius` | 0.340 m (유도) | 0.350 m (복도 때문에 고정) |
| `obstacle_max_range` | 6.0 m | 10.0 m |
| `raytrace_max_range` | 8.0 m | 12.0 m |
| 궤적 지평 | 1.360 m = 27.2 pose | 0.340 m = 6.8 pose |
| `BaseObstacle.scale` | 0.006122 | 0.027731 |
| 관통 비용 / 측면이동 비용 | 16.32 / 10.88 = 1.50 | 18.48 / 12.32 = 1.50 |

`BaseObstacle.scale`이 4.53배 차이 나는 것이 핵심입니다. DWB의 지평은
**시간**이라서 0.20 m/s에서는 0.34 m밖에 못 가고, 합산 critic은 같은
땅에서 4분의 1밖에 누적하지 않습니다. 두 로봇에 같은 숫자를 넣는 것이
오히려 버그입니다.

`navigation.observation`이 왜 공유 입력에 있는지도 여기서 갈립니다.
CostmapFilter는 costmap의 갱신 범위를 스스로 넓히지 않으므로, obstacle
layer가 닿는 거리 밖에 칠해진 Comfort Space는 costmap에 아예 도착하지
않습니다. 실제 로봇에서 측정됐습니다 — x=5.28에 정확히 발행한 mask는
costmap에서 0이었고, 같은 mask를 x=2.24에 두면 96으로 들어왔습니다.

### 아직 확인되지 않은 것

| 항목 | 상태 |
|---|---|
| 실제 로봇의 collision monitor | **없습니다.** 로봇의 `start_nav2.sh`는 Humble의 `navigation_launch.py`를 띄우고, 거기엔 collision_monitor 노드가 없습니다. `common.yaml`이 유도하는 보호 영역은 시뮬레이션 전용입니다 |
| Humble의 `velocity_polygon` 지원 | 미확인. 타입이 스택의 나머지보다 최신입니다. 로봇에서 검증하기 전에는 안전 계층이 전이된다고 쓰지 않습니다 |
| Humble 기본 controller가 DWB인지 | 미확인. `make_nav2_params.py`가 DWB가 아니면 critic을 쓰지 않고 경고합니다 (Jazzy 기본값은 MPPI였습니다) |

## 5c. 세 대를 모두 Nav2로 돌리기 (진행 중)

목표는 a1, a2도 AMR1과 같은 Nav2 스택으로 주행시키는 것입니다.
지금은 `isaac_sim/utils/fleet.py`가 매 틱 `set_world_pose()`로 위치를
찍어 넣습니다. 주행이 아니라 이동 애니메이션입니다.

### 1단계 (완료): ActionGraph 네임스페이스 분리

에셋을 뜯어보니 모든 ROS 2 노드에 `inputs:nodeNamespace`가 이미
있었고 비어 있을 뿐이었습니다. 인스턴스마다 채우면 토픽이 갈라집니다.

    /cmd_vel   ->  /a1/cmd_vel, /a2/cmd_vel
    /odom      ->  /a1/odom,    /a2/odom

`isaac_sim/utils/robot_graph.py`가 합성된 스테이지 위에서 이 작업을
합니다. USD 파일은 디스크에서 그대로입니다. `scene_setup.py`의 다른
수정들과 같은 방식입니다.

조용히 실패하는 길이 셋 있어서 `isaac_sim/scripts/check_robot_graph.py`가
22개 항목으로 검사합니다.

| 함정 | 증상 | 대응 |
|---|---|---|
| `topicName`이 `/cmd_vel`로 절대경로 | 네임스페이스가 **무시된다** | 상대경로로 고쳐 쓴다 |
| `ROS2PublishTransformTree`에 프레임 접두사 입력이 없음 | 세 로봇이 같은 `base_link`를 발행, tf는 마지막 것을 채택 | 이 노드를 끄고, 프레임 ID가 문자열인 `RawTransformTree` 3개를 쓴다 |
| `/clock` 발행자 3개 | 서로 경쟁 | AMR1만 발행 |

`RawTransformTree` 3개가 `odom -> base_footprint -> base_link ->
lidar_link` 전부를 덮습니다. Nav2가 필요로 하는 프레임이 전부 여기
있습니다.

relationship(`/World/AMR`)은 따로 손대지 않았습니다. 참조 시점에 USD가
`/World/Fleet/a1/model/AMR`로 자동 번역합니다 (검사 [7]번).

### 2단계 (시뮬레이터 쪽 완료): 물리 로봇으로 복원

`_reference_amr()`은 이제 두 가지로 갈립니다. 네임스페이스가 없으면
전과 같이 그래프를 끄고 kinematic으로 만들고, 있으면 **아무것도
벗겨내지 않습니다.** 벗겨냈던 이유가 전역 `/cmd_vel` 공유였고 1단계가
그것을 없앴기 때문입니다.

`fleet.driver`가 어느 쪽인지 정합니다. 기본값은 `scripted` -- 지금까지
측정한 모든 결과가 그것으로 나왔으므로, 기존 config는 한 글자도 고치지
않고 그대로 돕니다. `configs/fleet_nav2.yaml`이 검증용 오버레이입니다.

`robot_graph.instance_fixes()`가 반드시 같이 돌아야 합니다. `scene_setup`이
AMR1에만 적용하던 두 수리를 복제본도 필요로 합니다 -- 오도메트리
`orientation` 미연결(연결하지 않으면 회전해도 단위 쿼터니언을 보고),
LiDAR 부분 스캔(7절).

`scripts/check_fleet_nav2.py`가 돌고 있는 시뮬레이션에 붙어 20개를
확인합니다. 결정적인 것은 **한 대를 몰면 그 한 대만 움직이는가**입니다.
속성이 전부 맞아 보여도 런타임에 한 토픽으로 뭉개질 수 있고, 그때
나타나는 증상이 세 대가 같이 움직이는 것입니다.

    a1 moved when driven   0.860 m
    a2 stayed put          0.002 m

남은 것은 job list를 Nav2 goal로 보내는 노드이고, 그것은 3단계에서
Nav2 스택이 생긴 뒤에 붙습니다. 시뮬레이터 안이 아니라 ROS 2 노드로
만듭니다 -- 실제 차량이 같은 것을 쓸 수 있도록.

### 3단계 (완료): Nav2 스택 3벌

`scripts/run_nav2.sh --namespace a1`이 같은 스택을 `/a1` 아래 띄웁니다.
`scripts/run_fleet_nav2.sh`가 `fleet.driver: nav2`인 config의 enabled
AMR마다 그것을 한 번씩 부릅니다. 노드 목록은 `launch_nodes.nav2_stack()`
하나이고 AMR1도 같은 함수로 올라옵니다 -- 두 벌의 설명이 생기면
어느 한쪽만 고쳐지는 날이 옵니다.

파라미터는 `generate_nav2_params.py --namespace a1`이 같은 템플릿에서
냅니다. 수치는 전부 같은 식(5b)이고 -- 같은 차량이므로 -- 바뀌는 것은
이름뿐입니다.

| | AMR1 | 플릿 a1 |
|---|---|---|
| 프레임 | `odom`, `base_footprint` | `a1/odom`, `a1/base_footprint` |
| 토픽 | `/scan`, `/odom`, `/cmd_vel` | `/a1/scan`, `/a1/odom`, `/a1/cmd_vel` |
| 파라미터 파일 | `nav2_params.yaml` | `nav2_params_a1.yaml`, 전체가 `a1:` 아래 중첩 |
| Comfort Space mask | `/costmap_filter_info` | **같은** `/costmap_filter_info` |
| 속도 경로 | `cmd_vel_nav` → gate → `cmd_vel_safe` → monitor | `cmd_vel_safe` → monitor |

템플릿에서 하드코딩돼 있던 `odom`, `/scan`, `/odom`,
`/local_costmap/published_footprint` 여섯 곳을 변수로 바꿨고, 네임스페이스
없이 렌더한 결과는 세 config 모두에서 전과 **바이트 단위로 같습니다.**

플릿 스택에 없는 것 셋: `comfort_publisher`(mask는 하나, AMR1 쪽이
발행), `people_tracker`(같은 이유), `comfort_gate`(측정 대상 정책이지
교통이 아님). `collision_monitor`는 있습니다 -- 안전층은 어느 차량에서도
빼지 않습니다.

토픽 이름은 **절대경로**로 씁니다 (`/a1/scan`). Nav2 파라미터 안의
상대 이름은 그것을 읽는 노드 기준으로 풀리는데, costmap은 서버보다 한
단계 깊은 노드(`/a1/local_costmap/local_costmap`)라 `scan`은
`/a1/local_costmap/scan`이 됩니다. `comfort_core/frames.py`의
`namespaced_topic()`이 그 규칙 하나를 맡습니다.

tf는 remap하지 않습니다. `tf2_ros`는 절대경로 `/tf`를 구독하므로
네임스페이스에 들어간 노드도 전역 트리 하나를 읽습니다. nav2_bringup이
`/tf`를 네임스페이스 안으로 remap하는 것은 로봇마다 트리를 따로 두기
위해서이고, 여기서는 시뮬레이터가 트리를 하나만 올리므로 그 반대가
맞습니다.

스폰 방향은 `frames.fleet_spawn()`이 정합니다 -- 첫 job을 향하도록.
`Nav2FleetAMR`과 `map -> a1/odom` static transform이 같은 함수를 불러
물리가 출발하는 자세와 Nav2가 믿는 자세가 어긋날 수 없습니다. 전에는
yaw 0이었고 dock_east의 a2는 동쪽 벽을 보고 있었습니다.

`scripts/check_fleet_stacks.py`가 세 스택이 올라온 상태에서 18개를
확인합니다. 결정적인 것은 **a1에 goal을 주면 a1만 가는가**입니다.

    AMR1 stayed put   0.005 m
    a1 drove to the goal          1.810 m, succeeded
    a2 stayed put                 0.004 m
    /cmd_vel publishers           /collision_monitor 뿐

세 스택 + Isaac Sim에서 load average 7.7, 메모리 15 GB. 돌아갑니다.

### 감속(warning) 필드

collision_monitor에 `Slowdown` polygon을 하나 더 넣었습니다. stop field
앞에 있는 경고 필드로, 들어온 것이 있으면 Nav2 명령을 0.60배로 줄이고
멈추지는 않습니다. 실제 AMR의 경고 필드와 같은 역할입니다.

수치는 stop field와 같은 식에서 나옵니다(`generate_nav2_params.py`,
`configs/common.yaml`의 `navigation.collision_monitor.slowdown`).
비율 0.60은 이미 있던 mid 속도 밴드이고 -- 감속하면 그 밴드용 stop
field 안으로 들어가도록 -- 도달 거리는 그 감속 동안 차량이 가는 거리
+ 같은 시간에 걸어오는 사람이 좁히는 거리(ISO 13855 보행 접근속도
1.6 m/s) + mid용 stop field입니다.

    stop field (cruise)   1.207 m
    slowdown field        2.302 m

같은 `nav2_check` 시나리오에서 전후:

    감속 필드 없음   SUCCESS  28.55~28.63 s  worst_intrusion 0.879~0.938
    감속 필드 있음   SUCCESS  30.28 s        worst_intrusion 0.844, Slowdown 8.35 s

모든 정책에 같이 걸리므로 비교에는 영향이 없지만, 이 줄 이전의
녹화와 측정은 전부 감속 필드가 없는 조건입니다. trials.csv의 monitor
state 열에 `Slowdown`과 그 누적 시간이 남습니다.

### 4단계 (디스패처 완료, 우측통행 남음): 플릿이 Nav2로 달린다

`scripts/fleet_goals.py`가 플릿의 디스패처입니다. scripted 드라이버가
시뮬레이터 안에서 하던 일 -- 차량을 job list 따라 보내기 -- 을 실제
플릿이 하는 방식으로, 각 차량의 Nav2에 `NavigateToPose`를 하나씩
보냅니다. 조종은 하지 않습니다. 출발 시각은 scripted와 같은 식
(`trial_start_sim_time + start_at`, 시뮬레이션 시계)이라 AMR1이 만나는
교통이 부팅 시간에 좌우되지 않습니다. SUCCEEDED면 다음 job, ABORTED면
한 번 재시도 뒤 건너뜁니다. 진행은 로그와 `/fleet/status`로 나갑니다.

`run_scenario.sh`가 `fleet.driver: nav2`를 보면 AMR1 Nav2 다음에
`run_fleet_nav2.sh`(스택)와 `run_fleet_goals.sh`(디스패처)를 띄우고,
끝에 디스패처 로그를 결과 폴더에 남깁니다.

플릿의 출발지와 목적지는 전부 **빈 앞줄 대차 칸**입니다
(`fleet.places`, 레이아웃 이름 그대로 `n1_r2c2` 식, AMR1과 같은 입구
포즈 y ±5.32). 전에 있던 도크와 `carts_*`는 뺐습니다 -- `carts_n`은
대차가 서 있는 칸이라 Nav2 goal이 장애물 안에 들어가는 자리였습니다.
스폰과 goal의 방향은 통로 쪽(`frames.fleet_spawn`,
`fleet_goals._facing_aisle`)이라 차량이 칸에서 전진으로 나오고 보호
필드가 빈 바닥을 봅니다.

**scripted 드라이버가 만들던 교착이 사라졌습니다.** 같은 시나리오
(`paper_h8_a3`, B3-O)에서:

    scripted   a2 ( -6.00, -4.43) held by person (S1 at 1.18 m, needs 1.19)   -- 영원히
    nav2       a2 t= 50.2 -> s2_r2c2 ... t= 65.6 reached s2_r2c2

원인은 기하입니다. 작업자 기둥(x −6.5 / −0.5 / 5.5)과 빈 칸 기둥
(−6.0 / 0.0 / 6.0)이 0.5 m 옆이라 그 칸에서 나오는 길은 작업자 등 뒤
약 1.0 m를 지나는데, scripted의 hold는 이진 규칙(반경 + 반폭 ≥ 거리면
정지, 시한 없음)이고 S1은 움직이지 않습니다. Nav2는 같은 Comfort
Space를 비용으로 읽어 값을 치르고 나옵니다 -- AMR1이 같은 칸 (6.0,
5.32)에서 N3 등 뒤를 지나 출발하는 것과 같은 일이고, 그 값이 5단계
지표입니다. 반경도 hold 규칙도 손대지 않았습니다.

AMR1 결과는 플릿이 scripted일 때와 같은 급입니다 (SUCCESS, 침입 0,
Slowdown 4.6 s). 스택 3벌 + 디스패처 + Isaac에서 그대로 돕니다.

### 5단계 (완료): a1, a2도 같은 잣대로 잰다

`scripts/fleet_goals.py`가 디스패처이면서 플릿의 측정기입니다.
`nav2_goal.py`가 AMR1에 대고 매 틱 하는 계산을 같은 재료로 차량마다
합니다 -- 사람 위치는 같은 `/tf` 프레임, 침입은 같은 기준 Comfort
Space(`evaluation.comfort_reference`, b3), 안전층은 그 차량의
`/aN/collision_monitor_state`. 끝날 때 차량당 한 줄을
`fleet_trials.csv`에 씁니다 (`trials.csv` 옆, `comfort_core/metrics.py`
`FLEET_FIELDS`).

| 열 | 뜻 |
|---|---|
| `legs_total / done / skipped`, `set_off_at_s`, `first_leg_s` | job list 진행 (trial 시각 기준) |
| `travel_distance_m`, `observed_s` | 이 행이 그 차량을 지켜본 거리·시간 |
| `min_clearance_m / with` | 사람과의 최소 간격, 누구와 |
| `contact_s`, `contact_robot_s`, `contact_person_s` | 접촉 시간, 누가 마지막 1 cm를 좁혔는지 (AMR1과 같은 규칙) |
| `worst_intrusion_m / k / into`, `comfort_seconds` | 기준 Comfort Space 침입 |
| `safety_stop_s / slowdown_s / approach_s` | 그 차량의 안전층 |
| `min_gap_amr1_m`, `min_gap_fleet_m / with` | 다른 로봇과의 최소 중심거리 -- 우측통행 값을 정할 숫자 |

세 대는 **같은 정책 아래 같은 잣대로 재는 동등한 대상**입니다. 파일이
별도인 이유는 열이 다르기 때문입니다 -- AMR1은 goal 하나짜리 mission,
a1/a2는 job list와 다른 로봇 -- 그리고 이미 결과가 쌓인 `trials.csv`의
열을 옮기지 않기 위해서입니다. 두 파일은 (scenario, config, 같은 분의
stamp)로 이어집니다. 지금은 trial의 result 열이 AMR1의 mission만
보는데, a1/a2의 접촉·교착을 어떻게 합산할지는 아래 "남은 단계"입니다.

costmap 영상에도 a1/a2 footprint가 각자 색으로 그려집니다.

같은 `paper_h8_a3`, B3-O, 두 번 돌린 결과 (AMR1: 둘 다 SUCCESS, 침입 0):

    a1  SUCCESS  legs 1/3   9.3 m   min clearance 0.97 m   침입 0          slow 9.4 s
    a2  SUCCESS  legs 1/3   9.1 m   min clearance 0.14 m (E2, 16.1 s)
                                    0.90 m 깊이 (k 0.47), 6.6 s 안        slow 4.4 s
    a1-a2 최소 8.3 m, a2-AMR1 최소 4.9 m, 접촉 0

a2-E2는 두 번 다 같은 자리에서 났습니다: a2가 첫 job `s2_r2c2`
(0, −5.32)에 들어가는 순간 E2(남쪽 교대 보행자)가 그 칸 입구를 지나
벨트로 올라갑니다 -- `within 0.149 m of E2: robot (−0.25, −5.34)
person (−0.66, −4.67)`. 둘 다 움직이고 있었고, 접촉은 없었고, 사람은
로봇을 피하지 않는다는 전제대로 E2는 계속 걸었습니다. 이것이 이
측정이 잡으라고 있는 장면입니다.

첫 번째 run에서는 a1이 N2에 0.003 m까지 간 것으로 한 틱 찍혔습니다.
N2의 0.6 m 기준 공간에 들어갔다 나온 시간이 0.6 s라 0.8 m/s 차량으로는
불가능한 숫자이고, 두 번째 run에서는 재현되지 않았습니다. 원인은
확정하지 못했습니다. 그래서 AMR1의 `pose_is_sane()`에 해당하는 가드
(한 틱에 최고속도의 3배 넘게 "이동"하면 그 틱을 버리고 `pose jumped`로
남김)와, 0.20 m 이내 근접 시 양쪽 좌표를 찍는 줄을 넣었습니다. 다음에
또 나오면 그 줄이 어느 쪽이 튀었는지 말해 줍니다.

### 차량마다 자기 판정

trial 하나에 합성 판정을 만들지 않습니다. AMR1은 `trials.csv`의
`result`, a1/a2는 `fleet_trials.csv`의 `result` -- 차량마다 자기 결과를
적고, 성공률은 나중에 정책별로 세 대를 합쳐 셉니다. 판정 규칙은
AMR1과 같습니다:

| result | 뜻 |
|---|---|
| `SUCCESS` | trial 안에 시도한 leg를 전부 도달했거나 끝날 때 달리는 중 |
| `DEADLOCK` | goal이 있는데 `navigation.deadlock` (12 s 동안 0.15 m 진전 없음). `deadlock_cause`에 안전층이 세우고 있었는지 적힘 |
| `ABORTED` | Nav2가 한 leg를 두 번 포기 |

교착이면 goal을 취소하고 더 보내지 않되 측정은 계속합니다. job list가
mission보다 길어서 "전부 완료"를 성공 조건으로 둘 수 없고, 그래서
`legs_done`이 따로 있습니다.

### a3를 켠 네 대 (`paper_h8_a4`)

`amr_count` 4 = AMR1 + a1 + a2 + a3. a3는 북쪽 블록에서 남쪽 블록으로
건너가는 차량이라 a1과 북쪽 레인을, a2와 남쪽 레인을, AMR1과 동쪽
교차로를 공유합니다. 로봇끼리 실제로 마주치는 시나리오는 이것입니다.

첫 run에서 플릿 교착 판정기가 **거짓 교착**을 냈습니다. AMR1과 같은
"goal까지 직선거리가 12 s 동안 `d_threshold`만큼 안 줄면 교착" 규칙을
썼는데, a3의 goal은 남서쪽이고 Nav2는 동쪽 교차로로 도는 경로를
잡아서, 0.8 m/s로 달리는 20 m 동안 직선거리가 커집니다:

    [fleet] a3 t= 60.9  DEADLOCK 13.82 m from the goal: no progress for 12 s
    sim: a3 (+1.40, +5.32) -> (+2.62, +4.39) -> (+6.56, +4.49)   달리는 중

지금은 Nav2가 발행하는 `/aN/plan`의 **남은 경로 길이**로 진전을 잽니다
(`fleet_goals.Vehicle.remaining`). 이건 가는 내내 줄어듭니다. 같은
규칙을 쓰는 `nav2_goal.py`(AMR1)는 아직 직선거리입니다 -- 지금까지의
mission은 가까운 교차로로 나가서 걸린 적이 없지만, 먼 교차로로 도는
mission이 뽑히면 같은 거짓 교착이 납니다. 바꿀지는 측정 정의의 문제라
결정 대기.

고친 뒤 run (B3-O):

    AMR1  SUCCESS
    a1    SUCCESS  legs 2/3  10.0 m  min clearance 0.94 m (E1)             slow 9.0 s
    a2    SUCCESS  legs 1/3   9.4 m  min clearance 0.16 m (E2, 16.5 s)  1.02 m 침입  slow 8.0 s
    a3    SUCCESS  legs 0/1  19.0 m  (trial이 끝날 때 남쪽 레인을 서쪽으로 달리는 중)
    로봇 간: a3-AMR1 1.19 m (동쪽 교차로, 같은 방향), a3-a2 5.45 m, a1-a3 7.38 m

a2-E2는 네 번째도 같은 자리입니다. 로봇끼리는 아직 **같은 방향**으로
가까워진 것뿐이고(a3가 AMR1 뒤를 따라 동쪽 교차로를 돎), a3와 a2의
남쪽 레인 **정면 조우**는 trial이 AMR1 mission과 함께 끝나는 바람에 그
전에 끊겼습니다. 조우를 보려면 trial을 플릿 job이 끝날 때까지 잡아
두는 옵션이 필요합니다.

### 남은 단계

4b. 우측통행 규칙. 아래 참고 -- `min_gap_*` 열이 쌓인 뒤에
6. `summarise_trials.py`가 두 파일을 같이 읽어 차량별·정책별로 세기
   (집계 도구이지 결과 생성이 아님 -- 돌리는 것은 별개)

### 우측통행 규칙 (4b, 구현 완료, 값 비교 중)

Nav2 두 대가 좁은 레인에서 마주치면 서로를 LiDAR 장애물로 보고 각자
피하려 합니다. 둘 다 같은 쪽으로 피하면 다시 마주 보고, 교착이
됩니다. 사람 때문에 생기는 교착과 구분되지 않는 교착이 하나 더
생기는 셈이라, 연구 대상이 오염됩니다. 통행 규칙이 그 대칭을 깹니다.

**mask 하나로는 안 됩니다.** "왼쪽"은 진행 방향에 따라 다른 쪽이라
(동진 로봇의 왼쪽 = 북쪽 절반, 서진 로봇의 왼쪽 = 남쪽 절반) 공용 정적
mask는 방향을 담지 못합니다. 그래서 `scripts/traffic_rule.py`가
**로봇마다** 자기 `/plan`에서 레인(복도 2개, 교차로 2개)별 진행 방향을
읽어 자기 왼쪽 절반만 칠한 mask를 `/aN/traffic_mask`로 발행하고,
costmap에 두 번째 `KeepoutFilter`(`traffic_filter`)가 그것을 읽습니다.
plan이 지나지 않는 레인은 칠하지 않습니다.

* Comfort Space mask와 **섞지 않습니다** -- 사람 때문에 비껴갔는지
  규칙 때문인지 영상에서 구분되어야 합니다
* 값은 지어내지 않고 **`comfort_space.costmap.buffer_value`(55)**를
  씁니다: 왼쪽 절반은 남의 Comfort Space 바깥 버퍼만큼 비쌉니다.
  사람 공간(98)보다 약해서 사람 공간이 이기고, inflation보다 셉니다.
  `traffic_rule.value`로 바꿀 수 있습니다
* 금지가 아니라 선호입니다. 왼쪽이 유일한 길이면 비용을 내고 갑니다
* 레인 폭은 지도가 깎아 둔 여유(`lane_margin` 0.25)까지 포함해 중심선
  (셀 경계)에서 나눕니다 -- 여유가 반대편의 공짜 차선이 되지 않도록
* **기본값 OFF** (`traffic_rule.enabled: false`). 그 전의 모든 결과는
  규칙 없이 측정됐고, OFF면 렌더된 파라미터가 바이트 단위로 전과
  같습니다. `configs/fleet_nav2.yaml`이 켜고,
  `configs/fleet_nav2_norule.yaml`이 비교용으로 끕니다

`scripts/traffic_rule.py --dry-run`이 ROS 없이 두 방향 mask를 칠해
겹침 0을 확인합니다.

**plan에서 읽으면 안 됩니다 -- 측정으로 알게 된 것.** 첫 판은 로봇의
`/plan`이 지나는 모든 레인을 칠했습니다. 그러면 고리가 닫힙니다:
mask가 고른 경로의 셀 비용을 올리고, planner가 반대쪽 교차로로
바꾸고, mask가 새 plan을 따라가고, planner가 다시 돌아옵니다. a3는
두 경로 길이가 거의 같아 plan이 서쪽/동쪽 교차로 사이를 왔다 갔다
했고(`keep right on ... cross_01 -` / `cross_02 -` 교대), 1.9 m만 간
채 진전 규칙이 교착으로 불렀습니다. a2도 같은 run에서 교착.

    규칙 ON (plan 기반)   a1 SUCCESS 3/3   a2 DEADLOCK 2/3   a3 DEADLOCK 0/1
    규칙 OFF              a1 SUCCESS 3/3   a2 SUCCESS  3/3   a3 SUCCESS  1/1

그래서 규칙은 실제 도로 규칙처럼 **국소**가 됐습니다: 로봇이 **지금
들어 있는 레인**만, 그 레인 축을 따라 **실제로 움직인 방향**으로
칠합니다 (`min_run_m` 0.3 m 이상 움직여야 방향이 정해지고, 멈추면
마지막 방향을 유지). 앞쪽 레인의 비용은 바뀌지 않으므로 경로 선택에
손대지 않고, 바뀌는 것은 지금 레인의 어느 절반이 싼가뿐입니다.

### 순환 플릿 (`configs/fleet_loop.yaml`)

`paper_h8_a4`의 플릿은 a1이 북쪽, a2가 남쪽 레인만 왕복해 로봇끼리의
정면 조우가 타이밍 운에 달려 있었습니다 (네 run에서 a3-a2 최소 4.4 m).
`fleet_loop.yaml`은 같은 시나리오 위에서 세 대가 모두 **한 번씩
건너가게** job list만 바꾼 진단 오버레이입니다 -- a1 북→남→북, a2
남→북→남, a3 북→남→북. a1과 a2는 0.9 s 차이로 서쪽 교차로를 반대
방향으로 향하므로 정면 조우가 구조적으로 납니다. 자리는 전부 생성된
시나리오의 빈 앞줄 칸이고, `scripts/check_fleet_loop.py`가 mission
양 끝에서 1.80 m 밖인지 다시 셉니다(가장 가까운 것이 4.6 m).
`fleet_loop_norule.yaml`이 규칙 OFF 비교용입니다.

### `--fleet-tail`: 조우를 끝까지 보기

trial은 AMR1 mission과 함께 끝나는데(~28 s), 플릿 job list는 그보다
길어 로봇끼리의 정면 조우가 그 뒤에 납니다. `run_scenario.sh
--fleet-tail SECONDS`는 mission이 결정된 뒤에도 플릿이 job을 끝내거나
시간이 다할 때까지 모든 것을 돌려 둡니다. 진단용이고 기본값 0.
AMR1의 행은 어느 쪽이든 같고, 플릿의 행만 더 오래 봅니다.

### 교착 판정: 경로를 따라

플릿에서 거짓 교착이 측정된 뒤(위 "a3를 켠 네 대") AMR1의 판정도
같은 규칙으로 바꿨습니다: 진전은 goal까지 직선거리가 아니라 Nav2가
발행한 `/plan`의 **남은 길이**로 잽니다 (`nav2_goal.path_remaining`).
plan이 아직 없으면 직선거리로 시작합니다. 지금까지의 모든 run에서 두
규칙은 같은 답을 냈습니다(mission이 가까운 교차로로 나갔으므로);
먼 교차로로 도는 mission이 뽑히는 seed부터 달라집니다.

### 측정해서 알게 된 것: tf는 네임스페이스에 넣지 않는다

브리지는 `topicName`의 선행 슬래시를 떼고 네임스페이스를 다시 붙입니다.
`topicName = "/tf"`로 절대경로를 줘도 `nodeNamespace = "a1"`이면 결국
`/a1/tf`로 갑니다. 토픽 이름으로 네임스페이스를 빠져나갈 수 없습니다.

    네임스페이스를 붙였을 때   /a1/tf, /a2/tf 생성
                               /tf 의 frame_id 에 a1/ 이 0개
                               can_transform(a1/odom, a1/base_footprint) false

그래서 tf 노드 3개만 `nodeNamespace`를 비워 둡니다. 기본 상대명 `tf`가
네임스페이스 없이 `/tf`로 해석되고, 프레임 ID에는 접두사가 붙어 있으므로
전역 트리 하나에 `a1/base_link`와 `a2/base_link`가 공존합니다.

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
