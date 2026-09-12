# AMR Reference Source

이 프로젝트에서 외부 저장소로부터 가져온 파일과 그 수정 내역을 기록합니다.

---

## 1. 원본 저장소

| 항목 | 값 |
|---|---|
| Repository URL | https://github.com/dongtak2/AMR-Digital-Twin |
| 접근성 | private. `hun7407-lgtm` 계정으로 clone 확인 (2026-09-12) |
| Commit | `c54269ee80032c29b0548eb77c3ee38d68b87e78` |
| Commit date | 2026-09-11 18:00:51 +0900 |
| Author | dongtak2 |
| **LICENSE** | **없음** |

### 라이선스 주의

원본 저장소에 LICENSE 파일이 없습니다.
명시적 라이선스가 없는 저작물은 기본적으로 저작권자가 모든 권리를 보유합니다.

따라서 다음을 지킵니다.

- 이 저장소는 public이므로, 원저작자에게 재배포 동의를 받아야 합니다.
- 논문이나 발표에서 AMR 모델을 사용할 때 출처를 명시합니다.
- 원저작자에게 LICENSE 파일 추가를 요청하는 것이 가장 깔끔합니다.

동의를 받기 전까지는 `assets/amr/AMR_09_02_V2.usd`를 public push하지 않는 편이 안전합니다.

---

## 2. 원본 저장소 구조

```text
AMR-Digital-Twin/
├── README.md
├── .gitignore
├── isaac_sim/
│   └── AMR_09_02_V2.usd          1.2 MB, AMR 본체 + ActionGraph
├── nav2/
│   └── amr_nav2_params.yaml      11 KB, Nav2 파라미터
├── scripts/
│   ├── amr_arrow_teleop.py       방향키 teleop
│   ├── start_scan.sh             PointCloud2 -> LaserScan
│   ├── start_nav2.sh             Nav2 + RViz 실행
│   └── check_ros.sh              토픽 존재 확인
└── docs/
    └── HANDOFF.md
```

전체 9개 파일입니다.

---

## 3. USD 분석 결과

`isaac_sim/AMR_09_02_V2.usd`를 pxr로 직접 열어 확인한 내용입니다.

### Stage

| 항목 | 값 |
|---|---|
| Default prim | `/World` |
| Up axis | Z |
| Meters per unit | 1.0 |

### 중요: AMR 단독 USD입니다

`/World` 아래에 있는 것은 `_materials`, `AMR`, `AMR_Controller` 뿐입니다.
**PhysicsScene, 조명, 바닥면(ground plane)이 모두 없습니다.**

우리 입장에서는 오히려 유리합니다.
복도 환경을 우리가 직접 만들고 이 AMR을 reference로 얹으면 되기 때문입니다.
다만 씬 생성 시 PhysicsScene, 바닥, 조명을 반드시 추가해야 합니다.
그렇지 않으면 AMR이 무한 낙하합니다.

### Prim 구조

```text
/World
├── _materials/Material_0            (base_link 텍스처)
├── AMR                              [Xform] PhysicsArticulationRootAPI
│   ├── base_link                    [Xform] RigidBody
│   │   ├── base_link_visual         [Mesh]
│   │   └── lidar_link               [OmniLidar]  ← 외부 reference
│   ├── left_wheel                   [Xform] RigidBody
│   │   ├── left_wheel_visual        [Mesh]
│   │   ├── left_wheel_joint         [RevoluteJoint] axis=Y, DriveAPI:angular
│   │   └── left_wheel_collision     [Cylinder]
│   ├── right_wheel                  (좌측과 대칭)
│   ├── caster_left_mount            [Xform] RigidBody
│   │   └── caster_left_swivel_joint [RevoluteJoint] axis=Z
│   ├── caster_left                  [Xform] RigidBody
│   │   ├── caster_left_wheel_joint  [RevoluteJoint] axis=Y
│   │   └── caster_left_collision    [Cylinder]
│   ├── caster_right_mount           (좌측과 대칭)
│   └── caster_right                 (좌측과 대칭)
└── AMR_Controller                   [OmniGraph] 23 nodes
```

Articulation root는 `/World/AMR`입니다.
구동륜은 `left_wheel_joint`, `right_wheel_joint` 두 개이고
나머지 네 조인트는 수동 캐스터입니다. 전형적인 differential drive 구성입니다.

### ActionGraph: `/World/AMR_Controller`

노드 23개짜리 완성된 그래프입니다. 우리가 새로 만들 필요가 없습니다.

제어 경로입니다.

```text
ROS2SubscribeTwist (/cmd_vel)
   ├── linearVelocity  ─► IsaacScaleToFromStageUnit ─► BreakVector3 ─► .x ─┐
   └── angularVelocity ─► BreakVector3 ─► .z ─────────────────────────────┐│
                                                                          ││
   OnPlaybackTick.deltaSeconds ─► dt ────────────────────────────────────┐│││
                                                                         ▼▼▼▼
                          DifferentialController (wheelRadius, wheelDistance)
                                            │ velocityCommand
                                            ▼
              IsaacArticulationController (targetPrim=/World/AMR,
                                           jointNames=[left_wheel_joint,
                                                       right_wheel_joint])
```

센서 및 상태 발행 경로입니다.

| 노드 | 발행 내용 |
|---|---|
| `ros2_publish_clock` | `/clock` (IsaacReadSimulationTime 기반) |
| `ros2_publish_joint_state` | `/joint_states`, targetPrim `/World/AMR` |
| `isaac_compute_odometry_node` | chassisPrim `/World/AMR/base_link` |
| `ros2_publish_odometry` | `/odom` |
| `ros2_publish_transform_tree` | articulation TF, parentPrim `base_link` |
| `ros2_publish_raw_transform_tree` | `odom` → `base_footprint` |
| `ros2_publish_raw_transform_tree_01` | `base_footprint` → `base_link`, (0, 0, 0.21) |
| `ros2_publish_raw_transform_tree_02` | `base_link` → `lidar_link`, (0.08, 0, 0.1) |
| `ros2_rtx_lidar_helper` | `/lidar/cloud`, PointCloud2, frame `lidar_link` |
| `isaac_create_render_product` | cameraPrim = lidar_link |

노드 타입이 전부 Isaac Sim 5.x 이름(`isaacsim.ros2.bridge.*`,
`isaacsim.core.nodes.*`, `isaacsim.robot.wheeled_robots.DifferentialController`)입니다.
우리 Isaac Sim 5.1과 그대로 호환됩니다.

### 구동 파라미터

| 항목 | 값 | 출처 |
|---|---|---|
| Wheel radius | 0.127 m | `differential_controller.inputs:wheelRadius` |
| Wheel distance | 0.48 m | `differential_controller.inputs:wheelDistance` |
| base_footprint → base_link | z = 0.21 m | raw_transform_tree_01 |
| base_link → lidar_link | (0.08, 0, 0.1) | raw_transform_tree_02 |
| Footprint | 약 0.70 × 0.48 m | 원본 README |

### 외부 의존 리소스

USD가 자체 완결이 아닙니다. 두 개의 외부 참조가 있습니다.

| 대상 | 경로 | 상태 |
|---|---|---|
| LiDAR 센서 정의 | `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Sensors/NVIDIA/Example_Rotary.usda` | HTTP 200, 접근 가능 |
| base_link 텍스처 | `./textures/diffuse.jpg` | **원본 저장소에 없음** |

LiDAR reference는 Isaac Sim 5.1 공식 클라우드 에셋이라 인터넷만 되면 자동으로 받아집니다.
텍스처는 누락되어 있어 `base_link`가 흰색으로 렌더링됩니다.
외형 문제일 뿐 물리와 navigation에는 영향이 없으므로 지금은 무시합니다.

---

## 4. 알려진 결함

가져온 파일에서 발견한 문제입니다. 고치기 전에 여기에 기록해 둡니다.

### 4.1 Odometry publisher 결함 3건

`ros2_publish_odometry` 노드에 문제가 세 개 있습니다.

**(a) child frame 오타**

```
inputs:chassisFrameId = "bsae_footprint"
```

`base_footprint`의 오타입니다.
`/odom` 메시지의 `child_frame_id`가 TF 트리에 없는 프레임을 가리킵니다.
TF는 별도의 raw transform 노드가 올바른 이름으로 발행하므로 지금까지 드러나지 않았습니다.

**(b) orientation 미연결 (더 심각)**

`inputs:orientation`이 선언만 되어 있고 어디에도 연결되어 있지 않습니다.
`isaac_compute_odometry_node.outputs:orientation`이 존재하는데도 쓰이지 않습니다.

결과적으로 `/odom`이 로봇이 아무리 회전해도 항상 단위 사원수를 보고합니다.

STEP 4에서 직접 관측한 값입니다.

```
commanded 0.5 rad/s for 4 s = 2.000 rad
odometry reports 0.000 rad      ← 로봇은 실제로 회전하고 있었음
```

이 상태로 Nav2를 붙이면 로봇의 방향을 전혀 알 수 없으므로
경로 추종이 성립하지 않습니다.

**(c) odomFrameId 미지정**

`inputs:odomFrameId`가 비어 있습니다.
노드 기본값에 의존하고 있어 메시지와 TF가 어긋날 여지가 있습니다.

**조치 완료 (STEP 4)**: `isaac_sim/utils/scene_setup.py`의 `patch_odometry_graph()`가
씬 구성 시 composed stage 위에서 세 가지를 모두 고칩니다.
디스크의 원본 USD는 바이트 단위로 그대로 두었습니다.
수정이 코드에 남아 있어야 무엇을 왜 바꿨는지 추적되기 때문입니다.
프레임 이름은 `configs/common.yaml`의 `ros.frames`에서 읽습니다.

수정 후 관측값입니다.

```
patched: odometry chassisFrameId 'bsae_footprint' -> 'base_footprint'
patched: connected odometry orientation from IsaacComputeOdometry
patched: set odometry odomFrameId -> 'odom'

commanded 0.5 rad/s for 4.00 simulated s = 2.000 rad
odometry reports 2.080 rad
```

### 4.2 obstacle layer 중복

`amr_nav2_params.yaml`의 `front_rplidar_obstacle_layer`와
`back_rplidar_obstacle_layer`가 동일한 `/scan`을 소스로 씁니다.
원본 README도 이 점을 인정하고 하나로 정리할 것을 권장합니다.

**조치 예정**: PHASE D에서 하나로 통합합니다.
동일 스캔을 두 번 마킹하면 costmap 비용이 이중 계산되어
Comfort Space 실험의 통행 가능 판정을 왜곡할 수 있습니다.

### 4.3 RTX LiDAR render product

`spawn_amr.py`를 headless로 실행하면 다음 경고가 한 번 나옵니다.

```
[Warning] [isaacsim.ros2.bridge.ogn.python.nodes.OgnROS2RtxLidarHelper] Render product not valid
```

**해결됨 (STEP 4)**: 원인은 USD가 아니라 실행 방식이었습니다.
`SimulationContext.step(render=False)`는 물리만 전진시키고
`app.update()`를 호출하지 않습니다.
`app.update()`가 OmniGraph의 `OnPlaybackTick`을 구동하므로,
render를 끄면 ActionGraph 전체가 한 번도 tick하지 않습니다.

`isaac_sim/scripts/run_amr_sim.py`가 headless일 때도 항상 `render=True`로
step하도록 고친 뒤 경고가 사라지고 `/lidar/cloud`가 정상 발행됩니다.
자세한 내용은 `docs/SETUP.md`의 문제 해결 표에 있습니다.

### 4.4 teleop 방향키가 동작하지 않음

`scripts/amr_arrow_teleop.py`의 방향키가 전혀 인식되지 않습니다.
스페이스와 q도 동작하지 않습니다.

원인은 키 입력을 읽는 방식입니다.

```python
first = sys.stdin.read(1)
if first == '\x1b':
    if select.select([sys.stdin], [], [], 0.01)[0]:
        second = sys.stdin.read(1)
```

방향키는 `ESC [ A` 세 바이트로 도착합니다.
그런데 `sys.stdin`은 버퍼링되는 TextIOWrapper라서
`read(1)`이 세 바이트를 전부 OS에서 꺼내 Python 내부 버퍼에 넣고
`ESC` 하나만 반환합니다.
이어지는 `select()`는 OS 버퍼만 보므로 남은 입력이 없다고 판단하고,
방향키가 조용히 버려집니다.

pseudo-terminal 하니스로 확인한 결과입니다.

```
--- after UP arrow:   {(0.0, 0.0)}      ← 명령이 바뀌지 않음
--- after LEFT arrow: {(0.0, 0.0)}
```

**조치 완료**: 버퍼링되지 않는 `os.read(fd, 16)`으로 원시 디스크립터에서
escape sequence 전체를 한 번에 읽도록 다시 작성했습니다.
현재 동작 확인 결과입니다.

```
--- after UP arrow:   {(0.2, 0.0)}
--- after LEFT arrow: {(0.0, 0.5)}
```

명령이 먹혔는지 눈으로 확인할 수 있도록 현재 속도를 한 줄로 출력하는 것도 추가했습니다.
`scripts/run_teleop.sh`가 ROS 환경까지 잡아주는 래퍼입니다.

### 4.5 RTX LiDAR가 부분 스캔만 발행

`ros2_rtx_lidar_helper` 노드의 `inputs:fullScan`이 설정되어 있지 않아
기본값 False로 동작합니다. 프레임마다 한 바퀴의 일부만 발행합니다.

Nav2를 붙이면 costmap이 장애물을 지워버려서 로봇이 사람을 관통하려 듭니다.
원본 저장소는 이 조합을 쓰지 않았으므로 드러나지 않았습니다.

**조치 완료**: `isaac_sim/utils/scene_setup.py`의 `patch_lidar_graph()`가
씬 구성 시 True로 설정합니다. 자세한 내용은 `docs/NAV2.md` 7장입니다.

### 4.6 질량 미지정

RigidBody prim에 `physics:mass`가 authored되어 있지 않습니다.
PhysX가 collision geometry와 기본 밀도로 질량을 추정합니다.
주행이 불안정하면 이 부분을 먼저 확인합니다.

---

## 5. ROS 2 배포판 차이

원본은 **Humble** 기준입니다. 우리는 Ubuntu 24.04이므로 **Jazzy**를 씁니다.

| 항목 | 원본 (Humble) | 우리 (Jazzy) | 조치 |
|---|---|---|---|
| setup.bash | `/opt/ros/humble/` | `/opt/ros/jazzy/` | 복사 시 수정 완료 |
| `pointcloud_to_laserscan` | `ros-humble-...` | `ros-jazzy-pointcloud-to-laserscan` | 설치 필요 |
| `iw_hub_navigation` | `IsaacSim-ros_workspaces/humble_ws` | `jazzy_ws` | 존재 확인 완료 |
| Nav2 params | Humble 스키마 | 일부 항목 폐기됨 | PHASE D에서 조정 |

NVIDIA의 `IsaacSim-ros_workspaces` 저장소에 `jazzy_ws/src/navigation/iw_hub_navigation`이
존재하는 것을 확인했으므로, reference 파이프라인을 Jazzy에서 재현할 수 있습니다.

### PHASE D에서 확인할 것: Nav2의 cmd_vel 메시지 타입

USD의 `ros2_subscribe_twist` 노드는 `geometry_msgs/msg/Twist`를 구독합니다.
Nav2는 버전에 따라 `/cmd_vel`에 `geometry_msgs/msg/TwistStamped`를 발행할 수 있습니다.
타입이 다르면 토픽 이름이 같아도 연결되지 않고, 오류 없이 조용히 로봇이 멈춰 있습니다.

Nav2를 붙인 직후 반드시 확인합니다.

```bash
ros2 topic info /cmd_vel --verbose
```

publisher와 subscriber의 타입이 다르면
`controller_server`의 `enable_stamped_cmd_vel`를 false로 두거나
변환 노드를 하나 끼웁니다.

### Nav2 파라미터의 Jazzy 조정 예정 항목

지금은 손대지 않습니다. PHASE D에서 Nav2가 실제로 뜨는지 먼저 확인한 뒤 조정합니다.

- `amcl_map_client`, `amcl_rclcpp_node`, `bt_navigator_rclcpp_node`,
  `controller_server_rclcpp_node`, `planner_server_rclcpp_node`
  이 다섯 섹션은 구버전 Nav2의 잔재로, 현재는 아무 노드도 읽지 않습니다.
- `bt_navigator.plugin_lib_names`가 Humble 시점의 목록입니다.
  Jazzy 기본값을 쓰도록 이 항목을 통째로 지우는 편이 안전합니다.

---

## 6. 가져온 파일 목록

| Original file | Copied to | 수정 | 수정 내용 | 사용 이유 |
|---|---|---|---|---|
| `isaac_sim/AMR_09_02_V2.usd` | `assets/amr/AMR_09_02_V2.usd` | 없음 | 원본 그대로 | AMR 본체 + 완성된 ActionGraph |
| `nav2/amr_nav2_params.yaml` | `configs/nav2/amr_nav2_params.yaml` | 헤더만 | 출처 주석 추가 | Nav2 시작점. AMR 치수에 맞춰 튜닝된 값 |
| `scripts/amr_arrow_teleop.py` | `scripts/amr_arrow_teleop.py` | 있음 | 키 입력을 `os.read`로 재작성 (4.4), 속도 표시 추가, 출처 주석 | `/cmd_vel` 수동 조작 |
| `scripts/start_scan.sh` | `scripts/start_scan.sh` | 있음 | humble → jazzy, 출처 주석 | PointCloud2 → LaserScan. Nav2 입력 |
| `scripts/start_nav2.sh` | `scripts/start_nav2.sh` | 있음 | humble → jazzy, humble_ws → jazzy_ws, 출처 주석 | Nav2 + RViz 실행 |
| `scripts/check_ros.sh` | `scripts/check_ros_topics.sh` | 있음 | 파일명 변경, humble → jazzy, 출처 주석 | 토픽 존재 확인. `check_environment.sh`와 이름이 겹쳐 rename |

가져오지 않은 파일입니다.

| Original file | 이유 |
|---|---|
| `README.md` | 우리 README가 별도로 존재 |
| `docs/HANDOFF.md` | 인수인계 메모. 필요한 내용은 이 문서에 반영 |
| `.gitignore` | 우리 것이 더 넓은 범위를 다룸 |

### Git LFS

USD가 1.2 MB입니다. GitHub의 경고선 50 MB, 제한선 100 MB에 한참 못 미칩니다.
**Git LFS는 필요하지 않습니다.**
나중에 텍스처나 고해상도 메시가 추가되어 커지면 그때 재검토합니다.

---

## 7. 출처 표기 규칙

reference 저장소에서 가져온 모든 소스 파일 상단에 다음 주석이 있습니다.

```python
# ---------------------------------------------------------------
# Adapted from: https://github.com/dongtak2/AMR-Digital-Twin
#   original file : <원본 경로>
#   commit        : c54269ee80032c29b0548eb77c3ee38d68b87e78
#   modifications : <수정 내용, 없으면 "none">
# See docs/AMR_DIGITAL_TWIN_REFERENCE.md
# ---------------------------------------------------------------
```

USD는 바이너리라 주석을 넣을 수 없으므로 이 문서의 6장 표가 유일한 기록입니다.

새로 작성한 파일에는 이 주석을 넣지 않습니다.
주석의 유무로 외부 코드와 자체 작성 코드를 구분합니다.
