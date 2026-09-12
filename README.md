# Comfort_AMR

**Human State 기반 Comfort Space를 활용한 인간-로봇 공유 작업공간의 AMR 교착 해소 및 디지털 트윈 검증**

Isaac Sim 5.1과 ROS 2 Jazzy 기반의 실험 환경입니다.

---

## 1. 연구 목적

사람과 AMR이 함께 쓰는 좁은 복도에서, 사람 주변에 고정 크기의 Comfort Space를
설정하면 로봇의 통행 가능 공간이 사라져 교착이 발생할 수 있습니다.

이 프로젝트는 사람의 상태(Human State)에 따라 Comfort Space 크기를 조절했을 때
**동일한 물리 환경에서** 통행이 다시 가능해지는지를 시뮬레이션으로 검증합니다.

---

## 2. B2와 B3의 차이

| | B2 | B3 |
|---|---|---|
| Policy | Static Comfort Space | Human State-Adaptive Comfort Space |
| Radius | 모든 사람에게 동일 | `R_i = f(state_i)` |
| 기대 결과 | 통행 불가, `PLANNING_FAILURE` 또는 `DEADLOCK` | 통행 가능, `SUCCESS` |

**두 시나리오의 차이는 Comfort Space Policy 하나뿐입니다.**
복도 형상, 사람 위치, AMR 모델, planner, 속도, timestep, seed는 모두 동일합니다.

설계 근거와 판정 기준은 [docs/EXPERIMENT.md](docs/EXPERIMENT.md)에 있습니다.

---

## 3. 시스템 구조

```
┌─────────────────────────────┐        ┌──────────────────────────────┐
│  Isaac Sim  (Python 3.11)   │        │  ROS 2 Jazzy (Python 3.12)   │
│                             │        │                              │
│  corridor / human / AMR USD │        │  comfort space publisher     │
│  differential drive         │        │  navigation                  │
│  comfort space 시각화        │        │  metrics recorder            │
│                             │        │                              │
│    isaacsim.ros2.bridge     │◄──DDS──►    rclpy nodes               │
│    (OmniGraph)              │        │                              │
└─────────────────────────────┘        └──────────────────────────────┘
              │                                       │
              └──────── comfort_core/ (순수 Python) ───┘
                        양쪽에서 import 되는 공유 로직
```

두 프로세스는 서로 다른 Python 인터프리터를 씁니다.
Isaac Sim 쪽에서 `rclpy`를 import하지 않고, ROS 2 쪽에서 `omni`를 import하지 않습니다.
이유는 [docs/SETUP.md](docs/SETUP.md)의 2장에 있습니다.

제어 경로입니다.

```
navigation node ──► /cmd_vel ──► ROS 2 Bridge ──► Differential Controller ──► Wheel Joint
AMR odometry    ◄── /odom, /tf ◄── ROS 2 Bridge ◄── Isaac Sim physics
```

정책 교체 구조입니다.

```
Navigation                      Human
    │                             │
ComfortSpacePolicy          HumanStateProvider
    ├── StaticComfortPolicy       ├── GroundTruthStateProvider   (현재)
    └── StateAdaptiveComfortPolicy└── SensorStateProvider        (향후 4D LiDAR)
```

---

## 4. Requirements

| | 버전 |
|---|---|
| OS | Ubuntu 24.04 |
| GPU | NVIDIA, 8 GB VRAM 이상, driver 535 이상 |
| Isaac Sim | 5.1 |
| ROS 2 | Jazzy |

---

## 5. Setup

```bash
git clone https://github.com/hun7407-lgtm/Comfort_AMR.git
cd Comfort_AMR
./scripts/check_environment.sh
```

실패 항목이 나오면 [docs/SETUP.md](docs/SETUP.md)를 따라 설치합니다.

---

## 6. AMR 에셋 출처

AMR 모델은 외부 저장소에서 가져옵니다.
가져온 파일, 수정 내역, 라이선스는 전부
[docs/AMR_DIGITAL_TWIN_REFERENCE.md](docs/AMR_DIGITAL_TWIN_REFERENCE.md)에 기록합니다.

외부에서 가져온 소스 파일은 상단에 출처 주석이 붙어 있습니다.
주석이 없는 파일은 이 프로젝트에서 새로 작성한 것입니다.

---

## 7. AMR 동작 확인

AMR을 Isaac Sim에 spawn하고 물리가 정상인지 검사합니다.

```bash
./scripts/run_amr_test.sh              # viewport, 검사 후 종료
./scripts/run_amr_test.sh --hold       # viewport 유지, 직접 둘러보기
./scripts/run_amr_test.sh --headless   # 창 없이 빠른 검사
```

검사 항목은 articulation 인식, 구동 조인트 2개 존재, 중력 하에서 바닥에 안착입니다.
exit code 0이면 전부 통과입니다.

## 8. ROS 2 연동 확인

터미널 두 개가 필요합니다.

터미널 1에서 시뮬레이션을 띄웁니다.

```bash
./scripts/run_sim.sh                   # viewport, Ctrl+C 로 종료
./scripts/run_sim.sh --headless        # 창 없이
```

터미널 2에서 ROS 2로 구동을 검증합니다.

```bash
./scripts/run_cmd_vel_test.sh
```

정지, 직진, 정지, 제자리 회전, 정지 순으로 명령하고
`/odom`으로 실제 이동량을 확인합니다.
exit code 0이면 전부 통과입니다.

수동으로 조작하려면 방향키 teleop을 씁니다.

```bash
./scripts/run_teleop.sh
```

위아래로 전후진, 좌우로 회전, 스페이스로 정지, q로 종료입니다.
Nav2를 쓰기 전에는 반드시 teleop을 종료하세요.
두 노드가 동시에 `/cmd_vel`을 발행하면 서로 충돌합니다.

## 9. Start에서 Goal까지 이동

터미널 1에서 시뮬레이션, 터미널 2에서 주행을 실행합니다.

```bash
./scripts/run_goto_goal.sh                 # config의 goal 사용
./scripts/run_goto_goal.sh --goal 0 3      # goal 임시 변경
```

start와 goal은 `configs/common.yaml`의 `navigation` 섹션에서 바꿉니다.
AMR은 항상 `navigation.start`에 spawn되므로 시작 위치가 어긋날 수 없습니다.

결과로 result, travel time, travel distance, detour ratio가 출력됩니다.
exit code 0이면 SUCCESS입니다.

이 컨트롤러는 planner가 아닙니다.
목표를 향해 회전하고 직진할 뿐이며 장애물을 보지 않습니다.
PHASE D에서 Nav2로 교체합니다.

복도, 스테이션, 사람은 `configs/common.yaml`에서 치수를 바꿉니다.
시뮬레이션을 띄울 때 배치가 검증되고, 사람이 벽에 박혀 있거나
goal이 복도 밖이면 오류를 내고 멈춥니다.

## 10. Comfort Space

사람마다 원형 Comfort Space가 붙습니다. 반지름은 정책이 정합니다.

| 정책 | 반지름 | 시나리오 |
|---|---|---|
| `static` | 모두 `static_radius` | B2 |
| `state_adaptive` | `state_radius[state]` | B3 |

`configs/common.yaml`의 `comfort_space.policy`를 바꾸면 즉시 반영됩니다.
시뮬레이션을 띄우면 바닥에 반투명 원이 그려지고 시작 로그에 값이 찍힙니다.

```
comfort space: policy=state_adaptive, states from ground_truth
  human a at (-2.00, +1.30) state=station_engaged comfort R=0.60 m, planner R=0.84 m
  human b at (+2.00, -1.30) state=stationary       comfort R=0.90 m, planner R=1.14 m
  with Comfort Space: 2.16 m at x=+2.00, AMR needs 0.68 m -> PASSABLE
```

원이 두 겹입니다. 진한 안쪽은 Comfort Space 반지름이고,
연한 바깥쪽은 거기에 AMR 내접 반지름을 더한 값입니다.
planner는 로봇 중심점으로 계획하므로 실제로 피해야 하는 원은 바깥쪽입니다.

**반지름의 정의**는 사람 중심에서 **AMR의 가장 가까운 부분**까지입니다.
AMR 중심까지가 아닙니다. 사람이 실제로 느끼는 거리가 그쪽이기 때문입니다.
변환은 `comfort_core/comfort_space.py`의 `costmap_radii()` 한 곳에서만 합니다.

시각화는 그리기만 합니다. 충돌체를 만들지 않으므로 꺼도 결과가 달라지지 않습니다.

## 11. Build

```bash
./scripts/build_ros2.sh
```

_(아직 미구현. ROS 2 패키지를 만드는 STEP 4에서 추가)_

## 12. Run B2

```bash
./scripts/run_b2.sh
```

_(아직 미구현. PHASE E)_

## 13. Run B3

```bash
./scripts/run_b3.sh
```

_(아직 미구현. PHASE E)_

## 14. Results

trial마다 CSV 한 행이 남습니다.

```
results/b2/trial_001.csv
results/b3/trial_001.csv
results/summary.csv
```

기록 항목은 [docs/EXPERIMENT.md](docs/EXPERIMENT.md)의 9장을 보세요.

---

## 15. Project Structure

```
Comfort_AMR/
├── assets/amr/            AMR USD. 출처는 docs/AMR_DIGITAL_TWIN_REFERENCE.md
├── comfort_core/          순수 Python. omni/rclpy 의존 없음. 양쪽에서 import
│   ├── config.py            YAML 로더                        [작성됨]
│   ├── frames.py            odom 과 world 좌표 변환           [작성됨]
│   ├── layout.py            복도 기하, 통과 가능 판정          [작성됨]
│   ├── comfort_space.py     ComfortSpacePolicy 와 두 구현     [작성됨]
│   ├── human_state.py       HumanStateProvider 와 두 구현     [작성됨]
│   └── metrics.py           trial 판정과 CSV 기록             [STEP 10]
├── configs/
│   ├── common.yaml          B2/B3 공통. geometry, AMR, 판정 threshold
│   ├── scenario_b2.yaml     static policy 만
│   ├── scenario_b3.yaml     state-adaptive policy 만
│   └── nav2/                Nav2 파라미터 (reference 저장소에서 가져옴)
├── isaac_sim/
│   ├── environments/
│   │   └── corridor.py      벽, 스테이션, 사람 배치           [작성됨]
│   ├── scripts/
│   │   ├── spawn_amr.py     STEP 3 spawn + 물리 검증          [작성됨]
│   │   └── run_amr_sim.py   상시 실행되는 시뮬레이션 진입점    [작성됨]
│   └── utils/
│       ├── scene_setup.py   ground/light/physics, AMR 배치    [작성됨]
│       └── comfort_viz.py   Comfort Space 원 그리기            [작성됨]
├── ros2_ws/src/comfort_amr/ ROS 2 패키지 (nodes, launch)
├── results/{b2,b3}/         실험 결과 CSV
├── scripts/                 실행 스크립트
└── docs/                    SETUP / EXPERIMENT / AMR reference / GIT workflow
```

`comfort_space/`, `navigation/`, `metrics/`를 최상위에 따로 두지 않고
`comfort_core/` 하나로 모은 이유가 있습니다.
이 디렉터리는 Isaac Sim의 Python 3.11과 시스템 Python 3.12 양쪽에서
import되어야 하므로, "무거운 의존성이 들어가면 안 되는 경계"를 하나로 명시한 것입니다.

시나리오별 코드 디렉터리(`scenarios/b2_static/` 등)도 두지 않습니다.
B2와 B3의 차이가 config 두 파일에만 존재해야
"정책 하나만 다르다"는 실험 조건이 구조적으로 보장되기 때문입니다.

---

## 16. 주의

config에 들어 있는 Comfort Space radius 값은 실험으로 검증된 값이 아닙니다.

> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.

현재 단계에서는 Human Detection, Human State Recognition, 4D LiDAR 인식,
Human Study를 구현하지 않습니다.
Human state는 simulation ground truth로 config에서 직접 부여합니다.
