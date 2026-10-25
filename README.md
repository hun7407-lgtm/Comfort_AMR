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
변환은 `comfort_core/comfort_space.py`의 `robot_grow()` 한 곳에서만 합니다.

시각화는 그리기만 합니다. 충돌체를 만들지 않으므로 꺼도 결과가 달라지지 않습니다.

## 11. Run a scenario

```bash
./scripts/run_b1.sh              # Comfort Space 없음 (기준선)
./scripts/run_b2.sh              # Static Comfort Space
./scripts/run_b3.sh              # Human State-Adaptive Comfort Space
```

Isaac Sim, Nav2, Comfort Space 발행, 양보 게이트를 띄우고 trial 하나를
끝까지 돌린 뒤 정리합니다. 종료 코드 0은 SUCCESS입니다.

```bash
./scripts/run_b3.sh --headless                            # 창 없이
./scripts/run_b3.sh --config configs/demo_walker.yaml     # 걷는 사람 추가
./scripts/run_b3.sh --record --record-costmap             # 영상 저장
```

`--record`는 장면 영상, `--record-costmap`은 Nav2가 보는 화면을 남깁니다.
저장 위치는 `results/videos/<version>/<시나리오>/<조건>/`이고,
버전 폴더 이름은 `configs/common.yaml`의 `recording.version`입니다.

## 12. 센서로 사람 추정하기 (선택)

```bash
./scripts/run_people_tracker.sh --evaluate
```

시나리오가 도는 동안 따로 실행합니다. `/scan`에서 사람을 군집으로 찾아
위치와 속도를 추정해 `/comfort_detections`로 발행하고, `--evaluate`는
정답과 비교한 오차를 마지막에 출력합니다. B1/B2/B3 비교는 정답 기반으로
돌아가며, 이 추정은 그 옆에서 평가만 합니다.

## 13. 공장 Scene

현장 레이아웃을 참고해 만든 12 m x 12 m 베이입니다. 복도 Scene은 그대로
두고 따로 추가한 것이라, 기존 B1/B2/B3 결과는 영향을 받지 않습니다.

두 가지 배치가 있습니다.

| config | 모양 | 특징 |
|---|---|---|
| `configs/factory.yaml` | 12 × 12 m, 통로 두 줄이 양 끝에서 만나는 **순환로** | 막히면 반대쪽으로 돌아갈 수 있음 |
| `configs/factory_long.yaml` | 24 × 7.5 m, **한 줄 긴 복도** | 돌아갈 길이 없어 대피 공간에서 기다려야 함 |
| `configs/factory_u.yaml` | 24 × 16 m, **U자 조립 라인** + 바깥 순환로 | 세탁기 10공정, 작업자 12명이 등을 통로로 두고 섬 |

```bash
./scripts/build_factory_scene.sh                             # 순환형 USD 생성
./scripts/build_factory_scene.sh --config configs/factory_long.yaml
./scripts/build_factory_scene.sh --screenshot results/factory/preview.png
./scripts/view_factory_scene.sh                              # 창으로 열어서 보기/편집
./scripts/view_factory_scene.sh --config configs/factory_long.yaml
./scripts/read_factory_scene.sh [--apply]                    # 편집한 위치를 config로 역반영
./scripts/generate_map.py --config configs/factory.yaml      # Nav2 맵
```

`factory_long.yaml`과 `factory_u.yaml`은 `factory.yaml`을 상속해 배치만
바꿉니다. 모델, 도색, Comfort Space 정책, 판정 규칙은 공유하므로 세 Scene의
차이는 레이아웃뿐입니다.

### U자 라인 (`configs/factory_u.yaml`)

드럼 세탁기 라인입니다. 벨트를 U자로 접어 30 m 공정을 24 m 건물에 넣고,
부품 투입과 완성품 반출이 같은 쪽에서 일어납니다. 바깥에서 안쪽으로
**벽면 랙 → AMR 통로(1.5 m) → 작업자 → 벨트 → U 내부(공정 캐비닛·이송 커브)**
순입니다.

작업자는 벨트를 보고 서므로 **등이 AMR 통로를 향합니다.** 뒤에서 로봇이
지나가는 상황이 연출이 아니라 열 명의 정상 작업 자세입니다.

| 공정 | | 공정 | |
|---|---|---|---|
| S1 | 캐비닛/베이스 투입 | S6 | 급수 밸브·호스 |
| S2 | 외조(터브) 조립 | S7 | 배수 펌프·호스 |
| S3 | 내조·모터·벨트 | S8 | 배선·PCB·조작부 |
| S4 | 밸런스 웨이트 | S9 | 도어·개스킷·전면 패널 |
| S5 | 댐퍼·서스펜션 | S10 | 누수/시운전 검사 |

대피 공간은 **보관 쪽에만** 팝니다(`refuges.in`). 라인 쪽에 파면 사람이 서
있는 자리에 구멍을 내는 셈입니다. 부품 대차는 대피 공간과 랙 사이에 서고,
AMR이 후크로 끌어갑니다 — 실기 플랫폼의 견인장치가 그 방식입니다.

하늘(HDRI)은 **조명·반사 전용**이고 카메라에는 보이지 않습니다
(`assets.shell.sky.visible: false`). 유리창 너머로 남의 창고 사진이 보이면
곤란하지만, 끄면 바닥 광택과 알루미늄 반사가 밋밋해집니다.

구성은 `configs/factory.yaml` 하나에서 나옵니다. 설비 대부분은
`isaac_sim/environments/machines.py`가 알루미늄 프로파일로 직접 만들고
(`assets.modules`의 `build:`), 팔레트·제어반처럼 그대로 쓸 만한 것은 Isaac
에셋 서버의 모델을 참조합니다. 재질은 `materials.py`가 한곳에서 관리하며
(에폭시 녹색은 거칠기 0.08이라 반사가 생깁니다), 서버에 접속할 수 없으면
같은 치수의 상자로 자동 대체되므로 오프라인에서도 기하 검사와 실험은
그대로 돌아갑니다.

| 항목 | 값 | 이유 |
|---|---|---|
| 통로 폭 | 1.5 m | AMR 두 대, 또는 AMR 한 대와 비켜서는 사람 |
| 통로 형태 | 긴 통로 2줄 + 양 끝 가로 통로 2곳 = 순환로 | 막다른 길이 없고, 한쪽이 막히면 돌아갈 수 있음 |
| 기둥 간격 | 6.0 m | 베이 길이 12 m = 2 베이 |
| 작업 구역 | 통로 양쪽 3.0 m + 가운데 라인 3.0 m | 앞줄은 사람이 서는 설비, 뒷줄은 벽쪽 보관 |
| 조립 라인 | 컨베이어 3연장 8.2 m, 양쪽에서 작업 | 토막 여러 개가 아니라 한 줄 |
| 대피 공간 | 1.5 x 0.9 m, 8곳 (6 m 간격) | 1.5 m 통로에서는 걷는 사람을 통로 안에서 비켜줄 수 없음. 12 m 간격에서는 제때 도달하지 못해 교착 |
| 바닥 | 베이 전체 광택 녹색 에폭시 + 이중 황색 차선, 대피공간 점선, 방향 화살표 | 현장 영상 그대로. 반사가 보일 만큼 광택 |
| 설비 | 알루미늄 프로파일 조립 셀·작업대·선반·대차·조립 라인 | 40 mm 프로파일과 6 mm 투명 패널로 직접 제작 (`machines.py`) |
| 사람 | 설비 앞에 현장복 캐릭터 | 실험에서 쓰는 것과 같은 애니메이션 캐릭터 |
| 셸 | 천장까지 올라가는 벽, 천장 데크·보·퍼린·덕트·라인 조명, 벽면 창·케이블 트레이·배관·게시판·셔터·중2층 | 빈 벽/열린 하늘이 아니라 건물 안처럼 보이게 |
| 토트·상자·천장등 | Isaac 에셋 라이브러리의 실제 크레이트·KLT 빈·창고 펜던트 | 직접 그리지 않고 가져다 쓰되, 색만 현장 팔레트로 덧칠 |
| 잡동사니 | 구역에 주차된 대차, 바닥에 쌓인 크레이트, 기둥 하단 안전 표시 | 쓸고 치운 것처럼 보이지 않도록 |

작업자는 설비 앞, 통로 경계에서 0.45 m 떨어진 곳에 섭니다. 빌드할 때마다
통로에 남는 폭을 반경별로 출력합니다.

```
r 0.60  aisle_north  free 1.35 m   fits       station_engaged
r 0.90  aisle_north  free 0.90 m   fits       stationary
r 1.20  aisle_north  free 0.22 m   BLOCKED    moving / B2
```

여기서 BLOCKED는 "편하게 지나갈 틈이 없다"는 뜻이지 통행 불가가 아닙니다.
Comfort Space는 costmap에 **높지만 유한한 비용**(98)으로 들어가므로, 다른
길이 없으면 planner가 그 위로 지나갑니다. 그래서 반경은 **줄이지 않았습니다.**
`common.yaml` 값을 두 Scene이 똑같이 씁니다. 대신 B2는 시간으로 지불합니다
(아래 표).

### 실험도 이 Scene에서 돌아갑니다

```bash
./scripts/run_sim.sh --config configs/factory.yaml           # 씬만 띄우기
./scripts/run_b3.sh --headless --config configs/factory.yaml # 한 트라이얼
./scripts/run_b3.sh --headless --config configs/factory.yaml --record --record-costmap
```

측정 결과 — **이전 배치(상자 설비, 순환로 이전)에서 잰 값입니다. 지금 Scene에서는 재측정이 필요합니다.**
(각 1회, 걷는 사람 1명 + 작업자 4명, 9.6 m 주행)

| 시나리오 | 결과 | 주행 | 걷는 사람과 최소 거리 | 침범 | 접촉 |
|---|---|---|---|---|---|
| B1 | 성공 | 29.4 s | -0.058 m | 1.97 m / 4.4 s | 1.5 s |
| B2 | 성공 | 55.1 s | +1.170 m | 0.01 m / 0.1 s | 없음 |
| B3 | 성공 | 39.5 s | +0.619 m | 0.52 m / 3.5 s | 없음 |

복도(3.9 m)에서 교착이던 B2가 1.5 m 통로 + 대피 공간에서는 통과합니다.
대신 B3보다 15.6초 더 걸립니다. 영상은
`results/videos/v5_factory_bay/<시나리오>/factory/`에 있습니다.

`configs/factory.yaml`은 복도를 끄고(`corridor.enabled: false`) 같은
`humans` 항목으로 사람을 놓습니다. 콜라이더, Comfort Space, `/tf`, 상태가
전부 그 항목에서 나오므로 아래 코드는 어느 Scene인지 몰라도 됩니다.
배치된 사람은 북쪽 통로를 따라 작업자 4명(설비 앞 3명, 라인 1명)과
통로를 끝에서 끝까지 걷는 1명입니다. 맵과 Nav2 파라미터는 config에서
자동으로 공장 쪽을 고릅니다.

> 저장된 USD 안의 블록 사람은 파일을 혼자 열었을 때 보이라고 넣은 것이고,
> 실험에서는 그리지 않습니다(`build_bay(..., workers=False)`).

양보할 때 로봇이 대기할 수 있는 곳은 **통로와 대피 공간뿐**입니다. 작업
구역은 맵상 자유 공간이지만 사람이 일하는 자리이므로 후보에서 제외하고,
가는 경로까지 통로 위에 있어야 후보로 인정합니다(`comfort_core/layout.py`의
`drivable_area`). 복도 Scene에서는 이 제약이 없어 동작이 그대로입니다.

편집은 USD가 기준입니다. 창에서 설비를 끌어다 놓고 저장한 뒤
`./scripts/read_factory_scene.sh --apply`를 실행하면 그 위치가 config에
명시 좌표로 들어가고, 다시 빌드해도 같은 배치가 나옵니다. 설비 하나가
Xform 하나이므로 통째로 잡아서 옮기면 됩니다.

> 치수와 배치는 현장 도면을 참고해 의도적으로 변형한 근사치입니다.
> 도면과 사진은 저장소에 포함되지 않으며, 특정 설비를 재현하지 않습니다.

---

## 14. 3D LiDAR + SLAM 버전

같은 Scene, 같은 사람, 같은 정책을 쓰면서 **인식만** 3D로 바꾼 별도
버전입니다. 2D 파이프라인은 그대로 두었으므로 기존 결과는 영향받지 않고,
두 버전을 섞어 비교하지 않도록 녹화 폴더도 분리했습니다.

| | 2D (기존) | 3D |
|---|---|---|
| config | `configs/factory.yaml`, `factory_long.yaml` | `configs/factory_3d.yaml`, `factory_long_3d.yaml` |
| costmap 장애물 | `ObstacleLayer` ← `/scan` 한 겹 | `VoxelLayer` ← `/lidar/cloud` |
| 지도 | config에서 생성 | **SLAM으로 작성** (`nav3d/maps/`) |

```bash
./scripts/run_mapping.sh --config configs/factory_3d.yaml --headless  # 지도 한 번
./scripts/run_b3.sh      --config configs/factory_3d.yaml --headless  # 그 지도로 trial
```

지도는 **한 번만** 만들어 B1/B2/B3가 같은 파일을 읽습니다. trial마다 SLAM을
돌리면 세 조건이 서로 다른 건물에서 경로를 찾게 되고, 차이가 정책 때문인지
지도 때문인지 알 수 없습니다.

결과 — 두 레이아웃 모두 B1/B2/B3 전부 SUCCESS (카메라 + costmap 녹화본):

| 순환형 12×12 | 시간 | 이동 | c 최근접 | 접촉 | 안전계층 |
|---|---|---|---|---|---|
| B1 | 33.3 s | 14.14 m | **−0.059 m 접촉** | 2.4 s | 0.9 s |
| B2 | 55.9 s | 17.77 m | +0.774 m | 0 s | 0.1 s |
| B3 | 41.3 s | 12.91 m | +0.669 m | 0 s | 0.5 s |

| 긴 복도형 24×7.5 | 시간 | 이동 | c 최근접 | 접촉 | 안전계층 |
|---|---|---|---|---|---|
| B1 | 60.1 s | 23.84 m | **−0.056 m 접촉** | 1.7 s | 1.1 s |
| B2 | 119.6 s | 26.57 m | +0.720 m | 0 s | **11.4 s** |
| B3 | 74.7 s | 24.59 m | +0.717 m | 0 s | 1.0 s |

**돌아갈 길이 없으면 정책 차이가 훨씬 크게 벌어집니다.** 순환형에서 B2는 B3보다
14.6 s 느린데 긴 복도형에서는 44.9 s 느리고, 충돌 감시기 개입이 11.4 s 대 1.0 s로
갈립니다. 고정 1.20 m는 복도 폭이 모자랄 때 안전계층이 대신 멈추게 만들고,
상태적응은 서 있는 사람 옆에서 공간을 줄여 지나갑니다. 두 경우 모두 B2·B3의
최근접 거리는 사실상 같으므로(+0.72~0.77 m), B3가 얻는 것은 더 멀리 비켜주는
것이 아니라 **같은 거리를 지키면서 더 빨리 끝내는 것**입니다.

2D 순환형과 비교하면 최근접 거리는 모두 2 cm 안쪽으로 일치합니다
(B1 −0.062→−0.059, B2 +0.753→+0.774, B3 +0.649→+0.669). 인식을 바꿔도 정책의
효과는 같습니다.

자세한 내용은 [nav3d/README.md](nav3d/README.md),
결과 정리는 [results/notion/261006/](results/notion/261006/)에 있습니다.

## 15. Results

trial마다 CSV 한 행이 남습니다.

```
results/b2/trial_001.csv
results/b3/trial_001.csv
results/summary.csv
```

기록 항목은 [docs/EXPERIMENT.md](docs/EXPERIMENT.md)의 9장을 보세요.

---

## 16. Project Structure

```
Comfort_AMR/
├── assets/amr/            AMR USD. 출처는 docs/AMR_DIGITAL_TWIN_REFERENCE.md
├── comfort_core/          순수 Python. omni/rclpy 의존 없음. 양쪽에서 import
│   ├── config.py            YAML 로더                        [작성됨]
│   ├── frames.py            odom 과 world 좌표 변환           [작성됨]
│   ├── layout.py            복도 기하, 통과 가능 판정          [작성됨]
│   ├── comfort_space.py     ComfortSpacePolicy 와 두 구현     [작성됨]
│   ├── human_state.py       HumanStateProvider 와 두 구현     [작성됨]
│   ├── shapes.py            원과 타원 Comfort Space            [작성됨]
│   ├── clearance.py         로봇 외곽과 사람 사이 물리적 거리   [작성됨]
│   ├── intrusion.py         Comfort Space 침범 깊이            [작성됨]
│   ├── evasion.py           걷는 사람 양보: 대기 자리와 정지    [작성됨]
│   ├── factory_layout.py    공장 베이 기하, 설비 배치, 통로 폭  [작성됨]
│   ├── tracking.py          스캔 군집화와 사람 추적            [작성됨]
│   └── metrics.py           trial 판정과 CSV 기록             [STEP 10]
├── configs/
│   ├── common.yaml          공통. geometry, AMR, 판정 threshold
│   ├── scenario_b1.yaml     Comfort Space 없음 (기준선)
│   ├── scenario_b2.yaml     static policy 만
│   ├── scenario_b3.yaml     state-adaptive policy 만
│   ├── factory.yaml         공장 베이: 치수, 설비 목록, 경로
│   ├── factory_3d.yaml      같은 베이, 3D costmap + SLAM 지도
│   ├── factory_long_3d.yaml 같은 긴 복도, 3D costmap + SLAM 지도
│   ├── demo_walker.yaml     걷는 사람을 넣는 변형
│   └── nav2/                Nav2 파라미터 (reference 저장소에서 가져옴)
├── isaac_sim/
│   ├── environments/
│   │   ├── corridor.py      벽, 스테이션, 사람 배치           [작성됨]
│   │   ├── factory.py       바닥 도색, 셸, 천장, 벽 설비, 잡동사니 [작성됨]
│   │   ├── machines.py      알루미늄 프로파일 설비 6종         [작성됨]
│   │   ├── materials.py     에폭시·알루미늄·투명판 등 재질      [작성됨]
│   │   ├── props.py         Isaac 에셋 참조(피벗·회전 보정)    [작성됨]
│   │   ├── equipment.py     스테이션 배치와 콜라이더           [작성됨]
│   │   └── factory_workers.py 설비 앞 작업자                  [작성됨]
│   ├── scripts/
│   │   ├── spawn_amr.py     STEP 3 spawn + 물리 검증          [작성됨]
│   │   ├── run_amr_sim.py   상시 실행되는 시뮬레이션 진입점    [작성됨]
│   │   ├── build_factory_scene.py  공장 USD 생성              [작성됨]
│   │   ├── view_scene.py    저장된 Scene 열기                 [작성됨]
│   │   └── read_factory_scene.py   편집 결과 역반영           [작성됨]
│   └── utils/
│       ├── scene_setup.py   ground/light/physics, AMR 배치    [작성됨]
│       ├── comfort_viz.py   Comfort Space 원 그리기            [작성됨]
│       ├── human_motion.py  걷는 사람 콜라이더 동기화          [작성됨]
│       ├── human_tf.py      사람 위치 /tf 발행                 [작성됨]
│       └── recorder.py      장면 영상 프레임 저장              [작성됨]
├── nav3d/                  3D LiDAR + SLAM 버전 (2D는 그대로 둔 별도 버전)
│   ├── nav2_params_3d.template.yaml  VoxelLayer costmap
│   ├── mapping.launch.py    slam_toolbox 브링업 (매핑 전용)
│   ├── slam_params.yaml     slam_toolbox 튜닝
│   ├── patrol.py            통로 중심선 한 바퀴 (플래너 없음)
│   ├── place_map.py         저장된 지도를 월드 좌표로
│   └── maps/                SLAM이 만든 지도
├── ros2_ws/src/comfort_amr/ ROS 2 패키지 (nodes, launch)
├── results/{b2,b3}/         실험 결과 CSV
├── results/videos/          녹화 (버전/시나리오/조건별, git에는 제외)
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

## 17. 주의

config에 들어 있는 Comfort Space radius 값은 실험으로 검증된 값이 아닙니다.

> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.

현재 단계에서는 Human Detection, Human State Recognition, 4D LiDAR 인식,
Human Study를 구현하지 않습니다.
Human state는 simulation ground truth로 config에서 직접 부여합니다.
