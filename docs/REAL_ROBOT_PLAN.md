# 실기 AMR — SLAM → Human State 재현 → Comfort Space Costmap 검증

목표: 좁은 복도에서 실기 AMR을 운행하면서, 시뮬레이션에서 설계한
**Dynamic Comfort Space를 실기 Nav2 costmap에도 올릴 수 있는지** 확인한다.

최종 질문을 두 개로 분리해서 적는다. 섞으면 어디서 실패했는지 알 수 없다.

- **Q1 (메커니즘)** keepout 마스크가 실기 costmap에 정확한 위치·반경으로
  올라가고, 플래너가 그걸 피해 가는가 → Phase 6에서 **사람 없이** 끝난다.
- **Q2 (연구 결과)** B2 vs B3의 차이가 실기에서도 재현되는가 → Phase 7.

Q1이 안 되면 Q2는 의미가 없다. Q1을 사람 없이 먼저 닫는 것이 이 설계의 핵심이다.

---

## 지금 가진 것 / 없는 것

가진 것:

| | |
|---|---|
| 원격 접속 | `ssh clabil@clabil.local` (모니터 불필요) |
| 센서 | Unitree L1, 바닥에서 0.31 m — **시뮬과 장착 높이 동일** |
| 토픽 | `/cmd_vel` `/odom` `/lidar/cloud` `/joint_states` `/tf` — 시뮬과 이름 동일 |
| Comfort Space | `scripts/comfort_publisher.py` (KeepoutFilter 마스크 + CostmapFilterInfo) |
| Yield gate | `scripts/comfort_gate.py` (`/cmd_vel_nav` → `/cmd_vel`) |
| 상태 소스 seam | `comfort_core/human_state.py` — 설정/센서 교체 가능하게 이미 분리됨 |
| 3D 파이프라인 | `nav3d/` — cloud→scan→slam_toolbox, VoxelLayer costmap |

없는 것 (이 계획이 만드는 것):

- 실기 복도 지도
- 오도메트리 캘리브레이션 (바퀴 반경·트레드)
- 실기용 config (`configs/real_corridor.yaml`)
- Humble용 Nav2 파라미터 템플릿
- Isaac TF를 대신할 **실기 human TF 발행 노드**

---

## Phase 0 — 조사와 안전 (바퀴 한 바퀴도 안 돈다)

### 0.1 로봇에 뭐가 있는지 — [AMR]

```bash
ros2 pkg list | grep -E 'navigation2|nav2|slam_toolbox|pointcloud_to_laserscan'
ros2 topic list
ros2 topic echo /odom --once          # covariance, frame_id
ros2 run tf2_tools view_frames         # odom -> base_footprint 있나
ros2 topic list | grep -iE 'ultra|sonar|range'
```

마지막 줄이 중요하다. **L1은 0.31 m 아래를 원리적으로 못 본다.** 바닥에
놓인 물건은 costmap에 영원히 안 뜬다. 초음파가 토픽으로 나오면 collision
monitor에 소스로 추가한다. 안 나오면 **저속 + E-STOP 사람이 든다**가
유일한 대책이고, 이건 계획이 아니라 제약으로 적어둬야 한다.

### 0.2 Humble ↔ Jazzy DDS 통신 — [데스크톱]

```bash
export ROS_DOMAIN_ID=0        # 로봇과 같은 값
ros2 topic hz /lidar/cloud
ros2 topic echo /odom --once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'   # E-STOP 걸고
```

Humble(22.04) ↔ Jazzy(24.04) 라이브 통신은 **공식 지원이 아니다.**
bag 재생으로 점군을 본 것은 이걸 증명하지 않는다. 이 결과가 아래 구조를 정한다.

### 0.3 구조 결정 — Nav2를 어디서 돌리나

**권장: 제어 루프 전체를 Jetson(Humble)에서. 데스크톱은 RViz·bag·분석만.**

이유:
1. WiFi가 끊겨도 주행이 계속 성립한다. 데스크톱에서 돌리면 끊긴 순간
   로봇은 마지막 `/cmd_vel`을 들고 달린다 — 복잡한 환경에서 받아들일 수 없다.
2. 크로스 디스트로 리스크가 제어 루프에서 빠진다.
3. Jazzy Nav2는 `/cmd_vel`을 **TwistStamped**로 낸다. Humble 로봇은
   **Twist**를 기다린다. 온보드 Humble이면 이 변환이 애초에 없다.

비용(정직하게):
- `ros-humble-navigation2 ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan` apt 설치 — SSH로 가능, 물리적으로 안 만진다.
- **Humble용 파라미터 템플릿이 필요하다.** 플러그인 이름 표기가 다르다
  (`nav2_navfn_planner/NavfnPlanner` vs Jazzy의 `::`), Jazzy의 `navigators:`
  블록은 Humble에 없다. 속도·footprint는 이미 `${MAX_VEL_X}` `${FOOTPRINT}`
  템플릿 변수라 config만 바꾸면 된다.
- `comfort_publisher` / `comfort_gate`는 순수 rclpy + `nav2_msgs`라 Humble에서
  그대로 돈다. `KeepoutFilter`도 Humble에 있다.

0.2가 완전히 실패하면 RViz도 못 쓴다. 그때 대안은 `foxglove_bridge`
(웹소켓, 디스트로 무관).

### 0.4 안전 기준값 — `configs/real_corridor.yaml`

| 항목 | 시뮬 | 실기 |
|---|---|---|
| footprint | 0.70 × 0.48 | **0.864 × 0.570** (실측) |
| max_vel_x | 0.5 | **0.2** |
| max_vel_theta | — | 0.4 |

footprint를 안 고치면 costmap이 실제보다 작은 로봇을 피해 다닌다.
좁은 복도에서 이건 바로 접촉이다.

**Phase 0 완료 조건:** 구조가 정해지고, 초음파 유무를 알고, 안전 한계가 config에 들어갔다.

---

## Phase 1 — 오도메트리 캘리브레이션 (SLAM보다 먼저)

SLAM이 틀린 오도메트리를 받으면 지도가 휘거나 축척이 틀린다. 그 위에서
"Comfort Space 반경 0.9 m"를 논하는 건 무의미하다. **반드시 먼저 한다.**

1. 직선 2.0 m — 바닥에 테이프, 저속 전진, `/odom` 이동량 vs 줄자 → 바퀴 반경
2. 제자리 360° × 3회 — `/odom` yaw 누적 vs 실제 → 트레드(wheel separation)

오차 2 % 이내면 통과. 넘으면 bringup의 오도메트리 파라미터를 고친다.
(전부 `ros2 bag`으로 녹화해두면 나중에 시뮬 오도메트리와 비교할 수 있다.)

---

## Phase 2 — `/scan` 파이프라인 검증 (로봇 정지)

여기가 **이 센서로 2D SLAM이 되는가**의 판정 지점이다.

`/lidar/cloud` → `pointcloud_to_laserscan` → `/scan`, 슬랩은 시뮬과 동일하게
lidar_link 기준 −0.20 ~ +1.20 m.

L1은 597점/메시지를 **상반구 전체**에 뿌린다. 슬랩에 들어오는 건 그중 낮은
고도각 채널뿐이다 — 거리에 반비례한다. 다행히 **좁은 복도가 유리한 쪽이다**:
벽이 1 m에 있으면 고도각 50°까지 슬랩에 들어와 채널 대부분을 쓴다. 10 m
앞은 두어 채널뿐이다.

판정:

```bash
ros2 topic echo /scan --once | grep -c -v inf    # 유효 빔 수
```

- 유효 빔 ≥ 150개/메시지, 그리고 RViz에서 읽은 복도 폭이 줄자와 ±5 cm 이내 → 통과
- 미달이면 **2~3 스윕 누적**(≈0.3 s)해서 한 scan으로 합친다. 저속이 전제라
  0.2 m/s × 0.3 s = 6 cm 번짐, 지도 해상도 5 cm와 비슷해 허용 범위다.
- 그래도 안 되면 3D SLAM(`kiss_icp` 등) → 2D 투영으로 간다. 비용이 크므로
  마지막 수단.

---

## Phase 3 — SLAM 지도 작성 (teleop)

- `slam_toolbox` async, `nav3d/slam_params.yaml` 기반 (복도 길이에 맞게
  `loop_search_space_dimension`만 조정)
- **teleop 수동 주행, 0.2 m/s.** 자율 주행 아님. E-STOP 손에.
- 출발점으로 돌아와 루프 클로저를 만든다
- `map_saver_cli`로 저장 → `nav3d/maps/real_corridor.{pgm,yaml}`

검증: 지도에서 잰 복도 폭·길이 2~3곳을 줄자와 비교. 5 cm 이내.

**지도는 한 번만 만들고 모든 trial이 같은 파일을 읽는다.** B1/B2/B3가 서로
다른 건물에서 경로를 찾으면 차이가 정책 때문인지 지도 때문인지 알 수 없다
(`nav3d/README.md`와 같은 이유).

---

## Phase 4 — 위치추정 + Nav2 단독 주행 (사람 없음, Comfort Space 없음)

"주행 자체가 되는가"를 "Comfort Space가 되는가"와 분리한다.

- 위치추정은 **slam_toolbox localization 모드 권장** (AMCL 아님). 희소
  스캔에서는 포즈 그래프 스캔 매칭이 파티클 필터보다 안정적이고, Phase 2에서
  맞춘 스캔 파이프라인을 그대로 쓴다 — 튜닝 대상이 두 개로 늘지 않는다.
- Nav2 올리고 복도 안 3~4 m 목표 하나. 사람 없음, 마스크 없음.
- 확인: 목표 도달, costmap에 벽이 제 위치, collision monitor 오작동 없음

**완료 조건:** 왕복 3회 연속 성공, 접촉 0.

---

## Phase 5 — Human State 재현 인프라

시뮬에서 사람 위치는 ground truth였다(Isaac이 TF를 냈다). 실기에는
ground truth가 없다 — **테이프로 만든다.**

1. 바닥에 사람 위치를 테이프로 표시. 그 지점의 **지도 좌표**를 얻는다
   (로봇을 그 위로 몰고 가서 `/odom`을 읽거나, 저장된 지도에서 RViz로 클릭).
2. `configs/real_corridor.yaml`에 `humans:` — 위치 + 상태, `work_spots:`
3. **실기 human TF 발행 노드** 하나 추가. Isaac이 내던 프레임을 대신 낸다.
   `HumanTracker`는 이미 `/tf`에서 읽으므로 **Comfort Space 코드는 손 안 댄다**
   (프레임만 `world` → `map`). 이 seam이 `human_state.py`에 설계된 이유다.

상태별 난이도:

| 상태 | 재현 방법 | 난이도 |
|---|---|---|
| `station_engaged` | 작업 표시 지점에 서서 벽/설비 쪽을 본다 | 쉬움 |
| `stationary` | 복도 옆 표시 지점에 그냥 서 있는다 | 쉬움 |
| `moving` | 테이프 선을 따라 A→B, 메트로놈으로 속도 고정 | **어려움 — 마지막에** |

`moving`을 마지막에 두는 이유: 걷는 사람은 위치 오차가 그대로 들어가고,
`comfort_gate`의 회피 동작까지 엮인다. 서 있는 사람으로 Q1을 닫은 뒤에 한다.

---

## Phase 6 — Comfort Space Costmap 실기 적용 **(사람 없이)**

Q1의 답이 여기서 나온다. 사람 자리에 **아무도 안 세운다.** 마스크만 띄운다.

1. `comfort_publisher`를 실기에서 띄운다. 마스크 격자는 **SLAM 지도의
   geometry**(width/height/resolution/origin)를 읽어야 한다 — 한 셀이
   costmap 한 셀과 정확히 겹쳐야 painted radius가 설정값과 같다.
2. RViz 확인:
   - `/costmap_filter_mask`가 표시된 좌표에, 설정 반경으로 떠 있다
   - local·global costmap **둘 다**에 비용이 올라간다
   - `filter_info_topic`이 latched로 Nav2 활성화 이후에도 잡힌다
3. 목표를 마스크 건너편에 주고 플래너가 **돌아가는지** 본다
4. 정책만 바꿔(B2 → B3) 마스크 반경이 바뀌는 것을 같은 자리에서 확인

Humble의 `KeepoutFilter` 파라미터 이름이 Jazzy와 같은지 여기서 확인된다.

**완료 조건:** 마스크가 지도 좌표 ±1셀(5 cm) 안에 뜨고, 경로가 그걸 피한다.
사람은 아직 등장하지 않았다.

---

## Phase 7 — 사람 포함 trial

E-STOP을 든 사람이 **로봇을 따라다닌다.** 0.2 m/s.

**B1은 실제 사람에게 돌리지 않는다.** 시뮬에서 B1은 −0.059 m, 즉 접촉했다.
B1은 설계상 Comfort Space가 없는 조건이므로 실기에서는 사람을 치러 가는
조건이다. B1은 표시 지점에 **마네킹/박스**를 놓고 돌리거나, 시뮬 전용
baseline으로 남긴다.

순서: B2(정적) → B3(상태적응), 서 있는 사람으로. 각 3회. bag 전체 녹화.

지표는 시뮬과 동일하게: 결과, 시간, 이동거리, 최근접 거리, 접촉 시간,
침범 깊이/시간, 안전계층 개입 시간.

---

## Phase 8 — 비교와 정리

실기 vs 시뮬을 같은 표로. 여기서 비로소 **Isaac 센서를 `Unitree_L1.usda`로
교체**하는 작업이 의미를 갖는다 (현재 시뮬은 −15°~+10°, 실기는 0°~+90° —
정반대). 교체하면 기존 `nav3d` 3D 결과는 다시 받아야 한다.

---

## 나중 (이 계획 밖)

- `people_tracker.py`를 실기 `/scan`에 붙여 테이프 위치를 ground truth로
  검출 오차를 측정. 단, structure filter가 config 박스 대신 **SLAM 지도**를
  읽도록 바꿔야 한다.
- 비반복 스캐너는 프로파일이 재현하지 않는다 — 레이 사이에 낀 가는 물체는
  시뮬에서 안 보이고 실기는 다음 스윕에 잡는다.

---

## 요약: 순서와 이유

| | 단계 | 사람 | 로봇 | 왜 이 순서인가 |
|---|---|---|---|---|
| 0 | 조사·안전·구조 결정 | — | 정지 | 구조를 모르면 아래 전부가 다시 짠다 |
| 1 | 오도메트리 캘리브레이션 | — | 저속 직선/회전 | 틀린 오도메트리 위의 지도는 쓸 수 없다 |
| 2 | `/scan` 검증 | — | 정지 | 이 센서로 2D SLAM 가능한지 판정 |
| 3 | SLAM 지도 | — | teleop | 한 번만, 모든 trial 공유 |
| 4 | Nav2 단독 주행 | — | 자율 | 주행 실패와 마스크 실패를 분리 |
| 5 | human state 인프라 | — | 정지 | 테이프가 ground truth를 만든다 |
| 6 | **마스크 검증** | **없음** | 자율 | **Q1 — 사람 없이 닫는다** |
| 7 | B2/B3 trial | 서 있음 | 자율 | Q2 |
| 8 | 정리 | — | — | |

---

# Phase 0 실측 결과 (2026-10-18)

## 확정된 것

| 항목 | 결과 |
|---|---|
| nav2 / slam_toolbox / pointcloud_to_laserscan | **이미 설치됨** (Humble). 설치 비용 0 |
| 초음파 토픽 | **없음.** L1의 0.31 m 아래 사각지대를 메울 센서가 전무 |
| `/estop_active` | 토픽은 있으나 **값을 발행하지 않음** — 상태를 읽을 수 없다 |
| 복도 치수 | 폭 **약 2.1 m**, 전방 5 m / 후방 5.5 m 지점에 막힌 끝 |

## 구조 결정 — 실측으로 강제됨

**Humble과 Jazzy는 같은 DDS 도메인에 둘 수 없다.**

데스크톱(Jazzy)을 도메인 30에 넣는 순간 로봇의 모든 Humble 노드가

    'Bad alloc' exception deserializing
    rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_

를 뱉고 그래프가 죽었다. 로봇 자신의 `ros2 node list`조차 비었다. 이 타입은
두 배포판 사이에서 정의가 바뀐다. 네트워크 설정으로 고칠 수 있는 문제가 아니다.

→ **Nav2·SLAM·Comfort Space 전부 로봇에서 돌린다.** 데스크톱은
   (a) Humble 컨테이너 안의 RViz, 또는 (b) foxglove_bridge websocket으로만 본다.
   `results/real/261018/rviz_robot.sh`가 (a)이고, 검증 완료.

부수적으로, **AP가 무선 클라이언트 간 멀티캐스트를 차단**한다(유니캐스트는 통과).
그래서 컨테이너도 FastDDS 유니캐스트 프로파일(`fastdds_desktop.xml`)을 쓴다.
IP가 바뀌면 이 파일의 주소도 바꿔야 한다.

## Phase 2 통과 — 기존 지도가 망가진 원인

로봇 `.bashrc`의 `lidar` alias는 `max_height:=2.0`, `min_height` 미지정,
`range_min:=0.0`이었다. L1은 **위쪽만** 보므로 높이 0~2.0 m를 2D로 누르면
천장 구조물이 엉뚱한 수평거리의 벽으로 찍힌다. 그 결과가 `map_nsl1`의
자유공간 속 흩뿌려진 점들이고, 자유셀의 최근접 벽 거리 중앙값이 **0.16 m**였다
(로봇 내접반경 0.285 m — inflation을 먹이면 복도 전체가 통행 불가).

시뮬과 같은 슬랩(lidar_link 기준 −0.20~+1.20 m)으로 바꾼 결과:

| | 기준 | 실측 |
|---|---|---|
| 유효 빔 / 메시지 | ≥150 | **230** (min 207, max 237) |
| 3스윕 누적 | — | 526 |

`results/real/261018/scan_corridor_261018.png` — 벽 두 줄이 직선으로 깔끔하고
흩뿌린 점이 없다.

## 복도 폭 2.1 m가 의미하는 것

로봇 폭 0.57 m를 빼면 양쪽 0.77 m씩이다. `stationary`(0.90 m) 한 명이 복도를
완전히 막고, `station_engaged`(0.60 m)여야 겨우 틈이 생긴다. 상태별 costmap
세 장에서 이 차이가 그대로 드러날 것이다.

## 미해결 — 현재 블로커

**모터 제어 보드가 응답하지 않는다.**

    /battery_voltage    0.0      (2026-10-11에는 24.0 V)
    /battery_percent    0        (            "      100 %)
    /motor_encoder_sub  nan, nan
    board_status.py     전 구간 "응답 없음"

USB는 enumerate된다(`1a86:55d3` → `/dev/ttyACM0`). E-STOP은 눌려 있지 않다.
10월 11일에는 보드가 최소한 `BLOCK`이라고 응답했으므로, 그때와 다른 상태다.

엔코더가 없으면 `/odom`이 NaN이고, slam_toolbox는 한 칸도 만들지 못한다.
누가 로봇을 운전하든(스크립트든 조종기든) 이것부터 풀려야 한다.

가설: 조종기에 **수동/자율 모드 스위치**가 있고, 수동 모드에서는 보드가 시리얼
명령에 응답하지 않는다. 확인 방법 — 조종기로 로봇이 움직이는가.

대비책(보드가 끝내 안 살아날 경우): `rf2o_laser_odometry`를 소스 빌드해
`/scan`만으로 주행거리계를 만든다. apt에 없다.

## 운용 메모 (오늘 추가)

- `pkill -f "<패턴>"`을 **ssh 명령줄에 직접 쓰면 안 된다.** 패턴 문자열이
  ssh 세션 자신의 셸 명령줄에 들어 있어 자기 자신을 죽인다. 두 번 당했다.
  → `~/stop_all.sh` / `~/start_all.sh` 처럼 **파일로** 둔다.
- tmux 세션을 죽여도 `ros2 launch`의 자식들은 고아로 남는다. 노드가 두 벌
  뜨면 모터 컨트롤러 둘이 `/cmd_vel`을 먹고 `/odom`·`/tf`를 둘이 쏜다.
  반드시 `stop_all.sh`로 정리한다.
- 로봇 스크립트: `start_all.sh` `stop_all.sh` `start_scan.sh` `start_mapping.sh`
  `save_map.sh` `teleop.sh` `drive.py` `board_status.py` `robot_check.py`
