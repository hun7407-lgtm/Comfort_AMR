# 논문용 최종 실험 시나리오

> 이 문서는 `comfort_core/scenario.py`가 무엇을 만드는지, 그리고 그것이
> 왜 그렇게 만들어졌는지를 설명합니다. 실험 판정 기준은
> `docs/EXPERIMENT.md`, 연구 설계 근거는 `docs/RESEARCH_DESIGN.md`에
> 있습니다.

---

## 1. 왜 생성기가 필요한가

B1 / B2 / B3-R / B3 네 조건의 비교는 **네 번의 실행이 같은 장면을
만났을 때만** 의미가 있습니다. 지금까지 trial 조건은 두 곳 중 하나에
있었습니다.

- `configs/factory_u.yaml` — 손으로 쓴 값. 재현은 되지만 하나뿐.
- `scripts/random_scenario.py` — 실행 시점에 뽑는 값. 다양하지만 네
  정책이 같은 것을 봤다고 보장할 수 없음.

그래서 난수 추출을 실행에서 완전히 분리했습니다.

```
ExperimentFactors        실험자가 고른 것
    |                    human_count, amr_count, policy, provider, seed
    v
ScenarioGenerator        난수가 뽑히는 유일한 지점
    |
    v
ResolvedScenario         모든 값이 확정됨. 실행 중 재추출 없음
    |
    v
configs/scenarios/*.yaml  Isaac Sim과 ROS 2 노드가 읽는 파일
```

**정책은 추출의 입력이 아닙니다.** `ExperimentFactors.draw_key`는
`(seed, human_count)`뿐이고, `amr_count`조차 별도 스트림
(`fleet_key`)에서 뽑습니다. 따라서

- 같은 seed의 B1 / B2 / B3-R / B3 → 같은 사람, 같은 보행, 같은 mission
- 같은 seed의 AMR 1 / 2 / 3 → 같은 사람, 같은 mission (대수만 추가)

가 **설계상** 보장됩니다. `scripts/make_scenario.py --check`가 이를
매번 확인합니다.

---

## 2. 여덟 명

번호가 아니라 **서 있는 자리**로 이름을 붙입니다.

| id | 위치 | 초기 state | 비고 |
|---|---|---|---|
| N1 | (-6.50, +3.01) | `station_engaged` | 북측 벨트 서쪽 끝 작업 위치 |
| N2 | (-0.50, +3.01) | `station_engaged` | 북측 벨트 중앙 |
| N3 | (+5.50, +3.01) | `station_engaged` | 북측 벨트 동쪽 끝 |
| S1 | (-6.50, -3.01) | `station_engaged` | 남측 벨트 서쪽 끝 |
| S2 | (-0.50, -3.01) | `station_engaged` | 남측 벨트 중앙 |
| S3 | (+5.50, -3.01) | `station_engaged` | 남측 벨트 동쪽 끝 |
| E1 | (-3.00, +5.60) | `stationary` | 북측 대차 블록 사이 통로 |
| E2 | (-3.00, -5.60) | `stationary` | 남측 대차 블록 사이 통로 |

좌표는 이 문서에 적힌 숫자가 아니라 **layout에서 읽어옵니다**
(`comfort_core/scenario.py: role_positions`). N1/N2/N3는
`equipment.stations`가 선언한 열 개 작업 위치 중 북측 다리의
서쪽 끝 / 중앙 / 동쪽 끝이고, 6 m 간격으로 벨트 전체를 덮습니다 —
AMR이 통로 양 끝과 가운데에서 각각 한 명씩 만나게 하기 위함입니다
(§10A: 특정 위치 하나에 결과가 종속되는 것 방지).

E1/E2는 창고측에서 **marked slot도 racking도 작업 위치도 아닌 유일한
바닥**입니다. 그래서 거기 서 있는 사람은 `station_engaged`가 아니라
`stationary`로 읽힙니다. E1은 `configs/factory_u.yaml`의 w11이 원래
서 있던 자리 그대로입니다. E2는 그 남측 대칭점입니다 — 기존 w12는
picker station(`pick_s`)에 서 있어 지도상 `station_engaged`로 읽히므로
§4가 요구하는 `stationary`가 될 수 없습니다.

### Human Density

| | 인원 | 구성 |
|---|---|---|
| Low | 4 | N2, S2, E1, E2 |
| High | 8 | N1~N3, S1~S3, E1, E2 |

Low는 "여덟 명 중 앞의 넷"이 아니라 **두 교대에 참여하는 넷**입니다.
따라서 두 조건의 차이는 *구경꾼의 수*이지 *조우 횟수*가 아닙니다 —
양쪽 모두 교대 2회, 보행자 4명입니다. 이 점은 해석할 때 중요합니다.

---

## 3. 교대 (Worker Exchange)

측당 한 번. 벨트 작업자가 통로를 건너 창고측으로 가고, 그가 자리를
비우면 창고측 작업자가 반대로 건너와 그 자리에 섭니다.

```
벨트 작업자   station_engaged -> moving -> stationary
창고 작업자   stationary      -> moving -> station_engaged
```

두 번째 사람의 출발 시각은 타이머가 아니라 **첫 번째 사람의 실제
진행**에서 계산됩니다: 자기 속도로 작업 위치를 벗어나 통로에 들어서는
데 걸리는 시간 + `handover_delay`.

### 두 사람이 서로를 통과하지 않게 하는 방법

**타이밍이 아니라 기하로** 보장합니다. 타이밍으로 하려 했더니:

- 1차 시도 (두 번째 사람이 첫 번째 도착까지 대기): 생성된 240개
  시나리오 **전부**에서 두 사람이 0.00~0.08 m까지 접근. 둘이 맞바꾸는
  지점이 같은 점이기 때문.
- 2차 시도 (끝점만 좌우로 벌림): 120개 중 61개가 0.70 m 미만. 한 명이
  통로를 따라 걷는 선을 다른 한 명이 가로지르는 **직교 교차**
  (seed 20, t=17.1 s에서 0.02 m).

그래서 각자 **목적지를 지나쳐서** 통로를 건넙니다. 벨트 작업자는
창고측 선을 spot *너머*에서 가로지르고, 창고 작업자는 벨트측 선을 post
*너머*에서 가로지릅니다. 어느 쪽의 횡단점도 상대가 걷는 구간 위에
있지 않습니다. 공유하는 점은 맞바꾸는 두 지점뿐이고 거기엔 수십 초
간격으로 도착합니다.

측정: 생성된 200개 시나리오에서 교대 쌍의 최소 접근 거리는 **0.90 m**
(= 통로 위 두 보행선 간격). 전부 통과.

---

## 4. 왜 `state_source: scripted`가 필요한가

이것이 이번 작업에서 발견한 가장 중요한 구조적 문제입니다.

`GroundTruthStateProvider`는 config의 라벨을 읽고, **그 라벨은 절대
변하지 않습니다.** 기계 앞에 서 있는 두 사람이라면 그게 사실입니다.
교대를 중심으로 짠 시나리오에서는 아닙니다.

측정: `configs/factory_u.yaml`에서 w7과 w11은 둘 다 걷는데,
`state_source: ground_truth`에서는 120 s 내내 t=0의 state를 유지합니다.
costmap에서도, 측정에서도 마찬가지입니다. 즉 **통로 한가운데를 걷는
사람에게 B3가 0.60 m를 줍니다.** B3가 존재하는 이유인 바로 그 동작을
할 수 없고, trial은 그것을 한 것처럼 기록합니다.

`ScriptedStateProvider`는 시계가 아니라 **동작**을 따릅니다.

```
moving          참 속도가 임계값 이상
시작 시         config가 선언한 state
도착 후         humans.<key>.walk.ends_as
```

도착 판정은 타이머가 아니라 위치 판정입니다 — 자기 route의 마지막
waypoint로부터 `arrive_radius` 이내. 코너에서 속도가 떨어진 보행자가
"도착"으로 읽히는 것을 막습니다. 통로 한가운데서 멈춰 선 사람은
계속 `moving`으로 남습니다 (= 큰 Comfort Space 유지).

이것은 추정기가 아니라 **oracle**입니다. 참 pose와 script를 모두
받으므로 틀릴 수 없고, 그것이 Layer 1이 필요로 하는 것입니다. 추정
오차는 `SensorStateProvider`가 모든 trial에서 따로 측정합니다.

**기존 결과는 바뀌지 않습니다.** `state_source` 기본값은 여전히
`ground_truth`이고, `truth_provider()`는 script가 없는 config에서
`GroundTruthStateProvider`를 그대로 돌려줍니다.

---

## 5. 두 Layer

State Provider만 교체하면 됩니다. 같은 시나리오 정의, 같은 사람,
같은 보행, 같은 mission을 그대로 재사용합니다.

| | Layer 1 (Policy Evaluation) | Layer 2 (Perception-Integrated) |
|---|---|---|
| `humans.state_source` | `scripted` | `sensor` |
| `perception.source` | `tf` | `lidar` |
| 제어 루프가 보는 사람 | 참 pose, 참 state | 로봇 자신의 검출 (`p1`, `p2`…) |
| 측정 | 항상 참 pose · 참 state | 동일 |

```bash
python3 scripts/make_scenario.py --seed 1 --humans 8 --amrs 1 --provider scripted   # Layer 1
python3 scripts/make_scenario.py --seed 1 --humans 8 --amrs 1 --provider sensor      # Layer 2
```

두 파일은 사람도 보행도 mission도 동일하고 위 두 줄만 다릅니다.

---

## 6. AMR mission pool

출발지와 목적지는 임의 좌표가 아니라 **빈 대차 slot**입니다.

세 가지 필터가 있고 **셋 다 선호가 아니라 제약**입니다.

| 필터 | 값 | 근거 |
|---|---|---|
| `slot_mouth_offset` | 0.165 m | slot 중앙에 주차하면 뒤 칸 적재 대차가 자기 protective field 안에 들어와 collision monitor가 회전을 포함한 모든 명령을 거부. 전 구간 주행 16 trial 중 15건이 목표 0.25 m 앞 0.25~0.29 m에서 정지 |
| `mission_standoff` | 1.80 m | 1.20 (최대 Comfort Space) + 0.24 (AMR 내접 반경) + 0.36. E1/E2 양옆 slot은 1.62 m라 B2 Comfort Space 안. 그대로 두면 "주차 실패"가 "통행 실패"로 기록됨 |
| `min_mission_distance` | 12.0 m | **확정값 아님.** 3.6 m mission(같은 블록 내 두 칸)은 5초 만에 도달하며 아무것도 시험하지 않음 |

```bash
python3 scripts/make_scenario.py --humans 8 --pool
```

8인 조건에서 pool은 10 slot / 90 쌍이고, 12 m 기준으로 58쌍(교차 50,
동일측 8)이 남습니다. 16 m 이상은 전부 교차 mission입니다.

---

## 7. AMR 대수

**`amr_count`는 기존 `fleet`로 구현합니다.** 측정 대상 로봇은 언제나
한 대뿐이고, 추가 대수는 lane network 위를 도는 fleet AMR입니다.

| | 측정 로봇 | fleet AMR |
|---|---|---|
| 주행 | Nav2 + Comfort Space costmap + comfort gate | roadmap, 운동학적 |
| 사람에게 양보 | 시나리오 정책대로 | 시나리오 정책대로 (같은 반경) |
| 서로에게 | 충돌 회피 | 충돌 회피 |
| 측정 | trial 1행 | 측정하지 않음 |

측정 로봇을 두 대로 만드는 것은 **최소 수정이 아닙니다**:

1. AMR USD는 자기 default prim 경로 `/World`에 reference되어 있고, 그
   ActionGraph의 relationship target이 `/World/AMR`,
   `/World/AMR/base_link` 절대 경로입니다.
2. topic 이름이 graph에 박혀 있습니다 (`/cmd_vel`, `/odom`, `/scan`,
   `/lidar/cloud`). namespace 입력이 없습니다.
3. ROS 쪽 전체가 단일 로봇입니다 — Nav2 파라미터 1벌, costmap 1벌,
   collision monitor 1벌, comfort gate 1개, `nav2_goal` 1개, frame 이름
   고정, trial 1행.

fleet 쪽 변경은 `start_at` 한 줄뿐입니다. 종전에는 모든 fleet AMR이
t=0에 출발해서 Nav2 bringup 45초 동안 이미 돌아다니고 있었습니다 —
측정 로봇이 만나는 교통량이 bringup에 걸린 실제 시간에 좌우됐다는
뜻입니다. 이제 `hold_until_trial`로 trial 시작 시각에 맞춰 세워 두고,
`start_at`은 시나리오가 말하는 그대로 "trial 시작 후 n초"를 뜻합니다.

생성기는 선택된 mission의 양 끝 `mission_standoff` 이내에 있는 fleet
목적지를 **그 시나리오에서만** 빼버립니다. `configs/factory_u.yaml`이
x = -4.60 slot 하나를 손으로 빼 두었던 것과 같은 이유이고, mission이
seed마다 움직이므로 제외도 같이 움직여야 합니다.

---

## 8. 무엇이 랜덤이고 무엇이 아닌가

### 실험 인자 — 랜덤화 금지

| 인자 | 수준 | 지정 방법 |
|---|---|---|
| Human Count | 4 / 8 | `--humans` |
| AMR Count | 1 / 2 / 3 | `--amrs` |
| Policy | b1 / b2 / b3r / b3 | `run_scenario.sh <policy>` (생성 파일에 없음) |
| State Provider | scripted / sensor | `--provider` |

### 랜덤 변수 — seed 하나에서 파생

| 변수 | 범위 | 설정 위치 | 기록 위치 |
|---|---|---|---|
| 북측 교대 작업자 | N1/N2/N3 (Low는 N2 고정) | — | `scenario_meta.exchanges[].belt` |
| 남측 교대 작업자 | S1/S2/S3 (Low는 S2 고정) | — | 〃 |
| 보행 속도 (4명 각각) | 0.8~1.2 m/s | `scenario_pool.walk_speed` | `humans.<id>.walk.speed` |
| 북측 교대 시각 | 8.0 ± 3.0 s | `scenario_pool.swap` | `humans.<id>.walk.start_at` |
| 남측 교대 시각 | 12.0 ± 3.0 s | `scenario_pool.swap.side_offset` | 〃 |
| AMR start/goal | mission pool | `scenario_pool.min_mission_distance` | `navigation.start/goal` |
| 추가 AMR 출발 offset | 0~6 s | `scenario_pool.amr_offset` | `fleet.amrs.<id>.start_at` |

교대 시각은 **AMR 위치와 무관**합니다. 로봇 거리로 trigger를 걸면
사람의 작업 일정이 로봇 행동의 함수가 되고, 다르게 주행하는 정책은
다른 시나리오를 만나게 되어 비교가 성립하지 않습니다.

### 이번 실험에서 랜덤화하지 않는 것

사람 위치 jitter, AMR 속도, Comfort Space 반경, 사람 크기, 공장 기하,
moving ellipse 비율. 각각 별도의 sensitivity / robustness 실험 대상.

---

## 9. 보행 속도에 대한 솔직한 한계

속도는 사람마다 뽑히고, 기록되고, **보행을 추론하는 모든 것**(state
oracle, 일정 추정, 교대 접근 거리 검사)이 그 값을 씁니다.

그러나 **Isaac Sim 캐릭터는 아직 그 속도로 걷지 않습니다.**
`omni.anim.people`의 command file에는 속도 인자가 없고, 유일한 레버는
animation graph 변수 `Walk`(0~1 blend weight)인데 확장 자신의
`base_command.walk()`가 매 tick 1.0으로 덮어씁니다
(`omni/anim/people/scripts/commands/base_command.py:182`).

`isaac_sim/utils/human_motion.py`에 override 경로를 넣어 두었습니다 —
매 simulation step, 확장이 tick한 뒤에 `Walk`를 씁니다. 하지만
**blend weight가 몇 m/s인지는 측정해야 알 수 있고**, 측정 전까지는
`humans.walk_speed.enabled: false`로 꺼져 있습니다. 기록된 속도와
실제 보행 속도가 다른 것보다, 하나의 고정 속도를 정직하게 보고하는
편이 낫기 때문입니다.

지금 상태에서 조우 timing의 변동은 **출발 시각 jitter**가 만듭니다.
`calibration`을 측정해 넣으면 속도도 함께 변합니다.

---

## 10. 사용법

```bash
# 보기만 하기 (아무것도 쓰지 않음)
python3 scripts/make_scenario.py --seed 1 --humans 8 --amrs 1 --dry-run

# mission 후보 전체와 거리 (최소 거리 결정용)
python3 scripts/make_scenario.py --humans 8 --pool

# 결정성 / 정책 독립성 증명
python3 scripts/make_scenario.py --seed 1 --humans 8 --check

# 시나리오 쓰기
python3 scripts/make_scenario.py --seed 1 --humans 8 --amrs 1

# 평면도 PNG
python3 scripts/preview_figure.py --config configs/scenarios/paper_h8_a1_scr_s0001.yaml --policy b3

# Isaac Sim에서 눈으로 확인 (Nav2 없음, 측정 없음, 바닥 표시 ON)
./scripts/preview_scenario.sh --config configs/scenarios/paper_h8_a1_scr_s0001.yaml
```

실제 실험은 평소대로:

```bash
./scripts/run_scenario.sh b3 --config configs/scenarios/paper_h8_a1_scr_s0001.yaml
./scripts/run_batch.sh --scenarios b1,b2,b3r,b3 \
    --configs configs/scenarios/paper_h8_a1_scr_s0001.yaml --repeats 10
```
