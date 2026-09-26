# Experiment Definition: B2 vs B3

> 이 문서가 실험의 기준입니다. 구현은 `scripts/run_b1.sh`,
> `scripts/run_b2.sh`, `scripts/run_b3.sh`로 돌아갑니다.

---

## 1. 연구 질문

정적인 Comfort Space를 쓰면 인간과 로봇이 공유하는 좁은 복도에서
AMR이 통행 불가 상태에 빠질 수 있다.
사람의 상태(Human State)에 따라 Comfort Space 크기를 조절하면
같은 물리 환경에서 통행이 다시 가능해지는가.

---

## 2. 통제 변인

B2와 B3에서 **반드시 동일해야 하는** 항목입니다.
이 값들은 전부 `configs/common.yaml` 하나에만 존재하며,
시나리오 config가 이를 덮어쓰지 못하게 합니다.

- Corridor geometry (length, width, wall)
- Station A / Station B 위치
- Human A / Human B 위치
- Human A / Human B state
- AMR 모델, start, goal
- Planner, controller, 속도 제한
- Simulation timestep
- 정적 장애물
- Random seed
- 판정 threshold (goal tolerance, deadlock, timeout)

## 3. 조작 변인

Comfort Space Policy **하나뿐**입니다.

| | B1 | B2 | B3 |
|---|---|---|---|
| Policy | `NoComfortPolicy` | `StaticComfortPolicy` | `StateAdaptiveComfortPolicy` |
| Radius 결정 | 없음 | 모든 사람에게 동일한 `static_radius` | `R_i = f(state_i)` |
| 양보 게이트 | 꺼짐 | 켜짐 | 켜짐 |
| Config | `configs/scenario_b1.yaml` | `configs/scenario_b2.yaml` | `configs/scenario_b3.yaml` |

B1은 기준선입니다. 사람은 LiDAR에 보이는 장애물일 뿐이고, Comfort Space도
양보도 없습니다. 침범량은 세 시나리오 모두 같은 잣대로 재야 비교가 되므로
`evaluation.comfort_reference`가 가리키는 정책(기본값 B3)의 Comfort Space로
측정합니다.

두 시나리오 config에는 policy 관련 키만 들어갑니다.
geometry나 planner 값이 시나리오 config에 나타나면 설계 위반입니다.

---

## 4. 환경 치수

전부 `configs/common.yaml`에서 조절합니다. 코드에 숫자가 없습니다.

| 항목 | 키 | 현재 값 |
|---|---|---|
| 복도 길이 | `corridor.length` | 12.0 m |
| 복도 폭 | `corridor.width` | 4.0 m |
| 벽 높이 / 두께 | `corridor.wall_height` / `wall_thickness` | 1.5 / 0.1 m |
| Station 크기 | `stations.size` | 0.8 × 0.6 × 0.9 m |
| Station A | `stations.a.position` | (-2.0, +1.7) |
| Station B | `stations.b.position` | (+2.0, -1.7) |
| Human 반지름 / 키 | `humans.radius` / `height` | 0.25 / 1.70 m |
| Human A | `humans.a.position` | (-2.0, +1.3) |
| Human B | `humans.b.position` | (+2.0, -1.3) |
| Start | `navigation.start` | (-5.0, 0.0) |
| Goal | `navigation.goal` | (+5.0, 0.0) |
| AMR footprint | `amr.footprint_length` / `footprint_width` | 0.70 × 0.48 m |
| 통과 여유 | `navigation.passage_margin` | 0.10 m |

## 5. 통과 가능 판정

`comfort_core/layout.py`의 `free_gap()`이 복도 y축에서 어떤 Comfort Space에도
덮이지 않는 가장 넓은 띠를 계산합니다.
`required_width()`는 AMR이 필요로 하는 폭입니다.

```
free_gap >= required_width   →  통과 가능
free_gap <  required_width   →  통과 불가
```

현재 치수에서의 계산 결과입니다.

| 조건 | free gap | 통과 |
|---|---|---|
| Comfort Space 없음 (몸통만) | 2.10 m | 가능 |
| B2, 양쪽 반지름 1.2 m | 0.20 m | **불가** |
| B3, A는 0.6 m B는 1.2 m | 0.80 m | 가능 |

AMR이 필요로 하는 폭은 0.68 m입니다.

위 반지름은 예시입니다. 실제 값은 STEP 7 이후 시나리오 config에서 정합니다.

## 6. 환경 개념도

```
        Wall
  ┌──────────────────────────────────────────┐
  │                                          │
  │        [Station A]                       │
  │        (Human A)                         │
  │                                          │
  │  START  ──────────►──────────      GOAL  │
  │                                          │
  │                       (Human B)          │
  │                       [Station B]        │
  │                                          │
  └──────────────────────────────────────────┘
        Wall
```

Station A와 Station B는 복도 양쪽에 엇갈리게(staggered) 배치합니다.
두 Comfort Space가 복도 폭 방향으로 겹치면 통행 가능 공간이 사라집니다.

통행 가능 여부의 기하학적 조건입니다.
Comfort Space 원 두 개와 양쪽 벽 사이에 남는 가장 좁은 간격을
`gap`이라 하고, AMR 반경에 안전여유를 더한 값을 `w_amr`이라 하면

```
gap >= w_amr   →  통행 가능
gap <  w_amr   →  통행 불가
```

B2는 `gap < w_amr`이 되도록, B3는 state에 따라 반경이 줄어
`gap >= w_amr`이 되도록 config 값을 정합니다.

---

## 7. Human State

현재 단계에서는 센서 인식을 구현하지 않고
`GroundTruthStateProvider`가 config에서 state를 그대로 읽습니다.

| State | 의미 |
|---|---|
| `moving` | 사람이 복도를 이동 중 |
| `stationary` | 정지해 있으나 특정 작업에 몰입하지 않음 |
| `station_engaged` | 스테이션 쪽을 향해 작업 중, 복도 방향 주의가 낮음 |

B2/B3 비교에서는 우선 `stationary`와 `station_engaged`만 사용합니다.

향후 4D LiDAR 기반 인식을 붙일 때는 `SensorStateProvider`를 구현해
`HumanStateProvider` 인터페이스만 교체합니다.
navigation 코드는 바뀌지 않습니다.

---

## 8. Comfort Space 모델

원형 하나만 사용합니다.

```
C_i = { p | ||p - p_human_i|| <= R_i }
```

Gaussian이나 egg-shape 모델은 이번 단계에서 구현하지 않습니다.

> **주의**
> 현재 config의 radius 값은 실험으로 검증된 인간 쾌적 거리가 아닙니다.
> Initial engineering parameter for prototype validation.
> Not an experimentally validated human comfort distance.

시각화와 navigation cost 처리는 코드상 분리합니다.
Comfort Space를 viewport에 그리는 코드가 navigation 결과에 영향을 주면 안 됩니다.

---

## 9. Trial 판정

각 trial은 아래 넷 중 정확히 하나로 분류됩니다.
판정 순서가 중요합니다. 위에서부터 먼저 성립하는 것을 채택합니다.

| 결과 | 조건 |
|---|---|
| `PLANNING_FAILURE` | planner가 feasible path를 한 번도 생성하지 못함 |
| `SUCCESS` | AMR이 goal tolerance 이내 도달 |
| `DEADLOCK` | path는 존재하나 `deadlock_duration` 동안 선속도 < `v_threshold` 이고 변위 < `d_threshold` |
| `TIMEOUT` | 경과 시간이 `max_trial_time` 초과 |

현재 구현 상태입니다.

| 결과 | 구현 |
|---|---|
| `SUCCESS` | 구현됨 |
| `TIMEOUT` | 구현됨 |
| `DEADLOCK` | 구현됨. 양보로 인한 정체와 진척 없음을 따로 봅니다 |
| `PLANNING_FAILURE` | 구현됨. 출발 전 planner에 경로를 직접 물어봅니다 |

trial마다 사람별 최소 물리 거리, 겹친 시간, Comfort Space 침범 깊이와
머문 시간, 비켜선 시간도 함께 출력합니다.

`PLANNING_FAILURE`와 `DEADLOCK`을 구분하는 것이 핵심입니다.
전자는 경로 자체가 만들어지지 않은 것이고,
후자는 경로는 있으나 실제 주행이 정체된 것입니다.

threshold는 전부 `configs/common.yaml`에 두어
B2와 B3가 같은 기준으로 판정되게 합니다.

---

## 10. 기대 결과

| | B2 | B3 |
|---|---|---|
| 통행 공간 | 막힘 | 열림 |
| 예상 결과 | `PLANNING_FAILURE` 또는 `DEADLOCK` | `SUCCESS` |

### Trial 초기 상태

trial마다 Isaac Sim을 새로 띄웁니다.
로봇을 제자리로 되돌리는 리셋 기능을 만들지 않는 이유가 있습니다.
장애물에 눌린 뒤의 물리 상태는 위치만 되돌린다고 사라지지 않고,
B2와 B3가 서로 다른 초기 조건에서 출발하면 비교가 성립하지 않습니다.
프로세스를 새로 시작하는 것이 동일한 초기 조건을 보장하는 가장 확실한 방법입니다.

> **공정성 조건**
> B3가 유리해지도록 planner 파라미터나 환경을 바꾸면 안 됩니다.
> 두 시나리오의 diff는 Comfort Space Policy 하나여야 합니다.
> 실험 후 `diff configs/scenario_b2.yaml configs/scenario_b3.yaml`로 확인합니다.

---

## 11. 기록 항목

trial마다 CSV 한 행을 남깁니다.

필수 항목입니다.

```
trial_id, scenario, policy, start_time, end_time, result,
goal_reached, planning_success, deadlock_detected, timeout,
travel_time, travel_distance, waiting_time, replanning_count
```

추가 항목입니다. 나중에 파라미터를 바꿔가며 반복 실험할 때
어떤 설정에서 나온 결과인지 추적하기 위해 필요합니다.

```
human_a_state, human_b_state, human_a_radius, human_b_radius,
corridor_width, station_spacing, seed
```

저장 위치입니다.

```
results/b2/trial_001.csv
results/b3/trial_001.csv
results/summary.csv
```
