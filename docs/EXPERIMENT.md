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

네 가지가 하나의 표현을 공유합니다. 전부 `comfort_core/shapes.py`의
`ComfortShape` 하나로 표현되고, 정책이 치수만 다르게 요구합니다.

| 조건 | 정책 이름 | 모양 |
|---|---|---|
| B1 | `none` | 없음 |
| B2 | `static` | 모든 사람 같은 원 |
| B3-R | `state_adaptive` + `shape: circle` | 상태별 반경, 원 |
| B3 | `state_adaptive` + `shape: ellipse` | 상태별 반경, moving만 진행방향 타원 |
| B3-D | `state_directional` | 상태 + **방향** + 속도, offset 타원 |

### 문헌이 뒷받침하는 것 (정성적 관계)

```
서 있는 사람(주변을 볼 수 있음)   Rear > Side > Front
작업 중인 사람(등 뒤를 못 봄)     Front ≥ Side ≥ Rear   (뒤쪽 압축)
걷는 사람                          Front > Side ≈ Rear
속도가 빠를수록                    같은 거리에서 쾌적도 하락 / 예견적 공간 확대
```

근거:

- **Neggers et al.**, *Determining Shape and Size of Personal Space of a
  Human when Passed by a Robot* — 사람의 **orientation**에 따라 personal
  space의 모양과 크기가 달라지고, **후방 통과가 더 불편**하게 평가됨.
- *The effect of robot speed on comfortable passing distances* — **로봇
  속도**가 쾌적한 통과 거리와 지각된 쾌적도에 영향.
- **Hayduk, 1981** — 시각적으로 접근할 수 없는 **등 뒤의 personal space는
  압축**됨. **Hecht et al., 2019** (*The shape of personal space*, Acta
  Psychologica)도 대상의 뒤쪽에 앞쪽보다 가까이 섬. 작업 중인 사람의
  뒤쪽을 작게 두는 근거이며, `state_adaptive`가 `station_engaged`에 작은
  반경을 주는 전제와 같다. 후방에 대해 문헌이 갈리므로(Neggers: 뒤가 더
  불편 / Hayduk: 안 보이는 뒤는 압축) 이 저장소는 **레인을 볼 수 있는
  사람에게 Neggers를, 볼 수 없는 사람에게 Hayduk을** 적용한다.
  2026-11-05 이전에는 작업 중인 사람에게도 서 있는 사람의 규칙(F 0.70
  S 0.80 R 0.95)을 적용했고 `results/campaign_261105`의 B3-D 행은 그
  값으로 측정되었다.
- **Karamouzas et al., 2014**, *Universal Power Law Governing Pedestrian
  Interactions* — 보행자 상호작용이 현재 거리가 아니라 **예상 충돌
  시점(TTC)** 에 반응한다는 근거.

이 논문들은 **관계의 방향**을 보여줍니다. 아래 미터 값과 속도 매핑 식은
이 저장소의 **공학적 선택**이며, 해당 논문이 제시한 수치가 아닙니다.

### Engineering Parameters

> **literature-anchored engineering parameters**
>
> 실험으로 검증된 쾌적 거리가 **아니고**, ISO 거리도 **아닙니다**.
> 전부 `comfort_space.directional`에서 변경 가능하며 하드코딩이 없습니다.

```
station_engaged   F 0.80   S 0.70   R 0.60     heading: body   (R = B3의 engaged 반경)
stationary        F 0.80   S 0.90   R 1.05     heading: body
moving @ 1 m/s    F 1.20   S 0.90   R 0.90     heading: velocity
```

moving은 속도 v에서:

```
Front(v) = clip(0.90 + 0.30 v, 0.95, 1.35)
Side(v)  = clip(0.80 + 0.10 v, 0.80, 1.00)
Rear(v)  = clip(0.80 + 0.10 v, 0.80, 1.00)
```

### offset 타원

`front != rear`는 사람을 중심에 둔 타원으로 표현할 수 없습니다. 그래서
타원을 heading 방향으로 밀어냅니다:

```
along  = (front + rear) / 2
across = side
offset = (front - rear) / 2
```

정지 상태(F 0.80 / R 1.05)면 `offset = -0.125`, 즉 타원 중심이 사람보다
**0.125 m 뒤**에 놓입니다. `grown(g)`는 두 반축에 같은 g를 더하고 offset은
건드리지 않으므로 front·side·rear가 **각각 g만큼** 자라고 비대칭이
그대로 보존됩니다.

`across`는 **타원 중심**에서의 최대 반폭입니다. 사람 위치에서의 반폭은
`across · sqrt(1 - (offset/along)²)` 로 조금 작습니다 — 위 값에서 0.892 m,
8 mm 차이. 보정하지 않고 기록해둡니다. 플래너와 costmap이 쓰는 값은
최대 반폭이기 때문입니다.

### Heading 출처

```
station_engaged / stationary   body heading (humans.<key>.facing_deg)
moving                         velocity heading (atan2(vy, vx))
```

속도가 `heading_min_speed_mps`(0.10) 아래로 떨어지면 **마지막으로 믿을 수
있었던 진행 방향을 유지**합니다. body heading으로 되돌리는 방식은 정지
순간에 방향이 튀므로 쓰지 않습니다. 상태가 `moving`에서 벗어나면 진행
방향을 즉시 버리고 body heading으로 갑니다 — 걷던 방향으로 rear-biased
타원이 남는 일이 없습니다. (`comfort_core/comfort_space.py`의
`HeadingResolver`)

방향을 아는 수단이 **없는** 사람 — 위치만 주는 라이다 검출이 그렇습니다 —
에게는 방향을 만들어내지 않고, 최대 extent의 **원**을 줍니다.

### 안 섞는 것

**Comfort Space와 물리 안전거리는 완전히 분리됩니다.** 보호필드는
`navigation.collision_monitor`에서 로봇 자신의 제동거리로 산출되고, Comfort
Space를 줄여도 **절대 함께 줄어들지 않습니다**.

`comfort_space.aisle_scaling`(기본 off)은 좁은 통로에 맞춰 Comfort Space를
한 번에 좁히는 수단이고, 모든 상태에 같은 scale을 적용해 상태 간 관계를
바꾸지 않으며, 안전거리에는 적용되지 않습니다.

### 확장 지점 (미구현)

`comfort_space.stimulus_adaptation`(기본 off)은 TTC 기반 적응을 위한
자리입니다. 기하에 섞지 않습니다 — 접근 속도에 따라 모양이 커졌다 작아지면
Comfort Space가 "상태·방향·속도에 대해 하나의 크기"를 갖지 않게 되고,
이 실험이 측정하려는 기여가 분리되지 않습니다. 안전층을 대체해서도 안 됩니다.

### 그리기와 cost의 분리

시각화와 navigation cost 처리는 코드상 분리합니다.
Comfort Space를 viewport에 그리는 코드가 navigation 결과에 영향을 주면 안 됩니다.
`isaac_sim/utils/comfort_viz.py`는 collider도 rigid body도 만들지 않고,
`check_below_scan()`이 그려진 원반이 레이저 슬라이스 아래에 있음을 검사합니다.

---

## 9. Trial 판정

각 trial은 아래 넷 중 정확히 하나로 분류됩니다.
판정 순서가 중요합니다. 위에서부터 먼저 성립하는 것을 채택합니다.

| 결과 | 조건 |
|---|---|
| `PLANNING_FAILURE` | planner가 feasible path를 한 번도 생성하지 못함 |
| `SUCCESS` | Nav2가 SUCCEEDED를 보고하고 base가 goal tolerance + 0.05 m 이내 (Nav2의 goal checker가 같은 tolerance를 이미 적용했고, 이 스크립트는 /tf에서 잠시 뒤 다시 재므로 mm 단위 차이가 난다. 그 이상 어긋나면 `INVALID`이지 교착이 아니다. `GOAL_POSE_SLACK_M`) |
| `DEADLOCK` | path는 존재하나 `deadlock_duration` 동안 선속도 < `v_threshold` 이고 변위 < `d_threshold`. base가 이미 goal tolerance (+0.05 m) 안에 있는 동안은 무진전 타이머를 멈춘다 — 마지막 heading 정렬 중인 로봇은 교착이 아니고, Nav2의 결과가 판정한다 (2026-11-05, cross_h8 검증 trial에서 Nav2의 "Reached the goal"과 12 s 규칙이 20 ms 차로 경합해 0.23 m 지점의 도착이 DEADLOCK으로 기록된 뒤 추가) |
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

### 세 대일 때의 판정

판정은 **차량마다** 내립니다. AMR1은 `trials.csv`, a1/a2는
`fleet_trials.csv`에 각자 `result`(SUCCESS / DEADLOCK / ABORTED)를
적고, trial 하나에 합성 판정을 두지 않습니다. 교착 규칙은 세 대가
같습니다(`navigation.deadlock`). 성공률은 정책별로 차량을 합쳐 셉니다.
docs/NAV2.md 5c절 "차량마다 자기 판정".

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

### 플릿 차량의 기록

a1, a2(가 Nav2로 달릴 때)는 같은 잣대 -- 같은 사람 프레임, 같은 기준
Comfort Space, 각자의 안전층 -- 로 `fleet_trials.csv`에 차량당 한 줄로
기록됩니다 (`comfort_core/metrics.py` `FLEET_FIELDS`, docs/NAV2.md
5c절 5단계). 세 대는 같은 정책 아래 같은 잣대로 재는 동등한 대상이고,
파일이 다른 것은 열(mission vs job list)이 달라서입니다.
