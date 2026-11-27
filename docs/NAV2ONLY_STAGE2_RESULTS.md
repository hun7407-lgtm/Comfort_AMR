# Nav2-only Stage 1 재검증 + Stage 2 uniform-scale 본실험 — 결과 보고 (약식, 2026-11-27)

설계 기준: [NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md](NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md).
진행 기록·사전등록 규칙: [NAV2ONLY_RUN_LOG.md](NAV2ONLY_RUN_LOG.md).
**약식(ABBREVIATED)**: 사용자 요청("내일 오후 8시에는 마무리되도록 약식으로")으로 seed·반복 수를 줄였습니다. 모든 수치는 잠정값입니다.

> 예측 comfort = 공개된 comfort–거리 곡선(Neggers 2022, `comfort_core/comfort_reference.py`)을 **실제 최근접 중심거리와 방위**에 대입한 예측치입니다.
> 사람의 comfort를 측정한 값이 아니며, experimentally validated comfort distance가 아닙니다.
> "가장 작은 α까지 통과"는 comfort 하한이 아닙니다.

## 1. 결론 요약

| 상태 | 판정 (C\* = 5.0) | α\* | 실제 반경 (α\*, F/S/R m) | 근거 |
|---|---|---|---|---|
| station_engaged (작업 중 사람) | **COMMON_ALPHA_IDENTIFIED** (구간 확인, 확인실험 재현) | **0.60** | 0.570 / 0.499 / 0.428 | 작업 중 사람 뒤(EN-R) 하나의 상황 |
| stationary (서 있는 사람) | **UNIDENTIFIABLE_GEOMETRY** | — | — | 앞(ST-F)은 시험할 환경을 못 정함. 옆·뒤는 α 0.45까지 통과(하한 미관측) |
| moving (걷는 사람) | **UNIDENTIFIABLE_GEOMETRY** | — | — | α를 바꿔도 실제 거리가 0.074 m만 변함 |

- **Stage 1 재검증 (Nav2-only):** R_nav_upper = **0.95 m**, transition 1.00 m (n = 10 / 크기).
  - legacy 0.90 m는 gate 체인 결과이므로 재사용하지 않았습니다.
  - 0.95 m는 Stage 2의 R_ref(정규화 기준)로 썼습니다.
- **안전:** Comfort Space가 있는 조건(B2, α 축소)에서는 사람–AMR 접촉이 **0건**입니다 (pilot·coarse·confirm 390 launch 중 B2·S3 318 launch).
  - 접촉은 모두 B1(Comfort Space 없음)에서 났습니다 (72 launch 중 16).
  - Physical safety layer(collision monitor, footprint)는 바꾸지 않았습니다.
- **Trade-off:** 2.2 m 통로의 작업 중 사람 뒤에서 B2 0.95 m는 coarse 4/5, confirm 10/10이 벽 앞 protective stop 교착이었습니다. α 0.45–1.00은 모두 완주했습니다.
  - 완주한 경우 주행시간은 α와 무관하게 약 16 s입니다.
  - Trade-off는 "시간"이 아니라 **좁은 통로를 지나갈 수 있는지 여부**로 나타납니다.

## 2. Nav2-only 실행 증거

- 모든 AMR이 Nav2 controller / behavior_server → `cmd_vel_safe` → collision_monitor → `cmd_vel` 순서로 명령을 보냅니다.
- `comfort_gate.py`를 띄우지 않으므로 Yield Gate / Evasion / Backoff / Off-lane은 없습니다.
- Comfort Space는 costmap mask(값 98)로만 Nav2에 전달됩니다. buffer 0.10 m입니다.
- 매 trial 시작 시 `scripts/nav2_goal.py`가 실제 ROS graph를 검사합니다. 위반이 있으면 INVALID입니다.
  - comfort_gate 노드가 없을 것
  - `cmd_vel_safe` 발행자는 controller_server / behavior_server뿐일 것
  - `cmd_vel` 발행자는 collision_monitor뿐일 것
  - `cmd_vel_nav` 발행자가 없을 것
- 같은 검사를 병합된 config에도 합니다.
- 결과: Stage 1 50, pilot 110, coarse 180, confirm 100 launch 모두 `control.mode = nav2_only`, 위반 0건, INVALID 0건, 인프라 제외 0건입니다.
- 안전 smoke (`results/batches/261126_nav2only_runtime_smoke`):
  - 같은 차선 정면 보행자 시험에서 collision monitor STOP 2.07 s가 걸렸고, AMR 속도는 0.79 → 0.06 m/s로 줄었습니다.
  - 그 뒤 AMR을 피하지 않는 스크립트 보행자가 정지한 AMR을 밀었습니다.

## 3. Stage 1 재검증 — `results/batches/261126_nav2only_stage1`

Seed 1–10 × {B1, B2 0.85, 0.90, 0.95, 1.00}입니다. 사전등록 규칙은 `scripts/nav2only_stage1_report.py` docstring에 있습니다.

| 조건 | system success | comfort-mask no-progress | 접촉 trial | 통과 |
|---|---|---|---|---|
| B1 | 8/10 (49–94 %) | 0/10 | 9/10 | — |
| B2 0.85 | 10/10 (72–100 %) | 0/10 | 9/10 | ✔ |
| B2 0.90 | 10/10 | 0/10 | 9/10 | ✔ |
| B2 0.95 | 10/10 | 0/10 | 9/10 | ✔ |
| B2 1.00 | 8/10 (ROBOT_ROBOT_BLOCK, HUMAN_MOTION_BLOCK) | 0/10 | 10/10 | ✘ (접촉 10 > B1 9) |

- **1.00 m 탈락의 근거는 얇습니다.** 성공률만 보면 B1과 같은 8/10이라 통과입니다. 접촉 trial이 1건 더 많아서 탈락했습니다.
- 접촉은 거의 모두 fleet AMR a2(원래부터 gate가 없던 로봇)와 사람을 피하지 않는 보행자 사이에서 납니다.
  - legacy Stage 1(같은 seed)에서도 a2 접촉은 8–10/10이었습니다. 장면의 성질이며, Nav2-only로 바꿔서 생긴 것이 아닙니다.
- n = 10으로는 5 % 비율을 구분할 수 없습니다.

## 4. Stage 2 설계 (요약)

- **α 스케일:** E(α) = α · R_ref · E_raw / max(E_raw), R_ref = 0.95 m. F:S:R 비율은 유지합니다.
  - α 격자 1.00, 0.90, …, 0.50, 0.45. 최소 α 0.45는 min_radius 0.30 m 아래로 잘리지 않게 정했습니다(0.30 / (0.75·0.95) = 0.421).
  - α = 1에서의 반경 (F/S/R, m):

    | 상태 | F | S | R |
    |---|---|---|---|
    | 작업 중 사람 | 0.950 | 0.831 | 0.713 |
    | 서 있는 사람 | 0.724 | 0.814 | 0.950 |
    | 걷는 사람 (1.0 m/s 고정) | 0.950 | 0.713 | 0.713 |

- **비교군:** B1(Comfort Space 없음), B2(원형 0.95 m).
- **같은 상황 안에서는** physical signature hash(환경·제어·buffer·안전층)가 모든 조건에서 같은지 생성기가 검사합니다.
- **판정 J:** 다음 조건을 모두 만족하면 J = 1입니다.
  - 목표 도달
  - 접촉 없음
  - 지정된 쪽으로 통과
  - 의도한 방향(앞/옆/뒤)
  - 사람 상태 일치
  - 예측 comfort ≥ C\*
- **α\* 선정:** joint ≥ 0.80 그리고 nav ≥ 0.80을 만족하는 가장 작은 α. 단조성은 가정하지 않습니다.
- **식별 상태:**
  - IDENTIFIED_BRACKET: 바로 아래 α가 comfort 때문에 실패
  - FLOOR_LIMITED: 가장 작은 α까지 통과
  - UNIDENTIFIABLE: α 1.00 → 최소 α 사이 거리 반응 < 0.10 m
  - 상태 단위 판정은 그 상태에 속한 상황 중 하나라도 UNIDENTIFIABLE이면 UNIDENTIFIABLE이 됩니다.
- **시나리오:**
  - 서 있는 사람 앞(ST-F) / 옆(ST-S) / 뒤(ST-R)
  - 작업 중 사람 뒤(EN-R)
  - 걷는 사람 교행(MV-P): 좌우 차선을 분리하고, 사람과 AMR이 서로 반대 방향으로 갑니다.
  - 추월(MV-O)은 걸음 속도 1.04 m/s > AMR 최대 1.0 m/s라 불가능하여 실행하지 않았습니다.

## 5. Pilot과 환경 고정 — `results/batches/261126_nav2only_stage2_v2_pilot` (110 launch)

고정 규칙(데이터 전 커밋)은 다음과 같습니다.
1. B1이 모두 완주하고 접촉이 없어야 합니다.
2. 거리 반응이 0.10 m 이상이어야 합니다.
3. 정해둔 순서에서 처음으로 1·2를 만족하는 환경을 고릅니다.

결과는 `analysis/pilot_freeze.json`에 있습니다.

| 상황 | 후보 | 결정 | 이유 |
|---|---|---|---|
| ST-F | 2.2 / 2.6 m | **둘 다 탈락 → UNIDENTIFIABLE** | 각 폭에서 B1 2회 중 1회가 사람을 스치거나 정지 교착 |
| ST-S | 2.2 / **2.6** | 2.6 m | 2.2: B1 접촉·교착 |
| ST-R | 2.2 / **2.6** | 2.6 m | 2.2: B1 접촉·교착 |
| EN-R | **2.2** / 2.6 | 2.2 m | 2.6: B1 접촉 |
| MV-P | 3.0 m × 간격 0.80 / **0.65** / 1.00 | 간격 0.65 m | 0.80: 반응 0.046 m, 1.00: 0.015 m |

- **규칙의 한계:** B1을 "주행 가능성 탐침"으로 쓰므로, B1이 사람을 스치면 그 환경 전체가 빠집니다. ST-F가 이 때문에 빠졌습니다.
  - B1 접촉의 모양: 최근접 중심거리 0.48–0.49 m, footprint 끝–사람 중심 0.20–0.23 m(< 사람 반경 0.25 m). 그 뒤 collision monitor STOP이 걸렸습니다.
  - 데이터를 본 뒤 규칙을 바꾸지 않았습니다.

## 6. Coarse — `results/batches/261126_nav2only_stage2_v2_coarse` (180 launch, 조건당 5회)

| 상황 | 조건 | nav | joint@5.0 | 중앙 d_cc (m) | 중앙 예측 comfort | 접촉 |
|---|---|---|---|---|---|---|
| EN-R 2.2 | B1 | 4/5 | 0/5 | 0.51 | 3.63 | 1 |
| | B2 0.95 | **1/5** (4 교착) | 1/5 | 1.32 | 5.57 | 0 |
| | α 1.00 / 0.80 / 0.70 | 5/5 | 5/5 | 1.13 / 0.98 / 0.90 | 5.52 / 5.38 / 5.25 | 0 |
| | **α 0.60** | 5/5 | **5/5** | 0.83 | **5.11** | 0 |
| | α 0.50 / 0.45 | 5/5 | **0/5** | 0.74 / 0.73 | **4.82 / 4.78** | 0 |
| ST-S 2.6 | B1 | 5/5 | 0/5 | 0.53 | 3.64 | 0 |
| | B2 0.95 | 5/5 | 5/5 | 1.37 | 6.30 | 0 |
| | α 1.00 → 0.45 | 5/5 | 5/5 | 1.24 → 0.78 | 6.28 → 5.13 | 0 |
| ST-R 2.6 | B1 | 3/5 | 0/5 | 0.52 | 3.72 | 2 |
| | B2 0.95 | 5/5 | 5/5 | 1.38 | 5.57 | 0 |
| | α 1.00 → 0.45 | 5/5 | 5/5 | 1.37 → 0.83 | 5.58 → 5.09 | 0 |
| MV-P 3.0/0.65 | B1 | 5/5 | 5/5 | 0.72 | 5.21 | 0 |
| | B2 0.95 | 5/5 | 5/5 | 0.93 | 6.10 | 0 |
| | α 1.00 → 0.45 | 5/5 | 5/5 | 0.90 → 0.83 (비단조 0.77–0.90) | 6.00 → 5.72 | 0 |

그림: `analysis/figures/S2S_*.png`.

**민감도 (coarse)**

| 상황 | C\* 4.5 | C\* 5.5 | joint ≥ 0.90 |
|---|---|---|---|
| EN-R | FLOOR_LIMITED (0.45) | IDENTIFIED (α\* 1.00) | 결과 동일 |
| ST-S | FLOOR_LIMITED | IDENTIFIED (α\* 0.60) | 결과 동일 |
| ST-R | FLOOR_LIMITED | IDENTIFIED (α\* 0.80) | 결과 동일 |

→ 정지한 사람의 α\*는 C\* 선택에 크게 좌우됩니다.

## 7. 독립 확인 — `results/batches/261126_nav2only_stage2_v2_confirm` (100 launch, 조건당 10회)

- 조건은 규칙대로 B1, B2, α\*, 바로 아래 시험한 α입니다.
- 식별 상태는 coarse에서 가져오고, 확인 실험에서는 결정적인 셀이 재현되는지만 봅니다(`scripts/stage2scale_confirm.py` → `analysis/CONFIRM_VERDICT.md`).
- 이 배치의 `STAGE2_SCALE_REPORT.md` 식별표는 α 1.00이 없어 의미가 없습니다. 보고서에도 그렇게 표시했습니다.

| 상황 | 셀 | nav | joint@5.0 (95 % CI) | 중앙 d_cc | 중앙 예측 comfort | 판정 |
|---|---|---|---|---|---|---|
| EN-R | α 0.60 | 10/10 | **10/10 (0.72–1.00)** | 0.835 | 5.11 | **CONFIRMED** |
| | α 0.50 | 10/10 | **0/10 (0.00–0.28)** | 0.744 | 4.83 | comfort 실패 (주행은 성공) |
| | B2 0.95 | **0/10** (10 교착) | 0/10 | — | — | |
| | B1 | 4/10 | 0/10 | 0.515 | 3.71 | 접촉 6/10 |
| ST-S | α 0.45 | 10/10 | 10/10 (0.72–1.00) | 0.781 | 5.09 | 운영 α 재현 (하한 아님) |
| ST-R | α 0.45 | 10/10 | 10/10 (0.72–1.00) | 0.828 | 5.09 | 운영 α 재현 (하한 아님) |
| ST-S / ST-R | B2 0.95 | 10/10 | 10/10 | 1.37 | 6.35 / 5.57 | |
| ST-S / ST-R | B1 | 10/10 / 9/10 | 0/10 | 0.55 / 0.52 | 3.74 / 3.73 | 접촉 0 / 1 |

## 8. 통계적 불확실성

- 비율은 모두 Wilson 95 % CI입니다. 10/10의 하한은 0.72, 5/5의 하한은 0.57입니다.
  - 따라서 "joint ≥ 0.80"을 통계적으로 보장하지는 못합니다. 관찰된 비율이 기준을 넘었다는 뜻입니다.
- EN-R의 경계는 예측 comfort 중앙값 5.11(α 0.60)과 4.83(α 0.50) 사이에 있습니다.
  - 정지한 사람의 장면은 결정적이라 반복 간 d_cc 변동이 작습니다(같은 셀에서 수 cm). 그래서 0/10 ↔ 10/10이 깔끔하게 갈립니다.
  - 이것은 시뮬레이터 재현성이지, 사람 반응의 변동을 반영하지 않습니다.
- 경계의 위치는 C\*에 민감합니다(§6 민감도). C\*는 문헌 곡선 위의 가정값(ASSUMPTION)입니다.
- Stage 1은 크기당 n = 10이라 5 % 기준을 판별할 수 없습니다. 0.95 vs 1.00의 구분은 접촉 1건 차이입니다.

## 9. 식별되지 않은 조건과 한계

1. **ST-F (서 있는 사람 앞):** pilot 환경 고정 실패로 UNIDENTIFIABLE입니다. 그 때문에 stationary 상태 전체가 UNIDENTIFIABLE입니다.
   - 옆·뒤만 보면 α 0.45까지 통과(FLOOR_LIMITED)입니다. 이것은 하한이 아니라 운영 가능한 최소값입니다.
2. **MV-P (걷는 사람 교행):** 차선이 분리된 교행에서 Nav2 경로가 차선을 따르므로 α를 바꿔도 거리가 거의 변하지 않습니다(0.074 m).
   - B1도 예측 comfort 5.21로 기준을 넘습니다. 이 환경에서는 걷는 사람 Comfort Space의 하한을 구할 수 없습니다.
3. **MV-O (추월):** 실행하지 않았습니다(속도 조건상 불가능).
4. **기준 곡선 도메인 밖:**
   - 문헌 곡선은 약 0.35 m/s 로봇 기준인데, 최근접 시점의 AMR 속도 중앙값은 약 0.78 m/s입니다.
   - station_engaged는 stationary 곡선을 전이해서 썼습니다.
   - 각 trial 행에 `out_of_reference_domain`으로 표시했습니다.
5. **방향 전이:** 각 상태에서 시험한 방향은 1–2개뿐입니다. 나머지 방향은 템플릿 비율을 그대로 적용한 가설입니다(시험 증거 아님).
6. **약식:** Stage 1 seed 10개, coarse 5회, confirm 10회입니다. 원 설계보다 적습니다.
7. **Stage 3은 실행하지 않았습니다.** B3-O와 B2의 비교 결과는 없습니다.

## 10. Comfort–Navigation Trade-off

- **넓은 통로 (2.6 m, 정지한 사람):**
  - B2 0.95와 모든 α가 완주했습니다.
  - α를 줄이면 실제 거리가 1.24–1.37 m에서 0.78–0.83 m로 줄고, 예측 comfort는 6.3 / 5.6에서 5.1로 내려갑니다.
  - 주행시간은 거의 같습니다(16.0–16.2 s). 이 통로에서는 크게 둘수록 손해가 없습니다.
- **좁은 통로 (2.2 m, 작업 중 사람):**
  - B2 0.95는 confirm에서 0/10 주행 성공(10/10 protective-stop 교착)입니다.
  - 작업 중 사람 모양의 α 0.45–1.00은 모두 완주했습니다. 방향별 모양 덕분에 같은 최대 반경이라도 지나갈 수 있습니다.
  - α 0.60 아래로 줄이면 주행은 되지만 예측 comfort가 5.0 밑으로 내려갑니다.
- **B1:** 접촉이 반복되고(EN-R confirm 6/10), protective stop이 늘며(ST-S confirm 4/10), 예측 comfort는 3.6–3.7입니다. Comfort Space가 있으면 이것들이 사라집니다(접촉 0).

## 11. 실행 명령

```bash
python3 scripts/check_stage2scale.py                # 22 tests
python3 scripts/check_comfort_reference.py
# Stage 1 재검증
python3 scripts/final_run.py --dir results/batches/261126_nav2only_stage1
python3 scripts/nav2only_stage1_report.py --dir results/batches/261126_nav2only_stage1
# Stage 2 (configs/stage2scale/stage2scale_v2_nav2only.yaml)
python3 scripts/stage2scale_plan.py --phase pilot
python3 scripts/final_run.py --dir results/batches/261126_nav2only_stage2_v2_pilot
python3 scripts/stage2scale_report.py --dir results/batches/261126_nav2only_stage2_v2_pilot
python3 scripts/stage2scale_freeze.py --dir results/batches/261126_nav2only_stage2_v2_pilot --write
python3 scripts/stage2scale_plan.py --phase coarse
python3 scripts/final_run.py --dir results/batches/261126_nav2only_stage2_v2_coarse
python3 scripts/stage2scale_report.py --dir results/batches/261126_nav2only_stage2_v2_coarse
#   -> confirm_conditions / not_confirmed 를 규칙대로 yaml에 기록
python3 scripts/stage2scale_plan.py --phase confirm
python3 scripts/final_run.py --dir results/batches/261126_nav2only_stage2_v2_confirm
python3 scripts/stage2scale_report.py --dir results/batches/261126_nav2only_stage2_v2_confirm
python3 scripts/stage2scale_confirm.py --coarse results/batches/261126_nav2only_stage2_v2_coarse \
    --confirm results/batches/261126_nav2only_stage2_v2_confirm
```

## 12. 코드 변경 (기본값은 기존 동작 유지, `control.mode: nav2_only`일 때만 적용)

| 파일 | 변경 |
|---|---|
| `comfort_core/control_mode.py` (신규) | 모드 정의, config 검사, ROS graph 검사 |
| `scripts/launch_nodes.py`, `scripts/nav2_bringup.launch.py` | nav2_only: AMR1 controller → `cmd_vel_safe`, gate 미실행 |
| `scripts/nav2_goal.py`, `comfort_core/metrics.py` | 실행 중 graph 검사 → 위반 시 INVALID, trial JSON에 `control` 기록 |
| `configs/nav2only/` (신규) | control overlay, seed s1–s30, 안전 smoke |
| `comfort_core/stage2scale.py` | 상태 판정이 상황 상태를 상속, physical signature가 control / buffer / yield 포함 |
| `configs/stage2scale/stage2scale_v2_nav2only.yaml`, `scripts/stage2scale_plan.py` | v2: 버전별 출력, R_ref + 출처, frozen geometry, confirm 단계, UNIDENTIFIABLE / not_confirmed 상황 건너뜀 |
| `scripts/stage2scale_report.py` | R_ref 라벨, 계획에서 UNIDENTIFIABLE인 주요 상황을 상태로 상속, confirm 단계 경고 |
| `scripts/nav2only_stage1_report.py`, `scripts/stage2scale_freeze.py`, `scripts/stage2scale_confirm.py` (신규) | Stage 1 규칙, pilot 고정 규칙, 확인 판정 |

변경 없음: Nav2 planner / controller / BT / 기본 recovery, collision monitor와 field, footprint, mask 값 98, min_radius 0.30.

## 13. 논문용 서술 (허용 / 금지)

**사용 가능한 서술 (예시)**

> Nav2만으로 주행하는 조건(회피·양보 로직 없음, collision monitor 유지)에서 Comfort Space 반경의 상한을 재검증한 결과,
> 시뮬레이션 기반 운영 상한 R_nav_upper는 0.95 m였다(seed 10개, 잠정).
> 상태별 Comfort Space 모양을 이 값으로 정규화하고 하나의 배율 α로 크기를 줄였을 때,
> 작업 중인 사람의 뒤를 지나가는 2.2 m 통로 상황에서는 문헌 기반 예측 comfort 5.0을 만족하는 가장 작은 배율이 α = 0.60(전·측·후 0.57 / 0.50 / 0.43 m)으로 식별되었다.
> 그 아래 α = 0.50에서는 주행은 10/10 성공했으나 예측 comfort가 기준 아래(중앙값 4.83)로 떨어졌고, 독립 확인 실험(n = 10)에서 재현되었다.
> 같은 통로에서 0.95 m 원형 Comfort Space는 10/10 교착되었다.
> 서 있는 사람의 옆·뒤에서는 시험한 가장 작은 배율(α = 0.45)까지 예측 comfort 기준을 만족해 하한이 관측되지 않았고,
> 정면 상황과 차선이 분리된 보행자 교행 상황에서는 배율에 대한 거리 반응이 없거나 시험 환경을 확정할 수 없어 하한을 식별할 수 없었다.

**금지**

- "experimentally validated comfort distance", "ISO comfort distance", "사람이 편안하다고 느끼는 거리를 측정했다"
- 정지한 사람 α 0.45 또는 걷는 사람 결과를 "comfort 하한"으로 쓰는 것
- 시험하지 않은 방향(예: 작업 중 사람 앞·옆) 반경을 실험 증거로 쓰는 것. 템플릿 비율을 적용한 가설입니다.
- Stage 3(B3-O vs B2) 결과를 주장하는 것. 실행하지 않았습니다.
- `R_nav_max` 용어
