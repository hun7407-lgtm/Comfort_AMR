# Claude FINAL EXECUTION PROMPT — Stage 2 & Stage 3 ONLY (2026-11-27)

아래 작업은 `hun7407-lgtm/Comfort_AMR` 저장소의 `feature/b1-baseline-and-yield` 브랜치에서 실행하는 **석사논문 최종 Stage 2·3 실험 코드 구현 및 본실험 수행**이다.

## 읽기 순서와 문서 우선순위

1. **무조건 먼저 읽을 것:** `docs/FINAL_STAGE2_STAGE3_NAV2_ONLY_PROTOCOL_20261127.md` — 이번 최종 연구의 **authoritative locked protocol**.
2. 그다음 `docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md`, `docs/NAV2ONLY_STAGE2_RESULTS.md`, `docs/NAV2ONLY_RUN_LOG.md` — 기존 Nav2-only Stage 2 코드/예비 실험 참고.
3. `docs/ARCHITECTURE_4FIG_NAV2_ONLY_SPEC.md`, `AGENTS.md`, `configs/common.yaml`, `comfort_core/control_mode.py`, `comfort_core/stage2scale.py`, `comfort_core/comfort_reference.py`, `comfort_core/final_metrics.py`, `scripts/nav2_goal.py`, `scripts/launch_nodes.py`, `scripts/nav2_bringup.launch.py`, `scripts/stage2scale_plan.py`, `scripts/stage2scale_report.py`, `scripts/final_run.py`, `scripts/make_cross_campaign.py`.
4. `docs/FINAL_NAV2_ONLY_3STAGE_PROTOCOL_20261127.md` 및 `docs/CLAUDE_FINAL_NAV2_ONLY_3STAGE_PROMPT_20261127.md`는 **이전 3단계 계획**이다. **Stage 1을 다시 실행하라는 구 문서 지시를 따르지 말 것.** 이번 작업에서는 1번 문서가 이를 대체한다.

## 가장 중요한 사용자 결정

**Stage 1 재실험/재구현/결과 재산출은 전부 제외. 기존 Nav2-only 사전 실험에서 얻은 `R_ref=0.95m`만 모든 B2와 B3 정규화의 고정 기준 반경으로 사용하라.** 이것을 새로 확인한 universal Upper Bound로 표현하지 말고 **preliminary engineering reference**로 표시하라.

**실행하는 최종 본실험은 Stage 2와 Stage 3뿐이다.**

### Stage 2 — 상태별 Comfort Space Scale 검증

- Isaac Sim + ROS 2 Nav2, Human 1명, AMR 1대.
- 5개 context: `ST-F`(stationary 정면), `ST-S`(stationary 측면), `ST-R`(stationary 후면), `EN-R`(station_engaged 후면), `MV-P`(moving 교행).
- 9개 policy: `B1(no Comfort Space)`, `B2(circle r=0.95m)`, `B3(alpha=1.00,0.90,0.80,0.70,0.60,0.50,0.45)`.
- 5 × 9 × **조건별 30회 독립적인 실행 = 1,350 MAIN trials**. Pilot, Smoke는 main 반복 횟수에 포함 금지.
- B3의 상태별 비대칭 F/S/R 참고 형상은 기존 문헌 기반 템플릿을 그대로 사용하고 세 축 모두 **하나의 alpha로 균일하게 곱해** 크기만 바꿔라. `min_radius` clipping이 없어야 한다.
- 주행성공률, 경로길이, 이동시간, 실제 최소 사람–AMR 중심거리/Footprint clearance, 참조곡선 기반 **예측 Comfort**(실제 로그의 이격거리/방위 기준), 충돌/Collision Monitor stop, Joint success를 측정하라. `C*=5.0` 및 4.5/5.5 민감도. 실제 피험자의 Comfort 측정값처럼 표현하지 말 것.
- 새로운 pilot 단계에서 물리적으로 유효한 geometry와 counterwalking interaction을 정책 중립적으로 고정하고 실험 전 커밋해라. 이전 `ST-F`가 B1 접촉 때문에 통째로 빠진 문제, `MV-P`에서 alpha 변화가 실제 거리에 0.074m밖에 영향을 주지 못한 문제를 해결 가능한 범위에서 보완하라. `station_engaged`에는 실제 work-station geometry를 포함하고 모든 정책에서 동일하게 유지해라.
- Scale/Comfort 관계에서 명확한 하한을 못 찾으면 `FLOOR_LIMITED` 또는 `UNIDENTIFIABLE`로 기록한다. 임의로 alpha 0.60을 모든 사람 상태의 최적값으로 지정하지 말 것.

### Stage 3 — 다중 AMR Comfort–Navigation Trade-off 최종 비교

- Human 8명, 독립적인 Nav2 AMR 3대. **정확히 5개 policy × 30개 신규 matched seeds = 150 MAIN trials.**
- C1: `B2_circle_r095`
- C2: `B3_uniform_alpha060`
- C3: `B3_uniform_alpha070`
- C4: `B3_uniform_alpha080`
- C5: `B3_uniform_alpha090`
- 새로운 holdout `seed 1001–1030` 사용(과거 `101–130` 레거시 Gate 기반 Stage3 재사용 금지). 정책별 동일 seed는 Human State·동선·속도·미션·장애물·Nav2와 safety가 완전히 같고 Comfort Space policy만 달라야 한다. 30개를 일반 공장/작업 설비 병목/보행 교행 등 균형 잡힌 strata로 사전 등록·고정한다.
- **주 지표:** 3개 AMR 모두 미션 완료, 사람 신체 접촉 없음, 모든 채점 가능한 유효 Human–AMR encounter가 **예측 Comfort≥5.0**인 Trial 비율. Encounter가 없거나 Comfort 채점 불가능하면 자동 성공 처리하지 말고 coverage와 미채점 건수를 공개하라.
- **보조 지표:** system/navigation success, safe completion, travel time(완주만의 시간과 실패 포함 capped 시간 둘 다), path length, idle/stopped time, Nav2 replans, comfort-induced / robot-robot / physical protective-stop outcomes, actual clearance, predicted Comfort by state, collision/contact rate, 95% CI, matched-seed comparisons and Pareto analysis.
- 중간값 alpha `0.70` 또는 `0.80`이 균형점이라는 것은 **검증할 가설**이다. 그 값이 반드시 1등이 되게 corridor/traffic/safety/threshold를 조정하지 말고, 더 낮거나 높은 alpha가 좋거나 차이가 불명확해도 그대로 보고하라.
- 기존 Stage3는 같은 Mask를 사용하는 B2와 UPPER조차 30개 seed 중 11개에서 성공 여부가 달랐다. 따라서 정책 평가와 별도로 prespecified duplicate-run diagnostic으로 Isaac/ROS 비결정성 계측하고 유의하지 않은 작은 차이는 과대해석하지 말 것.

### 공통 필수 코드/설정 조건

- **Nav2-only**, AMR1/a1/a2 모두 `Nav2 controller/behavior_server → cmd_vel_safe → Collision Monitor → cmd_vel`. `comfort_gate`, custom Yield/HOLD, evade, backoff, off-lane, traffic-rule은 꺼라. Nav2 자체 planner/controller/native recovery와 물리적 Collision Monitor 보호는 유지해라.
- `control.mode=nav2_only`, buffer `0.10m`, finite mask cost 98, min radius `0.30m`, main `perception.source=tf`, `humans.state_source=ground_truth`; 병합 설정/실제 ROS graph 위반은 INVALID.
- `comfort_core/final_metrics.py`에서 nav2_only 정지시간을 기존 Gate `yield`로 잘못 표시하는 문제 수정(레거시 호환성 보존). `comfort_reference.py` 보조 clearance-matched metric의 반폭 보정 부호 검사/테스트; 기존 main comfort metric 의미는 유지.
- 정책별 전 로봇 mask가 똑같은 정책을 소비하는지 확인. 모든 AMR과 Human의 contact, success/failure, eligible encounter를 정확히 기록. Nav2 action이 성공해도 사람 접촉이 있으면 **safe/joint failure**다.
- Stage 3 다섯 정책은 **모든 상태에 동일 alpha**를 적용하되, 상태별 참조 형상 F/S/R가 다르다는 사실을 유지한다. **상태마다 alpha가 다른 dynamic optimizer의 효과**까지 입증한 것처럼 쓰지 말 것.
- 사용자 논문 초안 속 임의 예시 수치를 정답으로 사용하거나 결과를 그 수치로 조작/생성하지 말 것.

## 작업 범위와 완료 기준

1. 신규 Versioned Stage23 config generator / scenario manifest / run-and-resume scripts / metrics / reports / offline tests를 구현해라. 기존 `261125_*`, `261126_*` 결과·설정은 덮어쓰지 않는다.
2. 안전하고 일관된 merged-config와 runtime ROS graph/physical safety Smoke 검증 후, Stage 2/3 사전 pilot과 geometry freeze를 데이터 수집 전 커밋해라.
3. **Stage 2 1,350회 → Stage 3 150회**의 30-rep 본실험을 수행하고, 수행하지 못했으면 정확한 미실행 건수와 기술적 원인을 보고하라. Stage 1은 실행 0회.
4. Manifest/Raw trial/encounter CSV, frozen config hashes, log/commit SHA, Stage2 report, Stage3 report, Wilson 95% CI, 충분한 성능 그래프 및 모든 조건별 실패 원인을 산출하고 GitHub에 커밋해라.
5. 실제 데이터를 바탕으로 한글 석사논문용 **실험 설계 / 실험 결과 / 결론 각각 두 문단, 문단당 정확히 세 문장**을 작성해라. **사전 Stage 1은 설계 기준 반경 설명에만 등장**하며 본실험 실행/표에 포함하지 말 것.
6. 작업은 설계만 하는 것이 아니라 **최종 실험 코드 구현과 가능한 환경에서의 실제 실행 및 결과 검증**이 목표다. 로컬 Isaac Sim·ROS2 실행환경이 없으면 코드와 정적 검증까지 수행하고, 미실행을 명시하며 실행 가능한 정확한 명령어를 남겨라. 안전·학술적 유효성이 깨지면 데이터를 조작하지 않고 실패를 보고해라.

**종결 체크:** `Stage1_NEW=0`, `Stage2_MAIN=1350`, `Stage3_MAIN=150`, `TOTAL_MAIN=1500`; 추가 Pilot/Smoke/Diagnostics는 본 실험 횟수와 구분.
