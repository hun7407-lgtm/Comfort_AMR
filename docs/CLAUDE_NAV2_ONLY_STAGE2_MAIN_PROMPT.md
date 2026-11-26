# Claude prompt 1 — Stage 2 NAV2-only 본실험 구현·실행

`hun7407-lgtm/Comfort_AMR`의 `feature/b1-baseline-and-yield` 최신 HEAD를 기준으로 작업해. 이번에는 진단용 코드만 만드는 것이 아니라 **설정 검증, 필요한 Stage 1 재검증, Stage 2 Pilot/Coarse/Confirmatory 본실험 실행 및 분석까지** 진행하는 작업이다.

## 첫 번째로 읽을 문서 (우선순위)
- `AGENTS.md`: 기존 코드와 과거 결과 이해용. 이전에 기재된 Gate-assisted 결과는 역사적 결과다.
- **`docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md`: 이번 사용자 결정의 최우선 규칙.**
- `docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md`, `docs/STAGE2_SCALE_IMPLEMENTATION.md`: Uniform Scale, 기록, 테스트, 과거 Smoke 결과 참고. 단, Yield 사용 등 새 문서와 충돌하면 새 문서가 우선이다.
- `docs/ARCHITECTURE_4FIG_NAV2_ONLY_SPEC.md`: 논문 아키텍처와 동일한 사실관계를 사용.
- `configs/common.yaml`, `configs/final/seeds/s1.yaml`, `configs/final/seeds/s101.yaml`, `configs/stage2scale/stage2scale.yaml`, `scripts/launch_nodes.py`, `scripts/nav2_bringup.launch.py`, `scripts/comfort_gate.py`, `comfort_core/stage2scale.py`, `scripts/stage2scale_plan.py`, `scripts/stage2scale_report.py`, `scripts/check_stage2scale.py`, `comfort_core/comfort_reference.py`, `scripts/final_run.py`, `results/batches/261126_stage2scale_v1_smoke/analysis/STAGE2_SCALE_REPORT.md`.

## 연구 목표
석사논문의 Contribution은 새로운 비대칭 타원 형상의 제안이 아니다. **기존 문헌을 참고한 상태별 Comfort Space 형상의 '적절한 크기'를 동적으로 선택해서, 예측 Comfort와 다중 AMR Navigation Performance를 함께 확보할 수 있는 시스템 프레임워크를 실증적으로 검증**하는 것이다.

## 이번에 반드시 바꿔야 할 구현
1. **진정한 Nav2-only:** 신설 실험의 B1/B2/B3 모든 AMR에서 커스텀 `comfort_space.yield.enabled=false`, `yield.evade.enabled=false`, `recovery.backoff.enabled=false`, `recovery.off_lane.enabled=false`; Costmap Buffer는 0.10m. `comfort_gate.py`를 실제 주행 명령 경로에서 우회하거나 완전 pass-through임을 실증하고, AMR1 `cmd_vel_nav`를 collision monitor 입력 `cmd_vel_safe`로 정상 연결해. Nav2 planner/controller 및 Nav2 자체 recovery, Collision Monitor는 유지한다. 모든 AMR에 같은 원칙을 적용하며 안전 감시를 끄거나 임계값을 완화하지 마. ROS 토픽 실제 연결과 활성 노드를 검사해 커스텀 회피·정지 명령이 없는 것을 테스트로 증명해.
2. **과거 Stage 1 상한 검증:** 기존 `R_nav_upper=0.90m`는 `yield.enabled=true`인 과거 데이터에서 나온 값이므로 새 Nav2-only 조건에서 검증된 상한이 아니다. 0.85/0.90/0.95/1.00m 부근 Static Circle과 B1을 다중 AMR 통제환경에서 재검증하고, 새로운 Nav2-only 운용 상한 기준을 **데이터 보기 전에 먼저 정하고 기록**해. Gate Hold 기반 Deadlock 판정은 새 실험에 쓰지 말고 Nav2 no-progress/abort, Collision Monitor 정지, 실제 통과·접촉 및 성공률을 사용해. 상한이 확인되지 않으면 0.90은 'legacy reference'로만 유지하고 결과를 정직하게 보고해.
3. **Stage 2 모델:** `station_engaged`, `stationary`, `moving` 각각의 참고 F:S:R 비율을 보존하면서 `E_H(alpha)=alpha E_H(1)`만 Sweep. 독립 축 최적화 금지. 기본 후보 `alpha=1.00,0.90,0.80,0.70,0.60,0.50,0.45`; 상한 재평가로 `R_ref`가 달라지면 `min_radius=0.30` clipping이 없는 grid를 재설정한 뒤 실험을 동결한다. `aisle_scaling` 비활성 유지.
4. **통제 시나리오:** `ST-F, ST-S, ST-R, EN-R, MV-P`를 사용. Moving은 사람과 AMR이 서로 다른 좌·우 차선에서 반대방향으로 이동하여 측면 통과하도록 하고, 통로 폭·차선 간격·시작/목표·사람 이동 및 속도·충돌감시 설정은 같은 시나리오의 B1/B2/모든 alpha에서 동일하게 고정한다. `MV-O` overtaking은 실제 속도조건상 미지원이면 수행하지 않는다.
5. **평가:** 실제 시뮬레이션 궤적의 Human–AMR `d_cc`와 상대방향으로 `comfort_core/comfort_reference.py`의 문헌 기반 예측 Comfort를 계산한다. alpha나 Mask 값으로 점수를 직접 계산하면 안 된다. C*=5.0/7(가정), 감도 4.5/5.5; 실제 통과·목표 도착·접촉 없음·방향/상태 유효를 모두 만족할 때만 joint pass. 교착으로 멀리 서 있어도 Comfort 성공으로 처리하지 않는다. Nav2 완료율, 문헌 Comfort 조건부 충족률, joint rate, 최소 실제 중심/footprint 거리, 경로길이, 시간, 안전개입, Wilson 95% CI를 나눠 보고한다.
6. **가장 작은 alpha만 유리해지는 편향 방지:** 실제 이격거리가 alpha에 거의 반응하지 않으면 `UNIDENTIFIABLE_GEOMETRY`, 최저 alpha에서도 예측 Comfort 저하가 없다면 `FLOOR_LIMITED/BOUND_NOT_OBSERVED`. 반경 큰 조건의 Nav2 실패가 있어도 작은 조건을 자동 배제하지 않는 nonmonotone selector를 유지. 1~2회의 Smoke 결과로 `alpha*`를 확정하지 마.
7. **실험 순서:** 새 버전/새 배치 경로에서 (a) 정적 테스트 및 Nav2-only 실제 주행/안전 Smoke, (b) Stage 1 새 제어조건의 Upper 재검증, (c) Stage 2 새 Smoke, (d) 통로 폭·Moving 간격 Pilot, (e) geometry freeze, (f) 5 contexts × 9 conditions × 10 repeats ≈ 450 coarse launches, (g) 선택값과 바로 작은 alpha의 ≥30개 독립 Confirmatory launches + B1/B2 비교 및 wide transfer. 데이터가 없으면 만들어 쓰지 마. 시스템/안전 통제 실패 시 본실험을 중단해 원인과 재실행 조건을 보고하고, 통과하면 불필요한 추가 승인 질문 없이 본실험을 수행해.
8. **보존/재현성:** 기존 `results/batches/261125_*`, `261126_stage2sim_*`, `261126_stage2scale_v1_smoke`와 Stage 3 이전 결과를 변경하지 마. 재실행/재녹화 결과를 이전 trial 데이터로 위장하지 마. 동일 seed도 비결정적이므로 반복 launch, commit SHA, config hash, 시작 시간/속도/ROS graph, 효과 크기, 통계 불확실성을 기록해. 시뮬레이션을 못 돌리는 환경이면 오프라인 테스트까지만 실행하고 장비 PC에서 돌릴 정확한 명령어를 제공해.

## 최종 답변
(1) 변경·추가 파일과 코드 요약, (2) Nav2-only가 실제로 적용됐다는 증거, (3) Stage1 0.90m의 새 검증 결과/보류 상태, (4) Pilot 기하학 선택 로그, (5) 실험별 n/성공·접촉/CI/Comfort/거리/교착, (6) 상태별 `alpha*`의 식별/미식별 판정, (7) 재현 명령어·Git commit, (8) 석사논문에 쓸 수 있는 검증된 문장과 쓰면 안 되는 주장을 **한국어**로 보고해. Stage 3 데이터는 이 작업에서 새로 실행한 것이 아니라면 새 결과처럼 말하지 마.
