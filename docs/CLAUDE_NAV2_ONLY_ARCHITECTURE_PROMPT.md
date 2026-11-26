# Claude prompt 2 — 석사논문 아키텍처 4개 그림 개정

`hun7407-lgtm/Comfort_AMR`의 최신 `feature/b1-baseline-and-yield` 브랜치를 확인하고 **아키텍처 그림 정확히 4개**를 개정해줘. 이번 연구는 새로운 비대칭 Comfort Space **형상**을 제안하는 논문이 아니라, 작업자 **상태에 따라 그 공간 크기를 동적으로 결정하여 문헌 기반 예측 Comfort와 다중 AMR Navigation Performance의 균형을 검증하는 시스템 프레임워크**를 제안하는 석사논문이라는 점이 가장 중요하다.

## 반드시 먼저 읽을 문서
1. `docs/ARCHITECTURE_4FIG_NAV2_ONLY_SPEC.md` — 이번 작업의 최우선 그림별 구조, Mermaid 참조도, 정확한 라벨, 화살표 및 체크리스트.
2. `docs/NAV2_ONLY_STAGE2_MAIN_EXPERIMENT_SPEC.md` — 사용자가 새로 결정한 Nav2-only 제어 및 Stage 1/2/3 구분.
3. `AGENTS.md`, `docs/FINAL_261125_PLAN.md`, `docs/STAGE2_UNIFORM_SCALE_PROTOCOL.md`, `docs/STAGE2_SCALE_IMPLEMENTATION.md`, `configs/common.yaml`, `scripts/nav2_bringup.launch.py`, `scripts/comfort_publisher.py`, `scripts/launch_nodes.py`, `comfort_core/human_state.py` — 현 구현과 역사적 결과의 근거. 새 결정과 충돌하면 1·2번의 설계를 따르되, 이미 구현·실험됐다고 허위 표기하지 않는다.

## 기존 V1 PNG 4장
- `전체 아키텍처 v1(1).png`
- `Environment Modeling 아키텍처 v1(1).png`
- `Human worker modeling 아키텍처 v1(1).png`
- `Robot modeling 아키텍처 v1(1).png`

이 파일들은 ChatGPT 대화에 제공된 이미지로, GitHub에 바이너리 원본이 있다고 가정하지 마. Claude에 이 이미지들이 첨부됐으면 기존 레이아웃을 기준으로 수정하고, 없다면 최우선 사양 문서의 V1 묘사와 네 개 Mermaid 스케치를 기준으로 정확히 새로 그려줘.

## 공통 수정 규칙
- 4개는 **Overall Framework / Environment Modeling / Human Worker Modeling / Robot Modeling**이며 서로 연결되는 **하나의 프레임워크**로 구성.
- 제안 요소를 `State-wise Comfort Space Size Selection / Scale Policy`로 명확히 강조하고, 비대칭 타원은 `Literature-referenced Asymmetric Template`으로 표현. 새로운 형상 발명이라고 주장 금지.
- **Nav2-only:** 모든 AMR의 주행 명령은 Nav2 global/local planning/controller에서 나오고, 마지막에 Nav2 Collision Monitor가 독립 물리안전계층으로 존재. `Yield Gate`, `Hold Region`의 능동 정지 제어, 커스텀 `Evasion`, `Backoff`, `Off-lane recovery` 박스 및 관련 커맨드 화살표 모두 제거. Costmap의 Comfort Filter Mask는 계속 표현하고, Nav2 기본 recovery는 별도 제안 알고리즘으로 표시하지 않음.
- Stage 2는 **시뮬레이션 기반 1 Human + 1 AMR의 Uniform Scale Sweep + 실제 이격거리/Navigation 측정 + 문헌 기반 Predicted Comfort**임. 기존 전체도에 잘못 들어간 `Real World`, `Human Passing Experiment`, `Comfort Rating Analysis`는 제거. Stage 3은 **8 Humans + 3 AMRs**의 B1/B2/B3 새 Nav2-only holdout 검증.
- 이전 Stage 1의 `0.90m`는 Gate-enabled 역사적 상한이므로 `Legacy reference, Nav2-only revalidation pending`으로 표현. 기존 결과를 새 결과로 꾸미지 말 것.
- `perception.source=tf`, `humans.state_source=ground_truth`가 주 비교실험에서 사용됨. `LiDAR → Worker State Classifier`를 이미 검증된 실시간 입력처럼 그리면 안 되며, 센서기반 상태추정은 별도/선택적 가지로 표시.
- 사람은 Scripted Motion + FSM (Engaged, Moving, Stationary); 로봇을 회피하지 않음. 로봇에서 사람으로 가는 접근/이격 화살표는 **계측된 관계**이고 사람이 로봇에 반응해 경로를 변경한다는 의미가 아님.
- Comfort 평가 블록은 **오프라인 분석**: 실제 시뮬레이션 궤적의 d_cc/상대방향을 Neggers 기반 참조함수에 넣는 것으로, 로봇 제어 루프에 Human Rating을 입력하는 구조로 그리지 말 것.
- Stage1/2/3 완료·진행중/향후 검증 상태를 정확하게 구분하고, 기존 1 AMR Smoke와 다중 AMR 실험 조건을 혼동하지 마.

## 그림별 수정 목표
1. **Overall:** Configuration → Environment/Human/Robot Modeling → Stage 1/2/3 Simulation Evaluation. Stage1 새 Nav2-only Upper revalidation, Stage2 state-wise scale trade-off, Stage3 B1/B2/B3 multi-AMR validation. 최종 결과는 `Framework Validation`이지 `Validated Comfort Space`가 아님.
2. **Environment:** `Shift Schedule`(Scenario Seed/Shift Exchange/Tasks), `Semantic Layout`(Station Posts/Cart Slots/Aisles), `Geometric Layout`(Walls/Columns/U-line/Machines/Cart Racks); 사람 동작, Nav2 Map, 정책별 동일 환경 조건으로 출력을 명확하게 연결. Stage2는 별도의 통제 Corridor.
3. **Human Worker:** Engaged ↔ Moving ↔ Stationary FSM 및 depart/arrive/stop/resume, Route Follower → nominal Speed Profile → Character Animation/Collider → 실제 Pose/Velocity/Heading, Ground-truth State → Comfort Generator. 사람이 AMR을 스스로 회피하는 루프 금지.
4. **Robot:** State/Pose Source + Heading → B1/B2/B3 Policy & reference F/S/R/State Scale → non-lethal Costmap Mask → Nav2 Global/Local Costmap → NavFn → DWB → Collision Monitor → Drive. Logging/Predicted Comfort Post-analysis는 대시 라인으로 오른쪽 또는 아래에 배치.

## 산출물
- `docs/figures/architecture_nav2_only_v2/`에 그림별 **편집 가능한 원본**(Mermaid .mmd/벡터 소스 등) + `*.svg` + 학위논문에 넣을 해상도 `*.png`, 총 4종을 저장.
- 네 그림 모두 외부 Viewer에서 실제로 열어 레이아웃을 확인할 것. 제목/본문 검정이 배경 검정에 묻히는 오류, `Aisles & Crossing s` 같은 어색한 줄바꿈, 잘린 라벨, 엉킨 화살표가 없게 수정.
- English Figure title/caption 4개, 입출력 일치 확인표, V1→V2 변경내용 목록, 논문 주장과 불일치가 없는지 체크리스트 제출.
- **실행 코드나 실험결과를 변경하지 말 것.** 그림은 새로 합의한 목표 아키텍처를 나타내며, 아직 실제 Nav2-only 본실험이 안 끝났다면 `target architecture / validation pending`으로 명시.
- 변경 파일을 Commit하고 경로·프리뷰·비교표 및 검증여부를 한국어로 보고해줘.
