# 3단계 실험 구조 — Stage 1 / 2 / 3

2026-11-12부터의 메인 실험 구조. 이전의 B1/B2/B3/B3-O 4정책 비교
(`results/batches/261107_MAIN_cross_seeds1-33_merged` 등)는 그대로
보존하되 새 실험에서는 쓰지 않는다. 제안 방법은 **B3-O** 하나로
통합한다 (코드 내부 이름은 호환성 때문에 `state_directional` / `b3d`를
유지하고, 새 스크립트와 보고서는 모두 B3-O로 표기한다). 기존 B3
(`state_adaptive`)는 새 runner에서 제외하되 코드·config·결과는 삭제하지
않는다.

```text
Stage 1  Simulation   Comfort Space 크기 sweep  →  R_nav_max  (deadlock이 시작되기 직전의 최대 크기)
Stage 2  Real AMR     R_nav_max에서 Δ씩 줄이며 설문  →  R_comfort_min (acceptable comfort의 최소 크기)
Stage 3  Simulation   B3-O Initial vs B3-O Optimized (+ Static)  →  최종 성능 검증

        R_comfort_min ≤ R* ≤ R_nav_max
```

**물리 안전층(collision monitor, protective field, braking distance)은 어느
단계에서도 변하지 않는다.** 모든 조건 파일은 `scenario`와 `comfort_space`
섹션만 가질 수 있고 (`comfort_core/config.py`가 강제), Nav2 파라미터
생성기는 scenario를 읽지 않는다. `scripts/check_stage1.py` 4번 그룹이
이를 매번 확인한다.

> 어떤 크기도 "experimentally validated comfort distance"가 아니다.
> Stage 1의 크기는 설정 가능한 공학 파라미터의 sweep이고, Stage 2가 사람의
> 판단을 측정한다.

---

## 공통: 크기 조건 = scenario overlay

`comfort_core/size_sweep.py`가 세 단계가 공유하는 정의다.

| 조건 파일 | 내용 |
|---|---|
| `static_r1.00.yaml` | `policy: static`, `static_radius: 1.00` |
| `b3o_r1.00.yaml` | `policy: state_directional`, `aisle_scaling: {enabled: true, target_size_m: 1.00, floor_m: 0.0}` |

B3-O의 "크기"는 **기준 속도(1.0 m/s)에서의 moving front extent**이고,
비스케일 값이 1.20 m이다. `scale = target / 1.20`을 모든 state·방향·
speed mapping·clip에 곱한다 (`comfort_core/comfort_space.py::_scaling`).
front/side/rear 비율, engaged/stationary/moving 관계, heading source,
ellipse offset은 그대로다. `floor_m: 0.0`이면 순수 비례이고
`comfort_space.min_radius`(0.30 m, 사람 몸)만 하한으로 남는다.

적용값 (cross campaign config 기준, `scripts/make_size_conditions.py --table`):

| size | scale | engaged F/S/R | stationary F/S/R | moving@1.0 F/S/R |
|---|---|---|---|---|
| 0.80 | 0.6667 | 0.533/0.467/0.400 | 0.533/0.600/0.700 | 0.800/0.600/0.600 |
| 1.00 | 0.8333 | 0.667/0.583/0.500 | 0.667/0.750/0.875 | 1.000/0.750/0.750 |
| 1.20 | 1.0000 | 0.800/0.700/0.600 | 0.800/0.900/1.050 | 1.200/0.900/0.900 |
| 1.40 | 1.1667 | 0.933/0.817/0.700 | 0.933/1.050/1.225 | 1.400/1.050/1.050 |
| 1.60 | 1.3333 | 1.067/0.933/0.800 | 1.067/1.200/1.400 | 1.600/1.200/1.200 |

모든 trial row는 요청값이 아니라 **적용된 m 값**을 기록한다
(`comfort_size_m, comfort_scale, cs_engaged_*, cs_stationary_*, cs_moving_*`
— `comfort_core/metrics.py` FIELDS 끝, `comfort_core/comfort_space.py::geometry_record`).

### Deadlock 메커니즘 분리

`scripts/nav2_goal.py`가 판정 시점에 `deadlock_mechanism`을 기록한다:

| 값 | 뜻 | 크기 효과? |
|---|---|---|
| `comfort_gate_hold` | yield gate가 Comfort Space 진입을 막고 있는 채로 12 s 무진전 | **예** — 유일하게 크기 때문에 생기는 deadlock |
| `post_evade_stall` | 보행자 회피 후 다시 움직이지 않음 | 아니오 (회피 모듈/지형) |
| `nav2_abort` | Nav2가 goal을 포기 | 아니오 |
| `stall` | 위 어느 것도 아님 | 조사 필요 |
| `planning_failure`, `timeout`, `invalid` | 시작 실패 / 시간 초과 / 물리·측정 결함 | 아니오 |

Stage 1의 threshold는 `comfort_gate_hold`만 센다. 나머지는 옆에 같이
보고하고 절대 합치지 않는다.

---

## Stage 1 — Simulation, Navigation Feasibility Threshold

```bash
./scripts/stage1_sweep.sh --dry-run          # 계획과 적용 F/S/R 표만
./scripts/stage1_sweep.sh --smoke            # seed 5 × {0.8, 1.2, 1.6} × {static, b3o}
./scripts/stage1_sweep.sh                     # 33 seeds × 5 sizes × 2 policies = 330 trials
./scripts/stage1_sweep.sh --sizes "1.25 1.3 1.35" --out results/batches/<...>_stage1_refine
./scripts/stage1_sweep.sh --out <같은 폴더> --skip N   # 중단된 sweep 이어가기
python3 scripts/stage1_report.py --dir results/batches/<...>
python3 scripts/check_stage1.py --dir results/batches/<...>
```

- 시나리오: cross (`configs/campaigns/cross/campaign_h8_a3_cross_s<seed>.yaml`,
  AMR1 + a1 + a2, 작업자 8명). seed당 10개 조건이 **같은 파일**로 돈다.
- 매니페스트는 seed-major (`scripts/stage1_manifest.py`): 중간에 멈춰도
  완료된 seed 단위로 비교 가능.
- 실행은 `scripts/run_campaign.sh` (trial마다 새 시뮬레이션, watchdog,
  시작 실패 1회 재시도).
- 산출: `trials.csv`(AMR1), `fleet_trials.csv`(a1, a2 각각), trial별 JSON
  (궤적 `trajectory`, 첫 Nav2 plan `planned_path` 포함), `REPORT.md`,
  `tables.md`, `thresholds.json`, `figures/P1–P9`.

Threshold 정의 (`scripts/stage1_report.py`, 정책별):

```text
R_deadlock_onset = comfort_gate_hold이 처음 나타나는 가장 작은 size
R_nav_max        = 그 아래에서 comfort_gate_hold이 0인 가장 큰 size
```

모든 size에서 0이면 "R_nav_max ≥ 최대 size (onset은 sweep 밖)", 최소
size에서 이미 나오면 "R_nav_max는 sweep 아래"로 보고한다. size별 개수
표를 항상 함께 출력해 비단조 패턴을 숨기지 않는다. 필요하면
`--sizes`로 threshold 사이를 0.05–0.1 m 간격으로 refine한다.

---

## Stage 2 — Real Environment, Human Comfort Threshold

**아직 실행하지 않는다.** SLAM map과 시나리오가 정해지면 바로 돌 수 있게
interface만 준비되어 있다. 다음이 채워져야 한다
(`comfort_core/stage2.py::missing_placeholders`가 목록을 출력하고,
하나라도 null이면 `stage2_trial.py`는 시작을 거부한다):

| 넣을 곳 | 무엇 |
|---|---|
| `configs/stage2/scenes/<corridor>.yaml` (`scene_template.yaml` 복사) | `nav2.map`(SLAM map yaml), `navigation.start/goal(+yaw)`, `humans.P.position/state/facing_deg`, 복도 폭·통과 측 |
| `configs/stage2/stage2_real.yaml` | `stage2.r_nav_max_m` ← Stage 1 `thresholds.json`(B3-O), `comfort_step_m`(기본 0.1), `n_conditions`, `comfort_acceptance_score`(**null 유지**, pilot 후 결정) |

파일 레이어: `common.yaml → real_amr.yaml → stage2_real.yaml → scenes/<x>.yaml`
+ 조건 overlay `configs/stage2/conditions/b3o_r<size>.yaml`.

```bash
python3 scripts/stage2_make_conditions.py --orders 12        # R_nav_max, -Δ, -2Δ ... + 참가자별 순서
./scripts/stage2_trial.sh --scene configs/stage2/scenes/<x>.yaml \
    --scenario configs/stage2/conditions/b3o_r1.00.yaml \
    --session 2612xx_pilot --participant P01 --order 3 [--check]
python3 scripts/stage2_questionnaire.py --show-items          # 문항·anchor
python3 scripts/stage2_analysis.py --session-dir results/stage2/<session>
```

실기 파이프라인 (기존 노드 재사용):

```text
Real AMR pose (tf map→base) ─┐
Human pose/state/orientation ─┼→ scripts/comfort_publisher.py (B3-O, 조건 overlay)
                              │      → KeepoutFilter mask → Nav2 costmap → Nav2 → comfort_gate → collision monitor → AMR
                              └→ scripts/stage2_trial.py (기록) → results/stage2/<session>/
```

기록 (`comfort_core/stage2.py::TRIAL_FIELDS`): 궤적(`trajectories/<trial>.csv`,
10 Hz, 사람 위치·clearance·yield·safety action 포함), travel time/distance,
planned path length, min human distance·시각, human state/facing/speed,
적용 Comfort Space F/S/R, intrusion, yield s, safety stop/slowdown s,
Nav2 status, commit.

설문 (`QUESTIONNAIRE_FIELDS`): `participant_id, trial_id, condition,
comfort_size, comfort_score, perceived_safety, distance_acceptability,
trial_order, timestamp, notes`. Primary item은 Neggers et al. (2022, IJSR,
doi 10.1007/s12369-021-00805-6)의 7점 perceived comfort; secondary 두
항목의 문구는 사용 전 reference 확인(TODO). 개인정보는 pseudonym 코드만.

분석 (`stage2_analysis.py`): size별 mean/median/sd/분포, 참가자×size 표,
Friedman(scipy 있을 때), 2-segment breakpoint(탐색적), 그리고
`comfort_acceptance_score`가 설정된 경우에만 `R_comfort_min`. 기본은 null
이므로 cutoff 없음을 명시하고 R_comfort_min을 주장하지 않는다. raw
`questionnaire.csv`는 수정하지 않는다.

---

## Stage 3 — Simulation, Optimized B3-O Validation

```bash
python3 scripts/stage3_make_configs.py                 # static_baseline, b3o_initial, b3o_optimized(TEMPLATE)
./scripts/stage3_run.sh --dry-run
./scripts/stage3_run.sh --r-opt <R_opt>                 # b3o_initial vs b3o_optimized, 33 seeds
./scripts/stage3_run.sh --r-opt <R_opt> --with-static   # + static baseline (3 조건)
./scripts/stage3_run.sh --params calibrated.yaml        # 비율까지 바뀐 보정 블록이면
python3 scripts/stage3_report.py --dir results/batches/<...>_stage3_validation
```

- `b3o_initial` = 비스케일 B3-O (Stage 1의 1.20 m 조건과 동일).
- `b3o_optimized` = Stage 2 결과. `--r-opt`(크기)나 `--params`(directional
  블록)로만 채워지며, template(null) 상태에서는 runner가 거부한다.
- 보고: 조건별 success / comfort deadlock / 메커니즘 / travel time·
  distance / 첫 plan 길이와 detour / yield·wait / clearance / contact /
  intrusion / robot-robot gap, seed별 paired Δ(Optimized − Initial),
  궤적+plan overlay (`figures/V4_trajectories_s<seed>.png`).

---

## 검증 (코드만, 시뮬 없이)

```bash
python3 scripts/check_stage1.py                 # 1042 checks: config / scaling / same-seed / safety
python3 scripts/check_stage1.py --dir <결과>     # + CSV 검증
python3 scripts/check_comfort_shapes.py         # 기존 정책 불변
./scripts/stage1_sweep.sh --dry-run
./scripts/stage3_run.sh --dry-run
```
