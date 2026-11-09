# 배치 기록 (results/batches)

한 배치는 `./scripts/run_batch.sh`가 돌린 시행 묶음이고, 디렉터리 하나당

```
trials.csv        시행 한 건 = 한 행
<stamp>_*.json    시행별 상세 (최근접거리, 침입, 상태 혼동행렬, 최종 자세)
batch.txt         무엇을 요청했고 무엇이 돌았는지
```

`logs/`는 커밋하지 않습니다 (`.gitignore`) — 기계별 스크롤백이고 24 시행이 2 MB를 씁니다. 시행의 결과는 CSV 행과 JSON입니다.

## 폴더 이름 읽는 법

```
261105_MAIN_pilot_policy_x_amrs_seeds1-8
  │      │    └ 무엇을 비교했는지
  │      └ 상태 (없으면 유효·보조). MAIN / INVALID / PARTIAL / check — 아래 표
  └ 실행을 시작한 날짜 YYMMDD
```

`./scripts/run_campaign.sh`로 돌린 묶음(261105 이후)은 `batch.txt` 대신 다음을 가집니다.

```
manifest.txt      돌린 trial 목록 (한 줄 = 정책 + config). 그대로 다시 돌릴 수 있음
campaign.log      trial별 종료 코드·시간, 재시도, watchdog 유실
trials.csv        AMR1, trial 한 건 = 한 행
fleet_trials.csv  다른 AMR(a1, a2 …), 차량 한 대 = 한 행
REPORT.md         가설별 정리 (있는 폴더만)
tables.md 등      report 스크립트가 CSV에서 다시 만드는 표
figures/          같은 스크립트가 만드는 그래프 (P1…, F1…)
```

CSV의 `config` 열은 **실행 당시 경로**입니다. 261105–261106 행의 `configs/campaign_*.yaml`은 지금 `configs/campaigns/<묶음>/` 아래에 있습니다(파일 이름은 그대로, `configs/README.md` 참조). 결과 행은 고치지 않았습니다.

**어긋난 배치도 지우지 않습니다.** `comfort_core/metrics.py`가 어긋난 시행을 기록하고 표시하는 것과 같은 이유입니다 — 나중에 평균에 섞이지 않도록, 왜 못 쓰는지가 같이 남아 있어야 합니다.

| 배치 | 날짜 | 무엇을 비교 | n | 상태 |
|---|---|---|---|---|
| `261013_smoketest`, `261018_speed08_check1/2/3` | 261013 · 261018 | 0.8 m/s 복귀 확인 | 2씩 | 참고용. 1.2 m/s가 주행을 깬 뒤 속도를 되돌린 확인 사격입니다 |
| `261023_tf_vs_lidar_INVALID_starved` | 261023 | 참자세 vs 라이다 루프 | 23 | **무효.** 라이다 행은 ① Comfort Space가 하나도 만들어지지 않은 상태(검출 이름 `p1` vs 설정 이름 `w4`)에서 측정돼 B2·B3가 사실상 B1이었고, ② `people_tracker.py` 좀비 프로세스가 머신에 쌓여 시뮬이 실시간의 0.12배로 떨어진 상태였습니다. `host_load` 열이 없는 세대라 사후 판별도 안 됩니다 |
| `261023_tf_vs_lidar_before_parkfix` | 261023 | 참자세 vs 라이다 루프 | 23 | 유효하지만 **대체됨.** 주차 함정(목표가 칸 중앙 = 보호필드가 뒤 대차를 뭄)이 결과를 지배합니다. 그 함정의 증거로서 의미가 있습니다 |
| `261023_parkfix_check_b1` | 261023 | B1 ×4 ×2 (주차 수정 검증) | 8 | 유효. 8/8 도달 — 출발·목표를 칸 입구로 0.17 m 당긴 수정의 확인 |
| `261023_MAIN_tf_vs_lidar_b1b2b3` | 261023 | B1/B2/B3 × 참자세/라이다 | 24 | **유효.** 깨끗한 머신, 주차 수정 후. 콜라이더 원통이 보이는 상태의 마지막 측정 |
| `261024_speed05_b3` | 261024 | B3 0.5 m/s × 참자세/라이다 | 8 | 유효. "인식 거리에 속도를 맞추면 거리 유지가 회복된다"를 **반박**합니다 |
| `261024_nocylinder_check_b1` | 261024 | B1 ×3, 원통 없음 | 3 | 유효. 3/3 `robot drove in 0.0 s` — 캐릭터 메시만으로 보호필드가 동작함 |
| `261024_nocylinder_PARTIAL_9of24` | 261024 | B1/B2/B3 × 참자세/라이다, 원통 없음 | 9 | **미완.** 24 중 9에서 중단. `factory_u`의 B1·B2만 들어 있어 비교에 쓸 수 없습니다. 원통 없는 전체 비교는 아직 돌려야 합니다 |
| `261028_paper_scenarios_b1b2b3rb3` | 261028 | B1/B2/B3-R/B3 × 논문 시나리오 4개 (사람 4·8, AMR 1·2·3) × 3 반복 | 48 | **유효.** 도달 12/12 · 2/12 · 12/12 · 12/12. B2 교착 10건 전부 Comfort Space 양보가 원인(안전층 STOP 0.0 s). 한계: seed 1 하나뿐이고 B3-R과 B3의 대조가 0(움직이는 사람이 로봇 통로에 없었음). `RESULTS.md` 참조 |
| `261102_yield_tuning` | 261102 | `configs/yield_watch.yaml`에서 B3, B1 각 1회 (양보 로직을 눈으로 보기 위한 임무) | 2 | check. **비교용 아님.** 둘 다 DEADLOCK — B3는 출발 칸에서 1.33 m 가서 31.2 s 양보(`yielding to a Comfort Space`), B1은 13.75 m 가서 정지(원인 미기록). 오버레이 자체가 측정 조건이 아니라고 적혀 있습니다 |
| `261105_fleet_patterns_keepright` | 261105 00:08 | 정책 4개 × (AMR 1대 / 4대 roundtrip) + 플릿 패턴 loop·ring × 우측통행 on/off, 사람 8·0명, seed 1–5 | 81 (기록 78) | 유효하지만 **대체됨.** B3-O가 옛 station_engaged 값(F0.70/S0.80/R0.95)으로 돌았습니다. 결론: loop는 사람 없이도 교착, 우측통행은 효과 없음(off 유지), ring은 AMR1에 via가 없어 교착 |
| `261105_b3o_optionA_check` | 261105 12:48 | B3-O ×5 (seed 1–5, AMR 1대), station_engaged F0.80/S0.70/R0.60로 바꾼 뒤 | 5 | check. 5/5 도달 |
| `261105_pilot_PARTIAL_1of140` | 261105 13:00 | 140 trial manifest의 첫 시도 | 1 | **미완.** 첫 trial이 TIMEOUT으로 끝난 뒤 중단. 기록용 |
| `261105_MAIN_pilot_policy_x_amrs_seeds1-8` | 261105 13:02 | B1/B2/B3/B3-O × AMR 1·2·3대, 사람 8명, seed 1–8 | 96 (유효 95) | **유효. 인용할 것.** `REPORT.md`에 가설별 정리. B2 3/8, B3 7/8, B3-O 8/8·7/8, B1 8/8 |
| `261105_cross_design_check1` … `check4` | 261105 21:33–22:11 | 교차 시나리오 설계 검증, seed 2, B2·B3 | 1·2·2·2 | check. 1: a2가 교차로 코너 카트 옆에서 정지 + 목표 앞 오판. `check2_INVALID_no_fleet`: **무효**, dispatcher가 죽어 플릿 미출발. 3: a1이 동쪽 교차로 코너에서 정지. 4: B2·B3 모두 54 s 도달 → 본 실험 착수 |
| `261105_cross_seeds1-8` | 261105 22:20 | 교차 시나리오(AMR 3대가 서로·사람과 교차), 정책 4개 × seed 1–8 | 32 | 유효. `REPORT.md`. 아래 merged에 포함됩니다 |
| `261106_cross_seeds9-33` | 261106 18:23 | 같은 교차 시나리오, 정책 4개 × seed 9–33 | 100 (유효 99) | 유효. 원본 행. seed 23 B3는 물리 발산으로 유실 |
| `261107_MAIN_cross_seeds1-33_merged` | 261107 | 위 두 폴더를 합친 33 seed × 정책 4개 | 132 (유효 131) | **유효. 인용할 것.** `REPORT.md`, 조건별 표(`cross_tables.md`), 실패 메커니즘(`deadlock_mechanism.md`), 그림 P1–P12. `SOURCES.md`에 출처 |

## 이름 규칙

`YYMMDD_내용[_상태]`. 상태는 이름에서 바로 보이게 둡니다:

| 꼬리표 | 뜻 |
|---|---|
| `MAIN` | 그 날짜의 기준 비교. 인용할 것은 이것 |
| `INVALID` | 측정 조건이 깨져 있었음. 본문의 이유 참조 |
| `PARTIAL` | 중간에 멈춤. 셀이 비어 비교 불가 |
| `check` | 수정 하나를 확인한 소규모 실행 |
| (없음) | 유효하되 보조적인 실행 |

## 읽을 때

- `valid` 열이 `False`인 행은 시행 자체가 측정에 실패한 것입니다. `invalid_reason`에 이유가 있습니다.
- `host_load`는 시행이 끝난 시점의 1분 부하입니다. 24코어 기계에서 20을 넘기면 시뮬레이션이 마감을 놓치고 있었을 수 있습니다.
- `backoffs`, `off_lane_recoveries`는 게이트가 로봇을 빼낸 횟수입니다. 정책이 아니라 결함을 우회한 횟수이므로, 자주 나오는 칸은 배치 설계를 다시 봐야 합니다.
- 시행은 결정적이지 않습니다. n=4로는 도달률 차이를 가릴 수 없습니다.
