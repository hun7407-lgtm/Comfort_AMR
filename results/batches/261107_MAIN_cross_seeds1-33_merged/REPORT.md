# 261107_MAIN_cross_seeds1-33_merged 보고서 — 교차 시나리오 33 seed × 4 정책 = 132 trial (2026-11-05 22:20 ~ 11-07 02:06)

`results/batches/261105_cross_seeds1-8`(seed 1–8, 32 trial)와 `results/batches/261106_cross_seeds9-33`(seed 9–33, 100 trial)을 `scripts/merge_campaigns.py`로 합친 것. 시나리오·정책·코드는 두 캠페인에서 동일(HEAD 60cf683, 미커밋 수정 포함). 8 seed 보고서는 `results/batches/261105_cross_seeds1-8/REPORT.md`(설계 이유·검증 trial 이력은 거기에).

산출물: `trials.csv`, `fleet_trials.csv`, `tables.md`, `cross_tables.md`, `deadlock_mechanism.md`, `figures/P1–P12`, `logs/`(symlink).

---

## 0. 한 줄 요약

| | B1 none | B2 static | B3 state | B3-O directional |
|---|---|---|---|---|
| AMR1 성공 | 31/33 (94%) | 25/33 (76%) | 27/33 (82%, 1 유실) | 27/33 (82%) |
| 실패 원인: Comfort 게이트 홀드 | 0 | **5** | 0 | 1 |
| 실패 원인: 회피 후 정지(post-evade stall) | 0 | 3 | 4 | 4 |
| 실패 원인: Nav2 abort(사람이 로봇을 뚫음) | 2 | 0 | 1 | 1 |
| 이동시간 − B1 (성공, 중앙값 / 평균 / 최대) | 0 | +6.6 / +7.4 / +22.8 s | +3.4 / +5.3 / +15.4 s | +0.2 / +4.0 / +15.0 s |
| 벨트 작업자 최근접 (평균) | 0.12 m | 0.69 m | 0.42 m | 0.40 m |
| 로봇이 다가간 접촉이 있는 trial | 0% | 21% | 6% | 0% |
| fleet 차량 성공 (66행) | 88% | 90% | 89% | 94% |

**결론을 먼저 쓰면**: (1) B2의 실패는 대부분(5/8) 논문이 말하는 교착 — 서 있는 사람의 상태-무관 1.20 m 원이 통로를 막아 양보 게이트가 영원히 닫힌 것 — 이고, B3/B3-O에는 이 유형이 0/1건이다. (2) 그러나 8 seed 때와 달리 B3/B3-O도 각각 4건의 **회피 후 정지**로 실패했다. 걷는 사람을 피하려고 옆으로 비켜난 뒤(대피 슬롯·벨트 가장자리·교차로) Nav2가 다시 출발하지 못한 경우로, 같은 회피 모듈을 쓰는 B2에서도 3건 나왔다. 정책의 Comfort 크기가 아니라 회피 모듈 + 지형의 문제이며, 이 유형을 빼면 성공률은 B1 31/31, B2 25/30, B3 27/28, B3-O 27/29다. (3) 사람이 AMR1 앞을 가로지르지 않는 11 seed에서 B3/B3-O의 시간 비용은 0이고, 가로지르는 22 seed에서는 +6~9 s(양보 시간)다.

---

## 1. 무엇이 seed마다 바뀌는가 (조건)

임무·fleet 경로·지도는 고정. seed는 교대 짝과 출발 시각만 바꾼다. 북쪽 교대 짝이 AMR1이 겪는 상황을 결정한다.

| 북쪽 짝 | seed 수 | AMR1에게 일어나는 일 |
|---|---|---|
| N1 (x=−6.5) | 11 | AMR1 출발점 뒤 → 아무도 앞을 가로지르지 않음 |
| N2 (x=−0.5) | 13 | N2가 정면으로 걸어오고 E1이 지나감 → Comfort 정책은 양보. E1이 N2 작업대에 설 때 AMR1은 이미 옆을 지나는 중 |
| N3 (x=5.5) | 9 | 위와 같이 양보한 뒤, **E1이 AMR1 앞쪽 N3 작업대에 서 있음** → 상태-무관 Comfort Space가 통로를 막음 |

남쪽 짝(S1/S2/S3)은 모두 a2 셔틀의 경로를 가로지른다(fleet 결과, §4).

---

## 2. 데이터 유효성

- 132 trial 중 131 유효. **유실 1**: seed 23 B3 — t≈19 s에 로봇 바퀴 transform이 NaN(`TF_NAN_INPUT`, 물리 발산), 시뮬레이터가 멈춰 watchdog 1500 s에 종료. trials.csv에 TIMEOUT 행으로 남음, 성공률 분모에서 제외하지 않고 "유실"로 표기.
- 기동 재시도 9건(Isaac Sim 미기동 1, fleet Nav2 활성화 실패 7, TF NaN 1). 모두 재시도본만 기록(`*.attempt1.log` 보존). seed 10 B2는 1차 시도가 fleet 행 2개를 쓴 뒤 죽어 `fleet_trials.csv`에 a1/a2 중복 행이 있다(B2 fleet 행 68개). fleet 집계는 그대로 두고 여기 적는다.
- fleet 물리 튕김 1건(seed 6 B3 a1, 8 seed 보고서 §2).
- `deadlock_cause`의 " while yielding"은 "한 번이라도 양보한 적 있음"이라는 뜻이라 메커니즘 분류에 쓰지 않았다. 대신 `scripts/deadlock_mechanism.py`가 로그의 마지막 2 s 추적선 플래그(YIELDING / EVADING / 없음)와 Nav2 상태로 분류한다(§3.2).

---

## 3. 가설별 결과

### 3.1 전제: 세 교차는 일어났는가 — 예 (8 seed 보고서 §3.1과 동일)

`cross_tables.md`, P9, P10. 로봇–로봇: 성공 trial 전부에서 AMR1이 a1 2 m 이내에 2–4 s, 최소 간격 1.0–1.8 m, 멈춤 0 s, 로봇끼리 접촉 0. 사람–로봇: a2는 33 seed 전부에서 남쪽 보행자와 0.25 m 이내. AMR1은 22 seed(N2·N3)에서 북쪽 보행자와 마주침. 사람–사람: 교대 짝이 엇갈림(생성기 고정).

### 3.2 H-교착: 상태를 보지 않는 Comfort Space는 교착하고, 상태를 보는 것은 통과한다 — 증명, 단 조건부

P7(seed 행렬), P11(조건별), P12(메커니즘), `deadlock_mechanism.md`.

**N3 조건(9 seed)** — 사람이 걸어와 AMR1 앞 작업대에 서는 경우:

| | B1 | B2 | B3 | B3-O |
|---|---|---|---|---|
| 성공 | 8/9 | **2/9** | 7/9 | 5/9 |
| 게이트 홀드 | 0 | 5 (seed 5, 17, 28, 33 + N2의 10) | 0 | 1 (seed 25) |
| 회피 후 정지 | 0 | 2 (11, 13) | 1 (12) | 2 (12, 17) |
| Nav2 abort | 1 (12) | 0 | 1 (11) | 1 (13) |
| 이동시간 − B1 (성공) | 0 | +22.8 s | +9.0 s | +5.9 s |
| 양보 시간 (평균) | 0 | 16.6 s | 11.6 s | 11.8 s |

- B2의 게이트 홀드 5건은 모두 같은 모양이다: 걷는 두 사람을 보낸 뒤 (4.4–5.5, 3.8–4.3)에서 **N3 작업대(5.5, 3.01)에 선 E1로부터 1.20 + 0.35 m** 되는 지점에 양보 상태로 서서 12 s 이상 움직이지 않음(양보 21–25 s). seed 10(N2 조건)의 1건은 N2 작업대 옆 (−1.39, 3.92)에서 같은 방식. B3/B3-O는 같은 사람을 station_engaged로 읽어 0.60–0.80 m 원을 두고 통과했다(seed 5, 26, 28, 33: B3/B3-O 모두 성공, +8~12 s).
- B3-O의 게이트 홀드 1건(seed 25): 회피 후 벨트 쪽 (4.73, 3.69)에 있었고 N3 작업대까지 1.03 m = 방향성 옆 반지름 0.70 + 완충 0.35 — 옆에서 접근하면 B3-O도 막힌다. 같은 seed의 B3(옆도 0.60)는 통과(59.3 s). **B3와 B3-O가 갈린 유일한 사례**이며, 방향성 Comfort가 옆 접근에서 더 보수적이라는 설계대로다. 1건이라 경향으로만 둔다.
- **N1·N2 조건(24 seed)**에서는 어느 정책도 게이트 홀드가 없다(seed 10 B2 제외). B2 교착은 "서 있는 사람이 로봇 앞에 있다"는 조건에 묶여 있고, 8 seed 때 1/8이던 것이 33 seed에서 N3 조건 5/9(+N2 1/13)로 재현됐다.

**왜 이 시나리오가 이 가설을 보여주는가**: pilot의 B2 교착은 출발/도착 슬롯이 작업자 옆일 때 생겼다. 여기서는 임무가 고정돼 그런 슬롯이 없는데도, 교대로 걸어온 사람이 작업대에 서는 순간 통로 한가운데 상태-무관 1.20 m 원이 생긴다. 즉 교착은 배치가 아니라 "사람의 상태가 바뀌었는데 Comfort Space는 안 바뀐다"에서 온다. B3/B3-O는 radius를 줄여서가 아니라 서 있는 사람을 station_engaged로 읽었기 때문에 통과하고(상태 일치 98%), 걷고 있을 때는 B2와 같은 1.20 m로 양보한다(N2 조건 양보 시간 B2 9.8 / B3 9.0 / B3-O 8.9 s).

**B3/B3-O의 실패 8+2건은 다른 종류다 — 회피 후 정지.** 걷는 사람이 로봇을 피하지 않으므로, 정면으로 걸어오는 사람에게는 양보(정지)가 아니라 옆으로 비켜나야 한다(`comfort_core/evasion.py`, 세 Comfort 정책 공통). 비켜난 곳이 (a) 대피 슬롯 n2_r2c3 (1.4, 5.32)의 동쪽 끝 (1.75–1.85, 5.15–5.40) — 7 trial(B2 2, B3 2, B3-O 3), (b) 벨트 가장자리 y≈3.5 — 3 trial(B1 abort 포함), (c) 동쪽 교차로 코너 (9.4, −2.2) — 1 trial 이면 Nav2가 유효한 plan을 가진 채 12 s 동안 속도 0.02 m/s 이하로 머물렀다. 특히 (a)는 7 trial이 정확히 같은 자리에서 멈춰 지형 트랩(슬롯 동쪽 끝의 블록 모서리 + inflation)으로 보인다. pilot seed 7의 교차로 코너 실패와 같은 부류다. 이 trial들의 Nav2 로그는 보존되지 않아(기동 실패 시에만 보관) 컨트롤러가 왜 안 나왔는지는 확정하지 못했다. 정책 간 빈도는 B2 3 / B3 4 / B3-O 4로 비슷하다.

판정: **증명됨(조건부).** B2의 교착은 "서 있는 사람이 앞에 있는" 조건에서 5/9로 재현되고 B3/B3-O는 0/9·1/9. 그러나 B3/B3-O의 총 성공률이 B2보다 6%p 높은 데 그친 이유는 회피 모듈이 만드는 정책-공통 실패 때문이며, 이것을 Comfort 정책의 결함으로 쓰면 안 된다. 논문에는 실패를 메커니즘별로 나눠 쓰는 것이 맞다(P12).

### 3.3 H-시간: 상태 인식 Comfort Space의 시간 비용은 작다 — 증명

P2, P11 가운데, `tables.md`.

| 조건 | B2 | B3 | B3-O |
|---|---|---|---|
| N1 (11 seed, 아무도 안 지나감) | +2.2 s | −0.1 s | −0.1 s |
| N2 (13 seed, 보행자 가로지름) | +11.3 s | +7.5 s | +6.8 s |
| N3 (9 seed, 가로지름 + 앞에 섬) | +22.8 s (2 성공) | +9.0 s | +5.9 s |
| 전체 중앙값 / 평균 | +6.6 / +7.4 | +3.4 / +5.3 | +0.2 / +4.0 |

- 사람을 안 만나면 B3/B3-O는 B1과 같다. B2는 벨트 작업자의 1.20 m 원이 통로를 1.09 m 덮어 창고 쪽 벽을 타며 +2 s.
- 사람이 가로지르면 세 정책 모두 양보 7–12 s. B3-O < B3 < B2 순서가 세 조건에서 유지된다(B3-O가 B2보다 N2 4.5 s, N3 17 s 빠름). B3-O가 B3보다 빠른 것은 방향성 Comfort가 뒤쪽을 좁게 두어 회피에서 빨리 빠져나오기 때문으로 보이나, P2에서 두 정책의 분포는 겹친다.
- B3/B3-O가 B1보다 빠른 seed가 몇 개 있다(−3, −6, −12 s): B1이 사람에게 뚫려 보호정지로 기다린 seed다.

### 3.4 H-Comfort: 상태 인식 Comfort Space는 사람의 편안함을 B1보다 지키고, B2와 같은 급 — 증명(B1 대비), 같은 급(B2 대비)

P4, P10, `tables.md`.

- 벨트 작업자 최근접(33 seed 평균): B1 0.12 m, B2 0.69, B3 0.42, B3-O 0.40. B1은 N2/N3를 0.0–0.3 m로 지나가고 N3 조건 9 seed 전부에서 접촉(−0.01 m). B3/B3-O의 0.4 m는 두 성분이다: 사람을 안 만나는 seed에서는 0.72–0.76 m(stationary 0.90 원 바깥), 만나는 seed에서는 보행자가 양보 중인 로봇을 뚫고 지나가며 생기는 −0.08~+0.1 m.
- 창고측(E) 최근접은 B1 0.93 vs Comfort 정책 0.41–0.48로 거꾸로 보이지만, 8 seed 보고서와 같은 이유(E1이 양보한 로봇을 뚫고 지나감)다.
- **로봇이 다가간 접촉("robot drove in")이 있는 trial**: B1 0%, B2 21%(7/33), B3 6%(2/33), B3-O 0%. 모두 회피 중 보행자와 스친 순간(0.1–0.7 s)이다. B2가 많은 것은 회피 시작이 늦고(큰 원에 갇혀) 비켜날 공간이 적기 때문으로 보인다. B3-O 0%는 33 seed에서 유지됐다.
- B3 vs B3-O: 최근접·양보·접촉 모두 같은 급. seed 25(옆 접근 게이트 홀드)와 로봇 접촉 0% 외에는 가르는 데이터가 없다.

### 3.5 fleet(a1, a2) — 참고

P5, `cross_tables.md`.

- a1(북 통로 선행): 66행 중 실패 3(B3 seed 6 물리 튕김, B3 seed 23 유실, B3-O 1건). AMR1이 a1 때문에 멈춘 trial은 없다.
- a2(남 통로 셔틀, 보행자와 정면): 교착 B1 8 / B2 6(+abort 1) / B3 5 / B3-O 3 (각 33). 순서는 8 seed 때와 같고 표본이 4배인데 유지됐다. 모두 "보행자가 멈춘 a2를 뚫고 지나감 → Nav2 복구 → 창고측 사람 옆에서 안전층에 잡힘"의 순서. fleet에는 양보 게이트가 없고 Comfort costmap만 있다. 경향으로 기록한다.

---

## 4. 논문에 쓸 수 있는 문장과 쓰면 안 되는 문장

쓸 수 있음:
- "사람이 걸어와 로봇 앞 작업대에 서는 조건(9 seed)에서 상태-무관 Comfort Space(B2)는 5/9 교착했고, 상태 인식(B3)은 0/9, 방향성(B3-O)은 1/9였다. 교착 지점은 모두 서 있는 사람으로부터 1.20 + 0.35 m."
- "보행자를 만나지 않으면 상태 인식 정책의 시간 비용은 0이고, 만나면 양보 시간만큼(+6~9 s)이다. B2는 같은 상황에서 +11~23 s."
- "B1은 벨트 작업자를 평균 0.12 m로 지나고 N3 조건 9/9에서 접촉했다. B3/B3-O는 0.40 m, 로봇이 다가간 접촉 B3-O 0%."
- "로봇–로봇 교차는 어느 정책에서도 비용 0."

쓰면 안 됨:
- "B3가 B2보다 성공률이 높다"를 전체 성공률(82% vs 76%)로 주장하는 것. 차이의 대부분은 정책-공통 회피 후 정지가 섞여 작아진 것이고, 메커니즘별로 나눠야 한다.
- B3 vs B3-O 우열. 1건 + 접촉 0%뿐.
- a2의 정책별 교착 차이(양보 게이트 없음, 메커니즘 미분석).

---

## 5. 이번에 추가한 도구 (미커밋)

| 파일 | 내용 |
|---|---|
| `scripts/merge_campaigns.py` | 캠페인 폴더 병합(csv 연결, 로그 symlink에 폴더명 접두) |
| `scripts/cross_figures.py` | P11 조건별(북쪽 짝 N1/N2/N3) 그림·표 추가 |
| `scripts/deadlock_mechanism.py` | 실패 trial을 게이트 홀드 / 회피 후 정지 / Nav2 abort / 유실로 분류, 정지 위치(통로/대피 슬롯/벨트 가장자리/교차로), P12 + `deadlock_mechanism.md` |
| `scripts/pilot_extra_figures.py` | P7 폭을 seed 수에 맞춤 |
| `configs/campaigns/cross/campaign_h8_a3_cross_s9..33.yaml`, `configs/scenarios/paper_h8_a3_scr_s0009..0033.yaml` | 생성기 동일(seed 1 재생성 바이트 일치 확인) |

Comfort radius, safety, Nav2 내부, 사람 궤적, 회피 모듈은 손대지 않았다. 사람 회피 모듈은 사용자 지시로 만들지 않았다.

---

## 6. 한계와 다음 단계

1. **회피 후 정지(11건)**가 이번 데이터의 가장 큰 잡음이다. 7건이 대피 슬롯 n2_r2c3 동쪽 끝 같은 자리에서 멈췄다. 후보 원인: 슬롯 동쪽 모서리 + inflation 0.34 m가 컨트롤러의 모든 궤적을 막음. 확인하려면 그 자리에서 Nav2 로그를 보존한 재현 trial 1~2회가 필요하다. 고치는 방법은 회피 모듈이 그 슬롯을 대피처로 고르지 않게 하거나(`evasion.py`의 refuge 후보 검사), 회피 종료 후 off-lane 복구를 호출하는 것이며, 어느 쪽이든 정책 공통이라 B2/B3/B3-O 비교를 왜곡하지 않는다. 고친 뒤에는 N3 조건 seed만 다시 돌려도 된다.
2. 사람이 로봇을 뚫는 전제는 그대로다. 창고측 최근접과 a2 접촉은 이 전제의 산물.
3. fleet 양보 게이트 부재, `pose_is_sane` 미적용(물리 튕김이 DEADLOCK로 남음), " while yielding" 라벨, 교차로 코너 트랩 — 8 seed 보고서 §6과 같다.
4. B3 vs B3-O를 가르는 임무(서 있는 사람의 옆/뒤를 지나는 경로)는 여전히 없다. seed 25가 힌트: 옆 접근에서는 B3-O가 더 보수적이다.

---

## 7. 그림 목록과 재현

| 그림 | 내용 |
|---|---|
| `figures/P1_amr1_outcome.png` | AMR1 성공/교착률 |
| `figures/P2_time_cost_paired.png` | 같은 seed의 B1 대비 이동시간 차(33 seed) |
| `figures/P3_arrival_curves.png` | 도착 곡선 |
| `figures/P4_closest_by_whom.png` | 벨트 작업자 / 창고측 사람 최근접 |
| `figures/P5_fleet.png` | fleet 성공률·처리량 |
| `figures/P6_geometry.png` | Comfort Space가 통로를 덮는 폭 |
| `figures/P7_seed_matrix.png` | seed × 정책 결과 행렬 (33 seed) |
| `figures/P8_trace_seed{5,12,15,25}.png` | 궤적: 5 = B2 게이트 홀드, 12 = B3/B3-O 대피 슬롯 정지 + B1 abort, 15 = N1 조건인데 B3/B3-O 슬롯 정지, 25 = B3-O 옆 접근 홀드 |
| `figures/P9_convoy_gap.png` | AMR1–a1 간격 시계열 (33 패널) |
| `figures/P10_interactions.png` | 세 교차 요약 |
| `figures/P11_by_condition.png` | 북쪽 짝 N1/N2/N3별 성공률·시간 비용·양보 |
| `figures/P12_deadlock_mechanism.png` | 실패 메커니즘·위치 |

```
python3 scripts/merge_campaigns.py --out results/batches/261107_MAIN_cross_seeds1-33_merged results/batches/261105_cross_seeds1-8 results/batches/261106_cross_seeds9-33
python3 scripts/report_pilot.py --dir results/batches/261107_MAIN_cross_seeds1-33_merged
python3 scripts/cross_figures.py --dir results/batches/261107_MAIN_cross_seeds1-33_merged
python3 scripts/deadlock_mechanism.py --dir results/batches/261107_MAIN_cross_seeds1-33_merged
python3 scripts/pilot_extra_figures.py --dir results/batches/261107_MAIN_cross_seeds1-33_merged --trace-seed 12 --trace-stem campaign_h8_a3_cross_s12 \
    --scenario-yaml configs/campaigns/cross/campaign_h8_a3_cross_s12.yaml --trace-label "3 AMRs" --xlim=-8,12
```
