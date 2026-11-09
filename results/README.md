# results/ — 무엇이 어디에 있는지

| 폴더 | 내용 | 먼저 볼 것 |
|---|---|---|
| `batches/` | **실험 묶음 전부.** 폴더 하나 = 한 번의 실행, 이름은 `날짜_상태_내용` | `batches/README.md`의 표 (무엇을 비교, n, 유효 여부) |
| `diagnostics/` | 비교가 아닌 진단 실행. 코드가 바뀌는 도중에 한 번씩 돌린 것 | `diagnostics/261104_costmap_steering/tables.md` |
| `scenarios/` | 생성된 시나리오의 해석값 JSON (`configs/scenarios/*.yaml`과 1:1) | `docs/SCENARIO.md` |
| `notion/` | 날짜별 실험 기록 글과 작은 스틸 (`YYMMDD/YYMMDD_제목.md`) | 가장 최근 날짜 폴더 |
| `real/` | 실제 AMR 기록과 로봇에 올리는 스크립트 (`amr_deploy/`) | `docs/REAL_ROBOT_PLAN.md` |
| `stress/` | 무작위 변형 스트레스 실행 (`wave01` …) | `scripts/stress_wave.sh` |
| `trials/` | 묶음으로 돌리지 않은 초기 단발 trial | — |
| `videos/` | 녹화 (git 제외) | `README.md`의 녹화 절 |
| `cache/`, `factory/` | 생성물·미리보기 (git 제외) | — |
| `b2/`, `b3/` | 초기 구조의 빈 자리 (`.gitkeep`) | — |

논문에 인용할 결과는 `batches/`에서 이름에 `MAIN`이 붙은 폴더입니다.

- `batches/261105_MAIN_pilot_policy_x_amrs_seeds1-8/REPORT.md` — 정책 4개 × AMR 1·2·3대, 96 trial
- `batches/261107_MAIN_cross_seeds1-33_merged/REPORT.md` — 교차 시나리오, 33 seed, 132 trial

그래프는 각 폴더의 `figures/`에 있고, 같은 폴더의 `REPORT.md` 끝에 다시 만드는 명령이 있습니다.
