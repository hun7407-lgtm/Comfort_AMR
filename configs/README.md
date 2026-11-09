# configs/ — 어떤 파일이 무엇인지

모든 config는 `base:`로 아래 것을 물려받고 자기 차이만 적습니다. 정책(B1/B2/B3…)은 `--scenario`로 따로 얹습니다.

## 바탕

| 파일 | 내용 |
|---|---|
| `common.yaml` | 모든 정책이 공유하는 값 전부 (로봇, 사람, Comfort Space, Nav2 입력, 측정) |
| `factory_u.yaml` | **논문 시나리오의 장면.** U자 조립 라인 24 × 16 m, 통로 1.5 m |
| `factory.yaml`, `factory_long.yaml` | 다른 배치: 순환형 12 × 12 m, 한 줄 긴 복도 |
| `factory_3d.yaml`, `factory_long_3d.yaml` | 같은 배치를 3D LiDAR·자체 작성 지도로 |
| `factory_u_lidar.yaml` | U 베이, 사람을 로봇 자신의 LiDAR로 검출 (Layer 2) |
| `real_amr.yaml` | 실제 AMR만의 값 (footprint, 속도, LiDAR 거리) |

## 정책 (`--scenario b1|b2|b3r|b3|b3d`)

| 파일 | 정책 | 보고서 표기 |
|---|---|---|
| `scenario_b1.yaml` | Comfort Space 없음, 안전거리만 | B1 |
| `scenario_b2.yaml` | 고정 1.20 m 원 | B2 |
| `scenario_b3r.yaml` | 상태별 반경, 원형 (B3의 ablation) | B3-R |
| `scenario_b3.yaml` | 상태별 반경 + 보행 방향으로 늘림 | B3 |
| `scenario_b3d.yaml` | 상태 + 몸 방향 + 속도 (방향성) | B3-O |

## 생성된 시나리오 — `scenarios/`

`scripts/make_scenario.py`가 seed 하나에서 만듭니다. 손으로 고치지 않습니다.

```
paper_h8_a3_scr_s0012.yaml
      │  │  │   └ seed 12
      │  │  └ 사람 상태를 script에서 읽음 (scr). 센서로 추정하면 sen
      │  └ AMR 3대 (AMR1 + a1 + a2)
      └ 사람 8명
뒤에 _loop / _ring이 붙으면 플릿 패턴
```

## 실험 묶음용 오버레이 — `campaigns/`

생성된 시나리오 위에 임무나 플릿만 바꾼 것. 파일 이름은 실행 당시 그대로라 결과 CSV의 `config` 열과 이름으로 이어집니다(당시 경로는 `configs/campaign_*.yaml`).

| 폴더 | 파일 | 무엇 | 돌린 묶음 (`results/batches/`) |
|---|---|---|---|
| `campaigns/fleet_patterns/` | `campaign_h8_s3_a4.yaml` = 사람 8, seed 3, AMR 4대 roundtrip<br>`…_loop_rule` / `_loop_norule` = 순환 플릿, 우측통행 on/off<br>`…_ring` = 한 방향 순환<br>`h0` = 사람 없음 | 플릿 패턴과 우측통행 비교 | `261105_fleet_patterns_keepright` |
| `campaigns/pilot_amr_count/` | `campaign_h8_s5_a2.yaml` = 사람 8, seed 5, AMR 2대<br>`…_a3` = 3대 | AMR 대수 사다리 (1대는 `scenarios/paper_h8_a1_*`) | `261105_MAIN_pilot_policy_x_amrs_seeds1-8` |
| `campaigns/cross/` | `campaign_h8_a3_cross_s12.yaml` = 사람 8, AMR 3대, 교차 임무, seed 12 | AMR 3대와 사람이 모두 교차하는 고정 임무 | `261105_cross_seeds1-8`, `261106_cross_seeds9-33` |

## 검증·시연용 오버레이 (측정 조건 아님)

| 파일 | 내용 |
|---|---|
| `fleet_nav2.yaml`, `fleet_nav2_norule.yaml` | 플릿을 Nav2로 (우측통행 on / off) |
| `fleet_loop.yaml`, `fleet_loop_norule.yaml` | 순환 플릿 진단 |
| `nav2_check.yaml` | 게이트 없이 costmap만으로 조종되는지 확인 |
| `yield_watch.yaml` | 양보 로직을 눈으로 보기 위한 임무 |
| `demo_blocked.yaml`, `demo_walker.yaml`, `demo_weave.yaml` | 시연 |

## `nav2/`

`nav2_params.template.yaml`이 원본이고 `nav2_params*.yaml`, `*_map.*`은 실행할 때마다 다시 만들어지는 생성물입니다(git 제외). `amr_nav2_params.yaml`은 참고용으로 가져온 원본입니다.
