# 3D LiDAR + SLAM 버전

기존 2D 파이프라인은 **그대로 둡니다**. 이 폴더는 같은 Scene, 같은 사람,
같은 Comfort Space 정책을 쓰면서 **인식 부분만** 바꾼 별도 버전입니다.
`configs/factory.yaml`로 받은 결과는 계속 유효하고, 두 버전을 섞어
비교하지 않도록 녹화 폴더(`recording.version`)도 따로 갑니다.

| | 2D (기존) | 3D (이 폴더) |
|---|---|---|
| config | `configs/factory.yaml`, `configs/factory_long.yaml` | `configs/factory_3d.yaml`, `configs/factory_long_3d.yaml` |
| costmap 장애물 | `ObstacleLayer` ← `/scan` 한 겹 | `VoxelLayer` ← `/lidar/cloud` (PointCloud2) |
| collision monitor | `scan` 소스 | `pointcloud` 소스 |
| 지도 | `scripts/generate_map.py`가 config에서 생성 | **SLAM으로 직접 작성** (`nav3d/maps/*`) |
| Nav2 파라미터 템플릿 | `configs/nav2/nav2_params.template.yaml` | `nav3d/nav2_params_3d.template.yaml` |
| 녹화 | `v5_factory_bay`, `v6_factory_long` | `v7_factory_3d`, `v7_factory_long_3d` |

바꾸지 않은 것: Scene(건물·설비·도색), 사람 배치와 상태, B1/B2/B3 정책,
NavFn + DWB, 판정 규칙, Comfort Space keepout 필터, yield gate.

## 쓰는 방법

```bash
# 1. 지도를 한 번 만든다 (순환형)
./scripts/run_mapping.sh --config configs/factory_3d.yaml --headless

# 2. 그 지도로 trial을 돌린다
./scripts/run_b1.sh --headless --config configs/factory_3d.yaml
./scripts/run_b2.sh --headless --config configs/factory_3d.yaml
./scripts/run_b3.sh --headless --config configs/factory_3d.yaml
```

긴 복도형도 같습니다. `configs/factory_long_3d.yaml`로 바꾸면 됩니다.

지도가 없는 상태로 trial을 돌리면 `generate_nav2_params.py`가 먼저 막고
위 1번 명령을 알려줍니다. Nav2가 올라간 뒤 map_server만 조용히 활성화에
실패하는 것보다 낫습니다.

## 지도를 trial마다 만들지 않는 이유

B1·B2·B3는 **Comfort Space 정책 하나만** 달라야 합니다. trial마다 SLAM을
돌리면 세 조건이 서로 미묘하게 다른 건물에서 경로를 찾게 되고, 결과 차이가
정책 때문인지 지도 때문인지 구분할 수 없습니다. 그래서 지도는 한 번 만들어
세 조건이 **같은 파일**을 읽습니다. Scene을 바꿨을 때만 다시 만듭니다.
자동으로 다시 만들지 않습니다 — 실험이 자기 지도를 조용히 바꿀 수 있으면
안 됩니다.

## 결과 (2026-10-06, 녹화본)

순환형 12 × 12 m — `configs/factory_3d.yaml`

| | 결과 | 시간 | 이동거리 | c 최근접 | 접촉 | 침범 | 안전계층 |
|---|---|---|---|---|---|---|---|
| B1 | SUCCESS | 33.3 s | 14.14 m | **−0.059 m 접촉** | 2.4 s | 1.97 m / 5.4 s | 0.9 s |
| B2 | SUCCESS | 55.9 s | 17.77 m | +0.774 m | 0 s | 0.18 m / 2.0 s | 0.1 s |
| B3 | SUCCESS | 41.3 s | 12.91 m | +0.669 m | 0 s | 0.29 m / 2.5 s | 0.5 s |

긴 복도형 24 × 7.5 m — `configs/factory_long_3d.yaml`

| | 결과 | 시간 | 이동거리 | c 최근접 | 접촉 | 침범 | 안전계층 |
|---|---|---|---|---|---|---|---|
| B1 | SUCCESS | 60.1 s | 23.84 m | **−0.056 m 접촉** | 1.7 s | 1.96 m / 4.7 s | 1.1 s |
| B2 | SUCCESS | 119.6 s | 26.57 m | +0.720 m | 0 s | 0.24 m / 2.3 s | **11.4 s** |
| B3 | SUCCESS | 74.7 s | 24.59 m | +0.717 m | 0 s | 0.25 m / 2.2 s | 1.0 s |

지도는 설계값과 7 cm 이내로 일치합니다(순환형 `x[-6.07, +6.03] y[-6.30, +6.20]`,
설계 ±6.00). 2D 순환형 결과와 비교하면 최근접 거리가 모두 2 cm 안쪽으로 같습니다 —
인식을 바꿔도 정책의 효과는 같습니다.

돌아갈 길이 없는 긴 복도형에서 차이가 크게 벌어집니다. B2는 B3보다 44.9 s 느리고
(순환형에서는 14.6 s), 충돌 감시기가 11.4 s 동안 개입합니다(B3는 1.0 s).

영상은 `results/videos/v7_factory_3d/`, `results/videos/v7_factory_long_3d/`.
자세한 정리는 [results/notion/261006/](../results/notion/261006/).

## 매핑 주행

`scripts/run_mapping.sh`가 하는 일:

1. `nav3d/mapping_<config>.yaml`로 Isaac Sim을 띄웁니다. 이 config는
   원본 Scene을 상속하고 **사람만 끕니다**. 지도는 움직이지 않는 것의
   기록이어야 하고, 서 있는 작업자 셋을 구워 넣으면 모든 trial이 없을지도
   모르는 사람을 피해 경로를 잡습니다. `generate_map.py`가 사람을 넣지
   않는 것과 같은 이유입니다.
2. `nav3d/mapping.launch.py`로 `pointcloud_to_laserscan`과
   `slam_toolbox`(async, mapping)를 띄웁니다. 이때 `map -> odom`
   정적 변환은 **띄우지 않습니다** — 그건 지금 slam_toolbox가 내는 값이고,
   한 변환에 발행자가 둘이면 TF 트리가 둘 사이를 깜빡입니다.
3. `nav3d/patrol.py`가 통로 중심선을 따라 한 바퀴(기본 2바퀴) 돕니다.
   경로는 레이아웃 모델에서 뽑으므로 통로를 옮기거나 넓히면 따라옵니다.
   순환형은 통로 2줄과 가로 통로 2곳을 잇는 사각 루프, 긴 복도형은 왕복입니다.
   사람이 없는 빈 베이이므로 플래너 없이 open-loop로 몰아도 안전합니다
   (플래너를 쓰려면 지도가 있어야 하는데, 지도를 만들러 온 것입니다).
4. `map_saver_cli`로 저장하고, `nav3d/place_map.py`가 **월드 좌표로**
   옮깁니다.

## 왜 좌표를 옮겨야 하나

slam_toolbox는 첫 스캔을 받은 자리, 즉 `navigation.start`를 지도 원점으로
잡습니다. 그런데 이 프로젝트의 나머지 전부(목표점, 스테이션, 사람,
Comfort Space 마스크, 여유폭 검사)는 월드 좌표로 말합니다. 그대로 쓰면
목표점이 베이 반쯤 엉뚱한 곳을 가리킵니다.

소비자 쪽을 전부 고치는 대신 지도를 한 번 옮깁니다. `place_map.py`가
저장된 yaml의 `origin`만 다시 씁니다(픽셀은 건드리지 않습니다). 그 뒤로는
SLAM 지도와 생성 지도가 같은 월드를 가리키므로,
`scripts/nav2_bringup.launch.py`의 `map -> odom` 정적 변환도 2D와 똑같이
두면 됩니다.

같은 스크립트가 **지도 범위를 레이아웃 기대값과 나란히 출력**합니다. 한쪽이
0.5 m 이상 모자라면 주행이 건물을 다 보지 못한 것이고, 플래너가 벽으로
아는 곳에 들어가지 않는 것으로 나중에 깨닫는 것보다 여기서 아는 게 낫습니다.

## 3D 센서로 2D SLAM을 하는 이유

Nav2의 static layer는 2D 점유 격자입니다. 그게 인터페이스이고, 3D 지도를
넣으려 해도 결국 눌러야 합니다. 그래서 **지도는 스캔 한 겹으로** 만들고,
**costmap은 클라우드 전체를** 읽습니다. 선반 돌출부, 대차 프레임, 사람의
어깨가 각각 실제 높이에 표시되고 비워지는 쪽이 costmap입니다.

지도는 "건물이 어디 있나", voxel layer는 "지금 앞에 뭐가 있나"에 답하고,
3차원이 필요한 것은 두 번째뿐입니다.

진짜 3D SLAM(rtabmap)으로 가려면 `sudo apt install ros-jazzy-rtabmap-ros`가
필요합니다. 설치돼 있지 않아 지금은 여기까지입니다.

## 파일

```
nav3d/
├── README.md                   이 문서
├── nav2_params_3d.template.yaml  VoxelLayer costmap + pointcloud collision monitor
├── mapping.launch.py           slam_toolbox 브링업 (매핑 전용)
├── slam_params.yaml            slam_toolbox 튜닝 (프레임·토픽은 config에서 덮어씀)
├── mapping_factory_3d.yaml     매핑용 Scene: 사람 끈 순환형
├── mapping_factory_long_3d.yaml 매핑용 Scene: 사람 끈 긴 복도형
├── patrol.py                   통로 중심선 한 바퀴 (플래너 없음)
├── place_map.py                저장된 지도를 월드 좌표로
└── maps/                       결과물 (pgm + yaml)
```

`scripts/launch_nodes.py`는 2D와 3D 브링업이 **같은** `pointcloud_to_laserscan`
설정을 쓰도록 공용으로 뺀 것입니다. 두 곳에 따로 적어두면 매핑 때 보는
높이 구간과 주행 때 보는 구간이 어긋나고, 그러면 보이는 것과 다른 지도가
됩니다.
