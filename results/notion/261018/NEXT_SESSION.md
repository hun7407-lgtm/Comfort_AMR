# 다음 세션 체크리스트 — 세 자세 재녹화

2026-10-18에 `station_engaged`와 `stationary`는 **녹화 전에** 찍혀서 3D 점군이
없습니다. 이번에는 녹화를 먼저 걸고 세 자세를 연속으로 받습니다.

## 준비 (로봇 켠 뒤)

```bash
# [데스크톱]
ssh-copy-id clabil@<IP>                                   # 새 로봇이면 한 번만
~/Desktop/Comfort_AMR/results/real/amr_deploy/deploy.sh clabil@<IP>
```

IP가 바뀌면 `results/real/261018/fastdds_desktop.xml`의 주소도 바꿔야 RViz가 붙습니다.

## 1. 로봇 전원

매뉴얼 15쪽 그대로. **배터리 2개** 연결, 측면 동그란 버튼 **짧게**(길게 = 끄기),
E-STOP은 RESET 방향으로 돌려 해제.

## 2. 세션 시작 — 한 줄

```bash
# [AMR]
~/session_start.sh states
```

`start_all.sh` → `preflight.sh`(7항목) → SLAM → **bag 녹화 시작**까지 합니다.
preflight가 실패하면 **거기서 멈춥니다.** 2026-10-18에 세 시간을 날린 증상
(배터리 0.0 V / 엔코더 NaN / `parsed successfully` 0회 / USB error -71)이
전부 1분 안에 걸립니다.

## 3. 지도 — 주행해서

```bash
# [AMR]
~/remap_empty.sh      # 3.5 m 전후진 6회, 약 3분. 복도를 비우고
```

정지 상태로 만든 지도는 **점유 셀이 70개**뿐이라 벽이 점선이 되고, Nav2가 점 하나를
0.45 m로 부풀려 벽에서 튀어나온 동그라미를 만듭니다. 주행 지도는 709개입니다.

## 4. 세 자세 — 녹화는 계속 돌아가는 상태

각 자세 **20초**, 시작할 때 표시를 남깁니다.

```bash
# [AMR]
~/mark.sh 'engage start' ;  ~/capture_state.sh station_engaged
~/mark.sh 'idle start'   ;  ~/capture_state.sh stationary
~/mark.sh 'walk start'   ;  python3 ~/track_person.py 24 1.0
```

- **engage** — 책상 앞, 책상을 보고
- **idle** — 오른쪽 벽 쪽, 작업 위치가 아닌 곳
- **walk** — 복도 가운데를 일정한 속도로 로봇 쪽으로

배치는 `where_to_stand.png` 참조.

### Station 자리를 옮길 것

지금 책상 자리(x +5.32)는 **빈 복도에서도 0.05 m**뿐이라 `station_engaged`의
기여를 분리할 수 없습니다. 책상을 복도 중간으로 옮기거나, 복도가 열린 지점을
Station으로 가정해야 합니다.

## 5. 종료

```bash
# [AMR]
~/session_stop.sh        # bag을 정상 종료시켜 metadata.yaml을 씁니다
~/stop_all.sh
sudo shutdown -h now     # SD카드 보호. 그 뒤 본체 전원 OFF
```

`tmux kill-session`으로 끄면 rosbag2가 마무리를 못 해 `metadata.yaml`이 안 써지고,
`ros2 bag info`/`play`가 거부합니다(2026-10-18에 그렇게 됐습니다).

## 6. 데스크톱으로

```bash
# [데스크톱]
scp -r clabil@<IP>:'~/bags/<name>' ~/Desktop/Comfort_AMR/results/real/bags/
scp clabil@<IP>:'~/maps/empty_corridor.*' ~/Desktop/Comfort_AMR/results/real/<날짜>/
```

3D 뷰어 만들기:

```bash
python3 results/real/261018/bag_to_3d.py <bag>/<bag>_0.db3 results/real/261018/cloud_viewer
results/real/261018/cloud_viewer/view.sh
```

## 알려진 제약

- **Nav2 `CostmapFilter`는 갱신 영역을 넓히지 않습니다.** 마스크만 바뀌면 costmap이
  안 따라옵니다. 상태별 캡처 사이에 Nav2를 재시작해야 합니다. 실시간화를 하려면
  이것부터 풀어야 합니다.
- **Humble ↔ Jazzy는 같은 도메인 불가.** 데스크톱은 Humble 컨테이너나 foxglove로만.
- **WiFi가 복도 끝에서 끊깁니다.** 주행·녹화는 전부 tmux 안에서 돌립니다.
