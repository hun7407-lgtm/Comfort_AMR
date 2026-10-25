# 커밋 양식 (Comfort_AMR, 시뮬 · 실기 공통)

이 저장소와 실기 AMR 작업에 같이 쓰는 양식입니다. 다른 Claude 세션에서 실기
관련 커밋을 할 때도 이 파일을 먼저 읽히면 됩니다.

```
docs/COMMIT_STYLE.md
```

---

## 1. 제목 줄

```
Detect people in the 3D cloud by their vertical profile
```

- **명령형, 영어, 문장형 대문자, 마침표 없음.** "Added", "Adding"이 아니라 "Add".
- **72자 이내.** 넘으면 커밋이 두 개로 쪼개질 신호입니다.
- `feat:`, `fix:` 같은 접두사를 쓰지 않습니다. 이 저장소의 기존 커밋이 그렇습니다.
- 무엇을 했는지가 아니라 **무엇이 달라졌는지**를 씁니다.
  - 나쁨: `Update people_tracker.py`
  - 좋음: `Detect people in the 3D cloud by their vertical profile`

## 2. 본문

제목 다음 빈 줄, 그다음 본문. **왜**와 **측정값**을 씁니다. 코드가 무엇을 하는지는
코드가 말하고, 왜 그래야 하는지는 커밋만 말할 수 있습니다.

```
Detect people in the 3D cloud by their vertical profile

The flattened scan cannot tell a person from a pillar: both are a
narrow return at ankle height. The cloud can -- a person is a column
that reaches past 0.90 m and is 0.20 to 0.90 m wide, which is the test
the real robot already uses (results/real/amr_deploy/bin/detect_person.py).

Measured against the true poses in the U bay, B3, 585 sweeps:

    0-2 m        100% found (131 of 131)
    2-4 m        95.1% (547 of 575)
    position     0.06 m median error
    false alarms 2 -- no pillar or parked cart was reported as a person

Needs the sensor profile from the previous commit: with the asset's own
-15 to +15 degree LiDAR there are no returns above a person's waist.
```

규칙:

- **숫자가 있으면 넣습니다.** 이 프로젝트의 모든 판단은 측정에서 나왔고, 측정값
  없는 주장은 나중에 되돌릴 수 없습니다. 단위를 붙입니다.
- **직접 확인한 것만 씁니다.** 추정은 "measured"라고 쓰지 않습니다.
- 앞 커밋에 의존하면 그렇게 적습니다.
- 72자로 줄바꿈합니다.
- 한 가지를 고치면서 다른 것을 발견했다면, 그 발견은 **다음 커밋**입니다.

## 3. 한 커밋 = 한 주제

파일이 커밋 사이에 겹치지 않게 나눕니다. 겹칠 수밖에 없는 설정 파일은
**해당 주제의 hunk만** 스테이지합니다.

```bash
git diff configs/common.yaml > /tmp/c.patch      # 통째로 보고
# 주제별로 hunk를 골라 인덱스에만 적용
python3 - <<'PY'
import subprocess, sys
patch = open("/tmp/c.patch").read()
head, *hunks = patch.split("\n@@")
keep = [0, 3]                                     # 이 커밋에 들어갈 hunk 번호
out = head + "".join("\n@@" + hunks[i] for i in keep)
subprocess.run(["git", "apply", "--cached"], input=out, text=True, check=True)
PY
git commit            # 남은 hunk는 다음 커밋으로
```

주제를 가르는 기준은 "이것만 되돌릴 수 있는가"입니다. 센서 프로파일과 그
프로파일을 읽는 검출기는 따로 되돌릴 수 있으므로 따로 커밋합니다.

## 4. 커밋하지 않는 것

| 넣지 않음 | 이유 |
|---|---|
| rosbag (`*.db3`, `rosbag2_*/`), 점군 (`*.ply`), 패키지 (`*.deb`) | 크고, 저장소가 할 일이 아님. 그것을 만든 코드와 해석한 글이 커밋돼 있으면 됩니다 |
| 시행별 콘솔 로그 (`results/**/logs/`) | 기계별 스크롤백. 시행의 결과는 CSV 행과 JSON입니다 |
| 생성물 (`configs/nav2/*_map.*`, `nav2_params.yaml`) | 매 실행마다 설정에서 다시 만들어집니다. 커밋하면 낡은 사본이 설정과 어긋납니다 |
| 영상, GIF, 스크린샷 | `results/notion/`의 작은 스틸만 예외 |
| 기계별 경로 (`scripts/local_env.sh`) | |
| 자산 사본 (`... copy.usd`) | |

새로 생기는 종류는 `.gitignore`에 **이유를 주석으로** 적고 넣습니다.

## 5. 결과와 글

- 실험 기록: `results/notion/YYMMDD/YYMMDD_제목.md`.
  날짜는 항상 `YYMMDD` — 연·월·일을 두 자리씩, 구분자 없이. 2026년 10월 25일은
  `261025`입니다. 폴더 이름과 파일 이름 **양쪽**에 들어갑니다:

  ```
  results/notion/261025/261025_실기연결_LiDAR프로파일.md
  ```

  `date +%y%m%d`로 확인하고 쓰십시오. 세션이 날짜를 추측하면 폴더가 갈라집니다.
- 배치 결과: `results/batches/<YYMMDD_이름>/`, 그리고 `results/batches/README.md`에
  한 줄 추가 — 무엇을 비교했는지, n, **유효한지**.
- 어긋난 측정은 **지우지 않고 표시합니다.** 왜 못 쓰는지가 같이 남아야 나중에
  평균에 섞이지 않습니다.
- 실기 기록도 같은 규칙입니다: `results/real/YYMMDD/`.

## 6. 끝맺음

Claude가 만든 커밋은 attribution 줄로 끝냅니다 (세션의 모델 이름으로):

```
Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
```

## 7. 커밋 전 점검

```bash
python3 scripts/check_nodes.py        # 노드가 import 되는가
python3 scripts/check_evasion.py      # 양보 로직이 예외 없이 도는가
python3 scripts/check_perception.py   # 검출 → 상태 → Comfort Space가 이어지는가
git status --short                    # 의도한 파일만 들어가는가
git diff --cached --stat              # 커밋마다
```

세 검사는 각각 "조용히 아무것도 하지 않은" 사고에서 나왔습니다. 전부 1초면
끝나고, 배치 한 번은 한 시간입니다.

---

## 실기 작업에서 이 파일을 쓸 때

실기 코드는 `results/real/amr_deploy/`에 있고 같은 저장소입니다. 달라지는 것은
제목의 주어뿐입니다:

```
Separate people from the corridor by height on the real AMR

The 2D scan calls a desk and a person the same thing. Reading the
cloud instead and keeping 0.20-2.00 m, a person is 79 points of the
31,653 in a frame: 1.58 m tall, 1.21 m of vertical extent, 0.66 m
wide, against a desk at 0.99 m and flat.

Measured on the NSL corridor, 39 s stationary recording.
```

실기에서 측정한 값은 **시뮬 값과 섞어 쓰지 않습니다.** 같은 수를 비교할 때는
어느 쪽에서 나온 수인지 커밋 본문에 적습니다.
