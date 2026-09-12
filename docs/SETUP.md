# Setup

Ubuntu 24.04 + Isaac Sim 5.1 + ROS 2 Jazzy 기준입니다.

언제든 아래 명령으로 현재 상태를 확인할 수 있습니다.

```bash
./scripts/check_environment.sh
```

---

## 1. 확인된 환경

이 문서를 작성한 시점(2026-09-12)에 실제 시스템에서 확인한 값입니다.

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04.4 LTS (noble) |
| GPU | NVIDIA GeForce RTX 5080, 16 GB VRAM |
| Driver | 580.173.02 (CUDA 13.0) |
| Isaac Sim | 5.1.0-rc.19, `/home/hun/isaacsim` |
| Isaac Sim 내장 Python | 3.11.13 |
| System Python | 3.12.3 |

Isaac Sim 설치 경로는 코드에 고정하지 않습니다.
`check_environment.sh`가 `ISAACSIM_PATH` 환경변수, 관례적 경로, 홈 디렉터리 검색
순서로 찾습니다. 설치 위치가 다르다면 다음과 같이 지정하세요.

```bash
export ISAACSIM_PATH=/실제/설치/경로
```

---

## 2. 두 개의 Python 인터프리터

이 프로젝트에서 가장 중요한 구조적 제약입니다.

```
Isaac Sim 프로세스              ROS 2 프로세스
Python 3.11 (내장)             Python 3.12 (시스템)
omni.*, pxr 사용 가능           rclpy, nav2 사용 가능
rclpy 사용 불가                 omni.* 사용 불가
        │                              │
        └──── DDS (ROS 2 topics) ──────┘
```

두 인터프리터는 서로의 패키지를 import할 수 없습니다.
따라서 다음 규칙을 지킵니다.

1. Isaac Sim standalone script 안에서 `rclpy`를 import하지 않습니다.
   Isaac Sim은 `isaacsim.ros2.bridge` 확장의 OmniGraph 노드로만 ROS 2와 통신합니다.
2. ROS 2 노드 안에서 `omni`나 `pxr`을 import하지 않습니다.
3. 양쪽이 공유해야 하는 로직(Comfort Space 계산, Human State, Metrics)은
   `comfort_core/`에 두고, 표준 라이브러리와 numpy, PyYAML만 사용합니다.
   그래야 양쪽 인터프리터에서 모두 import됩니다.

Isaac Sim은 ROS 2 Jazzy 라이브러리를 자체적으로 동봉하고 있습니다.

```
/home/hun/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib   (344 files)
```

그래서 시스템에 ROS 2가 없어도 Isaac Sim 혼자서는 토픽을 낼 수 있습니다.
하지만 `ros2 topic pub` 같은 CLI 검증과 Nav2를 쓰려면 시스템 설치가 필요합니다.

---

## 3. ROS 2 Jazzy 설치

한 줄씩 실행하고, 각 줄이 성공했는지 확인한 뒤 다음으로 넘어가세요.

```bash
# universe 저장소 활성화.
# ROS 2가 의존하는 다수의 Ubuntu 패키지가 여기에 있습니다.
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

# ROS 패키지 서명 검증용 공개키.
# 이것이 없으면 apt가 저장소를 신뢰하지 않고 거부합니다.
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg

# apt 저장소 등록. noble = Ubuntu 24.04의 코드네임입니다.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# desktop = ROS 2 core + RViz2 + 데모 노드.
# RViz2는 costmap과 경로를 눈으로 확인할 때 필요합니다.
sudo apt install -y ros-jazzy-desktop \
                    python3-colcon-common-extensions \
                    python3-rosdep \
                    vulkan-tools

# rosdep init은 시스템 전역 설정이라 sudo가 필요합니다.
sudo rosdep init
# rosdep update는 사용자 홈에 캐시를 만듭니다. sudo 없이 실행하세요.
rosdep update
```

`colcon`은 ROS 2 workspace 빌드 도구이고,
`rosdep`은 패키지가 요구하는 시스템 의존성을 자동으로 찾아 설치하는 도구입니다.

### 설치 확인

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
echo $ROS_DISTRO       # jazzy 가 출력되어야 합니다
ros2 run demo_nodes_cpp talker
```

다른 터미널에서 다음을 실행해 메시지가 보이면 정상입니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

### 매번 source 하기 귀찮다면

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

다만 이 프로젝트는 `scripts/setup_env.sh`에서 필요한 환경을 모아 설정하므로,
`.bashrc`에 넣지 않고 스크립트를 통해 쓰는 쪽을 권장합니다.
전역 설정은 다른 ROS 프로젝트와 충돌할 수 있습니다.

---

## 4. Isaac Sim과 ROS 2 연결

### Isaac Sim을 실행할 때는 시스템 ROS 2를 source하지 않습니다

이 부분은 직관과 반대라서 STEP 3에서 실제로 확인했습니다.

Isaac Sim의 인터프리터는 Python 3.11이고 시스템 Jazzy는 3.12용으로 빌드되어 있습니다.
`/opt/ros/jazzy/setup.bash`를 source하면 3.12 site-packages가
Isaac Sim이 동봉한 cp311 rclpy보다 앞서게 되고, ROS 2 bridge가 import에 실패합니다.

실제로 관측한 오류입니다.

```
Attempting to load system rclpy
Could not import system rclpy: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so'
isn't present on the system.
Attempting to load internal rclpy for ROS Distro: jazzy
Could not import internal rclpy: ...
```

내부 라이브러리까지 실패한 이유는 시스템 경로가 이미 앞에 끼어 있었기 때문입니다.

Isaac Sim은 정확히 이 상황을 위해 Jazzy 라이브러리 한 벌을 자체적으로 동봉합니다.

```
$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/jazzy/lib      344 files
$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/jazzy/rclpy    cp311 빌드
```

`setup_ros_env.sh`가 `ROS_DISTRO`가 비어 있을 때 이 경로를 골라줍니다.
따라서 Isaac Sim을 띄우는 스크립트에서는 이것만 source합니다.

```bash
source "$ISAACSIM_PATH/setup_ros_env.sh"
export ROS_DOMAIN_ID=0
$ISAACSIM_PATH/isaac-sim.sh
```

이렇게 하면 로그에 다음이 나옵니다.

```
Attempting to load internal rclpy for ROS Distro: jazzy
rclpy loaded
```

`scripts/run_amr_test.sh`가 이 방식으로 되어 있습니다.

### ROS 2 노드를 실행할 때는 시스템 ROS 2를 source합니다

별도 터미널, 별도 프로세스입니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
```

양쪽 모두 Jazzy이고 FastDDS를 쓰므로 서로 통신됩니다.
`ROS_DOMAIN_ID`는 같은 네트워크의 다른 ROS 2 시스템과 섞이지 않게 하는 격리 번호입니다.
Isaac Sim과 ROS 2 노드가 **같은 값**을 써야 서로 보입니다.

---

## 5. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `ros2 topic list`에 Isaac Sim 토픽이 하나도 안 보임 | 아래 "ActionGraph가 tick하지 않는 경우" 참고 |
| 일부 토픽만 안 보임 | 양쪽 `ROS_DOMAIN_ID`가 같은가, `RMW_IMPLEMENTATION`이 같은가 |
| `Could not import rclpy` | Isaac Sim 실행 전에 `/opt/ros/jazzy/setup.bash`를 source했는지. 위 4장 참고 |
| 토픽은 보이는데 로봇이 안 움직임 | `ros2 topic info /cmd_vel --verbose`로 publisher와 subscriber의 **메시지 타입**이 같은지 확인 |
| Isaac Sim 첫 실행이 매우 느림 | 셰이더 캐시 생성 중입니다. RTX 50 시리즈는 수 분 걸릴 수 있습니다. |
| Isaac Sim 창이 뜨지 않음 | `vulkaninfo --summary`로 Vulkan 동작 확인 |

### ActionGraph가 tick하지 않는 경우

STEP 4에서 실제로 겪은 문제입니다. 원인이 직관적이지 않아 기록해 둡니다.

```python
simulation_context.step(render=False)   # 물리만 전진. 토픽이 하나도 안 나옴
simulation_context.step(render=True)    # app.update() 호출. 정상
```

`SimulationContext.step()`은 `render=True`일 때만 `app.update()`를 호출합니다.
그리고 `app.update()`가 OmniGraph의 `OnPlaybackTick`을 구동합니다.

따라서 `render=False`로 돌리면 물리는 정상적으로 진행되지만
ActionGraph는 한 번도 실행되지 않습니다.
ROS 2 publisher가 아예 생성되지 않으므로 `ros2 topic list`에
`/clock`, `/odom`, `/cmd_vel`이 전혀 나타나지 않습니다.
오류 메시지는 없습니다.

**headless 모드에서도 `render=True`여야 합니다.**
`--headless`는 창을 띄우지 않을 뿐이고, 렌더링 자체는 offscreen으로 수행됩니다.
`isaac_sim/scripts/run_amr_sim.py`가 이 방식으로 되어 있습니다.

### 좌표계: odom 과 world

`/odom`의 원점은 world 원점이 아니라 **로봇이 스폰된 지점**입니다.
ROS의 표준 동작이지만 이 프로젝트에서는 함정입니다.

복도, 스테이션, 사람, Comfort Space, start, goal은 전부 world 좌표로 정의됩니다.
`/odom`을 그대로 읽어 world 좌표의 goal과 비교하면 서로 다른 두 좌표계를 비교하게 됩니다.

STEP 5에서 실제로 겪은 증상입니다.
로봇이 world (-4, 0)에 스폰되고 `/odom`은 (0, 0)을 보고했습니다.
컨트롤러가 goal (4, 0)을 odom 좌표로 해석해 world (0, 0)까지만 가고
SUCCESS를 반환했습니다. 의도한 거리의 절반입니다.

`comfort_core/frames.py`가 이 변환을 담당합니다.
`/odom`을 읽는 곳에서 한 번 변환하고, 그 아래로는 전부 world 좌표만 씁니다.

```python
from comfort_core import odom_to_world
x, y, yaw = odom_to_world(cfg, (odom_x, odom_y, odom_yaw))
```

### 시뮬레이션 시간과 실제 시간

Isaac Sim은 real-time factor 1로 돌지 않습니다.
headless일 때는 실시간보다 빠르고, viewport를 켜면 느려집니다.

따라서 ROS 2 쪽 테스트에서 `time.time()`으로 시간을 재면 안 됩니다.
`/odom`이나 `/clock`의 header stamp를 쓰세요.
`scripts/test_cmd_vel.py`의 `sim_time()`이 그 예입니다.

실제로 wall clock으로 쟀을 때 0.8 m를 기대한 구간에서 1.275 m가 측정되었습니다.
로봇이 잘못 움직인 것이 아니라 측정 기준이 틀린 것이었습니다.
