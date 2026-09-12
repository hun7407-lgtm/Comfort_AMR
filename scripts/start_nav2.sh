#!/usr/bin/env bash

# ---------------------------------------------------------------
# Adapted from: https://github.com/dongtak2/AMR-Digital-Twin
#   original file : scripts/start_nav2.sh
#   commit        : c54269ee80032c29b0548eb77c3ee38d68b87e78 (2026-09-11)
#   modifications : ROS distro humble -> jazzy; ISAAC_ROS_WS example humble_ws -> jazzy_ws
# See docs/AMR_DIGITAL_TWIN_REFERENCE.md
# ---------------------------------------------------------------

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PARAMS_FILE="${REPO_DIR}/nav2/amr_nav2_params.yaml"

source /opt/ros/jazzy/setup.bash

# 사용자가 자신의 Isaac Sim ROS workspace 경로를 지정해야 함.
# 예:
# export ISAAC_ROS_WS="$HOME/IsaacSim-ros_workspaces/jazzy_ws"
if [ -z "${ISAAC_ROS_WS:-}" ]; then
  echo "[ERROR] ISAAC_ROS_WS가 설정되어 있지 않습니다."
  echo
  echo "먼저 자신의 Isaac Sim ROS workspace 경로를 지정하세요."
  echo '예: export ISAAC_ROS_WS="$HOME/IsaacSim-ros_workspaces/jazzy_ws"'
  echo
  echo "그 후 다시 실행:"
  echo "./scripts/start_nav2.sh"
  exit 1
fi

SETUP_FILE="${ISAAC_ROS_WS}/install/local_setup.bash"

if [ ! -f "${SETUP_FILE}" ]; then
  echo "[ERROR] 다음 파일을 찾을 수 없습니다:"
  echo "${SETUP_FILE}"
  echo
  echo "ISAAC_ROS_WS 경로가 맞는지, workspace가 빌드되어 있는지 확인하세요."
  exit 1
fi

source "${SETUP_FILE}"

if ! ros2 pkg prefix iw_hub_navigation >/dev/null 2>&1; then
  echo "[ERROR] iw_hub_navigation 패키지를 찾지 못했습니다."
  echo "Isaac Sim ROS workspace가 정상적으로 빌드/소스되었는지 확인하세요."
  exit 1
fi

ros2 launch iw_hub_navigation iw_hub_navigation.launch.py   params_file:="${PARAMS_FILE}"   use_sim_time:=true
