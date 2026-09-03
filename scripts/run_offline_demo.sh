#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
HIK_SETUP="${HOME}/ros2_hik_ws/install/setup.bash"
WORKSPACE="${HOME}/game26_ws"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble not found at $ROS_SETUP"
  exit 1
fi

if [[ ! -d "$WORKSPACE/install" ]]; then
  echo "Workspace not built yet: $WORKSPACE"
  echo "Please run ./scripts/build_game26_ws.sh first."
  exit 1
fi

source "$ROS_SETUP"
if [[ -f "$HIK_SETUP" ]]; then
  source "$HIK_SETUP"
fi
source "$WORKSPACE/install/setup.bash"

# Auto-detect OpenVINO if installed
if [[ -z "${OpenVINO_DIR:-}" ]]; then
  CAND=(
    "${HOME}/.local/lib/python3.10/site-packages/openvino/cmake"
    "${HOME}/.local/lib/python3.11/site-packages/openvino/cmake"
    "/usr/local/lib/python3.10/site-packages/openvino/cmake"
    "/usr/local/lib/python3.11/site-packages/openvino/cmake"
    "/opt/intel/openvino/cmake"
  )
  for p in "${CAND[@]}"; do
    if [[ -f "${p}/OpenVINOConfig.cmake" ]]; then
      export OpenVINO_DIR="$p"
      export LD_LIBRARY_PATH="${p%/cmake}/libs:${LD_LIBRARY_PATH:-}"
      break
    fi
  done
fi

VIDEO_PATH="${PROJECT_ROOT}/sp_vision_25/assets/demo/demo.avi"
CONFIG_PATH="${PROJECT_ROOT}/sp_vision_25/configs/demo.yaml"
PROJECT_SP_VISION="${PROJECT_ROOT}/sp_vision_25"

if [[ ! -f "$VIDEO_PATH" ]]; then
  echo "Video file not found: $VIDEO_PATH"
  exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "Config file not found: $CONFIG_PATH"
  exit 1
fi

ros2 launch auto_aim_ros2 offline_demo.launch.py \
  project_root:="$PROJECT_SP_VISION" \
  video_path:="$VIDEO_PATH" \
  config_path:="$CONFIG_PATH" \
  image_topic:=/image_raw
