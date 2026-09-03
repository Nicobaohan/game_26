#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
HIK_SETUP="${HOME}/ros2_hik_ws/install/setup.bash"
WORKSPACE="${HOME}/game26_ws"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble not found at $ROS_SETUP"
  exit 1
fi

if [[ ! -d "$WORKSPACE/src" ]]; then
  echo "Workspace not found: $WORKSPACE"
  echo "Please copy the repo into ${WORKSPACE}/src first."
  exit 1
fi

source "$ROS_SETUP"
if [[ -f "$HIK_SETUP" ]]; then
  source "$HIK_SETUP"
fi

# Try to auto-detect OpenVINO
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

if [[ -n "${OpenVINO_DIR:-}" ]]; then
  echo "Using OpenVINO_DIR=${OpenVINO_DIR}"
else
  echo "OpenVINO_DIR not set; build may fail if OpenVINO is required."
fi

cd "$WORKSPACE"
colcon build --packages-up-to auto_aim_ros2

source "$WORKSPACE/install/setup.bash"

echo "Build finished. Source the workspace with:"
echo "  source ${WORKSPACE}/install/setup.bash"
