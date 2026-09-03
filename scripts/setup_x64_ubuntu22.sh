#!/usr/bin/env bash
set -euo pipefail

# Setup environment for x64 + Ubuntu 22.04 ROS2 deployment
# Usage:
#   ./scripts/setup_x64_ubuntu22.sh

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
HIK_SETUP="${HOME}/ros2_hik_ws/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS2 Humble not found at $ROS_SETUP"
  echo "Please install ROS2 Humble first, then rerun this script."
  exit 1
fi

sudo apt update

sudo apt install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  pkg-config \
  python3 \
  python3-pip \
  python3-dev \
  python3-colcon-common-extensions \
  libopencv-dev \
  libeigen3-dev \
  libyaml-cpp-dev \
  nlohmann-json3-dev \
  libceres-dev \
  libboost-all-dev \
  libssl-dev

if [[ -d "${HOME}/ros2_hik_ws" ]]; then
  echo "Found ros2_hik_ws at ${HOME}/ros2_hik_ws"
fi

mkdir -p "${HOME}/game26_ws/src"

if [[ -d "${HOME}/game26_ws/src/auto_aim_ros2" ]]; then
  echo "Workspace already exists at ${HOME}/game26_ws"
else
  echo "Copy or clone this repo into: ${HOME}/game26_ws/src"
  echo "For example:"
  echo "  mkdir -p ${HOME}/game26_ws/src"
  echo "  cp -a ${PROJECT_ROOT} ${HOME}/game26_ws/src/"
fi

# Try detect OpenVINO config automatically
OPENVINO_CANDIDATES=(
  "${HOME}/.local/lib/python3.10/site-packages/openvino/cmake"
  "${HOME}/.local/lib/python3.11/site-packages/openvino/cmake"
  "/usr/local/lib/python3.10/site-packages/openvino/cmake"
  "/usr/local/lib/python3.11/site-packages/openvino/cmake"
  "/opt/intel/openvino/cmake"
)

OPENVINO_FOUND=""
for p in "${OPENVINO_CANDIDATES[@]}"; do
  if [[ -f "${p}/OpenVINOConfig.cmake" ]]; then
    OPENVINO_FOUND="${p}"
    break
  fi
done

if [[ -n "$OPENVINO_FOUND" ]]; then
  printf '\nOpenVINO found at: %s\n' "$OPENVINO_FOUND"
  echo "export OpenVINO_DIR=${OPENVINO_FOUND}" >> "${HOME}/.bashrc"
  echo "export LD_LIBRARY_PATH=${OPENVINO_FOUND%/cmake}/libs:${LD_LIBRARY_PATH:-}" >> "${HOME}/.bashrc"
  echo "OpenVINO env vars added to ~/.bashrc"
else
  echo "OpenVINO not found automatically."
  echo "You may need to install OpenVINO or set OpenVINO_DIR manually."
fi

cat <<EOF

Setup finished.

Next steps:
  1. Put this repo under: ${HOME}/game26_ws/src/
  2. Build workspace:
       source /opt/ros/humble/setup.bash
       source ${HOME}/ros2_hik_ws/install/setup.bash 2>/dev/null || true
       cd ${HOME}/game26_ws
       colcon build --packages-up-to auto_aim_ros2
  3. Run demo:
       source ${HOME}/game26_ws/install/setup.bash
       ./scripts/run_offline_demo.sh
EOF
