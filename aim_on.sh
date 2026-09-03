#!/usr/bin/env bash
# Orin local launcher for auto aim and the local OpenCV preview.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIM_SCRIPT="${SCRIPT_DIR}/scripts/aim"
BUILD_ROOT="${AUTOAIM_BUILD_ROOT:-${HOME}/game_26_build/ros2}"
INSTALL_SETUP="${BUILD_ROOT}/install/setup.bash"

if [[ ! -f "${AIM_SCRIPT}" ]]; then
  echo "找不到瞄准脚本: ${AIM_SCRIPT}" >&2
  echo "请把本脚本放到 game_26_current/ 目录下" >&2
  exit 2
fi

# No argument means fire mode. This wrapper supplies scripts/aim's AIM/FIRE
# confirmations so that the local launcher starts without a second prompt.
ACTION="${1:-fire}"
case "${ACTION}" in
  on|safe|fire|status|build|calibrate|extrinsic-capture) ;;
  -h|--help|help)
    cat <<'EOF'
用法: ./aim_on.sh [fire|on|safe|status|build|calibrate|extrinsic-capture]
  fire      开启云台跟随和连发（默认，无二次确认）
  on        开启云台跟随，不开火
  safe      预览模式，云台不动
  status    查看相机、串口和自瞄状态
  build     colcon 编译
  calibrate 标定相机（需要桌面 GUI）

可选环境变量:
  AIM_NO_DISPLAY=1       不打开 Orin 本机 OpenCV 预览窗口
  AIM_DISPLAY=:0         指定 Orin 桌面 DISPLAY
EOF
    exit 0
    ;;
  *)
    echo "未知动作: ${ACTION}" >&2
    exit 1
    ;;
esac

RUNTIME_ACTION=false
if [[ "${ACTION}" == "on" || "${ACTION}" == "safe" || "${ACTION}" == "fire" ]]; then
  RUNTIME_ACTION=true
fi

build_if_stale() {
  local auto_aim_binary="${BUILD_ROOT}/install/auto_aim_ros2/lib/auto_aim_ros2/auto_aim_node"
  local hik_binary="${BUILD_ROOT}/install/hik_camera/lib/hik_camera/hik_camera_node"
  local hik_build_stamp="${BUILD_ROOT}/build/hik_camera/CMakeFiles/hik_camera.dir/src/hik_camera_node.cpp.o"
  local needs_build=false

  if [[ ! -x "${auto_aim_binary}" || ! -x "${hik_binary}" || ! -f "${hik_build_stamp}" ]]; then
    needs_build=true
  else
    # --symlink-install leaves these entries as symlinks whose own timestamps
    # do not change after a rebuild. Compare sources with the resolved files.
    auto_aim_binary="$(readlink -f "${auto_aim_binary}")"
    if find \
        "${SCRIPT_DIR}/src/auto_aim_ros2" \
        "${SCRIPT_DIR}/sp_vision_25" \
        -type f \( -name '*.cpp' -o -name '*.hpp' -o -name 'CMakeLists.txt' \) \
        -newer "${auto_aim_binary}" -print -quit | grep -q .; then
      needs_build=true
    elif find \
        "${SCRIPT_DIR}/src/ros2-hik-camera" \
        -type f \( -name '*.cpp' -o -name '*.hpp' -o -name 'CMakeLists.txt' \) \
        -newer "${hik_build_stamp}" -print -quit | grep -q .; then
      needs_build=true
    fi
  fi

  if ${needs_build}; then
    echo "检测到源码比 Orin 中的运行程序新，先重新编译。"
    bash "${AIM_SCRIPT}" build
  fi
}

VIEWER_PID=""

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${VIEWER_PID}" ]] && kill -0 "${VIEWER_PID}" 2>/dev/null; then
    kill "${VIEWER_PID}" 2>/dev/null || true
    wait "${VIEWER_PID}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

start_local_preview() {
  [[ -z "${AIM_NO_DISPLAY:-}" ]] || return 0

  local viewer_script="${SCRIPT_DIR}/scripts/debug_image_viewer.py"
  if [[ ! -f "${viewer_script}" ]]; then
    echo "警告: 找不到本机预览程序 ${viewer_script}" >&2
    return 0
  fi
  if pgrep -u "$(id -u)" -f "${viewer_script}" >/dev/null 2>&1; then
    echo "Orin 本机图像预览已经在运行。"
    return 0
  fi

  local aim_display="${AIM_DISPLAY:-:0}"
  (
    sleep 3
    export DISPLAY="${aim_display}"
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    export ROS_LOCALHOST_ONLY=1
    set +u
    source /opt/ros/humble/setup.bash
    source "${INSTALL_SETUP}"
    set -u
    exec python3 "${viewer_script}"
  ) > /tmp/autoaim_debug_viewer.log 2>&1 &
  VIEWER_PID=$!
  echo "Orin 本机预览正在启动（PID ${VIEWER_PID}）。"
}

if [[ "${ACTION}" == "status" ]]; then
  bash "${AIM_SCRIPT}" status
  exit 0
fi

if ${RUNTIME_ACTION}; then
  build_if_stale
  start_local_preview
fi

# The wrapper is explicitly the local armed launcher, so it supplies the
# lower-level script's confirmations. Stopping with Ctrl+C closes the viewer.
CONFIRM_INPUT=""
case "${ACTION}" in
  on)   CONFIRM_INPUT='AIM' ;;
  fire) CONFIRM_INPUT=$'AIM\nFIRE' ;;
  *)    ;;
esac

if [[ -n "${CONFIRM_INPUT}" ]]; then
  printf '%s\n' "${CONFIRM_INPUT}" | bash "${AIM_SCRIPT}" "${ACTION}"
else
  bash "${AIM_SCRIPT}" "${ACTION}"
fi
