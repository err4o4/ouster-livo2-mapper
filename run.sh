#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <platform>"
  echo "  platform: amd64 | jeston | mac_os"
  exit 1
fi

PLATFORM_RAW="$1"

PORT_MAPPING=(-p 8765:8765)
COMMON_VOLUMES=(
  -v ./ros_overlay/usb_cam/launch:/opt/catkin_ws/src/usb_cam/launch

  -v ./ros_overlay/FAST-Calib/config:/opt/catkin_ws/src/FAST-Calib/config
  -v ./ros_overlay/FAST-Calib/output:/opt/catkin_ws/src/FAST-Calib/output
  -v ./ros_overlay/FAST-Calib/calib_data:/opt/catkin_ws/src/FAST-Calib/calib_data

  -v ./ros_overlay/LiDAR_IMU_Init/config:/opt/catkin_ws/src/LiDAR_IMU_Init/config
  -v ./ros_overlay/LiDAR_IMU_Init/launch:/opt/catkin_ws/src/LiDAR_IMU_Init/launch
  -v ./ros_overlay/LiDAR_IMU_Init/result:/opt/catkin_ws/src/LiDAR_IMU_Init/result

  -v ./ros_overlay/FAST_LIO/config:/opt/catkin_ws/src/FAST_LIO/config
  -v ./ros_overlay/FAST_LIO/launch:/opt/catkin_ws/src/FAST_LIO/launch

  -v ./ros_overlay/FAST-LIVO2/config:/opt/catkin_ws/src/FAST-LIVO2/config
  -v ./ros_overlay/FAST-LIVO2/launch:/opt/catkin_ws/src/FAST-LIVO2/launch
  
  -v ./scripts:/opt/scripts
)

NAME="hku_mars_${PLATFORM_RAW}"
IMAGE="hku_mars_${PLATFORM_RAW}"
RUNTIME_ARGS=(-it --rm)
ENV_FLAGS=()
EXTRA_VOLUMES=()

case "$PLATFORM_RAW" in
    amd64)
        # ===== Linux x86_64 (desktop/workstation with NVIDIA) =====
        export DISPLAY="${DISPLAY:-:0}"
        xhost +local:root

        RUNTIME_ARGS+=(--network host --gpus all --name "$NAME")
        ENV_FLAGS+=(
          -e "DISPLAY=${DISPLAY}"
          -e "XAUTHORITY=/root/.Xauthority"
          -e "QT_X11_NO_MITSHM=1"
          -e "NVIDIA_VISIBLE_DEVICES=all"
          -e "NVIDIA_DRIVER_CAPABILITIES=all"
        )
        EXTRA_VOLUMES+=(
          -v /tmp/.X11-unix:/tmp/.X11-unix
          -v "$HOME/.Xauthority":/root/.Xauthority:ro
        )
        ;;

    jeston)
        # ===== Linux aarch64 (Jetson/ARM64) =====
        export DISPLAY="${DISPLAY:-:1}"
        xhost +local:root

        RUNTIME_ARGS+=(--runtime nvidia --privileged --network host --name "$NAME")
        ENV_FLAGS+=(
          -e "DISPLAY=${DISPLAY}"
          -e "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
          -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}"
          -e "QT_X11_NO_MITSHM=1"
        )
        EXTRA_VOLUMES+=(
          -v /tmp/.X11-unix:/tmp/.X11-unix
          -v /dev:/dev
        )
        ;;

    mac_os)
        # ===== macOS (Docker Desktop / Colima) =====
        RUNTIME_ARGS+=(--name "$NAME")
        ;;

    *)
        echo "Unsupported platform: $PLATFORM_RAW"
        echo "Use one of: amd64 | jeston | mac_os"
        exit 1
        ;;
esac

set -x
docker run \
  "${RUNTIME_ARGS[@]}" \
  "${PORT_MAPPING[@]}" \
  "${ENV_FLAGS[@]}" \
  "${EXTRA_VOLUMES[@]}" \
  "${COMMON_VOLUMES[@]}" \
  "$IMAGE"
