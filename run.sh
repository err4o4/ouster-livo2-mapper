#!/usr/bin/env bash
set -Eeuo pipefail

# Parse options
INTERACTIVE=true
RESTART_POLICY=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--detached)
      INTERACTIVE=false
      shift
      ;;
    -r|--restart)
      RESTART_POLICY="--restart unless-stopped"
      shift
      ;;
    -*)
      echo "Unknown option: $1"
      echo "Usage: $0 [options] <platform>"
      echo "Options:"
      echo "  -d, --detached    Run in detached mode (no interactive bash)"
      echo "  -r, --restart     Add restart policy (unless-stopped)"
      echo "Platforms:"
      echo "  amd64 | jetson | mac_os"
      exit 1
      ;;
    *)
      # This should be the platform
      PLATFORM_RAW="$1"
      shift
      break
      ;;
  esac
done

# Check if platform was provided
if [[ -z "${PLATFORM_RAW:-}" ]]; then
  echo "Usage: $0 [options] <platform>"
  echo "Options:"
  echo "  -d, --detached    Run in detached mode (no interactive bash)"
  echo "  -r, --restart     Add restart policy (unless-stopped)"
  echo "Platforms:"
  echo "  amd64 | jetson | mac_os"
  exit 1
fi

set -x

PORT_MAPPING=(-p 8765:8765 -p 3001:3001)
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

  -v ./ros_overlay/data:/root/data
  -v ./ros_overlay/webapp:/root/webapp
)

NAME="hku_mars_${PLATFORM_RAW}"

# Set runtime args based on options
if [[ "$INTERACTIVE" == true ]]; then
  RUNTIME_ARGS=(-it --rm)
else
  RUNTIME_ARGS=(-d)
fi

# Add restart policy if specified
if [[ -n "$RESTART_POLICY" ]]; then
  RUNTIME_ARGS+=($RESTART_POLICY)
fi

ENV_FLAGS=()
EXTRA_VOLUMES=()

case "$PLATFORM_RAW" in
    amd64)
        # ===== Linux x86_64 (desktop/workstation with NVIDIA) =====
        IMAGE="hku_mars_amd64"
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

    jetson)
        # ===== Linux aarch64 (Jetson/ARM64) =====
        IMAGE="hku_mars_arm64"
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
        IMAGE="hku_mars_arm64"
        RUNTIME_ARGS+=(--name "$NAME")
        ;;

    *)
        echo "Unsupported platform: $PLATFORM_RAW"
        echo "Use one of: amd64 | jetson | mac_os"
        exit 1
        ;;
esac

set -x

# Build docker command with conditional array expansion
cmd=(docker run "${RUNTIME_ARGS[@]}" "${PORT_MAPPING[@]}")
((${#ENV_FLAGS[@]}))      && cmd+=("${ENV_FLAGS[@]}")
((${#EXTRA_VOLUMES[@]}))  && cmd+=("${EXTRA_VOLUMES[@]}")
cmd+=("${COMMON_VOLUMES[@]}" "$IMAGE")

exec "${cmd[@]}"