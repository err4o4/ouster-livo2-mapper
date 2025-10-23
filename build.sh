#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <platform>"
  echo "  platform: amd64 | arm64"
  exit 1
fi

PLATFORM_RAW="$1"

case "$PLATFORM_RAW" in
  amd64)
    BASE_IMAGE="osrf/ros:noetic-desktop-full"
    DOCKER_PLATFORM="linux/amd64"
    ;;
  arm64)
    BASE_IMAGE="dustynv/ros:noetic-desktop-l4t-r35.4.1"
    DOCKER_PLATFORM="linux/arm64"
    ;;
  *)
    echo "Unsupported platform: $PLATFORM_RAW"
    echo "Use one of: amd64 | arm64"
    exit 2
    ;;
esac

set -x

docker buildx build \
    --network=host \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -t "hku_mars_${PLATFORM_RAW}" \
    .