#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
   echo "Usage: $0 <platform> [--no-cache]"
  echo "  platform: amd64 | arm64"
  exit 1
fi

PLATFORM_RAW="$1"
FORCE_REBUILD=false

if [[ "${2:-}" == "--no-cache" ]]; then
  FORCE_REBUILD=true
fi


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

NO_CACHE_FLAG=""
if [[ "$FORCE_REBUILD" == true ]]; then
  NO_CACHE_FLAG="--no-cache"
fi

if [[ "$PLATFORM_RAW" == "amd64" ]]; then
  docker buildx build \
    --platform "${DOCKER_PLATFORM}" \
    --network=host \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    $NO_CACHE_FLAG \
    -t "hku_mars_${PLATFORM_RAW}" \
    .

elif [[ "$PLATFORM_RAW" == "arm64" ]]; then
  docker build \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    $NO_CACHE_FLAG \
    -t "hku_mars_${PLATFORM_RAW}" \
    .
fi