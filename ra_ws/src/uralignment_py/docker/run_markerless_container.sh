#!/usr/bin/env bash
set -euo pipefail

# Markerless object detector Docker launcher.
# This script is intentionally the only markerless-specific startup step.
# Start all uralignment_cpp nodes normally from the host.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RA_WS="$(cd "${PKG_DIR}/../.." && pwd)"

IMAGE="${MARKERLESS_IMAGE:-uralignment-py-markerless}"
NAME="${MARKERLESS_CONTAINER_NAME:-uralignment-py-markerless}"
CONTAINER_WS="/home/ur_user/ra_ws"

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"

if [[ ! -f "${SCRIPT_DIR}/Dockerfile" ]]; then
  echo "ERROR: Dockerfile not found at ${SCRIPT_DIR}/Dockerfile" >&2
  exit 1
fi

if [[ ! -d "${RA_WS}/config" ]]; then
  echo "ERROR: expected shared config directory not found: ${RA_WS}/config" >&2
  exit 1
fi

docker rm -f "${NAME}" >/dev/null 2>&1 || true

BUILD_ARGS=(-t "${IMAGE}" -f "${SCRIPT_DIR}/Dockerfile")
if [[ "${MARKERLESS_DOCKER_NO_CACHE:-0}" == "1" ]]; then
  BUILD_ARGS+=(--no-cache --pull)
fi
BUILD_ARGS+=("${SCRIPT_DIR}")

echo "Building Docker image: ${IMAGE}"
echo "Docker no-cache build: ${MARKERLESS_DOCKER_NO_CACHE:-0}"
docker build "${BUILD_ARGS[@]}"

DOCKER_ENV_ARGS=(
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
  -e "WS_DIR=${CONTAINER_WS}"
)

# Pass through host ROS/DDS environment only if it is already set.
# Normal C++ startup should not require extra exports.
for var in \
  RMW_IMPLEMENTATION \
  ROS_AUTOMATIC_DISCOVERY_RANGE \
  ROS_LOCALHOST_ONLY \
  FASTDDS_BUILTIN_TRANSPORTS \
  FASTRTPS_DEFAULT_PROFILES_FILE \
  CYCLONEDDS_URI
do
  if [[ -n "${!var:-}" ]]; then
    DOCKER_ENV_ARGS+=(-e "${var}=${!var}")
  fi
done

# Optional Docker-only diagnostic overrides.
if [[ -n "${MARKERLESS_RMW_IMPLEMENTATION:-}" ]]; then
  DOCKER_ENV_ARGS+=(-e "RMW_IMPLEMENTATION=${MARKERLESS_RMW_IMPLEMENTATION}")
fi

if [[ -n "${MARKERLESS_DISCOVERY_RANGE:-}" ]]; then
  DOCKER_ENV_ARGS+=(-e "ROS_AUTOMATIC_DISCOVERY_RANGE=${MARKERLESS_DISCOVERY_RANGE}")
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
echo "Host RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "Host ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<unset>}"
echo "Host ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo "Host FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-<unset>}"
echo "Mounting package: ${PKG_DIR} -> ${CONTAINER_WS}/src/uralignment_py"
echo "Mounting config:  ${RA_WS}/config -> ${CONTAINER_WS}/config:ro"

exec docker run --rm -it \
  --name "${NAME}" \
  --net=host \
  --ipc=host \
  --pid=host \
  --privileged \
  "${DOCKER_ENV_ARGS[@]}" \
  -v "${PKG_DIR}:${CONTAINER_WS}/src/uralignment_py:rw" \
  -v "${RA_WS}/config:${CONTAINER_WS}/config:ro" \
  "${IMAGE}" \
  bash -lc "
    set -eo pipefail

    source /opt/ros/jazzy/setup.bash

    echo '=== Container ROS/DDS environment ==='
    env | grep -E '^(ROS_|RMW_|FASTDDS_|FASTRTPS_|CYCLONEDDS_|WS_DIR=)' || true

    cd ${CONTAINER_WS}

    rm -rf build/uralignment_py install/uralignment_py log

    colcon build --packages-select uralignment_py --symlink-install

    source install/setup.bash

    echo '=== Container package executables ==='
    ros2 pkg executables uralignment_py

    echo '=== Starting markerless_pose_estimator ==='
    exec ros2 run uralignment_py markerless_pose_estimator
  "
