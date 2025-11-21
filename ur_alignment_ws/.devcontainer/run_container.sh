#!/usr/bin/env bash

set -euo pipefail

# -----------------------------------------------------------------------------
# DEFAULTS
# -----------------------------------------------------------------------------

## --- change here to your preferred default workspace path --- ##
WS_HOST="${WS_HOST:-$HOME/Robot-Alignment/ur_alignment_ws}"
## ------------------------------------------------------------ ##

IMAGE="${IMAGE:-ur-alignment-img}"
NAME="${NAME:-ur-alignment-container}"
WS_CONT="${WS_CONT:-/home/ur_user/ur_alignment_ws}"

REMOVE_ON_EXIT="${REMOVE_ON_EXIT:-1}"   # 1: --rm on exit
TTY="${TTY:-1}"                         # 1: -it, 0: -i
SHM_SIZE="${SHM_SIZE:-4g}"
ULIMIT_MEMLOCK="${ULIMIT_MEMLOCK:--1:-1}"
STOP_TIMEOUT="${STOP_TIMEOUT:-5}"

# -----------------------------------------------------------------------------
# ARGUMENT PARSING
# -----------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) IMAGE="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --ws) WS_HOST="$2"; shift 2 ;;
    --ws-in) WS_CONT="$2"; shift 2 ;;
    --rm) REMOVE_ON_EXIT=1; shift ;;
    --no-rm) REMOVE_ON_EXIT=0; shift ;;
    --tty) TTY=1; shift ;;
    --no-tty) TTY=0; shift ;;
    --shm) SHM_SIZE="$2"; shift 2 ;;
    --memlock) ULIMIT_MEMLOCK="$2"; shift 2 ;;
    --stop-timeout) STOP_TIMEOUT="$2"; shift 2 ;;
    -h|--help) print_help; exit 0 ;;
    *) echo "Unknown option: $1"; echo; print_help; exit 1 ;;
  esac
done

# -----------------------------------------------------------------------------
# PRE-CHECKS
# -----------------------------------------------------------------------------
command -v docker >/dev/null 2>&1 || { echo "Error: docker not found." >&2; exit 1; }
[[ -n "${WS_HOST}" && "${WS_HOST}" != "/" ]] || { echo "Error: WS_HOST invalid." >&2; exit 1; }
mkdir -p "${WS_HOST}"

# -----------------------------------------------------------------------------
# HELPERS: add device GIDs as supplementary groups
# -----------------------------------------------------------------------------
# This avoids 'group not found' issues and works even if the container doesn't have those group names.
add_gid_for_path() {
  local p="$1"
  [[ -e "$p" ]] || return 0
  local gid
  gid="$(stat -c %g "$p" 2>/dev/null || echo "")"
  [[ -n "${gid}" ]] || return 0
  # Avoid adding the primary GID again
  if [[ "${gid}" != "$(id -g)" ]]; then
    DOCKER_OPTS+=( --group-add "${gid}" )
  fi
}

add_gids_for_glob() {
  local glob="$1"
  local any=0
  for f in $glob; do
    [[ -e "$f" ]] || continue
    any=1
    add_gid_for_path "$f"
  done
  return $any
}

# -----------------------------------------------------------------------------
# BASE DOCKER OPTIONS
# -----------------------------------------------------------------------------
DOCKER_OPTS=()

# Name & user mapping
DOCKER_OPTS+=( --name "${NAME}" )
DOCKER_OPTS+=( --user "$(id -u)":"$(id -g)" )


# ---------------------------
# Workspace
# ---------------------------
DOCKER_OPTS+=( -v "${WS_HOST}:${WS_CONT}" )
DOCKER_OPTS+=( -e "WS_DIR=${WS_CONT}" )
DOCKER_OPTS+=( -e "CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH:-}" )

# ---------------------------
# Lifecycle / TTY
# ---------------------------
[[ "${REMOVE_ON_EXIT}" == "1" ]] && DOCKER_OPTS+=( --rm )
[[ "${TTY}" == "1" ]] && DOCKER_OPTS+=( -it ) || DOCKER_OPTS+=( -i )
DOCKER_OPTS+=( --stop-timeout "${STOP_TIMEOUT}" )

# ---------------------------
# Networking 
# ---------------------------
DOCKER_OPTS+=( --net=host )
DOCKER_OPTS+=( --ipc=host )
DOCKER_OPTS+=( --pid=host )
# ROS_DOMAIN_ID defaulted to 42 if not set 
DOCKER_OPTS+=( -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" )
# ROS 2 automatic discovery on local subnet
DOCKER_OPTS+=( -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET )

# ---------------------------
# Shared memory
# ---------------------------
DOCKER_OPTS+=( --ulimit "memlock=${ULIMIT_MEMLOCK}" )

# ---------------------------
# Time settings
# ---------------------------
[[ -e /etc/timezone  ]] && DOCKER_OPTS+=( -v /etc/timezone:/etc/timezone:ro )
[[ -e /etc/localtime ]] && DOCKER_OPTS+=( -v /etc/localtime:/etc/localtime:ro )

# ---------------------------
# X11 GUI
# ---------------------------
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  if command -v xhost >/dev/null 2>&1; then
    if xhost +SI:localuser:"${USER}" >/dev/null 2>&1; then
      trap 'xhost -SI:localuser:"${USER}" >/dev/null 2>&1 || true' EXIT
    fi
  fi
  DOCKER_OPTS+=( -e DISPLAY="${DISPLAY}" )
  DOCKER_OPTS+=( -v /tmp/.X11-unix:/tmp/.X11-unix:rw )
  DOCKER_OPTS+=( -e QT_X11_NO_MITSHM=1 )
fi

# ---------------------------
# GPU autodetect
# ---------------------------
if command -v nvidia-smi >/dev/null 2>&1 || [[ -e /dev/nvidiactl ]]; then
  DOCKER_OPTS+=( --gpus all )
fi

if [[ -e /dev/nvidiactl ]]; then
  add_gids_for_glob "/dev/nvidia*" || true
fi

# ---------------------------
# DRI / DRM (bind mount)
# ---------------------------
if [[ -d /dev/dri ]]; then
  DOCKER_OPTS+=( -v /dev/dri:/dev/dri )
  add_gids_for_glob "/dev/dri/*" || true
fi

# ---------------------------
# USB bus: bind mount for the whole tree 
# ---------------------------
if [[ -d /dev/bus/usb ]]; then
  DOCKER_OPTS+=( -v /dev/bus/usb:/dev/bus/usb )
  DOCKER_OPTS+=( --device-cgroup-rule='c 189:* rwm' )
  add_gids_for_glob "/dev/bus/usb/*/*" || true
fi

# ---------------------------
# Video devices: explicit map /dev/video*
# ---------------------------
for v in /dev/video*; do
  [[ -e "$v" ]] && DOCKER_OPTS+=( --device "$v":"$v" )
done
DOCKER_OPTS+=( --device-cgroup-rule='c 81:* rmw' )

# ---- Add supplementary GIDs of host devices (numeric) ----
# video devices
add_gids_for_glob "/dev/video*" || true
# DRM devices
add_gids_for_glob "/dev/dri/*" || true
# USB bus nodes
add_gids_for_glob "/dev/bus/usb/*/*" || true

# -----------------------------------------------------------------------------
# RUN: Andrea
# -----------------------------------------------------------------------------
#set -x
#exec docker run "${DOCKER_OPTS[@]}" "${IMAGE}"
# ------------------------------------------------------------
# RUN: Giffen
# ------------------------------------------------------------
set -e

# Make /sys writable (RealSense IMU uses IIO sysfs)
DOCKER_OPTS+=( -v /sys:/sys:rw )

# Security/capabilities (alternative DEBUG path is --privileged)
DOCKER_OPTS+=( --security-opt seccomp=unconfined --cap-add SYS_ADMIN --cap-add SYS_RAWIO )
DOCKER_OPTS+=( --privileged )   # <- use once only if you still hit permission issues

# Device cgroup rules (video + usb are added above; add hidraw too)
DOCKER_OPTS+=( --device-cgroup-rule='c 81:* rmw'  )   # video
DOCKER_OPTS+=( --device-cgroup-rule='c 189:* rmw' )   # usb
DOCKER_OPTS+=( --device-cgroup-rule='c 237:* rmw' )   # hidraw

# Attach hidraw / ttyACM if present (you already map /dev/video* above)
for d in /dev/hidraw* /dev/ttyACM*; do
  [[ -e "$d" ]] && DOCKER_OPTS+=( --device "$d":"$d" )
done

# Ensure we inherit host groups that grant device access
add_gid_if_exists() { local g="$1"; local id; id=$(getent group "$g" | cut -d: -f3 || true); [[ -n "$id" ]] && DOCKER_OPTS+=( --group-add "$id" ); }
add_gid_if_exists video
add_gid_if_exists plugdev

# Launch
set -x
exec docker run "${DOCKER_OPTS[@]}" "${IMAGE}"