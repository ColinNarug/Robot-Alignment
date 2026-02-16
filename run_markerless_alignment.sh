#!/usr/bin/env bash
set -euo pipefail

# 
# Host-side runner.
# - Builds Docker image (no wiping)
# - Builds ur_alignment inside the running container
# - Verifies executable names EXACTLY (including .py if present)
# - Opens 3 vertical panes (tmux):
#     1) teleop_key
#     2) ur_e_series
#     3) launch markerless pipeline (camera + markerless + UI)
# 

IMAGE_NAME="${IMAGE_NAME:-ur-alignment-img}"
CONTAINER_NAME="${CONTAINER_NAME:-ur-alignment-container}"
DOCKER_USER="${DOCKER_USER:-ur_user}"

# Locate .devcontainer and workspace (host)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVCONTAINER_DIR="${SCRIPT_DIR}"
WS_HOST_DIR="$(cd "${DEVCONTAINER_DIR}/.." && pwd)"

# Container workspace path 
WS_CONT_DIR="/home/${DOCKER_USER}/ur_alignment_ws"

# Launch file name
LAUNCH_FILE="markerless_alignment.launch.py"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "ERROR: Required command not found: $1"
    exit 1
  }
}

container_running() {
  docker ps -q --filter "name=^/${CONTAINER_NAME}$" | grep -q .
}

container_exists() {
  docker ps -aq --filter "name=^/${CONTAINER_NAME}$" | grep -q .
}

exec_in_container() {
  # Usage: exec_in_container "command..."
  docker exec -it --user "${DOCKER_USER}" -w "${WS_CONT_DIR}" "${CONTAINER_NAME}" \
    bash -lc "set -e; $1"
}

echo "=== [1/4] Sanity checks (host) ==="
need_cmd docker
need_cmd tmux

echo "Workspace (host):       ${WS_HOST_DIR}"
echo ".devcontainer (host):   ${DEVCONTAINER_DIR}"
echo "Image tag:              ${IMAGE_NAME}"
echo "Container name:         ${CONTAINER_NAME}"
echo "Container workspace:    ${WS_CONT_DIR}"
echo

echo "=== [2/4] Build Docker image (no wipe, uses cache) ==="
docker build -t "${IMAGE_NAME}" "${DEVCONTAINER_DIR}"

echo
echo "=== [3/4] Ensure container is running ==="
if container_running; then
  echo "Container is already running: ${CONTAINER_NAME}"
else
  if container_exists; then
    echo "Container exists but is stopped -> starting: ${CONTAINER_NAME}"
    docker start "${CONTAINER_NAME}" >/dev/null
  else
    echo "ERROR: Container '${CONTAINER_NAME}' does not exist."
    echo
    echo "Start it using your required workflow in a terminal:"
    echo "  cd ${WS_HOST_DIR}/.devcontainer"
    echo "  chmod +x ./run_container.sh"
    echo "  ./run_container.sh"
    echo
    echo "Then re-run:"
    echo "  ${WS_HOST_DIR}/.devcontainer/run_markerless_alignment.sh"
    exit 1
  fi
fi

echo
echo "=== [3.5/4] Build ur_alignment inside container + verify executables EXACTLY ==="
exec_in_container "source /opt/ros/jazzy/setup.bash; colcon build --packages-select ur_alignment --symlink-install; source install/setup.bash; ros2 pkg executables ur_alignment"

# Verify the exact executable names exist (fail if mismatch)
REQ_EXES=(
  "realsense_data_publisher.py"
  "markerless_pose_estimator.py"
  "user_interface.py"
  "teleop_key"
  "ur_e_series"
)

echo
echo "Checking required executables..."
for exe in "${REQ_EXES[@]}"; do
  if ! exec_in_container "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 pkg executables ur_alignment | awk '{print \$2}' | grep -xq '${exe}'"; then
    echo
    echo "ERROR: Expected executable not found in 'ros2 pkg executables ur_alignment': ${exe}"
    echo "Here is the full list from inside the container:"
    exec_in_container "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 pkg executables ur_alignment"
    exit 1
  fi
done
echo "All required executables are present."

echo
echo "=== [4/4] Launch 3 vertical panes (tmux) ==="

SESSION="markerless_alignment"

# If session exists, kill it (only the tmux session; does NOT wipe docker)
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}"
fi

# Helper commands for each pane: docker exec + cd + source + run
CMD_TELEOP="docker exec -it --user ${DOCKER_USER} -w ${WS_CONT_DIR} ${CONTAINER_NAME} bash -lc 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 run ur_alignment teleop_key'"
CMD_ROBOT="docker exec -it --user ${DOCKER_USER} -w ${WS_CONT_DIR} ${CONTAINER_NAME} bash -lc 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 run ur_alignment ur_e_series'"
CMD_OTHERS="docker exec -it --user ${DOCKER_USER} -w ${WS_CONT_DIR} ${CONTAINER_NAME} bash -lc 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch ur_alignment ${LAUNCH_FILE}'"

# Create tmux session with 3 side-by-side panes (vertical splits on screen)
tmux new-session -d -s "${SESSION}" -n "run" "${CMD_TELEOP}"
tmux split-window -h -t "${SESSION}:run" "${CMD_ROBOT}"
tmux split-window -h -t "${SESSION}:run" "${CMD_OTHERS}"
tmux select-layout -t "${SESSION}:run" even-horizontal

echo
echo "Attached to tmux session: ${SESSION}"
echo "Pane 1: teleop_key"
echo "Pane 2: ur_e_series"
echo "Pane 3: markerless launch (camera + pose + UI)"
echo
echo "Detach: Ctrl+b then d"
tmux attach -t "${SESSION}"


#
# cd ~/Robot-Alignment
# chmod +x run_markerless_alignment.sh
#
# nano ~/Robot-Alignment/run_markerless_alignment.desktop
# [Desktop Entry]
# Type=Application
# Name=run_markerless_alignment
# Comment=Robot-Alignment: docker + build + launch markerless alignment stack
# Terminal=false
# Exec=terminator --fullscreen -x bash -lc "$HOME/Robot-Alignment/run_markerless_alignment.sh"
# Icon=utilities-terminal
# Categories=Development;
# StartupNotify=true
#
# chmod +x ~/Robot-Alignment/run_markerless_alignment.desktop
# gio set ~/Robot-Alignment/run_markerless_alignment.desktop metadata::trusted true
#
# nano ~/Desktop/run_markerless_alignment.desktop
# [Desktop Entry]
# Type=Application
# Name=run_markerless_alignment
# Comment=Robot-Alignment: docker + build + launch markerless alignment stack
# Terminal=false
# Exec=terminator --fullscreen -x bash -lc "$HOME/Robot-Alignment/run_markerless_alignment.sh"
# Icon=utilities-terminal
# Categories=Development;
# StartupNotify=true
#
# chmod +x ~/Desktop/run_markerless_alignment.desktop
# gio set ~/Desktop/run_markerless_alignment.desktop metadata::trusted true
#
