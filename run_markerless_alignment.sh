#!/usr/bin/env bash
set -Eeuo pipefail

STEP_DELAY=2

HOST_ROOT="$HOME/Robot-Alignment"
HOST_WS="$HOST_ROOT/ur_alignment_ws"
DEVCONTAINER_DIR="$HOST_WS/.devcontainer"

IMAGE_TAG="ur-alignment-img"
CONTAINER_NAME="ur-alignment-container"

CONTAINER_WS="/home/ur_user/ur_alignment_ws"
PKG="ur_alignment"
LAUNCH_FILE="markerless_alignment.launch.py"

TMUX_SESSION="markerless_alignment"

need() {
  command -v "$1" >/dev/null 2>&1 || {
    echo
    echo "[FATAL] Missing dependency: $1"
    echo "Install it with:"
    echo "  sudo apt update"
    echo "  sudo apt install -y tmux terminator"
    echo
    echo "Keeping this window open for 30s so you can read the error..."
    sleep 30
    exit 1
  }
}

cleanup() {
  set +e
  echo
  echo "=== Cleanup: stopping container + tmux ==="
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
  tmux kill-session -t "$TMUX_SESSION" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

trap 'rc=$?; echo; echo "[FATAL] Script failed (exit $rc). Window will stay open for 30s..."; sleep 30; exit $rc' ERR

need docker
need tmux

if [[ ! -d "$DEVCONTAINER_DIR" ]]; then
  echo
  echo "[FATAL] Not found: $DEVCONTAINER_DIR"
  echo "Expected: ~/Robot-Alignment/ur_alignment_ws/.devcontainer"
  echo "Keeping this window open for 30s..."
  sleep 30
  exit 1
fi

if [[ ! -f "$DEVCONTAINER_DIR/Dockerfile" ]]; then
  echo
  echo "[FATAL] Dockerfile not found in: $DEVCONTAINER_DIR"
  echo "Keeping this window open for 30s..."
  sleep 30
  exit 1
fi

# Avoid nested tmux
if [[ -n "${TMUX:-}" ]]; then
  echo
  echo "[FATAL] You are already inside tmux. Detach first, then rerun."
  sleep 10
  exit 1
fi

echo "=== Markerless alignment (docker + tmux) ==="
echo "Host workspace:       $HOST_WS"
echo ".devcontainer dir:    $DEVCONTAINER_DIR"
echo "Image tag:            $IMAGE_TAG"
echo "Container name:       $CONTAINER_NAME"
echo "Container workspace:  $CONTAINER_WS"
echo "Step delay:           ${STEP_DELAY}s"
echo

# Start fresh tmux session each time
if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
  tmux kill-session -t "$TMUX_SESSION"
fi

# Create tmux session with 3 vertical panes (side-by-side columns)
tmux new-session -d -s "$TMUX_SESSION" -n "markerless"
tmux split-window -h -t "$TMUX_SESSION:0.0"
tmux split-window -h -t "$TMUX_SESSION:0.1"
tmux select-layout -t "$TMUX_SESSION:0" even-horizontal

tmux set -t "$TMUX_SESSION" -g mouse on
tmux set -t "$TMUX_SESSION" -g focus-events on

PANE_TELEOP="$TMUX_SESSION:0.0"
PANE_UR="$TMUX_SESSION:0.1"
PANE_STACK="$TMUX_SESSION:0.2"

tmux send-keys -t "$PANE_STACK" "bash -lc '
set -Eeuo pipefail

echo \"=== [1/7] docker build (uses cache; no workspace wipe) ===\"
cd \"$DEVCONTAINER_DIR\"
sleep $STEP_DELAY
docker build -t \"$IMAGE_TAG\" . --progress=plain
sleep $STEP_DELAY

echo \"=== [2/7] remove stale container (prevents name conflict) ===\"
docker rm -f \"$CONTAINER_NAME\" >/dev/null 2>&1 || true
sleep $STEP_DELAY

echo \"=== [3/7] start container (detached, stays alive) ===\"
# X11: allow local docker clients (non-fatal if xhost missing)
if command -v xhost >/dev/null 2>&1 && [[ -n \"\${DISPLAY:-}\" ]]; then
  xhost +local: >/dev/null 2>&1 || true
fi

docker run -d --rm --name \"$CONTAINER_NAME\" \
  --privileged \
  --network host \
  -e DISPLAY=\"\${DISPLAY:-:0}\" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/bus/usb:/dev/bus/usb \
  -v \"$HOST_WS\":\"$CONTAINER_WS\":rw \
  -w \"$CONTAINER_WS\" \
  \"$IMAGE_TAG\" \
  bash -lc \"tail -f /dev/null\" >/dev/null

# wait until running
until docker ps --format \"{{.Names}}\" | grep -qx \"$CONTAINER_NAME\"; do
  sleep 1
done
sleep $STEP_DELAY

echo \"=== [4/7] colcon build ur_alignment (no rm -rf build/install/log) ===\"
# marker file so other panes wait until build is done
docker exec \"$CONTAINER_NAME\" bash -lc \"rm -f /tmp/markerless_ready\"
sleep $STEP_DELAY

docker exec -it \"$CONTAINER_NAME\" bash -lc \"
  set -e
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  cd $CONTAINER_WS
  colcon build --packages-select $PKG
  source install/setup.bash
  touch /tmp/markerless_ready
\"
sleep $STEP_DELAY

echo \"=== [5/7] launch markerless stack (camera -> markerless -> UI) ===\"
docker exec -it \"$CONTAINER_NAME\" bash -lc \"
  set -e
  source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
  cd $CONTAINER_WS
  source install/setup.bash
  ros2 launch $PKG $LAUNCH_FILE
\"
' " C-m

tmux send-keys -t "$PANE_TELEOP" "bash -lc '
set -Eeuo pipefail
echo \"Waiting for container: $CONTAINER_NAME\"
until docker ps --format \"{{.Names}}\" | grep -qx \"$CONTAINER_NAME\"; do sleep 1; done
echo \"Waiting for build marker...\"
until docker exec \"$CONTAINER_NAME\" bash -lc \"test -f /tmp/markerless_ready\" >/dev/null 2>&1; do sleep 1; done
sleep $STEP_DELAY

# retry loop (if container restarts)
while true; do
  docker exec -it \"$CONTAINER_NAME\" bash -lc \"
    source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
    cd $CONTAINER_WS
    source install/setup.bash
    ros2 run $PKG teleop_key
  \" && break
  echo \"[WARN] teleop_key exited; retrying in ${STEP_DELAY}s...\"
  sleep $STEP_DELAY
done
' " C-m

tmux send-keys -t "$PANE_UR" "bash -lc '
set -Eeuo pipefail
echo \"Waiting for container: $CONTAINER_NAME\"
until docker ps --format \"{{.Names}}\" | grep -qx \"$CONTAINER_NAME\"; do sleep 1; done
echo \"Waiting for build marker...\"
until docker exec \"$CONTAINER_NAME\" bash -lc \"test -f /tmp/markerless_ready\" >/dev/null 2>&1; do sleep 1; done
sleep $STEP_DELAY

while true; do
  docker exec -it \"$CONTAINER_NAME\" bash -lc \"
    source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash
    cd $CONTAINER_WS
    source install/setup.bash
    ros2 run $PKG ur_e_series
  \" && break
  echo \"[WARN] ur_e_series exited; retrying in ${STEP_DELAY}s...\"
  sleep $STEP_DELAY
done
' " C-m

# Attach to tmux
tmux attach -t "$TMUX_SESSION"

#
# Make executable + desktop launcher
#
# (If another computer "just closes", install the terminal tools first:)
# sudo apt update
# sudo apt install -y tmux terminator
#
# cd ~/Robot-Alignment
# chmod +x run_markerless_alignment.sh
#
# Launcher in ~/Robot-Alignment
# nano ~/Robot-Alignment/run_markerless_alignment.desktop
# [Desktop Entry]
# Type=Application
# Name=run_markerless_alignment
# Comment=Robot-Alignment: docker build + launch markerless alignment stack
# Terminal=false
# Exec=terminator --fullscreen -x bash -lc "$HOME/Robot-Alignment/run_markerless_alignment.sh"
# Icon=utilities-terminal
# Categories=Development;
# StartupNotify=true
#
# chmod +x ~/Robot-Alignment/run_markerless_alignment.desktop
# gio set ~/Robot-Alignment/run_markerless_alignment.desktop metadata::trusted true
#
<<<<<<< HEAD
# Launcher on ~/Desktop
# nano ~/Desktop/run_markerless_alignment.desktop
# [Desktop Entry]
# Type=Application
# Name=run_markerless_alignment
# Comment=Robot-Alignment: docker build + launch markerless alignment stack
# Terminal=false
# Exec=terminator --fullscreen -x bash -lc "$HOME/Robot-Alignment/run_markerless_alignment.sh"
# Icon=utilities-terminal
# Categories=Development;
# StartupNotify=true
#
# chmod +x ~/Desktop/run_markerless_alignment.desktop
# gio set ~/Desktop/run_markerless_alignment.desktop metadata::trusted true
#
=======
>>>>>>> 4bba20e (updates)
