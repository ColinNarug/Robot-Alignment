#!/usr/bin/env bash
set -Eeuo pipefail

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
    echo "[FATAL] Missing dependency: $1"
    exit 1
  }
}

echo "=== Markerless alignment (docker + tmux) ==="
echo "Host workspace:       $HOST_WS"
echo ".devcontainer dir:    $DEVCONTAINER_DIR"
echo "Image tag:            $IMAGE_TAG"
echo "Container name:       $CONTAINER_NAME"
echo "Container workspace:  $CONTAINER_WS"
echo

need docker
need tmux

if [[ ! -d "$DEVCONTAINER_DIR" ]]; then
  echo "[FATAL] Not found: $DEVCONTAINER_DIR"
  exit 1
fi

# Don't nest sessions unintentionally
if [[ -n "${TMUX:-}" ]]; then
  echo "[FATAL] You're already inside tmux. Detach first, then re-run this script."
  exit 1
fi

# Start fresh session each time 
if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
  tmux kill-session -t "$TMUX_SESSION"
fi

# Create tmux session with 3 vertical panes
tmux new-session -d -s "$TMUX_SESSION" -n "markerless"
tmux split-window -h -t "$TMUX_SESSION:0.0"
tmux split-window -h -t "$TMUX_SESSION:0.1"
tmux select-layout -t "$TMUX_SESSION:0" even-horizontal

PANE_TELEOP="$TMUX_SESSION:0.0"
PANE_UR="$TMUX_SESSION:0.1"
PANE_STACK="$TMUX_SESSION:0.2"

# Right pane: build image + run container
tmux send-keys -t "$PANE_STACK" \
"cd \"$DEVCONTAINER_DIR\" && \
echo '=== [1/4] docker build (no wipe, uses cache) ===' && \
docker build -t \"$IMAGE_TAG\" . --progress=plain && \
echo '=== [2/4] run_container.sh (enters container) ===' && \
chmod +x ./run_container.sh 2>/dev/null || true; \
./run_container.sh" C-m

(
  # Wait until container exists + is running
  while ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; do
    sleep 0.5
  done
  sleep 1.0

  # Single chained command (prevents interleaving while colcon is running)
  IN_CONTAINER_CMD="cd $CONTAINER_WS && \
colcon build --packages-select $PKG && \
source $CONTAINER_WS/install/setup.bash && \
ros2 launch $PKG $LAUNCH_FILE"

  tmux send-keys -t "$PANE_STACK" "$IN_CONTAINER_CMD" C-m
) >/tmp/markerless_alignment_autotype.log 2>&1 & disown || true

# Left pane: teleop_key 
tmux send-keys -t "$PANE_TELEOP" \
"echo 'Waiting for container: $CONTAINER_NAME' && \
until docker ps --format '{{.Names}}' | grep -qx '$CONTAINER_NAME'; do sleep 1; done; \
docker exec -it '$CONTAINER_NAME' bash -lc 'cd $CONTAINER_WS && source $CONTAINER_WS/install/setup.bash && ros2 run $PKG teleop_key'" C-m

# Middle pane: ur_e_series
tmux send-keys -t "$PANE_UR" \
"echo 'Waiting for container: $CONTAINER_NAME' && \
until docker ps --format '{{.Names}}' | grep -qx '$CONTAINER_NAME'; do sleep 1; done; \
docker exec -it '$CONTAINER_NAME' bash -lc 'cd $CONTAINER_WS && source $CONTAINER_WS/install/setup.bash && ros2 run $PKG ur_e_series'" C-m

# Attach
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
