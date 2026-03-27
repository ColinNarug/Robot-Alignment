#
# cd ~/Robot-Alignment
# chmod +x dev_offset.sh
#
# nano ~/Desktop/dev_offset.desktop
#
# [Desktop Entry]
# Type=Application
# Name=dev_offset
# Comment=Robot-Alignment: rebuild + launch dev_offset stack
# Terminal=false
# Exec=terminator --fullscreen -x bash -lc "$HOME/Robot-Alignment/ShellScripts/dev_offset.sh"
# Icon=utilities-terminal
# Categories=Development;
# StartupNotify=true
#
# chmod +x ~/Desktop/dev_offset.desktop
# gio set ~/Desktop/dev_offset.desktop metadata::trusted true
#

#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/Robot-Alignment/ra_ws"
SESSION="ra_dev_offset"
LAUNCH_FILE="start_offset.launch.py"

# Toggle these when you're unplugged:
#   USE_CAMERA=false USE_ROBOT=false ~/Robot-Alignment/ShellScripts/dev_offset.sh
USE_CAMERA="${USE_CAMERA:-true}"
USE_ROBOT="${USE_ROBOT:-true}"

# --- sanity checks ---
if [[ ! -d "$WS" ]]; then
  echo "ERROR: Workspace not found: $WS"
  exit 1
fi

# --- build (clean) ---
cd "$WS"

# Prevent colcon warnings caused by stale overlay paths after deleting install/
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

# ROS setup scripts sometimes reference env vars that may be unset.
# With `set -u`, that can crash the script. So:
# 1) define the variable (safe default)
# 2) temporarily disable nounset while sourcing
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

colcon build --packages-select uralignment_cpp

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "ERROR: $WS/install/setup.bash not found after build."
  exit 1
fi

# --- requirements ---
if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux not found. Install with: sudo apt install tmux"
  exit 1
fi

# Prefer Terminator fullscreen; fall back to gnome-terminal if needed
if command -v terminator >/dev/null 2>&1; then
  TERM_CMD=(terminator --fullscreen -x bash -lc)
elif command -v gnome-terminal >/dev/null 2>&1; then
  TERM_CMD=(gnome-terminal --title="RA: dev_offset" -- bash -lc)
else
  echo "ERROR: Neither terminator nor gnome-terminal found."
  exit 1
fi

# --- tmux two-pane layout: left=teleop, right=launch ---
TMUX_BOOTSTRAP=$(cat <<EOF
set -e
WS="$WS"
SETUP="\$WS/install/setup.bash"
SESSION="$SESSION"
LAUNCH_FILE="$LAUNCH_FILE"
USE_CAMERA="$USE_CAMERA"
USE_ROBOT="$USE_ROBOT"

tmux has-session -t "\$SESSION" 2>/dev/null && tmux kill-session -t "\$SESSION"

# Left pane: teleop (interactive)
tmux new-session -d -s "\$SESSION" \
  "bash -lc 'cd \"\$WS\"; source \"\$SETUP\"; ros2 run uralignment_cpp teleop_key; exec bash'"

# Right pane: everything else (ROS2 launch)
tmux split-window -h -t "\$SESSION" \
  "bash -lc 'cd \"\$WS\"; source \"\$SETUP\"; ros2 launch uralignment_cpp \"\$LAUNCH_FILE\" use_camera:=\$USE_CAMERA use_robot:=\$USE_ROBOT; exec bash'"

tmux select-layout -t "\$SESSION" even-horizontal
tmux set-option -t "\$SESSION" -g mouse on
tmux select-pane -t "\$SESSION".0
tmux attach -t "\$SESSION"
EOF
)

# Actually open the terminal and run tmux
"${TERM_CMD[@]}" "$TMUX_BOOTSTRAP"

