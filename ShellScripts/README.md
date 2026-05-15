# Robot-Alignment Startup Shell Script Setup

This README explains how to set up the Robot-Alignment startup shell scripts on a local Ubuntu machine.

Assumed repository location:

`~/Robot-Alignment`

Assumed shell script location:

`~/Robot-Alignment/ShellScripts`

Startup scripts covered:

- `dev_offset.sh`
- `start_pbvs.sh`
- `start_pbvs_xdisp.sh`
- `start_offset.sh`
- `start_offset_xdisp.sh`
- `run_markerless_alignment.sh`

## 1. Install required terminal tools

```bash
sudo apt update
sudo apt install -y tmux terminator python3
```

## 2. Make the scripts executable

```bash
cd ~/Robot-Alignment/ShellScripts

chmod +x dev_offset.sh
chmod +x start_pbvs.sh
chmod +x start_pbvs_xdisp.sh
chmod +x start_offset.sh
chmod +x start_offset_xdisp.sh
chmod +x run_markerless_alignment.sh
```

## 3. Create desktop launchers

Use this single block instead of manually creating each `.desktop` file one at a time.

```bash
python3 - <<'PY'
from pathlib import Path

shell_scripts = Path.home() / "Robot-Alignment" / "ShellScripts"
desktop = Path.home() / "Desktop"
desktop.mkdir(exist_ok=True)

launchers = {
    "dev_offset.desktop": (
        "dev_offset",
        "Robot-Alignment: rebuild + launch dev_offset stack",
        "dev_offset.sh",
    ),
    "start_pbvs.desktop": (
        "start_pbvs",
        "Robot-Alignment: rebuild + launch PBVS stack",
        "start_pbvs.sh",
    ),
    "start_pbvs_xdisp.desktop": (
        "start_pbvs_xdisp",
        "Robot-Alignment: rebuild + launch PBVS xdisp stack",
        "start_pbvs_xdisp.sh",
    ),
    "start_offset.desktop": (
        "start_offset",
        "Robot-Alignment: rebuild + launch offset stack",
        "start_offset.sh",
    ),
    "start_offset_xdisp.desktop": (
        "start_offset_xdisp",
        "Robot-Alignment: rebuild + launch offset xdisp stack",
        "start_offset_xdisp.sh",
    ),
    "run_markerless_alignment.desktop": (
        "run_markerless_alignment",
        "Robot-Alignment: docker build + launch markerless alignment stack",
        "run_markerless_alignment.sh",
    ),
}

for filename, (name, comment, script) in launchers.items():
    script_path = shell_scripts / script
    if not script_path.exists():
        raise FileNotFoundError(f"Missing shell script: {script_path}")

    path = desktop / filename
    path.write_text(f"""[Desktop Entry]
Type=Application
Name={name}
Comment={comment}
Terminal=false
Exec=terminator --fullscreen -x bash -lc \"$HOME/Robot-Alignment/ShellScripts/{script}\"
Icon=utilities-terminal
Categories=Development;
StartupNotify=true
""")
    path.chmod(0o755)

print("Desktop launchers created in ~/Desktop")
PY
```

## 4. Trust the desktop launchers

```bash
gio set ~/Desktop/dev_offset.desktop metadata::trusted true
gio set ~/Desktop/start_pbvs.desktop metadata::trusted true
gio set ~/Desktop/start_pbvs_xdisp.desktop metadata::trusted true
gio set ~/Desktop/start_offset.desktop metadata::trusted true
gio set ~/Desktop/start_offset_xdisp.desktop metadata::trusted true
gio set ~/Desktop/run_markerless_alignment.desktop metadata::trusted true
```

## 5. Verify the desktop launchers

Each launcher should have a clean `Name=` line and a clean `Exec=` line.

```bash
for f in \
  ~/Desktop/dev_offset.desktop \
  ~/Desktop/start_pbvs.desktop \
  ~/Desktop/start_pbvs_xdisp.desktop \
  ~/Desktop/start_offset.desktop \
  ~/Desktop/start_offset_xdisp.desktop \
  ~/Desktop/run_markerless_alignment.desktop
  do
    echo "===== $f ====="
    grep -E '^(Name|Exec)=' "$f"
    echo
  done
```

## 6. Optional Docker setup for markerless alignment

This is only needed for `run_markerless_alignment.sh`.

Do not install `docker.io` if Docker's official apt repository is being used. Use Docker Engine packages from Docker's official repository.

First check whether Docker is already installed and working.

```bash
docker --version
sudo systemctl status docker --no-pager
docker run hello-world
```

If Docker is not installed, install Docker Engine from Docker's official apt repository.

```bash
sudo apt update
sudo apt install -y ca-certificates curl

sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo \"${UBUNTU_CODENAME:-$VERSION_CODENAME}\") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

If Docker is installed but fails with `Unit docker.socket is masked`, unmask and restart Docker.

```bash
sudo systemctl unmask docker.socket
sudo systemctl unmask docker.service
sudo systemctl unmask containerd.service

sudo systemctl daemon-reload

sudo systemctl enable --now containerd.service
sudo systemctl enable --now docker.socket
sudo systemctl enable --now docker.service
```

Add the current user to the Docker group.

```bash
sudo usermod -aG docker "$USER"
newgrp docker
docker run hello-world
```

A full logout/login can be used instead of `newgrp docker`.

## 7. Run scripts manually from terminal

PBVS:

```bash
cd ~/Robot-Alignment/ShellScripts
./start_pbvs.sh
```

PBVS with X display:

```bash
cd ~/Robot-Alignment/ShellScripts
./start_pbvs_xdisp.sh
```

Offset:

```bash
cd ~/Robot-Alignment/ShellScripts
./start_offset.sh
```

Offset with X display:

```bash
cd ~/Robot-Alignment/ShellScripts
./start_offset_xdisp.sh
```

Development offset:

```bash
cd ~/Robot-Alignment/ShellScripts
./dev_offset.sh
```

Markerless alignment with Docker:

```bash
cd ~/Robot-Alignment/ShellScripts
./run_markerless_alignment.sh
```

## 8. Markerless alignment first-run behavior

On the first Docker run, the markerless script may spend several minutes building the Docker image, downloading image layers, creating the container, and building the ROS 2 package inside the container.

During this time, the teleop and robot panes may print retry messages while waiting for the container and build marker. This is expected while the right-side build/launch pane is still active.

The startup is ready when the panes have reached these commands:

```bash
ros2 run ur_alignment teleop_key
```

```bash
ros2 run ur_alignment ur_e_series
```

```bash
ros2 launch ur_alignment markerless_alignment.launch.py
```

## 9. Run without camera or robot

For local testing without the RealSense camera or UR robot:

```bash
cd ~/Robot-Alignment/ShellScripts
USE_CAMERA=false USE_ROBOT=false ./start_pbvs.sh
```

```bash
cd ~/Robot-Alignment/ShellScripts
USE_CAMERA=false USE_ROBOT=false ./start_offset.sh
```

```bash
cd ~/Robot-Alignment/ShellScripts
USE_CAMERA=false USE_ROBOT=false ./start_pbvs_xdisp.sh
```

```bash
cd ~/Robot-Alignment/ShellScripts
USE_CAMERA=false USE_ROBOT=false ./start_offset_xdisp.sh
```

```bash
cd ~/Robot-Alignment/ShellScripts
USE_CAMERA=false USE_ROBOT=false ./dev_offset.sh
```

## 10. Controls

Inside the teleop pane:

- `x` or `X` starts alignment.
- `Spacebar` stops alignment.
- `q`, `Q`, or `Esc` quits.