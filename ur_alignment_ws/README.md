# Running Markerless Alignmnet

## Docker Installation

### 0) Installations

Before starting, install the following:

- **Docker Engine**
👉 [Install on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)


- **NVIDIA Container Toolkit** 
👉 [NVIDIA Official Install Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

  
- **VS Code** with extension: *Dev Containers* (recommended)


Check installation:
```bash
docker --version
docker run --rm nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

### 1. Ethernet Connection to the Robot

First, configure the host to communicate with the robot at `192.168.1.101` via Ethernet.  

**Steps**

0. **Make sure that the robot is in "remote control"**

1. **Test the host connection first**
   ```bash
   ping -c 3 192.168.1.101
   ```
   - ✅ If you see replies (e.g., `64 bytes from 192.168.1.101...`), the connection is working: no further action is needed.
   - ❌ If you see `100% packet loss`, continue with the steps below.

2. **Clear old IP configuration**
   ```bash
   sudo ip addr flush dev enp67s0
   ```
   Removes any previous IP addresses assigned to the interface.

3. **Assign a static IP to your PC**
   ```bash
   sudo ip addr add 192.168.1.100/24 dev enp67s0
   ```
   Sets the PC IP to `192.168.1.100` in the same subnet as the robot.

4. **Bring the interface up**
   ```bash
   sudo ip link set enp67s0 up
   ```
   Activates the network interface so it can send/receive packets.

5. **Test the connection again**
   ```bash
   ping -c 3 192.168.1.101
   ```
   ✅ You should see replies confirming that the connection is established. (e.g., `64 bytes from 192.168.1.101...`)
---

After this setup, the PC and the robot are on the same subnet and can communicate over Ethernet.

### 2. Use Docker on terminal

Open a terminal and run:

1. **Go to the `.devcontainer` folder**
```bash
cd ~/Robot-Alignment/ur_alignment_ws/.devcontainer
```

2. **Build the container** (only the first time or when Dockerfile changes)
```bash
docker build -t ur-alignment-img .
```

3. **Run the container with the custom script**
```bash
./run_container.sh # First create an Executable if needed: chmod +x ./run_container.sh
```
**You are now in the container!**

4. **Go to the alignment workspace**
```bash
cd /home/ur_user/ur_alignment_ws
```

5. **Build the package**
```bash
colcon build --packages-select ur_alignment
```

6. **Source the build**
```bash
source ~/ur_alignment_ws/install/setup.bash
```

7. **List available executables for reference**
```bash
ros2 pkg executables ur_alignment
```

8. **Start 1st Node (Camera Node)**
```bash
ros2 run ur_alignment realsense_data_publisher.py
```

9. **Start every subsequent node**

*Step 1: Start another terminal window in docker*
```bash
docker exec -it ur-alignment-container bash
```
*Step 2: Go to the alignment workspace*
```bash
cd /home/ur_user/ur_alignment_ws
```
*Step 3: Source the Build*
```bash
source ~/ur_alignment_ws/install/setup.bash
```
*Step 4: Source the Build*
```bash
ros2 run ur_alignment [node executable]
```

> Inside the container, your workspace is typically mounted at `/home/ur_user/ur_alignment_ws`.


### 3. Use Docker on VScode
To use docker inside VScode you need: **VS Code Dev Containers**.

Once you navigated to the base folder, a pop-up will ask you to rebuild the container. 
Since VScode does not provide any logs while building, my suggestion is to first build the container in a terminal uising:
```bash
docker build -t ur-alignment-img .
```

Once built the container, you can open it VScode

- Press **CTRL + SHIFT + P** to open the **Command Palette**.

use the following commands:

- **Dev Containers: Reopen in Container**.   --> to start the container inside VScode, and open the workspace **inside** the container.
- **Dev Containers: Rebuild Container**.     --> once the container is started VScode, you can rebuild it. Suggested if there are any problems with the camera
- **Dev Containers: Reopen Folder Locally**. --> to get out from the container


## Acknowledgment
**Nickolas Giffen**
  - Northern Illinois University
  - nickolas.giffen@outlook.com
    
**Andrea Vivai**
  - University of Pisa
  - a.vivai@studenti.unipi.it
    
**Alessandro Ciaramella**
  - University of Pisa
  - a.ciaramella@tii.ae


