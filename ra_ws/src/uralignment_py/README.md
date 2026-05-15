# Markerless Alignment Startup Commands

## 0. Build once after changes

Run this after changing code or after a clean rebuild.
```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For normal startup, each new terminal should start with:
```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---
# 1. Start markerless alignment without Docker

This is the current working configuration.

Do **not** start the AprilTag node during markerless alignment. The markerless node replaces the AprilTag object-pose publisher by publishing the same `cMo` topic.

---

## Terminal 1 — camera node

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_cpp d435i_camera
```

Leave this running.

---

## Terminal 2 — markerless pose estimator

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_py markerless_pose_estimator
```

Leave this running.

Expected startup message:
markerless_pose_estimator ready: subscribed to /camera/color/image_raw, publishing cMo

---

## Terminal 3 — displays node

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_cpp displays
```

---

# 2. Start markerless node in Docker


## Terminal 1 — camera node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_cpp d435i_camera
```

Leave this running.

---

## Terminal 2 — markerless Docker container

```bash
cd ~/Robot-Alignment/ra_ws/src/uralignment_py
./docker/run_markerless_container.sh
```

Expected startup message inside the container:
markerless_pose_estimator ready: subscribed to /camera/color/image_raw, publishing cMo

---

## Optional Docker status check from host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'

ros2 node list
ros2 topic list -t
ros2 topic info /camera/color/image_raw -v
ros2 topic info /cMo -v
```

---
## Optional check from inside the Docker container

```bash
docker exec -it uralignment-py-markerless bash
```

Inside the container:
```bash
source /opt/ros/jazzy/setup.bash
source /home/ur_user/ra_ws/install/setup.bash

ros2 node list
ros2 topic list -t
ros2 topic info /camera/color/image_raw -v
ros2 topic info /cMo -v
```
