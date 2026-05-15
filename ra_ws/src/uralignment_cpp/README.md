# Configure Runtime cdMo

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

# 1. Start `ur_e_series` for each `cdMo_mode`

Only run **one** `ur_e_series` command at a time.

The robot node subscribes to `cMo`, `/teleop/last_key`, and `/offset/cmd_twist`. Start the camera, object-pose node, and teleop before commanding robot motion.

---

## 1.1 `saved_yaml`

Uses the saved desired pose from `cdPo.yaml`.

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_cpp ur_e_series --ros-args   -p cdMo_mode:=saved_yaml
```

---

## 1.2 `pseudo_parameterized_offset`

Uses the pseudo-parameterized offset mode.

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_cpp ur_e_series --ros-args   -p cdMo_mode:=pseudo_parameterized_offset
```

---

## 1.3 `calibrated_parameterized_offset`

Uses the extrinsics calibrated parameterized offset mode.

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uralignment_cpp ur_e_series --ros-args   -p cdMo_mode:=calibrated_parameterized_offset
```
