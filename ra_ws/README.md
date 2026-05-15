# Save cdMo.yaml + Offset Distance Parameter

![[Screenshot from 2026-05-15 12-00-22.png]]

## 0. Setup
### Terminal 1 — C++ camera node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp d435i_camera
```
Leave this running.
### Terminal 2 — C++ object detection node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp apriltags
```
Leave this running.
### Terminal 3 — C++ displays node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp displays
```
Leave this running.

## 1. Manual Alignment

### 1.1 Manually align the tooling (coupler flange) to the target (cavity flange).

Tip: To save time, run the PBVS alignment normally, manually close the offset distance, and then switch to manual control and correct the misalignment.

### 1.2 Back off in the z-direction

Manually move the robot tooling away from cavity along the offset axis.
Stop once satisfied with the offset alignment distance.
Check the camera view from the displays node to ensure target object (cavity flange) sits comfortably within the camera field-of-view.

### 1.3 Update cdPo.yaml

Select the displays node camera view and press `c` to make the currect cPo (RGB Pose) the new cdPo (saved to YAML as the configured desired alignment position)

### 1.4 Update the corresponding offset distance.

To parameterize the offset distance, the corresponding offset distance at the configured alignment position must be changed in the script, otherwise needing to have that parameter set in terminal upon every subsequent startup. It is easier to simply update the parameter in script and rebuild the package.
Open the robot node script `ur_e_series.cpp`:
```bash
cd ~/Robot-Alignment/ra_ws/src/uralignment_cpp/src
code ur_e_series.cpp
```
Find 2 instances where the offset parameter is set `desired_offset_d_m` and replace the measured value in Meters:
```text
desired_offset_d_m = this->declare_parameter<double>("desired_offset_d_m", 0.2159);
```
```text
desired_offset_d_m = this->declare_parameter<double>("desired_offset_d_m", 0.2159);
```
Here, you would replace `0.2159` Meters with the updated value in both lines of code.

## 2. Rebuild and Start ROS 2 Nodes

### Terminal 1 — C++ camera node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build
source install/setup.bash
ros2 run uralignment_cpp d435i_camera
```
Leave this running.
### Terminal 2 — C++ object detection (apriltags) node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp apriltags
```
Leave this running.

### Terminal 3 — C++ TUI node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp teleop_key
```
Leave this running.

### Terminal 4 — C++ ur_e_series node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp ur_e_series
```
Leave this running.
### Terminal 5 — C++ displays node on host

```bash
cd ~/Robot-Alignment/ra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uralignment_cpp displays
```
Leave this running.

## 2.1 Test Accuracy

Once the ROS 2 Nodes are started, run PBVS until the alignment is complete.
Switch to manual mode and close the offset distance.
Confirm that the newly updated cdPo.yaml is accurate.