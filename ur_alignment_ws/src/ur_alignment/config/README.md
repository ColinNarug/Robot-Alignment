# Configure cdMo.yaml & Camera Intrinsics for Markerless in ur_alignment_ws

## 0. Setup
The following files and procedures must be conducted prior and will be the values used to be pasted into ur_alignment_ws
### 0.1 Locate cdPo.yaml
Follow the instructions for configuring and saving an updated cdPo.yaml, which can be found using the following terminal command:
```bash
cd ~/Robot-Alignment/ra_ws/src/uralignment_cpp
code README.md
```
The actual cdPo.yaml file is located here:
```bash
cd ~/Robot-Alignment/ra_ws/src/calibration_cpp/config
code cdPo.yaml
```
### 0.2 Locate Camera Intrinsics
Find the camera serial number and resolution associated with the 
```bash
cd ~/Robot-Alignment/ra_ws/src/calibration_cpp/config
code .
```
There willl be a YAML file with the intrinsics you need. For example, if the camera serial is `050422071950` and the resolution is `1920x1080`, you need to find YAML named `050422071950_1920x1080_intrinsics.yaml`, which is located here:
```bash
cd ~/Robot-Alignment/ra_ws/src/calibration_cpp/config
code 050422071950_1920x1080_intrinsics.yaml
```
If it does not yet exist, you must run through the intrinsics camera calibration prodedure to create the file, otherwise use another camera and/or resolution to match what is currently available.

## 1. Replacing Values
### 1.1 Desired Alignment Position YAML
Copy and Paste the values of cdPo.yaml into the file at this location:
```bash
cd ~/Robot-Alignment/ur_alignment_ws/src/ur_alignment/config
code ur_cdMo.yaml
```

### 1.2 Camera Intrinsics
Copy and Paste the correct matrix entries into the file at this location:
```bash
cd ~/Robot-Alignment/ur_alignment_ws/src/ur_alignment/config
code camera_intrinsics.yaml
```
Note that the formatting is different and will require inspecting the formatting differences to find the corresponding values