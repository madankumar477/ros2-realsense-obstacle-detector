# ROS 2 RealSense Obstacle Detector

A ROS 2 Humble-based obstacle detection system developed and tested using the **Intel RealSense D435i depth camera**. The package processes depth images to detect obstacles within the robot's configured travel corridor and estimates the minimum obstacle distance in real time.

Designed for integration with autonomous mobile robots (AMRs), the system provides obstacle detection status and distance information for navigation and motion-safety applications.

**Maintainer:** E MADAN KUMAR  
**Repository:** `ros2-realsense-obstacle-detector`  
**ROS 2:** Humble  
**Operating System:** Ubuntu 22.04  
**Camera:** Intel RealSense D435i

---

## Project Overview

The detector uses depth information from the Intel RealSense D435i to identify obstacles within a configurable detection region.

It uses camera intrinsics and a floor model to distinguish obstacles from the surrounding floor, filters valid depth measurements and publishes obstacle information through ROS 2 topics.

## Key Features

- Real-time depth-based obstacle detection.
- Intel RealSense D435i integration.
- Configurable obstacle detection distance.
- Region of Interest (ROI) filtering.
- Floor-aware obstacle detection.
- Minimum obstacle distance estimation.
- Fail-safe obstacle state handling.
- Hysteresis and clean-frame requirements.
- Runtime parameter configuration.
- ROS 2 topic-based communication.
- Integration capability with AMR navigation and safety systems.

## Project Architecture

![System Architecture](assets/system_architecture.png)

**Detection pipeline:**

1. Receive depth images and camera information.
2. Decode and process depth measurements.
3. Apply valid-depth and detection-distance filtering.
4. Estimate the floor using camera geometry.
5. Filter obstacles by height and robot travel corridor.
6. Calculate minimum obstacle distance.
7. Publish obstacle state and diagnostic information.

## Repository Structure

```text
ros2-realsense-obstacle-detector/
├── README.md
├── LICENSE
├── .gitignore
├── assets/
│   ├── illustrative_lab_scene.png
│   ├── synthetic_depth_detection.png
│   └── system_architecture.png
├── docs/
│   └── CAMERA_SETUP.md
└── src/
    └── realsense_obstacle_detector/
        ├── config/
        ├── launch/
        ├── realsense_obstacle_detector/
        ├── resource/
        ├── test/
        ├── package.xml
        └── setup.py
```

The `src/` directory contains only the `realsense_obstacle_detector` package. The RealSense camera driver is installed separately.

## Camera Configuration

Configuration file:

`src/realsense_obstacle_detector/config/realsense_obstacle_detector.yaml`

| Parameter | Value | Description |
|---|---:|---|
| `detection_distance` | 0.40 m | Maximum detection distance |
| `min_valid_depth` | 0.25 m | Minimum valid depth |
| `camera_height` | 0.077 m | Camera height above floor |
| `camera_pitch` | -0.0274 rad | Floor-model pitch estimate |
| `depth_image_topic` | `/camera/camera/depth/image_rect_raw` | Depth image input |
| `camera_info_topic` | Derived | Camera information input |
| `corridor_half_width` | 0.45 m | Half-width of detection corridor |

These values correspond to the robot's camera installation. Adjust the geometry and corridor parameters when changing the camera mounting or robot configuration.

## Technologies Used

- ROS 2 Humble
- Python
- OpenCV
- NumPy
- Intel RealSense D435i
- ROS 2 Camera Information
- TF2
- Linux / Ubuntu 22.04

## Dependencies

- ROS 2 Humble
- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `rcl_interfaces`
- `tf2_ros`
- `launch`
- `launch_ros`
- `python3-numpy`
- `realsense2_camera`

Install the Intel RealSense ROS 2 wrapper using its official repository:

[Intel RealSense ROS 2 Wrapper](https://github.com/realsenseai/realsense-ros)

## Installation and Build

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Navigate to the project:

```bash
cd ~/ros2-realsense-obstacle-detector
```

Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```bash
colcon build --symlink-install --packages-select realsense_obstacle_detector
```

Source the workspace:

```bash
source install/setup.bash
```

## Running the Package

**Terminal 1 — Start the RealSense camera**

```bash
source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
camera_namespace:=camera \
camera_name:=camera
```

**Terminal 2 — Start obstacle detection**

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2-realsense-obstacle-detector

source install/setup.bash

ros2 launch realsense_obstacle_detector \
realsense_obstacle_detector.launch.py
```

The detector starts in a blocked state and clears only after receiving valid depth frames that satisfy its configured safety conditions.

## Published ROS 2 Topics

| Topic | Message Type | Description |
|---|---|---|
| `/realsense/obstacle_detected` | `std_msgs/msg/Bool` | Obstacle detection status |
| `/realsense/min_obstacle_distance` | `std_msgs/msg/Float32` | Minimum obstacle distance in metres |
| `/realsense/obstacle_alert` | `std_msgs/msg/String` | Diagnostic alert and blocking reason |

A `
