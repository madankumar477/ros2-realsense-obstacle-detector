# Project scope and validation notes

- This repository contains one ROS 2 package: `realsense_obstacle_detector`.
- The root `src/` directory contains only that package. The RealSense ROS 2 camera driver is an external dependency and is not vendored here.
- The package consumes a RealSense depth image and CameraInfo, estimates close-range obstacle distance in a configured robot corridor, and publishes obstacle state and diagnostics.
- Camera topic names and geometry in the included YAML are installation-specific. Measure the camera mounting height and pitch on the target robot before use.
- Build/install/log outputs, Python bytecode, caches, and nested Git repositories are not included.
- Offline tests use synthetic depth inputs and can be run without ROS 2 or a physical camera. A physical-camera test and full ROS 2 Humble build must be performed on the target platform.
- The project is authored and maintained by E MADAN KUMAR. Third-party dependencies remain separately licensed; preserve their applicable notices when installing or redistributing them.
