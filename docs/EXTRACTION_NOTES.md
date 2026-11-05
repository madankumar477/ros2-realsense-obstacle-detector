## Project Scope and Validation

- **Single ROS 2 package:** This repository contains only `realsense_obstacle_detector`.
- **Standalone source structure:** The root `src/` directory contains only the obstacle detector package. The RealSense ROS 2 camera driver is installed separately.
- **Detection functionality:** Processes RealSense depth images and CameraInfo to estimate obstacle distance within a configured robot travel corridor and publishes obstacle status and diagnostic information.
- **Camera configuration:** The included camera topics and geometric parameters are specific to the robot installation. Measure the camera mounting height and pitch before deployment.
- **Clean repository:** Build, install and log directories, Python bytecode, caches and nested Git repositories are excluded.
- **Offline testing:** Synthetic depth inputs are used to test detector logic without requiring ROS 2 or a physical camera.
- **Hardware validation:** Full ROS 2 Humble build and physical-camera validation are performed on the target platform.

## Authorship and Licensing

**Author and Maintainer:** E MADAN KUMAR

This project is authored and maintained by E MADAN KUMAR. Third-party dependencies retain their respective licenses and copyright notices. Applicable notices must be preserved when installing or redistributing those dependencies.
