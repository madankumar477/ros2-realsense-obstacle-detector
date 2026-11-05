# RealSense D435i camera setup notes

This package consumes depth and CameraInfo messages; it does not launch or vendor the RealSense driver. Install and configure the official `realsense2_camera` ROS 2 wrapper separately.

For the camera topic names used by the source workspace, start the wrapper with the camera namespace and name set to `camera`:

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=camera
```

Confirm the topics before starting the detector:

```bash
ros2 topic list | grep /camera/camera/depth
```

Expected inputs:

- `/camera/camera/depth/image_rect_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)

The detector derives the CameraInfo topic from the depth topic by replacing the `image_rect_raw` suffix with `camera_info`. If you use a different namespace or topic convention, set `camera_info_topic` explicitly along with `depth_image_topic` when launching the detector.

## Configured installation geometry

- Lens height: `0.077 m`
- Camera pitch: `-0.0274 rad`
- Minimum valid depth: `0.25 m`
- Detection distance: `0.40 m`

These settings describe one physical installation. Measure the lens height and pitch on the target platform; do not copy them blindly to another robot.

Official wrapper documentation: [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros).
