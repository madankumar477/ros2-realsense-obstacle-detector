# Standalone launch documentation/defaults updated by E MADAN KUMAR (2026).
#!/usr/bin/env python3
"""
Standalone launch for the RealSense D435i close-range obstacle detector.

This launch file starts the detector independently with this package configuration.

Published topics (available to a robot safety controller):
  /realsense/obstacle_detected      (std_msgs/Bool)
  /realsense/min_obstacle_distance  (std_msgs/Float32)
  /realsense/obstacle_alert         (std_msgs/String)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    depth_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/camera/camera/depth/image_rect_raw',
        description='Depth image topic published by the RealSense ROS2 driver',
    )

    detection_dist_arg = DeclareLaunchArgument(
        'detection_distance',
        default_value='0.40',
        description='Far edge of the blind-spot detection band in metres',
    )

    config_file = PathJoinSubstitution([
        FindPackageShare('realsense_obstacle_detector'),
        'config',
        'realsense_obstacle_detector.yaml',
    ])

    detector_node = Node(
        package='realsense_obstacle_detector',
        executable='realsense_obstacle_detector',
        name='realsense_obstacle_detector',
        output='screen',
        parameters=[
            config_file,
            {
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'detection_distance': ParameterValue(
                    LaunchConfiguration('detection_distance'), value_type=float),
            },
        ],
    )

    return LaunchDescription([
        depth_topic_arg,
        detection_dist_arg,
        detector_node,
    ])
