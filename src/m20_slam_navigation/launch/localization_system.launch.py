"""
M20 Pro — Prior Map Localization System Launch

Launches:
  - localization_node: NDT Relocalization + ESKF Tracking
  - Static TF transforms

Usage:
  ros2 launch m20_slam_navigation localization_system.launch.py map_path:=/path/to/map.pcd
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("m20_slam_navigation")

    map_path = LaunchConfiguration("map_path", default="full_cloud.pcd")
    log_level = LaunchConfiguration("log_level", default="info")
    config_dir = os.path.join(pkg_dir, "config")

    declare_map_path = DeclareLaunchArgument("map_path", default_value="full_cloud.pcd",
                                             description="Path to pre-built PCD map")
    declare_log_level = DeclareLaunchArgument("log_level", default_value="info")

    # Static TF
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.15", "0.0", "0.05", "0.0", "0.0", "0.0", "1.0",
                   "base_link", "lidar_link"],
        name="static_tf_base_lidar",
    )

    static_tf_base_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0",
                   "base_link", "imu_link"],
        name="static_tf_base_imu",
    )

    # Localization node
    localization_node = Node(
        package="m20_slam_navigation",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[
            os.path.join(config_dir, "sensors.yaml"),
            os.path.join(config_dir, "localization_params.yaml"),
            {"map_path": map_path},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_map_path,
        declare_log_level,
        static_tf_base_lidar,
        static_tf_base_imu,
        localization_node,
    ])