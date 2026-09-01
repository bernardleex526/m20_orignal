"""
M20 Pro — Prior Map Localization System Launch

Launches:
  - localization_node: NDT Relocalization + ESKF Tracking

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

    map_path = LaunchConfiguration("map_path")
    log_level = LaunchConfiguration("log_level", default="info")
    config_dir = os.path.join(pkg_dir, "config")

    declare_map_path = DeclareLaunchArgument(
        "map_path",
        default_value="",
        description=(
            "Optional PCD override; empty uses the native "
            "/var/opt/robot/data/maps/active/full_cloud.pcd"
        ),
    )
    declare_log_level = DeclareLaunchArgument("log_level", default_value="info")

    # Localization node
    localization_node = Node(
        package="m20_slam_navigation",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[
            os.path.join(config_dir, "localization_params.yaml"),
            {"adapter.map_path": map_path},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_map_path,
        declare_log_level,
        localization_node,
    ])
