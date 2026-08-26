"""
M20 Pro — Full Navigation System Launch

Launches:
  - navigation_node: Terrain Analysis + Global Planner + Local Controller
  - Requires SLAM or Localization to be running separately (for TF /map → odom → base_link)

Usage:
  ros2 launch m20_slam_navigation navigation_system.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("m20_slam_navigation")
    config_dir = os.path.join(pkg_dir, "config")

    log_level = LaunchConfiguration("log_level", default="info")
    declare_log_level = DeclareLaunchArgument("log_level", default_value="info")

    navigation_node = Node(
        package="m20_slam_navigation",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[
            os.path.join(config_dir, "terrain_params.yaml"),
            os.path.join(config_dir, "planner_params.yaml"),
            os.path.join(config_dir, "controller_params.yaml"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    return LaunchDescription([
        declare_log_level,
        navigation_node,
    ])