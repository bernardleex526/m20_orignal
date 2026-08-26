"""
M20 Pro — Complete System Launch (SLAM + Navigation)

Brings up the full autonomous navigation stack:
  1. slam_node (LIO + Backend)
  2. navigation_node (Terrain + Planner + Controller)
  3. Static TF
  4. RViz

Topics expected from robot:
  /LIDAR/pointcloud, /IMU, /ODOM

Usage:
  ros2 launch m20_slam_navigation m20_full_system.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("m20_slam_navigation")
    config_dir = os.path.join(pkg_dir, "config")

    use_rviz = LaunchConfiguration("use_rviz", default="true")
    log_level = LaunchConfiguration("log_level", default="info")

    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    declare_log_level = DeclareLaunchArgument("log_level", default_value="info")

    # Static transforms
    static_tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.15", "0.0", "0.05", "0.0", "0.0", "0.0", "1.0",
                       "base_link", "lidar_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0",
                       "base_link", "imu_link"],
        ),
    ]

    # SLAM
    slam_node = Node(
        package="m20_slam_navigation",
        executable="slam_node",
        name="slam_node",
        output="screen",
        parameters=[
            os.path.join(config_dir, "sensors.yaml"),
            os.path.join(config_dir, "lio_params.yaml"),
            os.path.join(config_dir, "backend_params.yaml"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    # Navigation
    nav_node = Node(
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

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "rviz", "slam_view.rviz")],
        condition=lambda _: use_rviz.value == "true",
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_log_level,
        *static_tfs,
        slam_node,
        nav_node,
        rviz_node,
    ])