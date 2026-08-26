"""
M20 Pro — Full SLAM System Launch

Launches:
  - slam_node: LIO Front-End + Back-End Factor Graph + Loop Closure
  - RViz visualization (optional)
  - Static TF transforms (LiDAR → IMU → Base)

Usage:
  ros2 launch m20_slam_navigation slam_system.launch.py use_rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("m20_slam_navigation")

    # ---- Launch Arguments ----
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    log_level = LaunchConfiguration("log_level", default="info")
    config_dir = os.path.join(pkg_dir, "config")

    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="true",
                                             description="Launch RViz")
    declare_log_level = DeclareLaunchArgument("log_level", default_value="info",
                                              description="Logging level")

    # ---- Static TF: base_link → lidar_link ----
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.15", "0.0", "0.05", "0.0", "0.0", "0.0", "1.0",
                   "base_link", "lidar_link"],
        name="static_tf_base_lidar",
    )

    # Static TF: base_link → imu_link
    static_tf_base_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0",
                   "base_link", "imu_link"],
        name="static_tf_base_imu",
    )

    # ---- SLAM Node (LIO + Back-End) ----
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

    # ---- RViz ----
    rviz_config = os.path.join(pkg_dir, "rviz", "slam_view.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=lambda _: use_rviz.value == "true",
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_log_level,
        static_tf_base_lidar,
        static_tf_base_imu,
        slam_node,
        rviz_node,
    ])