"""M20Pro prior-map localization plus native-contract navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("m20_slam_navigation")
    config_dir = os.path.join(package_dir, "config")

    map_path = LaunchConfiguration("map_path")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    enable_motion_output = LaunchConfiguration("enable_motion_output")

    localization_node = Node(
        package="m20_slam_navigation",
        executable="localization_node",
        name="localization_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(config_dir, "localization_params.yaml"),
            {"adapter.map_path": map_path},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    navigation_node = Node(
        package="m20_slam_navigation",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(config_dir, "native_navigation.yaml"),
            {"navigation.enable_motion_output": enable_motion_output},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="m20_navigation_rviz",
        arguments=["-d", os.path.join(package_dir, "rviz", "navigation_view.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value="",
                description="Optional PCD override; empty uses the OEM active-map path",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "enable_motion_output",
                default_value="false",
                description="Keep false until real DrDDS motion acceptance is authorized",
            ),
            localization_node,
            navigation_node,
            rviz,
        ]
    )
