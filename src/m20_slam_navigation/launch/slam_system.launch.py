"""One-launch M20 Pro mapping using the official ROS 2 / DDS interfaces."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    package_dir = get_package_share_directory("m20_slam_navigation")
    config_path = os.path.join(package_dir, "config", "m20_mapping.yaml")
    rviz_path = os.path.join(package_dir, "rviz", "slam_view.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_topic = LaunchConfiguration("lidar_topic")
    lidar_transport = LaunchConfiguration("lidar_transport")
    drdds_socket_path = LaunchConfiguration("drdds_socket_path")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_transport = LaunchConfiguration("imu_transport")
    drdds_imu_socket_path = LaunchConfiguration("drdds_imu_socket_path")
    map_save_path = LaunchConfiguration("map_save_path")
    max_lidar_queue_size = LaunchConfiguration("max_lidar_queue_size")
    checkpoint_save_period_s = LaunchConfiguration("checkpoint_save_period_s")
    use_vendor_topic_names = LaunchConfiguration("use_vendor_topic_names")
    publish_tf = LaunchConfiguration("publish_tf")
    log_level = LaunchConfiguration("log_level")

    slam_node = LifecycleNode(
        package="m20_slam_navigation",
        executable="slam_node",
        name="slam_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            config_path,
            {
                "use_sim_time": use_sim_time,
                "lidar_topic": lidar_topic,
                "lidar_transport": lidar_transport,
                "drdds.socket_path": drdds_socket_path,
                "imu_topic": imu_topic,
                "imu_transport": imu_transport,
                "drdds.imu_socket_path": drdds_imu_socket_path,
                "map_save_path": map_save_path,
                "lio.max_lidar_queue_size": max_lidar_queue_size,
                "checkpoint_save_period_s": checkpoint_save_period_s,
                "use_vendor_topic_names": use_vendor_topic_names,
                "publish_tf": publish_tf,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="m20_mapping_rviz",
        arguments=["-d", rviz_path],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lidar_topic", default_value="/LIDAR/POINTS"),
            DeclareLaunchArgument("lidar_transport", default_value="drdds"),
            DeclareLaunchArgument(
                "drdds_socket_path", default_value="/tmp/m20_drdds_lidar.sock"
            ),
            DeclareLaunchArgument("imu_topic", default_value="/IMU"),
            DeclareLaunchArgument("imu_transport", default_value="drdds"),
            DeclareLaunchArgument(
                "drdds_imu_socket_path", default_value="/tmp/m20_drdds_imu.sock"
            ),
            DeclareLaunchArgument(
                "map_save_path",
                default_value="/var/opt/robot/data/maps/active/full_cloud.pcd",
            ),
            DeclareLaunchArgument("max_lidar_queue_size", default_value="3"),
            DeclareLaunchArgument("checkpoint_save_period_s", default_value="10.0"),
            DeclareLaunchArgument("use_vendor_topic_names", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            slam_node,
            rviz,
        ]
    )
