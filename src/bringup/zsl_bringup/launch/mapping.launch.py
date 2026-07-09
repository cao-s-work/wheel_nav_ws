"""
mapping.launch.py — ZSL-1W 建图模式。
需要先启动 robot_base.launch.py（LiDAR + FAST-LIO + zsl_driver）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_save_path = LaunchConfiguration("map_save_path", default="/home/nvidia/gb_maps")

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "odom_frame": "camera_init",
            "map_frame": "map",
            "base_frame": "base_link",
            "scan_topic": "/scan",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_save_path", default_value="/home/nvidia/gb_maps"),
        slam_node,
    ])
