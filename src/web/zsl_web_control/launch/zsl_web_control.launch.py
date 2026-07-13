"""Launch the commercial ZSL web operations console."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("zsl_web_control")
    default_config = os.path.join(package_share, "config", "web_control.yaml")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="zsl_web_control",
                executable="web_control_node",
                name="zsl_web_control_node",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="zsl_web_control",
                executable="topic_rate_monitor",
                name="topic_rate_monitor",
                output="screen",
                parameters=[{
                    "lidar_topic": "/livox/lidar",
                    "scan_topic": "/scan",
                    "odom_topic": "/Odometry",
                    "output_topic": "/system/topic_rates",
                    "publish_period_s": 0.5,
                    "rate_window_s": 2.0,
                    "alive_timeout_s": 1.5,
                }],
            ),
        ]
    )
