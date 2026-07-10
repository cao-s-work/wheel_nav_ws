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
        ]
    )
