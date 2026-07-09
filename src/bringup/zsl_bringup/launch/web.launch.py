"""
web.launch.py — 启动 Web 控制网关。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("zsl_web_control"),
                         "launch", "zsl_web_control.launch.py")
        ])
    )

    return LaunchDescription([web_launch])
