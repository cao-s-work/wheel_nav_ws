"""
localization.launch.py — ZSL-1W AMCL 定位。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("amcl_registration"),
                         "launch", "amcl.launch.py")
        ])
    )

    return LaunchDescription([amcl_launch])
