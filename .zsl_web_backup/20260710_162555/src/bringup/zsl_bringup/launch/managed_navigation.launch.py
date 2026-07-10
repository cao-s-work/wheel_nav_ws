"""Navigation sub-stack managed by the web console.

This launch intentionally does not start robot_base or another web node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("zsl_bringup")
    default_params = os.path.join(
        get_package_share_directory("robot_navigation2"),
        "config",
        "nav2_params.yaml",
    )
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "localization.launch.py"))
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "navigation.launch.py")),
        launch_arguments={
            "map_file": map_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "rviz": rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_file", description="Absolute path to selected map YAML"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            localization,
            navigation,
        ]
    )
