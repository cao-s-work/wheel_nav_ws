"""ZSL-1W navigation mode: map server + Nav2 navigation servers.

The Nav2 Humble internal velocity chain is preserved:
controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel.
The external cmd_vel_mux subscribes to /cmd_vel.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("robot_navigation2"),
        "config",
        "nav2_params.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file, "use_sim_time": use_sim_time}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "use_sim_time": use_sim_time,
                "node_names": ["map_server"],
            }
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz",
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map_file", description="Absolute path to map YAML"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz", default_value="false"),
            map_server,
            map_lifecycle,
            navigation,
            rviz_node,
        ]
    )
