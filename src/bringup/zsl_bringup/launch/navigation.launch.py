"""
navigation.launch.py — ZSL-1W 导航模式（AMCL + Nav2）。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_file = LaunchConfiguration("map_file", default="")
    params_file = LaunchConfiguration("params_file", default="")
    rviz = LaunchConfiguration("rviz", default="false")

    # 默认 indoor 参数
    default_params = os.path.join(
        get_package_share_directory("robot_navigation2"),
        "config", "nav2_params.yaml"
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("nav2_bringup"),
                         "launch", "navigation_launch.py")
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file if params_file != "" else default_params,
            "autostart": "true",
        }.items(),
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    # map_server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )

    # RViz（可选，默认关闭）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        arguments=["-d", os.path.join(
            get_package_share_directory("nav2_bringup"),
            "rviz", "nav2_default_view.rviz"
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_file", default_value="",
                              description="Path to map.yaml"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="Path to nav2_params.yaml (leave empty for indoor default)"),
        DeclareLaunchArgument("rviz", default_value="false"),
        map_server_node,
        map_lifecycle,
        nav2_launch,
        rviz_node,
    ])
