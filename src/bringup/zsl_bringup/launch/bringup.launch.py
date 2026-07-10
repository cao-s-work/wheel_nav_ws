"""
bringup.launch.py — ZSL-1W 统一入口。

用法:
  ros2 launch zsl_bringup bringup.launch.py mode:=mapping web:=true
  ros2 launch zsl_bringup bringup.launch.py mode:=navigation map:=/path/to/map.yaml web:=true
  ros2 launch zsl_bringup bringup.launch.py mode:=outdoor_nav map:=/path/to/map.yaml web:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mode = LaunchConfiguration("mode", default="base")
    web = LaunchConfiguration("web", default="false")
    map_file = LaunchConfiguration("map", default="")
    rviz = LaunchConfiguration("rviz", default="false")
    read_only = LaunchConfiguration("read_only", default="true")

    pkg_dir = get_package_share_directory("zsl_bringup")
    nav_pkg = get_package_share_directory("robot_navigation2")

    # 条件表达式
    is_mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    is_navigation = IfCondition(PythonExpression([
        "'", mode, "' in ['navigation', 'outdoor_nav']"
    ]))
    is_web = IfCondition(web)
    is_outdoor = PythonExpression(["'", mode, "' == 'outdoor_nav'"])

    indoor_params = os.path.join(nav_pkg, "config", "nav2_params.yaml")
    outdoor_params = os.path.join(nav_pkg, "config", "nav2_params_outdoor.yaml")

    # ---- robot_base (始终启动) ----
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, "launch", "robot_base.launch.py")
        ]),
        launch_arguments={
            "read_only": read_only,
            "rviz": rviz,
        }.items(),
    )

    # ---- mapping ----
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, "launch", "mapping.launch.py")
        ]),
        condition=is_mapping,
    )

    # ---- localization ----
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, "launch", "localization.launch.py")
        ]),
        condition=is_navigation,
    )

    # ---- navigation ----
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, "launch", "navigation.launch.py")
        ]),
        launch_arguments={
            "map_file": map_file,
            "params_file": indoor_params,
            "rviz": rviz,
        }.items(),
        condition=is_navigation,
    )

    # ---- web ----
    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, "launch", "web.launch.py")
        ]),
        condition=is_web,
    )

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="base",
                              description="base / mapping / navigation / outdoor_nav"),
        DeclareLaunchArgument("web", default_value="false",
                              description="启动 Web 控制台"),
        DeclareLaunchArgument("map", default_value="",
                              description="Path to map.yaml"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("read_only", default_value="true"),
        base_launch,
        mapping_launch,
        localization_launch,
        nav_launch,
        web_launch,
    ])
