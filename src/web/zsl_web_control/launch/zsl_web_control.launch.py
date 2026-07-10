"""
zsl_web_control.launch.py — Web 控制网关启动文件。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("static_dir", default_value=""),

        Node(
            package="zsl_web_control",
            executable="web_control_node",
            name="zsl_web_control_node",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "host": LaunchConfiguration("host"),
                "static_dir": LaunchConfiguration("static_dir"),
            }],
        ),
    ])
