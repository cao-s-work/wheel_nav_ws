"""
zsl_driver.launch.py — ZSL-1W 轮足钢镚驱动启动文件。
对标铜锤 tonchui_control 的 launch。

用法:
  ros2 launch zsl_driver zsl_driver.launch.py
  ros2 launch zsl_driver zsl_driver.launch.py read_only:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sdk_local_ip", default_value="192.168.168.216"),
        DeclareLaunchArgument("sdk_local_port", default_value="43988"),
        DeclareLaunchArgument("sdk_dog_ip", default_value="192.168.168.168"),
        DeclareLaunchArgument("read_only", default_value="true"),
        DeclareLaunchArgument("cmd_vel_timeout_ms", default_value="500"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        DeclareLaunchArgument("angular_scale", default_value="1.0"),
        DeclareLaunchArgument("cmd_vel_publish_rate", default_value="50"),
        DeclareLaunchArgument("state_publish_rate", default_value="10.0"),

        Node(
            package="zsl_driver",
            executable="zsl_driver_node",
            name="zsl_driver_node",
            output="screen",
            parameters=[{
                "sdk_local_ip": LaunchConfiguration("sdk_local_ip"),
                "sdk_local_port": LaunchConfiguration("sdk_local_port"),
                "sdk_dog_ip": LaunchConfiguration("sdk_dog_ip"),
                "read_only": LaunchConfiguration("read_only"),
                "cmd_vel_timeout_ms": LaunchConfiguration("cmd_vel_timeout_ms"),
                "speed_scale": LaunchConfiguration("speed_scale"),
                "angular_scale": LaunchConfiguration("angular_scale"),
                "cmd_vel_publish_rate": LaunchConfiguration("cmd_vel_publish_rate"),
                "state_publish_rate": LaunchConfiguration("state_publish_rate"),
            }],
        ),
    ])
