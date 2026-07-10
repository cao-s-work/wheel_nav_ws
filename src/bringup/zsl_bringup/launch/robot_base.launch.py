"""
robot_base.launch.py — ZSL-1W 机器人基础驱动启动（Livox + FAST-LIO + zsl_driver）。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    read_only = LaunchConfiguration("read_only", default="true")
    use_gpu = LaunchConfiguration("use_gpu", default="false")
    rviz = LaunchConfiguration("rviz", default="false")

    # 1. Livox MID-360
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("livox_ros_driver2"),
                         "launch_ROS2", "msg_MID360_launch.py")
        ])
    )

    # 2. FAST-LIO
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("fast_lio"),
                         "launch", "mapping.launch.py")
        ]),
        launch_arguments={
            "config_file": "mid360.yaml",
            "rviz": "false",
        }.items(),
    )

    # 3. pointcloud_to_laserscan
    pcl_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[{
            "target_frame": "base_link",
            "min_height": -0.5,
            "max_height": 0.5,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.3,
            "range_max": 10.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            ("scan", "/scan"),
        ],
    )

    # 4. zsl_driver
    zsl_driver_node = Node(
        package="zsl_driver",
        executable="zsl_driver_node",
        name="zsl_driver_node",
        output="screen",
        parameters=[{
            "read_only": read_only,
            "sdk_local_ip": "192.168.168.216",
            "sdk_local_port": 43988,
            "sdk_dog_ip": "192.168.168.168",
            "cmd_vel_timeout_ms": 500,
        }],
    )

    # 5. cmd_vel_mux (合并 teleop + nav → selected)
    cmd_vel_mux_node = Node(
        package="zsl_driver",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
        parameters=[{
            "nav_timeout_s": 0.5,
            "teleop_timeout_s": 0.5,
        }],
    )

    # 6. cmd_vel_safety (限速/加速度/watchdog/estop/read_only → safe)
    safety_node = Node(
        package="zsl_driver",
        executable="cmd_vel_safety",
        name="cmd_vel_safety",
        output="screen",
        parameters=[{
            "publish_rate": 50.0,
            "input_timeout_s": 0.30,
            "max_vx": 0.30,
            "min_vx": -0.15,
            "max_vy": 0.0,
            "max_wz": 0.50,
            "max_ax": 0.30,
            "max_ay": 0.0,
            "max_aw": 0.50,
        }],
        remappings=[
            ("~/estop_latched", "/zsl_driver_node/estop_latched"),
            ("~/read_only", "/zsl_driver_node/read_only"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("read_only", default_value="true"),
        DeclareLaunchArgument("use_gpu", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        livox_launch,
        fast_lio_launch,
        pcl_node,
        zsl_driver_node,
        cmd_vel_mux_node,
        safety_node,
    ])
