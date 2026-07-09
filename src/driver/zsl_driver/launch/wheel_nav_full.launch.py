"""
wheel_nav_full.launch.py — ZSL-1W 轮足钢镚全链路启动。

链路：
  Livox MID-360 → FAST-LIO → /Odometry + /cloud_registered_body
  → pointcloud_to_laserscan → /scan
  → Nav2 (AMCL + planner + controller + costmaps)
  → /cmd_vel → zsl_driver → SDK move()
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    read_only = LaunchConfiguration("read_only", default="true")
    rviz = LaunchConfiguration("rviz", default="false")

    # =========================================================================
    # 1. Livox MID-360 LiDAR 驱动
    # =========================================================================
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch_ROS2", "msg_MID360_launch.py"
            )
        ])
    )

    # =========================================================================
    # 2. FAST-LIO
    # =========================================================================
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("fast_lio"),
                "launch", "mapping.launch.py"
            )
        ]),
        launch_arguments={
            "config_file": "mid360.yaml",
            "rviz": "false",
        }.items(),
    )

    # =========================================================================
    # 3. pointcloud_to_laserscan
    # =========================================================================
    pcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("pointcloud_to_laserscan"),
                "launch", "pointcloud_to_laserscan_launch.py"
            )
        ])
    )

    # =========================================================================
    # 4. Nav2 (AMCL + 导航)
    # =========================================================================
    nav2_params = os.path.join(
        get_package_share_directory("robot_navigation2"),
        "config", "nav2_params.yaml"
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch", "navigation_launch.py"
            )
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    # map_server (独立启动，提供 /map)
    map_file = LaunchConfiguration("map_file", default="")
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_file},
        ],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # =========================================================================
    # 5. zsl_driver
    # =========================================================================
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
            "speed_scale": 1.0,
            "angular_scale": 1.0,
        }],
    )

    # =========================================================================
    # 6. RViz (可选)
    # =========================================================================
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
        DeclareLaunchArgument("read_only", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "map_file", default_value="",
            description="Path to map.yaml for map_server"),
        livox_launch,
        fast_lio_launch,
        pcl_launch,
        map_server,
        map_lifecycle,
        nav2_launch,
        zsl_driver_node,
        rviz_node,
    ])
