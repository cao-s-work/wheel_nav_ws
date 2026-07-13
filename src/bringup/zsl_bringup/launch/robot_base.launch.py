"""ZSL-1W 基础链路：Livox + FAST-LIO + Scan + 控制安全链。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    read_only = LaunchConfiguration("read_only")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    # 保留兼容参数；当前 CPU FAST-LIO 启动文件暂未使用它。
    use_gpu = LaunchConfiguration("use_gpu")

    # ------------------------------------------------------------------
    # 1. Livox MID360 驱动
    # ------------------------------------------------------------------
    livox_launch_file = os.path.join(
        get_package_share_directory("livox_ros_driver2"),
        "launch",
        "msg_MID360_launch.py",
    )
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_file),
    )

    # ------------------------------------------------------------------
    # 2. FAST-LIO
    # ------------------------------------------------------------------
    fast_lio_launch_file = os.path.join(
        get_package_share_directory("fast_lio"),
        "launch",
        "mapping.launch.py",
    )
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_file),
        launch_arguments={
            "config_file": "mid360.yaml",
            "use_sim_time": use_sim_time,
            "rviz": rviz,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 3. PointCloud2 -> LaserScan
    # ------------------------------------------------------------------
    pointcloud_config = os.path.join(
        get_package_share_directory("zsl_bringup"),
        "config",
        "pointcloud_to_laserscan.yaml",
    )
    pointcloud_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[pointcloud_config],
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            ("scan", "/scan"),
        ],
    )

    # ------------------------------------------------------------------
    # 4. ZSL 驱动
    # ------------------------------------------------------------------
    zsl_driver_node = Node(
        package="zsl_driver",
        executable="zsl_driver_node",
        name="zsl_driver_node",
        output="screen",
        parameters=[
            {
                "read_only": read_only,
                "sdk_local_ip": "192.168.168.216",
                "sdk_local_port": 43988,
                "sdk_dog_ip": "192.168.168.168",
                "cmd_vel_timeout_ms": 500,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 5. 速度仲裁
    # ------------------------------------------------------------------
    cmd_vel_mux_node = Node(
        package="zsl_driver",
        executable="cmd_vel_mux",
        name="cmd_vel_mux",
        output="screen",
        parameters=[
            {
                "nav_timeout_s": 0.5,
                "teleop_timeout_s": 0.5,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 6. 最终安全层
    # ------------------------------------------------------------------
    safety_node = Node(
        package="zsl_driver",
        executable="cmd_vel_safety",
        name="cmd_vel_safety",
        output="screen",
        parameters=[
            {
                "publish_rate": 50.0,
                "input_timeout_s": 0.30,
                "max_vx": 0.25,
                "min_vx": -0.15,
                "max_vy": 0.0,
                "max_wz": 0.45,
                "max_ax": 0.30,
                "max_ay": 0.0,
                "max_aw": 0.50,
            }
        ],
        remappings=[
            (
                "~/estop_latched",
                "/zsl_driver_node/estop_latched",
            ),
            (
                "~/read_only",
                "/zsl_driver_node/read_only",
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "read_only",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_gpu",
                default_value="false",
                description="Reserved for future GPU FAST-LIO selection",
            ),
            livox_launch,
            fast_lio_launch,
            pointcloud_to_scan_node,
            zsl_driver_node,
            cmd_vel_mux_node,
            safety_node,
        ]
    )
