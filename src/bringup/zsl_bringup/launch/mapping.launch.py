"""ZSL-1W mapping sub-stack.

Assumes robot_base.launch.py is already running. Starts SLAM Toolbox and a
Nav2 map_saver_server so the commercial web console can save maps directly.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": "mapping",
            }
        ],
    )
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "save_map_timeout": 10.0,
                "free_thresh_default": 0.25,
                "occupied_thresh_default": 0.65,
                "map_subscribe_transient_local": True,
            }
        ],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["slam_toolbox", "map_saver"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            slam,
            map_saver,
            lifecycle,
        ]
    )
