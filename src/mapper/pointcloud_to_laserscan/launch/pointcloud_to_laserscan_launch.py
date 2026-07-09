from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0.37', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0.1478', '--qz', '0', '--qw', '0.9890',
                '--frame-id', 'base_link', '--child-frame-id' , 'livox_frame'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/livox/lidar'),   # 直接订阅真实点云
                ('scan', '/scan')  # 输出 /scanner/scan
            ],
            parameters=[{
                'target_frame': 'base_link',  # 坐标系要与 TF 匹配
                'transform_tolerance': 0.01,
                'min_height': -0.5,  # 可根据实际调整
                'max_height': 0.1,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])