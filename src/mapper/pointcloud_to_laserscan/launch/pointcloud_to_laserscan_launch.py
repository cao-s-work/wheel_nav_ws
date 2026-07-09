"""
pointcloud_to_laserscan_launch.py — ZSL-1W 轮足钢镚适配版。

改动：
  1. cloud_in 订阅 /cloud_registered_body（FAST-LIO 输出的 base_link 帧点云）
  2. 静态 TF 改为 identity（点云已在 base_link，无需旋转）
  3. 高度切片 ZSL-1W 室内参数
"""
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
        # 静态 TF: base_link -> livox_frame
        # 旋转由 FAST-LIO extrinsic 参数控制，此处保持零度
        # x=0.18, z=0.30 为 LiDAR 在 base_link 坐标系中的安装位置
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_livox',
            arguments=[
                '--x', '0.18', '--y', '0', '--z', '0.30',
                '--roll', '0', '--pitch', '0.2793', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
            ]
        ),
        # pointcloud → laserscan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/cloud_registered_body'),   # FAST-LIO body 帧输出
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',      # 已在 base_link，无变换
                'transform_tolerance': 0.05,
                'min_height': 0.05,               # ZSL-1W 轮足室内
                'max_height': 0.60,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.30,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            name='pointcloud_to_laserscan',
            output='screen',
        ),
    ])
