from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = os.path.join(get_package_share_directory('amcl_registration'), 'config', 'amcl_params.yaml')

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'set_initial_pose',
            default_value='true',
            description='Set initial pose via RViz'
        ),

        # 启动 AMCL 节点
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('scan', '/scan')  # 确保你的 scan 数据是 /scan
            ]
        ),

        # 启动 lifecycle_manager 控制 AMCL 状态
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['amcl']
            }]
        )
    ])