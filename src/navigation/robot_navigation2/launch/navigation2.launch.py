import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

#因为
def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map_file',
        default='')

    rviz = launch.substitutions.LaunchConfiguration('rviz', default='false')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),
        launch.actions.DeclareLaunchArgument(
            'map_file',
            default_value=map_yaml_path,
            description='Path to map.yaml for map_server'
        ),
        launch.actions.DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2'
        ),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'autostart': 'true',
            }.items(),
        ),

        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path},
                       {'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': True},
                       {'node_names': ['map_server']}]),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=launch.conditions.IfCondition(rviz)),
    ])


# import os
# import launch
# import launch_ros
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#     # 获取与拼接默认路径
#     fishbot_navigation2_dir = get_package_share_directory(
#         'robot_navigation2')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     rviz_config_dir = os.path.join(
#         nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
#     # 创建 Launch 配置
#     use_sim_time = launch.substitutions.LaunchConfiguration(
#         'use_sim_time', default='False')
#     map_yaml_path = launch.substitutions.LaunchConfiguration(
#         'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'map.yaml'))
#     nav2_param_path = launch.substitutions.LaunchConfiguration(
#         'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

#     return launch.LaunchDescription([
#         # 声明新的 Launch 参数
#         launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
#                                              description='Use simulation (Gazebo) clock if true'),
#         launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
#                                              description='Full path to map file to load'),
#         launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
#                                              description='Full path to param file to load'),

#         launch.actions.IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
#             # 使用 Launch 参数替换原有参数
#             launch_arguments={
#                 'map': map_yaml_path,
#                 'use_sim_time': use_sim_time,
#                 'params_file': nav2_param_path}.items(),
#         ),
#         launch_ros.actions.Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             parameters=[{'use_sim_time': use_sim_time}],
#             output='screen'),
#     ])