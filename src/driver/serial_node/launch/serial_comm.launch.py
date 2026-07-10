from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_node',
            executable='serial_twist_publisher',
            name='serial_twist_publisher',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200},
                {'linear_scale': 5000.0},     
                {'angular_scale': 500.0},   
            ],
            output='screen'
        )
    ])