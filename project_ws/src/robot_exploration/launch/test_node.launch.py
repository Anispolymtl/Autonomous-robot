from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_exploration',
            executable='test_node',
            name='cmd_vel_publisher',
            output='both'
        ),
    ])