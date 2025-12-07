# capstone_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_actions',
            executable='nav_action_server',
            name='nav_action_server',
            output='screen'
        ),
        Node(
            package='robot_actions',
            executable='manip_action_server',
            name='manip_action_server',
            output='screen'
        ),
    ])
