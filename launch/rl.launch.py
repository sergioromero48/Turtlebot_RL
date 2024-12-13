from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marvin2',
            executable='q_learning',  # Q-learning navigator
            name='q_learning',
            output='screen'
        )
    ])