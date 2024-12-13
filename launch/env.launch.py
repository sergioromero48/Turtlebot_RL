from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marvin2',
            executable='marker_pose_node',  # Marker pose node
            name='marker_pose_node',
            output='screen'
        ),
        Node(
            package='marvin2',
            executable='marvin_pose_node',  # Robot pose node
            name='marvin_pose_node',
            output='screen'
        ),
        Node(
            package='marvin2',
            executable='spawn_marker_node',  # Random marker spawner
            name='spawn_marker_node',
            output='screen'
        )
    ])
