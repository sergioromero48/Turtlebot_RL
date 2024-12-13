from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_pkg',
            executable='yolo_test',
            name='yolo_test'
        ),
        Node(
            package='marvin2',
            executable='yolo_control',
            name='yolo_control'
        )
    ])