from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[
                '/home/sergio/ros2_ws/src/marvin2/config/apriltag.yaml',
                {'use_intra_process_comms': True},
                {'use_sim_time': True}
            ],
            remappings=[
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ]
        ),
    ])
