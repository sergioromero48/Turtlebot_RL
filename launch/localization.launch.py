from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    navigation2_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'launch',
        'navigation2.launch.py'
    ])
    tb3wrld_path = PathJoinSubstitution([
        FindPackageShare('marvin2'),
        'launch',
        'tb3.launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([navigation2_path]),
            launch_arguments={
                'use_sim_time': 'true',
                'map': '/home/sergio/ros2_ws/hw3.yaml'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([tb3wrld_path]),
        )
    ])
