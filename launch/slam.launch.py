import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    rviz_cfg = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('marvin2'), '/launch/tb3.launch.py'])
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        spawn,
        #nav2_bringup,
        slam,
        rviz
    ])