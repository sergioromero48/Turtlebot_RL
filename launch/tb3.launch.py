import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Path to the Gazebo world file
    pkg = 'marvin2'
#    robot_subpath = 'models/tb3.urdf'

    world = os.path.join(get_package_share_directory(pkg), 'worlds', 'empty_world.sdf')
#    file = os.path.join(get_package_share_directory(pkg), robot_subpath)


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

#    with open(file, 'r') as file:
#        robot_desc = file.read()

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg), 'launch'), '/tb3_state_pub.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world}.items()
        )
    
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg), 'launch'), '/tb3_spawn.launch.py']),
            launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
        )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn
    ])