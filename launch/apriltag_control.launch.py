from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='marvin2',
			executable='apriltag_control',
			name='apriltag_control',
			output='screen'
		)
	])