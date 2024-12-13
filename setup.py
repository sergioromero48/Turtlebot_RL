import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'marvin2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Install models
        *[(os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
          for root, _, files in os.walk('models')],
        # Install worlds
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        # Install config
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergio',
    maintainer_email='sergioromero48@outlook.com',
    description='Robotics Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_control = marvin2.apriltag_control:main',
            'yolo_control = marvin2.yolo_control:main',
            'marker_pose_node = marvin2.marker_pose_node:main',
            'spawn_marker_node = marvin2.spawn_marker_node:main',
            'marvin_pose_node = marvin2.marvin_pose_node:main',
            'q_learning = marvin2.q_learning:main',
        ],
    },
)
