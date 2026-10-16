"""
lidar_and_map.launch.py

Starts the RPLidar driver and slam_toolbox together, using the settings in
config/rplidar.yaml and config/slam_toolbox.yaml, so building a map is one
command instead of two nodes started by hand in separate terminals.

Usage:
    ros2 launch lucia_lidar lidar_and_map.launch.py

NOT YET VERIFIED: the slam_toolbox executable name below
(async_slam_toolbox_node) is that package's conventional name for online
mapping, but it hasn't been confirmed against what's actually installed here.
The rplidar_ros package turned out to have a different executable name than
assumed (see docs/changelogs/2026-10-09.md) - check this one the same way
before trusting it:
    ros2 pkg executables slam_toolbox
If it doesn't match, fix the `executable=` line below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('lucia_lidar'), 'config')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',  # confirmed 2026-10-09, not a guess
        name='rplidar_node',               # must match rplidar.yaml's top-level key
        parameters=[os.path.join(config_dir, 'rplidar.yaml')],
        output='screen',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # NOT YET VERIFIED - see note above
        name='slam_toolbox',                   # must match slam_toolbox.yaml's top-level key
        parameters=[os.path.join(config_dir, 'slam_toolbox.yaml')],
        output='screen',
    )

    return LaunchDescription([rplidar_node, slam_toolbox_node])
