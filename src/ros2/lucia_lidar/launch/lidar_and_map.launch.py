"""
lidar_and_map.launch.py

Starts the RPLidar driver and slam_toolbox together, using the settings in
config/rplidar.yaml and config/slam_toolbox.yaml, so building a map is one
command instead of two nodes started by hand in separate terminals.

Usage:
    ros2 launch lucia_lidar lidar_and_map.launch.py
    ros2 launch lucia_lidar lidar_and_map.launch.py laser_yaw:=0.0

The control node (lucia_control roomba_bridge) must also be running: it
publishes the odom->base_link transform slam_toolbox needs.

slam_toolbox is started through its own online_async_launch.py rather than as
a plain Node. In Jazzy it is a lifecycle node, which sits idle until it's told
to configure and activate; the package's launch file does that. Starting it
directly would show the node running but never build a map.

laser_yaw: which way the lidar faces on the robot, as the rotation from
base_link (robot forward) to the lidar's frame. Default pi, because
last semester's code treats the lidar's own 0 deg mark as robot-forward
(slam_avoid_server.py get_front), and rplidar_ros publishes that direction at
180 deg in its frame. NOT YET VERIFIED on the robot - check with
`ros2 run lucia_lidar scan_check` (see that file) before trusting a map.
The lidar's x/y offset from the robot's center isn't measured yet either; it's
0 for now.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('lucia_lidar'), 'config')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw',
        default_value=str(math.pi),
        description='Rotation (radians) from robot-forward to the lidar frame',
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',  # confirmed 2026-10-09, not a guess
        name='rplidar_node',               # must match rplidar.yaml's top-level key
        parameters=[os.path.join(config_dir, 'rplidar.yaml')],
        output='screen',
    )

    # Where the lidar sits on the robot. Without this, slam_toolbox can't relate
    # a scan (frame 'laser') to the robot (frame 'base_link') and drops every scan.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',    # must match frame_id in rplidar.yaml
            '--yaw', LaunchConfiguration('laser_yaw'),
        ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': os.path.join(config_dir, 'slam_toolbox.yaml'),
            'use_sim_time': 'false',        # the package's launch file defaults this to true
        }.items(),
    )

    return LaunchDescription([laser_yaw_arg, rplidar_node, laser_tf, slam_toolbox])
