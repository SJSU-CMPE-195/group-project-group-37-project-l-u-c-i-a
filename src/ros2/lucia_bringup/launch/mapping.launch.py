"""
mapping.launch.py

Everything needed to build a map, in one command:
- roomba_bridge (wheels, /odom)
- imu_bridge (Ling's BNO08X Pico, /imu/data) + where the IMU sits on the robot
- robot_localization EKF combining /odom and /imu/data into odom->base_link
  (config/ekf.yaml)
- lucia_lidar's lidar_and_map.launch.py (lidar + slam_toolbox)

Usage:
    ros2 launch lucia_bringup mapping.launch.py
    ros2 launch lucia_bringup mapping.launch.py use_imu:=false   # wheels only, as before
Then drive (for now) from a second terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5

use_imu:=false falls back to the first-map setup: roomba_bridge publishes
odom->base_link itself and the IMU/EKF aren't started. Useful for comparing
maps with and without the IMU, or if the IMU isn't plugged in.

The IMU transform is identity: mount the board flat, chip side up. Which way it
points doesn't matter - only heading and turn rate are used, and the EKF treats
its heading at startup as 0 (imu0_relative in ekf.yaml).

NOT YET RUN on the Pi.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_config = os.path.join(get_package_share_directory('lucia_bringup'), 'config')
    lidar_launch = os.path.join(
        get_package_share_directory('lucia_lidar'), 'launch', 'lidar_and_map.launch.py')

    use_imu = LaunchConfiguration('use_imu')
    with_imu = IfCondition(use_imu)

    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Fuse the IMU heading via robot_localization'),
        DeclareLaunchArgument('roomba_port', default_value='/dev/roomba'),
        DeclareLaunchArgument('imu_port', default_value='/dev/imu'),

        Node(
            package='lucia_control',
            executable='roomba_bridge',
            name='roomba_bridge',
            parameters=[{
                'port': LaunchConfiguration('roomba_port'),
                'publish_tf': NotSubstitution(use_imu),   # the EKF publishes it instead
            }],
            output='screen',
        ),

        Node(
            package='lucia_imu',
            executable='imu_bridge',
            name='imu_bridge',
            parameters=[{'port': LaunchConfiguration('imu_port')}],
            output='screen',
            condition=with_imu,
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            condition=with_imu,
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',             # must match ekf.yaml's top-level key
            parameters=[os.path.join(bringup_config, 'ekf.yaml')],
            output='screen',
            condition=with_imu,
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),
    ])
