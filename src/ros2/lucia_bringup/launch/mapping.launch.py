"""
mapping.launch.py

Everything needed to build a map, in one command:
- roomba_bridge (wheels, /odom), listening on /cmd_vel_safe
- safety_gate: passes /cmd_vel on to /cmd_vel_safe, blocking forward motion
  while the ToF sensors see something close (only with use_tof:=true)
- imu_bridge (Ling's BNO08X Pico, /imu/data) + where the IMU sits on the robot
- robot_localization EKF combining /odom and /imu/data into odom->base_link
  (config/ekf.yaml)
- lucia_lidar's lidar_and_map.launch.py (lidar + slam_toolbox)
- lucia_nav's navigation.launch.py (Nav2 + the mission node: explore, return
  home, go to a spot), unless use_nav:=false

Usage:
    ros2 launch lucia_bringup mapping.launch.py
    ros2 launch lucia_bringup mapping.launch.py use_imu:=false   # wheels only, as before
    ros2 launch lucia_bringup mapping.launch.py use_nav:=false   # mapping only, no Nav2
    ros2 launch lucia_bringup mapping.launch.py use_tof:=true    # ToF safety stop on
      (needs lucia_tof's node running, or forward driving stays blocked)
Normally started from the web page (lucia_web). From a terminal instead:
    ros2 topic pub --once /mission/command std_msgs/String "data: explore"
or drive by hand (test tool):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5

use_imu:=false falls back to the first-map setup: roomba_bridge publishes
odom->base_link itself and the IMU/EKF aren't started. Useful for comparing
maps with and without the IMU, or if the IMU isn't plugged in.

The IMU transform is identity: mount the board flat, chip side up. Which way it
points doesn't matter - only heading and turn rate are used, and the EKF treats
its heading at startup as 0 (imu0_relative in ekf.yaml).

settings_file:=<params.yaml>: the web page's settings menu (lucia_web
settings.py), as a ROS 2 parameters file. Every node that has settings there
gets it after its own (roomba_bridge, safety_gate, and through
navigation.launch.py the mission node and Nav2), so it overrides them. Default:
config/no_settings.yaml, which changes nothing.

Runs on the Pi: mapping since 2026-10-16, with Nav2 + Explore since 2026-10-17.
use_tof:=true is not yet tried (the ToF sensors aren't mounted).
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
    nav_launch = os.path.join(
        get_package_share_directory('lucia_nav'), 'launch', 'navigation.launch.py')

    use_imu = LaunchConfiguration('use_imu')
    with_imu = IfCondition(use_imu)
    settings = LaunchConfiguration('settings_file')     # last in each list: overrides the rest

    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Fuse the IMU heading via robot_localization'),
        DeclareLaunchArgument('use_nav', default_value='true',
                              description='Start Nav2 + the mission node (explore, return home)'),
        DeclareLaunchArgument('use_tof', default_value='false',
                              description='ToF safety stop: block forward driving while tof/near is true'),
        DeclareLaunchArgument('roomba_port', default_value='/dev/roomba'),
        DeclareLaunchArgument('imu_port', default_value='/dev/imu'),
        DeclareLaunchArgument('settings_file', default_value=os.path.join(bringup_config, 'no_settings.yaml'),
                              description="Parameters file from the web page's settings menu"),

        Node(
            package='lucia_control',
            executable='roomba_bridge',
            name='roomba_bridge',
            parameters=[{
                'port': LaunchConfiguration('roomba_port'),
                'publish_tf': NotSubstitution(use_imu),   # the EKF publishes it instead
            }, settings],
            remappings=[('cmd_vel', 'cmd_vel_safe')],      # every drive command goes through safety_gate
            output='screen',
        ),

        Node(
            package='lucia_bringup',
            executable='safety_gate',
            name='safety_gate',
            parameters=[{'use_tof': LaunchConfiguration('use_tof')}, settings],
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

        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch),
                                 launch_arguments={'settings_file': settings}.items(),
                                 condition=IfCondition(LaunchConfiguration('use_nav'))),
    ])
