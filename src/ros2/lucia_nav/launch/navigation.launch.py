"""
navigation.launch.py

Nav2 set up for the Roomba (config/nav2.yaml), plus the mission node that
gives it goals (explore / home / goto, see lucia_nav/mission.py).

Needs slam_toolbox running: it supplies /map and the map->odom transform Nav2
plans in. lucia_bringup's mapping.launch.py starts both together, which is the
normal way to run this.

Only the four Nav2 servers this robot uses are started, instead of
nav2_bringup's navigation_launch.py. That one also starts the docking server,
collision monitor, velocity smoother and others, which each need their own
settings, and if any of them fails to start the lifecycle manager doesn't
start the rest either.

Usage:
    ros2 launch lucia_nav navigation.launch.py

NOT YET RUN on the Pi.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

NAV2_NODES = [
    # (package, executable, node name)
    ('nav2_controller', 'controller_server', 'controller_server'),
    ('nav2_planner', 'planner_server', 'planner_server'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
]


def generate_launch_description():
    nav2_params = os.path.join(get_package_share_directory('lucia_nav'), 'config', 'nav2.yaml')

    nav2 = [
        Node(package=pkg, executable=exe, name=name, parameters=[nav2_params], output='screen')
        for pkg, exe, name in NAV2_NODES
    ]

    # Nav2's servers are lifecycle nodes: they sit idle until configured and
    # activated, in order. The lifecycle manager does that at startup.
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'autostart': True,
            'node_names': [name for _, _, name in NAV2_NODES],
            'bond_timeout': 4.0,
        }],
        output='screen',
    )

    mission = Node(
        package='lucia_nav',
        executable='mission',
        name='mission',
        output='screen',
    )

    return LaunchDescription(nav2 + [lifecycle, mission])
