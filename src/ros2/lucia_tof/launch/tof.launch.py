import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lucia_tof'), 'config', 'tof.yaml')
    return LaunchDescription([
        Node(
            package='lucia_tof',
            executable='tof_node',
            name='tof_node',
            parameters=[config],
            output='screen',
        ),
    ])
