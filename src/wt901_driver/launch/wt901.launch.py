from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('wt901_driver'),
        'config',
        'wt901.yaml'
    )

    return LaunchDescription([
        Node(
            package='wt901_driver',
            executable='wt901_node',
            name='wt901_node',
            output='screen',
            parameters=[config_file],
        )
    ])
