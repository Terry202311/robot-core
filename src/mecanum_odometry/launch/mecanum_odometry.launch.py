#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_directory = get_package_share_directory(
        'mecanum_odometry'
    )

    parameter_file = os.path.join(
        package_directory,
        'config',
        'mecanum_odometry.yaml'
    )

    return LaunchDescription([
        Node(
            package='mecanum_odometry',
            executable='mecanum_odometry_node',
            name='mecanum_odometry',
            output='screen',
            parameters=[parameter_file]
        )
    ])
