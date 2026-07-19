#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('robot_description')
    )

    default_model_path = (
        package_share / 'urdf' / 'robot.urdf.xacro'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=str(default_model_path),
        description='Robot Xacro file',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('model'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
    ])
