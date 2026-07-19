#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory('robot_description')
    )

    default_model_path = (
        package_share / 'urdf' / 'robot.urdf.xacro'
    )

    model_argument = DeclareLaunchArgument(
        'model',
        default_value=str(default_model_path),
        description='Absolute path to the robot Xacro model',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('model'),
        ]),
        value_type=str,
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
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
        model_argument,
        joint_state_publisher,
        robot_state_publisher,
    ])
