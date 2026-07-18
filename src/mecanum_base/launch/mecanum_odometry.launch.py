from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory('mecanum_base')
    )

    config_file = (
        package_share
        / 'config'
        / 'mecanum_odometry.yaml'
    )

    return LaunchDescription([
        Node(
            package='mecanum_base',
            executable='mecanum_odometry_node',
            name='mecanum_odometry',
            output='screen',
            parameters=[str(config_file)],
        ),
    ])
