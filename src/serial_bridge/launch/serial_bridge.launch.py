from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('serial_bridge')
    config_file = os.path.join(pkg_share, 'config', 'serial_bridge.yaml')

    return LaunchDescription([
        Node(
            package='serial_bridge',
            executable='serial_bridge_node',
            name='serial_bridge',
            output='screen',
            parameters=[config_file]
        )
    ])
