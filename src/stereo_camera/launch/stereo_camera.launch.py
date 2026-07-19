from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("stereo_camera")
    )

    default_config = package_share / "config" / "stereo_camera.yaml"

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="Stereo camera parameter YAML file",
            ),
            Node(
                package="stereo_camera",
                executable="stereo_camera_node",
                name="stereo_camera_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
