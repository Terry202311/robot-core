from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    launches = []

    packages = [
        "robot_description",
        "stereo_camera",
        "wt901_driver",
        "serial_bridge",
        "mecanum_odometry",
    ]

    launch_files = [
        "robot_state.launch.py",
        "stereo_camera.launch.py",
        "wt901.launch.py",
        "serial_bridge.launch.py",
        "mecanum_odometry.launch.py",
    ]


    for pkg, file in zip(packages, launch_files):

        launch_path = os.path.join(
            get_package_share_directory(pkg),
            "launch",
            file
        )


        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_path
                )
            )
        )


    return LaunchDescription(launches)
