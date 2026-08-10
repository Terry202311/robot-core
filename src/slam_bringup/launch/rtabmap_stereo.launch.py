from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stereo_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_bringup'),
                'launch',
                'stereo_proc.launch.py'
            )
        )
    )

    return LaunchDescription([

        stereo_proc,
        Node(
            package='rtabmap_sync',
            executable='stereo_sync',
            name='stereo_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'queue_size': 30
            }],
            remappings=[
                ('left/image_rect',
                 '/left/image_rect'),

                ('right/image_rect',
                 '/right/image_rect'),

                ('left/camera_info',
                 '/left/camera_info'),

                ('right/camera_info',
                 '/right/camera_info')
            ]
        ),

        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[
                {
                  'Vis/CorNNDR': '0.9',
                  'Vis/MinInliers': '5',
                  'Vis/MaxFeatures': '1000',

                  'frame_id': 'base_link',
                  'odom_frame_id': 'visual_odom',
                  'publish_tf': False,
                  'wait_imu_to_init': False,

                  'approx_sync': True,
                  'queue_size': 30,
                  'approx_sync_max_interval': 0.05,
                }
            ],
            remappings=[
                ('odom', '/visual_odom'),
                ('imu', '/imu/data')
            ]
        )

    ])