from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',

            parameters=[{

                'frame_id': 'base_link',

                'odom_frame_id': 'odom',

                'publish_tf': False,

                'subscribe_stereo': True,

                'wait_imu_to_init': False,

                'Vis/FeatureType': '6',

                'Vis/MaxFeatures': '500'

            }],

            remappings=[

                ('odom',
                 '/visual_odom'),

                ('left/image_rect',
                 '/left/image_rect'),

                ('right/image_rect',
                 '/right/image_rect'),

                ('left/camera_info',
                 '/left/camera_info'),

                ('right/camera_info',
                 '/right/camera_info'),

            ]
        )

    ])