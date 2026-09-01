from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',

            parameters=[{
                'frame_id': 'base_link',

                'subscribe_stereo': True,
                'subscribe_odom_info': False,

                'approx_sync': True,
                'odom_sensor_sync': True,

                'topic_queue_size': 50,
                'sync_queue_size': 50,

                'RGBD/LinearUpdate': '0.05',
                'RGBD/AngularUpdate': '0.05',

                'Rtabmap/DetectionRate': '1.0',

                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'true',
                'Mem/RehearsalSimilarity': '0.6',

                'database_path':
                    '/mnt/orp_data/OpenRobotPlatform/slam/databases/orp_manual_stereo.db',

                'RGBD/NeighborLinkRefining': 'false',
                'RGBD/LoopClosureReextractFeatures': 'false',

                'Grid/FromDepth': 'true'
            }],

            remappings=[
                ('left/image_rect',
                 '/left/image_rect'),

                ('right/image_rect',
                 '/right/image_rect'),

                ('left/camera_info',
                 '/left/camera_info'),

                ('right/camera_info',
                 '/right/camera_info'),

                ('odom',
                 '/visual_odom'),

                ('odom_info',
                 '/odom_info'),
            ]
        )

    ])