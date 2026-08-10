from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # 左目去畸变
        Node(
            package='image_proc',
            executable='rectify_node',
            namespace='left',
            name='rectify_node',
            output='screen',
            remappings=[
                ('image', '/left/image_raw'),
                ('camera_info', '/left/camera_info'),
            ]
        ),

        # 右目去畸变
        Node(
            package='image_proc',
            executable='rectify_node',
            namespace='right',
            name='rectify_node',
            output='screen',
            remappings=[
                ('image', '/right/image_raw'),
                ('camera_info', '/right/camera_info'),
            ]
        ),


        # 双目视差
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            output='screen',

            parameters=[
                {
                    "disparity_range": 64,
                    "correlation_window_size": 11,
                    "queue_size": 20
                }
            ],

            remappings=[
                ('left/image_rect',
                 '/left/image_rect'),

                ('left/camera_info',
                 '/left/camera_info'),

                ('right/image_rect',
                 '/right/image_rect'),

                ('right/camera_info',
                 '/right/camera_info'),
            ]
        ),


        # 点云（后续RTAB-Map可不用，暂留）
   #     Node(
        #    package='stereo_image_proc',
         #   executable='point_cloud_node',
        #    name='point_cloud_node',
        #    output='screen'
     #   )

    ])
