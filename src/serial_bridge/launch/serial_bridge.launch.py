from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="serial_bridge",
            executable="serial_bridge_node",
            name="serial_bridge",
            output="screen",

            parameters=[{

                "port":
                "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_850363135303511101D1-if00",

                "baudrate":
                115200

            }]
        )

    ])
