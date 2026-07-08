import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from serial_bridge.protocol import build_cmd_vel_packet


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout = float(self.get_parameter('timeout').value)

        self.serial_port = None
        self.open_serial()

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('serial_bridge node started')

    def open_serial(self):
        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)
            self.get_logger().info(
                f'Connected to Arduino on {self.port} @ {self.baudrate}'
            )
        except Exception as exc:
            self.serial_port = None
            self.get_logger().error(
                f'Failed to open serial port {self.port}: {exc}'
            )

    def cmd_vel_callback(self, msg: Twist):
        packet = build_cmd_vel_packet(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        )

        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn(f'Serial not open, drop packet: {packet.strip()}')
            return

        try:
            self.serial_port.write(packet.encode('utf-8'))
            self.get_logger().info(f'SEND: {packet.strip()}')
        except Exception as exc:
            self.get_logger().error(f'Failed to write serial packet: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
