import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info(f'Connected to Arduino: {port}, {baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Failed to open serial port {port}: {e}')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        command = f'CMD,{vx:.3f},{vy:.3f},{wz:.3f}\n'

        if self.ser and self.ser.is_open:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f'SEND: {command.strip()}')
        else:
            self.get_logger().warn(f'Serial not open. Drop: {command.strip()}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
