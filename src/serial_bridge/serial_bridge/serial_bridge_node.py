import time
from typing import Optional

import serial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

from serial_bridge.protocol import (
    build_cmd_vel_packet,
    parse_encoder_packet,
)


class SerialBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.02)
        self.declare_parameter('read_period', 0.01)
        self.declare_parameter('reconnect_period', 2.0)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.read_period = float(self.get_parameter('read_period').value)
        self.reconnect_period = float(
            self.get_parameter('reconnect_period').value
        )

        self.serial_port: Optional[serial.Serial] = None
        self.last_reconnect_attempt = 0.0
        self.receive_buffer = ''

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.encoder_pub = self.create_publisher(
            Int64MultiArray,
            '/wheel_encoder_counts',
            10,
        )

        self.read_timer = self.create_timer(
            self.read_period,
            self.read_serial,
        )

        self.open_serial()

        self.get_logger().info('serial_bridge node started')

    def open_serial(self) -> None:
        """Open the configured serial port."""
        self.last_reconnect_attempt = time.monotonic()

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )

            time.sleep(2.0)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.get_logger().info(
                f'Connected to Arduino: '
                f'{self.port} @ {self.baudrate}'
            )

        except serial.SerialException as exc:
            self.serial_port = None
            self.get_logger().error(
                f'Failed to open serial port {self.port}: {exc}'
            )

    def ensure_serial_connected(self) -> bool:
        """Reconnect the serial port when disconnected."""
        if self.serial_port is not None and self.serial_port.is_open:
            return True

        now = time.monotonic()

        if now - self.last_reconnect_attempt >= self.reconnect_period:
            self.get_logger().warn(
                f'Trying to reconnect serial port {self.port}'
            )
            self.open_serial()

        return (
            self.serial_port is not None
            and self.serial_port.is_open
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Send ROS2 velocity commands to Arduino."""
        if not self.ensure_serial_connected():
            return

        packet = build_cmd_vel_packet(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )

        try:
            self.serial_port.write(packet.encode('utf-8'))
            self.get_logger().info(
                f'SEND: {packet.strip()}'
            )

        except serial.SerialException as exc:
            self.get_logger().error(
                f'Serial write failed: {exc}'
            )
            self.close_serial()

    def read_serial(self) -> None:
        """Read and parse packets returned by Arduino."""
        if not self.ensure_serial_connected():
            return

        try:
            available = self.serial_port.in_waiting

            if available <= 0:
                return

            raw_data = self.serial_port.read(available)
            text = raw_data.decode('utf-8', errors='ignore')

            self.receive_buffer += text

            while '\n' in self.receive_buffer:
                line, self.receive_buffer = self.receive_buffer.split(
                    '\n',
                    1,
                )

                line = line.strip()

                if not line:
                    continue

                self.handle_serial_line(line)

        except serial.SerialException as exc:
            self.get_logger().error(
                f'Serial read failed: {exc}'
            )
            self.close_serial()

    def handle_serial_line(self, line: str) -> None:
        """Handle a complete line received from Arduino."""
        encoder_values = parse_encoder_packet(line)

        if encoder_values is not None:
            message = Int64MultiArray()
            message.data = list(encoder_values)

            self.encoder_pub.publish(message)

            lf, rf, lb, rb = encoder_values

            self.get_logger().info(
                f'RECV ENC: '
                f'LF={lf} RF={rf} LB={lb} RB={rb}'
            )
            return

        self.get_logger().debug(
            f'Unhandled serial line: {line}'
        )

    def close_serial(self) -> None:
        """Safely close the serial port."""
        if self.serial_port is None:
            return

        try:
            if self.serial_port.is_open:
                self.serial_port.close()
        except serial.SerialException:
            pass

        self.serial_port = None

    def destroy_node(self) -> bool:
        self.close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SerialBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
