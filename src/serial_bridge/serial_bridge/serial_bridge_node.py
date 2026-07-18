import time
from typing import Optional
from std_msgs.msg import Int32

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

        # 单轮测试接口
        self.wheel_test_sub = self.create_subscription(
            Int32,
            '/wheel_test',
            self.wheel_test_callback,
            10
        )

        # 单轮测试状态
        self.wheel_test_active = False
        self.wheel_test_command = 0
        self.wheel_test_last_time = 0.0

        # 最长允许单轮连续运行时间
        self.wheel_test_timeout = 10.0

        # 每 0.1 秒检查测试超时
        self.wheel_test_watchdog_timer = self.create_timer(
            0.1,
            self.wheel_test_watchdog
        )

        self.get_logger().info(
            'Wheel test enabled: 0=STOP, 1=LF, 2=RF, 3=LB, 4=RB'
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

            # 串口连接后先确保所有电机停止
            self.serial_port.write(b'S\n')
            self.serial_port.flush()

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
            self.get_logger().warning(
                f'Trying to reconnect serial port {self.port}'
            )
            self.open_serial()

        return (
            self.serial_port is not None
            and self.serial_port.is_open
        )

    def send_serial_command(self, command: str) -> bool:
        """安全地向 Arduino 发送一条命令。"""
        if not self.ensure_serial_connected():
            self.get_logger().error(
                f'Cannot send command {command!r}: serial port disconnected'
            )
            return False

        try:
            if not command.endswith('\n'):
                command += '\n'

            self.serial_port.write(command.encode('utf-8'))
            self.serial_port.flush()
            return True

        except serial.SerialException as exc:
            self.get_logger().error(
                f'Failed to send serial command {command!r}: {exc}'
            )
            self.close_serial()
            return False

    def wheel_test_callback(self, msg: Int32) -> None:
        """
        /wheel_test:
          0 = stop
          1 = LF
          2 = RF
          3 = LB
          4 = RB
        """
        command_map = {
            0: 'S',
            1: '1',
            2: '2',
            3: '3',
            4: '4',
        }

        wheel_names = {
            0: 'STOP',
            1: 'LF',
            2: 'RF',
            3: 'LB',
            4: 'RB',
        }

        command = command_map.get(msg.data)

        if command is None:
            self.get_logger().warning(
                f'Invalid /wheel_test value: {msg.data}; '
                'valid values are 0, 1, 2, 3, 4'
            )
            return

        if msg.data == 0:
            self.send_serial_command('S')
            self.wheel_test_active = False
            self.wheel_test_command = 0
            self.get_logger().info('Wheel test stopped')
            return

        if self.send_serial_command(command):
            self.wheel_test_active = True
            self.wheel_test_command = msg.data
            self.wheel_test_last_time = time.monotonic()

            self.get_logger().warning(
                f'Wheel test started: {wheel_names[msg.data]}; '
                f'auto-stop after {self.wheel_test_timeout:.1f} seconds'
            )

    def wheel_test_watchdog(self) -> None:
        """单轮测试超时自动停车。"""
        if not self.wheel_test_active:
            return

        elapsed = time.monotonic() - self.wheel_test_last_time

        if elapsed >= self.wheel_test_timeout:
            self.send_serial_command('S')
            self.wheel_test_active = False
            self.wheel_test_command = 0

            self.get_logger().warning(
                'Wheel test timed out; motor stopped automatically'
            )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Send ROS2 velocity commands to Arduino."""
        if not self.ensure_serial_connected():
            return

        is_zero_command = (
                abs(msg.linear.x) < 1e-6
                and abs(msg.linear.y) < 1e-6
                and abs(msg.angular.z) < 1e-6
        )

        if self.wheel_test_active:
            if is_zero_command:
                # 避免周期性的零速度消息打断单轮测试
                return

            # 非零运动命令优先，退出测试模式
            self.send_serial_command('S')
            self.wheel_test_active = False
            self.wheel_test_command = 0

            self.get_logger().warning(
                'Wheel test cancelled by non-zero /cmd_vel'
            )

        packet = build_cmd_vel_packet(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )

        try:
            self.serial_port.write(packet.encode('utf-8'))
            self.serial_port.flush()

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
        if self.serial_port is not None and self.serial_port.is_open:
            self.send_serial_command('S')
            time.sleep(0.05)

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
