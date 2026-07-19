#!/usr/bin/env python3

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import serial


class WT901Node(Node):

    def __init__(self):
        super().__init__('wt901_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic_name', '/imu/data_raw')

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        topic_name = self.get_parameter('topic_name').value

        self.publisher = self.create_publisher(Imu, topic_name, 20)

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.1
        )

        self.buffer = bytearray()

        self.acceleration = None
        self.angular_velocity = None

        self.new_acceleration = False
        self.new_angular_velocity = False

        self.running = True
        self.read_thread = threading.Thread(
            target=self.read_serial,
            daemon=True
        )
        self.read_thread.start()

        self.get_logger().info(
            f'WT901 started: port={self.port}, '
            f'baudrate={self.baudrate}, topic={topic_name}'
        )

    def read_serial(self):
        while self.running and rclpy.ok():
            try:
                data = self.serial_port.read(256)

                if data:
                    self.buffer.extend(data)
                    self.parse_buffer()

            except serial.SerialException as exc:
                self.get_logger().error(f'Serial error: {exc}')
                break

    def parse_buffer(self):
        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue

            frame = self.buffer[:11]

            checksum = sum(frame[:10]) & 0xFF

            if checksum != frame[10]:
                del self.buffer[0]
                continue

            frame_type = frame[1]

            if frame_type == 0x51:
                self.parse_acceleration(frame)

            elif frame_type == 0x52:
                self.parse_angular_velocity(frame)

            del self.buffer[:11]

    @staticmethod
    def int16(low_byte, high_byte):
        value = low_byte | (high_byte << 8)

        if value >= 32768:
            value -= 65536

        return value

    def parse_acceleration(self, frame):
        raw_ax = self.int16(frame[2], frame[3])
        raw_ay = self.int16(frame[4], frame[5])
        raw_az = self.int16(frame[6], frame[7])

        gravity = 9.80665

        ax = raw_ax / 32768.0 * 16.0 * gravity
        ay = raw_ay / 32768.0 * 16.0 * gravity
        az = raw_az / 32768.0 * 16.0 * gravity

        self.acceleration = (ax, ay, az)
        self.new_acceleration = True

        self.publish_imu()

    def parse_angular_velocity(self, frame):
        raw_gx = self.int16(frame[2], frame[3])
        raw_gy = self.int16(frame[4], frame[5])
        raw_gz = self.int16(frame[6], frame[7])

        gx_deg = raw_gx / 32768.0 * 2000.0
        gy_deg = raw_gy / 32768.0 * 2000.0
        gz_deg = raw_gz / 32768.0 * 2000.0

        gx = math.radians(gx_deg)
        gy = math.radians(gy_deg)
        gz = math.radians(gz_deg)

        self.angular_velocity = (gx, gy, gz)
        self.new_angular_velocity = True

        self.publish_imu()

    def publish_imu(self):
        if self.acceleration is None:
            return

        if self.angular_velocity is None:
            return

        if not self.new_acceleration:
            return

        if not self.new_angular_velocity:
            return

        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # 当前 WT901 没有输出姿态帧 0x53，
        # 因此 orientation 数据标记为无效。
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]

        msg.linear_acceleration.x = self.acceleration[0]
        msg.linear_acceleration.y = self.acceleration[1]
        msg.linear_acceleration.z = self.acceleration[2]

        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        msg.linear_acceleration_covariance = [
            0.10, 0.0, 0.0,
            0.0, 0.10, 0.0,
            0.0, 0.0, 0.10
        ]

        self.publisher.publish(msg)

        self.new_acceleration = False
        self.new_angular_velocity = False

    def destroy_node(self):
        self.running = False

        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)

        if self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WT901Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
