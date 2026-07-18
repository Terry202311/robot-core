#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotionTest(Node):
    """Publish a low-speed Twist command for a limited duration."""

    def __init__(self) -> None:
        super().__init__('motion_test')

        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('wz', 0.0)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('publish_rate', 10.0)

        self.vx = float(self.get_parameter('vx').value)
        self.vy = float(self.get_parameter('vy').value)
        self.wz = float(self.get_parameter('wz').value)
        self.duration = float(self.get_parameter('duration').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        if self.duration <= 0.0:
            raise ValueError('duration must be greater than zero')

        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.command = Twist()
        self.command.linear.x = self.vx
        self.command.linear.y = self.vy
        self.command.angular.z = self.wz

        self.start_time = time.monotonic()
        self.finished = False

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback,
        )

        self.get_logger().info(
            f'Starting motion test: '
            f'vx={self.vx:.3f}, vy={self.vy:.3f}, '
            f'wz={self.wz:.3f}, duration={self.duration:.1f}s'
        )

    def timer_callback(self) -> None:
        elapsed = time.monotonic() - self.start_time

        if elapsed < self.duration:
            self.publisher.publish(self.command)
            return

        self.publish_stop()

        if not self.finished:
            self.finished = True
            self.get_logger().info('Motion completed. Robot stopped.')

    def publish_stop(self) -> None:
        stop = Twist()

        # Send several stop frames for reliability.
        for _ in range(5):
            self.publisher.publish(stop)
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = MotionTest()

        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning('Interrupted. Sending stop command.')

    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'Motion test failed: {exc}')
        else:
            print(f'Motion test failed: {exc}')

    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
