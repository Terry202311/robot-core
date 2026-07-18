#!/usr/bin/env python3

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int64MultiArray
from tf2_ros import TransformBroadcaster


class MecanumOdometryNode(Node):
    """Calculate mecanum-wheel odometry from cumulative encoder counts."""

    def __init__(self) -> None:
        super().__init__('mecanum_odometry')

        # ---------------- Parameters ----------------
        self.declare_parameter('wheel_radius', 0.040)
        self.declare_parameter('encoder_cpr', 1248.0)

        # Front-to-rear wheel-center distance
        self.declare_parameter('wheel_base', 0.165)

        # Left-to-right wheel-center distance
        self.declare_parameter('wheel_track', 0.130)

        self.declare_parameter(
            'encoder_topic',
            '/wheel_encoder_counts'
        )
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_speed_topic', '/wheel_speeds')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # Adjustable signs for mecanum roller layout.
        # Default assumes a conventional X-pattern mecanum chassis.
        self.declare_parameter('lateral_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.encoder_cpr = float(
            self.get_parameter('encoder_cpr').value
        )
        self.wheel_base = float(
            self.get_parameter('wheel_base').value
        )
        self.wheel_track = float(
            self.get_parameter('wheel_track').value
        )

        self.encoder_topic = str(
            self.get_parameter('encoder_topic').value
        )
        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self.wheel_speed_topic = str(
            self.get_parameter('wheel_speed_topic').value
        )

        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )
        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.publish_tf = bool(
            self.get_parameter('publish_tf').value
        )
        self.lateral_sign = float(
            self.get_parameter('lateral_sign').value
        )
        self.yaw_sign = float(
            self.get_parameter('yaw_sign').value
        )

        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be positive')

        if self.encoder_cpr <= 0.0:
            raise ValueError('encoder_cpr must be positive')

        if self.wheel_base <= 0.0 or self.wheel_track <= 0.0:
            raise ValueError(
                'wheel_base and wheel_track must be positive'
            )

        # Distance from robot center to wheel along x and y.
        self.half_length = self.wheel_base / 2.0
        self.half_width = self.wheel_track / 2.0
        self.rotation_radius = self.half_length + self.half_width

        self.radians_per_tick = (
            2.0 * math.pi / self.encoder_cpr
        )

        # ---------------- Odometry state ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.previous_counts: Optional[List[int]] = None
        self.previous_time_ns: Optional[int] = None

        # ---------------- ROS interfaces ----------------
        self.encoder_subscriber = self.create_subscription(
            Int64MultiArray,
            self.encoder_topic,
            self.encoder_callback,
            20
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            20
        )

        self.wheel_speed_publisher = self.create_publisher(
            Float64MultiArray,
            self.wheel_speed_topic,
            20
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Mecanum odometry node started')
        self.get_logger().info(
            f'wheel_radius={self.wheel_radius:.4f} m, '
            f'encoder_cpr={self.encoder_cpr:.1f}, '
            f'wheel_base={self.wheel_base:.4f} m, '
            f'wheel_track={self.wheel_track:.4f} m'
        )
        self.get_logger().info(
            f'Subscribing to {self.encoder_topic}'
        )

    def encoder_callback(
        self,
        message: Int64MultiArray
    ) -> None:
        if len(message.data) != 4:
            self.get_logger().warning(
                'Expected four encoder counts in order '
                '[LF, RF, LB, RB]'
            )
            return

        current_counts = [int(value) for value in message.data]
        current_time = self.get_clock().now()
        current_time_ns = current_time.nanoseconds

        if (
            self.previous_counts is None
            or self.previous_time_ns is None
        ):
            self.previous_counts = current_counts
            self.previous_time_ns = current_time_ns
            return

        dt = (
            current_time_ns - self.previous_time_ns
        ) / 1_000_000_000.0

        if dt <= 0.0 or dt > 1.0:
            self.get_logger().warning(
                f'Invalid encoder time interval: {dt:.6f} s'
            )
            self.previous_counts = current_counts
            self.previous_time_ns = current_time_ns
            return

        delta_ticks = [
            current_counts[index] - self.previous_counts[index]
            for index in range(4)
        ]

        self.previous_counts = current_counts
        self.previous_time_ns = current_time_ns

        # Wheel order:
        # 0 = left front
        # 1 = right front
        # 2 = left rear
        # 3 = right rear
        wheel_angular_velocities = [
            delta * self.radians_per_tick / dt
            for delta in delta_ticks
        ]

        (
            omega_lf,
            omega_rf,
            omega_lb,
            omega_rb
        ) = wheel_angular_velocities

        wheel_linear_velocities = [
            omega * self.wheel_radius
            for omega in wheel_angular_velocities
        ]

        (
            velocity_lf,
            velocity_rf,
            velocity_lb,
            velocity_rb
        ) = wheel_linear_velocities

        # Conventional X-pattern mecanum inverse kinematics.
        velocity_x = (
            velocity_lf
            + velocity_rf
            + velocity_lb
            + velocity_rb
        ) / 4.0

        velocity_y = self.lateral_sign * (
            -velocity_lf
            + velocity_rf
            + velocity_lb
            - velocity_rb
        ) / 4.0

        angular_z = self.yaw_sign * (
            -velocity_lf
            + velocity_rf
            - velocity_lb
            + velocity_rb
        ) / (4.0 * self.rotation_radius)

        # Integrate body-frame velocity into odom coordinates.
        delta_x = (
            velocity_x * math.cos(self.yaw)
            - velocity_y * math.sin(self.yaw)
        ) * dt

        delta_y = (
            velocity_x * math.sin(self.yaw)
            + velocity_y * math.cos(self.yaw)
        ) * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw = self.normalize_angle(
            self.yaw + angular_z * dt
        )

        self.publish_wheel_speeds(
            wheel_angular_velocities
        )

        self.publish_odometry(
            current_time.to_msg(),
            velocity_x,
            velocity_y,
            angular_z
        )

    def publish_wheel_speeds(
        self,
        wheel_angular_velocities: List[float]
    ) -> None:
        message = Float64MultiArray()
        message.data = wheel_angular_velocities
        self.wheel_speed_publisher.publish(message)

    def publish_odometry(
        self,
        stamp,
        velocity_x: float,
        velocity_y: float,
        angular_z: float
    ) -> None:
        quaternion = self.yaw_to_quaternion(self.yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame

        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation = quaternion

        odometry.twist.twist.linear.x = velocity_x
        odometry.twist.twist.linear.y = velocity_y
        odometry.twist.twist.angular.z = angular_z

        # Initial covariance values; calibrate later.
        odometry.pose.covariance[0] = 0.02
        odometry.pose.covariance[7] = 0.02
        odometry.pose.covariance[35] = 0.05

        odometry.twist.covariance[0] = 0.03
        odometry.twist.covariance[7] = 0.03
        odometry.twist.covariance[35] = 0.08

        self.odom_publisher.publish(odometry)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MecanumOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
