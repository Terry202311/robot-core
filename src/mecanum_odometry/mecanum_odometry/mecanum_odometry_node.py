#!/usr/bin/env python3

import math
from typing import Optional, List


import rclpy
from rclpy.node import Node


from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray


from nav_msgs.msg import Odometry

from geometry_msgs.msg import (
    Quaternion,
    TransformStamped
)


from tf2_ros import TransformBroadcaster



class MecanumOdometryNode(Node):

    """
    ORP-MEGA Mecanum Wheel Odometry

    Input:
        /wheel_encoder_counts

        [LF, RF, LB, RB]


    Output:

        /odom

        TF:
            odom -> base_link

    """


    def __init__(self):

        super().__init__(
            "mecanum_odometry"
        )


        # =========================
        # Parameters
        # =========================


        self.declare_parameter(
            "wheel_radius",
            0.04
        )


        self.declare_parameter(
            "encoder_cpr",
            1248.0
        )


        self.declare_parameter(
            "wheel_base",
            0.165
        )


        self.declare_parameter(
            "wheel_track",
            0.130
        )


        self.declare_parameter(
            "encoder_topic",
            "/wheel_encoder_counts"
        )


        self.declare_parameter(
            "odom_topic",
            "/odom"
        )


        self.declare_parameter(
            "odom_frame",
            "odom"
        )


        self.declare_parameter(
            "base_frame",
            "base_link"
        )


        self.declare_parameter(
            "publish_tf",
            True
        )


        self.declare_parameter(
            "lateral_sign",
            1.0
        )


        self.declare_parameter(
            "yaw_sign",
            1.0
        )



        # read parameters


        self.wheel_radius = float(
            self.get_parameter(
                "wheel_radius"
            ).value
        )


        self.encoder_cpr = float(
            self.get_parameter(
                "encoder_cpr"
            ).value
        )


        self.wheel_base = float(
            self.get_parameter(
                "wheel_base"
            ).value
        )


        self.wheel_track = float(
            self.get_parameter(
                "wheel_track"
            ).value
        )


        self.encoder_topic = (
            self.get_parameter(
                "encoder_topic"
            ).value
        )


        self.odom_topic = (
            self.get_parameter(
                "odom_topic"
            ).value
        )


        self.odom_frame = (
            self.get_parameter(
                "odom_frame"
            ).value
        )


        self.base_frame = (
            self.get_parameter(
                "base_frame"
            ).value
        )


        self.publish_tf = bool(
            self.get_parameter(
                "publish_tf"
            ).value
        )


        self.lateral_sign = float(
            self.get_parameter(
                "lateral_sign"
            ).value
        )


        self.yaw_sign = float(
            self.get_parameter(
                "yaw_sign"
            ).value
        )


        # =========================
        # Geometry
        # =========================


        self.rotation_radius = (
            self.wheel_base
            +
            self.wheel_track
        ) / 2.0



        self.radians_per_tick = (
            2.0 *
            math.pi /
            self.encoder_cpr
        )


        # =========================
        # State
        # =========================


        self.x = 0.0

        self.y = 0.0

        self.yaw = 0.0



        self.last_encoder: Optional[List[int]] = None

        self.last_time = None



        # =========================
        # ROS interface
        # =========================


        self.encoder_sub = self.create_subscription(
            Int64MultiArray,
            self.encoder_topic,
            self.encoder_callback,
            20
        )


        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            20
        )


        self.speed_pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_speeds",
            20
        )


        self.tf_broadcaster = (
            TransformBroadcaster(self)
        )


        self.get_logger().info(
            "ORP-MEGA mecanum odometry started"
        )



    # ==================================================
    # Encoder callback
    # ==================================================


    def encoder_callback(
        self,
        msg
    ):


        if len(msg.data) != 4:

            self.get_logger().warning(
                "Encoder data length error"
            )

            return



        current = [
            int(x)
            for x in msg.data
        ]



        now = self.get_clock().now()



        if self.last_encoder is None:

            self.last_encoder = current

            self.last_time = now

            return



        dt = (
            now.nanoseconds
            -
            self.last_time.nanoseconds
        ) / 1e9



        if dt <= 0:

            return



        delta = [

            current[i]
            -
            self.last_encoder[i]

            for i in range(4)

        ]



        self.last_encoder = current

        self.last_time = now



        # ticks -> wheel angular velocity


        wheel_velocity = [

            d *
            self.radians_per_tick *
            self.wheel_radius /
            dt

            for d in delta

        ]



        lf, rf, lb, rb = wheel_velocity



        self.publish_wheel_speed(
            wheel_velocity
        )



        # ==========================
        # Mecanum forward kinematics
        # ==========================


        vx = (

            lf
            +
            rf
            +
            lb
            +
            rb

        ) / 4.0



        vy = self.lateral_sign * (

            -lf
            +
            rf
            +
            lb
            -
            rb

        ) / 4.0



        wz = self.yaw_sign * (

            -lf
            +
            rf
            -
            lb
            +
            rb

        ) / (

            4.0 *
            self.rotation_radius

        )



        # integrate


        dx = (

            vx *
            math.cos(self.yaw)
            -
            vy *
            math.sin(self.yaw)

        ) * dt



        dy = (

            vx *
            math.sin(self.yaw)
            +
            vy *
            math.cos(self.yaw)

        ) * dt



        self.x += dx

        self.y += dy

        self.yaw += wz * dt



        self.publish_odom(
            vx,
            vy,
            wz
        )



    # ==================================================
    # Publish wheel speed
    # ==================================================


    def publish_wheel_speed(
        self,
        speed
    ):

        msg = Float64MultiArray()

        msg.data = speed

        self.speed_pub.publish(msg)



    # ==================================================
    # Publish odom
    # ==================================================


    def publish_odom(
        self,
        vx,
        vy,
        wz
    ):


        msg = Odometry()


        stamp = self.get_clock().now().to_msg()



        msg.header.stamp = stamp

        msg.header.frame_id = self.odom_frame

        msg.child_frame_id = self.base_frame



        msg.pose.pose.position.x = self.x

        msg.pose.pose.position.y = self.y



        q = self.yaw_to_quaternion(
            self.yaw
        )


        msg.pose.pose.orientation = q



        msg.twist.twist.linear.x = vx

        msg.twist.twist.linear.y = vy

        msg.twist.twist.angular.z = wz



        # covariance

        msg.pose.covariance[0] = 0.02

        msg.pose.covariance[7] = 0.02

        msg.pose.covariance[35] = 0.05


        msg.twist.covariance[0] = 0.03

        msg.twist.covariance[7] = 0.03

        msg.twist.covariance[35] = 0.08



        self.odom_pub.publish(msg)



        if self.publish_tf:

            self.publish_tf_transform(
                stamp,
                q
            )



    # ==================================================
    # TF
    # ==================================================


    def publish_tf_transform(
        self,
        stamp,
        q
    ):


        t = TransformStamped()


        t.header.stamp = stamp

        t.header.frame_id = self.odom_frame

        t.child_frame_id = self.base_frame



        t.transform.translation.x = self.x

        t.transform.translation.y = self.y

        t.transform.translation.z = 0.0



        t.transform.rotation = q



        self.tf_broadcaster.sendTransform(t)



    # ==================================================
    # Quaternion
    # ==================================================


    def yaw_to_quaternion(
        self,
        yaw
    ):

        q = Quaternion()


        q.x = 0.0

        q.y = 0.0

        q.z = math.sin(
            yaw / 2.0
        )

        q.w = math.cos(
            yaw / 2.0
        )


        return q





def main(args=None):


    rclpy.init(
        args=args
    )


    node = MecanumOdometryNode()


    rclpy.spin(node)



    node.destroy_node()


    rclpy.shutdown()



if __name__ == "__main__":

    main()