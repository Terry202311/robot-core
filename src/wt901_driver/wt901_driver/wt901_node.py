#!/usr/bin/env python3


import serial
import math


import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Imu


from .protocol import (
    check_frame,
    parse_acc,
    parse_gyro,
)



class WT901Node(Node):


    def __init__(self):

        super().__init__(
            "wt901_driver"
        )


        self.declare_parameter(
            "port",
            "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
        )


        self.declare_parameter(
            "baudrate",
            9600
        )


        port = self.get_parameter(
            "port"
        ).value


        baud = self.get_parameter(
            "baudrate"
        ).value



        try:

            self.ser = serial.Serial(
                port,
                baud,
                timeout=0.1
            )


            self.get_logger().info(
                f"WT901 connected: {port}"
            )


        except Exception as e:

            self.get_logger().error(
                str(e)
            )

            self.ser=None



        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/data",
            20
        )


        self.acc = [0,0,0]

        self.gyro = [0,0,0]



        self.timer = self.create_timer(
            0.01,
            self.read_sensor
        )



    def read_sensor(self):


        if self.ser is None:

            return



        while self.ser.in_waiting >= 11:


            frame = self.ser.read(11)



            if not check_frame(frame):

                continue



            frame_type = frame[1]



            if frame_type == 0x51:


                self.acc = parse_acc(frame)



            elif frame_type == 0x52:


                self.gyro = parse_gyro(frame)



            self.publish_imu()



    def publish_imu(self):


        msg = Imu()



        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )


        msg.header.frame_id = (
            "imu_link"
        )



        # WT901 acceleration

        g = 9.80665


        msg.linear_acceleration.x = (
            self.acc[0]
            /
            32768.0
            *
            16
            *
            g
        )


        msg.linear_acceleration.y = (
            self.acc[1]
            /
            32768.0
            *
            16
            *
            g
        )


        msg.linear_acceleration.z = (
            self.acc[2]
            /
            32768.0
            *
            16
            *
            g
        )



        # gyro

        # ±2000 deg/s

        scale = (
            2000.0
            /
            32768.0
        )


        msg.angular_velocity.x = math.radians(
            self.gyro[0] * scale
        )


        msg.angular_velocity.y = math.radians(
            self.gyro[1] * scale
        )


        msg.angular_velocity.z = math.radians(
            self.gyro[2] * scale
        )

        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04



        # 当前没有55 53姿态帧

        msg.orientation_covariance[0] = -1



        self.imu_pub.publish(msg)





def main(args=None):


    rclpy.init(args=args)


    node = WT901Node()


    rclpy.spin(node)



    node.destroy_node()


    rclpy.shutdown()



if __name__ == "__main__":

    main()