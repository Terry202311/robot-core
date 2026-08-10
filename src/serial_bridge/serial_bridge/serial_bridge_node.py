#!/usr/bin/env python3

import threading
import time

import serial

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist

from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String



class SerialBridgeNode(Node):

    """
    ORP-MEGA Serial Bridge

    ROS2 <----> Arduino Mega


    ROS2 -> Mega

        /cmd_vel

        CMD,vx,vy,wz


    Mega -> ROS2

        ENC,lf,rf,lb,rb

        /wheel_encoder_counts

    """



    def __init__(self):

        super().__init__(
            "serial_bridge"
        )



        # ==========================
        # Parameters
        # ==========================


        self.declare_parameter(
            "port",
            "/dev/ttyACM0"
        )


        self.declare_parameter(
            "baudrate",
            115200
        )


        self.declare_parameter(
            "timeout",
            0.1
        )


        self.port = (
            self.get_parameter(
                "port"
            ).value
        )


        self.baudrate = int(
            self.get_parameter(
                "baudrate"
            ).value
        )


        timeout = float(
            self.get_parameter(
                "timeout"
            ).value
        )



        # ==========================
        # Serial
        # ==========================


        try:

            self.serial_port = serial.Serial(

                self.port,

                self.baudrate,

                timeout=timeout

            )


            self.get_logger().info(
                f"Serial connected: {self.port}"
            )


        except Exception as e:


            self.get_logger().error(
                f"Serial open failed: {e}"
            )


            self.serial_port = None



        # ==========================
        # ROS interface
        # ==========================



        self.cmd_sub = self.create_subscription(

            Twist,

            "/cmd_vel",

            self.cmd_vel_callback,

            10

        )



        self.encoder_pub = self.create_publisher(

            Int64MultiArray,

            "/wheel_encoder_counts",

            10

        )


        # Debug

        self.raw_pub = self.create_publisher(

            String,

            "/serial_raw",

            10

        )



        # ==========================
        # Serial read thread
        # ==========================


        self.running = True


        self.thread = threading.Thread(

            target=self.read_serial,

            daemon=True

        )


        self.thread.start()



        self.get_logger().info(

            "ORP-MEGA serial bridge started"

        )




    # =================================================
    # ROS2 cmd_vel -> Mega
    # =================================================


    def cmd_vel_callback(
        self,
        msg: Twist
    ):


        if self.serial_port is None:

            return



        vx = msg.linear.x

        vy = msg.linear.y

        wz = msg.angular.z



        command = (

            f"CMD,"

            f"{vx:.3f},"

            f"{vy:.3f},"

            f"{wz:.3f}\n"

        )



        try:


            self.serial_port.write(

                command.encode("utf-8")

            )


        except Exception as e:


            self.get_logger().warning(

                f"Serial write error: {e}"

            )





    # =================================================
    # Mega -> ROS2
    # =================================================


    def read_serial(self):


        while self.running:


            if self.serial_port is None:


                time.sleep(1)

                continue



            try:


                line = (

                    self.serial_port.readline()

                    .decode(
                        "utf-8",
                        errors="ignore"
                    )

                    .strip()

                )



                if not line:

                    continue



                # debug

                raw = String()

                raw.data = line

                self.raw_pub.publish(raw)



                if line.startswith(
                    "ENC,"
                ):

                    self.parse_encoder(
                        line
                    )



            except Exception as e:


                self.get_logger().warning(

                    f"Serial read error: {e}"

                )


                time.sleep(0.1)





    # =================================================
    # Parse ENC
    # =================================================


    def parse_encoder(
        self,
        line
    ):


        try:


            data = line.split(",")



            if len(data) != 5:

                return



            msg = Int64MultiArray()



            msg.data = [

                int(data[1]),

                int(data[2]),

                int(data[3]),

                int(data[4])

            ]



            self.encoder_pub.publish(
                msg
            )



        except Exception as e:


            self.get_logger().warning(

                f"Encoder parse error: {e}"

            )





    # =================================================
    # Shutdown
    # =================================================


    def destroy_node(self):


        self.running = False



        if self.serial_port:


            self.serial_port.close()



        super().destroy_node()






def main(args=None):


    rclpy.init(
        args=args
    )


    node = SerialBridgeNode()



    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass



    finally:


        node.destroy_node()


        rclpy.shutdown()





if __name__ == "__main__":

    main()