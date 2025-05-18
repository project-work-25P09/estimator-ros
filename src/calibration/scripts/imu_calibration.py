#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class IMUCalibration(Node):
    def __init__(self):
        super().__init__("imu_calibration_node")
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 200)

    def imu_callback(self, imu_msg: Imu):
        a = np.array(
            [
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z,
            ]
        )
        w = np.array(
            [
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z,
            ]
        )
        o = np.array(
            [
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w,
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    node = IMUCalibration()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
