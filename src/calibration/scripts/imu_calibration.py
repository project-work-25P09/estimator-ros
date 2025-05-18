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
        w_raw = np.array([imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z])
        a_raw = np.array([imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z])
        m_raw = np.array([imu_msg.magnetic_field.x,
                        imu_msg.magnetic_field.y,
                        imu_msg.magnetic_field.z])

        w_cal = w_raw - self.gyro_bias
        a_cal = (a_raw - self.accel_bias) * self.accel_scale
        m_cal = (m_raw - self.mag_bias)   * self.mag_scale

        pass

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
