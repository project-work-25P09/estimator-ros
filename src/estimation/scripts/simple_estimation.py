#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, MagneticField
from estimation.msg import Estimation
import numpy as np
import math
import os
import time
import yaml
from estimation_pkg.ekf import InertialEKF, quaternion_to_euler


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")

        self.est_pub = self.create_publisher(Estimation, "/estimation", 10)
        self.create_subscription(Point, "/optical", self.optical_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 200)
        self.create_subscription(MagneticField, "/imu/mag", self.imu_mag_callback, 200)

        self.opt_int_x = 0.0
        self.opt_int_y = 0.0

        self.latest_imu = None

        self.config_calibration_fp = "./config/calibration.yml"
        self.config_estimation_fp = "./config/est_params.yml"

        with open(self.config_calibration_fp, "r") as f:
            calib = yaml.safe_load(f)
        opt_cfg = calib.get("optical", {})
        self.optical_x_to_m = opt_cfg.get("x_to_m", 1.0)
        self.optical_y_to_m = opt_cfg.get("y_to_m", 1.0)

        with open(self.config_estimation_fp, "r") as f:
            est_cfg = yaml.safe_load(f)

        dt = est_cfg.get("dt", 1.0 / 200.0)
        Q = np.array(est_cfg.get("Q", np.eye(15).tolist()))
        R_imu = np.array(est_cfg.get("R_imu", np.eye(3).tolist()))
        R_opt = np.array(est_cfg.get("R_opt", np.eye(2).tolist()))
        m_ref = np.array(est_cfg.get("m_ref", [0.0, 0.0, 0.0]))

        self.ekf = InertialEKF(dt, Q, R_imu, R_opt, m_ref)
        self.get_logger().info("EstimatorNode started.")

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

        self.latest_imu = imu_msg

        self.ekf.predict(a, w, o)
        self.publish_estimation()

    def imu_mag_callback(self, mag_msg: MagneticField):
        pass

    def optical_callback(self, opt_msg: Point):
        self.opt_int_x += opt_msg.x
        self.opt_int_y += opt_msg.y
        flow_x = opt_msg.x * self.optical_x_to_m
        flow_y = opt_msg.y * self.optical_y_to_m

        self.ekf.update_optical(flow_x, flow_y)

        self.publish_estimation()

    def publish_estimation(self):
        est = Estimation()
        est.stamp = self.get_clock().now().to_msg()

        # Get the latest position
        est.x, est.y, est.z = self.ekf.p.tolist()

        # Use quaternion to euler conversion for roll, pitch, yaw
        roll, pitch, yaw = quaternion_to_euler(self.ekf.q)
        est.roll, est.pitch, est.yaw = roll, pitch, yaw

        # Use raw accelerometer data from the latest IMU message if available
        if self.latest_imu is not None:
            est.acc_x = self.latest_imu.linear_acceleration.x
            est.acc_y = self.latest_imu.linear_acceleration.y
            est.acc_z = self.latest_imu.linear_acceleration.z
            est.acc_yaw = self.latest_imu.angular_velocity.x
            est.acc_pitch = self.latest_imu.angular_velocity.y
            est.acc_roll = self.latest_imu.angular_velocity.z
        else:
            est.acc_x, est.acc_y, est.acc_z = 0.0, 0.0, 0.0
            est.acc_yaw, est.acc_pitch, est.acc_roll = 0.0, 0.0, 0.0

        # Use raw magnetometer data from the latest IMU message if available
        est.mag_x, est.mag_y, est.mag_z = 0.0, 0.0, 0.0
        est.mag_strength = 0.0

        # optical‐flow diagnostics
        est.mouse_integrated_x = self.opt_int_x
        est.mouse_integrated_y = self.opt_int_y

        self.est_pub.publish(est)


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
