#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, MagneticField
from estimation.msg import Estimation, Measurements
import numpy as np
import math
import os
import time
import yaml
from estimation_pkg.ekf import InertialEKF
import estimation_pkg.utils as utils


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")

        self.create_subscription(Point, "/optical", self.optical_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 200)
        self.create_subscription(MagneticField, "/imu/mag", self.imu_mag_callback, 200)

        self.est_pub = self.create_publisher(Estimation, "/estimation", 10)
        self.measurements = Measurements

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
        acceleration = np.array(
            [
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z,
            ]
        )
        angular_velocity = np.array(
            [
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z,
            ]
        )
        orientation = np.array(
            [
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w,
            ]
        )

        self.update_imu(acceleration, angular_velocity, orientation)
        self.publish_estimation()

    def imu_mag_callback(self, mag_msg: MagneticField):
        magnetic_field = np.array(mag_msg.magnetic_field)
        magnetic_field_strength = np.norm(magnetic_field)
        self.update_magnetic_field(magnetic_field, magnetic_field_strength)
        self.publish_estimation()

    def optical_callback(self, opt_msg: Point):
        flow_x = opt_msg.x * self.optical_x_to_m
        flow_y = opt_msg.y * self.optical_y_to_m
        self.update_optical(flow_x, flow_y)
        self.publish_estimation()

    def update_imu(self, acceleration, angular_velocity, orientation):
        self.measurements.acceleration = acceleration
        self.measurements.angular_velocity = angular_velocity
        (
            self.measurements.imu_est_yaw,
            self.measurements.imu_est_pitch,
            self.measurements.imu_est_roll,
        ) = utils.quaternion_to_euler(orientation)

    def update_magnetic_field(self, magnetic_field):
        self.measurements.magnetic_field = magnetic_field
        self.measurements.magnetic_field_strength = np.norm(magnetic_field)

    def update_optical(self, flow_x, flow_y):
        self.measurements.mouse_integrated_x += flow_x
        self.measurements.mouse_integrated_y += flow_y

    def publish_estimation(self):
        est = self.ekf.get_estimation_msg()
        est.stamp = self.get_clock().now().to_msg()
        est.measurements = self.measurements
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
