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

        self.do_save = True
        self.do_save_imu = True
        self.do_save_optical = True
        self.do_save_estimation = True
        self.save_fp = "./data/trajectory002"
        self.save_optical_fp = os.path.join(self.save_fp, "optical.csv")
        self.save_imu_fp = os.path.join(self.save_fp, "imu.csv")
        self.save_estimation_fp = os.path.join(self.save_fp, "estimation.csv")
        self.file_optical = None
        self.file_imu = None
        self.file_estimation = None

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

        self.start_saving()

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

        # Store the latest IMU data for acceleration values
        self.latest_imu = imu_msg

        self.ekf.predict(a, w, o)
        self.publish_estimation()
        self.cb_imu_save(imu_msg)

    def imu_mag_callback(self, mag_msg: MagneticField):
        pass

    def optical_callback(self, opt_msg: Point):
        self.opt_int_x += opt_msg.x
        self.opt_int_y += opt_msg.y
        flow_x = opt_msg.x * self.optical_x_to_m
        flow_y = opt_msg.y * self.optical_y_to_m

        self.ekf.update_optical(flow_x, flow_y)

        self.publish_estimation()
        self.cb_optical_save(opt_msg)

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
        self.cb_estimation_save(est)

    def cb_imu_save(self, imu):
        if not self.do_save or not self.do_save_imu:
            return
        # Use ROS timestamp from the IMU message
        ros_timestamp = imu.header.stamp.sec + (imu.header.stamp.nanosec / 1e9)
        data = [
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
        ]
        self.file_imu.write(f"{ros_timestamp},{','.join(f'{x}' for x in data)}\n")

    def cb_optical_save(self, optical):
        if not self.do_save or not self.do_save_optical:
            return
        # Use current ROS time since Point messages don't have timestamps
        ros_time = self.get_clock().now()
        ros_timestamp = ros_time.seconds_nanoseconds()
        ros_timestamp_sec = ros_timestamp[0] + (ros_timestamp[1] / 1e9)
        data = [
            optical.x,
            optical.y,
        ]
        self.file_optical.write(
            f"{ros_timestamp_sec},{','.join(str(x) for x in data)}\n"
        )

    def cb_estimation_save(self, estimation):
        if not self.do_save or not self.do_save_estimation:
            return
        # Use ROS timestamp instead of system time
        ros_timestamp = estimation.stamp.sec + (estimation.stamp.nanosec / 1e9)
        data = [
            estimation.x,
            estimation.y,
            estimation.z,
            estimation.yaw,
            estimation.pitch,
            estimation.roll,
            estimation.acc_x,
            estimation.acc_y,
            estimation.acc_z,
            estimation.acc_yaw,
            estimation.acc_pitch,
            estimation.acc_roll,
            estimation.mag_x,
            estimation.mag_y,
            estimation.mag_z,
            estimation.mag_strength,
            estimation.mouse_integrated_x,
            estimation.mouse_integrated_y,
        ]
        self.file_estimation.write(
            f"{ros_timestamp},{','.join(str(x) for x in data)}\n"
        )

    def start_saving(self):
        self.get_logger().info(f"Started saving to {self.save_fp}")
        if not os.path.exists(self.save_fp):
            os.mkdir(self.save_fp)
        if self.do_save_optical:
            self.file_optical = open(self.save_optical_fp, "w")
        if self.do_save_imu:
            self.file_imu = open(self.save_imu_fp, "w")
        if self.do_save_estimation:
            self.file_estimation = open(self.save_estimation_fp, "w")

    def end_saving(self):
        self.get_logger().info(f"Ended saving to {self.save_fp}")
        if self.file_optical is not None:
            self.file_optical.close()
            self.file_optical = None
        if self.file_imu is not None:
            self.file_imu.close()
            self.file_imu = None
        if self.file_estimation is not None:
            self.file_estimation.close()
            self.file_estimation = None


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
