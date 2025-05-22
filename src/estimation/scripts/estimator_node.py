#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, MagneticField
from estimation.msg import Estimation, Measurements
import numpy as np
from estimation_pkg.ekf import EKF
import estimation_pkg.utils as utils


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")

        self.ekf = utils.get_ekf()

        self.measurements = Measurements()
        self.publisher = self.create_publisher(Estimation, "/estimation", 10)

        self.create_subscription(Point, "/optical", self.optical_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 200)
        self.create_subscription(MagneticField, "/imu/mag", self.imu_mag_callback, 200)

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
        self.measurements.est_orientation = orientation

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
        self.publisher.publish(est)


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
