#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, MagneticField
from estimation.msg import Estimation, Measurements
import numpy as np
import estimation_pkg.utils as utils
from estimation.srv import SwitchEstimator, ResetEstimator
import message_filters


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")

        self.estimator = utils.get_estimator('optical_imu_integrator')
        if self.estimator is None:
            self.get_logger().error("Failed to initialize estimator.")
            rclpy.shutdown()
            return

        self.measurements = Measurements()
        self.optical_x_to_m = 0.000017929959
        self.optical_y_to_m = 0.000019627030

        self.publisher = self.create_publisher(Estimation, "/estimation", 10)

        self.create_subscription(Point, "/optical", self.optical_callback, 10)

        self.imu_subscriber = message_filters.Subscriber(self, Imu, "/imu/data")
        self.mag_subscriber = message_filters.Subscriber(self, MagneticField, "/imu/mag")
        self.ts = message_filters.TimeSynchronizer(
            [self.imu_subscriber, self.mag_subscriber], queue_size=200
        )
        self.ts.registerCallback(self.imu_mag_callback)
        
        self.create_service(SwitchEstimator, 'switch_estimator', self.switch_estimator_callback)
        self.create_service(ResetEstimator, 'reset_estimator', self.reset_estimator_callback)

        self.get_logger().info("EstimatorNode started.")

    def imu_mag_callback(self, imu_msg: Imu, mag_msg: MagneticField):
        self.measurements.linear_acceleration = imu_msg.linear_acceleration
        self.measurements.angular_velocity = imu_msg.angular_velocity
        self.measurements.est_orientation = imu_msg.orientation
        self.measurements.magnetic_field = mag_msg.magnetic_field
        self.measurements.magnetic_field_strength = np.linalg.norm(
            np.array([
                mag_msg.magnetic_field.x,
                mag_msg.magnetic_field.y,
                mag_msg.magnetic_field.z
            ])
        )

        self.measurements.stamp = self.get_clock().now().to_msg()
        if self.estimator is not None:
            self.estimator.update_measurements(self.measurements)
            self.publish_estimation()

    def optical_callback(self, opt_msg: Point):
        flow_x = opt_msg.x * self.optical_x_to_m
        flow_y = opt_msg.y * self.optical_y_to_m

        self.measurements.mouse_integrated_x += float(flow_x)
        self.measurements.mouse_integrated_y += float(flow_y)

    def publish_estimation(self):
        est = self.estimator.get_estimation_msg()
        est.stamp = self.get_clock().now().to_msg()
        est.measurements = self.measurements
        self.publisher.publish(est)

    def switch_estimator_callback(self, request, response):
        new_estimator = utils.get_estimator(request.estimator_name)
        if new_estimator is not None:
            self.estimator = new_estimator
            self.get_logger().info(f"Switched to estimator: {request.estimator_name}")
            response.success = True
            response.message = f"Switched to estimator: {request.estimator_name}"
        else:
            self.get_logger().error(f"Failed to switch to estimator: {request.estimator_name}")
            response.success = False
            response.message = f"Invalid estimator name: {request.estimator_name}"
        return response

    def reset_estimator_callback(self, request, response):
        self.get_logger().info("Resetting estimator...")
        if self.estimator is not None:
            self.estimator.reset()
            self.get_logger().info("Estimator reset.")
            response.success = True
            response.message = "Estimator reset."
        else:
            self.get_logger().error("Failed to reset estimator: Estimator is None.")
            response.success = False
            response.message = "Estimator is None."
        return response


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
