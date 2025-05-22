#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, Quaternion
from sensor_msgs.msg import Imu, MagneticField
from estimation.msg import Estimation, Measurements
import numpy as np
import estimation_pkg.utils as utils
from estimation.srv import SwitchEstimator


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator_node")

        self.estimator = utils.get_estimator('imu_dead_reckoning')
        if self.estimator is None:
            self.get_logger().error("Failed to initialize estimator.")
            rclpy.shutdown()
            return

        self.imu_updated = False
        self.mag_updated = False
        self.opt_updated = False
        self.measurements = Measurements()

        self.optical_x_to_m = 0.000017929959
        self.optical_y_to_m = 0.000019627030

        self.publisher = self.create_publisher(Estimation, "/estimation", 10)

        self.create_subscription(Point, "/optical", self.optical_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 200)
        self.create_subscription(MagneticField, "/imu/mag", self.imu_mag_callback, 200)
        
        self.create_service(SwitchEstimator, 'switch_estimator', self.switch_estimator_callback)

        # TODO: reset service

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

        self.imu_updated = True
        self.cb_measurement()

    def imu_mag_callback(self, mag_msg: MagneticField):
        magnetic_field = np.array([
            mag_msg.magnetic_field.x,mag_msg.magnetic_field.y, mag_msg.magnetic_field.z
        ])
        self.update_magnetic_field(magnetic_field)

        self.mag_updated = True
        self.cb_measurement()

    def optical_callback(self, opt_msg: Point):
        flow_x = opt_msg.x * self.optical_x_to_m
        flow_y = opt_msg.y * self.optical_y_to_m
        self.update_optical(flow_x, flow_y)

        self.opt_updated = True
        self.cb_measurement()

    def update_imu(self, acceleration, angular_velocity, orientation):
        self.measurements.acceleration =  Vector3(**dict(zip(("x","y","z"), acceleration.tolist())))
        self.measurements.angular_velocity = Vector3(**dict(zip(("x","y","z"), angular_velocity.tolist())))
        self.measurements.est_orientation = Quaternion(**dict(zip(("x","y","z","w"), orientation.tolist())))

    def update_magnetic_field(self, magnetic_field):
        self.measurements.magnetic_field = Vector3(**dict(zip(("x","y","z"), magnetic_field.tolist())))
        self.measurements.magnetic_field_strength = np.linalg.norm(magnetic_field)

    def update_optical(self, flow_x, flow_y):
        self.measurements.mouse_integrated_x += float(flow_x)
        self.measurements.mouse_integrated_y += float(flow_y)

    def cb_measurement(self):
        if self.imu_updated and self.mag_updated: # and self.opt_updated:
            self.imu_updated = False
            self.mag_updated = False
            self.measurements.stamp = self.get_clock().now().to_msg()
            if self.estimator is not None:
                self.estimator.update_measurements(self.measurements)
            self.publish_estimation()

    def publish_estimation(self):
        if self.estimator is None:
            return
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
