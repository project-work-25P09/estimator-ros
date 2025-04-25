#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from estimation.msg import Estimation

class EstimatorNode(Node):
    def __init__(self):
        super().__init__('estimator_node')

        self.est_pub = self.create_publisher(Estimation, '/estimation', 10)
        self.create_subscription(Point, '/optical', self.optical_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.optical_to_mm = 3000.0 / 19000.0

        self.current_position = Estimation(
          x=0.0, y=0.0, z=0.0,
          yaw=0.0, pitch=0.0, roll=0.0,
          acc_x=0.0, acc_y=0.0, acc_z=0.0,
          acc_yaw=0.0, acc_pitch=0.0, acc_roll=0.0,
          mag_x=0.0, mag_y=0.0, mag_z=0.0,
          mag_strength=0.0,
          mouse_movement=0.0,
          mouse_speed=0.0,
          mouse_direction=0.0,
          mouse_distance=0.0,
        )
        self.get_logger().info("EstimatorNode initialized.")

    def optical_callback(self, optical_msg: Point):
        self.current_position.x += optical_msg.x * self.optical_to_mm
        self.current_position.y += optical_msg.y * self.optical_to_mm
        self.current_position.z = 0.0

        self.est_pub.publish(self.current_position)
        # self.get_logger().info(
        #     f"x={optical_msg.x}, y={optical_msg.y} "
        #     f"-> New est_position: (x={self.current_position.x}, y={self.current_position.y}, z={self.current_position.z})"
        # )

    def imu_callback(self, imu_msg: Imu):
        # self.current_position.z = imu_msg.linear_acceleration.z
        # self.get_logger().info(f"IMU update: linear_acceleration.z = {imu_msg.linear_acceleration.z}")
        pass

def main(args=None):
    rclpy.init(args=args)
    estimator_node = EstimatorNode()

    try:
        rclpy.spin(estimator_node)
    except KeyboardInterrupt:
        pass
    finally:
        estimator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
Example of one imu reading:

---
header:
  stamp:
    sec: 1744340905
    nanosec: 425988435
  frame_id: imu_link
orientation:
  x: 0.05693411199815065
  y: 0.026914445137809447
  z: -0.6838679977988614
  w: 0.7268828524050367
orientation_covariance:
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
angular_velocity:
  x: -0.006033787038177252
  y: -0.0025542210787534714
  z: -0.0006926205824129283
angular_velocity_covariance:
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
linear_acceleration:
  x: -1.1329937596444275
  y: 0.5039420764421088
  z: 9.753446633748448
linear_acceleration_covariance:
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
- 0.0
- 0.0
- 0.0
- 0.01
---

"""
