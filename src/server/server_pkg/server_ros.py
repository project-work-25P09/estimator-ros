from rclpy.node import Node
from estimation.msg import Estimation
from estimation.srv import SwitchEstimator
from rpi_hw_monitor.msg import HardwareStatus

import rclpy

class ServerROS(Node):
    def __init__(self):
        super().__init__("server_ros_node")

        # Subscribe to the estimation topic
        self.subscription = self.create_subscription(
            Estimation, "/estimation", self.estimation_callback, 10
        )

        # Subscribe to the hardware monitor topic
        self.hw_monitor_subscription = self.create_subscription(
            HardwareStatus, "/hw_status", self.hw_monitor_callback, 10
        )

        self.switch_estimator_client = self.create_client(SwitchEstimator, 'switch_estimator')
        while not self.switch_estimator_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for switch_estimator service...')

        # Initialize data storage
        self.latest_data = None
        self.latest_hw_data = None

    def estimation_callback(self, msg):
        """Store the latest estimation data"""
        # Convert ROS timestamp to seconds (for plotting)
        timestamp_sec = msg.stamp.sec + (msg.stamp.nanosec / 1e9)

        self.latest_data = {
            "timestamp": timestamp_sec,
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "yaw": msg.yaw,
            "pitch": msg.pitch,
            "roll": msg.roll,
            "acc_x": msg.measurements.acceleration.x,
            "acc_y": msg.measurements.acceleration.y,
            "acc_z": msg.measurements.acceleration.z,
            "acc_yaw": msg.yaw,  # TODO: change
            "acc_pitch": msg.pitch,
            "acc_roll": msg.roll,
            "mag_x": msg.measurements.magnetic_field.x,
            "mag_y": msg.measurements.magnetic_field.y,
            "mag_z": msg.measurements.magnetic_field.z,
            "mag_strength": msg.measurements.magnetic_field_strength,
            "mouse_integrated_x": msg.measurements.mouse_integrated_x,
            "mouse_integrated_y": msg.measurements.mouse_integrated_y,
        }

    def hw_monitor_callback(self, msg):
        """Store the latest hardware monitor data"""
        self.latest_hw_data = {
            "cpu_usage": msg.cpu_usage,
            "memory_mb": msg.memory_mb,
            "disk_rx_mb": msg.disk_rx_mb,
            "disk_tx_mb": msg.disk_tx_mb,
            "network_rx_mb": msg.network_rx_mb,
            "network_tx_mb": msg.network_tx_mb,
            "power_consumption": msg.power_consumption,
            "temperature": msg.temperature,
        }

    def get_latest_data(self):
        """Return the latest estimation data"""
        return self.latest_data

    def get_latest_hw_data(self):
        """Return the latest hardware monitor data"""
        return self.latest_hw_data

    def switch_estimator(self, estimator_name):
        """Switch the active estimator by calling the ROS service."""
        request = SwitchEstimator.Request()
        request.estimator_name = estimator_name

        future = self.switch_estimator_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        return True
        # if future.result() is not None:
        #     self.get_logger().info(f"Service call succeeded: {future.result().message}")
        #     return future.result().success
        # else:
        #     self.get_logger().error("Service call failed")
        #     return False
