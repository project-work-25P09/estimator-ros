#!/usr/bin/env python3
import threading
from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class OpticalSensorNode(Node):
    def __init__(self):
        super().__init__('optical_sensor_node')

        self.publisher_ = self.create_publisher(Point, '/optical', 10)
        self.device_path = '/dev/input/event0'
        
        try:
            self.device = InputDevice(self.device_path)
            self.get_logger().info(f"Opened device: {self.device.name} ({self.device.path})")
        except Exception as e:
            self.get_logger().error(f"Failed to open device {self.device_path}: {e}")
            self.device = None

        if self.device is not None:
            self.thread = threading.Thread(target=self.read_device_events, daemon=True)
            self.thread.start()

    def read_device_events(self):
        for event in self.device.read_loop():
            if event.type == ecodes.EV_REL:
                point = Point()
                if event.code == ecodes.REL_X:
                    point.x = float(event.value)
                    point.y = 0.0
                    point.z = 0.0
                    # self.get_logger().info(f"X movement: {event.value}")
                    self.publisher_.publish(point)
                elif event.code == ecodes.REL_Y:
                    point.x = 0.0
                    point.y = float(event.value)
                    point.z = 0.0
                    # self.get_logger().info(f"Y movement: {event.value}")
                    self.publisher_.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = OpticalSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OpticalSensorNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
