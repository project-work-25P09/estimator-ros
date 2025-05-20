#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RecordingServiceNode(Node):
    def __init__(self):
        super().__init__("recording_service_node")

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = RecordingServiceNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
