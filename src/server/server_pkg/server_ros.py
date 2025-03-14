from rclpy.node import Node

class ServerROS(Node):
    def __init__(self):
        super().__init__('server_ros_node')
