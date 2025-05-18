from rclpy.node import Node
from estimation.msg import Estimation

class ServerROS(Node):
    def __init__(self):
        super().__init__('server_ros_node')
        
        # Subscribe to the estimation topic
        self.subscription = self.create_subscription(
            Estimation,
            '/estimation',
            self.estimation_callback,
            10
        )
        
        # Initialize data storage
        self.latest_data = None
    
    def estimation_callback(self, msg):
        """Store the latest estimation data"""
        self.latest_data = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "yaw": msg.yaw,
            "pitch": msg.pitch,
            "roll": msg.roll,
            "acc_x": msg.acc_x,
            "acc_y": msg.acc_y,
            "acc_z": msg.acc_z,
            "acc_yaw": msg.acc_yaw,
            "acc_pitch": msg.acc_pitch,
            "acc_roll": msg.acc_roll,
            "mag_x": msg.mag_x,
            "mag_y": msg.mag_y,
            "mag_z": msg.mag_z,
            "mag_strength": msg.mag_strength,
            "mouse_movement": msg.mouse_movement,
            "mouse_speed": msg.mouse_speed,
            "mouse_direction": msg.mouse_direction,
            "mouse_distance": msg.mouse_distance
        }
    
    def get_latest_data(self):
        """Return the latest estimation data"""
        return self.latest_data
