from rclpy.node import Node
from estimation.msg import Estimation
from rpi_hw_monitor.msg import HardwareStatus

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
        
        # Subscribe to the hardware monitor topic
        self.hw_monitor_subscription = self.create_subscription(
            HardwareStatus,
            '/hw_status',
            self.hw_monitor_callback,
            10
        )
        
        # Initialize data storage
        self.latest_data = None
        self.latest_hw_data = None
    
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
            "temperature": msg.temperature
        }
    
    def get_latest_data(self):
        """Return the latest estimation data"""
        return self.latest_data
    
    def get_latest_hw_data(self):
        """Return the latest hardware monitor data"""
        return self.latest_hw_data
