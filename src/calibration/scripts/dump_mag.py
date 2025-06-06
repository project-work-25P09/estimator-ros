#!/usr/bin/env python3
import csv, rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

class Dumper(Node):
    def __init__(self):
        super().__init__('mag_dump')
        self.csv = csv.writer(open('data/magsweep_dump.csv','w',newline=''))
        self.csv.writerow(['x','y','z'])
        self.sub = self.create_subscription(
            MagneticField, '/imu/mag', self.cb, 10)

    def cb(self,msg):
        m = msg.magnetic_field
        self.csv.writerow([m.x, m.y, m.z])

if __name__=='__main__':
    rclpy.init()
    node = Dumper()
    rclpy.spin(node)
