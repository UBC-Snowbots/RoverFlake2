#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import time

class FakeGPS(Node):
    def __init__(self):
        super().__init__('fake_gps_publisher')
        
        # Create a publisher for /gps/fix
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_gps_fix)
        
        # Fixed GPS coordinates (Example: Vancouver)
        self.latitude = 49.2827    # Vancouver latitude
        self.longitude = -123.1207 # Vancouver longitude
        self.altitude = 0.0        # Assume sea level

    def publish_gps_fix(self):
        msg = NavSatFix()
        
        # Set the GPS status (assume fix available)
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Set the fixed location
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude
        
        # Set covariance (0 for perfect fix, adjust if needed)
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'  # Frame name
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing fixed GPS: ({self.latitude}, {self.longitude})')

rclpy.init()
node = FakeGPS()
rclpy.spin(node)