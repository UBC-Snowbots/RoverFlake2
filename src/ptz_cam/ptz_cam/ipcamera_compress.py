#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import 

class CompressedFeedNode(Node):
    def __init__(self):
        super().__init__('compressed_feed_node')

        self.declare_parameter('camera_user', 'admin')
        self.declare_parameter('camera_pass', '123456')
        self.declare_parameter('camera_ip', '192.168.0.95')
        self.declare_parameter('stream_path', 'stream0')
        self.declare_parameter('interface', 'eth0')

        self.user = self.get_parameter('camera_user').get_parameter_value().string_value
        self.password = self.get_parameter('camera_pass').get_parameter_value().string_value
        self.ip = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.stream_path = self.get_parameter('stream_path').get_parameter_value().string_value
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        # Dynamically build the RTSP URL
        self.camera_url = f'rtsp://{self.user}:{self.password}@{self.ip}/{self.stream_path}'
        self.get_logger().info(f'Connecting to camera at: {self.camera_url}')