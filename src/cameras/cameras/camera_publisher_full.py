#!/usr/bin/env python3

"""
Created by: Cameron Basara
Date: , 2024
Purpose: Publish video feed from all camera topics for complete vision

        run: 'ros2 run cameras camera_publisher_full' --ros-args -p device_path:=/dev/video#' 
        along side display launch files: 'ros2 launch cameras camera_full_launch.launch.py' eg.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisherFull(Node):
    def __init__(self):
        super().__init__('camera_publisher_full')
        # Initialize path to usb, currently set for my laptop
        self.bridge = CvBridge()  # init bridge  

        # Auto-detect available camera devices
        device_paths = self.detect_available_cameras()

        self._publishers = []
        self._caps = []
        self._dev_paths = []
        
        for i, dev_path in enumerate(device_paths):
            pub = self.create_publisher(Image, f'cam_{i}', 10)
            cap = cv2.VideoCapture(dev_path) 

            if not cap.isOpened():
                self.get_logger().error(f'Could not open video device at: {dev_path}')
                continue

            self._publishers.append(pub)
            self._caps.append(cap)
            self._dev_paths.append(dev_path)
        
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)  # Timer

        if len(self._caps) == 0:
            self.get_logger().error('No active video devices found.')
            exit(1)
    
    def detect_available_cameras(self):
        # To detect any cameras in use
        available_cameras = []
        max_tested = 10  

        for i in range(max_tested):
            cap = cv2.VideoCapture(f'/dev/video{i}')
            if cap.isOpened():
                available_cameras.append(f'/dev/video{i}')
                cap.release()

        self.get_logger().info(f'Detected camera devices: {available_cameras}')
        return available_cameras

    def timer_callback(self):
        for i in range(len(self._caps)):
            cap = self._caps[i]
            dev_path = self._dev_paths[i]
            ret, frame = cap.read()
            
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                self._publishers[i].publish(msg)
            else:
                self.get_logger().error(f'Error capturing frame from {dev_path}')

def main(args=None):
    # Initialize node
    rclpy.init(args=args)
    node = CameraPublisherFull()

    try:
        # Spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        for cap in node._caps:
            cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
