"""
Created by: Cameron Basara
Date: March 31, 2024
Purpose: Publish video feed from a arbitruary usb camera to a topic

        run: 'ros2 run cameras camera_publisher --ros-args -p device_path:=/dev/video#' 
        along side display launch files: 'ros2 launch cameras camera_viewer_launch.launch.py' eg.
"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Initialize path to usb, currently set for my laptop
        self.declare_parameter('device_path', '/dev/video0') # Sets default to /dev/video0 
        device_path = self.get_parameter('device_path').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, '/usb_cam', 10) # Initialize pub
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback) # Timer
        self.bridge = CvBridge() # Converts between ROS image messages and Cv images 
        self.cap = cv2.VideoCapture(device_path)

        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device at: {device_path}')
            exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8') # Using the bridge as to convert OpenCv images -> ROS im
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Error capturing frame')

def main(args=None):
    # Initialize node
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        # Spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
