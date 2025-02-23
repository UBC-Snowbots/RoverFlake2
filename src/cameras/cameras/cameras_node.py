#!/usr/bin/env python3
"""
Created by: Cameron Basara
Date: , 2024
Purpose: Publish video feed from all camera topics for complete vision,
         with special handling for RealSense cameras
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
from typing import List, Dict, Tuple, Optional

class CameraPublisherFull(Node):
    def __init__(self):
        super().__init__('camera_publisher_full')
        self.bridge = CvBridge()
        
        # Initialize RealSense if present (but skip IMU)
        self.rs_pipeline = None
        self.rs_config = None
        self.rs_publishers = {}
        self.has_realsense = self.setup_realsense()
        
        # Initialize standard cameras (regardless of RealSense detection)
        self.std_publishers = []
        self.std_caps = []
        self.std_dev_paths = []
        
        # Auto-detect available standard camera devices
        device_paths = self.detect_available_cameras()
        for i, dev_path in enumerate(device_paths):
            pub = self.create_publisher(Image, f'cam_{i}', 30)
            cap = cv2.VideoCapture(dev_path)
            if not cap.isOpened():
                self.get_logger().error(f'Could not open video device at: {dev_path}')
                continue
            self.std_publishers.append(pub)
            self.std_caps.append(cap)
            self.std_dev_paths.append(dev_path)
        
        # Check if we have any camera at all
        if not self.has_realsense and len(self.std_caps) == 0:
            self.get_logger().error('No active video devices found.')
            exit(1)
        
        # Setup timer callback
        self.timer_period = 0.033  # ~30 FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def setup_realsense(self) -> bool:
        """Initialize RealSense camera if available, but skip IMU"""
        try:
            # Create pipeline and config
            self.rs_pipeline = rs.pipeline()
            self.rs_config = rs.config()
            
            # Try to find RealSense devices
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.get_logger().info('No RealSense devices detected')
                return False
            
            # Setup streams for the first RealSense device
            self.get_logger().info(f'Found RealSense device: {devices[0].get_info(rs.camera_info.name)}')
            
            # Enable streams with reasonable resolution and framerate
            self.rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.rs_config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            self.rs_config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
            self.rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Skip IMU for now
            self.get_logger().info('Skipping IMU streams to avoid permission issues')
            
            # Start streaming
            self.rs_pipeline.start(self.rs_config)
            
            # Create publishers for each stream
            self.rs_publishers['color'] = self.create_publisher(Image, '/camera/color/image_raw', 30)
            self.rs_publishers['infra1'] = self.create_publisher(Image, '/camera/infra1/image_rect_raw', 30)
            self.rs_publishers['infra2'] = self.create_publisher(Image, '/camera/infra2/image_rect_raw', 30)
            self.rs_publishers['depth'] = self.create_publisher(Image, '/camera/depth/image_rect_raw', 30)
            
            self.get_logger().info('RealSense camera initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().warn(f'Failed to initialize RealSense: {e}')
            return False
    
    def detect_available_cameras(self) -> List[str]:
        """Detect standard USB cameras, excluding RealSense devices"""
        available_cameras = []
        max_tested = 10
        
        for i in range(max_tested):
            cap = cv2.VideoCapture(f'/dev/video{i}')
            if cap.isOpened():
                # Check if this is not a RealSense camera by looking at properties
                name = cap.getBackendName()
                if 'realsense' not in name.lower():
                    available_cameras.append(f'/dev/video{i}')
                else:
                    self.get_logger().info(f'Skipping RealSense camera at /dev/video{i}')
            cap.release()
        
        self.get_logger().info(f'Detected standard camera devices: {available_cameras}')
        return available_cameras
    
    def publish_realsense_frame(self, frame, frame_type):
        if frame_type in self.rs_publishers:
            if frame_type in ['color', 'infra1', 'infra2', 'depth']:
                # Process image based on type
                if frame_type == 'color':
                    # Convert BGR to RGB
                    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
                elif frame_type in ['infra1', 'infra2']:
                    # Infrared is already grayscale (y8)
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
                else:  # depth
                    # Depth is 16-bit
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="16UC1")
                
                self.rs_publishers[frame_type].publish(msg)
    
    # IMU support removed for now
    
    def timer_callback(self):
        # Process RealSense frames if available
        if self.has_realsense:
            try:
                frames = self.rs_pipeline.wait_for_frames()
                
                # Process camera frames
                color_frame = frames.get_color_frame()
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    self.publish_realsense_frame(color_image, 'color')
                
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    self.publish_realsense_frame(depth_image, 'depth')
                
                # Process infrared frames
                for i in [1, 2]:
                    ir_frame = frames.get_infrared_frame(i)
                    if ir_frame:
                        ir_image = np.asanyarray(ir_frame.get_data())
                        self.publish_realsense_frame(ir_image, f'infra{i}')
                
                # IMU processing removed
                
            except Exception as e:
                self.get_logger().error(f'Error processing RealSense frames: {e}')
        
        # Process standard camera frames
        for i in range(len(self.std_caps)):
            cap = self.std_caps[i]
            dev_path = self.std_dev_paths[i]
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                self.std_publishers[i].publish(msg)
            else:
                # self.get_logger().error(f'Error capturing frame from {dev_path}')
                pass

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
        if node.rs_pipeline:
            node.rs_pipeline.stop()
        for cap in node.std_caps:
            cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()