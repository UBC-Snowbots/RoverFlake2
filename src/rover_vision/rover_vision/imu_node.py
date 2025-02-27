#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import pyrealsense2 as rs
import numpy as np

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('full_cameras_node')
        self.imu_pub = self.create_publisher(Imu, '/camera/imu/data', 10)
        self.rs_pipeline = None
        self.has_realsense = self.initialize_camera()
        
        if self.has_realsense:
            self.get_logger().info('Starting IMU data processing')
            # Increase timer period to reduce load (50Hz instead of 100Hz)
            self.timer = self.create_timer(0.02, self.timer_callback)

    def initialize_camera(self):
        try:
            self.rs_pipeline = rs.pipeline()
            self.rs_config = rs.config()
            
            # Try to find RealSense devices
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                self.get_logger().info('No RealSense devices detected')
                return False
                
            self.get_logger().info(f"Found RealSense device: {devices[0].get_info(rs.camera_info.name)}")
            
            # Set specific IMU fps
            self.rs_config.enable_stream(rs.stream.accel)
            self.rs_config.enable_stream(rs.stream.gyro)
            
            # Start pipeline with configuration
            self.rs_pipeline.start(self.rs_config)
            self.get_logger().info('RealSense camera initialized successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RealSense: {e}')
            return False

    @staticmethod
    def gyro_data(gyro):
        return np.asarray([gyro.x, gyro.y, gyro.z])

    @staticmethod
    def accel_data(accel):
        return np.asarray([accel.x, accel.y, accel.z])

    def timer_callback(self):
        if self.has_realsense:
            try:
                # Add a short timeout for frame waiting (5ms)
                frames = self.rs_pipeline.wait_for_frames(5000)
                
                # Get motion frame set
                accel = frames.first_or_default(rs.stream.accel)
                gyro = frames.first_or_default(rs.stream.gyro)
                
                if accel and gyro:
                    # Extract motion data
                    accel_data = self.accel_data(accel.as_motion_frame().get_motion_data())
                    gyro_data = self.gyro_data(gyro.as_motion_frame().get_motion_data())
                    
                    # Create and populate IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = "camera_imu_frame"
                    
                    # Set angular velocity (gyro)
                    imu_msg.angular_velocity.x = gyro_data[0]
                    imu_msg.angular_velocity.y = gyro_data[1]
                    imu_msg.angular_velocity.z = gyro_data[2]
                    
                    # Set linear acceleration (accelerometer)
                    imu_msg.linear_acceleration.x = accel_data[0]
                    imu_msg.linear_acceleration.y = accel_data[1]
                    imu_msg.linear_acceleration.z = accel_data[2]
                    
                    # Publish the message
                    self.imu_pub.publish(imu_msg)
                    
            except Exception as e:
                self.get_logger().error(f'Error processing frames: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.rs_pipeline:
            node.rs_pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()