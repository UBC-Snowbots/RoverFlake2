#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sweeppy import Sweep
import math


class SweepScanNode(Node):
    def __init__(self):
        super().__init__('sweep_scan_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('rotation_speed', 5)  # Hz
        self.declare_parameter('sample_rate', 1000)  # Hz
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Create publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Initialize Sweep device with context manager
        self.sweep_context = Sweep(self.port).__enter__()
        
        try:
            # Configure device settings
            self.sweep_context.set_motor_speed(self.rotation_speed)
            
            self.sweep_context.set_sample_rate(self.sample_rate)
            
            self.get_logger().info('Starting scan...')
            self.sweep_context.start_scanning()
            
            # Initialize the scans iterator
            self.scans_iterator = self.sweep_context.get_scans()
            
            # Create timer for publishing scans
            self.create_timer(1.0 / self.rotation_speed, self.publish_scan)
            
            self.get_logger().info('Sweep SLAM node initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Sweep: {str(e)}')
            if self.sweep_context:
                self.sweep_context.__exit__(None, None, None)
    
    def publish_scan(self):
        if not self.sweep_context or not self.scans_iterator:
            return

        try:
            scan = next(self.scans_iterator)
            if not scan:
                self.get_logger().warn("Received empty scan")
                return

            # Check if scan is a nested list and flatten it if needed
            if len(scan) == 1 and isinstance(scan[0], list):
                samples = scan[0]
            else:
                samples = scan

            # Create and fill LaserScan message
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Set scan parameters
            msg.angle_min = 0.0
            msg.angle_max = 2.0 * math.pi
            num_samples = len(samples)
            msg.angle_increment = (msg.angle_max - msg.angle_min) / num_samples
            msg.scan_time = 1.0 / self.rotation_speed
            msg.time_increment = msg.scan_time / num_samples
            msg.range_min = 0.1   # in meters
            msg.range_max = 40.0  # in meters

            # Initialize ranges and intensities with defaults
            msg.ranges = [float('inf')] * num_samples
            msg.intensities = [0.0] * num_samples

            # Process each sample (assuming samples are in order)
            for idx, sample in enumerate(samples):
                # Convert the distance from centimeters to meters (if distance is in cm)
                distance_m = sample.distance / 100.0
                # Validate the reading; if outside acceptable range, keep it as inf
                if msg.range_min <= distance_m <= msg.range_max:
                    msg.ranges[idx] = distance_m
                else:
                    msg.ranges[idx] = float('inf')
                msg.intensities[idx] = float(sample.signal_strength)

            # Publish the LaserScan message
            self.scan_pub.publish(msg)

        except StopIteration:
            self.get_logger().warn("No more scans available.")

    def __del__(self):
        if hasattr(self, 'sweep_context') and self.sweep_context:
            try:
                self.sweep_context.stop_scanning()
                self.sweep_context.__exit__(None, None, None)
                self.get_logger().info('Stopped scanning and closed Sweep device')
            except Exception as e:
                self.get_logger().error(f'Error closing Sweep device: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SweepScanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()