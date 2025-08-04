#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Bool, Float32
from geopy import distance
from geopy.point import Point
import math

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')

        # States
        self.MANUAL = "MANUAL"  # do nothing
        self.LIGHT = "LIGHT"    # follow light
        self.AUTO = "AUTO"      # go to GPS point
        self.state = self.MANUAL

        # Position data
        self.rover_gps = None
        self.target_gps = None
        self.heading = None  # compass heading in radians
        self.light_bearing = None  # bearing to light

        # Control parameters
        self.linear_speed = 0.5
        self.angular_gain = 1.5

        # Subscribers
        self.gnss_sub = self.create_subscription(
            NavSatFix, '/gnss_fix', self.gnss_callback, 10)
        self.target_sub = self.create_subscription(
            NavSatFix, '/gnss_coord', self.target_callback, 10)
        self.heading_sub = self.create_subscription(
            Float32, '/heading', self.heading_callback, 10)
        self.state_sub = self.create_subscription(
            String, '/rover_state', self.state_callback, 10)
        self.color_sub = self.create_subscription(
            String, '/color_detection_status', self.color_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reached_pub = self.create_publisher(Bool, '/target_reached', 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Simple waypoint node started')

    def gnss_callback(self, msg):
        """Update rover position"""
        self.rover_gps = msg

    def target_callback(self, msg):
        """Update target position"""
        self.target_gps = msg
        self.get_logger().info(f'New target: ({msg.latitude:.6f}, {msg.longitude:.6f})')

    def heading_callback(self, msg):
        """Update compass heading (radians)"""
        self.heading = msg.data

    def state_callback(self, msg):
        """Update state"""
        self.state = msg.data
        self.get_logger().info(f'State: {self.state}')

    def color_callback(self, msg):
        """Get bearing from color detection"""
        if self.state == self.LIGHT:
            try:
                # Parse string-like "Tracking red at bearing -6.9°"
                parts = msg.data.split('bearing')
                if len(parts) > 1:
                    bearing_str = parts[1].strip().rstrip('°')
                    self.light_bearing = math.radians(float(bearing_str))
            except:
                pass

    def get_gps_bearing(self):
        """Calculate bearing to GPS target"""
        if not self.rover_gps or not self.target_gps or self.heading is None:
            return None, None

        # Calculate distance
        rover_pt = Point(self.rover_gps.latitude, self.rover_gps.longitude)
        target_pt = Point(self.target_gps.latitude, self.target_gps.longitude)
        dist = distance.distance(rover_pt, target_pt).meters

        # Calculate bearing (degrees from North)
        bearing_deg = distance.distance().bearing(rover_pt, target_pt)
        bearing_rad = math.radians(bearing_deg)

        # Convert to relative bearing
        relative_bearing = bearing_rad - self.heading
        
        # Normalize to [-pi, pi]
        while relative_bearing > math.pi:
            relative_bearing -= 2 * math.pi
        while relative_bearing < -math.pi:
            relative_bearing += 2 * math.pi

        return relative_bearing, dist

    def control_loop(self):
        """Main control - send velocity commands"""
        cmd = Twist()

        if self.state == self.MANUAL:
            # Do nothing
            pass

        elif self.state == self.AUTO:
            # Navigate to GPS waypoint
            bearing, dist = self.get_gps_bearing()
            
            if bearing is not None:
                self.get_logger().info(f'Target: {dist:.1f}m at {math.degrees(bearing):.0f}°', 
                                      throttle_duration_sec=1.0)
                
                # Check if reached
                if dist < 3.0:
                    msg = Bool()
                    msg.data = True
                    self.reached_pub.publish(msg)
                    self.get_logger().info('Target reached!')
                    self.target_gps = None
                else:
                    # Turn if needed
                    if abs(bearing) > 0.1:  
                        cmd.angular.z = self.angular_gain * bearing
                        cmd.linear.x = 0.0
                    else:
                        # Go forward
                        cmd.linear.x = self.linear_speed
                        cmd.angular.z = bearing * 0.5  

        elif self.state == self.LIGHT:
            # Follow light
            if self.light_bearing is not None:
                self.get_logger().info(f'Light at {math.degrees(self.light_bearing):.0f}°',
                                      throttle_duration_sec=0.5)
                
                if abs(self.light_bearing) > 0.1:
                    cmd.angular.z = self.angular_gain * self.light_bearing
                    cmd.linear.x = self.linear_speed * 0.5
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = self.light_bearing * 0.5
            else:
                # No light - spin to search
                cmd.angular.z = 0.3

        # Send command
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()