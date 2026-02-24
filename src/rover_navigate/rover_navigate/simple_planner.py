"""
Simple obstacle-avoiding path planner.

Subscribes to:
  /scan              - LaserScan (real or synthetic)
  /odometry/filtered - UKF-filtered odometry (orientation from IMU)

Publishes:
  /cmd_vel           - Twist velocity commands

Behavior:
  1. Drive forward at a configurable speed
  2. If an obstacle is detected within a threshold distance in front,
     stop and rotate until the path is clear
  3. Prefer the direction with the most open space

This is a reactive planner (no global map). It's a starting point
that works with IMU-only localization and can be extended later with
costmap-based planning (Nav2).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SimplePlanner(Node):
    """Reactive obstacle-avoidance planner using LaserScan data."""

    def __init__(self):
        super().__init__('simple_planner')

        # --- Parameters ---
        self.declare_parameter('linear_speed', 0.3)          # m/s forward speed
        self.declare_parameter('angular_speed', 0.5)         # rad/s turning speed
        self.declare_parameter('obstacle_threshold', 1.5)    # meters - stop if obstacle closer
        self.declare_parameter('side_threshold', 0.8)        # meters - side clearance
        self.declare_parameter('front_arc_degrees', 60.0)    # degrees - frontal detection arc
        self.declare_parameter('control_rate', 10.0)         # Hz

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.side_threshold = self.get_parameter('side_threshold').value
        self.front_arc_deg = self.get_parameter('front_arc_degrees').value
        self.control_rate = self.get_parameter('control_rate').value

        # --- State ---
        self.latest_scan = None
        self.current_yaw = 0.0
        self.state = 'FORWARD'  # FORWARD, TURNING_LEFT, TURNING_RIGHT, STOPPED

        # --- Subscribers ---
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

        # --- Publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control loop timer ---
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f'Simple planner started: speed={self.linear_speed} m/s, '
            f'obstacle_threshold={self.obstacle_threshold}m, '
            f'front_arc={self.front_arc_deg}°'
        )

    def scan_callback(self, msg: LaserScan):
        """Store the latest scan."""
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        """Extract yaw from odometry quaternion."""
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw (z-rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def get_sector_min_range(self, scan: LaserScan,
                              start_deg: float, end_deg: float) -> float:
        """
        Get the minimum range in a sector defined by angles in degrees.
        0° = forward, positive = counter-clockwise.
        """
        num_samples = len(scan.ranges)
        if num_samples == 0:
            return scan.range_max

        min_range = scan.range_max

        for deg in range(int(start_deg), int(end_deg)):
            angle_rad = math.radians(deg % 360)
            index = int((angle_rad - scan.angle_min) / scan.angle_increment) % num_samples
            r = scan.ranges[index]
            if scan.range_min <= r <= scan.range_max:
                min_range = min(min_range, r)

        return min_range

    def get_sector_avg_range(self, scan: LaserScan,
                              start_deg: float, end_deg: float) -> float:
        """
        Get the average range in a sector (ignoring invalid readings).
        """
        num_samples = len(scan.ranges)
        if num_samples == 0:
            return scan.range_max

        total = 0.0
        count = 0

        for deg in range(int(start_deg), int(end_deg)):
            angle_rad = math.radians(deg % 360)
            index = int((angle_rad - scan.angle_min) / scan.angle_increment) % num_samples
            r = scan.ranges[index]
            if scan.range_min <= r <= scan.range_max:
                total += r
                count += 1

        return (total / count) if count > 0 else scan.range_max

    def control_loop(self):
        """Main control logic — called at control_rate Hz."""
        cmd = Twist()

        if self.latest_scan is None:
            # No scan data yet — stay still
            self.cmd_pub.publish(cmd)
            return

        scan = self.latest_scan
        half_arc = self.front_arc_deg / 2.0

        # Sector ranges
        front_min = self.get_sector_min_range(scan, 360 - half_arc, 360 + half_arc)
        left_avg = self.get_sector_avg_range(scan, 30, 150)
        right_avg = self.get_sector_avg_range(scan, 210, 330)
        left_min = self.get_sector_min_range(scan, 30, 90)
        right_min = self.get_sector_min_range(scan, 270, 330)

        prev_state = self.state

        # --- Decision logic ---
        if front_min < self.obstacle_threshold:
            # Obstacle ahead — turn toward the side with more space
            if left_avg > right_avg:
                self.state = 'TURNING_LEFT'
                cmd.angular.z = self.angular_speed
            else:
                self.state = 'TURNING_RIGHT'
                cmd.angular.z = -self.angular_speed
            cmd.linear.x = 0.0

        elif left_min < self.side_threshold:
            # Too close on the left — veer right slightly
            self.state = 'FORWARD'
            cmd.linear.x = self.linear_speed * 0.5
            cmd.angular.z = -self.angular_speed * 0.3

        elif right_min < self.side_threshold:
            # Too close on the right — veer left slightly
            self.state = 'FORWARD'
            cmd.linear.x = self.linear_speed * 0.5
            cmd.angular.z = self.angular_speed * 0.3

        else:
            # Clear path — drive forward
            self.state = 'FORWARD'
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        # Log state changes
        if self.state != prev_state:
            self.get_logger().info(
                f'State: {prev_state} -> {self.state} | '
                f'front={front_min:.2f}m, left={left_avg:.2f}m, right={right_avg:.2f}m'
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
