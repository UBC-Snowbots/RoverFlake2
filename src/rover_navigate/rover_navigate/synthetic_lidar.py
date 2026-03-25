"""
Synthetic LiDAR data publisher.

Simulates a 2D LiDAR (like RPLidar A1/A2 or Hokuyo) by publishing
sensor_msgs/LaserScan on /scan.

Typical real LiDAR specs (RPLidar A1):
  - Range: 0.15m to 12m
  - Angular resolution: ~1 degree (360 samples)
  - Scan rate: ~5.5 Hz
  - Field of view: 360 degrees

The synthetic environment is a simple rectangular room with optional
obstacles. Obstacles can be added/removed at runtime to test the
path planner's reaction.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SyntheticLidar(Node):
    """Publishes fake LaserScan data simulating a 2D LiDAR in a room."""

    def __init__(self):
        super().__init__('synthetic_lidar')

        # --- LiDAR parameters (mimics RPLidar A1) ---
        self.declare_parameter('scan_rate', 5.5)         # Hz
        self.declare_parameter('num_samples', 360)       # rays per scan
        self.declare_parameter('range_min', 0.15)        # meters
        self.declare_parameter('range_max', 12.0)        # meters
        self.declare_parameter('frame_id', 'laser_link')

        # --- Room parameters (rectangular room in meters) ---
        self.declare_parameter('room_width', 10.0)       # X dimension
        self.declare_parameter('room_height', 10.0)      # Y dimension
        self.declare_parameter('robot_x', 5.0)           # robot position in room
        self.declare_parameter('robot_y', 5.0)

        # --- Obstacles: list of [x, y, radius] ---
        # Default: a few obstacles scattered in the room
        self.declare_parameter('obstacles', [
            3.0, 3.0, 0.5,    # obstacle 1: x=3, y=3, r=0.5m
            7.0, 4.0, 0.8,    # obstacle 2: x=7, y=4, r=0.8m
            5.0, 7.5, 0.3,    # obstacle 3: x=5, y=7.5, r=0.3m
            2.0, 6.0, 1.0,    # obstacle 4: x=2, y=6, r=1.0m
        ])

        # --- Noise ---
        self.declare_parameter('noise_std', 0.02)        # Gaussian noise std dev (meters)

        # Read parameters
        self.scan_rate = self.get_parameter('scan_rate').value
        self.num_samples = self.get_parameter('num_samples').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.frame_id = self.get_parameter('frame_id').value
        self.room_width = self.get_parameter('room_width').value
        self.room_height = self.get_parameter('room_height').value
        self.robot_x = self.get_parameter('robot_x').value
        self.robot_y = self.get_parameter('robot_y').value
        self.noise_std = self.get_parameter('noise_std').value

        # Parse obstacles
        obs_flat = self.get_parameter('obstacles').value
        self.obstacles = []
        for i in range(0, len(obs_flat), 3):
            if i + 2 < len(obs_flat):
                self.obstacles.append({
                    'x': obs_flat[i],
                    'y': obs_flat[i + 1],
                    'r': obs_flat[i + 2],
                })

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.scan_rate, self.publish_scan)

        self.get_logger().info(
            f'Synthetic LiDAR started: {self.num_samples} rays, '
            f'{self.scan_rate} Hz, room {self.room_width}x{self.room_height}m, '
            f'{len(self.obstacles)} obstacles'
        )

    def ray_cast(self, angle: float) -> float:
        """
        Cast a ray from the robot position at the given angle and return
        the distance to the nearest intersection (wall or obstacle).
        """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        min_dist = self.range_max

        # --- Check walls (axis-aligned bounding box) ---
        # Right wall (x = room_width)
        if cos_a > 1e-9:
            t = (self.room_width - self.robot_x) / cos_a
            y_hit = self.robot_y + t * sin_a
            if 0.0 <= y_hit <= self.room_height and t > 0:
                min_dist = min(min_dist, t)

        # Left wall (x = 0)
        if cos_a < -1e-9:
            t = -self.robot_x / cos_a
            y_hit = self.robot_y + t * sin_a
            if 0.0 <= y_hit <= self.room_height and t > 0:
                min_dist = min(min_dist, t)

        # Top wall (y = room_height)
        if sin_a > 1e-9:
            t = (self.room_height - self.robot_y) / sin_a
            x_hit = self.robot_x + t * cos_a
            if 0.0 <= x_hit <= self.room_width and t > 0:
                min_dist = min(min_dist, t)

        # Bottom wall (y = 0)
        if sin_a < -1e-9:
            t = -self.robot_y / sin_a
            x_hit = self.robot_x + t * cos_a
            if 0.0 <= x_hit <= self.room_width and t > 0:
                min_dist = min(min_dist, t)

        # --- Check circular obstacles ---
        for obs in self.obstacles:
            # Vector from robot to obstacle center
            dx = obs['x'] - self.robot_x
            dy = obs['y'] - self.robot_y

            # Project onto ray direction
            proj = dx * cos_a + dy * sin_a
            if proj < 0:
                continue  # obstacle is behind the ray

            # Perpendicular distance from obstacle center to ray
            perp = abs(dx * sin_a - dy * cos_a)
            if perp >= obs['r']:
                continue  # ray misses the obstacle

            # Distance to intersection point
            half_chord = math.sqrt(obs['r'] ** 2 - perp ** 2)
            t = proj - half_chord
            if t > 0:
                min_dist = min(min_dist, t)

        return min_dist

    def publish_scan(self):
        """Generate and publish one LaserScan message."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / self.num_samples
        msg.time_increment = (1.0 / self.scan_rate) / self.num_samples
        msg.scan_time = 1.0 / self.scan_rate
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = []
        for i in range(self.num_samples):
            angle = msg.angle_min + i * msg.angle_increment
            dist = self.ray_cast(angle)

            # Add Gaussian noise
            if self.noise_std > 0:
                dist += np.random.normal(0, self.noise_std)

            # Clamp to valid range
            dist = max(self.range_min, min(self.range_max, dist))
            ranges.append(dist)

        msg.ranges = ranges
        msg.intensities = [100.0] * self.num_samples  # uniform intensity

        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
