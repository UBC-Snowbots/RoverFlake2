"""Pretend 2D lidar in a pretend rock field.

A real lidar spins a laser and measures the distance to whatever each beam hits.
This node fakes that: it knows where the rocks are (and where the rover is, from /odom),
and works out how far each beam would travel before hitting a rock.

  - /scan    what the "lidar" sees (this is all Nav2 gets)
  - /rocks   where the rocks REALLY are, for RViz only (Nav2 never sees this)
  - Drop a new rock anywhere with RViz's "2D Pose Estimate" button (there's no localizer
    in this setup, so that button is free to reuse) or with "Publish Point".

Person mode (person:=True): a person (two legs) stands in the field. "Publish Point" makes
them walk to the clicked spot instead of adding a rock. For testing lidar_follow.py.
dropout > 0 leaves that share of directions with no reading (NaN), like the Sweep lidar.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

# x, y, radius (m), in the odom frame (the rover starts at 0, 0 facing +x)
DEFAULT_ROCKS = [
    2.5, 0.2, 0.40,
    3.2, 1.5, 0.35,
    4.0, -0.9, 0.50,
    5.5, 0.6, 0.45,
    6.8, 2.4, 0.35,
    3.8, 3.2, 0.50,
    7.5, -1.6, 0.40,
    2.0, -2.4, 0.40,
    9.0, 1.0, 0.60,
    # a "ridge" across the way at x = 11, with a gap between y = 0.5 and 2
    11.0, -3.0, 0.5, 11.0, -2.0, 0.5, 11.0, -1.0, 0.5, 11.0, 0.0, 0.5,
    11.0, 2.5, 0.5, 11.0, 3.5, 0.5, 11.0, 4.5, 0.5,
]


class FakeLidar(Node):

    def __init__(self):
        super().__init__('fake_lidar')
        self.declare_parameter('frame_id', 'laser')     # the lidar's own frame (mounted on the rover)
        self.declare_parameter('mount_x', 0.0)          # m: where the lidar sits, ahead of the rover's centre
        self.declare_parameter('rate', 10.0)            # Hz: scans per second
        self.declare_parameter('beams', 360)            # one beam per degree, all the way round
        self.declare_parameter('range_min', 0.15)       # m: closer than this, it can't measure
        self.declare_parameter('range_max', 12.0)       # m: beyond this, it sees nothing
        self.declare_parameter('noise', 0.01)           # m: random measurement error
        self.declare_parameter('rocks', DEFAULT_ROCKS)  # flat list: x, y, radius, x, y, radius, ...
        self.declare_parameter('clicked_rock_radius', 0.3)
        self.declare_parameter('person', False)         # True: add a walking person
        self.declare_parameter('person_x', 2.0)         # m: where the person starts
        self.declare_parameter('person_y', 0.0)
        self.declare_parameter('person_speed', 0.7)     # m/s walking speed
        self.declare_parameter('dropout', 0.0)          # share of directions with no reading (0-1)
        p = lambda name: self.get_parameter(name).value   # noqa: E731

        self.frame_id, self.mount_x = p('frame_id'), p('mount_x')
        self.range_min, self.range_max, self.noise = p('range_min'), p('range_max'), p('noise')
        self.clicked_radius = p('clicked_rock_radius')
        n = p('beams')
        self.angle_increment = 2 * math.pi / n
        self.beam_angles = -math.pi + self.angle_increment * np.arange(n)   # relative to the lidar's front
        flat = list(p('rocks'))
        self.rocks = np.array(flat, dtype=float).reshape(-1, 3) if flat else np.zeros((0, 3))
        self.pose = None                                  # (x, y, yaw, stamp) from /odom
        self.dropout = float(p('dropout'))
        self.person = None                                # [x, y, heading] when person mode is on
        self.person_goal = None
        self.person_speed = float(p('person_speed'))
        self.rate = float(p('rate'))
        if p('person'):
            self.person = [float(p('person_x')), float(p('person_y')), math.pi]   # facing the rover

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.rock_pub = self.create_publisher(MarkerArray, '/rocks', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 lambda msg: self.add_rock(msg.pose.pose.position.x, msg.pose.pose.position.y), 10)
        self.create_timer(1.0 / p('rate'), self.publish_scan)
        self.create_timer(1.0, self.publish_rocks)
        self.get_logger().info(f'Fake lidar ready: {len(self.rocks)} rocks. '
                               'Drop more with "2D Pose Estimate" in RViz.')
        if self.person is not None:
            self.get_logger().info('Person mode: click "Publish Point" in RViz and the person walks there.')

    def on_clicked_point(self, msg):
        if self.person is None:
            self.add_rock(msg.point.x, msg.point.y)
        else:
            self.person_goal = (msg.point.x, msg.point.y)
            self.get_logger().info(f'Person walking to x {msg.point.x:.2f}  y {msg.point.y:.2f}')

    def move_person(self):
        """Walk the person toward their goal, one scan's worth of time."""
        if self.person is None or self.person_goal is None:
            return
        gx, gy = self.person_goal
        dx, dy = gx - self.person[0], gy - self.person[1]
        dist = math.hypot(dx, dy)
        step = self.person_speed / self.rate
        if dist <= step:
            self.person[0], self.person[1] = gx, gy
            self.person_goal = None
        else:
            self.person[0] += dx / dist * step
            self.person[1] += dy / dist * step
            self.person[2] = math.atan2(dy, dx)

    def person_legs(self):
        """The person as the lidar sees them: two legs, 7 cm radius, 24 cm apart."""
        x, y, h = self.person
        nx, ny = -math.sin(h), math.cos(h)
        return np.array([[x + 0.12 * nx, y + 0.12 * ny, 0.07], [x - 0.12 * nx, y - 0.12 * ny, 0.07]])

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.header.stamp)

    def add_rock(self, x, y):
        self.rocks = np.vstack([self.rocks, [x, y, self.clicked_radius]])
        self.get_logger().info(f'Rock added at x {x:.2f}  y {y:.2f}')
        self.publish_rocks()

    def publish_scan(self):
        if self.pose is None:
            return
        x, y, yaw, stamp = self.pose
        sx, sy = x + self.mount_x * math.cos(yaw), y + self.mount_x * math.sin(yaw)   # lidar position
        self.move_person()
        things = self.rocks if self.person is None else np.vstack([self.rocks, self.person_legs()])

        # For every beam and every rock: where does the beam's straight line meet the rock's circle?
        # Point on beam = lidar + t * direction. Solve |point - centre| = radius for t (distance along beam).
        world_angles = yaw + self.beam_angles
        dx, dy = np.cos(world_angles)[:, None], np.sin(world_angles)[:, None]   # beam directions
        cx, cy = things[:, 0] - sx, things[:, 1] - sy                          # rock centres, from the lidar
        r = things[:, 2]
        b = dx * cx + dy * cy                                  # how far along the beam the rock centre is
        gap2 = (cx * cx + cy * cy) - b * b                     # (sideways miss distance)^2
        inside = r * r - gap2                                  # >= 0 means the beam touches the rock
        t = b - np.sqrt(np.clip(inside, 0.0, None))            # distance to the near edge
        t = np.where((inside >= 0) & (t > self.range_min), t, np.inf)
        ranges = t.min(axis=1) if len(things) else np.full(len(self.beam_angles), np.inf)

        hit = np.isfinite(ranges) & (ranges <= self.range_max)
        ranges = np.where(hit, ranges + np.random.normal(0.0, self.noise, ranges.shape), np.inf)
        if self.dropout > 0.0:                    # like the Sweep: many directions get no reading
            ranges = np.where(np.random.random(ranges.shape) < self.dropout, np.nan, ranges)

        scan = LaserScan()
        scan.header.stamp = stamp                 # same moment as the pose we used
        scan.header.frame_id = self.frame_id
        scan.angle_min = float(self.beam_angles[0])
        scan.angle_max = float(self.beam_angles[-1])
        scan.angle_increment = self.angle_increment
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = self.range_min, self.range_max
        scan.ranges = [float(v) for v in ranges]  # inf = beam hit nothing (open ground)
        self.scan_pub.publish(scan)
        if self.person is not None:
            self.publish_rocks()

    def publish_rocks(self):
        markers = MarkerArray()
        for i, (rx, ry, rr) in enumerate(self.rocks):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns, m.id = 'rocks', i
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(rx), float(ry), 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = float(2 * rr)
            m.scale.z = 0.4
            m.color.r, m.color.g, m.color.b, m.color.a = 0.55, 0.45, 0.35, 0.9
            markers.markers.append(m)
        if self.person is not None:
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns, m.id = 'person', 0
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(self.person[0]), float(self.person[1]), 0.85
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4
            m.scale.z = 1.7
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 1.0, 0.8
            markers.markers.append(m)
        self.rock_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
