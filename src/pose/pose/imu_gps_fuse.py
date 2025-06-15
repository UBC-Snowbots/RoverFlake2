#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# WGS84 Constants
WGS84_A = 6378137.0          # major axis
WGS84_E2 = 6.69437999014e-3  # first eccentricity squared

def latlon_to_ecef(lat, lon, alt):
    """
    Convert geodetic coordinates (latitude, longitude, altitude) to
    Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
    
    lat, lon in radians
    alt in meters
    """
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Radius of curvature in the prime vertical
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * (sin_lat ** 2))

    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt) * sin_lat
    return x, y, z

def ecef_to_enu(x, y, z, x0, y0, z0, lat0, lon0):
    """
    Convert ECEF coordinates (x, y, z) to local ENU coordinates,
    given the local origin's ECEF coords (x0, y0, z0) and latitude/longitude.
    lat0, lon0 in radians (origin).
    """
    # Precompute sines/cosines
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    # Translate so the origin is at (0,0,0)
    dx = x - x0
    dy = y - y0
    dz = z - z0

    # ECEF to ENU
    # [ -sin(lon0),              cos(lon0),              0 ]
    # [ -sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0),  cos(lat0) ]
    # [  cos(lat0)*cos(lon0),  cos(lat0)*sin(lon0),  sin(lat0) ]
    enu_x = -sin_lon0 * dx + cos_lon0 * dy
    enu_y = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    enu_z =  cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz

    return enu_x, enu_y, enu_z


class GpsImuFuse(Node):
    def __init__(self):
        super().__init__('gps_to_path_node')

        # Subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for the path
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # Initialize variables
        self.origin_set = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # Frame ID for the path

        # Store the latest IMU orientation
        self.last_orientation = None

    def gps_callback(self, msg: NavSatFix):
        # Check if valid data
        if msg.status.status < 0:
            self.get_logger().warn("Received invalid GPS fix.")
            return

        # Convert lat/lon to radians
        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

        # If this is the first GPS reading, set the origin
        if not self.origin_set:
            self.lat0 = lat_rad
            self.lon0 = lon_rad
            self.x0, self.y0, self.z0 = latlon_to_ecef(self.lat0, self.lon0, alt)
            self.origin_set = True
            self.get_logger().info(
                f"Origin set to lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={alt:.2f}"
            )

        # Convert current lat/lon/alt to ECEF
        x, y, z = latlon_to_ecef(lat_rad, lon_rad, alt)

        # Convert ECEF to ENU using origin
        enu_x, enu_y, enu_z = ecef_to_enu(
            x, y, z,
            self.x0, self.y0, self.z0,
            self.lat0, self.lon0
        )

        # Create a PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"

        pose_stamped.pose.position.x = enu_x
        pose_stamped.pose.position.y = enu_y
        pose_stamped.pose.position.z = enu_z

        # If we have an IMU orientation, use it. Otherwise, default.
        if self.last_orientation is not None:
            pose_stamped.pose.orientation = self.last_orientation
        else:
            # Simple constant orientation (w=1, others=0)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

        # Append to path
        self.path_msg.header.stamp = pose_stamped.header.stamp
        self.path_msg.poses.append(pose_stamped)

        # Publish the path
        self.path_pub.publish(self.path_msg)
        self.get_logger().info(
            f"Added new point to path: ENU=({enu_x:.2f}, {enu_y:.2f}, {enu_z:.2f})"
        )

    def imu_callback(self, msg: Imu):
        # Store the latest orientation from the IMU
        self.last_orientation = msg.orientation


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuFuse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
