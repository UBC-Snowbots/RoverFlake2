#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geographiclib.geodesic import Geodesic
import math
"""
ros2 topic pub --once /waypoint_gnss sensor_msgs/msg/NavSatFix "{
  header: {frame_id: 'gps'},
  latitude: 51.241240,
  longitude: -121.124124,
  altitude: 0.0,
  position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  position_covariance_type: 0
}"
"""

class LatLonMarkerTool(Node):
    def __init__(self):
        super().__init__('latlon_marker_tool')

        # Subscribe to rover's GNSS to set local origin
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.gnss_origin_callback, 1)

        # Subscribe to GNSS waypoints from user
        self.wp_sub = self.create_subscription(NavSatFix, '/waypoint_gnss', self.waypoint_callback, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/waypoints', 10)

        self.origin_set = False
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.marker_id = 0

    def gnss_origin_callback(self, msg: NavSatFix):
        if not self.origin_set and msg.status.status != -1:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.origin_set = True
            self.get_logger().info(f"Origin set to lat={self.origin_lat:.8f}, lon={self.origin_lon:.8f}")

    def waypoint_callback(self, msg: NavSatFix):
        if not self.origin_set:
            self.get_logger().warn("Origin not set yet, ignoring waypoint.")
            return

        x, y = self.latlon_to_enu(msg.latitude, msg.longitude)
        self.publish_marker(x, y)
        self.get_logger().info(f"Published waypoint: lat={msg.latitude:.8f}, lon={msg.longitude:.8f} "
                               f"(ENU: x={x:.2f}, y={y:.2f})")

    def latlon_to_enu(self, lat, lon):
        g = Geodesic.WGS84.Inverse(self.origin_lat, self.origin_lon, lat, lon)
        azimuth = math.radians(g['azi1'])
        distance = g['s12']  # meters
        x = distance * math.sin(azimuth)  # East
        y = distance * math.cos(azimuth)  # North
        return x, y

    def publish_marker(self, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'waypoints'
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime.sec = 0  # persistent
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = LatLonMarkerTool()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
