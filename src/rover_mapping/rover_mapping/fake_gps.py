"""Bench-test GPS publisher: walks a small loop with ~0.5 m jitter at 5 Hz.

    ros2 run rover_mapping fake_gps
    ros2 run rover_mapping fake_gps --ros-args -p lat:=49.2621 \
        -p lon:=-123.2488
"""
from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

M_PER_DEG_LAT = 111_320.0  # good enough for a bench-test wander


class FakeGps(Node):
    def __init__(self) -> None:
        super().__init__('fake_gps')
        # default start: inside the ubc_test tile set, so bench tests
        # land on real imagery
        self.declare_parameter('lat', 49.262680)
        self.declare_parameter('lon', -123.265137)
        self.declare_parameter('alt', 70.0)
        self.declare_parameter('topic', '/gnss_fix')
        self.declare_parameter('radius_m', 8.0)       # loop radius
        self.declare_parameter('speed_mps', 1.0)
        self.declare_parameter('jitter_m', 0.5)

        def fparam(name: str) -> float:
            return self.get_parameter(name).get_parameter_value().double_value

        self._lat0 = fparam('lat')
        self._lon0 = fparam('lon')
        self._alt = fparam('alt')
        self._radius = fparam('radius_m')
        self._speed = fparam('speed_mps')
        self._jitter = fparam('jitter_m')
        self._t = 0.0

        topic = (self.get_parameter('topic')
                 .get_parameter_value().string_value)
        self._pub = self.create_publisher(NavSatFix, topic, 10)
        self._timer = self.create_timer(0.2, self._tick)  # 5 Hz
        self.get_logger().info(
            f'fake GPS on {topic}: loop r={self._radius} m around '
            f'({self._lat0:.6f}, {self._lon0:.6f})')

    def _tick(self) -> None:
        self._t += 0.2
        # circular walk + gaussian jitter, all in metres then converted to deg
        theta = self._speed * self._t / self._radius
        dx = self._radius * math.cos(theta) + random.gauss(0.0, self._jitter)
        dy = self._radius * math.sin(theta) + random.gauss(0.0, self._jitter)
        m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(self._lat0))

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self._lat0 + dy / M_PER_DEG_LAT
        msg.longitude = self._lon0 + dx / m_per_deg_lon
        msg.altitude = self._alt + random.gauss(0.0, self._jitter)
        msg.position_covariance = [0.25, 0.0, 0.0,
                                   0.0, 0.25, 0.0,
                                   0.0, 0.0, 1.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeGps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
