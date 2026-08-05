"""WGS84 <-> local-XY conversion helpers.

Wraps ``swri_transform_util.wgs84_transformer.Wgs84Transformer`` (the Python
bindings shipped with ROS 2 Humble) so waypoint markers can be published in
the same ``map`` frame that ``initialize_origin.py`` anchors. The origin is
the ``/local_xy_origin`` PoseStamped that swri's OriginManager publishes
(position.x = longitude, position.y = latitude, position.z = altitude).
"""
from __future__ import annotations

from typing import Tuple

from geometry_msgs.msg import PoseStamped
from swri_transform_util.wgs84_transformer import Wgs84Transformer


def make_origin(lat: float, lon: float, alt: float = 0.0) -> PoseStamped:
    """Build a /local_xy_origin-style PoseStamped from lat/lon/alt."""
    origin = PoseStamped()
    origin.header.frame_id = 'map'
    origin.pose.position.x = float(lon)
    origin.pose.position.y = float(lat)
    origin.pose.position.z = float(alt)
    origin.pose.orientation.w = 1.0
    return origin


def wgs84_to_local_xy(origin: PoseStamped, lat: float, lon: float) -> Tuple[float, float]:
    """Convert one lat/lon to (x, y) metres in the local_xy (map) frame."""
    tf = Wgs84Transformer(origin)
    x, y = tf.wgs84_to_local_xy([(lat, lon)])[0]
    return float(x), float(y)


def local_xy_to_wgs84(origin: PoseStamped, x: float, y: float) -> Tuple[float, float]:
    """Convert one (x, y) in the local_xy (map) frame to (lat, lon)."""
    tf = Wgs84Transformer(origin)
    lat, lon = tf.local_xy_to_wgs84([(x, y)])[0]
    return float(lat), float(lon)
