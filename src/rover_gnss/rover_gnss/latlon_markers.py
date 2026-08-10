"""Show named lat/lon points in RViz2 alongside the live nmea_reader fix.

RViz2 has no NavSatFix display, so everything is projected into a local ENU
frame (flat-earth, fine over a few km) and published as one MarkerArray:

    /gnss_points   visualization_msgs/MarkerArray

The array holds the waypoint spheres, one text label per waypoint, the
rover's current position, and the base station, all on that one topic:

    cyan spheres   named waypoints from params
    orange sphere  rover, from `fix_topic` (nmea_reader's /gnss_fix)
    yellow cube    base station, from `base_topic` (nmea_reader's /base_fix)

The ENU origin is the `datum` param if set, otherwise the base station's
position once it arrives, otherwise (after `base_datum_timeout` seconds)
the first waypoint, otherwise the first rover fix.

Waypoints are two parallel params — coordinates as lat/lon pairs, names in
matching order. Unnamed points fall back to "wp0", "wp1", ...

    ros2 run rover_gnss latlon_markers --ros-args \
        -p waypoints:="[49.262680, -123.265137, 49.263100, -123.264500]" \
        -p waypoint_names:="[start, gate]" \
        -p datum:="[49.262680, -123.265137]"

or from a params yaml:

    latlon_markers:
      ros__parameters:
        waypoint_names: [start, gate]
        waypoints: [49.262680, -123.265137,
                    49.263100, -123.264500]

In RViz: Add -> MarkerArray, topic /gnss_points, fixed frame "map".

For a hardware-free demo of all of the above:

    ros2 launch rover_gnss latlon_markers_demo.launch.py
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

M_PER_DEG_LAT = 111_320.0

WAYPOINT_COLOR = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)  # cyan
ROVER_COLOR = ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0)     # orange
BASE_COLOR = ColorRGBA(r=0.9, g=0.9, b=0.2, a=1.0)      # yellow
LABEL_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


class LatLonMarkers(Node):
    def __init__(self) -> None:
        super().__init__('latlon_markers')

        # flat [lat, lon, lat, lon, ...] so it stays a plain double array param
        self.declare_parameter('waypoints', [49.262680, -123.265137])
        self.declare_parameter('waypoint_names', [''])
        self.declare_parameter('datum', [0.0, 0.0])  # [0,0] => see below
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('fix_topic', '/gnss_fix')
        self.declare_parameter('base_topic', '/base_fix')
        self.declare_parameter('base_datum_timeout', 5.0)
        self.declare_parameter('point_size', 1.0)
        self.declare_parameter('label_size', 0.8)

        flat = list(self.get_parameter('waypoints')
                    .get_parameter_value().double_array_value)
        if len(flat) % 2:
            self.get_logger().warn(
                'waypoints has an odd number of values, dropping the last one')
            flat = flat[:-1]
        names = [n for n in self.get_parameter('waypoint_names')
                 .get_parameter_value().string_array_value if n]

        # (name, lat, lon), names padded out if fewer were given than points
        self._waypoints = [
            (names[i] if i < len(names) else f'wp{i}', lat, lon)
            for i, (lat, lon) in enumerate(zip(flat[0::2], flat[1::2]))
        ]
        if len(names) > len(self._waypoints):
            self.get_logger().warn(
                f'{len(names)} names for {len(self._waypoints)} waypoints, '
                'the extra names are ignored')

        # datum priority: the param, else the base station, else (after
        # base_datum_timeout) the first waypoint, else the first rover fix
        datum = list(self.get_parameter('datum')
                     .get_parameter_value().double_array_value)
        self._datum: tuple[float, float] | None = None
        if len(datum) == 2 and (datum[0] or datum[1]):
            self._datum = (datum[0], datum[1])
            self.get_logger().info(
                f'datum from param: {datum[0]:.6f}, {datum[1]:.6f}')
        self._base_deadline = (
            self.get_clock().now().nanoseconds * 1e-9
            + self.get_parameter('base_datum_timeout')
            .get_parameter_value().double_value)

        self._frame = (self.get_parameter('frame_id')
                       .get_parameter_value().string_value)
        self._size = (self.get_parameter('point_size')
                      .get_parameter_value().double_value)
        self._label_size = (self.get_parameter('label_size')
                            .get_parameter_value().double_value)

        self._fix: tuple[float, float] | None = None   # latest rover lat/lon
        self._base: tuple[float, float] | None = None  # base station lat/lon

        self._pub = self.create_publisher(MarkerArray, '/gnss_points', 10)

        # RViz refuses to render anything when its fixed frame is absent from
        # the TF tree, so put the frame there ourselves. Also gives the
        # frame_id="gnss" stamps out of nmea_reader a parent.
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._frame
        tf.child_frame_id = 'gnss'
        tf.transform.rotation.w = 1.0
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(tf)

        fix_topic = (self.get_parameter('fix_topic')
                     .get_parameter_value().string_value)
        base_topic = (self.get_parameter('base_topic')
                      .get_parameter_value().string_value)
        self.create_subscription(NavSatFix, fix_topic, self._on_fix, 10)
        self.create_subscription(NavSatFix, base_topic, self._on_base, 10)

        # re-sent on a timer so RViz picks the array up whenever it connects,
        # no transient-local QoS games needed
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            f'{len(self._waypoints)} waypoint(s) in frame "{self._frame}", '
            f'rover from {fix_topic}, datum={self._datum}')

    # lat/lon -> local ENU metres (east, north) about the datum
    def _to_local(self, lat: float, lon: float) -> tuple[float, float]:
        lat0, lon0 = self._datum
        m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat0))
        return (lon - lon0) * m_per_deg_lon, (lat - lat0) * M_PER_DEG_LAT

    def _marker(self, ns: str, mid: int, mtype: int) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._frame
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _label(self, ns: str, mid: int, text: str,
               east: float, north: float) -> Marker:
        m = self._marker(ns, mid, Marker.TEXT_VIEW_FACING)
        m.pose.position = Point(x=east, y=north, z=self._size)
        m.scale.z = self._label_size
        m.color = LABEL_COLOR
        m.text = text
        return m

    def _on_base(self, msg: NavSatFix) -> None:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        self._base = (msg.latitude, msg.longitude)
        if self._datum is None:
            self._datum = self._base
            self.get_logger().info(
                f'datum from base station: {msg.latitude:.6f}, '
                f'{msg.longitude:.6f}')
            self._publish()

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if self._datum is None:
            self._datum = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'datum set from first fix: {msg.latitude:.6f}, '
                f'{msg.longitude:.6f}')
        self._fix = (msg.latitude, msg.longitude)
        self._publish()

    def _publish(self) -> None:
        if self._datum is None:
            # base station never showed up: fall back to the first waypoint
            now = self.get_clock().now().nanoseconds * 1e-9
            if now < self._base_deadline or not self._waypoints:
                return
            self._datum = self._waypoints[0][1:]
            self.get_logger().warn(
                f'no base fix after {self._base_deadline - now:.0f}s, datum '
                f'falls back to waypoint "{self._waypoints[0][0]}"')

        arr = MarkerArray()

        spheres = self._marker('waypoints', 0, Marker.SPHERE_LIST)
        spheres.scale.x = spheres.scale.y = spheres.scale.z = self._size
        spheres.color = WAYPOINT_COLOR
        for i, (name, lat, lon) in enumerate(self._waypoints):
            east, north = self._to_local(lat, lon)
            spheres.points.append(Point(x=east, y=north, z=0.0))
            arr.markers.append(
                self._label('waypoint_labels', i, name, east, north))
        arr.markers.append(spheres)

        if self._fix is not None:
            east, north = self._to_local(*self._fix)
            rover = self._marker('rover', 0, Marker.SPHERE)
            rover.pose.position = Point(x=east, y=north, z=0.0)
            rover.scale.x = rover.scale.y = rover.scale.z = self._size * 1.2
            rover.color = ROVER_COLOR
            arr.markers.append(rover)
            arr.markers.append(
                self._label('rover', 1, 'rover', east, north))

        if self._base is not None:
            east, north = self._to_local(*self._base)
            base = self._marker('base', 0, Marker.CUBE)
            base.pose.position = Point(x=east, y=north, z=0.0)
            base.scale.x = base.scale.y = base.scale.z = self._size
            base.color = BASE_COLOR
            arr.markers.append(base)
            arr.markers.append(
                self._label('base', 1, 'base', east, north))

        self._pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatLonMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
