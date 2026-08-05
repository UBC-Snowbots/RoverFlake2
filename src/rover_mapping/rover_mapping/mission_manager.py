"""Mission manager: the one node that owns a run's lifecycle.

Services (all usable from other nodes — HMI, autonomy — or `rover`):
  /mission/start   StartMission  create mission dir, start bag recording
  /mission/stop    Trigger       finalize the bag, close the mission
  /tag_point       TagPoint      save current GPS position as a waypoint
  /segment_start   Segment       open a named route leg
  /segment_end     Segment       close a leg (empty name = last open)

State lives on disk, not in the node: every tag is an atomic YAML write
into the mission directory, and the bag is recorded in-process with
rosbag2_py — a crash or Ctrl-C never loses data. /waypoints markers are
published latched so mapviz shows them whenever it joins.
"""
from __future__ import annotations

import datetime
import os
import threading
from typing import List, Optional

import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from rover_mapping_interfaces.srv import Segment, StartMission, TagPoint

from rover_mapping import mission_io
from rover_mapping.launch_helpers import source_root
from rover_mapping.local_xy import wgs84_to_local_xy
from rover_mapping.mission_io import CATEGORIES, CATEGORY_COLORS

LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class BagRecorder:
    """In-process rosbag2 recording with a clean stop."""

    def __init__(self, bag_dir: str, topics: List[str]) -> None:
        self._recorder = rosbag2_py.Recorder()
        storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='')
        record = rosbag2_py.RecordOptions()
        record.topics = topics
        self._thread = threading.Thread(
            target=self._recorder.record, args=(storage, record),
            daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._recorder.cancel()
        self._thread.join(timeout=5.0)


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')
        self.declare_parameter('site', '')
        self.declare_parameter('fix_topic', '/gnss_fix')
        self.declare_parameter('missions_root', '')
        self.declare_parameter('stale_fix_sec', 2.0)

        self._site = self._str_param('site')
        self._fix_topic = self._str_param('fix_topic')
        self._missions_root = (self._str_param('missions_root')
                               or os.path.join(source_root(), 'missions'))
        self._stale_sec = (self.get_parameter('stale_fix_sec')
                           .get_parameter_value().double_value)

        self._fix: Optional[NavSatFix] = None
        self._origin: Optional[PoseStamped] = None
        self._mission_dir: Optional[str] = None
        self._waypoints: List[dict] = []
        self._recorder: Optional[BagRecorder] = None

        self.create_subscription(NavSatFix, self._fix_topic,
                                 self._on_fix, 10)
        self.create_subscription(PoseStamped, '/local_xy_origin',
                                 self._on_origin, LATCHED)
        self._marker_pub = self.create_publisher(MarkerArray, '/waypoints',
                                                 LATCHED)

        self.create_service(StartMission, '/mission/start', self._on_start)
        self.create_service(Trigger, '/mission/stop', self._on_stop)
        self.create_service(TagPoint, '/tag_point', self._on_tag_point)
        self.create_service(Segment, '/segment_start', self._on_seg_start)
        self.create_service(Segment, '/segment_end', self._on_seg_end)

        self.get_logger().info(
            f'ready — site "{self._site}", fix topic {self._fix_topic}, '
            f'missions in {self._missions_root}')

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    # ------------------------------------------------------------ mission dir
    def _sidecar(self, name: str) -> str:
        assert self._mission_dir is not None
        return os.path.join(self._mission_dir, name)

    def _require_mission(self) -> Optional[str]:
        """Error text if no mission is active, else None."""
        if self._mission_dir is None:
            return 'no active mission — call /mission/start first'
        return None

    # ---------------------------------------------------------- subscriptions
    def _on_fix(self, msg: NavSatFix) -> None:
        self._fix = msg

    def _on_origin(self, msg: PoseStamped) -> None:
        self._origin = msg
        if self._mission_dir is not None:
            self._write_origin_yaml()
        self._publish_markers()

    def _write_origin_yaml(self) -> None:
        assert self._origin is not None
        mission_io.atomic_dump_yaml(self._sidecar('origin.yaml'), [{
            'lat': self._origin.pose.position.y,
            'lon': self._origin.pose.position.x,
            'alt': self._origin.pose.position.z,
        }])

    def _fresh_fix(self) -> Optional[NavSatFix]:
        if self._fix is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp_to_float(self._fix.header.stamp) > self._stale_sec:
            return None
        return self._fix

    # ------------------------------------------------------- mission lifecycle
    def _on_start(self, req: StartMission.Request,
                  res: StartMission.Response) -> StartMission.Response:
        if self._mission_dir is not None:
            res.ok = False
            res.message = (f'mission already running in {self._mission_dir}'
                           ' — /mission/stop it first')
            return res
        name = req.name.strip()
        if not name:
            res.ok = False
            res.message = 'mission name is required'
            return res

        date = datetime.date.today().isoformat()
        mission_dir = os.path.join(self._missions_root, f'{date}_{name}')
        if os.path.exists(os.path.join(mission_dir, 'rosbag2')):
            res.ok = False
            res.message = f'{mission_dir} already recorded — pick a new name'
            return res

        os.makedirs(os.path.join(mission_dir, 'report'), exist_ok=True)
        self._mission_dir = mission_dir
        self._waypoints = []
        mission_io.atomic_dump_yaml(self._sidecar('waypoints.yaml'), [])
        mission_io.atomic_dump_yaml(self._sidecar('segments.yaml'), [])
        with open(self._sidecar('mission.yaml'), 'w') as f:
            f.write(f'name: {name}\nsite: {self._site}\n'
                    f'date: {datetime.datetime.now().isoformat()}\n')
        if self._origin is not None:
            self._write_origin_yaml()

        self._recorder = BagRecorder(
            os.path.join(mission_dir, 'rosbag2'),
            [self._fix_topic, '/waypoints', '/tf', '/tf_static',
             '/local_xy_origin', '/diagnostics'])
        self._publish_markers()   # clear leftovers from a previous mission

        self.get_logger().info(f'mission started: {mission_dir}')
        res.ok = True
        res.message = f'recording {mission_dir}'
        res.mission_dir = mission_dir
        return res

    def _on_stop(self, _req: Trigger.Request,
                 res: Trigger.Response) -> Trigger.Response:
        if self._mission_dir is None:
            res.success = False
            res.message = 'no active mission'
            return res
        mission_dir = self._mission_dir
        self._stop_recording()
        self._mission_dir = None
        self.get_logger().info(f'mission stopped: {mission_dir}')
        res.success = True
        res.message = (f'saved {mission_dir} '
                       f'({len(self._waypoints)} waypoints)')
        return res

    def _stop_recording(self) -> None:
        if self._recorder is not None:
            self._recorder.stop()
            self._recorder = None

    # ---------------------------------------------------------------- tagging
    def _on_tag_point(self, req: TagPoint.Request,
                      res: TagPoint.Response) -> TagPoint.Response:
        error = self._require_mission()
        category = req.category.strip().lower()
        if error is None and category not in CATEGORIES:
            error = (f'unknown category "{req.category}" '
                     f'(want one of {", ".join(CATEGORIES)})')
        fix = self._fresh_fix()
        if error is None and fix is None:
            error = ('no GPS fix yet' if self._fix is None
                     else f'fix is stale (> {self._stale_sec:.0f} s)')
        if error is not None:
            res.ok = False
            res.message = error
            return res
        assert fix is not None

        wp_id = mission_io.next_waypoint_id(self._waypoints, category)
        label = req.label.strip() or mission_io.default_label(wp_id)
        waypoint = {
            'id': wp_id,
            'label': label,
            'category': category,
            'lat': float(fix.latitude),
            'lon': float(fix.longitude),
            'alt': float(fix.altitude),
            'stamp': stamp_to_float(fix.header.stamp),
            'notes': req.notes,
        }
        self._waypoints = mission_io.append_waypoint(
            self._sidecar('waypoints.yaml'), waypoint)
        self._publish_markers()
        self.get_logger().info(
            f'tagged {wp_id} "{label}" at '
            f'({waypoint["lat"]:.6f}, {waypoint["lon"]:.6f})')
        res.ok = True
        res.message = f'saved {wp_id}'
        res.id = wp_id
        res.lat = waypoint['lat']
        res.lon = waypoint['lon']
        return res

    # --------------------------------------------------------------- segments
    def _now_stamp(self) -> float:
        fix = self._fresh_fix()
        if fix is not None:
            return stamp_to_float(fix.header.stamp)
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_seg_start(self, req: Segment.Request,
                      res: Segment.Response) -> Segment.Response:
        error = self._require_mission()
        if error is not None:
            res.ok = False
            res.message = error
            return res
        name = req.name.strip()
        if not name:
            n = len(mission_io.load_yaml_list(self._sidecar('segments.yaml')))
            name = f'segment_{n + 1}'
        mission_io.open_segment(self._sidecar('segments.yaml'), name,
                                self._now_stamp())
        self.get_logger().info(f'segment "{name}" started')
        res.ok = True
        res.message = f'segment "{name}" started'
        res.name = name
        return res

    def _on_seg_end(self, req: Segment.Request,
                    res: Segment.Response) -> Segment.Response:
        error = self._require_mission()
        if error is not None:
            res.ok = False
            res.message = error
            return res
        closed = mission_io.close_segment(self._sidecar('segments.yaml'),
                                          self._now_stamp(),
                                          req.name.strip() or None)
        if closed is None:
            res.ok = False
            res.message = 'no open segment to end'
            return res
        self.get_logger().info(f'segment "{closed}" ended')
        res.ok = True
        res.message = f'segment "{closed}" ended'
        res.name = closed
        return res

    # ---------------------------------------------------------------- markers
    def _publish_markers(self) -> None:
        if self._origin is None:
            return
        arr = MarkerArray()
        wipe = Marker()
        wipe.header.frame_id = 'map'
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)

        now = self.get_clock().now().to_msg()
        for i, wp in enumerate(self._waypoints):
            x, y = wgs84_to_local_xy(self._origin, wp['lat'], wp['lon'])
            r, g, b, a = CATEGORY_COLORS[wp['category']]

            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'waypoints'
            sphere.id = 2 * i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.5
            sphere.color.r, sphere.color.g = r, g
            sphere.color.b, sphere.color.a = b, a
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'labels'
            text.id = 2 * i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 1.5
            text.pose.orientation.w = 1.0
            text.scale.z = 1.2
            text.color.r, text.color.g = r, g
            text.color.b, text.color.a = b, a
            text.text = wp['label']
            arr.markers.append(text)

        self._marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node._stop_recording()   # finalize the bag on any shutdown path
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
