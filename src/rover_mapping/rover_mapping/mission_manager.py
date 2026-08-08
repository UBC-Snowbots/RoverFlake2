"""Mission manager: the one node that owns a run's lifecycle.

Services (the whole interface — HMI, tag CLI, and autonomy all use these):
  /mission/start   StartMission  create mission dir, start bag + track log
  /mission/stop    Trigger       finalize the bag, close the mission
  /tag_point       TagPoint      save a waypoint (current GPS fix, or a
                                 manually entered coordinate)
  /segment_start   Segment       open a named route leg
  /segment_end     Segment       close a leg (empty name = last open)
  /mission/export  ExportMap     render report/route_map.png (Pillow)

Publishes the active mission name latched on /mission/active so late-joining
UIs recover state. State lives on disk: waypoints/segments are atomic YAML
writes, the fix track is appended to track.csv as it streams, and the bag is
recorded in-process with rosbag2_py — a crash never loses data.
"""
from __future__ import annotations

import datetime
import os
import threading
from typing import List, Optional, TextIO

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rover_mapping_interfaces.srv import (ExportMap, Segment, StartMission,
                                          TagPoint)

from rover_mapping import mission_io, paths
from rover_mapping.exporter import export_mission
from rover_mapping.mission_io import CATEGORIES

LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
TRACK_MIN_SEC = 0.5    # track.csv sample interval


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
                               or paths.missions_root())
        self._stale_sec = (self.get_parameter('stale_fix_sec')
                           .get_parameter_value().double_value)

        self._fix: Optional[NavSatFix] = None
        self._fix_arrival = 0.0
        self._mission_dir: Optional[str] = None
        self._waypoints: List[dict] = []
        self._recorder: Optional[BagRecorder] = None
        self._track: Optional[TextIO] = None
        self._track_last = 0.0

        self.create_subscription(NavSatFix, self._fix_topic,
                                 self._on_fix, 10)
        self._active_pub = self.create_publisher(String, '/mission/active',
                                                 LATCHED)
        self._publish_active()

        self.create_service(StartMission, '/mission/start', self._on_start)
        self.create_service(Trigger, '/mission/stop', self._on_stop)
        self.create_service(TagPoint, '/tag_point', self._on_tag_point)
        self.create_service(Segment, '/segment_start', self._on_seg_start)
        self.create_service(Segment, '/segment_end', self._on_seg_end)
        self.create_service(ExportMap, '/mission/export', self._on_export)

        self.get_logger().info(
            f'ready — fix topic {self._fix_topic}, '
            f'missions in {self._missions_root}')

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _sidecar(self, name: str) -> str:
        assert self._mission_dir is not None
        return os.path.join(self._mission_dir, name)

    def _require_mission(self) -> Optional[str]:
        if self._mission_dir is None:
            return 'no active mission — call /mission/start first'
        return None

    def _publish_active(self) -> None:
        msg = String()
        msg.data = (os.path.basename(self._mission_dir)
                    if self._mission_dir else '')
        self._active_pub.publish(msg)

    # ---------------------------------------------------------- fix + track
    def _on_fix(self, msg: NavSatFix) -> None:
        self._fix = msg
        self._fix_arrival = self.get_clock().now().nanoseconds * 1e-9
        if self._track is None or msg.status.status < 0:
            return
        stamp = stamp_to_float(msg.header.stamp)
        if stamp - self._track_last < TRACK_MIN_SEC:
            return
        self._track_last = stamp
        self._track.write(f'{stamp:.3f},{msg.latitude:.8f},'
                          f'{msg.longitude:.8f},{msg.altitude:.2f}\n')
        self._track.flush()

    def _fresh_fix(self) -> Optional[NavSatFix]:
        # Freshness = arrival age, not header.stamp age: GNSS drivers stamp
        # with GPS time or zero, which can't be compared to the node clock.
        # A tag therefore always lands on the fix the rover just reported.
        if self._fix is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._fix_arrival > self._stale_sec:
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

        os.makedirs(mission_dir, exist_ok=True)
        self._mission_dir = mission_dir
        self._waypoints = []
        mission_io.atomic_dump_yaml(self._sidecar('waypoints.yaml'), [])
        mission_io.atomic_dump_yaml(self._sidecar('segments.yaml'), [])
        with open(self._sidecar('mission.yaml'), 'w') as f:
            f.write(f'name: {name}\nsite: {self._site}\n'
                    f'date: {datetime.datetime.now().isoformat()}\n')
        self._track = open(self._sidecar('track.csv'), 'w')
        self._track.write('stamp,lat,lon,alt\n')
        self._track_last = 0.0
        self._recorder = BagRecorder(os.path.join(mission_dir, 'rosbag2'),
                                     [self._fix_topic])
        self._publish_active()

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
        self._publish_active()
        self.get_logger().info(f'mission stopped: {mission_dir}')
        res.success = True
        res.message = (f'saved {mission_dir} '
                       f'({len(self._waypoints)} waypoints)')
        return res

    def _stop_recording(self) -> None:
        if self._recorder is not None:
            self._recorder.stop()
            self._recorder = None
        if self._track is not None:
            self._track.close()
            self._track = None

    # ---------------------------------------------------------------- tagging
    def _on_tag_point(self, req: TagPoint.Request,
                      res: TagPoint.Response) -> TagPoint.Response:
        error = self._require_mission()
        category = req.category.strip().lower()
        if error is None and category not in CATEGORIES:
            error = (f'unknown category "{req.category}" '
                     f'(want one of {", ".join(CATEGORIES)})')
        # Manual coordinates bypass the fix entirely: the point is somewhere
        # the rover isn't (yet), so fix age says nothing about its validity.
        manual = bool(req.use_manual)
        fix = None if manual else self._fresh_fix()
        if error is None and manual:
            error = mission_io.coord_error(req.manual_lat, req.manual_lon)
        if error is None and not manual and fix is None:
            error = ('no GPS fix yet' if self._fix is None
                     else f'fix is stale (> {self._stale_sec:.0f} s)')
        if error is not None:
            res.ok = False
            res.message = error
            return res
        assert manual or fix is not None

        # id from the file, not the in-memory cache — stays collision-free
        # even if another writer (CLI, stray node) appended since our last tag
        wp_id = mission_io.next_waypoint_id(
            mission_io.load_yaml_list(self._sidecar('waypoints.yaml')), category)
        label = req.label.strip() or mission_io.default_label(wp_id)
        waypoint = {
            'id': wp_id,
            'label': label,
            'category': category,
            'lat': float(req.manual_lat) if manual else float(fix.latitude),
            'lon': float(req.manual_lon) if manual else float(fix.longitude),
            'alt': 0.0 if manual else float(fix.altitude),
            'stamp': (self.get_clock().now().nanoseconds * 1e-9 if manual
                      else stamp_to_float(fix.header.stamp)),
            'source': 'manual' if manual else 'fix',
            'notes': req.notes,
        }
        self._waypoints = mission_io.append_waypoint(
            self._sidecar('waypoints.yaml'), waypoint)
        self.get_logger().info(
            f'tagged {wp_id} "{label}" at '
            f'({waypoint["lat"]:.6f}, {waypoint["lon"]:.6f})'
            f'{" (manual)" if manual else ""}')
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

    # ----------------------------------------------------------------- export
    def _on_export(self, req: ExportMap.Request,
                   res: ExportMap.Response) -> ExportMap.Response:
        try:
            mission = paths.resolve_mission(req.mission.strip())
            res.path = export_mission(mission, site=self._site or None)
            res.ok = True
            res.message = f'wrote {res.path} (+ mission.geojson, poi.csv)'
            self.get_logger().info(res.message)
        except Exception as e:   # noqa: BLE001 — a service must always respond
            res.ok = False
            res.message = f'{type(e).__name__}: {e}'
        return res


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
