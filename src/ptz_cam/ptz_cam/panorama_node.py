#!/usr/bin/env python3
"""Automatic panorama capture for the PTZ camera.

Drives the two gimbal servos (pan + tilt via /ptz/control, see pitch_tilt_node)
through a timed stop-and-go serpentine sweep, grabs a settled frame at each stop,
and stitches the set with cv2.Stitcher per the OpenCV "High level stitching API"
tutorial (docs.opencv.org/4.x/d8/d19/tutorial_stitcher.html).

The gimbal is open-loop (no position feedback), so all motion is timed: aim the
camera at the TOP-LEFT of the desired view before starting. Rows alternate pan
direction; between rows the tilt servo steps down.

Interface:
    srv /ptz/panorama/start    std_srvs/Trigger    begin a sweep (fails if busy)
    srv /ptz/panorama/cancel   std_srvs/Trigger    abort; gimbal is zeroed
    pub /ptz/panorama/status   std_msgs/String     progress; HMI mirrors it verbatim.
                                                   Terminal messages start with
                                                   "done", "error" or "cancelled".
    pub /ptz/panorama/image    sensor_msgs/Image   stitched result (rgb8)
    sub /ip_camera/image_raw   sensor_msgs/Image   live feed (gscam)
    pub /ptz/control           geometry_msgs/Vector3  x=pan, y=tilt, 0 = stop

Saving the result to disk (where, what format, rover vs base station) is left
for discussion — for now the panorama only goes out on the image topic.
"""

import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Sweep state machine phases
IDLE, MOVE, SETTLE, ROW_SHIFT, STITCH = range(5)


class PanoramaNode(Node):
    def __init__(self):
        super().__init__('panorama_node')

        # All timings are open-loop; tune against the real gimbal speed.
        self.declare_parameter('pan_speed', 1.0)        # |x| sent while panning
        self.declare_parameter('tilt_speed', 1.0)       # |y| sent while row-shifting
        self.declare_parameter('steps_per_row', 8)      # capture stops per row
        self.declare_parameter('step_move_secs', 0.6)   # pan time between stops
        self.declare_parameter('settle_secs', 0.4)      # vibration settle before capture
        self.declare_parameter('rows', 1)               # tilt rows (2+ = 2D mosaic)
        self.declare_parameter('row_shift_secs', 0.5)   # tilt time between rows
        self.declare_parameter('image_topic', '/ip_camera/image_raw')

        self._bridge = CvBridge()
        self._frame = None            # latest feed frame (BGR), written by _on_image
        self._frames = []             # captured sweep frames
        self._phase = IDLE
        self._deadline = None         # rclpy Time when the current phase ends
        self._step = 0
        self._row = 0
        self._row_dir = +1            # serpentine: +1 pan right, -1 pan left

        self._ctrl_pub = self.create_publisher(Vector3, '/ptz/control', 10)
        self._status_pub = self.create_publisher(String, '/ptz/panorama/status', 10)
        self._image_pub = self.create_publisher(Image, '/ptz/panorama/image', 1)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._on_image, rclpy.qos.qos_profile_sensor_data)
        self.create_service(Trigger, '/ptz/panorama/start', self._on_start)
        self.create_service(Trigger, '/ptz/panorama/cancel', self._on_cancel)
        # 20 ms tick advances the deadline-driven state machine.
        self.create_timer(0.02, self._tick)

        self.get_logger().info('Panorama node ready.')

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _on_image(self, msg):
        try:
            self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # bad encoding etc. — keep the node alive
            self.get_logger().warn(f'frame conversion failed: {exc}')

    def _on_start(self, request, response):
        if self._phase != IDLE:
            response.success, response.message = False, 'panorama already in progress'
            return response
        if self._frame is None:
            response.success, response.message = False, 'no video feed'
            self._status('error: no video feed')
            return response
        self._frames = []
        self._step = 0
        self._row = 0
        self._row_dir = +1
        self._capture()               # frame 1 is the current aim point
        self._begin_move()
        response.success, response.message = True, 'sweep started'
        return response

    def _on_cancel(self, request, response):
        was_running = self._phase != IDLE
        self._abort('cancelled')
        response.success = was_running
        response.message = 'cancelled' if was_running else 'nothing in progress'
        return response

    # ── State machine ──────────────────────────────────────────────────────

    def _tick(self):
        if self._phase in (IDLE, STITCH) or self.get_clock().now() < self._deadline:
            return
        if self._phase == MOVE:
            self._gimbal(0.0, 0.0)    # stop panning, let the mount settle
            self._enter(SETTLE, self._p('settle_secs'))
        elif self._phase == SETTLE:
            self._capture()
            self._advance()
        elif self._phase == ROW_SHIFT:
            self._gimbal(0.0, 0.0)
            self._enter(SETTLE, self._p('settle_secs'))

    def _advance(self):
        """After a capture: next pan step, next row, or stitch."""
        self._step += 1
        if self._step < int(self._p('steps_per_row')):
            self._begin_move()
            return
        self._row += 1
        if self._row < int(self._p('rows')):
            self._step = 0
            self._row_dir = -self._row_dir            # serpentine turn-around
            self._status(f'row {self._row + 1}: tilting down')
            self._gimbal(0.0, -self._p('tilt_speed'))  # negative y = tilt down
            self._enter(ROW_SHIFT, self._p('row_shift_secs'))
            return
        self._gimbal(0.0, 0.0)
        self._phase = STITCH
        self._status(f'stitching {len(self._frames)} frames…')
        # Stitching costs seconds of CPU — keep it off the executor thread.
        threading.Thread(target=self._stitch, args=(self._frames,), daemon=True).start()

    def _begin_move(self):
        self._status(f'row {self._row + 1}/{int(self._p("rows"))}: '
                     f'frame {self._step + 1}/{int(self._p("steps_per_row"))}')
        self._gimbal(self._row_dir * self._p('pan_speed'), 0.0)
        self._enter(MOVE, self._p('step_move_secs'))

    def _stitch(self, frames):
        try:
            stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
            ok, pano = stitcher.stitch(frames)
            stitched = ok == cv2.Stitcher_OK
            if not stitched:
                # Fallback strip: common height, side by side — still useful.
                h = 480
                pano = cv2.hconcat([cv2.resize(f, (int(f.shape[1] * h / f.shape[0]), h))
                                    for f in frames])
            # TODO(discussion): persist the panorama (path/format/rover-vs-base).
            msg = self._bridge.cv2_to_imgmsg(cv2.cvtColor(pano, cv2.COLOR_BGR2RGB),
                                             encoding='rgb8')
            self._image_pub.publish(msg)
            self._status('done: stitched' if stitched else 'done: unstitched strip (stitch failed)')
        except Exception as exc:
            self._status(f'error: stitch crashed: {exc}')
        finally:
            self._phase = IDLE
            self._frames = []

    # ── Helpers ────────────────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    def _enter(self, phase, secs):
        self._phase = phase
        self._deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=float(secs))

    def _capture(self):
        if self._frame is not None:
            self._frames.append(self._frame.copy())

    def _gimbal(self, pan, tilt):
        msg = Vector3()
        msg.x, msg.y = float(pan), float(tilt)
        self._ctrl_pub.publish(msg)

    def _abort(self, reason):
        self._gimbal(0.0, 0.0)
        self._phase = IDLE
        self._frames = []
        self._status(reason)

    def _status(self, text):
        self._status_pub.publish(String(data=text))
        self.get_logger().info(f'pano: {text}')


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node._gimbal(0.0, 0.0)   # never leave the servos running
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
