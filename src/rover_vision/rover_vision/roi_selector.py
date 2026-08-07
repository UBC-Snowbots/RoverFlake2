#!/usr/bin/env python3
"""
roi_selector.py

Prototype OpenCV node for selecting a camera ROI with the mouse.

The selected box is published as sensor_msgs/RegionOfInterest on roi_topic.
Pressing 'r' publishes an all-zero ROI, which camera_enhancer.py treats as a
reset signal.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge

MIN_BOX_SIZE = 10
COLOR_IN_PROGRESS = (0, 255, 255)
COLOR_COMMITTED = (0, 255, 0)


class RoiSelectorNode(Node):
    def __init__(self):
        super().__init__('roi_selector')

        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('roi_topic', '/camera/roi_select')
        self.declare_parameter('window_name', 'ROI Selector')

        self.image_topic = self.get_parameter('image_topic').value
        self.roi_topic = self.get_parameter('roi_topic').value
        self.window_name = self.get_parameter('window_name').value

        # Must match camera_enhancer.py or the ROI topic will not connect.
        roi_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.roi_pub = self.create_publisher(RegionOfInterest, self.roi_topic, roi_qos)

        self.dragging = False
        self.anchor = None
        self.current = None
        self.committed_box = None

        self.gui_ready = self._setup_gui()
        if not self.gui_ready:
            return

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f'ROI selector subscribed to {self.image_topic}, publishing ROIs to {self.roi_topic}. '
            f"Drag with the left mouse button to select a box, 'r' to reset, 'q'/ESC to quit.")

    def _setup_gui(self):
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.window_name, self._on_mouse)
        except cv2.error as e:
            self.get_logger().error(
                'Failed to create an OpenCV display window. This node needs a '
                'reachable display/X server (e.g. run natively on a Linux '
                'desktop, or forward X11 to a host X server such as VcXsrv '
                'when running in a container) and an OpenCV build with GUI '
                f'support (not opencv-python-headless). Underlying error: {e}')
            return False
        except Exception as e:
            self.get_logger().error(
                f'Unexpected error setting up the ROI selector GUI: {e}')
            return False
        return True

    def _on_mouse(self, event, x, y, flags, param):
        # Window pixels map to image pixels because the frame is shown 1:1.
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.anchor = (x, y)
            self.current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.dragging:
                self.current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            if self.dragging:
                self.dragging = False
                self.current = (x, y)
                self._commit_drag()

    def _commit_drag(self):
        if self.anchor is None or self.current is None:
            return

        ax, ay = self.anchor
        cx, cy = self.current

        x1, x2 = min(ax, cx), max(ax, cx)
        y1, y2 = min(ay, cy), max(ay, cy)

        if (x2 - x1) < MIN_BOX_SIZE or (y2 - y1) < MIN_BOX_SIZE:
            return

        self.committed_box = (x1, y1, x2, y2)

        roi = RegionOfInterest()
        roi.x_offset = int(x1)
        roi.y_offset = int(y1)
        roi.width = int(x2 - x1)
        roi.height = int(y2 - y1)
        roi.do_rectify = False
        self.roi_pub.publish(roi)
        self.get_logger().info(
            f'Published ROI: x={roi.x_offset} y={roi.y_offset} '
            f'w={roi.width} h={roi.height}')

    def _reset(self):
        self.committed_box = None
        roi = RegionOfInterest()
        roi.x_offset = 0
        roi.y_offset = 0
        roi.width = 0
        roi.height = 0
        roi.do_rectify = False
        self.roi_pub.publish(roi)
        self.get_logger().info('ROI reset - camera_enhancer will revert to its default corner crop')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        display = frame.copy()

        if self.dragging and self.anchor is not None and self.current is not None:
            ax, ay = self.anchor
            cx, cy = self.current
            x1, x2 = min(ax, cx), max(ax, cx)
            y1, y2 = min(ay, cy), max(ay, cy)
            cv2.rectangle(display, (x1, y1), (x2, y2), COLOR_IN_PROGRESS, 2)

        if self.committed_box is not None:
            x1, y1, x2, y2 = self.committed_box
            cv2.rectangle(display, (x1, y1), (x2, y2), COLOR_COMMITTED, 2)

        try:
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as e:
            self.get_logger().error(
                f'Lost the ability to display frames (display/X server gone?): {e}')
            return

        if key == ord('r'):
            self._reset()
        elif key == ord('q') or key == 27:  # 27 == ESC
            self.get_logger().info('Quit key pressed, shutting down ROI selector')
            cv2.destroyWindow(self.window_name)
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = RoiSelectorNode()
    try:
        if node.gui_ready:
            rclpy.spin(node)
        else:
            node.get_logger().error('ROI selector GUI unavailable - shutting down without spinning')
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
