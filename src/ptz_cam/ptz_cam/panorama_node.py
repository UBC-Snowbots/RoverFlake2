#!/usr/bin/env python3
"""Capture frames on demand and stitch them into a panorama with cv2.Stitcher.

Workflow (all std_srvs/Trigger, so they drop straight onto HMI buttons):

    ros2 service call /ip_camera/snap    std_srvs/srv/Trigger   # rotate, settle, snap
    ros2 service call /ip_camera/snap    std_srvs/srv/Trigger   # ... repeat, left->right
    ros2 service call /ip_camera/stitch  std_srvs/srv/Trigger   # build the panorama
    ros2 service call /ip_camera/reset   std_srvs/srv/Trigger   # clear buffer, start over

Frames are kept in memory (in capture order, which is what cv2.Stitcher wants) and also
written to out_dir as 00.png, 01.png, ... for offline retries. The stitched result is saved
as panorama.png and published latched on ~/panorama so a late-joining HMI still receives it.

Follows the same service-per-action pattern as ip_camera_zoom_node so both can be driven
from the same HMI bindings.
"""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# cv2.Stitcher status code -> human-readable reason.
_STITCH_STATUS = {
    cv2.Stitcher_OK: 'ok',
    cv2.Stitcher_ERR_NEED_MORE_IMGS: 'need more images / not enough overlap between frames',
    cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL: 'homography estimation failed (too little overlap?)',
    cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: 'camera parameter adjustment failed',
}


class PanoramaNode(Node):
    def __init__(self):
        super().__init__('panorama_node')

        self.declare_parameter('image_topic', '/ip_camera/camera/image_raw')
        self.declare_parameter('out_dir', '/tmp/pano')
        # 'panorama' for a rotating camera (default), 'scans' for planar/affine captures.
        self.declare_parameter('stitch_mode', 'panorama')

        self._out_dir = self.get_parameter('out_dir').value
        os.makedirs(self._out_dir, exist_ok=True)

        mode = str(self.get_parameter('stitch_mode').value).lower()
        self._stitch_mode = (
            cv2.Stitcher_SCANS if mode == 'scans' else cv2.Stitcher_PANORAMA
        )

        self._bridge = CvBridge()
        self._latest = None      # most recent frame (bgr8)
        self._frames = []        # captured frames, in capture order

        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self._on_image, 10)

        # Latched so an HMI that subscribes after the stitch still gets the panorama.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pano_pub = self.create_publisher(Image, 'panorama', latched)

        self.create_service(Trigger, 'snap', self._on_snap)
        self.create_service(Trigger, 'stitch', self._on_stitch)
        self.create_service(Trigger, 'reset', self._on_reset)

        self.get_logger().info(
            f'Panorama node ready. Listening on {topic}, saving to {self._out_dir} '
            f'(mode={mode}).'
        )

    def _on_image(self, msg):
        try:
            self._latest = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')

    def _on_snap(self, request, response):
        if self._latest is None:
            response.success = False
            response.message = 'no frames received yet -- is the camera publishing?'
            self.get_logger().warn(response.message)
            return response

        frame = self._latest.copy()
        idx = len(self._frames)
        self._frames.append(frame)

        path = os.path.join(self._out_dir, f'{idx:02d}.png')
        cv2.imwrite(path, frame)

        response.success = True
        response.message = f'captured frame {idx} -> {path}'
        self.get_logger().info(response.message)
        return response

    def _on_stitch(self, request, response):
        if len(self._frames) < 2:
            response.success = False
            response.message = f'need at least 2 frames to stitch, have {len(self._frames)}'
            self.get_logger().warn(response.message)
            return response

        stitcher = cv2.Stitcher_create(self._stitch_mode)
        status, pano = stitcher.stitch(self._frames)

        if status != cv2.Stitcher_OK:
            reason = _STITCH_STATUS.get(status, f'unknown status {status}')
            response.success = False
            response.message = f'stitch failed: {reason}'
            self.get_logger().error(response.message)
            return response

        path = os.path.join(self._out_dir, 'panorama.png')
        cv2.imwrite(path, pano)

        try:
            pano_msg = self._bridge.cv2_to_imgmsg(pano, 'bgr8')
            pano_msg.header.stamp = self.get_clock().now().to_msg()
            self._pano_pub.publish(pano_msg)
        except Exception as exc:
            self.get_logger().error(f'failed to publish panorama: {exc}')

        response.success = True
        response.message = f'stitched {len(self._frames)} frames -> {path} ({pano.shape[1]}x{pano.shape[0]})'
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, request, response):
        n = len(self._frames)
        self._frames.clear()
        response.success = True
        response.message = f'cleared {n} captured frame(s)'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()