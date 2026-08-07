#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge

# Fallback crop used before a dynamic ROI is selected.
ROI_SIZE = 240
ZOOM_FACTOR = 2


class CameraEnhancerNode(Node):
    def __init__(self):
        super().__init__('camera_enhancer')

        self.bridge = CvBridge()

        self.roi = None
        self.last_valid_crop = None

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('enhanced_topic', '/camera/roi_enhanced')
        self.declare_parameter('roi_topic', '/camera/roi_select')
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)

        image_topic = self.get_parameter('image_topic').value
        enhanced_topic = self.get_parameter('enhanced_topic').value
        roi_topic = self.get_parameter('roi_topic').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value

        self.enhanced_pub = self.create_publisher(Image, enhanced_topic, 10)

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # Must match roi_selector.py so late-starting nodes receive the last ROI.
        roi_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.roi_sub = self.create_subscription(
            RegionOfInterest, roi_topic, self.roi_callback, roi_qos)

        self.get_logger().info(
            f'Camera enhancer subscribed to {image_topic} (image) and '
            f'{roi_topic} (ROI), publishing to {enhanced_topic}')

    def roi_callback(self, msg):
        # All-zero ROI is the reset signal shared with roi_selector.py.
        if (msg.x_offset == 0 and msg.y_offset == 0
                and msg.width == 0 and msg.height == 0):
            self.roi = None
            self.last_valid_crop = None
            self.get_logger().info('ROI reset - reverting to default corner crop')
            return

        # Keep the previous ROI rather than passing an empty crop to cv2.resize.
        if msg.width <= 0 or msg.height <= 0:
            self.get_logger().warn(
                f'Ignoring invalid ROI (width={msg.width}, height={msg.height})')
            return
        self.roi = msg

    def image_callback(self, msg):
        try:
            # Zero-copy conversion: cv_bridge hands back a view into msg.data
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = frame.shape[:2]

        if self.roi is None:
            x2, y2 = width, height
            x1 = max(0, x2 - ROI_SIZE)
            y1 = max(0, y2 - ROI_SIZE)

            roi = frame[y1:y2, x1:x2]
            enhanced_roi = cv2.resize(
                roi, None, fx=ZOOM_FACTOR, fy=ZOOM_FACTOR, interpolation=cv2.INTER_LINEAR)
        else:
            # Clamp against the current frame size, which can change at runtime.
            x1 = min(max(0, self.roi.x_offset), width)
            y1 = min(max(0, self.roi.y_offset), height)
            x2 = min(x1 + self.roi.width, width)
            y2 = min(y1 + self.roi.height, height)

            if x2 <= x1 or y2 <= y1:
                if self.last_valid_crop is not None:
                    px1, py1, px2, py2 = self.last_valid_crop
                    px1, py1 = min(px1, width), min(py1, height)
                    px2, py2 = min(px2, width), min(py2, height)
                    if px2 > px1 and py2 > py1:
                        x1, y1, x2, y2 = px1, py1, px2, py2
                    else:
                        x1, y1, x2, y2 = None, None, None, None
                else:
                    x1, y1, x2, y2 = None, None, None, None

                if x1 is None:
                    self.get_logger().warn(
                        'ROI fully outside current frame bounds and no valid '
                        'previous crop to retain; using fallback corner crop')
                    x2f, y2f = width, height
                    x1f = max(0, x2f - ROI_SIZE)
                    y1f = max(0, y2f - ROI_SIZE)
                    roi = frame[y1f:y2f, x1f:x2f]
                else:
                    roi = frame[y1:y2, x1:x2]
            else:
                roi = frame[y1:y2, x1:x2]
                self.last_valid_crop = (x1, y1, x2, y2)

            enhanced_roi = cv2.resize(
                roi, (self.output_width, self.output_height), interpolation=cv2.INTER_LINEAR)

        try:
            enhanced_msg = self.bridge.cv2_to_imgmsg(enhanced_roi, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert enhanced ROI: {e}')
            return

        enhanced_msg.header = msg.header
        self.enhanced_pub.publish(enhanced_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraEnhancerNode()
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
