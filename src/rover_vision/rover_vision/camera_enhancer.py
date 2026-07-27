#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Default frame dimensions this node is tuned for (standard 1080p feed)
FRAME_WIDTH = 480
FRAME_HEIGHT = 640

# ROI size for the bottom-right corner box
ROI_SIZE = 240

# Digital zoom factor applied to the extracted ROI
ZOOM_FACTOR = 2


class CameraEnhancerNode(Node):
    def __init__(self):
        super().__init__('camera_enhancer')

        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('enhanced_topic', '/camera/roi_enhanced')

        image_topic = self.get_parameter('image_topic').value
        enhanced_topic = self.get_parameter('enhanced_topic').value
        self.enhanced_pub = self.create_publisher(Image, enhanced_topic, 10)

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        self.get_logger().info(
            f'Camera enhancer subscribed to {image_topic}, publishing to {enhanced_topic}')

    def image_callback(self, msg):
        try:
            # Zero-copy conversion: cv_bridge hands back a view into msg.data
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = frame.shape[:2]

        # Bottom-right corner box, clamped to the actual frame size
        x2, y2 = width, height
        x1 = max(0, x2 - ROI_SIZE)
        y1 = max(0, y2 - ROI_SIZE)

        # NumPy slicing yields a view, not a copy - no full-frame duplication
        roi = frame[y1:y2, x1:x2]
        enhanced_roi = cv2.resize(
            roi, None, fx=ZOOM_FACTOR, fy=ZOOM_FACTOR, interpolation=cv2.INTER_LINEAR)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
