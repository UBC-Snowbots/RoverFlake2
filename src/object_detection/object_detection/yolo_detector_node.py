#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from ultralytics import YOLO


class YOLODetectorNode(Node):
    """
        Run yolo on a detected camera feed

        How to use:
            first run: 'ros2 run rover_vision camera_pub_node'  
            then run: 'ros2 run object_detection yolo_detector_node' 
            now to display the results: 'ros2 run rqt_image_view rqt_image_view'

            Make sure to build and source before running
    """
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.publisher_ = self.create_publisher(Image, 'camera/yolo/object_detection', 30) # Changed the topic name for some more clarity
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Realsense raw
            self.image_callback,
            10)
        self.yolo = YOLO('bestFullData.pt')

    def get_colours(self, cls_num):
        base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        color_index = cls_num % len(base_colors)
        increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
        color = [
            base_colors[color_index][i] + increments[color_index][i] * 
            (cls_num // len(base_colors)) % 256 for i in range(3)
        ]
        return tuple(color)

    def image_callback(self, msg): 
        # Convert ROS Image message to OpenCV image
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Process frame with YOLO
        results = self.yolo.track(frame, stream=True)
        
        # Draw detections on frame
        annotated_frame = frame.copy()
        for result in results:
            classes_names = result.names
            for box in result.boxes:
                if box.conf[0] > 0.4:
                    [x1, y1, x2, y2] = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cls = int(box.cls[0])
                    class_name = classes_names[cls]
                    colour = self.get_colours(cls)

                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), colour, 2)
                    cv2.putText(annotated_frame, f'{class_name} {box.conf[0]:.2f}',
                                (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
        
        # Convert frame to ROS2 Image message and publish
        detected_msg = Image()
        detected_msg.header = msg.header  # Preserve original timestamp and frame_id
        detected_msg.height = annotated_frame.shape[0]
        detected_msg.width = annotated_frame.shape[1]
        detected_msg.encoding = 'bgr8'
        detected_msg.is_bigendian = False
        detected_msg.step = annotated_frame.shape[1] * 3
        detected_msg.data = annotated_frame.tobytes()
        self.publisher_.publish(detected_msg)
        self.get_logger().debug("Published frame with detections")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()