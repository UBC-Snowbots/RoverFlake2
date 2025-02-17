import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.publisher_ = self.create_publisher(Image, 'detected_frames', 10)
        self.bridge = CvBridge()

        # TODO Change this to whatever YOLO model we decide to use
        self.yolo = YOLO('yolov8n.pt') 

        # TODO Not sure if this is the right camera for video capture for the rover
        self.videoCap = cv2.VideoCapture(0)

        # Timer to process frames
        self.timer = self.create_timer(0.1, self.process_frame)

    def get_colours(self, cls_num):
        base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        color_index = cls_num % len(base_colors)
        increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
        color = [
            base_colors[color_index][i] + increments[color_index][i] * 
            (cls_num // len(base_colors)) % 256 for i in range(3)
        ]
        return tuple(color)

    def process_frame(self):
        ret, frame = self.videoCap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame.")
            return

        results = self.yolo.track(frame, stream=True)

        for result in results:
            classes_names = result.names
            for box in result.boxes:
                if box.conf[0] > 0.4:
                    [x1, y1, x2, y2] = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cls = int(box.cls[0])
                    class_name = classes_names[cls]
                    colour = self.get_colours(cls)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                    cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', 
                                (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)

        # Convert frame to ROS2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info("Published frame with detections.")

    def destroy_node(self):
        # Release video capture when node is destroyed
        self.videoCap.release()
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
