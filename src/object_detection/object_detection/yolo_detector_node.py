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


# ? idk just keeping this from the merge

# #!/usr/bin/env python3
# # This line should be added at the beginning of any ROS node. It specifies the location of the Python interpreter
# # The 'env' command helps ensure that the correct interpreter is used based on your system’s environment
# # This concept is useful beyond ROS, as it deals with specifying the interpreter path in scripts
# # For more details, you can check this link: https://stackoverflow.com/questions/7670303/purpose-of-usr-bin-python3-shebang
# # If you're already familiar with this and just forgot to include it, feel free to ignore this comment up to yo

# """
#     Run yolo on a detected camera feed

#     How to use:
#         first run: 'ros2 run cameras cameras_node'  
#         then run: 'ros2 run object_detection yolo_detector_node' 
#         now to display the results: 'ros2 run rqt_image_view rqt_image_view'

#         do this seperately in 3 terminals. TODO: setup a launch file so this is easier lol

#         Make sure to build and source before running
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO

# class YOLODetectorNode(Node):
#     def __init__(self):
#         super().__init__('yolo_detector_node')
#         self.publisher_ = self.create_publisher(Image, 'camera/yolo/object_detection', 30) # Changed the topic name for some more clarity
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',  # This can be changed to any camera topic we want to use (realsense raw rn), for comp this will be the ptz topic
#             self.image_callback,
#             10)
#         self.bridge = CvBridge()

#         # TODO Change this to whatever YOLO model we decide to use
#         # For this I want to do some unit testing to use it for the KRIA on fpga, not sure if yolov8 will be able to be used for this
#         self.yolo = YOLO('yolov8n.pt') # right now this is fine 

#         ## We do this via ros since we have the topics we can use a subscriber for this
#         # # TODO Not sure if this is the right camera for video capture for the rover
#         # self.videoCap = cv2.VideoCapture(0)

#         # Timer to process frames
#         # self.timer = self.create_timer(0.1, self.process_frame)
        

#     def get_colours(self, cls_num):
#         # You'll have to explain to me whats going on here lol
#         base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
#         color_index = cls_num % len(base_colors)
#         increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
#         color = [
#             base_colors[color_index][i] + increments[color_index][i] * 
#             (cls_num // len(base_colors)) % 256 for i in range(3)
#         ]
#         return tuple(color)

#     # For now lets try it this way
#     # Instead of using a timer to process frames periodically, the node now processes frames as they arrive via the callback
#     # process frame here the same way, but more reactively
#     # Callback meth is called when each msg is received, so each video frame  
#     def image_callback(self, msg): 
#         # Convert ROS Image message to OpenCV image
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f'Error converting image: {e}')
#             return

#         # Process frame with YOLO
#         results = self.yolo.track(frame, stream=True)
        
#         # Draw detections on frame
#         annotated_frame = frame.copy()
#         for result in results:
#             classes_names = result.names
#             for box in result.boxes:
#                 if box.conf[0] > 0.4:
#                     [x1, y1, x2, y2] = box.xyxy[0]
#                     x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
#                     cls = int(box.cls[0])
#                     class_name = classes_names[cls]
#                     colour = self.get_colours(cls)
#                     cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), colour, 2)
#                     cv2.putText(annotated_frame, f'{class_name} {box.conf[0]:.2f}',
#                                 (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
        
#         # Convert frame to ROS2 Image message and publish
#         detected_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
#         detected_msg.header = msg.header  # Preserve original timestamp and frame_id, just allows us to communicate timestamped data if needed
#         self.publisher_.publish(detected_msg)
#         self.get_logger().debug("Published frame with detections")

#     def destroy_node(self):
#         # Release video capture when node is destroyed
#         self.videoCap.release()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLODetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

