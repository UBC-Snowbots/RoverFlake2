import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import cv2
import threading

class CameraDecoder(Node):
    def __init__(self):
        super().__init__('camera_decoder')
        self.bridge = CvBridge()

        self.udp_ports = {
            'wrist': 5000,
            'arm_rgb': 5002,
            'arm_ir': 5003,
            'front_rgb': 5004,
            'ptz': 5005
        }

        for name, port in self.udp_ports.items():
            threading.Thread(target=self.decode_loop, args=(name, port), daemon=True).start()

    def decode_loop(self, name, port):
        cap = cv2.VideoCapture(f'udp://127.0.0.1:{port}', cv2.CAP_FFMPEG)
        pub = self.create_publisher(Image, f'/camera/{name}/image_raw', 10)

        while rclpy.ok():
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                pub.publish(msg)

def main():
    rclpy.init()
    node = CameraDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
