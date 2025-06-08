import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class IPCameraNode(Node):
    def __init__(self):
        super().__init__('ip_camera_node')

        # Declare parameters for user, pass, IP, stream path, interface
        self.declare_parameter('camera_user', 'admin')
        self.declare_parameter('camera_pass', '123456')
        self.declare_parameter('camera_ip', '192.168.0.95')
        self.declare_parameter('stream_path', 'stream0')
        self.declare_parameter('interface', 'eth0')

        self.user = self.get_parameter('camera_user').get_parameter_value().string_value
        self.password = self.get_parameter('camera_pass').get_parameter_value().string_value
        self.ip = self.get_parameter('camera_ip').get_parameter_value().string_value
        self.stream_path = self.get_parameter('stream_path').get_parameter_value().string_value
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        # Dynamically build the RTSP URL
        self.camera_url = f'rtsp://{self.user}:{self.password}@{self.ip}/{self.stream_path}'
        self.get_logger().info(f'Connecting to camera at: {self.camera_url}')

        # Create publisher
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Open video stream
        self.cap = cv2.VideoCapture(self.camera_url)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream')

        # Create timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to read frame')

def main(args=None):
    rclpy.init(args=args)
    node = IPCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
