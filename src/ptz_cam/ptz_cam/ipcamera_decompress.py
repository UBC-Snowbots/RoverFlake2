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

        self.cameras = {
            'wrist': {
                'port': 5000,
                'image_topic': '/camera/wrist/image_raw'
            },
            'arm_rgb': {
                'port': 5002,
                'image_topic': '/camera/arm/rgb/image_raw'
            },
            'arm_ir': {
                'port': 5003,
                'image_topic': '/camera/arm/ir/image_raw'
            },
            'front_rgb': {
                'port': 5004,
                'image_topic': '/camera/front/rgb/image_raw'
            },
            'ptz': {
                'port': 5005,
                'image_topic': '/camera/ptz/image_raw'
            }
        }

        self.captures = {}
        self.timers = {}
        self.publishers = {}

        for name, config in self.cameras.items():
            self.publishers[name] = self.create_publisher(
                Image, 
                config['image_topic'], 
                10
            )

        for name in self.cameras.keys():
            self.toggle_decoder(name, True)

    def toggle_decoder(self, name, active):
        if active:
            if name not in self.captures:
                config = self.cameras[name]
                udp_url = f'udp://127.0.0.1:{config["port"]}'
                self.get_logger().info(f"Starting decoder for {name} from {udp_url}")
                
            
                cap = cv2.VideoCapture(udp_url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    
                if not cap.isOpened():
                    self.get_logger().error(f"Cannot open UDP stream for {name}")
                    return
                    
                self.captures[name] = cap
                    
                    # Create timer to decode frames at 30 FPS
                self.timers[name] = self.create_timer(
                    0.033,  # ~30 FPS
                    lambda n=name: self.decode_and_publish(n)
                )
                    
                self.get_logger().info(f"Successfully started decoder for {name}")
                    
            
            else:
                self.get_logger().warn(f"Decoder for {name} is already running")
        else:
            if name in self.captures:
                self.get_logger().info(f"Stopping decoder for {name}")
                
                if name in self.timers:
                    self.timers[name].cancel()
                    del self.timers[name]
            
                self.captures[name].release()
                del self.captures[name]
                
                self.get_logger().info(f"Successfully stopped decoder for {name}")
            else:
                self.get_logger().warn(f"Decoder for {name} is not running")

    def decode_and_publish(self, name):
        if name not in self.captures:
            return
        
        cap = self.captures[name]
        ret, frame = cap.read()
        
        if not ret:
            return
        
        try:
            # Convert to ROS2 Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = name
            
            # Publish
            self.publishers[name].publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame from {name}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraDecoder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
