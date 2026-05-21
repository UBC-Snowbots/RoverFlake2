import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading
import time

"""
The existing CameraPublisher node is too slow to achieve 30+ FPS, which is required for the Morse Code task
This node provides a faster implementation, based on the original CameraPublisher. Speedup is achieved in 2 ways:
    1. Remove RealSense support and support for multiple camera
    2. Rather than transmit full camera data, the node calculates the on/off state of the Morse LED, and transmits a simple flag
"""
class MorseSignalPublisher(Node):
    def __init__(self):
        super().__init__("morse_camera_pub_node")

        self.std_publisher = None
        self.std_cap = None
        self.std_dev_path = None
        self.running = True

        self.BRIGHTNESS_THRESHOLD = 200

        # Auto-detect available standard camera devices. Stop after detecting one camera
        self.detect_available_cameras()

        self.get_logger().info("Creating publisher for camera 0")
        self.std_publisher = self.create_publisher(Int32, "cam_0/morse_led_brightness", 10)

        if not self.std_cap:
            self.get_logger().error("No active video devices found")
            exit(1)

        self.capture_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.capture_thread.start()

    def detect_available_cameras(self):
        """Detect standard USB cameras, excluding RealSense devices"""
        max_tested = 10
        
        for i in range(max_tested):
            dev_path = f"/dev/video{i}"

            cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
            if cap.isOpened():
                # Force MJPG to increase possible frame rate
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

                # 30 FPS. Anything lower will not work for the 66.7ms Morse code frequency
                cap.set(cv2.CAP_PROP_FPS, 30.0)
                cap.set(cv2.CAP_PROP_EXPOSURE, 10)

                self.std_dev_path = dev_path
                self.std_cap = cap

                break
            else:
                cap.release()
    
    def camera_loop(self):
        while rclpy.ok() and self.running:
            ret, frame = self.std_cap.read()
            
            if ret:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                max_brightness = int(np.max(gray))

                msg = Int32()
                msg.data = max_brightness
                self.std_publisher.publish(msg)
            else:
                time.sleep(0.005)

    def stop(self):
        self.running = False
        for cap in self.std_caps:
            cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = MorseSignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
