#!/usr/bin/env python3
"""
H.264 Decoder node - shows how to consume H.264 compressed streams
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import threading
import queue

class H264DecoderNode(Node):
    def __init__(self):
        super().__init__('h264_decoder_node')
        
        # Subscribe to H.264 compressed stream
        self.subscription = self.create_subscription(
            CompressedImage,
            'ipcamera/compressed',
            self.h264_callback,
            10
        )
        
        # Publisher for decoded images
        self.image_pub = self.create_publisher(Image, 'decoded/image_raw', 10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Frame dimensions (must match encoder)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Initialize FFmpeg decoder
        self.init_decoder()
        
        # Decoded frame queue
        self.frame_queue = queue.Queue(maxsize=5)
        
        # Start processing timer
        self.timer = self.create_timer(0.033, self.publish_decoded)  # 30 FPS
        
        self.get_logger().info('H.264 decoder initialized')
        
    def init_decoder(self):
        """Initialize FFmpeg H.264 decoder"""
        self.decode_cmd = [
            'ffmpeg',
            '-f', 'h264',
            '-i', '-',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-'
        ]
        
        self.decoder = subprocess.Popen(
            self.decode_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=10**8
        )
        
        # Start decoder output thread
        self.decoder_thread = threading.Thread(target=self.decoder_loop)
        self.decoder_thread.daemon = True
        self.decoder_thread.start()
        
    def decoder_loop(self):
        """Read decoded frames from FFmpeg"""
        frame_size = self.width * self.height * 3
        
        while rclpy.ok() and self.decoder.poll() is None:
            raw_frame = self.decoder.stdout.read(frame_size)
            if len(raw_frame) == frame_size:
                frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3))
                
                # Store decoded frame
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                try:
                    self.frame_queue.put_nowait(frame)
                except:
                    pass
                    
    def h264_callback(self, msg):
        """Handle incoming H.264 data"""
        if msg.format == 'h264':
            try:
                # Feed H.264 data to decoder
                self.decoder.stdin.write(msg.data)
                self.decoder.stdin.flush()
            except Exception as e:
                self.get_logger().error(f'Error decoding H.264: {str(e)}')
                self.init_decoder()  # Reinitialize decoder on error
                
        elif msg.format == 'jpeg':
            # Fallback for JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except:
                        pass
                try:
                    self.frame_queue.put_nowait(frame)
                except:
                    pass
                    
    def publish_decoded(self):
        """Publish decoded frames"""
        try:
            frame = self.frame_queue.get_nowait()
            
            # Convert to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            
            self.image_pub.publish(img_msg)
            
            # Example: Do some processing
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # self.get_logger().info(f'Decoded frame: {frame.shape}')
            
        except queue.Empty:
            pass
            
    def destroy_node(self):
        """Cleanup"""
        if hasattr(self, 'decoder'):
            self.decoder.terminate()
            self.decoder.wait()
        super().destroy_node()


class SimpleH264Viewer(Node):
    """Simple viewer using OpenCV's built-in H.264 support"""
    
    def __init__(self):
        super().__init__('h264_viewer')
        
        self.subscription = self.create_subscription(
            CompressedImage,
            'ipcamera/compressed',
            self.compressed_callback,
            10
        )
        
        # Buffer for accumulating H.264 data
        self.h264_buffer = b''
        
        # Try using OpenCV's VideoWriter/VideoCapture for H.264
        # Note: This approach may not work on all systems
        self.fourcc = cv2.VideoWriter_fourcc(*'H264')
        
        self.get_logger().info('H.264 viewer started')
        
    def compressed_callback(self, msg):
        """Display compressed video"""
        if msg.format == 'h264':
            # Accumulate H.264 NAL units
            self.h264_buffer += msg.data
            
            # For viewing, you'd typically need to decode this
            # OpenCV doesn't directly support H.264 stream decoding
            # So we'd need FFmpeg as shown in the decoder node above
            
            self.get_logger().info(f'Received H.264 data: {len(msg.data)} bytes')
            
        elif msg.format == 'jpeg':
            # Easy to display JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('IP Camera', frame)
                cv2.waitKey(1)


# ===== Usage Instructions =====
"""
USAGE INSTRUCTIONS FOR H.264 COMPRESSION:

1. Run IP camera with H.264 compression:
   ros2 run your_package ip_camera_node --ros-args \
       -p compression_type:=h264 \
       -p h264_bitrate:=2M \
       -p h264_preset:=ultrafast

2. Check the compressed stream:
   ros2 topic echo /ipcamera/compressed --no-arr
   ros2 topic bw /ipcamera/compressed

3. Run the decoder to get raw images back:
   ros2 run your_package h264_decoder_node

4. Save H.264 stream to file:
   ros2 bag record /ipcamera/compressed -o h264_recording

5. Stream over network with minimal bandwidth:
   # On robot:
   ros2 run your_package ip_camera_node --ros-args -p compression_type:=h264 -p h264_bitrate:=500K
   
   # On remote computer:
   ros2 run your_package h264_decoder_node

COMPRESSION COMPARISON:
- Raw video (1920x1080 @ 30fps): ~180 MB/s
- JPEG (quality 80): ~15-30 MB/s  
- H.264 (2Mbps): ~0.25 MB/s
- H.264 (500Kbps): ~0.06 MB/s

H.264 PARAMETERS:
- h264_bitrate: Lower = smaller files, lower quality (500K-4M typical)
- h264_preset: ultrafast = low CPU, larger files | slow = high CPU, smaller files
- Recommended: ultrafast for real-time, fast for recording

TROUBLESHOOTING:
- If H.264 doesn't work, check: apt install ffmpeg libx264-dev
- For hardware acceleration: Add -hwaccel cuda or -hwaccel vaapi to FFmpeg commands
- Reduce bitrate for WiFi: -p h264_bitrate:=500K
- Increase GOP size for better compression: Modify -g parameter in encode_cmd
"""

def main(args=None):
    rclpy.init(args=args)
    
    # Choose which node to run
    node = H264DecoderNode()
    # node = SimpleH264Viewer()  # Alternative viewer
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
