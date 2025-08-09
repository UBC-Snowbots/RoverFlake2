#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import threading
import queue
import struct

class IPCameraNode(Node):
    def __init__(self):
        super().__init__('ip_camera_node')
        
        # Declare parameters for camera connection
        self.declare_parameter('camera_user', 'admin')
        self.declare_parameter('camera_pass', '123456')
        self.declare_parameter('camera_ip', '192.168.0.95')
        self.declare_parameter('stream_path', 'stream0')
        
        # Compression parameters
        self.declare_parameter('compression_type', 'h264')  # 'h264', 'jpeg', or 'none'
        self.declare_parameter('h264_bitrate', '2M')  # For H.264: 500K, 1M, 2M, 4M
        self.declare_parameter('h264_preset', 'ultrafast')  # ultrafast, fast, medium, slow
        self.declare_parameter('jpeg_quality', 80)  # For JPEG fallback
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 1920)  # Camera resolution
        self.declare_parameter('height', 1080)
        
        # Get parameters
        self.user = self.get_parameter('camera_user').value
        self.password = self.get_parameter('camera_pass').value
        self.ip = self.get_parameter('camera_ip').value
        self.stream_path = self.get_parameter('stream_path').value
        self.compression_type = self.get_parameter('compression_type').value
        self.h264_bitrate = self.get_parameter('h264_bitrate').value
        self.h264_preset = self.get_parameter('h264_preset').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Build RTSP URL
        self.camera_url = f'rtsp://{self.user}:{self.password}@{self.ip}/{self.stream_path}'
        self.get_logger().info(f'Connecting to camera at: {self.camera_url}')
        self.get_logger().info(f'Compression: {self.compression_type}, Resolution: {self.width}x{self.height}')
        
        # Create publishers
        self.raw_publisher = self.create_publisher(Image, 'ipcamera/image_raw', 10)
        self.compressed_publisher = self.create_publisher(CompressedImage, 'ipcamera/compressed', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Frame queues
        self.raw_frame_queue = queue.Queue(maxsize=2)
        self.h264_queue = queue.Queue(maxsize=10)
        
        # Initialize capture and compression
        if self.compression_type == 'h264':
            self.init_h264_pipeline()
        else:
            self.init_simple_capture()
        
        # Create timer for publishing
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
    def init_h264_pipeline(self):
        """Initialize FFmpeg pipeline for H.264 compression"""
        # FFmpeg command for capturing from RTSP
        self.capture_cmd = [
            'ffmpeg',
            '-rtsp_transport', 'tcp',
            '-i', self.camera_url,
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-'
        ]
        
        # FFmpeg command for H.264 encoding
        self.encode_cmd = [
            'ffmpeg',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.fps),
            '-i', '-',
            '-c:v', 'libx264',
            '-preset', self.h264_preset,
            '-tune', 'zerolatency',  # Reduces latency
            '-b:v', self.h264_bitrate,
            '-maxrate', self.h264_bitrate,
            '-bufsize', '1M',
            '-g', str(int(self.fps)),  # GOP size = fps (1 keyframe per second)
            '-f', 'h264',
            '-'
        ]
        
        # Start capture process
        self.capture_process = subprocess.Popen(
            self.capture_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=10**8
        )
        
        # Start encode process
        self.encode_process = subprocess.Popen(
            self.encode_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=10**8
        )
        
        # Start threads
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.encode_thread = threading.Thread(target=self.encode_loop)
        self.encode_thread.daemon = True
        self.encode_thread.start()
        
        self.h264_thread = threading.Thread(target=self.h264_reader_loop)
        self.h264_thread.daemon = True
        self.h264_thread.start()
        
        self.get_logger().info(f'H.264 pipeline started: {self.h264_bitrate} bitrate, {self.h264_preset} preset')
        
    def capture_loop(self):
        """Read raw frames from RTSP"""
        frame_size = self.width * self.height * 3
        
        while rclpy.ok() and self.capture_process.poll() is None:
            raw_frame = self.capture_process.stdout.read(frame_size)
            if len(raw_frame) == frame_size:
                frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.height, self.width, 3))
                
                # Store for raw publishing
                if self.raw_frame_queue.full():
                    try:
                        self.raw_frame_queue.get_nowait()
                    except:
                        pass
                try:
                    self.raw_frame_queue.put_nowait(frame)
                except:
                    pass
                
                # Send to encoder
                try:
                    self.encode_process.stdin.write(raw_frame)
                    self.encode_process.stdin.flush()
                except:
                    break
    
    def encode_loop(self):
        """Just keep the encoder process alive"""
        # The actual H.264 reading happens in h264_reader_loop
        pass
    
    def h264_reader_loop(self):
        """Read H.264 NAL units from encoder"""
        buffer = b''
        
        while rclpy.ok() and self.encode_process.poll() is None:
            # Read chunks of H.264 data
            chunk = self.encode_process.stdout.read(4096)
            if not chunk:
                break
                
            buffer += chunk
            
            # Look for NAL unit boundaries (0x00000001 or 0x000001)
            while True:
                # Find start code
                pos4 = buffer.find(b'\x00\x00\x00\x01')
                pos3 = buffer.find(b'\x00\x00\x01')
                
                if pos4 == -1 and pos3 == -1:
                    # No complete NAL unit yet
                    break
                
                # Determine which start code we found first
                if pos4 != -1 and (pos3 == -1 or pos4 < pos3):
                    start_pos = pos4
                    start_code_len = 4
                else:
                    start_pos = pos3
                    start_code_len = 3
                
                if start_pos > 0:
                    # We have data before the start code - this is a complete NAL unit
                    nal_unit = buffer[:start_pos]
                    buffer = buffer[start_pos:]
                    
                    # Queue the NAL unit
                    if self.h264_queue.full():
                        try:
                            self.h264_queue.get_nowait()
                        except:
                            pass
                    try:
                        self.h264_queue.put_nowait(nal_unit)
                    except:
                        pass
                else:
                    # Start code is at the beginning, look for the next one
                    next_pos4 = buffer.find(b'\x00\x00\x00\x01', start_code_len)
                    next_pos3 = buffer.find(b'\x00\x00\x01', start_code_len)
                    
                    if next_pos4 == -1 and next_pos3 == -1:
                        # No next NAL unit yet
                        break
                    
                    # Determine end position
                    if next_pos4 != -1 and (next_pos3 == -1 or next_pos4 < next_pos3):
                        end_pos = next_pos4
                    else:
                        end_pos = next_pos3
                    
                    nal_unit = buffer[:end_pos]
                    buffer = buffer[end_pos:]
                    
                    # Queue the NAL unit  
                    if self.h264_queue.full():
                        try:
                            self.h264_queue.get_nowait()
                        except:
                            pass
                    try:
                        self.h264_queue.put_nowait(nal_unit)
                    except:
                        pass
    
    def init_simple_capture(self):
        """Initialize simple OpenCV capture for JPEG compression"""
        self.cap = cv2.VideoCapture(self.camera_url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream with OpenCV')
            return
            
        self.capture_thread = threading.Thread(target=self.simple_capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info(f'OpenCV capture started, compression: {self.compression_type}')
        
    def simple_capture_loop(self):
        """Simple capture for JPEG/no compression"""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                if self.raw_frame_queue.full():
                    try:
                        self.raw_frame_queue.get_nowait()
                    except:
                        pass
                try:
                    self.raw_frame_queue.put_nowait(frame)
                except:
                    pass
                    
    def publish_frame(self):
        """Publish frames based on compression type"""
        try:
            # Publish raw if anyone is listening
            if self.raw_publisher.get_subscription_count() > 0:
                frame = self.raw_frame_queue.get_nowait()
                raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                raw_msg.header.stamp = self.get_clock().now().to_msg()
                raw_msg.header.frame_id = 'camera_link'
                self.raw_publisher.publish(raw_msg)
                
                # Put frame back for compression if needed
                if self.compression_type == 'jpeg':
                    self.raw_frame_queue.put_nowait(frame)
        except queue.Empty:
            pass
        
        # Publish compressed
        if self.compressed_publisher.get_subscription_count() > 0:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = 'camera_link'
            
            if self.compression_type == 'h264':
                try:
                    # Get H.264 NAL units from queue
                    h264_data = b''
                    # Grab multiple NAL units if available for better streaming
                    for _ in range(min(5, self.h264_queue.qsize())):
                        try:
                            nal = self.h264_queue.get_nowait()
                            h264_data += nal
                        except queue.Empty:
                            break
                    
                    if h264_data:
                        compressed_msg.format = 'h264'
                        compressed_msg.data = h264_data
                        self.compressed_publisher.publish(compressed_msg)
                        
                except Exception as e:
                    self.get_logger().error(f'Error publishing H.264: {str(e)}')
                    
            elif self.compression_type == 'jpeg':
                try:
                    frame = self.raw_frame_queue.get_nowait()
                    compressed_msg.format = 'jpeg'
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    _, compressed_data = cv2.imencode('.jpg', frame, encode_param)
                    compressed_msg.data = compressed_data.tobytes()
                    self.compressed_publisher.publish(compressed_msg)
                except queue.Empty:
                    pass
                    
    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'capture_process'):
            self.capture_process.terminate()
            self.capture_process.wait()
        if hasattr(self, 'encode_process'):
            self.encode_process.terminate()
            self.encode_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IPCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
