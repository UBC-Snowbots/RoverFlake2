import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess

class CameraEncoder(Node):
    def __init__(self):
        super().__init__('camera_encoder')

        # Camera mapping: name -> (activation_topic, ffmpeg_command_template)
        self.cameras = {
            'wrist': (
                '/camera/wrist/active',
                lambda: [
                    'ffmpeg', '-f', 'v4l2', '-i', '/dev/video0',
                    '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
                    '-f', 'mpegts', 'udp://127.0.0.1:5000?pkt_size=1316'                ]
            ),
            'arm_rgb': (
                '/camera/arm/rgb_active',
                lambda: [
                    'gst-launch-1.0', 'rs2src', 'device=/dev/video2', 'stream-type=0',
                    '!', 'videoconvert',
                    '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast',
                    '!', 'rtpmp4gpay', 'config-interval=1',
                    '!', 'udpsink', 'host=127.0.0.1', 'port=5002'
                ]
            ),
            'arm_ir': (
                '/camera/arm/ir_active',
                lambda: [
                    'gst-launch-1.0', 'rs2src', 'device=/dev/video2', 'stream-type=2',
                    '!', 'videoconvert',
                    '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast',
                    '!', 'rtpmp4gpay', 'config-interval=1',
                    '!', 'udpsink', 'host=127.0.0.1', 'port=5003'
                ]
            ),
            'front_rgb': (
                '/camera/front/rgb_active',
                lambda: [
                    'gst-launch-1.0', 'rs2src', 'device=/dev/video8', 'stream-type=0',
                    '!', 'videoconvert',
                    '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast',
                    '!', 'rtpmp4gpay', 'config-interval=1',
                    '!', 'udpsink', 'host=127.0.0.1', 'port=5004'
                ]
            ),
            'ptz': (
                '/camera/ptz/active',
                lambda: [
                    'ffmpeg', '-rtsp_transport', 'tcp', '-i', 'rtsp://admin:123456@192.168.0.95:554',
                    '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
                    '-f', 'mpegts', 'udp://127.0.0.1:5005?pkt_size=1316'
                ]
            )
        }

        self.processes = {}
        for name, (topic, cmd_fn) in self.cameras.items():
            self.create_subscription(Bool, topic, lambda msg, n=name: self.toggle_camera(n, msg.data), 10)

    def toggle_camera(self, name, active):
        if active:
            if name not in self.processes:
                cmd = self.cameras[name][1]()
                self.get_logger().info(f"Starting {name}: {' '.join(cmd)}")
                self.processes[name] = subprocess.Popen(cmd)
        else:
            if name in self.processes:
                self.get_logger().info(f"Stopping {name}")
                self.processes[name].terminate()
                self.processes[name].wait()
                del self.processes[name]

def main():
    rclpy.init()
    node = CameraEncoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
