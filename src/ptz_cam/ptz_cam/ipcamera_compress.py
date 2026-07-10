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
                    '-f', 'mpegts', 'udp://127.0.0.1:5000?pkt_size=1316'
                ]
            ),
            'arm_rgb': (
                '/camera/arm/rgb_active',
                lambda: [
                    'gst-launch-1.0',
                    'v4l2src', 'device=/dev/video2',
                    '!', 'video/x-raw,width=640,height=480,framerate=30/1',
                    '!', 'videoconvert',
                    '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast', 'bitrate=2000',
                    '!', 'rtph264pay', 'config-interval=1', 'pt=96',
                    '!', 'udpsink', 'host=127.0.0.1', 'port=5002'
                ]
            ),
            'arm_ir': (
                '/camera/arm/ir_active',
                lambda: [
                    'gst-launch-1.0',
                    'v4l2src', 'device=/dev/video4',
                    '!', 'video/x-raw,width=640,height=480,framerate=30/1',
                    '!', 'videoconvert',
                    '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast', 'bitrate=2000',
                    '!', 'rtph264pay', 'config-interval=1', 'pt=96',
                    '!', 'udpsink', 'host=127.0.0.1', 'port=5003'
                ]
            ),
            'front_rgb': (
                '/camera/front/rgb_active',
                lambda: [
                    'gst-launch-1.0',
                    'v4l2src', 'device=/dev/video8',
                    '!', 'video/x-raw,width=640,height=480,framerate=30/1',
                    '!', 'videoconvert',
                    '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast', 'bitrate=2000',
                    '!', 'rtph264pay', 'config-interval=1', 'pt=96',
                    '!', 'udpsink', 'host=127.0.0.1', 'port=5004'
                ]
            ),
            'ptz': (
                '/camera/ptz/active',
                lambda: [
                    'ffmpeg', '-rtsp_transport', 'tcp', '-i', 'rtsp://admin:123456@192.168.2.95:554',
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
    
    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping all cameras...")
        
        for name in list(self.processes.keys()):
            self.processes[name].terminate()
            self.processes[name].wait(timeout=5)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraEncoder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()