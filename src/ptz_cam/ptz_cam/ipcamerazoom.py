#!/usr/bin/env python3

from onvif import ONVIFCamera
import time
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node


# Camera connection details
CAMERA_IP = '192.168.0.95'
PORT = 80
USERNAME = 'admin'
PASSWORD = '123456'
ZOOM_DURATION = 1.5
DEFAULT_SPEED = 0.5
POLL_INTERVAL = 0.1


# Connects to the cameras ONVIF services.
def get_ptz_service():
   cam = ONVIFCamera(CAMERA_IP, PORT, USERNAME, PASSWORD)
   media = cam.create_media_service()
   token = media.GetProfiles()[0].token  # select the first camera profile
   ptz = cam.create_ptz_service()
   return ptz, token

# Get PTZ status
def get_ptz_status(ptz, token):
   status = ptz.GetStatus({'ProfileToken': token})
   print(f"Current PTZ status: {status}")


# Sends a continuous zoom command to the camera.
# speed -1 to 1
# duration: how many seconds to keep zooming
def zoom(ptz, token, speed):
   """Zoom in or out for a given duration and speed."""
   req = ptz.create_type('ContinuousMove')
   req.ProfileToken = token
   req.Velocity = {'Zoom': {'x': speed}}
   ptz.ContinuousMove(req)
   time.sleep(0.1)
   ptz.Stop({'ProfileToken': token})

class PTZZoomNode(Node):
    def __init__(self):
        super().__init__('ptz_zoom_node')
        self.ptz, self.token = get_ptz_service()
        self.get_logger().info('PTZ Zoom node started. Use ← / → or ESC to exit.')

        self.zoom_sub = self.create_subscription(
            Vector3, '/ptz/control', self.zoom, 10
        ) 

    def zoom(self, msg):
        if msg.z == 1:
            self.get_logger().info('Zooming in')
            zoom(self.ptz, self.token)
        elif msg.z == -1:
            self.get_logger().info('Zooming out')
            zoom(self.ptz, self.token)
        else:
            self.get_logger().info('Stopping zoom...')
            self.ptz.Stop({'ProfileToken': self.token})
        

def main(args=None):
    rclpy.init(args=args)
    node = PTZZoomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()