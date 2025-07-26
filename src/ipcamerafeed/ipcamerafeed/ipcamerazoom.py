#!/usr/bin/env python3


from onvif import ONVIFCamera
import time
import keyboard
import rclpy
from rclpy.node import Node


# Camera connection details
CAMERA_IP = '192.168.0.95'
PORT = 80
USERNAME = 'admin'
PASSWORD = '123456'
DEFAULT_DURATION = 1.5
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
def zoom(ptz, token, speed, duration=DEFAULT_DURATION):
   """Zoom in or out for a given duration and speed."""
   req = ptz.create_type('ContinuousMove')
   req.ProfileToken = token
   req.Velocity = {'Zoom': {'x': speed}}
   ptz.ContinuousMove(req)
   time.sleep(duration)
   ptz.Stop({'ProfileToken': token})




class PTZZoomNode(Node):
   def __init__(self):
       super().__init__('ptz_zoom_node')
       self.ptz, self.token = get_ptz_service()
       self.get_logger().info('PTZ Zoom node started. Use ← / → or ESC to exit.')
       # Timer to poll keyboard state
       self.timer = self.create_timer(POLL_INTERVAL, self.poll_keyboard)


   def poll_keyboard(self):
       try:
           if keyboard.is_pressed('right'):
               self.get_logger().info('Zooming in...')
               zoom(self.ptz, self.token, +DEFAULT_SPEED)
           elif keyboard.is_pressed('left'):
               self.get_logger().info('Zooming out...')
               zoom(self.ptz, self.token, -DEFAULT_SPEED)
           elif keyboard.is_pressed('esc'):
               self.get_logger().info('Exit key pressed, shutting down...')
               rclpy.shutdown()
       except Exception as e:
           self.get_logger().error(f'Error polling keyboard: {e}')


   def destroy_node(self):
       # Clean up if needed
       super().destroy_node()




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