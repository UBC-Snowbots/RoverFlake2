#!/usr/bin/env python3

from onvif import ONVIFCamera
import time
from std_srvs.srv import SetBool, Trigger
from ipcamerafeed.srv import SetFloat64
import rclpy
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
def zoom(ptz, token, speed, duration=ZOOM_DURATION):
   """Zoom in or out for a given duration and speed."""
   req = ptz.create_type('ContinuousMove')
   req.ProfileToken = token
   req.Velocity = {'Zoom': {'x': speed}}
   ptz.ContinuousMove(req)
   time.sleep(duration)
   ptz.Stop({'ProfileToken': token})

class PTZZoomNode(Node):
    """
    To zoom in:
        ros2 service call /zoom std_srvs/srv/SetBool "{data: true}"
    To zoom in:
        ros2 service call /zoom std_srvs/srv/SetBool "{data: false}"
    To stop (to be honest I dont even know if this works lol ^ those work tho):
        ros2 service call /stop_zoom std_srvs/srv/Trigger "{}" 
    To change zoom duration:
        ros2 service call /zoom_duration ipcamerafeed/srv/SetFloat64 "{data: 6.9}"
    """
    def __init__(self):
        super().__init__('ptz_zoom_node')
        self.ptz, self.token = get_ptz_service()
        self.get_logger().info('PTZ Zoom node started. Use ← / → or ESC to exit.')

        # Asign service [type] to methods for zooming in/out and stopping
        self.zoom_srvs = self.create_service(SetBool, 'zoom', self.zoom) 
        self.zoom_duration_srvs = self.create_service(SetFloat64, 'zoom_duration', self.zoom_duration)
        self.stop_srvs = self.create_service(Trigger, 'stop_zoom', self.stop)

    def zoom(self,  request, response):
        # Service is bool, T for in, F for out
        if request.data:
            self.get_logger().info('Zooming in')
            zoom(self.ptz, self.token, DEFAULT_SPEED)
        else:
            self.get_logger().info('Zooming out')
            zoom(self.ptz, self.token, -DEFAULT_SPEED)
        
        # Service feedback
        response.success = True 
        response.message = 'Zoom cmd complete'
        return response
    
    def zoom_duration(self, request, response):
        if request.data > 0:
            ZOOM_DURATION = request.data
            response.success = True
            response.message = f'Zoom duration set to {ZOOM_DURATION}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "Duration > 0"
        return response
    
    def stop(self, request, response):
        self.get_logger().info('Stopping zoom...')
        self.ptz.Stop({'ProfileToken': self.token})
        response.success = True
        response.message = 'Zoom stopped'
        return response

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