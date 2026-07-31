#!/usr/bin/env python3
"""Zoom + focus control for the Hi3510 IP camera via its ptzctrl.cgi endpoint.

Both zoom and focus are continuous, motor-driven moves: the motor keeps moving until it
receives a stop. This node therefore exposes press/release-style services rather than a
single fire-and-forget trigger (start = button down, stop = button up):

    /ip_camera/zoom_in_start    (std_srvs/Trigger)  -> -act=zoomin
    /ip_camera/zoom_out_start   (std_srvs/Trigger)  -> -act=zoomout
    /ip_camera/zoom_stop        (std_srvs/Trigger)  -> -act=stop
    /ip_camera/focus_in_start   (std_srvs/Trigger)  -> -act=focusin
    /ip_camera/focus_out_start  (std_srvs/Trigger)  -> -act=focusout
    /ip_camera/focus_stop       (std_srvs/Trigger)  -> -act=stop

Note: -act=stop halts whichever motor is moving, so zoom_stop and focus_stop are the same
underlying call; both exist so each HMI button has its own release binding.

sudo ip addr add 192.168.1.100/24 dev  <--- change to correct subnet
sudo ip link set eth0 up               <--- change to correct subnet

HFOV of cam = 48.3 degrees

focal length ~= 2143 px

All connection/zoom params come from the shared ip_camera_params.yaml (see the launch file);
nothing here is hardcoded.
"""

import requests
from requests.auth import HTTPBasicAuth

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class IPCameraZoomNode(Node):
    def __init__(self):
        super().__init__('ip_camera_zoom_node')

        self.declare_parameter('camera_ip', '192.168.1.88')
        self.declare_parameter('camera_user', 'admin')
        self.declare_parameter('camera_pass', 'admin')
        self.declare_parameter('http_port', 80)
        self.declare_parameter('zoom_endpoint_path', '/web/cgi-bin/hi3510/ptzctrl.cgi')
        self.declare_parameter('zoom_speed', 45)
        self.declare_parameter('zoom_step', 0)
        self.declare_parameter('request_timeout', 2.0)

        ip = self.get_parameter('camera_ip').value
        port = self.get_parameter('http_port').value
        path = self.get_parameter('zoom_endpoint_path').value
        self._url = f'http://{ip}:{port}{path}'
        self._auth = HTTPBasicAuth(
            self.get_parameter('camera_user').value,
            self.get_parameter('camera_pass').value,
        )
        self._speed = self.get_parameter('zoom_speed').value
        self._step = self.get_parameter('zoom_step').value
        self._request_timeout = self.get_parameter('request_timeout').value

        # Each service just fires a fixed -act value on the CGI endpoint.
        for name, act in (
            ('zoom_in_start', 'zoomin'),
            ('zoom_out_start', 'zoomout'),
            ('zoom_stop', 'stop'),
            ('focus_in_start', 'focusin'),
            ('focus_out_start', 'focusout'),
            ('focus_stop', 'stop'),
        ):
            self.create_service(Trigger, name, self._make_cb(act))

        self.get_logger().info(f'IP camera zoom/focus node ready. Endpoint: {self._url}')

    def _make_cb(self, act):
        def _cb(request, response):
            response.success, response.message = self._send_action(act)
            return response
        return _cb

    def _send_action(self, act):
        """Fire the ptzctrl.cgi endpoint. Returns (success, message)."""
        params = {'-step': self._step, '-act': act, '-speed': self._speed}
        try:
            resp = requests.get(
                self._url, params=params, auth=self._auth, timeout=self._request_timeout
            )
        except requests.exceptions.Timeout:
            msg = f"'{act}' timed out after {self._request_timeout}s (camera unreachable?)"
            self.get_logger().error(msg)
            return False, msg
        except requests.exceptions.ConnectionError as exc:
            msg = f"'{act}' failed to connect: {exc}"
            self.get_logger().error(msg)
            return False, msg
        except requests.exceptions.RequestException as exc:
            msg = f"'{act}' request error: {exc}"
            self.get_logger().error(msg)
            return False, msg

        if resp.status_code == 401:
            msg = f"'{act}' unauthorized (401): check camera_user/camera_pass"
            self.get_logger().error(msg)
            return False, msg
        if resp.status_code != 200:
            msg = f"'{act}' returned HTTP {resp.status_code}"
            self.get_logger().error(msg)
            return False, msg

        self.get_logger().info(f"'{act}' ok")
        return True, f"'{act}' ok"


def main(args=None):
    rclpy.init(args=args)
    node = IPCameraZoomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()