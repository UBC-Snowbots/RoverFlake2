import rclpy
import utm  # Install with `pip install utm`
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GpsToOdom(Node):
    def __init__(self):
        super().__init__('gps_to_odom')
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def gps_callback(self, msg):
        utm_x, utm_y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_link'  # Child frame
        t.transform.translation.x = utm_x
        t.transform.translation.y = utm_y
        t.transform.translation.z = 0.0  # Assuming flat ground

        # No orientation data, setting identity quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

rclpy.init()
node = GpsToOdom()
rclpy.spin(node)
