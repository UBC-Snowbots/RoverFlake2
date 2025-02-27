import rclpy
import utm  # Install with `pip install utm`
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GpsImuToOdom(Node):
    def __init__(self):
        super().__init__('gps_imu_to_odom')
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Flags to check initialization
        self.initialized = False
        self.utm_x_init = 0.0
        self.utm_y_init = 0.0

        # Default IMU quaternion (identity quaternion)
        self.imu_quat = [0.0, 0.0, 0.0, 1.0]

    def gps_callback(self, msg):
        """ Converts GPS to odometry and publishes tf transform. """
        utm_x, utm_y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        # Initialize reference point
        if not self.initialized:
            self.utm_x_init = utm_x
            self.utm_y_init = utm_y
            self.initialized = True
            self.get_logger().info(f"Odometry initialized at ({self.utm_x_init}, {self.utm_y_init})")

        # Compute relative position
        odom_x = utm_x - self.utm_x_init
        odom_y = utm_y - self.utm_y_init

        # Publish transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_link'  # Child frame
        t.transform.translation.x = odom_x
        t.transform.translation.y = odom_y
        t.transform.translation.z = 0.0  # Assuming flat ground

        # Use IMU quaternion directly
        t.transform.rotation.x = self.imu_quat[0]
        t.transform.rotation.y = self.imu_quat[1]
        t.transform.rotation.z = self.imu_quat[2]
        t.transform.rotation.w = self.imu_quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published odom -> base_link at ({odom_x}, {odom_y})")

    def imu_callback(self, msg):
        """ Stores IMU quaternion directly for use in tf. """
        q = msg.orientation
        self.imu_quat = [q.x, q.y, q.z, q.w]  # Store IMU quaternion directly

rclpy.init()
node = GpsImuToOdom()
rclpy.spin(node)
