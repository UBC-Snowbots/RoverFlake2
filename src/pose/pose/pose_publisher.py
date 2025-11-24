import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix.
    The quaternion is assumed to represent a rotation from body frame to navigation frame.
    
    Reference (one common form):
      R = [[1 - 2(y^2 + z^2),     2(xy - z w ),     2(xz + y w )],
           [2(xy + z w),         1 - 2(x^2 + z^2),  2(yz - x w )],
           [2(xz - y w),         2(yz + x w),       1 - 2(x^2 + y^2)]]
    """
    x, y, z, w = q

    # Precompute squares
    xx = x * x
    yy = y * y
    zz = z * z
    ww = w * w

    # Precompute cross terms
    xy = x * y
    xz = x * z
    yz = y * z
    xw = x * w
    yw = y * w
    zw = z * w

    # Rotation matrix
    R = np.array([
        [1 - 2*(yy + zz),     2*(xy - zw),         2*(xz + yw)],
        [2*(xy + zw),         1 - 2*(xx + zz),     2*(yz - xw)],
        [2*(xz - yw),         2*(yz + xw),         1 - 2*(xx + yy)]
    ], dtype=np.float64)

    return R

class PosePathPublisher(Node):
    def __init__(self):
        super().__init__('pose_path_publisher')

        # Define QoS profile for IMU data (important for reliable data)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_profile
        )
        self.imu_subscription  # prevent unused variable warning

        # Publisher for estimated pose
        self.pose_publisher = self.create_publisher(PoseStamped, '/estimated_pose', 10)

        # Publisher for estimated path
        self.path_publisher = self.create_publisher(Path, '/estimated_path', 10)

        # Initialize variables for pose and path estimation
        self.current_pose = PoseStamped()
        self.path = Path()
        self.path.header.frame_id = 'map'  # The frame in which we estimate position
        self.current_pose.header.frame_id = 'map'

        self.previous_time = None

        # Velocity and position in the "map" (navigation) frame
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.position = np.zeros(3)  # [x, y, z]

        # Just store the orientation for the pose message
        self.orientation = Quaternion()

        self.get_logger().info('Pose and Path publisher with manual quaternion->R conversion started.')

    def imu_callback(self, msg: Imu):
        """
        IMU callback where we:
          1. Convert quaternion to rotation matrix (body->nav).
          2. Transform measured acceleration into nav frame.
          3. Subtract gravity in the nav frame.
          4. Naively integrate to update velocity and position.
          5. Publish PoseStamped and Path.
        """
        current_time = self.get_clock().now().to_msg()

        if self.previous_time is not None:
            dt = (current_time.sec + current_time.nanosec * 1e-9) \
               - (self.previous_time.sec + self.previous_time.nanosec * 1e-9)
        else:
            dt = 0.0
        self.previous_time = current_time

        # Store the IMU's orientation for publishing
        self.orientation = msg.orientation

        # Convert quaternion (x, y, z, w) to a rotation matrix
        # Note: The geometry_msgs/Quaternion is (x, y, z, w)
        q = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        R_b2n = quaternion_to_rotation_matrix(q)  # 3x3 rotation matrix

        # IMU linear acceleration is in the body frame
        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Rotate acceleration into navigation frame
        a_nav = R_b2n.dot(a_body)

        # If +Z is up in the navigation frame, gravity = -9.81 in Z
        # => Subtract 9.81 from the z-component
        a_nav[2] -= 9.807

        # Integrate acceleration -> velocity -> position in the nav frame
        if dt > 0.0:
            self.velocity += a_nav * dt
            self.position += self.velocity * dt

        # (Very basic integration approach; real systems use sensor fusion.)

        # Debug print (set to .debug if too verbose)
        self.get_logger().info(
            f"dt={dt:.3f}, a_nav=({a_nav[0]:.2f}, {a_nav[1]:.2f}, {a_nav[2]:.2f}), "
            f"pos=({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})"
        )

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(self.position[0])
        pose_msg.pose.position.y = float(self.position[1])
        pose_msg.pose.position.z = float(self.position[2])
        pose_msg.pose.orientation = self.orientation
        self.pose_publisher.publish(pose_msg)

        # Update and publish Path
        self.current_pose = pose_msg
        self.path.header.stamp = current_time
        self.path.poses.append(self.current_pose)
        self.path_publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    pose_path_publisher = PosePathPublisher()
    rclpy.spin(pose_path_publisher)
    pose_path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
