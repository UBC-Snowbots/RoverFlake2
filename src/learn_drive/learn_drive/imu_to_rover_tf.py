import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    """Roll/pitch/yaw in radians -> quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def quat_multiply(a, b):
    """Rotation a followed by rotation b (in a's frame). Both (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def to_roll_pitch_yaw(w, x, y, z):
    """Quaternion -> (roll, pitch, yaw) in degrees. Roll = tip sideways, pitch = nose up/down, yaw = heading."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class ImuToRoverTf(Node):
    """Tilts the rover model in RViz to match the real IMU: publishes odom -> rover frame from the IMU's orientation."""

    def __init__(self):
        super().__init__('imu_to_rover_tf')
        self.declare_parameter('rover_frame', 'base_link')  # the model's top frame
        self.declare_parameter('height', 0.3)               # lift so the wheels sit on the grid
        # How the IMU is mounted on the rover (degrees). 0,0,0 = IMU axes match the rover's:
        # x forward, y left, z up. Fill these in from your axis test.
        self.declare_parameter('mount_roll_deg', 0.0)
        self.declare_parameter('mount_pitch_deg', 0.0)
        self.declare_parameter('mount_yaw_deg', 0.0)

        self.rover_frame = self.get_parameter('rover_frame').value
        self.height = self.get_parameter('height').value
        mount = quat_from_rpy(*(math.radians(self.get_parameter(n).value)
                                for n in ('mount_roll_deg', 'mount_pitch_deg', 'mount_yaw_deg')))
        w, x, y, z = mount
        self.mount_inverse = (w, -x, -y, -z)   # undoing a rotation = same axes, opposite sign

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 10)   # filtered: has orientation
        self.get_logger().info('Waiting for /imu/data (is the Madgwick filter running?)')

    def on_imu(self, msg):
        # The filter gives the IMU's orientation. Undo the mounting rotation to get the rover's.
        o = msg.orientation
        w, x, y, z = quat_multiply((o.w, o.x, o.y, o.z), self.mount_inverse)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'              # fixed reference that never moves
        t.child_frame_id = self.rover_frame     # the rover
        t.transform.translation.z = self.height
        t.transform.rotation.w, t.transform.rotation.x = w, x
        t.transform.rotation.y, t.transform.rotation.z = y, z
        self.broadcaster.sendTransform(t)

        roll, pitch, yaw = to_roll_pitch_yaw(w, x, y, z)
        self.get_logger().info(
            f'rover roll {roll:6.1f}   pitch {pitch:6.1f}   yaw {yaw:6.1f}  (degrees)',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToRoverTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()