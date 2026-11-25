"""Pretend rover base for testing navigation without hardware.

It does the two jobs the real rover's drive code must do for Nav2:
  1. Obey /cmd_vel: turn (forward speed, turn rate) into motion, with real-world limits.
  2. Report odometry: where it thinks it is now (/odom topic + the odom -> base_footprint TF).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw):
    """Rotation about 'up' as a quaternion (w, x, y, z)."""
    return math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)


def step_toward(current, target, max_step):
    """Move current toward target, but by no more than max_step."""
    return current + max(-max_step, min(max_step, target - current))


class FakeRover(Node):

    def __init__(self):
        super().__init__('fake_rover')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate', 50.0)                # Hz: how often it moves and reports
        self.declare_parameter('max_speed', 0.8)            # m/s
        self.declare_parameter('max_turn_rate', 1.5)        # rad/s
        self.declare_parameter('max_accel', 1.0)            # m/s^2: motors can't change speed instantly
        self.declare_parameter('max_turn_accel', 3.0)       # rad/s^2
        self.declare_parameter('cmd_timeout', 0.5)          # s: no command for this long -> stop
        self.declare_parameter('left_wheel_joints', '')     # comma-separated, for spinning wheels in RViz
        self.declare_parameter('right_wheel_joints', '')
        self.declare_parameter('wheel_radius', 0.0625)
        self.declare_parameter('wheel_separation', 0.358)
        p = lambda name: self.get_parameter(name).value   # noqa: E731

        self.odom_frame, self.base_frame = p('odom_frame'), p('base_frame')
        self.max_speed, self.max_turn_rate = p('max_speed'), p('max_turn_rate')
        self.max_accel, self.max_turn_accel = p('max_accel'), p('max_turn_accel')
        self.cmd_timeout = p('cmd_timeout')
        self.left_joints = [j.strip() for j in p('left_wheel_joints').split(',') if j.strip()]
        self.right_joints = [j.strip() for j in p('right_wheel_joints').split(',') if j.strip()]
        self.wheel_radius, self.wheel_separation = p('wheel_radius'), p('wheel_separation')

        self.x = self.y = self.yaw = 0.0            # where the rover is (in the odom frame)
        self.speed = self.turn_rate = 0.0           # what it's actually doing right now
        self.target_speed = self.target_turn = 0.0  # what /cmd_vel last asked for
        self.last_cmd_time = None
        self.left_angle = self.right_angle = 0.0
        self.was_driving = False

        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10) if self.left_joints else None
        self.trail_pub = self.create_publisher(Path, '/rover_trail', 10)
        self.trail = Path()
        self.trail.header.frame_id = self.odom_frame
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.last_step = self.get_clock().now()
        self.create_timer(1.0 / p('rate'), self.step)
        self.get_logger().info('Fake rover ready at (0, 0). Waiting for /cmd_vel ...')

    def on_cmd_vel(self, msg):
        self.target_speed = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.target_turn = max(-self.max_turn_rate, min(self.max_turn_rate, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0 or dt > 0.5:
            return

        # 1. Safety: if commands stop arriving, the target becomes "stop".
        silent = (self.last_cmd_time is None
                  or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout)
        target_speed = 0.0 if silent else self.target_speed
        target_turn = 0.0 if silent else self.target_turn

        # 2. Motors can't jump to a new speed: ramp toward the target.
        self.speed = step_toward(self.speed, target_speed, self.max_accel * dt)
        self.turn_rate = step_toward(self.turn_rate, target_turn, self.max_turn_accel * dt)

        # 3. Move: turn a little, then roll forward along the new heading (differential drive).
        self.yaw += self.turn_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

        # 4. Report where we are: TF (for everyone) and /odom (pose + current speeds).
        stamp = now.to_msg()
        qw, qx, qy, qz = yaw_to_quat(self.yaw)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.w, t.transform.rotation.x = qw, qx
        t.transform.rotation.y, t.transform.rotation.z = qy, qz
        self.tf_pub.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x = qw, qx
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = qy, qz
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.angular.z = self.turn_rate
        self.odom_pub.publish(odom)

        # 5. Spin the wheels in RViz: each side rolls at (speed -/+ turn rate x half the width).
        half = self.wheel_separation / 2.0
        left_speed = self.speed - self.turn_rate * half       # m/s along the ground
        right_speed = self.speed + self.turn_rate * half
        self.left_angle = (self.left_angle + left_speed / self.wheel_radius * dt) % (2 * math.pi)
        self.right_angle = (self.right_angle + right_speed / self.wheel_radius * dt) % (2 * math.pi)
        if self.joint_pub is not None:
            js = JointState()
            js.header.stamp = stamp
            js.name = self.left_joints + self.right_joints
            js.position = [self.left_angle] * len(self.left_joints) + [self.right_angle] * len(self.right_joints)
            self.joint_pub.publish(js)

        # 6. Trail of where it has driven (a point every 10 cm).
        last = self.trail.poses[-1].pose.position if self.trail.poses else None
        if last is None or math.hypot(self.x - last.x, self.y - last.y) > 0.1:
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.odom_frame
            pose.pose.position.x, pose.pose.position.y = self.x, self.y
            self.trail.poses = (self.trail.poses + [pose])[-5000:]
            self.trail.header.stamp = stamp
            self.trail_pub.publish(self.trail)

        # 7. Say when it starts/stops driving, and what it was told.
        driving = not silent and (abs(target_speed) > 1e-3 or abs(target_turn) > 1e-3)
        if driving:
            self.get_logger().info(
                f'cmd_vel: forward {target_speed:+.2f} m/s, turn {target_turn:+.2f} rad/s  '
                f'-> at x {self.x:5.2f}  y {self.y:5.2f}  facing {math.degrees(self.yaw):+5.0f} deg',
                throttle_duration_sec=1.0)
        elif self.was_driving:
            self.get_logger().info(f'Stopped at x {self.x:.2f}  y {self.y:.2f}'
                                   f'{"  (commands went silent)" if silent else ""}')
        self.was_driving = driving


def main(args=None):
    rclpy.init(args=args)
    node = FakeRover()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
