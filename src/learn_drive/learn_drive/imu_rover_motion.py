import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

G = 9.80665  # gravity, m/s^2


# ---------- small quaternion helpers (w, x, y, z) ----------
def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def rotate(q, v):
    """Express vector v (given in q's frame) in the world frame."""
    w, x, y, z = q
    p = quat_multiply(quat_multiply(q, (0.0, *v)), (w, -x, -y, -z))
    return p[1], p[2], p[3]


def norm(v):
    return math.sqrt(sum(c * c for c in v))


# ---------- the motion logic (no ROS in here, so it can be tested on its own) ----------
class MotionModel:
    """Turns IMU readings into rover motion along the ground, with the tricks that stop drift.

    Idea: work out how fast the IMU (your hand) is moving forward, then move the rover
    `scale` times that. Speed comes from adding up acceleration over time, which drifts,
    so the tricks below keep it honest.
    """

    def __init__(self, scale=3.0, deadband=0.1, still_change=0.15, still_gyro=0.15,
                 still_time=0.25, coast_time=0.3, friction_time=0.4, turn_gate=0.6,
                 smoothing=0.2, max_speed=4.5, max_move_time=4.0,
                 settle_accel=0.15, settle_time=0.12, settle_fraction=0.35, still_window=0.4):
        self.scale = scale                  # hand motion -> rover motion multiplier
        self.deadband = deadband            # m/s^2: weaker than this = not a real push
        self.still_change = still_change    # m/s^2: raw reading changes less than this = not moving
        self.still_gyro = still_gyro        # rad/s: rotating slower than this = not rotating
        self.still_time = still_time        # s: still this long -> speed is exactly 0
        self.coast_time = coast_time        # s: no push for this long -> friction starts
        self.friction_time = friction_time  # s: then speed fades with this time constant
        self.turn_gate = turn_gate          # rad/s: turning (left/right) faster than this -> ignore pushes
        self.smoothing = smoothing          # 0..1: low-pass filter on acceleration (smaller = smoother)
        self.max_speed = max_speed          # m/s: the rover in RViz never goes faster than this
        self.max_move_time = max_move_time  # s: a slide never keeps going one way longer than this
        self.settle_accel = settle_accel    # m/s^2: "hand stopped pushing" threshold
        self.settle_time = settle_time      # s: ...for this long = end of a slide
        self.settle_fraction = settle_fraction  # end of slide only if speed fell below this share of its peak
        self.still_window = still_window    # s: "not changing" compares against this long an average
        self.warmup_needed = 60             # readings to average at startup (~0.5 s at 125 Hz)
        self.reset()

    def reset(self):
        self.x = self.y = 0.0
        self.hand_speed = 0.0               # m/s along the rover's front: real (unscaled) IMU speed
        self.gravity = [0.0, 0.0, 0.0]      # what gravity reads in the IMU's own axes (learned when still)
        self.gyro_error = [0.0, 0.0, 0.0]   # what the gyro reads when NOT rotating (learned when still)
        self.warmup_samples = 0
        self.raw_avg = [0.0, 0.0, 0.0]      # raw accelerometer reading, averaged over ~0.4 s
        self.raw_now = [0.0, 0.0, 0.0]      # raw accelerometer reading, very lightly smoothed
        self.fast = [0.0, 0.0, 0.0]         # push in world directions, lightly smoothed
        self.still_for = 0.0
        self.no_push_for = 0.0
        self.quiet_for = 0.0                # how long the push has been ~0
        self.slide_time = 0.0               # how long since the last stop
        self.slide_peak = 0.0               # fastest speed since the last stop
        self.move_time = 0.0                # how long the speed has kept the same direction
        self.move_sign = 0
        self.slides_tidied = 0              # slides whose leftover speed was cleaned up
        self.errors_caught = 0              # times the "kept going too long" rule fired
        self.peak_front = self.peak_side = 0.0   # strongest push since last status line (m/s^2)

    @property
    def speed(self):
        return self.scale * self.hand_speed  # the rover's speed in RViz, m/s

    @property
    def is_still(self):
        return self.still_for >= self.still_time

    @property
    def status(self):
        if self.warmup_samples < self.warmup_needed:
            return 'learning gravity - keep it still'
        return 'still' if self.is_still else 'MOVING'

    def update(self, dt, q_imu, q_rover, accel_imu, gyro_imu):
        # 0. Warm-up: for the first half second, just average the readings. Sitting still,
        #    the accelerometer reads only gravity and the gyro reads only its own error.
        if self.warmup_samples < self.warmup_needed:
            self.warmup_samples += 1
            n = self.warmup_samples
            self.gravity = [g + (r - g) / n for g, r in zip(self.gravity, accel_imu)]
            self.gyro_error = [e + (r - e) / n for e, r in zip(self.gyro_error, gyro_imu)]
            self.raw_avg = list(self.gravity)
            self.raw_now = list(self.gravity)
            return

        # 1. Take the gyro's own error off its reading.
        gyro = [w - e for w, e in zip(gyro_imu, self.gyro_error)]
        rotating = norm(gyro)

        # 2. Stillness check, from the RAW readings only: the accelerometer isn't changing
        #    (compared with its last ~0.4 s) and it isn't rotating.
        k = min(1.0, dt / self.still_window)
        self.raw_avg = [m + k * (r - m) for m, r in zip(self.raw_avg, accel_imu)]
        self.raw_now = [n + 0.5 * (r - n) for n, r in zip(self.raw_now, accel_imu)]
        change = norm([n - m for n, m in zip(self.raw_now, self.raw_avg)])
        still = change < self.still_change and rotating < self.still_gyro
        self.still_for = self.still_for + dt if still else 0.0

        # 3. When the IMU rotates, gravity's direction in the IMU's axes turns the opposite way.
        #    The gyro says exactly how much, so keep our gravity reading up to date.
        wx, wy, wz = gyro
        gx, gy, gz = self.gravity
        self.gravity = [gx - (wy * gz - wz * gy) * dt,
                        gy - (wz * gx - wx * gz) * dt,
                        gz - (wx * gy - wy * gx) * dt]

        # 4. While still, re-learn both errors from scratch:
        #    the accelerometer reads exactly gravity, and the gyro should read exactly 0.
        if self.is_still:
            k = min(1.0, dt / 0.3)
            self.gravity = [g + k * (r - g) for g, r in zip(self.gravity, accel_imu)]
            if rotating < 0.05 and self.still_for > 1.0:
                k = min(1.0, dt / 2.0)
                self.gyro_error = [e + k * (r - e) for e, r in zip(self.gyro_error, gyro_imu)]

        # 5. Push = reading minus gravity, turned into world directions, smoothed.
        push = rotate(q_imu, [a - g for a, g in zip(accel_imu, self.gravity)])
        self.fast = [f + self.smoothing * (p - f) for f, p in zip(self.fast, push)]

        # 6. Which way is the rover's front, flattened onto the ground?
        fx, fy, _ = rotate(q_rover, (1.0, 0.0, 0.0))
        length = math.hypot(fx, fy)
        if length < 1e-3:                   # pointing straight up/down: no ground direction
            return
        fx, fy = fx / length, fy / length

        # 7. Differential drive: wheels only roll forward/back, so keep only the push along the front.
        a = self.fast[0] * fx + self.fast[1] * fy
        side = -self.fast[0] * fy + self.fast[1] * fx       # the part we throw away
        if abs(a) > abs(self.peak_front):
            self.peak_front = a
        if abs(side) > abs(self.peak_side):
            self.peak_side = side
        # Turning left/right swings the IMU around a little circle, which feels like a push
        # toward the middle of the turn. So while it's clearly turning, ignore pushes.
        # (Turning rate = gyro reading along "up", and "up" is the direction gravity reads.)
        up = [g / max(norm(self.gravity), 1e-6) for g in self.gravity]
        turn_rate = abs(sum(w * u for w, u in zip(gyro, up)))
        pushing = abs(a) > self.deadband and turn_rate < self.turn_gate
        self.no_push_for = 0.0 if pushing else self.no_push_for + dt
        self.quiet_for = self.quiet_for + dt if abs(a) < self.settle_accel else 0.0

        # 8. Update speed.
        if self.is_still:
            self.hand_speed = 0.0                                    # zero-velocity update: kills drift
            self.slide_time = self.slide_peak = 0.0
        else:
            if pushing:
                self.hand_speed += a * dt                            # speed = sum of acceleration x time
            elif self.no_push_for > self.coast_time:
                self.hand_speed *= math.exp(-dt / self.friction_time)   # nothing pushing for a while
            self.slide_time += dt
            self.slide_peak = max(self.slide_peak, abs(self.hand_speed))

            # 9. End of a slide: your hand has stopped pushing and the speed has dropped most of
            #    the way. A real hand is at rest here, so whatever speed is left is error.
            #    Snap it to 0, and use it to correct gravity (leftover / time = how wrong it was).
            if (self.quiet_for >= self.settle_time and self.slide_peak > 0.05
                    and abs(self.hand_speed) < self.settle_fraction * self.slide_peak):
                leak = max(-0.5, min(0.5, 0.5 * self.hand_speed / max(self.slide_time, 0.2)))
                fix = rotate((q_imu[0], -q_imu[1], -q_imu[2], -q_imu[3]), (leak * fx, leak * fy, 0.0))
                self.gravity = [g + c for g, c in zip(self.gravity, fix)]
                self.hand_speed = 0.0
                self.slide_time = self.slide_peak = 0.0
                self.slides_tidied += 1

        limit = self.max_speed / self.scale                          # safety cap
        self.hand_speed = max(-limit, min(limit, self.hand_speed))

        # 10. Safety rule: a slide on a desk lasts a second or so. Speed that keeps going the
        #     same way for much longer is built-up error, not motion: throw it away.
        sign = (self.hand_speed > 0) - (self.hand_speed < 0)
        self.move_time = self.move_time + dt if (sign != 0 and sign == self.move_sign) else 0.0
        self.move_sign = sign
        if self.move_time > self.max_move_time:
            self.hand_speed = 0.0
            self.move_time = 0.0
            self.errors_caught += 1

        # 11. Move along the front direction.
        self.x += self.speed * fx * dt
        self.y += self.speed * fy * dt


# ---------- the ROS node ----------
class ImuRoverMotion(Node):

    def __init__(self):
        super().__init__('imu_rover_motion')
        self.declare_parameter('rover_frame', 'base_link')
        self.declare_parameter('height', 0.3)
        self.declare_parameter('mount_roll_deg', 0.0)
        self.declare_parameter('mount_pitch_deg', 0.0)
        self.declare_parameter('mount_yaw_deg', 0.0)
        self.declare_parameter('motion_scale', 3.0)     # rover moves this many times your hand's distance
        self.declare_parameter('accel_deadband', 0.1)   # m/s^2: pushes weaker than this are ignored
        self.declare_parameter('friction_time', 0.4)    # s: how fast leftover speed fades out
        self.declare_parameter('flat_ground', True)     # True: rover stays flat on the ground, only turns
        # Differential drive wheels to spin in RViz (comma-separated joint names; empty = none)
        self.declare_parameter('left_wheel_joints', '')
        self.declare_parameter('right_wheel_joints', '')
        self.declare_parameter('wheel_radius', 0.0625)      # m
        self.declare_parameter('wheel_separation', 0.358)   # m, left wheels to right wheels
        p = lambda name: self.get_parameter(name).value   # noqa: E731

        self.rover_frame, self.height = p('rover_frame'), p('height')
        self.flat_ground = p('flat_ground')
        w, x, y, z = quat_from_rpy(*(math.radians(p(n)) for n in
                                     ('mount_roll_deg', 'mount_pitch_deg', 'mount_yaw_deg')))
        self.mount_inverse = (w, -x, -y, -z)
        self.model = MotionModel(scale=p('motion_scale'), deadband=p('accel_deadband'),
                                 friction_time=p('friction_time'))
        self.last_stamp = None
        self.last_header_stamp = None
        self.yaw = None
        self.turn_rate = 0.0                # rad/s, + = turning left

        self.left_joints = [j.strip() for j in p('left_wheel_joints').split(',') if j.strip()]
        self.right_joints = [j.strip() for j in p('right_wheel_joints').split(',') if j.strip()]
        self.wheel_radius, self.wheel_separation = p('wheel_radius'), p('wheel_separation')
        self.left_angle = self.right_angle = 0.0

        self.broadcaster = TransformBroadcaster(self)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.path_pub = self.create_publisher(Path, '/imu_path', 10)
        self.ground_pub = self.create_publisher(Marker, '/ground', 10)
        # The same kind of message the real rover drives from (/cmd_vel), on a harmless topic name
        self.twist_pub = self.create_publisher(Twist, '/imu_twist', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10) if self.left_joints else None
        self.create_subscription(Imu, '/imu/data', self.on_imu, 10)
        self.create_service(Trigger, '/reset_position', self.on_reset)
        self.create_timer(1.0, self.publish_ground)
        self.create_timer(0.5, self.print_status)
        self.create_timer(1.0 / 30.0, self.publish_drive)
        self.get_logger().info('Waiting for /imu/data. Keep the IMU still for ~3 s. Then slide it forward/back '
                               'to drive and twist it to turn. Pause briefly now and then.')

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_stamp is None else min(max(stamp - self.last_stamp, 0.0), 0.05)
        self.last_stamp = stamp
        self.last_header_stamp = msg.header.stamp

        o = msg.orientation
        q_imu = (o.w, o.x, o.y, o.z)
        q_rover = quat_multiply(q_imu, self.mount_inverse)
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.model.update(dt, q_imu, q_rover, (a.x, a.y, a.z), (g.x, g.y, g.z))

        # Which way the rover faces (yaw), and how fast that's changing (turn rate)
        fx, fy, _ = rotate(q_rover, (1.0, 0.0, 0.0))
        if math.hypot(fx, fy) > 1e-3:
            new_yaw = math.atan2(fy, fx)
            if self.yaw is not None and dt > 0.0:
                change = math.atan2(math.sin(new_yaw - self.yaw), math.cos(new_yaw - self.yaw))
                self.turn_rate += 0.3 * (change / dt - self.turn_rate)
            self.yaw = new_yaw

        # Spin the wheels like a differential drive: each side rolls (speed -/+ turn x half width)
        v, half = self.model.speed, self.wheel_separation / 2.0
        self.left_angle = (self.left_angle + (v - self.turn_rate * half) / self.wheel_radius * dt) % (2 * math.pi)
        self.right_angle = (self.right_angle + (v + self.turn_rate * half) / self.wheel_radius * dt) % (2 * math.pi)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = self.rover_frame
        t.transform.translation.x = self.model.x
        t.transform.translation.y = self.model.y
        t.transform.translation.z = self.height
        if self.flat_ground:
            # Solid flat ground: the rover can't tip into it. Keep only which way it faces (yaw).
            q_show = quat_from_rpy(0.0, 0.0, self.yaw or 0.0)
        else:
            q_show = q_rover
        (t.transform.rotation.w, t.transform.rotation.x,
         t.transform.rotation.y, t.transform.rotation.z) = q_show
        self.broadcaster.sendTransform(t)

        # Trail: add a point every ~10 cm moved
        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or math.hypot(self.model.x - last.x, self.model.y - last.y) > 0.1:
            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x, pose.pose.position.y = self.model.x, self.model.y
            self.path.poses = (self.path.poses + [pose])[-2000:]
            self.path.header.stamp = t.header.stamp
            self.path_pub.publish(self.path)

    def publish_drive(self):
        cmd = Twist()
        cmd.linear.x = float(self.model.speed)     # forward speed, m/s
        cmd.angular.z = float(self.turn_rate)      # turn rate, rad/s (+ = left)
        self.twist_pub.publish(cmd)
        if self.joint_pub is not None and self.last_header_stamp is not None:
            js = JointState()
            js.header.stamp = self.last_header_stamp
            js.name = self.left_joints + self.right_joints
            js.position = ([self.left_angle] * len(self.left_joints)
                           + [self.right_angle] * len(self.right_joints))
            self.joint_pub.publish(js)

    def print_status(self):
        mdl = self.model
        self.get_logger().info(
            f'x {mdl.x:6.2f}  y {mdl.y:6.2f}  speed {mdl.speed:5.2f} m/s  turn {self.turn_rate:+5.2f} rad/s | '
            f'push fwd/back {mdl.peak_front:+5.2f}  sideways {mdl.peak_side:+5.2f} m/s^2 | {mdl.status}'
            f'{f"  (safety resets: {mdl.errors_caught})" if mdl.errors_caught else ""}')
        mdl.peak_front = mdl.peak_side = 0.0

    def on_reset(self, request, response):
        self.model.x = self.model.y = self.model.hand_speed = 0.0   # keep what it learned about gravity
        self.path.poses = []
        self.path_pub.publish(self.path)
        response.success, response.message = True, 'Rover moved back to (0, 0)'
        return response

    def publish_ground(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.ns, m.id = 'ground', 0
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose.position.z = -0.027            # top surface just under z = 0, so RViz's grid shows on it
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = 40.0, 40.0, 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.76, 0.60, 0.42, 1.0   # sand
        self.ground_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRoverMotion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()