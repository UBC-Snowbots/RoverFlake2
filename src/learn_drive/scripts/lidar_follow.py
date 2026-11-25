#!/usr/bin/env python3
"""Follow a person using only the lidar, keeping about 2 m away.

What it does, every time a new lidar turn (/scan) arrives:
  1. Keeps only readings in front of the rover (a cone, default +-60 degrees) and further than
     min_range. Readings to the sides and behind are the rover's own body, so they are ignored.
  2. Groups neighbouring readings into blobs. A wall makes a long blob; a person's legs make a
     small blob (up to ~0.9 m wide).
  3. Not locked on yet: locks onto the closest small blob that is straight-ish ahead (+-30 deg)
     and within 3 m. That's you, standing in front of the rover when you start it.
     Locked on: picks the small blob nearest to where you were last time.
  4. Drives: turns toward you (faster the further off to the side you are) and drives forward
     (faster the further you are beyond 2 m). Stops at 2 m. Never reverses.

Safety:
  - Loses you for 1 s -> stops. Step back in front of it (within 3 m) to start again.
  - Anything closer than 0.7 m straight ahead -> no driving forward.
  - No lidar data for 1 s -> stops.
  - Sends /cmd_vel 20 times a second (the mini rover stops after 0.5 s without commands).
  - Ctrl+C -> sends stop.
  - dry_run:=true -> does everything except drive, so you can check what it sees.

Distances are measured from the lidar (front middle of the rover), x forward, y left.

Run:
  python3 lidar_follow.py                                   # laptop simulation
  python3 lidar_follow.py --ros-args -p dry_run:=true       # check what it sees, no driving
  python3 lidar_follow.py --ros-args -p turn_sign:=-1 -p min_linear_speed:=0.5 -p min_turn_speed:=0.4
"""
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped, Twist
    from sensor_msgs.msg import LaserScan
except ImportError:          # lets the follow logic be tested without ROS installed
    rclpy = None
    Node = object


# ---------------------------------------------------------------- settings
DEFAULTS = {
    # where to find things
    'scan_topic': '/scan',
    'cmd_vel_topic': '/cmd_vel',
    'target_topic': '/follow/target',     # where it thinks you are, for RViz
    # how the lidar sits on the rover
    'lidar_yaw_deg': 0.0,        # which way the lidar's 0 deg points, relative to the rover's front (+ = left)
    'view_half_angle_deg': 60.0, # only use readings within this angle of straight ahead (rest = rover body)
    'min_range': 0.35,           # m: ignore anything closer (rover parts in front of the lidar)
    'max_range': 6.0,            # m: ignore anything further
    # recognising a person
    'cluster_gap': 0.3,          # m: readings closer together than this belong to the same blob...
    'cluster_gap_per_m': 0.05,   # ...plus this much per metre of distance (readings spread out further away)
    'min_points': 2,             # a blob needs at least this many readings
    'max_width': 0.9,            # m: wider blobs are walls/furniture, not a person
    'start_half_angle_deg': 30.0,  # lock-on: you must be within this angle of straight ahead...
    'start_max_distance': 3.0,     # ...and closer than this
    'max_jump': 0.5,             # m: how far you can be from where it expected you...
    'walk_speed': 1.5,           # m/s: ...plus how far you could walk since it last saw you
    'lost_timeout': 1.0,         # s: not seen for this long (and at least 3 lidar turns) -> stop and unlock
    # driving
    'follow_distance': 2.0,      # m: stop this far from you
    'distance_deadband': 0.2,    # m: within this of follow_distance counts as "there"
    'k_linear': 0.5,             # forward speed per metre beyond follow_distance
    'max_linear_speed': 0.6,     # m/s
    'min_linear_speed': 0.0,     # m/s: the rover doesn't move below some command; set that here
    'k_turn': 1.5,               # turn speed per radian off-centre
    'max_turn_speed': 1.0,       # rad/s
    'min_turn_speed': 0.0,       # rad/s: same idea as min_linear_speed
    'turn_deadband_deg': 8.0,    # within this angle of straight ahead, don't turn
    'drive_max_angle_deg': 35.0, # further off-centre than this: turn on the spot first, then drive
    'stop_distance': 0.7,        # m: anything this close straight ahead -> no forward driving
    'safety_half_angle_deg': 25.0,
    'turn_sign': 1,              # -1 if a + (left) turn command makes the rover turn right
    'rate_hz': 20.0,             # commands per second
    'scan_timeout': 1.0,         # s: no lidar data for this long -> stop
    'dry_run': False,            # True: work out everything, print it, but don't drive
}


def wrap(a):
    """Angle in radians, wrapped to -pi..pi."""
    return math.atan2(math.sin(a), math.cos(a))


class Settings:
    def __init__(self, **overrides):
        values = dict(DEFAULTS)
        values.update(overrides)
        for k, v in values.items():
            setattr(self, k, v)
        # handy radian versions
        self.lidar_yaw = math.radians(self.lidar_yaw_deg)
        self.view_half_angle = math.radians(self.view_half_angle_deg)
        self.start_half_angle = math.radians(self.start_half_angle_deg)
        self.turn_deadband = math.radians(self.turn_deadband_deg)
        self.drive_max_angle = math.radians(self.drive_max_angle_deg)
        self.safety_half_angle = math.radians(self.safety_half_angle_deg)


# ---------------------------------------------------------------- reading the scan
def front_points(ranges, angle_min, angle_increment, s):
    """Readings we trust, in front of the rover, ordered from right to left.

    Each point is (x, y, angle, distance): x forward, y left, angle relative to the rover's front."""
    pts = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < s.min_range or r > s.max_range:
            continue                     # NaN = no reading, inf = nothing hit
        a = wrap(angle_min + i * angle_increment + s.lidar_yaw)
        if abs(a) > s.view_half_angle:
            continue                     # to the side or behind: that's the rover itself
        pts.append((a, r))
    pts.sort()
    return [(r * math.cos(a), r * math.sin(a), a, r) for a, r in pts]


class Blob:
    def __init__(self, pts):
        self.n = len(pts)
        self.x = sum(p[0] for p in pts) / self.n
        self.y = sum(p[1] for p in pts) / self.n
        self.width = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
        self.distance = math.hypot(self.x, self.y)
        self.angle = math.atan2(self.y, self.x)


def find_blobs(pts, s):
    """Group neighbouring readings: a new blob starts where there's a gap."""
    blobs, cur = [], []
    for p in pts:
        if cur:
            gap = math.hypot(p[0] - cur[-1][0], p[1] - cur[-1][1])
            if gap > s.cluster_gap + s.cluster_gap_per_m * p[3]:
                blobs.append(Blob(cur))
                cur = []
        cur.append(p)
    if cur:
        blobs.append(Blob(cur))
    return blobs


def person_sized(blobs, s):
    return [b for b in blobs if b.n >= s.min_points and b.width <= s.max_width]


# ---------------------------------------------------------------- the follower
class Follower:
    """All the decisions, no ROS. Feed it scans with update(); ask command() what to send."""

    def __init__(self, s):
        self.s = s
        self.target = None        # (x, y) where it thinks you are, from the lidar, x forward y left
        self.last_seen = None     # time it last saw you
        self.last_scan = None     # time of the last scan
        self.scan_period = 0.2    # s per lidar turn (measured as scans arrive)
        self.sent = (0.0, 0.0)    # (forward, turn) it last told the rover, before turn_sign
        self.nearest_ahead = math.inf
        self.nearest_in_view = None   # (distance, angle) of the closest reading, for the status line
        self.event = None             # 'locked' / 'lost' happened in the last update, for logging

    def update(self, ranges, angle_min, angle_increment, now):
        s = self.s
        dt = 0.0 if self.last_scan is None else min(now - self.last_scan, 1.0)
        if dt > 0.0:
            self.scan_period += 0.2 * (dt - self.scan_period)     # running average time per lidar turn
        self.last_scan = now
        self.event = None

        pts = front_points(ranges, angle_min, angle_increment, s)
        self.nearest_in_view = min(((p[3], p[2]) for p in pts), default=None)
        ahead = [p[3] for p in pts if abs(p[2]) <= s.safety_half_angle]
        self.nearest_ahead = min(ahead, default=math.inf)
        cands = person_sized(find_blobs(pts, s), s)

        # The rover moved since the last scan, so where you were has shifted relative to it
        if self.target is not None and dt > 0.0:
            v, w = self.sent
            x, y = self.target[0] - v * dt, self.target[1]
            th = -w * dt
            self.target = (x * math.cos(th) - y * math.sin(th), x * math.sin(th) + y * math.cos(th))

        if self.target is None:
            starts = [b for b in cands
                      if abs(b.angle) <= s.start_half_angle and b.distance <= s.start_max_distance]
            if starts:
                b = min(starts, key=lambda b: b.distance)
                self.target, self.last_seen, self.event = (b.x, b.y), now, 'locked'
            return

        # How far you could have walked since it last saw you (more if it missed you in a turn or two)
        allowed = s.max_jump + s.walk_speed * (now - self.last_seen)
        best = min(cands, key=lambda b: math.hypot(b.x - self.target[0], b.y - self.target[1]), default=None)
        if best is not None and math.hypot(best.x - self.target[0], best.y - self.target[1]) <= allowed:
            self.target, self.last_seen = (best.x, best.y), now
        elif now - self.last_seen > self.lost_after():
            self.target, self.event = None, 'lost'

    def lost_after(self):
        """Seconds without seeing you before giving up: lost_timeout, but at least 3 lidar turns."""
        return max(self.s.lost_timeout, 3.0 * self.scan_period)

    def command(self, now):
        """(forward m/s, turn rad/s with + = left, reason). Before turn_sign is applied."""
        s = self.s
        if self.last_scan is None or now - self.last_scan > s.scan_timeout:
            return 0.0, 0.0, 'no lidar data'
        if self.target is None:
            return 0.0, 0.0, 'searching'
        if now - self.last_seen > self.lost_after():
            return 0.0, 0.0, 'lost'

        x, y = self.target
        dist, ang = math.hypot(x, y), math.atan2(y, x)

        turn = 0.0
        if abs(ang) > s.turn_deadband:
            turn = math.copysign(max(s.min_turn_speed, min(s.max_turn_speed, s.k_turn * abs(ang))), ang)

        fwd = 0.0
        extra = dist - s.follow_distance
        if extra > s.distance_deadband and abs(ang) <= s.drive_max_angle:
            fwd = max(s.min_linear_speed, min(s.max_linear_speed, s.k_linear * extra))
        reason = 'following'
        if fwd > 0.0 and self.nearest_ahead < s.stop_distance:
            fwd, reason = 0.0, 'blocked'
        return fwd, turn, reason


# ---------------------------------------------------------------- ROS node
class LidarFollow(Node):

    def __init__(self):
        super().__init__('lidar_follow')
        values = {name: self.declare_parameter(name, default).value for name, default in DEFAULTS.items()}
        self.s = Settings(**values)
        self.brain = Follower(self.s)
        self.frame_id = 'laser'
        self.status_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.s.cmd_vel_topic, 10)
        self.target_pub = self.create_publisher(PointStamped, self.s.target_topic, 10)
        # Best effort accepts both kinds of publisher (the Sweep driver is reliable)
        self.create_subscription(LaserScan, self.s.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / self.s.rate_hz, self.on_timer)

        s = self.s
        self.get_logger().info(
            f'Lidar follow: keep {s.follow_distance:.1f} m away, up to {s.max_linear_speed} m/s and '
            f'{s.max_turn_speed} rad/s, turn_sign {s.turn_sign}, looking +-{s.view_half_angle_deg:.0f} deg '
            f'from {s.min_range} to {s.max_range} m' + ('  | DRY RUN: not driving' if s.dry_run else ''))
        self.get_logger().info(
            f'Stand {s.start_max_distance:.0f} m or closer, straight in front of the rover, to start. '
            'Ctrl+C to stop.')

    @staticmethod
    def now():
        return time.monotonic()

    def on_scan(self, msg):
        self.frame_id = msg.header.frame_id or self.frame_id
        self.brain.update(list(msg.ranges), msg.angle_min, msg.angle_increment, self.now())
        b = self.brain
        if b.event == 'locked':
            x, y = b.target
            self.get_logger().info(f'Locked on: {math.hypot(x, y):.2f} m away, '
                                   f'{self.side(math.atan2(y, x))}')
        elif b.event == 'lost':
            self.get_logger().warn('Lost you: stopped. Step back in front of the rover to start again.')
        if b.target is not None:
            x, y = b.target
            d, a = math.hypot(x, y), math.atan2(y, x) - self.s.lidar_yaw   # back to the lidar's own angles
            p = PointStamped()
            p.header = msg.header
            p.point.x, p.point.y = d * math.cos(a), d * math.sin(a)
            self.target_pub.publish(p)

    def on_timer(self):
        now = self.now()
        fwd, turn, reason = self.brain.command(now)
        if self.s.dry_run:
            self.brain.sent = (0.0, 0.0)          # the rover isn't moving
        else:
            self.brain.sent = (fwd, turn)
            cmd = Twist()
            cmd.linear.x = float(fwd)
            cmd.angular.z = float(turn * self.s.turn_sign)
            self.cmd_pub.publish(cmd)
        if now - self.status_time >= 1.0:
            self.status_time = now
            self.status(fwd, turn, reason)

    @staticmethod
    def side(a):
        deg = math.degrees(a)
        if abs(deg) < 3:
            return 'straight ahead'
        return f'{abs(deg):.0f} deg {"left" if deg > 0 else "right"}'

    def status(self, fwd, turn, reason):
        b, s = self.brain, self.s
        dry = ' (dry run: not driving)' if s.dry_run else ''
        if reason == 'no lidar data':
            self.get_logger().warn(f'No lidar data on {s.scan_topic}: stopped')
        elif reason in ('searching', 'lost'):
            near = 'nothing'
            if b.nearest_in_view is not None:
                near = f'{b.nearest_in_view[0]:.2f} m, {self.side(b.nearest_in_view[1])}'
            self.get_logger().info(f'Searching: stand within {s.start_max_distance:.0f} m straight in front. '
                                   f'Nearest thing it can see: {near}')
        else:
            x, y = b.target
            what = f'forward {fwd:.2f} m/s'
            if turn:
                what += f', turn {"left" if turn > 0 else "right"} {abs(turn):.2f} rad/s'
            if reason == 'blocked':
                what += f' (something {b.nearest_ahead:.2f} m ahead: not driving forward)'
            self.get_logger().info(f'Following: you are {math.hypot(x, y):.2f} m away, '
                                   f'{self.side(math.atan2(y, x))} -> {what}{dry}')

    def stop(self):
        if self.s.dry_run:
            return
        try:
            for _ in range(3):
                self.cmd_pub.publish(Twist())
        except Exception:
            pass        # the rover stops by itself 0.5 s after commands stop anyway


def main(args=None):
    if rclpy is None:
        raise SystemExit('ROS 2 is not sourced. Run: source /opt/ros/jazzy/setup.bash')
    # Handle Ctrl+C ourselves, so ROS is still running when we send the final stop
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LidarFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info('Stopping the rover')
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
