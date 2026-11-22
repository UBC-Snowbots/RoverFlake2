#!/usr/bin/env python3
"""
Drive a square with the rover, using its sensors to know when each side and each corner is done.

It only talks to ROS topics, so the same script runs against the SimFlake sim and a real rover:
  sends   /cmd_vel    geometry_msgs/Twist     drive commands (m/s forward, rad/s turning)
  reads   /odom       nav_msgs/Odometry       how far the rover has driven   -> ends each side
  reads   /imu/data   sensor_msgs/Imu         which way the rover is facing  -> ends each corner
  reads   /gps/fix    sensor_msgs/NavSatFix   optional, only to check the result at the end

Run it (ROS sourced, and the sim playing or the rover's drive running):
  python3 square_drive.py
  python3 square_drive.py --ros-args -p side_length:=1.0 -p turn_direction:=right

If the robot doesn't have one of those sensors, fall back to timing for that part:
  -p distance_source:=time    no /odom: drive each side for side_length / linear_speed seconds
  -p heading_source:=time     no IMU:   turn each corner for 90 deg / turn_speed seconds
  -p heading_source:=odom     use odometry's heading instead of the IMU's

If /odom or the IMU goes quiet mid-run (sim froze, link hiccup), the rover holds still and the square carries on
when they're back (gives up after sensor_timeout_s, default 15 s).

Ctrl+C stops the rover. Each run saves the path to square_<date>_<time>.csv in the current folder.
"""

import csv
import math
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

EARTH_RADIUS_M = 6378137.0


def wrap(angle):
    """Fold an angle into -pi..pi, so 350 degrees counts as -10 degrees."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    """Heading (rotation about the vertical axis) from a ROS orientation quaternion. 0 = +x, counter-clockwise positive."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SquareController:
    """The square itself, with no ROS in it.

    Every tick it gets the time, position and heading, and answers with the speed to command.
    Each of the 4 sides is: drive straight, pause, turn 90 degrees, pause.
    Corner targets are measured from the starting heading (90, 180, 270, 360 degrees), not from the
    previous corner, so a small miss at one corner doesn't add up over the square.
    """

    def __init__(self, side_length=2.0, linear_speed=0.3, turn_speed=0.6, min_turn_speed=0.35,
                 turn_left=True, heading_tolerance_deg=2.0, use_odom_distance=True, use_heading=True,
                 pause_s=0.5):
        self.side_length = side_length
        self.linear_speed = linear_speed
        self.turn_speed = turn_speed
        self.min_turn_speed = min(min_turn_speed, turn_speed)
        self.sign = 1.0 if turn_left else -1.0
        self.heading_tolerance = math.radians(heading_tolerance_deg)
        self.use_odom_distance = use_odom_distance
        self.use_heading = use_heading
        self.pause_s = pause_s

        self.phase = 'idle'
        self.side = 0              # which side we're on, 0..3
        self.done = False
        self.failed = None         # reason, if the run had to stop
        self.messages = []         # progress lines for the caller to print

    def start(self, now, x, y, yaw):
        self.start_heading = yaw
        self._begin('drive', now, x, y)

    def target_heading(self, corner):
        return self.start_heading + self.sign * corner * math.pi / 2

    def _begin(self, phase, now, x, y):
        self.phase = phase
        self.t0 = now
        self.x0, self.y0 = x, y

    def pause_for(self, seconds):
        """Time spent holding still (sensors gone quiet) doesn't count toward the current move."""
        self.t0 += seconds

    def _fail(self, reason):
        self.failed = reason
        self.phase = 'failed'
        return 0.0, 0.0

    def update(self, now, x, y, yaw):
        """Returns (linear m/s, angular rad/s) to command right now."""
        if self.done or self.failed:
            return 0.0, 0.0
        elapsed = now - self.t0

        if self.phase == 'drive':
            target = self.target_heading(self.side)
            if self.use_odom_distance:
                gone = math.hypot(x - self.x0, y - self.y0)
            else:
                gone = self.linear_speed * elapsed
            if gone >= self.side_length:
                line = f'side {self.side + 1}/4 done: drove {gone:.2f} m'
                if self.use_heading:
                    line += f', heading off by {math.degrees(wrap(yaw - target)):+.1f} deg'
                self.messages.append(line)
                self._begin('pause_after_drive', now, x, y)
                return 0.0, 0.0
            if elapsed > self.side_length / self.linear_speed * 3 + 5:
                return self._fail(f'side {self.side + 1}: only {gone:.2f} m after {elapsed:.0f} s. '
                                  'Stuck, or not moving at all?')
            speed = self.linear_speed
            if self.use_odom_distance:
                # slow down over the last few tens of cm so it doesn't roll past the corner
                speed = clamp(self.side_length - gone, 0.15, self.linear_speed)
            steer = 0.0
            if self.use_heading:
                # steer back toward this side's heading if it drifts
                steer = clamp(1.5 * wrap(target - yaw), -0.3, 0.3)
            return speed, steer

        if self.phase == 'pause_after_drive':
            if elapsed >= self.pause_s:
                self._begin('turn', now, x, y)
            return 0.0, 0.0

        if self.phase == 'turn':
            target = self.target_heading(self.side + 1)
            if self.use_heading:
                error = wrap(target - yaw)
                if abs(error) < self.heading_tolerance:
                    turned = math.degrees(wrap(yaw - self.target_heading(self.side)))
                    self.messages.append(f'corner {self.side + 1}/4 done: turned {turned:+.1f} deg '
                                         f'(target {math.degrees(self.sign * math.pi / 2):+.0f})')
                    self._begin('pause_after_turn', now, x, y)
                    return 0.0, 0.0
                if elapsed > (math.pi / 2) / self.min_turn_speed * 5 + 10:
                    return self._fail(f'corner {self.side + 1}: still {math.degrees(error):.0f} deg short after '
                                      f'{elapsed:.0f} s. The wheels may not be strong enough to turn at low '
                                      'speed: try -p min_turn_speed:=0.6')
                # turn fast when far from the target, slower near it (but never below min_turn_speed)
                rate = clamp(1.5 * abs(error), self.min_turn_speed, self.turn_speed)
                return 0.0, math.copysign(rate, error)
            if elapsed >= (math.pi / 2) / self.turn_speed:
                self.messages.append(f'corner {self.side + 1}/4 done (timed, {elapsed:.1f} s)')
                self._begin('pause_after_turn', now, x, y)
                return 0.0, 0.0
            return 0.0, self.sign * self.turn_speed

        if self.phase == 'pause_after_turn':
            if elapsed >= self.pause_s:
                self.side += 1
                if self.side == 4:
                    self.done = True
                    self.phase = 'done'
                else:
                    self._begin('drive', now, x, y)
            return 0.0, 0.0

        return 0.0, 0.0


class SquareDriver(Node):
    """Connects SquareController to ROS: reads the sensors, sends /cmd_vel 20 times a second."""

    def __init__(self):
        super().__init__('square_driver')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.side_length = float(param('side_length', 2.0))
        self.linear_speed = float(param('linear_speed', 0.3))
        self.turn_speed = float(param('turn_speed', 0.6))
        self.min_turn_speed = float(param('min_turn_speed', 0.35))
        self.turn_direction = str(param('turn_direction', 'left'))
        self.heading_tolerance_deg = float(param('heading_tolerance_deg', 2.0))
        self.distance_source = str(param('distance_source', 'odom'))   # odom | time
        self.heading_source = str(param('heading_source', 'imu'))      # imu | odom | time
        self.rate_hz = float(param('rate_hz', 20.0))
        self.log_csv = bool(param('log_csv', True))
        # If a sensor we steer by goes quiet for sensor_stale_s, hold still (keep sending stop) until it's back.
        # Give up after sensor_timeout_s.
        self.sensor_stale_s = float(param('sensor_stale_s', 0.5))
        self.sensor_timeout_s = float(param('sensor_timeout_s', 15.0))
        cmd_vel_topic = str(param('cmd_vel_topic', '/cmd_vel'))
        odom_topic = str(param('odom_topic', '/odom'))
        imu_topic = str(param('imu_topic', '/imu/data'))
        gps_topic = str(param('gps_topic', '/gps/fix'))
        self.topics = {'odom': odom_topic, 'imu': imu_topic, 'cmd_vel': cmd_vel_topic, 'gps': gps_topic}

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        # sensor-data QoS accepts both "reliable" and "best effort" publishers
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        # GPS isn't used for driving, only to check the result at the end (in the sim it's the true position)
        self.create_subscription(NavSatFix, gps_topic, self.on_gps, qos_profile_sensor_data)

        self.odom = None            # (x, y, yaw) from the latest /odom
        self.imu_yaw = None
        self.imu_has_orientation = True
        self.gps = None             # (lat, lon)
        self.last_odom = self.last_imu = None
        self.stopped = True         # True once a full stop has been sent and nothing has moved since

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.last_odom = time.monotonic()

    def on_imu(self, msg):
        q = msg.orientation
        # An IMU that only measures turning rate (no filter running) reports orientation covariance -1
        # or an all-zero quaternion: there's no heading in it.
        if msg.orientation_covariance[0] == -1.0 or (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0):
            self.imu_has_orientation = False
            return
        self.imu_has_orientation = True
        self.imu_yaw = yaw_from_quaternion(q)
        self.last_imu = time.monotonic()

    def on_gps(self, msg):
        if not (math.isnan(msg.latitude) or math.isnan(msg.longitude)):
            self.gps = (msg.latitude, msg.longitude)

    @staticmethod
    def gps_offset_m(a, b):
        """(north, east) metres from GPS point a to GPS point b."""
        north = math.radians(b[0] - a[0]) * EARTH_RADIUS_M
        east = math.radians(b[1] - a[1]) * EARTH_RADIUS_M * math.cos(math.radians(a[0]))
        return north, east

    def needed(self):
        need = set()
        if self.distance_source == 'odom' or self.heading_source == 'odom':
            need.add('odom')
        if self.heading_source == 'imu':
            need.add('imu')
        return need

    def pose(self):
        """(x, y, heading) using the chosen sources. Missing parts are 0 (unused in time mode)."""
        x, y = (self.odom[0], self.odom[1]) if self.odom else (0.0, 0.0)
        if self.heading_source == 'imu':
            yaw = self.imu_yaw if self.imu_yaw is not None else 0.0
        elif self.heading_source == 'odom':
            yaw = self.odom[2] if self.odom else 0.0
        else:
            yaw = 0.0
        return x, y, yaw

    def send(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
        if linear or angular:
            self.stopped = False

    def stop(self, seconds=1.0):
        """Send stop for a full second, so it still lands if the link hiccups for a moment."""
        for _ in range(max(1, int(seconds / 0.05))):
            self.send(0.0, 0.0)
            time.sleep(0.05)
        self.stopped = True

    def quiet_sensors(self, now):
        """Sensors this run steers by that have gone quiet, and the ones still arriving."""
        quiet, arriving = [], []
        for key, last in (('odom', self.last_odom), ('imu', self.last_imu)):
            fresh = last is not None and now - last <= self.sensor_stale_s
            if key in self.needed() and not fresh:
                quiet.append(self.topics[key])
            elif fresh:
                arriving.append(self.topics[key])
        return quiet, arriving

    def check_settings(self):
        problems = []
        if self.distance_source not in ('odom', 'time'):
            problems.append(f"distance_source must be odom or time, not '{self.distance_source}'")
        if self.heading_source not in ('imu', 'odom', 'time'):
            problems.append(f"heading_source must be imu, odom or time, not '{self.heading_source}'")
        if self.turn_direction not in ('left', 'right'):
            problems.append(f"turn_direction must be left or right, not '{self.turn_direction}'")
        if self.side_length <= 0 or self.linear_speed <= 0 or self.turn_speed <= 0 or self.min_turn_speed <= 0:
            problems.append('side_length, linear_speed, turn_speed and min_turn_speed must be above 0')
        for p in problems:
            self.get_logger().error(p)
        return not problems

    def wait_for_inputs(self, seconds=5.0):
        log = self.get_logger()
        need = self.needed()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            have = {'odom'} if self.odom else set()
            if self.imu_yaw is not None:
                have.add('imu')
            if need <= have and self.cmd_pub.get_subscription_count() > 0:
                # If something publishes GPS, grab a starting fix too (only used to check the result)
                gps_deadline = time.monotonic() + 2.0
                while (self.gps is None and self.count_publishers(self.topics['gps']) > 0
                       and time.monotonic() < gps_deadline):
                    rclpy.spin_once(self, timeout_sec=0.1)
                return True
        if 'imu' in need and self.imu_yaw is None:
            if not self.imu_has_orientation:
                log.error(f"{self.topics['imu']} has no orientation in it (raw IMU, no filter running). "
                          'Start the IMU filter, or use -p heading_source:=odom or -p heading_source:=time')
            else:
                log.error(f"Nothing on {self.topics['imu']}. Use -p heading_source:=odom or "
                          '-p heading_source:=time if this robot has no IMU')
        if 'odom' in need and self.odom is None:
            log.error(f"Nothing on {self.topics['odom']}. Use -p distance_source:=time (and "
                      'heading_source imu or time) if this robot has no odometry')
        if self.cmd_pub.get_subscription_count() == 0:
            log.error(f"Nothing is listening on {self.topics['cmd_vel']}. Is the sim playing / the rover's "
                      'drive running, and on the same ROS_DOMAIN_ID as this terminal?')
        return False

    def run(self):
        log = self.get_logger()
        if not self.check_settings():
            return
        log.info(f'side {self.side_length} m, {self.linear_speed} m/s, turning {self.turn_direction} '
                 f'at up to {self.turn_speed} rad/s | distance from: {self.distance_source}, '
                 f'heading from: {self.heading_source}')
        log.info('Waiting for sensors and a /cmd_vel listener...')
        if not self.wait_for_inputs():
            return

        ctrl = SquareController(
            side_length=self.side_length, linear_speed=self.linear_speed, turn_speed=self.turn_speed,
            min_turn_speed=self.min_turn_speed, turn_left=(self.turn_direction == 'left'),
            heading_tolerance_deg=self.heading_tolerance_deg,
            use_odom_distance=(self.distance_source == 'odom'),
            use_heading=(self.heading_source != 'time'))

        writer = csv_file = None
        if self.log_csv:
            path = datetime.now().strftime('square_%Y%m%d_%H%M%S.csv')
            csv_file = open(path, 'w', newline='')
            writer = csv.writer(csv_file)
            writer.writerow(['t_s', 'est_x_m', 'est_y_m', 'heading_deg', 'gps_north_m', 'gps_east_m',
                             'cmd_linear', 'cmd_angular', 'phase'])

        start = time.monotonic()
        x, y, yaw = self.pose()
        start_heading = yaw
        start_gps = self.gps
        ctrl.start(start, x, y, yaw)
        log.info('Driving. Ctrl+C to stop.')

        # The script's own idea of where the rover is: distance travelled from odometry, direction from the
        # chosen heading source. Wheel odometry's own x/y goes wrong when the wheels slip in turns (its heading
        # is wrong), but the distance it reports driving straight is still good.
        track = self.distance_source == 'odom' and self.heading_source != 'time'
        est_x = est_y = 0.0
        prev_xy = (x, y)

        period = 1.0 / self.rate_hz
        next_tick = time.monotonic()
        hold_since = None           # when the sensors went quiet, while holding still
        lost_sensor = False
        try:
            while rclpy.ok() and not ctrl.done and not ctrl.failed:
                rclpy.spin_once(self, timeout_sec=max(0.0, next_tick - time.monotonic()))
                now = time.monotonic()
                if now < next_tick:
                    continue
                next_tick = max(next_tick + period, now - period)   # don't try to catch up after a stall

                # A sensor we steer by went quiet: hold still and keep sending stop every tick (so it lands
                # as soon as the link is back), then carry on. Give up if it stays quiet too long.
                quiet, arriving = self.quiet_sensors(now)
                if quiet:
                    self.send(0.0, 0.0)
                    if hold_since is None:
                        hold_since = now
                        if arriving:
                            why = f"but {', '.join(arriving)} is still arriving"
                        else:
                            why = 'nothing is arriving from the robot/sim at all (froze, or lost its ROS link?)'
                        log.warn(f"No {', '.join(quiet)} for {self.sensor_stale_s} s, {why}. "
                                 'Holding still until it comes back...')
                    elif now - hold_since > self.sensor_timeout_s:
                        ctrl.failed = f"{', '.join(quiet)} didn't come back within {self.sensor_timeout_s:.0f} s"
                        lost_sensor = True
                        break
                    continue
                if hold_since is not None:
                    log.info(f'Sensors back after {now - hold_since:.1f} s. Continuing.')
                    ctrl.pause_for(now - hold_since)
                    hold_since = None

                x, y, yaw = self.pose()
                if track:
                    step = math.hypot(x - prev_xy[0], y - prev_xy[1])
                    est_x += step * math.cos(yaw - start_heading)
                    est_y += step * math.sin(yaw - start_heading)
                prev_xy = (x, y)
                linear, angular = ctrl.update(now, x, y, yaw)
                self.send(linear, angular)
                for line in ctrl.messages:
                    log.info(line)
                ctrl.messages.clear()
                if writer:
                    gn, ge = self.gps_offset_m(start_gps, self.gps) if (start_gps and self.gps) else ('', '')
                    writer.writerow([f'{now - start:.3f}', f'{est_x:.4f}' if track else '',
                                     f'{est_y:.4f}' if track else '', f'{math.degrees(yaw - start_heading):.2f}',
                                     f'{gn:.4f}' if gn != '' else '', f'{ge:.4f}' if ge != '' else '',
                                     f'{linear:.3f}', f'{angular:.3f}', ctrl.phase])
        finally:
            self.stop()
            if csv_file:
                csv_file.close()

        if ctrl.failed:
            log.error(f'Stopped: {ctrl.failed}')
            if lost_sensor:
                log.error('The stop command may not have reached the robot. If it is still moving: in the sim, '
                          "press Play twice in Unity; or run  ros2 topic pub --once /cmd_vel "
                          "geometry_msgs/msg/Twist '{}'")
            return

        # How close did it get back to where it started? (0 = perfect square)
        log.info(f'Square finished in {time.monotonic() - start:.1f} s.')
        if track:
            log.info(f'Own estimate (odometry distance + {self.heading_source} heading): ended '
                     f'{math.hypot(est_x, est_y):.2f} m from the start.')
        if start_gps and self.gps:
            north, east = self.gps_offset_m(start_gps, self.gps)
            log.info(f'GPS: ended {math.hypot(north, east):.2f} m from the start '
                     f'(north {north:+.2f}, east {east:+.2f}). In the sim this is the true position; '
                     'on a real rover GPS is only good to a few metres.')
        if self.heading_source != 'time':
            _, _, yaw = self.pose()
            log.info(f'Facing {math.degrees(wrap(yaw - start_heading)):+.1f} deg from its starting direction '
                     f'({self.heading_source}).')
        if csv_file:
            log.info(f'Path saved to {csv_file.name}')


def main():
    # Handle Ctrl+C ourselves, so ROS is still running long enough to send the stop command
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SquareDriver()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C: stopping the rover')
        if not node.stopped:
            node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
