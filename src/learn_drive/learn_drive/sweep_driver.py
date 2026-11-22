"""ROS 2 driver for the Scanse Sweep lidar: reads it over USB and publishes /scan.

What it does, in order:
  1. Opens the serial port (115200 baud) and tells the Sweep to stop, in case it was mid-scan.
  2. Sets the spin speed and how many distance samples per second it takes (only if different;
     changing them makes the motor re-settle for ~10 s).
  3. Waits until the motor is steady, then says "DS" (start scanning).
  4. Reads 7-byte samples: angle, distance, signal strength. Collects one full turn
     (the Sweep flags the first sample of each turn), then publishes it as one LaserScan.

Run:  ros2 run learn_drive sweep_driver
      ros2 run learn_drive sweep_driver --ros-args -p port:=/dev/ttyUSB1 -p motor_hz:=3
"""
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:
    serial = None

SAMPLE_RATE_CODES = {500: 1, 750: 2, 1000: 3}


class SweepError(Exception):
    pass


class SweepSerial:
    """The Sweep's command language and data stream. No ROS in here."""

    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.1)
        self.buf = b''
        self.counts = {'total': 0, 'error': 0}

    def close(self):
        try:
            self.stop()
        finally:
            self.s.close()

    def _send(self, cmd):
        self.s.write(cmd.encode() + b'\n')
        self.s.flush()

    def _read(self, n, timeout=1.0):
        data = b''
        end = time.time() + timeout
        while len(data) < n and time.time() < end:
            data += self.s.read(n - len(data))
        return data

    def ask(self, cmd, reply_len, timeout=1.0):
        self._send(cmd)
        reply = self._read(reply_len, timeout)
        if len(reply) != reply_len or reply[:2] != cmd[:2].encode():
            raise SweepError(f'no proper reply to "{cmd}" (got {reply!r})')
        return reply.decode(errors='replace')

    def stop(self):
        self._send('DX')
        time.sleep(0.1)
        self.s.reset_input_buffer()
        self._send('DX')
        self._read(6, 0.5)
        self.buf = b''

    def version(self):
        t = self.ask('IV', 21)
        return f'{t[2:7]} firmware {t[9]}.{t[10]} serial {t[12:20]}'

    def motor_hz(self):
        return int(self.ask('MI', 5)[2:4])

    def sample_rate_code(self):
        return int(self.ask('LI', 5)[2:4])

    def set_motor_hz(self, hz):
        t = self.ask(f'MS{hz:02d}', 9)
        if t[5:7] != '00':
            raise SweepError(f'motor speed {hz} refused (status {t[5:7]})')

    def set_sample_rate(self, rate):
        t = self.ask(f'LR{SAMPLE_RATE_CODES[rate]:02d}', 9)
        if t[5:7] != '00':
            raise SweepError(f'sample rate {rate} refused (status {t[5:7]})')

    def wait_until_steady(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                if self.ask('MZ', 5, 0.5)[2:4] == '00':
                    return
            except SweepError:
                pass
            time.sleep(0.5)
        raise SweepError('motor never reported steady')

    def start(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            status = self.ask('DS', 6)[2:4]
            if status == '00':
                return
            if status != '12':                       # 12 = motor not steady yet: keep trying
                raise SweepError(f'start refused (status {status})')
            time.sleep(0.5)
        raise SweepError('start kept being refused: motor not steady')

    def samples(self):
        """Yield (first_of_turn, angle_deg, distance_m, strength) forever.
        A reading flagged as an error still yields its "first of turn" flag, with distance None."""
        while True:
            self.buf += self.s.read(1024)
            while len(self.buf) >= 7:
                p = self.buf[:7]
                if sum(p[:6]) % 255 != p[6]:          # out of step: slide one byte and retry
                    self.buf = self.buf[1:]
                    continue
                self.buf = self.buf[7:]
                sync_err, angle, dist = p[0], *struct.unpack('<HH', p[1:5])
                self.counts['total'] += 1
                if sync_err >> 1:                     # error bits set: keep only the "new turn" flag
                    self.counts['error'] += 1
                    yield bool(sync_err & 1), angle / 16.0, None, 0
                    continue
                yield bool(sync_err & 1), angle / 16.0, dist / 100.0, p[5]


class SweepDriver(Node):

    def __init__(self):
        super().__init__('sweep_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('motor_hz', 5)          # turns per second (1-10). Slower = more points per turn
        self.declare_parameter('sample_rate', 1000)    # distance samples per second: 500, 750 or 1000
        self.declare_parameter('angle_bins', 360)      # LaserScan slots per turn (1 degree each)
        self.declare_parameter('range_min', 0.1)       # m
        self.declare_parameter('range_max', 40.0)      # m
        self.declare_parameter('flip', False)          # True if the room shows up mirror-imaged
        self.declare_parameter('min_strength', 10)     # weaker echoes than this are junk ("no echo")
        p = lambda name: self.get_parameter(name).value   # noqa: E731
        self.port, self.frame_id = p('port'), p('frame_id')
        self.motor_hz, self.sample_rate = int(p('motor_hz')), int(p('sample_rate'))
        self.bins, self.range_min, self.range_max = int(p('angle_bins')), p('range_min'), p('range_max')
        self.flip = p('flip')
        self.min_strength = int(p('min_strength'))
        if self.sample_rate not in SAMPLE_RATE_CODES:
            raise ValueError('sample_rate must be 500, 750 or 1000')

        # Reliable delivery: every kind of listener (RViz, costmaps, ros2 topic) can accept it
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.running = True
        self.dev = None
        self.scans = self.points = self.good = 0
        self.why = {'too close': 0, 'nothing came back': 0, 'echo too weak': 0}
        self.create_timer(5.0, self.report)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def report(self):
        dev = self.dev
        if dev is None:
            return
        c = dev.counts
        rejected = ', '.join(f'{v / 5:.0f} {k}' for k, v in self.why.items() if v)
        self.get_logger().info(
            f'{self.scans / 5:.1f} scans/s, ~{self.points / max(self.scans, 1):.0f} points per turn | '
            f'readings/s: {c["total"] / 5:.0f} received, {self.good / 5:.0f} good, '
            f'{c["error"] / 5:.0f} flagged as errors' + (f' | thrown out: {rejected}' if rejected else ''))
        self.scans = self.points = self.good = 0
        self.why = {'too close': 0, 'nothing came back': 0, 'echo too weak': 0}
        dev.counts = {'total': 0, 'error': 0}

    def run(self):
        while self.running and rclpy.ok():
            dev = None
            try:
                dev = SweepSerial(self.port)
                self.dev = dev
                dev.stop()
                self.get_logger().info(f'Connected: {dev.version()}')
                if self.current(dev.motor_hz) != self.motor_hz:
                    self.get_logger().info(f'Setting spin speed to {self.motor_hz} Hz ...')
                    dev.set_motor_hz(self.motor_hz)
                if self.current(dev.sample_rate_code) != SAMPLE_RATE_CODES[self.sample_rate]:
                    dev.set_sample_rate(self.sample_rate)
                self.get_logger().info('Waiting for the motor to be steady (up to ~10 s) ...')
                dev.wait_until_steady()
                dev.start()
                self.get_logger().info(f'Scanning on /scan (frame "{self.frame_id}")')
                self.stream(dev)
            except serial.SerialException as e:
                msg = str(e)
                hint = f'  Fix: sudo chmod 666 {self.port}' if 'ermission' in msg else ''
                self.get_logger().error(f'Serial problem: {msg}{hint}. Retrying in 2 s.')
            except SweepError as e:
                self.get_logger().error(f'Sweep problem: {e}. Retrying in 2 s.')
            finally:
                if dev is not None:
                    try:
                        dev.close()
                    except Exception:
                        pass
            if self.running:
                time.sleep(2.0)

    @staticmethod
    def current(ask):
        """Read a setting from the Sweep; None if it won't say (then we just set it)."""
        try:
            return ask()
        except (SweepError, ValueError):
            return None

    def stream(self, dev):
        turn, last_angle = [], None
        for first, angle, dist, strength in dev.samples():
            if not self.running:
                return
            # New turn: the Sweep says so, or the angle jumped back past 0 degrees (backup if that flag is lost)
            wrapped = last_angle is not None and angle < last_angle - 180.0
            last_angle = angle
            if (first or wrapped) and turn:
                self.publish(turn)
                turn = []
            if dist is not None:
                turn.append((angle, dist, strength))

    def publish(self, turn):
        inc = 2 * math.pi / self.bins
        ranges = [math.nan] * self.bins          # NaN = "no reading in this direction" (ignored downstream)
        strengths = [0.0] * self.bins
        for angle, dist, strength in turn:
            if dist < self.range_min:
                self.why['too close'] += 1
                continue
            if dist > self.range_max:
                self.why['nothing came back'] += 1
                continue
            if strength < self.min_strength:
                self.why['echo too weak'] += 1
                continue
            self.good += 1
            a = math.radians(-angle if self.flip else angle)
            i = int(round(a / inc)) % self.bins
            if math.isnan(ranges[i]) or dist < ranges[i]:
                ranges[i], strengths[i] = dist, float(strength)
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min, scan.angle_max, scan.angle_increment = 0.0, 2 * math.pi - inc, inc
        scan.scan_time = 1.0 / self.motor_hz
        scan.time_increment = scan.scan_time / self.bins
        scan.range_min, scan.range_max = float(self.range_min), float(self.range_max)
        scan.ranges, scan.intensities = ranges, strengths
        self.pub.publish(scan)
        self.scans += 1
        self.points += len(turn)


def main(args=None):
    if serial is None:
        raise SystemExit('pyserial is missing. Install it with:  sudo apt install python3-serial')
    rclpy.init(args=args)
    node = SweepDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.thread.join(timeout=2.0)       # lets it send "stop scanning" before exit


if __name__ == '__main__':
    main()
