import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def wrap(angle):
    """Put an angle into -pi..pi, so 'turn 350 deg left' becomes 'turn 10 deg right'."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Keep value between -limit and +limit."""
    return max(-limit, min(limit, value))


class SquareClosedLoop(Node):

    def __init__(self):
        super().__init__('square_closed_loop')

        # --- Settings (parameters).
        self.declare_parameter('side_length', 2.0)   # size of the square, turtle units
        self.declare_parameter('max_speed', 1.0)     # fastest forward speed, units/s
        self.declare_parameter('max_turn', 1.0)      # fastest turn rate, rad/s
        self.side = self.get_parameter('side_length').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value

        self.arrive_distance = 0.02   # "at the corner" when closer than this
        self.aim_tolerance = 0.02     # rad (~1 deg): aimed well enough to start driving
        self.off_course = 0.5         # rad (~30 deg): knocked off course, stop and re-aim

        # --- Input: where the turtle is and which way it faces (~60 times/s).
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

        # --- Output: speed + turn commands.
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # --- The plan: 4 corners (worked out once we know where we start),
        #     which corner we're heading to, and whether we're aiming or driving.
        self.corners = None
        self.target = 0
        self.mode = 'aim'
        self.finished = False
        self.start_time = self.now()

        self.timer = self.create_timer(0.05, self.tick)   # 20 times/s

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pose(self, msg):
        self.pose = msg                       # just remember the latest; tick() uses it

    def tick(self):
        # 0. Don't start until we know where we are AND turtlesim is listening to us.
        #    Commands sent before that are silently lost.
        if self.pose is None or self.publisher.get_subscription_count() == 0:
            self.get_logger().info('Waiting for turtlesim...', throttle_duration_sec=2.0)
            return
        if self.corners is None:
            self.plan_square()
            return

        # 1. Measure: where is the corner relative to me?
        target_x, target_y = self.corners[self.target]
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.hypot(dx, dy)                                  # how far away
        heading_error = wrap(math.atan2(dy, dx) - self.pose.theta)     # how far I must turn to face it

        # 2. Arrived? Stop, then go for the next corner (or finish).
        if distance < self.arrive_distance:
            self.publisher.publish(Twist())
            self.get_logger().info(
                f'Reached corner {self.target + 1} at ({self.pose.x:.2f}, {self.pose.y:.2f})')
            self.target += 1
            self.mode = 'aim'
            if self.target == len(self.corners):
                self.get_logger().info('Square complete')
                self.finished = True
            return

        # 3. Decide: aim first, then drive. Re-aim if something knocks us off course.
        if self.mode == 'aim' and abs(heading_error) < self.aim_tolerance:
            self.mode = 'drive'
        elif self.mode == 'drive' and abs(heading_error) > self.off_course:
            self.mode = 'aim'

        # 4. Act: turn harder the further off we are; slow down as we get close.
        cmd = Twist()
        cmd.angular.z = clamp(3.0 * heading_error, self.max_turn)
        if self.mode == 'drive':
            cmd.linear.x = min(self.max_speed, 1.5 * distance)
        self.publisher.publish(cmd)

    def plan_square(self):
        x0, y0 = self.pose.x, self.pose.y
        s = self.side
        # Left turns, sides lined up with the window: right, up, left, back down to start.
        self.corners = [(x0 + s, y0), (x0 + s, y0 + s), (x0, y0 + s), (x0, y0)]
        self.get_logger().info(
            f'Connected after {self.now() - self.start_time:.1f} s. Start ({x0:.2f}, {y0:.2f}), '
            f'heading {math.degrees(self.pose.theta):.0f} deg. Square side {s}.')
        # The turtlesim window is 0..11 in each direction: refuse a square that won't fit.
        if not all(0.5 < x < 10.5 and 0.5 < y < 10.5 for x, y in self.corners):
            self.get_logger().error('Square would hit the wall. Reset the turtle or use a smaller side_length.')
            self.finished = True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SquareClosedLoop()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.publisher.publish(Twist())          # last message: all zeros = stop
    node.get_logger().info('Sent stop command, exiting')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()