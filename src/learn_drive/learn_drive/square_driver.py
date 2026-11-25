import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # --- Settings (parameters). Change them at run time without editing code.
        self.declare_parameter('forward_speed', 0.3)   # m/s while driving straight
        self.declare_parameter('turn_speed', 0.5)      # rad/s while turning
        self.declare_parameter('side_time', 2.0)       # seconds of driving per side

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.side_time = self.get_parameter('side_time').value
        self.turn_time = (math.pi / 2) / self.turn_speed   # seconds needed to turn 90 degrees

        # --- The output: speed + turn commands on the same channel the joystick uses.
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- The plan: which part of the square we're in, and when that part started.
        self.phase = 'straight'
        self.sides_done = 0
        self.finished = False
        self.phase_start = self.now()

        # --- The clock: run tick() 10 times per second.
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'Driving a square: {self.side_time:.1f} s straight at {self.forward_speed} m/s, '
            f'{self.turn_time:.1f} s turning at {self.turn_speed} rad/s')

    def now(self):
        """Current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        """Runs every 0.1 s: decide which phase we're in, then send its command."""
        elapsed = self.now() - self.phase_start

        # 1. Decide: is the current phase over? If so, move to the next one.
        if self.phase == 'straight' and elapsed >= self.side_time:
            self.switch_to('turn')
        elif self.phase == 'turn' and elapsed >= self.turn_time:
            self.sides_done += 1
            self.switch_to('done' if self.sides_done == 4 else 'straight')

        # 2. Act: send the command for the phase we're in now.
        cmd = Twist()                        # a new Twist is all zeros = stand still
        if self.phase == 'straight':
            cmd.linear.x = self.forward_speed
        elif self.phase == 'turn':
            cmd.angular.z = self.turn_speed
        else:
            self.finished = True             # 'done': tell main() to stop the loop

        self.publisher.publish(cmd)

    def switch_to(self, new_phase):
        self.get_logger().info(f'{self.phase} -> {new_phase} (sides done: {self.sides_done})')
        self.phase = new_phase
        self.phase_start = self.now()


def main(args=None):
    # NO: let us handle Ctrl+C ourselves, so ROS is still running when we send the stop command.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SquareDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)   # handle the next timer tick
    except KeyboardInterrupt:
        pass
    node.publisher.publish(Twist())          # last message: all zeros = stop
    node.get_logger().info('Sent stop command, exiting')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()