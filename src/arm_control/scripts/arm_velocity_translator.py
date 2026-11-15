#!/usr/bin/env python3
# Turns ros2_control velocity commands (JointState, rad/s) into ArmCommand V (deg/s)
# for the old stepper arm driver (stepper_arm_driver). Used by real_arm_rviz.launch.py.
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from rover_msgs.msg import ArmCommand

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
# Copy of ArmConstants::axis_dirs (rover_arm_common/arm_constants.h), the ones the driver uses
AXIS_DIRS = [1, 1, 1, 1, 1, 1]
# Same per-axis caps the driver uses when returning after homing (RETURN_MAX_VEL_DPS), deg/s
MAX_DEG_S = [50.0, 23.0, 30.0, 90.0, 55.0, 90.0]
# Firmware treats anything slower than this as stopped (ARM_DEG_VELOCITY_RESOLUTION)
DEADBAND_DEG_S = 0.05
# Stop the arm if commands stop arriving for this long
TIMEOUT_S = 0.25


class ArmVelocityTranslator(Node):
    def __init__(self):
        super().__init__("arm_velocity_translator")
        self.pub = self.create_publisher(ArmCommand, "/arm/command", 10)
        self.create_subscription(JointState, "/arm/joint_commands", self.on_command, 10)
        self.create_timer(0.05, self.watchdog)
        self.last_command_time = time.monotonic()
        self.moving = False

    def on_command(self, msg):
        velocity_of = dict(zip(msg.name, msg.velocity))
        if not all(joint in velocity_of for joint in JOINTS):
            return
        self.last_command_time = time.monotonic()

        deg_s = []
        for i, joint in enumerate(JOINTS):
            v = math.degrees(velocity_of[joint]) * AXIS_DIRS[i]
            v = max(-MAX_DEG_S[i], min(MAX_DEG_S[i], v))
            if abs(v) < DEADBAND_DEG_S:
                v = 0.0
            deg_s.append(v)

        any_motion = any(v != 0.0 for v in deg_s)
        if any_motion or self.moving:
            self.send_velocities(deg_s)
        self.moving = any_motion

    def watchdog(self):
        if self.moving and time.monotonic() - self.last_command_time > TIMEOUT_S:
            self.get_logger().warn("No commands for %.2f s, stopping the arm" % TIMEOUT_S)
            self.send_velocities([0.0] * len(JOINTS))
            self.moving = False

    def send_velocities(self, deg_s):
        out = ArmCommand()
        out.cmd_type = "V"
        out.velocities = deg_s + [0.0]
        out.end_effector = 0.0
        self.pub.publish(out)


def main():
    # Handle Ctrl+C ourselves, so ROS is still running when we send the final stop
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ArmVelocityTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_velocities([0.0] * len(JOINTS))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
