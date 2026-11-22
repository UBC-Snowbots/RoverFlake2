import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose


class TurtleTfBroadcaster(Node):
    """Turns the turtle's pose into a TF transform: where frame 'turtle1' is inside frame 'world'."""

    def __init__(self):
        super().__init__('turtle_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)                     # publishes on /tf
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

    def on_pose(self, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()   # when this was true
        t.header.frame_id = 'world'                        # parent: fixed reference (window's bottom-left corner)
        t.child_frame_id = 'turtle1'                       # child: the frame that rides on the turtle

        # Offset: where the turtle's origin is, measured in the world frame.
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = 0.0

        # Rotation: which way the turtle faces. A turn of theta about the vertical axis,
        # written as a quaternion (the 4-number rotation format ROS uses).
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(pose.theta / 2)
        t.transform.rotation.w = math.cos(pose.theta / 2)

        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTfBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass    # Ctrl+C: just exit


if __name__ == '__main__':
    main()