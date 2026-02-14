import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
from example_interfaces.msg import Int32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='integration_test',
                    namespace='',
                    executable='example_multiply_by_two_node',
                    name='multiply_by_two_node',
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )

# Active tests
class TestExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_example')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_pose(self, proc_output):
        """Check whether pose messages published"""
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        msgs_rx = []
        sub = self.node.create_subscription(
            Int32, '/integration/test_int/output',
            lambda msg: msgs_rx.append(msg), qos)
        pub = self.node.create_publisher(Int32, '/integration/test_int/input', qos)
        try:
            # Listen to the pose topic for 10 s
            injection = Int32()
            injection.data = 5
            pub.publish(injection)
            end_time = time.time() + 1
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=1)
            # There should have been 1 message received
            assert len(msgs_rx) > 0
            assert msgs_rx[0].data == 10
        finally:
            self.node.destroy_subscription(sub)

    def test_logs_spawning(self, proc_output):
        """Check whether logging properly"""
        proc_output.assertWaitFor(
            "Example Log message: Node has started",
            timeout=5, stream='stderr')
