import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
from example_interfaces.msg import Int32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rover_msgs.msg import GenericPanel
from rover_msgs.msg import ArmPanel
from rover_msgs.msg import ArmCommand
from geometry_msgs.msg import TwistStamped



def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='arm_control',
                    namespace='',
                    executable='cbs_arm_interface',
                    name='cbs_arm_interface',
                ),
                # Launch tests 1.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )



# Active tests

# Control Base Station Input -> arm command output test
# Injects inputs that would come from the CBS
# Asserts on arm command topics
CBS_INTERFACE_TEST_TIME_SECONDS = 1
CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS = 0.05 # Node under test has a qos depth of 1 (can only process one msg per sub at a time), so rapid firing publishers will cause undef behaviour.
class CBSInterfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('cbs_interface_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_publishes_pose(self, proc_output):
        """Check whether pose messages published"""
        qos = QoSProfile(depth=10) # using a higher qos just to facilitate slower spin times
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        arm_cmd_msgs = [] # raw arm msgs
        ik_twist_msgs = [] # ik messages

        ik_sub = self.node.create_subscription(
            TwistStamped, '/arm_moveit_control/delta_twist_cmds',
            lambda msg: ik_twist_msgs.append(msg), qos)
        arm_sub = self.node.create_subscription(
            ArmCommand, '/arm/command',
            lambda msg: arm_cmd_msgs.append(msg), qos)
        
        arm_panel_pub = self.node.create_publisher(ArmPanel, '/cbs/arm_panel', qos)
        left_panel_a_pub = self.node.create_publisher(GenericPanel, '/cbs/left_panel_a', qos)
        try:
            
            # Inject an arm panel message
            injection = ArmPanel()
            injection.left.x = 50
            injection.left.y = 50
            injection.left.z = 50
            injection.right.x = 50
            injection.right.y = 50
            injection.right.z = 50
            injection.right.button = 0
            injection.left.button = 0
            arm_panel_pub.publish(injection) # injection 0, should be all 0s
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS) # needed to let the node under test respond
            injection.right.button = 1
            arm_panel_pub.publish(injection) # injection 1, end effector should move
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS)
            injection.right.button = 0
            injection.left.button = 1
            arm_panel_pub.publish(injection) # injection 2, end effector should move in other direction
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS)
            injection.left.button = 0
            injection.left.x = 80
            arm_panel_pub.publish(injection) # injection 3, axis 1 should move positive
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS)    
            injection.left.x = 23
            arm_panel_pub.publish(injection) # injection 4, axis 1 should move negative
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS)    

            end_time = time.time() + CBS_INTERFACE_TEST_TIME_SECONDS
            while time.time() < end_time:
                # spin to get subscriber callback executed
                rclpy.spin_once(self.node, timeout_sec=0.1)
            # There should have been 1 message received
            assert len(arm_cmd_msgs) > 0
            assert len(ik_twist_msgs) == 0 # control pipeline should not be sending any twist messages yet
            for value in arm_cmd_msgs[0].velocities:
                assert(value == 0)
            assert(arm_cmd_msgs[0].end_effector == 0.0)
            assert(arm_cmd_msgs[0].cmd_type == ord('V'))
            assert(arm_cmd_msgs[1].end_effector > 0.0)
            assert(arm_cmd_msgs[2].end_effector < 0.0)
            assert(arm_cmd_msgs[3].velocities[0] > 0.0)
            assert(arm_cmd_msgs[4].velocities[0] < 0.0)

            num_direct_commands = len(arm_cmd_msgs)
            left_panel_injection = GenericPanel()
            left_panel_injection.switches.append(False) # Inject IK switch setting
            left_panel_injection.switches[0] = True # Inject IK switch setting
            left_panel_a_pub.publish(left_panel_injection)
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS)
            arm_panel_pub.publish(injection)
            time.sleep(CBS_INTERFACE_TIME_NEEDED_TO_RESPOND_SECONDS)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            assert len(arm_cmd_msgs) == num_direct_commands # No new arm cmd messages should be published
            assert len(ik_twist_msgs) == 1
            assert(ik_twist_msgs[0].twist.linear.x != 0)



        finally:
            self.node.destroy_subscription(arm_sub)
            self.node.destroy_subscription(ik_sub)
            self.node.destroy_publisher(arm_panel_pub)
            self.node.destroy_publisher(left_panel_a_pub)

