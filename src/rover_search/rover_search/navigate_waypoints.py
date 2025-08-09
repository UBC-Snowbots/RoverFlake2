#!/usr/bin/env python3

"""
    This node is designed to navigate a robot to a series of predefined waypoints using the Nav2 navigation system. 
    It initializes the navigation system, sets up a list of waypoints, and sends them to the navigator to execute.
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped
import time

class WaypointNav(Node):
    def __init__(self):
        # Instantiates parent ROS node
        super().__init__('waypoint_nav')
        # Instantiates the navigation client to interact with nav2
        self.navigator = BasicNavigator() 
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active!")

        # Define a list of waypoints
        self.waypoints = []
        self.setup_waypoints()
        self.send_waypoints()

    def setup_waypoints(self):
        # Add your waypoint poses here (in map frame)
        # TODO This is hard coded. change to work with topic/service receiving. 
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.header.stamp = self.get_clock().now().to_msg()
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 0.0
        pose1.pose.orientation.w = 1.0

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.header.stamp = self.get_clock().now().to_msg()
        pose2.pose.position.x = 1.0
        pose2.pose.position.y = 1.0
        pose2.pose.orientation.w = 1.0

        self.waypoints.append(pose1)
        self.waypoints.append(pose2)

    def send_waypoints(self):
        self.get_logger().info("Navigating through waypoints...")
        self.navigator.followWaypoints(self.waypoints)

        # Sends all waypoints in a batch to Nav2 
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Current waypoint: {feedback.current_waypoint + 1}/{len(self.waypoints)}')
            time.sleep(1)

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Waypoints complete!')
        else:
            self.get_logger().warn('Navigation failed or was canceled.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()