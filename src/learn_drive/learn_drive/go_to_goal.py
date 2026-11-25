import math
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw):
    """A position + heading on the map, in the format Nav2 expects."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'                       # the coordinates are map coordinates
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2)        # heading as a quaternion (Lesson 3)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def main():
    # Goal from the command line:  go_to_goal X Y [HEADING_DEGREES]
    if len(sys.argv) < 3:
        print('usage: ros2 run learn_drive go_to_goal X Y [HEADING_DEGREES]')
        return
    x, y = float(sys.argv[1]), float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    # NO: handle Ctrl+C ourselves, so ROS is still running when we tell Nav2 to cancel.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    navigator = BasicNavigator()

    # Wait until Nav2's servers are up. 'robot_localization' = "don't wait for AMCL",
    # the map-matching localizer this demo doesn't run (the simulator provides map->odom).
    navigator.waitUntilNav2Active(localizer='robot_localization')

    # Send the goal to Nav2's NavigateToPose action.
    if not navigator.goToPose(make_pose(navigator, x, y, yaw)):
        print('Nav2 rejected the goal')
        rclpy.shutdown()
        return

    # While Nav2 drives, print its feedback about once a second.
    ticks = 0
    try:
        while not navigator.isTaskComplete():          # waits ~0.1 s each call
            ticks += 1
            feedback = navigator.getFeedback()
            if feedback and ticks % 10 == 0:
                seconds = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
                print(f'{seconds:5.1f} s  distance left {feedback.distance_remaining:5.2f} m  '
                      f'recoveries {feedback.number_of_recoveries}')
    except KeyboardInterrupt:
        navigator.cancelTask()                          # Ctrl+C: tell Nav2 to stop driving
        while not navigator.isTaskComplete():           # wait for Nav2 to confirm
            pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal reached!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled')
    else:
        print('Goal failed (Nav2 could not get there)')

    rclpy.shutdown()


if __name__ == '__main__':
    main()