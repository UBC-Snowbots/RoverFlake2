"""Slider sim: robot_state_publisher + joint_state_publisher_gui + RViz.

Moves the URDF from GUI sliders and publishes /joint_states, which is what you
read to get a pose's joint angles in radians:

    ros2 topic echo /joint_states

The URDF must be passed as a parameter *value*, not on the command line --
`--ros-args -p robot_description:="$(cat ...)"` fails because rcl's argument
parser rejects the newlines in the XML. Reading the file here sidesteps that.

Runs either from the source tree or from the installed share directory:

    ros2 launch src/dev_arm_description_v2/launch/display.launch.py   # no build
    ros2 launch dev_arm_description_v2 display.launch.py              # installed
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_urdf():
    # launch/ and urdf/ are siblings in both the source tree and share/.
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), "urdf", "dev_arm.urdf")


def _launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf").perform(context)
    if not os.path.isfile(urdf_path):
        raise RuntimeError(f"URDF not found: {urdf_path}")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rviz_config = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "display.rviz")

    rviz_args = ["-d", rviz_config] if os.path.isfile(rviz_config) else []

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        # One slider per non-fixed joint; also the easiest way to dial in a
        # specific pose deliberately rather than by dragging.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf",
            default_value=_default_urdf(),
            description="Absolute path to the URDF to display",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
