# Importing necessary modules from the ROS2 package
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LaunchDescription is a class that stores the launch configuration
    launch_description = LaunchDescription()

    # Creating a node using the Node class from launch_ros.actions
    # The Node class is used to start a ROS node from a launch file
    # Parameters:
    # - package: the package where the node executable is located
    # - executable: the name of the node executable
    # - name: (optional) a custom name for the node
    # - output: where to output the node's log messages, e.g., 'screen'
    arm_hardware_node = Node(
        package='arm_hardware_interface',
        executable='arm_serial_driver',
        name='arm_serial_driver',
        output='screen'
    )
    arm_joy = Node(
        package='arm_control',
        executable='joy_arm_control',
        name='arm_joystick_controller',
        output='screen'
    )
    arm_gui = Node(
    package='rover_qt_gui',
    executable='rds_qt',
    name='arm_gui',
    output='screen'
    )


    # Adding the node to the launch description
    # This step is required for every node you want to launch
    launch_description.add_action(arm_hardware_node)
    launch_description.add_action(arm_joy)
    # launch_description.add_action(arm_gui)

    # The function returns the LaunchDescription object, which will be executed by the ROS2 launch system
    return launch_description
