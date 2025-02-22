from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy =  Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/eventX'}]
    )
       
    hmi_node = Node(
        package='rover_hmi',
        executable='arm_hmi_node',
        name='arm_hmi_node'
    )

    sim_helper_node = Node(
        package='arm_control',
        executable='sim_helper',
        name='sim_helper_node'
    )

    return LaunchDescription([
        hmi_node,
        sim_helper_node
    ])
