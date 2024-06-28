from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='middle_screen',
        # parameters=[{'config': '/dev/input/eventX'}]
    )

    rover_virtual_twin = Node(
        package='rover_hmi',
        executable='middle_rover_viz_node',
        name='rover_virtual_twin',
    )

    return LaunchDescription([
        rviz_node,
        rover_virtual_twin,
    ])
