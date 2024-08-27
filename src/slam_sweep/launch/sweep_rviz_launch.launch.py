from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sweep_ros_share_dir = FindPackageShare(package='slam_sweep').find('slam_sweep')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('slam_sweep'), 'rviz', 'sweep_laser_scan.rviz']
    )

    return LaunchDescription([
        Node(
            package='slam_sweep',
            executable='sweep_node',
            name='sweep_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'rotation_speed': 5,
                'sample_rate': 500,  # Can change rate 
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
