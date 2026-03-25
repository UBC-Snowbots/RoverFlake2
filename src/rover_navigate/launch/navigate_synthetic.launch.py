"""
Launch file for the path planner with synthetic LiDAR.

Launches:
  1. synthetic_lidar   — fake LaserScan data (/scan)
  2. simple_planner    — reactive obstacle avoidance (/cmd_vel)
  3. static_transform  — laser_link -> base_link TF

Usage:
  ros2 launch rover_navigate navigate_synthetic.launch.py

  # With custom parameters:
  ros2 launch rover_navigate navigate_synthetic.launch.py \
      linear_speed:=0.5 obstacle_threshold:=2.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        # --- Launch arguments ---
        DeclareLaunchArgument('linear_speed', default_value='0.3',
                              description='Forward speed in m/s'),
        DeclareLaunchArgument('obstacle_threshold', default_value='1.5',
                              description='Stop distance in meters'),
        DeclareLaunchArgument('room_width', default_value='10.0',
                              description='Synthetic room width in meters'),
        DeclareLaunchArgument('room_height', default_value='10.0',
                              description='Synthetic room height in meters'),

        # --- Static TF: laser_link -> base_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
        ),

        # --- Synthetic LiDAR ---
        Node(
            package='rover_navigate',
            executable='synthetic_lidar',
            name='synthetic_lidar',
            output='screen',
            parameters=[{
                'scan_rate': 5.5,
                'num_samples': 360,
                'range_min': 0.15,
                'range_max': 12.0,
                'room_width': LaunchConfiguration('room_width'),
                'room_height': LaunchConfiguration('room_height'),
                'robot_x': 5.0,
                'robot_y': 5.0,
                'noise_std': 0.02,
            }],
        ),

        # --- Simple planner ---
        Node(
            package='rover_navigate',
            executable='simple_planner',
            name='simple_planner',
            output='screen',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': 0.5,
                'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
                'side_threshold': 0.8,
                'front_arc_degrees': 60.0,
                'control_rate': 10.0,
            }],
        ),
    ])
