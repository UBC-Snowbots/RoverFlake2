"""Nav2 driving our rover in simulation: no map, obstacles seen by a lidar.

Runs:
  1. robot_state_publisher  - the rover model (Leo for now) for RViz and TF
  2. fake_rover             - pretend drive + odometry (stands in for the real rover's drive code)
  3. the lidar's position, and (fake mode only) the pretend lidar in a pretend rock field
  4. Nav2: planner, controller, behaviors, BT navigator (+ lifecycle manager to start them)
  5. RViz                   - "2D Goal Pose" sends a goal

Two lidar modes:
  lidar:=fake  (default)  pretend lidar riding on the rover, pretend rocks
                          ros2 launch ~/learn_ws/src/learn_drive/launch/rover_nav_sim.launch.py
  lidar:=real             your real lidar, sitting still on the floor where the rover starts.
                          Start the lidar's driver first (frame_id laser), then:
                          ros2 launch ~/learn_ws/src/learn_drive/launch/rover_nav_sim.launch.py lidar:=real
                          Optional: lidar_yaw:=180 if the room appears rotated the wrong way round.
  person:=true            (fake mode) an open field with a person in it, for testing lidar_follow.py.
                          "Publish Point" in RViz makes the person walk to the clicked spot.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.join(HERE, '..', 'config')
NAV2_PARAMS = os.path.join(CONFIG_DIR, 'rover_nav2.yaml')
RVIZ_CONFIG = os.path.join(CONFIG_DIR, 'rover_nav.rviz')

# The Nav2 servers we use, in the order the lifecycle manager starts them
NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

# Person mode's field (x, y, radius): mostly open, a few rocks to the sides and a wall-like
# row of rocks further out, so the follower has non-people to ignore
FOLLOW_ROCKS = [
    -2.0, 3.0, 0.4, -2.0, -3.0, 0.4, 6.0, 3.5, 0.5, 6.0, -3.5, 0.5,
    9.0, -4.0, 0.5, 9.0, -3.2, 0.5, 9.0, -2.4, 0.5, 9.0, -1.6, 0.5, 9.0, -0.8, 0.5, 9.0, 0.0, 0.5,
    9.0, 0.8, 0.5, 9.0, 1.6, 0.5, 9.0, 2.4, 0.5, 9.0, 3.2, 0.5, 9.0, 4.0, 0.5,
]


def make_nodes(context):
    lidar = LaunchConfiguration('lidar').perform(context)
    person = LaunchConfiguration('person').perform(context).lower() == 'true'
    lidar_yaw_rad = str(float(LaunchConfiguration('lidar_yaw').perform(context)) * 3.141592653589793 / 180.0)
    leo_urdf = os.path.join(get_package_share_directory('leo_description'), 'urdf', 'leo.urdf.xacro')
    robot_description = xacro.process_file(leo_urdf).toxml()

    if lidar == 'real':
        lidar_nodes = [
            # The real lidar sits still on the floor at the rover's start point, 10 cm up.
            # So its frame hangs off the world (odom), not off the rover.
            Node(package='tf2_ros', executable='static_transform_publisher', name='lidar_in_room',
                 arguments=['--x', '0', '--y', '0', '--z', '0.1', '--yaw', lidar_yaw_rad,
                            '--frame-id', 'odom', '--child-frame-id', 'laser']),
        ]
    else:
        lidar_nodes = [
            # Pretend lidar, mounted 30 cm up, centred on the rover
            Node(package='tf2_ros', executable='static_transform_publisher', name='laser_mount',
                 arguments=['--x', '0', '--y', '0', '--z', '0.3',
                            '--frame-id', 'base_footprint', '--child-frame-id', 'laser']),
            Node(package='learn_drive', executable='fake_lidar', output='screen',
                 parameters=[{'person': True, 'person_x': 2.0, 'person_y': 0.0,
                              'dropout': 0.4, 'rocks': FOLLOW_ROCKS}] if person else []),
        ]

    return [
        # 1. Rover model
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # 2. Pretend rover base: obeys /cmd_vel, publishes /odom + TF odom -> base_footprint
        Node(package='learn_drive', executable='fake_rover', output='screen',
             parameters=[{'left_wheel_joints': 'wheel_FL_joint,wheel_RL_joint',
                          'right_wheel_joints': 'wheel_FR_joint,wheel_RR_joint',
                          'wheel_radius': 0.0625, 'wheel_separation': 0.358}]),

        # 3. Lidar
        *lidar_nodes,

        # 4. Nav2 servers (all read the same settings file)
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[NAV2_PARAMS]),
        Node(package='nav2_planner', executable='planner_server', output='screen',
             parameters=[NAV2_PARAMS]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen',
             parameters=[NAV2_PARAMS]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen',
             parameters=[NAV2_PARAMS]),
        # Starts the servers above in order, then watches that they stay alive
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': NAV2_NODES}]),

        # 5. RViz
        Node(package='rviz2', executable='rviz2', arguments=['-d', RVIZ_CONFIG]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='fake', description='fake or real'),
        DeclareLaunchArgument('lidar_yaw', default_value='0',
                              description='real mode: which way the lidar faces, in degrees'),
        DeclareLaunchArgument('person', default_value='false',
                              description='fake mode: open field with a walking person (for lidar_follow.py)'),
        OpaqueFunction(function=make_nodes),
    ])
