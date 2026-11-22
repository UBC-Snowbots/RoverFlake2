import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# How the IMU board sits on the rover (worked out in the IMU lesson)
MOUNT = {'mount_roll_deg': 180.0, 'mount_pitch_deg': 0.0, 'mount_yaw_deg': 90.0}
RVIZ_CONFIG = os.path.expanduser('~/.rviz2/imu.rviz')


def make_nodes(context):
    model = LaunchConfiguration('model').perform(context)
    use_mag = LaunchConfiguration('use_mag').perform(context) == 'true'
    motion = LaunchConfiguration('motion').perform(context) == 'true'
    flat = LaunchConfiguration('flat').perform(context) == 'true'
    scale = float(LaunchConfiguration('scale').perform(context))

    if model == 'leo':
        path = os.path.join(get_package_share_directory('leo_description'), 'urdf', 'leo.urdf.xacro')
        robot_description = xacro.process_file(path).toxml()
        rover_frame, imu_frame, height = 'base_footprint', 'imu_frame', 0.0
        # Leo is a differential drive: two wheels per side (from its URDF)
        wheels = {'left_wheel_joints': 'wheel_FL_joint,wheel_RL_joint',
                  'right_wheel_joints': 'wheel_FR_joint,wheel_RR_joint',
                  'wheel_radius': 0.0625, 'wheel_separation': 0.358}
    else:
        path = os.path.join(get_package_share_directory('rover_description'), 'urdf', 'Chassis_jan25.SLDASM.urdf')
        with open(path, 'r') as f:
            robot_description = f.read()
        rover_frame, imu_frame, height = 'base_link', 'imu_link', 0.3
        wheels = {}   # the team model's wheels aren't separate joints, so they can't spin

    rviz_args = ['-d', RVIZ_CONFIG] if os.path.exists(RVIZ_CONFIG) else []
    # Our node publishes the wheel angles itself when driving Leo; otherwise hold every joint at 0
    spin_wheels = motion and bool(wheels)

    return [
        # 1. IMU driver: USB -> /imu/data_raw and /imu/mag
        ComposableNodeContainer(
            name='phidget_container', namespace='', package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='phidgets_spatial', plugin='phidgets::SpatialRosI', name='phidgets_spatial',
                parameters=[{'frame_id': imu_frame, 'data_interval_ms': 8}])]),

        # 2. Madgwick filter: raw readings -> orientation on /imu/data
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
             output='screen',
             parameters=[{'use_mag': use_mag, 'world_frame': 'enu', 'publish_tf': False,
                          'publish_debug_topics': True}]),

        # 3. Rover model for RViz
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        *([] if spin_wheels else [Node(package='joint_state_publisher', executable='joint_state_publisher')]),

        # 4. Our node: places the rover in the world (tilt only, or tilt + driving)
        Node(package='learn_drive', executable='imu_rover_motion' if motion else 'imu_to_rover_tf',
             output='screen',
             parameters=[{'rover_frame': rover_frame, 'height': height, **MOUNT,
                          **({'flat_ground': flat, 'motion_scale': scale, **wheels} if motion else {})}]),

        # 5. RViz
        Node(package='rviz2', executable='rviz2', arguments=rviz_args),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='team', description='team or leo'),
        DeclareLaunchArgument('use_mag', default_value='false',
                              description='use the magnetometer (compass) for heading. Off: the laptop\'s '
                                          'magnets bend it, and gyro-only turning follows your hand 1:1'),
        DeclareLaunchArgument('motion', default_value='true',
                              description='true: IMU pushes drive the rover. false: tilt only'),
        DeclareLaunchArgument('scale', default_value='3.0',
                              description='rover moves this many times as far as your hand (1.0 = life size)'),
        DeclareLaunchArgument('flat', default_value='true',
                              description='true: rover stays flat on the ground (only turns). false: shows tilt too'),
        OpaqueFunction(function=make_nodes),
    ])