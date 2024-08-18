import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file

# these functions should just work
def xacro_to_urdf(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    robot_description = process_file(absolute_file_path).toxml()
    return robot_description

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Path to the URDF file
    # pkg_share = get_package_share_directory('rover_urdf')
    urdf_file_path = (
        get_package_share_directory("rover_urdf") + "/urdf/chassis_urdf_24.urdf"
    )
    # default_model_path = os.path.join(pkg_share, 'urdf/chassis_urdf_24.urdf')
    full_urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro_to_urdf("rover_urdf", urdf_file_path) }]
    )
    gazebo_spawn_args = '{name: \"chassis_24\", xml: \"' + urdf_file_path + '"}'

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'model', 
        #     default_value=default_model_path,
        #     description='Absolute path to robot urdf file'
        # ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # Start Gazebo server
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Launch robot_state_publisher to publish robot state
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     arguments=[LaunchConfiguration('model')]
        # ),
        full_urdf_publisher,

        # Spawn the robot in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
                 '-entity', 'chassis', 
                 '-file', gazebo_spawn_args],
            output='screen'
        )
    ])
