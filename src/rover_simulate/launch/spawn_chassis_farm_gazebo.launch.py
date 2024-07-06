import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import ParameterValue

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    raw_urdf_pkg_name = 'rover_urdf'
    file_subpath = 'urdf/chassis_urdf_24_gazebo.urdf'


    # Use xacro to process the file, even though its just a urdf file
    xacro_file = os.path.join(get_package_share_directory(raw_urdf_pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # robot_description_ros2_dont_like_raw = ParameterValue(robot_description_raw, value_type=str) # which parses the param as YAML instead of string

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': False}] # add other parameters here if required
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': os.path.join(get_package_share_directory('rover_simulate'), 'gazebo_worlds', 'basic.world')}.items(),
        )

    controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(get_package_share_directory('rover_simulate'), 'config', 'chassis_controller_params.yaml')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'chassis_24',
                    '-x', '0.0', '-y', '0.0', '-z', '3.0',
                    '-timeout', '100.0'],
        output='screen')
    

    control_node = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller', '--controller-manager', '/controller_manager'], 
        )
    
    joint_pubber = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        )
    







    # Run the node
    return LaunchDescription([
        gazebo,
        joint_pubber,
        node_robot_state_publisher,
        spawn_entity,
        controller_manager_node,
        control_node
    ])

# meow
