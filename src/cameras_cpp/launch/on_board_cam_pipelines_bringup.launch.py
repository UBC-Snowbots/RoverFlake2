import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    cam_pipelines_yaml = os.path.join(
        get_package_share_directory('cameras_cpp'),
        'config',
        'camera_pipelines.yaml'
    )
   
    science_cam_pipelines = Node(
        package='cameras_cpp',
        executable='camera_pipeline_node',
        name='science_cam_pipeline_node',
        parameters=[cam_pipelines_yaml]
    )

    return LaunchDescription([
        science_cam_pipelines
    ])