from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_pullup = Node(
        package='rover_vision',
        executable='camera_pub_node',
        name='camera_pub_node',
        output='screen'
    )
    camera_broadcaster = Node(
        package='cameras_cpp',  
        executable='dyn_cam_pipeline_node',
        name='dyn_cam_pipeline_node',
        output='screen'
    )
    broadcast_encoder_0 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_encoder_0',
        remappings=[
            ('in/image_raw', '/broadcast/cam_0/image_raw'),
            ('out/ffmpeg', '/broadcast/cam_0/h265'),
        ],
        parameters=[
            {'ffmpeg_image_transport.encoding': 'libx264',
             'ffmpeg_image_transport.profile': 'high',
             'ffmpeg_image_transport.preset': 'ultrafast',
             'ffmpeg_image_transport.tune': 'zerolatency',
             'ffmpeg_image_transport.gop_size': 30,
             'ffmpeg_image_transport.delay': "20ms",
             'ffmpeg_image_transport.frame_rate': 30,
             'ffmpeg_image_transport.qmax': 80,
             'ffmpeg_image_transport.crf': 28
             }       
        ],
        arguments=['raw', 'in:=/broadcast/cam_0/image_raw', 'ffmpeg', 'out:=/broadcast/cam_0/h265']
    )
    
    broadcast_encoder_1 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_encoder_1',
        remappings=[
            ('in/image_raw', '/broadcast/cam_1/image_raw'),
            ('out/ffmpeg', '/broadcast/cam_1/h265'),
        ],
        parameters=[
            {'ffmpeg_image_transport.encoding': 'libx264',
             'ffmpeg_image_transport.profile': 'high',
             'ffmpeg_image_transport.preset': 'ultrafast',
             'ffmpeg_image_transport.tune': 'zerolatency',
             'ffmpeg_image_transport.gop_size': 30,
             'ffmpeg_image_transport.delay': "20ms",
             'ffmpeg_image_transport.frame_rate': 30,
             'ffmpeg_image_transport.qmax': 80,
             'ffmpeg_image_transport.crf': 28
             }       
        ],
        arguments=['raw', 'in:=/broadcast/cam_1/image_raw', 'ffmpeg', 'out:=/broadcast/cam_1/h265']
    )
    
    broadcast_encoder_2 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_encoder_2',
        remappings=[
            ('in/image_raw', '/broadcast/cam_2/image_raw'),
            ('out/ffmpeg', '/broadcast/cam_2/h265'),
        ],
        parameters=[
            {'ffmpeg_image_transport.encoding': 'libx264',
             'ffmpeg_image_transport.profile': 'high',
             'ffmpeg_image_transport.preset': 'ultrafast',
             'ffmpeg_image_transport.tune': 'zerolatency',
             'ffmpeg_image_transport.gop_size': 30,
             'ffmpeg_image_transport.delay': "20ms",
             'ffmpeg_image_transport.frame_rate': 30,
             'ffmpeg_image_transport.qmax': 80,
             'ffmpeg_image_transport.crf': 28
             }       
        ],
        arguments=['raw', 'in:=/broadcast/cam_2/image_raw', 'ffmpeg', 'out:=/broadcast/cam_2/h265']
    )
    
    broadcast_encoder_3 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_encoder_3',
        remappings=[
            ('in/image_raw', '/broadcast/cam_3/image_raw'),
            ('out/ffmpeg', '/broadcast/cam_3/h265'),
        ],
        parameters=[
            {'ffmpeg_image_transport.encoding': 'libx264',
             'ffmpeg_image_transport.profile': 'high',
             'ffmpeg_image_transport.preset': 'ultrafast',
             'ffmpeg_image_transport.tune': 'zerolatency',
             'ffmpeg_image_transport.gop_size': 30,
             'ffmpeg_image_transport.delay': "20ms",
             'ffmpeg_image_transport.frame_rate': 30,
             'ffmpeg_image_transport.qmax': 80,
             'ffmpeg_image_transport.crf': 28
             }       
        ],
        arguments=['raw', 'in:=/broadcast/cam_3/image_raw', 'ffmpeg', 'out:=/broadcast/cam_3/h265']
    )

    return LaunchDescription([
        camera_pullup,
        camera_broadcaster,
        broadcast_encoder_0,
        broadcast_encoder_1,
        broadcast_encoder_2,
        broadcast_encoder_3
    ])