"""The live mapping stack: tile server + origin + mapviz + mission_manager.

    ros2 launch rover_mapping mapviz_offline.launch.py site:=drumheller

Leave it running all day; missions are started/stopped against it via the
/mission/start and /mission/stop services (`rover start` / `rover stop`).

args:
  site       tile-set name under imagery_pipeline/imagery/<site>/
  origin     center -> local-XY origin anchored at the imagery center
             (default: valid transform before any GPS, no mapviz warnings)
             auto   -> anchored on the first GPS fix instead
  fix_topic  NavSatFix topic (default /gnss_fix — rover_gnss nmea_reader)
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_mapping.launch_helpers import (imagery_pipeline_dir,
                                          render_site_config, tiles_center)


def mapviz_stack(context, *_args, **_kw):
    site = LaunchConfiguration('site').perform(context)
    port = LaunchConfiguration('port').perform(context)
    origin = LaunchConfiguration('origin').perform(context)
    fix_topic = LaunchConfiguration('fix_topic').perform(context)
    max_zoom = int(LaunchConfiguration('max_zoom').perform(context))
    use_sim_time = (LaunchConfiguration('use_sim_time').perform(context)
                    in ('true', 'True', '1'))
    run_manager = (LaunchConfiguration('run_manager').perform(context)
                   in ('true', 'True', '1'))

    cfg = render_site_config(site, int(port), max_zoom=max_zoom,
                             fix_topic=fix_topic)
    serve = os.path.join(imagery_pipeline_dir(), 'scripts', 'serve_tiles.sh')

    if origin == 'auto':
        origin_params = {'local_xy_origin': 'auto'}
    else:
        if origin == 'static':      # explicit coords (used by replay)
            lat = float(LaunchConfiguration('static_lat').perform(context))
            lon = float(LaunchConfiguration('static_lon').perform(context))
            alt = float(LaunchConfiguration('static_alt').perform(context))
        else:                       # 'center': middle of the tile set
            lat, lon = tiles_center(os.path.join(
                imagery_pipeline_dir(), 'imagery', site, 'tiles'))
            alt = 0.0
        # a *string* containing YAML — that is how ROS 2 initialize_origin
        # takes its static origin list
        origin_params = {
            'local_xy_origin': 'anchor',
            'local_xy_origins':
                f'[{{name: anchor, latitude: {lat:.6f}, '
                f'longitude: {lon:.6f}, altitude: {alt}, heading: 0.0}}]',
        }

    actions = [
        # Static tile server (plain process, not a ROS node).
        ExecuteProcess(cmd=[serve, site, port], name='tile_server',
                       output='screen'),
        Node(package='swri_transform_util', executable='initialize_origin.py',
             name='initialize_origin', output='screen',
             parameters=[{
                 'local_xy_frame': 'map',
                 'local_xy_navsatfix_topic': fix_topic,
                 'use_sim_time': use_sim_time,
                 **origin_params,
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_anchor',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin'],
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='mapviz', executable='mapviz', name='mapviz',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ]
    if run_manager:
        actions.append(Node(
            package='rover_mapping', executable='mission_manager',
            name='mission_manager', output='screen',
            parameters=[{'site': site, 'fix_topic': fix_topic}]))
    print(f'[mapviz_offline] site={site} config={cfg} origin={origin}')
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='ubc_test'),
        DeclareLaunchArgument('port', default_value='8000'),
        DeclareLaunchArgument('max_zoom', default_value='19'),
        DeclareLaunchArgument('origin', default_value='center'),
        DeclareLaunchArgument('static_lat', default_value='0.0'),
        DeclareLaunchArgument('static_lon', default_value='0.0'),
        DeclareLaunchArgument('static_alt', default_value='0.0'),
        DeclareLaunchArgument('fix_topic', default_value='/gnss_fix'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('run_manager', default_value='true'),
        OpaqueFunction(function=mapviz_stack),
    ])
