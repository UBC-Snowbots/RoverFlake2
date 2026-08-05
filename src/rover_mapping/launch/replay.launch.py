"""Replay a recorded mission (or one leg of it) over the offline imagery.

    ros2 launch rover_mapping replay.launch.py \
        mission_dir:=missions/2026-08-05_run1 site:=drumheller rate:=4.0
    ros2 launch rover_mapping replay.launch.py \
        mission_dir:=... segment:=route_to_site_2

The local-XY origin is re-anchored exactly where the mission recorded it
(origin.yaml), so offset replays still line up with the imagery and the
recorded /waypoints markers. Humble's rosbag2 has no --playback-duration:
for segment replay the end time is printed — Ctrl-C when reached.
"""
from __future__ import annotations

import os

import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from rover_mapping.mission_io import load_yaml_list, segment_offsets


def _segment_offset(mission_dir: str, segment: str) -> float:
    """Start offset (s) of a named segment relative to the bag start."""
    segments = load_yaml_list(os.path.join(mission_dir, 'segments.yaml'))
    match = next((s for s in segments if s['name'] == segment), None)
    if match is None:
        have = ', '.join(s['name'] for s in segments) or '(none)'
        raise RuntimeError(f'no segment "{segment}" — have: {have}')
    with open(os.path.join(mission_dir, 'rosbag2', 'metadata.yaml')) as f:
        meta = yaml.safe_load(f)
    bag_start = meta['rosbag2_bagfile_information'][
        'starting_time']['nanoseconds_since_epoch'] * 1e-9
    offset, duration = segment_offsets(match, bag_start)
    print(f'[replay] segment "{segment}": starts {offset:.1f}s into the '
          f'bag, lasts {duration:.1f}s — Ctrl-C at the end' if duration
          else f'[replay] segment "{segment}" (still open): '
          f'starts {offset:.1f}s in')
    return offset


def replay_stack(context, *_args, **_kw):
    here = os.path.dirname(os.path.realpath(__file__))
    mission_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('mission_dir').perform(context)))
    rate = LaunchConfiguration('rate').perform(context)
    segment = LaunchConfiguration('segment').perform(context)
    offset = float(LaunchConfiguration('start_offset').perform(context))
    if segment:
        offset = _segment_offset(mission_dir, segment)

    # Re-anchor at the recorded origin so everything lines up.
    mapviz_args = {
        'site': LaunchConfiguration('site'),
        'port': LaunchConfiguration('port'),
        'max_zoom': LaunchConfiguration('max_zoom'),
        'fix_topic': LaunchConfiguration('fix_topic'),
        'use_sim_time': 'true',
        'run_manager': 'false',   # never record during a replay
    }
    origin_yaml = os.path.join(mission_dir, 'origin.yaml')
    if os.path.exists(origin_yaml):
        with open(origin_yaml) as f:
            origin = yaml.safe_load(f)[0]
        mapviz_args.update(origin='static',
                           static_lat=str(origin['lat']),
                           static_lon=str(origin['lon']),
                           static_alt=str(origin.get('alt', 0.0)))

    play_cmd = ['ros2', 'bag', 'play',
                os.path.join(mission_dir, 'rosbag2'),
                '--clock', '--rate', rate]
    if offset > 0.0:
        play_cmd += ['--start-offset', str(offset)]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(here, 'mapviz_offline.launch.py')),
            launch_arguments=mapviz_args.items()),
        ExecuteProcess(cmd=play_cmd, name='bag_play', output='screen'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('mission_dir'),
        DeclareLaunchArgument('site', default_value='ubc_test'),
        DeclareLaunchArgument('port', default_value='8000'),
        DeclareLaunchArgument('max_zoom', default_value='19'),
        DeclareLaunchArgument('fix_topic', default_value='/gnss_fix'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_offset', default_value='0'),
        DeclareLaunchArgument('segment', default_value=''),
        OpaqueFunction(function=replay_stack),
    ])
