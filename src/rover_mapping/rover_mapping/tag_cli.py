"""Frictionless field CLI for tagging points and segments.

    ros2 run rover_mapping tag site "rock outcrop"
    ros2 run rover_mapping tag sample
    ros2 run rover_mapping tag obstacle "boulder field" -n "east of track"
    ros2 run rover_mapping tag site "given target" --at 51.453361 -112.722667
    ros2 run rover_mapping tag seg start route_to_site_2
    ros2 run rover_mapping tag seg end
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from rover_mapping_interfaces.srv import Segment, TagPoint

from rover_mapping.mission_io import CATEGORIES

TIMEOUT = 5.0


def _call(node: Node, srv_type, srv_name: str, request):
    client = node.create_client(srv_type, srv_name)
    if not client.wait_for_service(timeout_sec=TIMEOUT):
        print(f'ERROR: {srv_name} unavailable — is mission_manager running?',
              file=sys.stderr)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
    if not future.done():
        print(f'ERROR: {srv_name} timed out', file=sys.stderr)
        return None
    return future.result()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog='tag', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('what',
                        help=f'one of {", ".join(CATEGORIES)}, or "seg"')
    parser.add_argument('label', nargs='?', default='',
                        help='waypoint label / segment start|end')
    parser.add_argument('name', nargs='?', default='',
                        help='segment name (with "seg start")')
    parser.add_argument('-n', '--notes', default='', help='free-form notes')
    parser.add_argument('--at', nargs=2, type=float, metavar=('LAT', 'LON'),
                        help='tag this coordinate instead of the current fix')
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('tag_cli')
    try:
        if args.what == 'seg':
            if args.label not in ('start', 'end'):
                parser.error('usage: tag seg start <name> | tag seg end')
            if args.at:
                parser.error('--at applies to waypoints, not segments')
            req = Segment.Request()
            req.name = args.name
            res = _call(node, Segment, f'/segment_{args.label}', req)
        else:
            if args.what not in CATEGORIES:
                parser.error(f'"{args.what}" is not one of '
                             f'{", ".join(CATEGORIES)} (or "seg")')
            req = TagPoint.Request()
            req.category = args.what
            req.label = args.label
            req.notes = args.notes
            if args.at:
                req.use_manual = True
                req.manual_lat, req.manual_lon = args.at
            res = _call(node, TagPoint, '/tag_point', req)

        if res is None:
            return 2
        if not res.ok:
            print(f'REJECTED: {res.message}', file=sys.stderr)
            return 1
        if hasattr(res, 'lat'):
            print(f'{res.message}  ({res.lat:.6f}, {res.lon:.6f})')
        else:
            print(res.message)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
