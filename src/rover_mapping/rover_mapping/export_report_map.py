"""Render the competition report map from a recorded mission.

    ros2 run rover_mapping export_report_map -- \
        --mission missions/2026-08-05_test1 \
        --imagery imagery_pipeline/imagery/ubc_test/mosaic.tif

Also runs as plain python (no ROS env needed beyond rosbag2_py).
Deps (pip): rasterio matplotlib pyproj pyyaml  — see requirements.txt.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Tuple

import yaml

# style per category: (color, matplotlib marker) — matches mapviz markers
CATEGORY_STYLE = {
    'start':    ('#21bf21', '*'),
    'site':     ('#e61a1a', 'o'),
    'sample':   ('#9426cc', 'X'),
    'obstacle': ('#ff8c00', '^'),
    'landmark': ('#2666f2', 's'),
}


def read_fixes(bag_dir: str, fix_topic: str) -> List[Tuple[float, float]]:
    """Read (lat, lon) of every NavSatFix on fix_topic from a rosbag2 dir."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import NavSatFix

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=''),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    fixes: List[Tuple[float, float]] = []
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic != fix_topic:
            continue
        msg = deserialize_message(data, NavSatFix)
        if msg.status.status >= 0:      # STATUS_NO_FIX is -1
            fixes.append((msg.latitude, msg.longitude))
    return fixes


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', required=True,
                        help='mission dir (contains rosbag2/, waypoints.yaml)')
    parser.add_argument('--imagery', required=True,
                        help='web-mercator mosaic.tif from the pipeline')
    parser.add_argument('--out', default='',
                        help='output PNG (default <mission>/report/route_map.png)')
    parser.add_argument('--fix-topic', default='/gnss_fix')
    parser.add_argument('--attribution', default='',
                        help='imagery attribution line (tile services)')
    parser.add_argument('--dpi', type=int, default=300)
    args = parser.parse_args(argv)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    import rasterio
    from matplotlib.lines import Line2D
    from pyproj import Transformer

    mission = os.path.abspath(os.path.expanduser(args.mission))
    out = args.out or os.path.join(mission, 'report', 'route_map.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)

    fixes = read_fixes(os.path.join(mission, 'rosbag2'), args.fix_topic)
    if not fixes:
        print(f'ERROR: no fixes on {args.fix_topic} in the bag',
              file=sys.stderr)
        return 1
    wps_path = os.path.join(mission, 'waypoints.yaml')
    waypoints = (yaml.safe_load(open(wps_path)) or []
                 if os.path.exists(wps_path) else [])

    to_3857 = Transformer.from_crs('EPSG:4326', 'EPSG:3857',
                                   always_xy=True)
    route = np.array([to_3857.transform(lon, lat) for lat, lon in fixes])

    fig, ax = plt.subplots(figsize=(11, 8.5))
    with rasterio.open(args.imagery) as src:
        if src.crs and src.crs.to_epsg() != 3857:
            print(f'WARNING: {args.imagery} is {src.crs}, expected '
                  'EPSG:3857 (web mercator) — positions may be off',
                  file=sys.stderr)
        img = src.read()
        extent = (src.bounds.left, src.bounds.right,
                  src.bounds.bottom, src.bounds.top)
        ax.imshow(np.transpose(img[:3], (1, 2, 0)), extent=extent,
                  interpolation='bilinear')

    ax.plot(route[:, 0], route[:, 1], color='#00ccff', lw=1.8,
            label='route', zorder=3)

    site_n = 0
    for wp in waypoints:
        color, marker = CATEGORY_STYLE.get(wp['category'], ('#aaaaaa', 'o'))
        x, y = to_3857.transform(wp['lon'], wp['lat'])
        size = 220 if wp['category'] == 'start' else 120
        ax.scatter([x], [y], c=color, marker=marker, s=size,
                   edgecolors='white', linewidths=0.8, zorder=4)
        text = wp['label']
        if wp['category'] == 'site':
            site_n += 1
            text = f'{site_n}. {text}'
        ax.annotate(text, (x, y), textcoords='offset points',
                    xytext=(8, 8), fontsize=8, color='white', zorder=5,
                    path_effects=None,
                    bbox=dict(boxstyle='round,pad=0.25', fc='black',
                              alpha=0.55, ec='none'))

    # zoom to route + waypoints with margin
    xs = list(route[:, 0]) + [to_3857.transform(w['lon'], w['lat'])[0]
                              for w in waypoints]
    ys = list(route[:, 1]) + [to_3857.transform(w['lon'], w['lat'])[1]
                              for w in waypoints]
    pad = max(max(xs) - min(xs), max(ys) - min(ys), 50.0) * 0.15
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])

    # legend (categories present + route)
    handles = [Line2D([], [], color='#00ccff', lw=1.8, label='route')]
    for cat, (color, marker) in CATEGORY_STYLE.items():
        if any(w['category'] == cat for w in waypoints):
            handles.append(Line2D([], [], color=color, marker=marker,
                                  ls='', ms=9, label=cat))
    ax.legend(handles=handles, loc='lower right', fontsize=8,
              framealpha=0.85)

    # scale bar (web-mercator metres ~ true metres at mid-lat / cos(lat))
    import math
    mid_lat = float(np.mean([lat for lat, _ in fixes]))
    span_m = (ax.get_xlim()[1] - ax.get_xlim()[0]) * math.cos(
        math.radians(mid_lat))
    for nice in (10, 20, 50, 100, 200, 500, 1000):
        if nice >= span_m / 8:
            break
    bar_len = nice / math.cos(math.radians(mid_lat))
    x0 = ax.get_xlim()[0] + (ax.get_xlim()[1] - ax.get_xlim()[0]) * 0.05
    y0 = ax.get_ylim()[0] + (ax.get_ylim()[1] - ax.get_ylim()[0]) * 0.05
    ax.plot([x0, x0 + bar_len], [y0, y0], color='white', lw=3, zorder=6)
    ax.annotate(f'{nice} m', (x0 + bar_len / 2, y0),
                textcoords='offset points', xytext=(0, 5), ha='center',
                color='white', fontsize=8, zorder=6)

    # north arrow
    ax.annotate('N', xy=(0.96, 0.90), xycoords='axes fraction',
                ha='center', fontsize=12, color='white', zorder=6)
    ax.annotate('', xy=(0.96, 0.965), xytext=(0.96, 0.905),
                xycoords='axes fraction',
                arrowprops=dict(fc='white', ec='white', width=2,
                                headwidth=8), zorder=6)

    title = os.path.basename(mission)
    ax.set_title(f'Route map — {title}')
    if args.attribution:
        ax.annotate(args.attribution, xy=(0.01, 0.01),
                    xycoords='axes fraction', fontsize=6, color='white')

    fig.tight_layout()
    fig.savefig(out, dpi=args.dpi)
    print(f'wrote {out} ({len(fixes)} fixes, {len(waypoints)} waypoints)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
