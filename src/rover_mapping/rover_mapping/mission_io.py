"""Pure helpers for mission sidecar files (waypoints.yaml / segments.yaml).

Everything here is ROS-free and unit-tested. Writes are atomic: dump to a
temp file in the same directory, fsync, then os.replace() — a crash mid-write
can never lose previously-saved entries.
"""
from __future__ import annotations

import os
import tempfile
from typing import List, Optional

import yaml

CATEGORIES = ('start', 'site', 'sample', 'obstacle', 'landmark')

# Category color map (RGBA 0-1) — used by the report-map exporter.
CATEGORY_COLORS = {
    'start':    (0.13, 0.75, 0.13, 1.0),   # green
    'site':     (0.90, 0.10, 0.10, 1.0),   # red
    'sample':   (0.58, 0.15, 0.80, 1.0),   # purple
    'obstacle': (1.00, 0.55, 0.00, 1.0),   # orange
    'landmark': (0.15, 0.40, 0.95, 1.0),   # blue
}


def load_yaml_list(path: str) -> List[dict]:
    """Load a YAML file expected to hold a list; empty/missing -> []."""
    if not os.path.exists(path):
        return []
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if data is None:
        return []
    if not isinstance(data, list):
        raise ValueError(f'{path}: expected a YAML list, got {type(data).__name__}')
    return data


def atomic_dump_yaml(path: str, data: list) -> None:
    """Write YAML atomically (temp file + rename in the same directory)."""
    dirname = os.path.dirname(os.path.abspath(path))
    os.makedirs(dirname, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=dirname, prefix='.tmp_', suffix='.yaml')
    try:
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False,
                           allow_unicode=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def coord_error(lat: float, lon: float) -> Optional[str]:
    """Reject coordinates a typo would produce; None means they're usable.

    0, 0 is out in the Atlantic — as a manually typed waypoint it is always
    an empty field or a parse slip, never a real target.
    """
    if not -90.0 <= lat <= 90.0:
        return f'latitude {lat} out of range (-90..90)'
    if not -180.0 <= lon <= 180.0:
        return f'longitude {lon} out of range (-180..180)'
    if lat == 0.0 and lon == 0.0:
        return 'refusing to tag 0, 0 — enter a real coordinate'
    return None


def next_waypoint_id(waypoints: List[dict], category: str) -> str:
    """Auto-increment ids per category: site_1, site_2, ... unique per run."""
    existing = {wp.get('id') for wp in waypoints}
    n = sum(1 for wp in waypoints if wp.get('category') == category) + 1
    wp_id = f'{category}_{n}'
    while wp_id in existing:          # ids must stay unique even after edits
        n += 1
        wp_id = f'{category}_{n}'
    return wp_id


def default_label(wp_id: str) -> str:
    """site_2 -> 'Site 2'."""
    category, _, n = wp_id.rpartition('_')
    return f'{category.capitalize()} {n}'


def append_waypoint(path: str, waypoint: dict) -> List[dict]:
    """Append one waypoint and atomically rewrite the file. Returns the list."""
    waypoints = load_yaml_list(path)
    waypoints.append(waypoint)
    atomic_dump_yaml(path, waypoints)
    return waypoints


def open_segment(path: str, name: str, start_stamp: float) -> List[dict]:
    """Record a segment start (end_stamp filled in later)."""
    segments = load_yaml_list(path)
    segments.append({'name': name, 'start_stamp': float(start_stamp),
                     'end_stamp': None})
    atomic_dump_yaml(path, segments)
    return segments


def close_segment(path: str, end_stamp: float,
                  name: Optional[str] = None) -> Optional[str]:
    """Close the named (or last open) segment. Returns its name or None."""
    segments = load_yaml_list(path)
    for seg in reversed(segments):
        if seg.get('end_stamp') is None and (not name or seg.get('name') == name):
            seg['end_stamp'] = float(end_stamp)
            atomic_dump_yaml(path, segments)
            return seg['name']
    return None


def segment_offsets(segment: dict, bag_start_stamp: float):
    """(start_offset, duration) in seconds of a segment relative to bag start."""
    start = max(0.0, float(segment['start_stamp']) - bag_start_stamp)
    end = segment.get('end_stamp')
    duration = None if end is None else max(0.0, float(end) - float(segment['start_stamp']))
    return start, duration
