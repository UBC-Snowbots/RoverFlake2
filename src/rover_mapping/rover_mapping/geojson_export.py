"""Mission data -> GeoJSON (RFC 7946) for report/mission.geojson.

One FeatureCollection: every waypoint as a Point feature, the drive track as
a LineString, and each segment as a LineString sliced from the track by its
stamp range. Coordinates are [lon, lat(, alt)] per the spec, so the file
drops straight into QGIS / geojson.io / Google Earth. Pure functions, ROS-free.
"""
from __future__ import annotations

import json
import os
import tempfile
from typing import List, Optional, Sequence, Tuple

Track = Sequence[Tuple[float, float, float]]   # (stamp, lat, lon)


def _point(wp: dict) -> dict:
    coords = [round(float(wp['lon']), 8), round(float(wp['lat']), 8)]
    if wp.get('alt') is not None:
        coords.append(float(wp['alt']))
    props = {'kind': 'waypoint'}
    props.update({k: wp[k] for k in ('id', 'label', 'category', 'stamp', 'notes')
                  if wp.get(k) not in (None, '')})
    return {'type': 'Feature',
            'geometry': {'type': 'Point', 'coordinates': coords},
            'properties': props}


def _line(points: Track, props: dict) -> dict:
    coords = [[round(lon, 8), round(lat, 8)] for _, lat, lon in points]
    return {'type': 'Feature',
            'geometry': {'type': 'LineString', 'coordinates': coords},
            'properties': props}


def mission_geojson(waypoints: List[dict], track: Track,
                    segments: Sequence[dict] = (),
                    meta: Optional[dict] = None) -> dict:
    features = [_point(w) for w in waypoints]
    if len(track) > 1:
        features.append(_line(track, {
            'kind': 'track', 'points': len(track),
            'start_stamp': track[0][0], 'end_stamp': track[-1][0]}))
    for seg in segments:
        start = float(seg['start_stamp'])
        end = seg.get('end_stamp')
        end = float(end) if end is not None else float('inf')
        pts = [p for p in track if start <= p[0] <= end]
        if len(pts) > 1:
            features.append(_line(pts, {
                'kind': 'segment', 'name': seg.get('name'),
                'start_stamp': start, 'end_stamp': seg.get('end_stamp')}))
    fc = {'type': 'FeatureCollection', 'features': features}
    if meta:
        fc['mission'] = meta   # foreign member, allowed by RFC 7946 §6.1
    return fc


def write_geojson(path: str, collection: dict) -> str:
    """Atomic write (temp + rename), like mission_io. default=str keeps
    YAML-loaded values JSON-safe (mission.yaml's date parses as datetime)."""
    dirname = os.path.dirname(os.path.abspath(path))
    os.makedirs(dirname, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=dirname, prefix='.tmp_', suffix='.geojson')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(collection, f, indent=2, default=str)
            f.write('\n')
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path
