import datetime
import json

from rover_mapping.geojson_export import mission_geojson, write_geojson

TRACK = [(1.0, 51.4530, -112.7220), (2.0, 51.4532, -112.7223),
         (3.0, 51.4534, -112.7226)]
WAYPOINTS = [
    {'id': 'site_1', 'label': 'Site 1', 'category': 'site',
     'lat': 51.4532, 'lon': -112.7223, 'alt': 700.0, 'stamp': 2.0,
     'notes': ''},
    {'id': 'obstacle_1', 'label': 'Boulder', 'category': 'obstacle',
     'lat': 51.4534, 'lon': -112.7226},
]


def test_waypoints_become_placed_points():
    fc = mission_geojson(WAYPOINTS, [])
    assert fc['type'] == 'FeatureCollection'
    pts = [f for f in fc['features'] if f['geometry']['type'] == 'Point']
    assert len(pts) == 2
    lon, lat, alt = pts[0]['geometry']['coordinates']   # GeoJSON order
    assert (lat, lon, alt) == (51.4532, -112.7223, 700.0)
    assert pts[0]['properties']['id'] == 'site_1'
    assert pts[0]['properties']['category'] == 'site'
    assert 'notes' not in pts[0]['properties']           # empty fields dropped
    assert len(pts[1]['geometry']['coordinates']) == 2   # no alt recorded


def test_track_and_segments_become_lines():
    segments = [{'name': 'leg_1', 'start_stamp': 1.5, 'end_stamp': 3.0},
                {'name': 'open', 'start_stamp': 1.5, 'end_stamp': None},
                {'name': 'empty', 'start_stamp': 9.0, 'end_stamp': 10.0}]
    fc = mission_geojson([], TRACK, segments)
    lines = {f['properties'].get('name', f['properties']['kind']): f
             for f in fc['features']}
    assert len(lines['track']['geometry']['coordinates']) == 3
    assert lines['track']['properties']['start_stamp'] == 1.0
    assert len(lines['leg_1']['geometry']['coordinates']) == 2  # stamps 2, 3
    assert len(lines['open']['geometry']['coordinates']) == 2   # runs to track end
    assert 'empty' not in lines                          # <2 points: dropped


def test_write_geojson_is_valid_json_with_meta(tmp_path):
    # mission.yaml's date parses as datetime — must not break serialization
    meta = {'mission': '2026-08-08_t',
            'date': datetime.datetime(2026, 8, 8, 12, 0)}
    fc = mission_geojson(WAYPOINTS, TRACK, meta=meta)
    path = write_geojson(str(tmp_path / 'report' / 'mission.geojson'), fc)
    with open(path) as f:
        loaded = json.load(f)
    assert loaded['mission']['mission'] == '2026-08-08_t'
    assert loaded['mission']['date'].startswith('2026-08-08')
    assert len(loaded['features']) == 3
