import json
import os

from rover_mapping import exporter, mission_io


def test_data_bbox_pads_and_never_collapses():
    bbox = exporter.data_bbox([51.0], [-112.0])
    lat_min, lon_min, lat_max, lon_max = bbox
    assert lat_min < 51.0 < lat_max
    assert lon_min < -112.0 < lon_max
    assert lat_max - lat_min >= 0.0002
    assert lon_max - lon_min > lat_max - lat_min   # lon degrees are shorter


def test_pick_zoom_respects_pixel_cap():
    bbox = (51.0, -112.01, 51.01, -112.0)   # ~1 km square
    z = exporter.pick_zoom(bbox, [13, 15, 17, 19], max_px=1000)
    assert z in (13, 15)                     # z17+ would exceed 1000 px
    assert exporter.pick_zoom(bbox, [13], max_px=10) == 13  # floor


def test_export_without_imagery(tmp_path, monkeypatch):
    """Track + waypoints but no tiles: still writes a PNG."""
    monkeypatch.setenv('ROVER_MAPPING_ROOT', str(tmp_path))
    mission = tmp_path / 'missions' / '2026-08-05_t'
    mission.mkdir(parents=True)
    (mission / 'track.csv').write_text(
        'stamp,lat,lon,alt\n1,51.4530,-112.7220,700\n2,51.4534,-112.7226,700\n')
    mission_io.atomic_dump_yaml(str(mission / 'waypoints.yaml'), [
        {'id': 'site_1', 'label': 'Site 1', 'category': 'site',
         'lat': 51.4532, 'lon': -112.7223}])

    out = exporter.export_mission(str(mission))
    assert os.path.exists(out)
    assert out.endswith('report/route_map.png')
    assert os.path.getsize(out) > 1000

    poi = os.path.join(os.path.dirname(out), 'poi.csv')
    with open(poi) as f:
        rows = f.read().splitlines()
    assert rows[0].startswith('id,label,category')
    assert rows[1].startswith('track_start')
    assert rows[-1].startswith('track_end')
    assert any(r.startswith('site_1') for r in rows)

    with open(os.path.join(os.path.dirname(out), 'mission.geojson')) as f:
        fc = json.load(f)
    kinds = [ft['properties']['kind'] for ft in fc['features']]
    assert kinds == ['waypoint', 'track']
    assert fc['mission']['mission'] == '2026-08-05_t'
