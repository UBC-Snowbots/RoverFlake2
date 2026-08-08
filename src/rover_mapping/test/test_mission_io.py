import os

import pytest
import yaml

from rover_mapping import mission_io


def test_next_waypoint_id_increments_per_category():
    wps = []
    assert mission_io.next_waypoint_id(wps, 'site') == 'site_1'
    wps.append({'id': 'site_1', 'category': 'site'})
    assert mission_io.next_waypoint_id(wps, 'site') == 'site_2'
    assert mission_io.next_waypoint_id(wps, 'sample') == 'sample_1'


def test_next_waypoint_id_stays_unique_after_edits():
    wps = [{'id': 'site_2', 'category': 'site'}]  # site_1 deleted by hand
    assert mission_io.next_waypoint_id(wps, 'site') == 'site_3'


def test_default_label():
    assert mission_io.default_label('site_2') == 'Site 2'
    assert mission_io.default_label('obstacle_11') == 'Obstacle 11'


def test_append_waypoint_atomic(tmp_path):
    path = str(tmp_path / 'waypoints.yaml')
    mission_io.append_waypoint(path, {'id': 'start_1', 'category': 'start'})
    mission_io.append_waypoint(path, {'id': 'site_1', 'category': 'site'})
    data = yaml.safe_load(open(path))
    assert [w['id'] for w in data] == ['start_1', 'site_1']
    # no stray temp files left behind
    assert os.listdir(tmp_path) == ['waypoints.yaml']


def test_load_yaml_list_empty_and_missing(tmp_path):
    missing = str(tmp_path / 'nope.yaml')
    assert mission_io.load_yaml_list(missing) == []
    empty = tmp_path / 'empty.yaml'
    empty.write_text('')
    assert mission_io.load_yaml_list(str(empty)) == []


def test_load_yaml_list_rejects_non_list(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('key: value')
    with pytest.raises(ValueError):
        mission_io.load_yaml_list(str(bad))


def test_segments_open_close(tmp_path):
    path = str(tmp_path / 'segments.yaml')
    mission_io.open_segment(path, 'leg_1', 100.0)
    mission_io.open_segment(path, 'leg_2', 200.0)
    assert mission_io.close_segment(path, 250.0) == 'leg_2'   # last open
    assert mission_io.close_segment(path, 300.0, 'leg_1') == 'leg_1'
    assert mission_io.close_segment(path, 400.0) is None      # all closed
    segs = yaml.safe_load(open(path))
    assert segs[0]['end_stamp'] == 300.0
    assert segs[1]['end_stamp'] == 250.0


def test_segment_offsets():
    seg = {'name': 'leg', 'start_stamp': 150.0, 'end_stamp': 210.5}
    start, dur = mission_io.segment_offsets(seg, 100.0)
    assert start == pytest.approx(50.0)
    assert dur == pytest.approx(60.5)
    # segment started before the bag (shouldn't happen, but clamp)
    start, dur = mission_io.segment_offsets(seg, 200.0)
    assert start == 0.0
    # open segment -> no duration
    _, dur = mission_io.segment_offsets(
        {'name': 'x', 'start_stamp': 1.0, 'end_stamp': None}, 0.0)
    assert dur is None


def test_coord_error_accepts_real_coordinates():
    assert mission_io.coord_error(51.453361, -112.722667) is None
    assert mission_io.coord_error(-90.0, 180.0) is None   # edges are valid


@pytest.mark.parametrize('lat,lon', [
    (91.0, 0.0), (-90.5, 0.0),      # latitude out of range
    (0.0, 181.0), (0.0, -180.5),    # longitude out of range
    (0.0, 0.0),                     # empty fields parsed as zeros
])
def test_coord_error_rejects_bad_input(lat, lon):
    assert mission_io.coord_error(lat, lon)
