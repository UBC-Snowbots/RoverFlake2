import os
import time

import pytest

from rover_mapping import paths


def test_tile_roundtrip():
    lat, lon = 51.453361, -112.722667
    x, y = paths.deg2tile(lat, lon, 18)
    back = paths.tile2deg(x, y, 18)
    assert abs(back[0] - lat) < 1e-9
    assert abs(back[1] - lon) < 1e-9


def test_meters_per_pixel_halves_per_zoom():
    a = paths.meters_per_pixel(51.0, 17)
    b = paths.meters_per_pixel(51.0, 18)
    assert abs(a / b - 2.0) < 1e-9


def test_resolve_mission(tmp_path, monkeypatch):
    monkeypatch.setenv('ROVER_MAPPING_ROOT', str(tmp_path))
    root = tmp_path / 'missions'
    old = root / '2026-08-01_alpha'
    new = root / '2026-08-02_beta'
    for d in (old, new):
        d.mkdir(parents=True)
    os.utime(old, (1, 1))

    assert paths.resolve_mission('last') == str(new)
    assert paths.resolve_mission('') == str(new)
    assert paths.resolve_mission('alpha') == str(old)
    assert paths.resolve_mission('2026-08-01_alpha') == str(old)
    with pytest.raises(FileNotFoundError):
        paths.resolve_mission('nope')


def test_resolve_mission_ambiguous(tmp_path, monkeypatch):
    monkeypatch.setenv('ROVER_MAPPING_ROOT', str(tmp_path))
    root = tmp_path / 'missions'
    for d in ('2026-08-01_run', '2026-08-02_run'):
        (root / d).mkdir(parents=True)
    with pytest.raises(FileNotFoundError, match='ambiguous'):
        paths.resolve_mission('run')
