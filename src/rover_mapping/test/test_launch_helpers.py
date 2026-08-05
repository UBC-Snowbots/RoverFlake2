import os

import pytest

from rover_mapping.launch_helpers import tile_to_latlon, tiles_center


def test_tile_to_latlon_world_corners():
    lat, lon = tile_to_latlon(0, 0, 0)
    assert lon == pytest.approx(-180.0)
    assert lat == pytest.approx(85.0511, abs=1e-3)
    # center of the single z0 tile is (0, 0)
    lat, lon = tile_to_latlon(0, 0.5, 0.5)
    assert lat == pytest.approx(0.0, abs=1e-9)
    assert lon == pytest.approx(0.0, abs=1e-9)


def test_tiles_center_from_layout(tmp_path):
    # single z10 tile containing UBC: x=161, y=350
    tile_dir = tmp_path / 'tiles' / '10' / '161'
    os.makedirs(tile_dir)
    (tile_dir / '350.png').touch()
    lat, lon = tiles_center(str(tmp_path / 'tiles'))
    # center of that tile
    exp_lat, exp_lon = tile_to_latlon(10, 161.5, 350.5)
    assert lat == pytest.approx(exp_lat)
    assert lon == pytest.approx(exp_lon)
    assert 48 < lat < 50 and -124 < lon < -123   # sanity: near Vancouver
