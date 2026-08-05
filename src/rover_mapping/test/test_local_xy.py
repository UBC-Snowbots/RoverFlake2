import math

import pytest

from rover_mapping.local_xy import (local_xy_to_wgs84, make_origin,
                                    wgs84_to_local_xy)

LAT0, LON0 = 49.262100, -123.248800   # UBC


def test_origin_maps_to_zero():
    origin = make_origin(LAT0, LON0, 70.0)
    x, y = wgs84_to_local_xy(origin, LAT0, LON0)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_north_east_offsets():
    origin = make_origin(LAT0, LON0)
    # +1e-4 deg latitude ~ 11.1 m north
    x, y = wgs84_to_local_xy(origin, LAT0 + 1e-4, LON0)
    assert x == pytest.approx(0.0, abs=0.01)
    assert y == pytest.approx(11.1, abs=0.2)
    # +1e-4 deg longitude ~ 11.1 * cos(lat) m east
    x, y = wgs84_to_local_xy(origin, LAT0, LON0 + 1e-4)
    assert y == pytest.approx(0.0, abs=0.01)
    assert x == pytest.approx(11.1 * math.cos(math.radians(LAT0)), abs=0.2)


def test_roundtrip():
    origin = make_origin(LAT0, LON0)
    lat, lon = LAT0 + 0.0013, LON0 - 0.0021
    x, y = wgs84_to_local_xy(origin, lat, lon)
    lat2, lon2 = local_xy_to_wgs84(origin, x, y)
    assert lat2 == pytest.approx(lat, abs=1e-9)
    assert lon2 == pytest.approx(lon, abs=1e-9)
