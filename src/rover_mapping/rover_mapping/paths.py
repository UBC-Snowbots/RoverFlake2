"""Filesystem layout and slippy-tile math for rover_mapping.

All data lives in the source tree (in-repo, like HMI layouts): missions in
src/rover_mapping/missions/, imagery in src/rover_mapping/imagery/. The root
resolves the same way on every machine:

  1. $ROVER_MAPPING_ROOT           explicit override
  2. $ROVERFLAKE_ROOT              the workspace convention (set in the
                                   container and the rover env scripts)
  3. walk up from this file        works for --symlink-install builds

Pure functions only — unit-testable, no ROS.
"""
from __future__ import annotations

import math
import os
from typing import List, Optional, Tuple


def package_root() -> str:
    """Absolute path of src/rover_mapping in the source tree."""
    env = os.environ.get('ROVER_MAPPING_ROOT')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    flake = os.environ.get('ROVERFLAKE_ROOT')
    if flake:
        cand = os.path.join(flake, 'src', 'rover_mapping')
        if os.path.isdir(cand):
            return cand
    here = os.path.dirname(os.path.realpath(__file__))
    cand = os.path.dirname(here)
    if os.path.isdir(os.path.join(cand, 'imagery')) or \
       os.path.basename(cand) == 'rover_mapping':
        return cand
    raise RuntimeError(
        'cannot locate src/rover_mapping — set ROVERFLAKE_ROOT (or '
        'ROVER_MAPPING_ROOT), or build with --symlink-install')


def missions_root() -> str:
    return os.path.join(package_root(), 'missions')


def imagery_root() -> str:
    return os.path.join(package_root(), 'imagery')


def tiles_dir(site: str) -> str:
    return os.path.join(imagery_root(), site, 'tiles')


def list_sites() -> List[str]:
    root = imagery_root()
    if not os.path.isdir(root):
        return []
    return sorted(s for s in os.listdir(root)
                  if os.path.isdir(os.path.join(root, s, 'tiles')))


def default_site() -> Optional[str]:
    """Newest-fetched site, or None."""
    sites = list_sites()
    if not sites:
        return None
    return max(sites, key=lambda s: os.path.getmtime(tiles_dir(s)))


def site_covering(lat: float, lon: float) -> Optional[str]:
    """The site whose tiles contain this point, or None."""
    for site in list_sites():
        tiles = tiles_dir(site)
        zs = zoom_levels(tiles)
        if not zs:
            continue
        x, y = deg2tile(lat, lon, zs[-1])
        stem = os.path.join(tiles, str(zs[-1]), str(int(x)), str(int(y)))
        if any(os.path.exists(stem + '.' + e) for e in ('jpg', 'jpeg', 'png')):
            return site
    return None


def resolve_mission(ref: str) -> str:
    """Mission dir from a path, exact dir name, bare name, or ''/'last'
    (= newest by mtime). Raises FileNotFoundError with a helpful message."""
    if ref and os.path.isdir(ref):
        return os.path.abspath(ref)
    root = missions_root()
    dirs = [os.path.join(root, d) for d in os.listdir(root)
            if os.path.isdir(os.path.join(root, d))] if os.path.isdir(root) else []
    if not dirs:
        raise FileNotFoundError(f'no missions recorded yet in {root}')
    if not ref or ref == 'last':
        return max(dirs, key=os.path.getmtime)
    for d in dirs:
        if os.path.basename(d) == ref:
            return d
    matches = [d for d in dirs if os.path.basename(d).endswith('_' + ref)]
    if len(matches) == 1:
        return matches[0]
    if matches:
        raise FileNotFoundError(
            f'"{ref}" is ambiguous: {", ".join(map(os.path.basename, matches))}')
    raise FileNotFoundError(f'no mission "{ref}" in {root}')


# ------------------------------------------------------------ slippy tiles
def deg2tile(lat: float, lon: float, z: int) -> Tuple[float, float]:
    """lat/lon -> fractional XYZ tile coordinates."""
    n = 2.0 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def tile2deg(x: float, y: float, z: int) -> Tuple[float, float]:
    n = 2.0 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def zoom_levels(tiles: str) -> List[int]:
    if not os.path.isdir(tiles):
        return []
    return sorted(int(d) for d in os.listdir(tiles) if d.isdigit())


def meters_per_pixel(lat: float, z: int) -> float:
    """Web-mercator ground resolution at 256 px/tile."""
    return 156543.03392 * math.cos(math.radians(lat)) / (2.0 ** z)
