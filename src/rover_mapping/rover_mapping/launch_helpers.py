"""Shared path/config helpers for the rover_mapping launch files.

The imagery, config, scripts and missions directories live in the *source*
tree (tiles are big and gitignored — they are not installed). Build with
``colcon build --symlink-install`` so ``__file__`` resolves back to the
source; set ``ROVER_MAPPING_ROOT`` to override explicitly.
"""
from __future__ import annotations

import math
import os
import re
from typing import Tuple


def tile_to_latlon(z: int, x: float, y: float) -> Tuple[float, float]:
    """Slippy tile indices -> (lat, lon) of the tile's top-left corner
    (pass fractional x/y for points inside a tile)."""
    n = 2.0 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lon


def tiles_center(tiles_dir: str) -> Tuple[float, float]:
    """(lat, lon) center of a downloaded XYZ tile set — used to anchor the
    local-XY origin so mapviz has a valid wgs84<->map transform before any
    GPS fix arrives."""
    z = max(int(d) for d in os.listdir(tiles_dir) if d.isdigit())
    zdir = os.path.join(tiles_dir, str(z))
    xs = sorted(int(d) for d in os.listdir(zdir) if d.isdigit())
    ys = sorted(int(f.split('.')[0])
                for x in xs
                for f in os.listdir(os.path.join(zdir, str(x)))
                if f.split('.')[0].isdigit())
    return tile_to_latlon(z, (xs[0] + xs[-1] + 1) / 2.0,
                          (ys[0] + ys[-1] + 1) / 2.0)


def source_root() -> str:
    """Absolute path of src/rover_mapping in the source tree."""
    env = os.environ.get('ROVER_MAPPING_ROOT')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    here = os.path.dirname(os.path.realpath(__file__))
    root = os.path.dirname(here)
    if not os.path.isdir(os.path.join(root, 'imagery_pipeline')):
        raise RuntimeError(
            'cannot locate the rover_mapping source tree — build with '
            '--symlink-install or export ROVER_MAPPING_ROOT')
    return root


def imagery_pipeline_dir() -> str:
    return os.path.join(source_root(), 'imagery_pipeline')


def config_dir() -> str:
    return os.path.join(source_root(), 'config')


def render_site_config(site: str, port: int = 8000, ext: str = 'png',
                       max_zoom: int = 20,
                       fix_topic: str = '/gnss_fix') -> str:
    """Render config/mapviz_offline.mvc for a tile set + fix topic.

    mapviz 2.6.5 has NO "config" node parameter — it loads
    $ROS_WORKSPACE/.mapviz_config, falling back to ~/.mapviz_config. So we
    render config/<site>.mvc and symlink ~/.mapviz_config at it (mapviz's
    autosave then writes GUI tweaks back into the repo config, which is
    what we want). Returns the rendered path.
    """
    template = os.path.join(config_dir(), 'mapviz_offline.mvc')
    out = os.path.join(config_dir(), f'{site}.mvc')
    base_url = (f'http://localhost:{port}/{site}/tiles/'
                '{level}/{x}/{y}.' + ext)

    with open(template) as f:
        text = f.read()
    pat = r'^(\s*-?\s*)%s.*$'
    text = re.sub(pat % 'base_url:', rf'\1base_url: {base_url}',
                  text, flags=re.M)
    text = re.sub(pat % 'max_zoom:', rf'\1max_zoom: {max_zoom}',
                  text, flags=re.M)
    text = re.sub(pat % 'name: offline_', rf'\1name: offline_{site}',
                  text, flags=re.M)
    text = re.sub(pat % 'source: offline_', rf'\1source: offline_{site}',
                  text, flags=re.M)
    text = re.sub(pat % r'topic: /gnss_fix', rf'\1topic: {fix_topic}',
                  text, flags=re.M)
    with open(out, 'w') as f:
        f.write(text)

    target = os.path.expanduser('~/.mapviz_config')
    if os.path.lexists(target) and not os.path.islink(target):
        os.rename(target, target + '.bak')
    tmp = target + '.tmp'
    if os.path.lexists(tmp):
        os.unlink(tmp)
    os.symlink(out, tmp)
    os.replace(tmp, target)
    return out
