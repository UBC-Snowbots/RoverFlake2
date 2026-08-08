"""Download an XYZ tile bounding box for offline use.

    ros2 run rover_mapping fetch_tiles -- --name circ \\
        --center 51.453361 -112.722667 --km 1.5

Tiles land in src/rover_mapping/imagery/<name>/tiles/{z}/{x}/{y}.<ext>
(the layout GnssMapWidget and the exporter read). Esri World Imagery by
default — check the provider's terms and keep their attribution on report
maps. Stdlib only.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from rover_mapping import paths

ESRI_URL = ('https://server.arcgisonline.com/ArcGIS/rest/services/'
            'World_Imagery/MapServer/tile/{z}/{y}/{x}')
ESRI_ATTR = 'Imagery: Esri, Maxar, Earthstar Geographics'
UA = {'User-Agent': 'rover-imagery-pipeline/2.0'}


def fetch_one(url: str, dest: Path, retries: int = 3) -> bool:
    if dest.exists():
        return True
    req = urllib.request.Request(url, headers=UA)
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                data = r.read()
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
            return True
        except Exception as e:
            if attempt == retries - 1:
                print(f'  FAILED {url}: {e}', file=sys.stderr)
                return False
            time.sleep(1.5 * (attempt + 1))
    return False


def probe_max_zoom(url_tpl: str, lat: float, lon: float, zmax: int,
                   zmin: int) -> int:
    """Step max zoom down past 'not yet available' placeholder tiles: real
    imagery differs between adjacent tiles; placeholders are byte-identical."""
    for z in range(zmax, zmin, -1):
        x, y = (int(v) for v in paths.deg2tile(lat, lon, z))
        try:
            a = urllib.request.urlopen(
                urllib.request.Request(url_tpl.format(z=z, x=x, y=y),
                                       headers=UA), timeout=30).read()
            b = urllib.request.urlopen(
                urllib.request.Request(url_tpl.format(z=z, x=x + 1, y=y),
                                       headers=UA), timeout=30).read()
        except Exception:
            continue
        if a != b:
            return z
        print(f'z{z}: placeholder tiles, stepping down')
    return zmin


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', required=True, help='site name (imagery/<name>/)')
    box = ap.add_mutually_exclusive_group(required=True)
    box.add_argument('--center', nargs=2, type=float, metavar=('LAT', 'LON'))
    box.add_argument('--bbox', help='lat_min,lon_min,lat_max,lon_max')
    ap.add_argument('--km', type=float, default=1.5,
                    help='half-width around --center (default 1.5)')
    ap.add_argument('--zoom', nargs=2, type=int, default=(13, 19),
                    metavar=('MIN', 'MAX'))
    ap.add_argument('--url', default=ESRI_URL,
                    help='tile URL template with {z} {x} {y}')
    ap.add_argument('--ext', default='jpg')
    args = ap.parse_args(argv)

    if args.center:
        lat, lon = args.center
        dlat = args.km / 111.32
        dlon = args.km / (111.32 * math.cos(math.radians(lat)))
        lat_min, lat_max = lat - dlat, lat + dlat
        lon_min, lon_max = lon - dlon, lon + dlon
    else:
        lat_min, lon_min, lat_max, lon_max = map(float, args.bbox.split(','))

    zmin, zmax = args.zoom
    zmax = probe_max_zoom(args.url, (lat_min + lat_max) / 2,
                          (lon_min + lon_max) / 2, zmax, zmin)

    tiles_root = Path(paths.tiles_dir(args.name))
    jobs = []
    for z in range(zmin, zmax + 1):
        x0, y1 = (int(v) for v in paths.deg2tile(lat_min, lon_min, z))
        x1, y0 = (int(v) for v in paths.deg2tile(lat_max, lon_max, z))
        for x in range(min(x0, x1), max(x0, x1) + 1):
            for y in range(min(y0, y1), max(y0, y1) + 1):
                jobs.append((args.url.format(z=z, x=x, y=y),
                             tiles_root / str(z) / str(x) / f'{y}.{args.ext}'))

    print(f'==> {args.name}: {len(jobs)} tiles, z{zmin}-{zmax} -> {tiles_root}')
    with ThreadPoolExecutor(max_workers=8) as pool:
        results = list(pool.map(lambda j: fetch_one(*j), jobs))
    failed = results.count(False)
    print(f'==> done: {len(jobs) - failed} ok, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
