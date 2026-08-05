#!/usr/bin/env python3
"""
fetch_tiles.py — Download an XYZ tile-service bounding box into the same
offline layout that prepare_imagery.sh produces, for sites where no free
GeoTIFF exists (e.g. Drumheller). Also stitches the max-zoom tiles into
imagery/<name>/mosaic.tif so the Phase 6 report exporter works identically.

Usage:
  ./fetch_tiles.py --name drumheller \
      --bbox 51.4300,-112.6300,51.4600,-112.5800 \
      --zoom 13 19 \
      --url "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

  --bbox is lat_min,lon_min,lat_max,lon_max (SW corner, NE corner).
  --url  is any template with {z} {x} {y}. Note Esri's order is {z}/{y}/{x}.

Check the imagery provider's terms of service before bulk-downloading and
caching tiles offline; include their required attribution on your report map.

No dependencies beyond the Python stdlib (+ gdal-bin CLI for the mosaic step).
"""

import argparse
import math
import shutil
import subprocess
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

WEB_MERCATOR_MAX = 20037508.342789244  # metres, EPSG:3857 extent


def deg2tile(lat: float, lon: float, z: int) -> tuple[int, int]:
    """lat/lon -> XYZ (slippy) tile indices."""
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return x, y


def tile_bounds_3857(x: int, y: int, z: int) -> tuple[float, float, float, float]:
    """Tile indices -> (min_x, min_y, max_x, max_y) in EPSG:3857 metres."""
    n = 2 ** z
    size = 2 * WEB_MERCATOR_MAX / n
    min_x = -WEB_MERCATOR_MAX + x * size
    max_y = WEB_MERCATOR_MAX - y * size
    return min_x, max_y - size, min_x + size, max_y


def fetch_one(url: str, dest: Path, retries: int = 3) -> bool:
    if dest.exists():
        return True
    req = urllib.request.Request(url, headers={"User-Agent": "rover-imagery-pipeline/1.0"})
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                data = r.read()
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
            return True
        except Exception as e:
            if attempt == retries - 1:
                print(f"  FAILED {url}: {e}", file=sys.stderr)
                return False
            time.sleep(1.5 * (attempt + 1))
    return False


def stitch_mosaic(tile_dir: Path, out_tif: Path, z: int, ext: str) -> None:
    """Georeference each max-zoom tile with a world file, then GDAL-merge."""
    tiles = sorted((tile_dir / str(z)).glob(f"*/*.{ext}"))
    if not tiles:
        print("No tiles at max zoom; skipping mosaic.", file=sys.stderr)
        return
    print(f"==> Stitching {len(tiles)} tiles at z{z} into {out_tif.name}")
    wld_ext = {"png": "pgw", "jpg": "jgw", "jpeg": "jgw"}.get(ext, "wld")
    for t in tiles:
        x, y = int(t.parent.name), int(t.stem)
        min_x, min_y, max_x, max_y = tile_bounds_3857(x, y, z)
        px = (max_x - min_x) / 256.0
        # world file: x-scale, rot, rot, -y-scale, center of top-left pixel
        t.with_suffix("." + wld_ext).write_text(
            f"{px}\n0\n0\n{-px}\n{min_x + px / 2}\n{max_y - px / 2}\n"
        )
    vrt = out_tif.with_suffix(".vrt")
    subprocess.run(
        ["gdalbuildvrt", "-overwrite", "-a_srs", "EPSG:3857", str(vrt)]
        + [str(t) for t in tiles],
        check=True, capture_output=True,
    )
    subprocess.run(
        ["gdal_translate", "-q", "-of", "GTiff",
         "-co", "COMPRESS=JPEG", "-co", "TILED=YES", str(vrt), str(out_tif)],
        check=True,
    )
    vrt.unlink(missing_ok=True)
    for w in (tile_dir / str(z)).glob(f"*/*.{wld_ext}"):
        w.unlink()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True, help="tile-set name (imagery/<name>/)")
    ap.add_argument("--bbox", required=True,
                    help="lat_min,lon_min,lat_max,lon_max")
    ap.add_argument("--zoom", nargs=2, type=int, metavar=("MIN", "MAX"),
                    default=[13, 19])
    ap.add_argument("--url", required=True,
                    help="XYZ template with {z} {x} {y}")
    ap.add_argument("--ext", default="png", help="tile file extension (png/jpg)")
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--no-mosaic", action="store_true",
                    help="skip building mosaic.tif")
    args = ap.parse_args()

    lat_min, lon_min, lat_max, lon_max = map(float, args.bbox.split(","))
    if lat_min >= lat_max or lon_min >= lon_max:
        ap.error("bbox must be lat_min,lon_min,lat_max,lon_max with min < max")

    repo = Path(__file__).resolve().parent.parent
    out = repo / "imagery" / args.name
    tile_dir = out / "tiles"
    tile_dir.mkdir(parents=True, exist_ok=True)

    jobs: list[tuple[str, Path]] = []
    for z in range(args.zoom[0], args.zoom[1] + 1):
        x0, y1 = deg2tile(lat_min, lon_min, z)   # SW corner -> min x, max y
        x1, y0 = deg2tile(lat_max, lon_max, z)   # NE corner -> max x, min y
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                url = args.url.format(z=z, x=x, y=y)
                jobs.append((url, tile_dir / str(z) / str(x) / f"{y}.{args.ext}"))

    print(f"==> {len(jobs)} tiles, zoom {args.zoom[0]}-{args.zoom[1]} -> {tile_dir}")
    ok = 0
    with ThreadPoolExecutor(max_workers=args.threads) as pool:
        for done, good in enumerate(pool.map(lambda j: fetch_one(*j), jobs), 1):
            ok += good
            if done % 200 == 0 or done == len(jobs):
                print(f"    {done}/{len(jobs)} ({ok} ok)")

    if not args.no_mosaic and shutil.which("gdalbuildvrt"):
        stitch_mosaic(tile_dir, out / "mosaic.tif", args.zoom[1], args.ext)
    elif not args.no_mosaic:
        print("gdal-bin not found; skipped mosaic.tif", file=sys.stderr)

    size = sum(f.stat().st_size for f in tile_dir.rglob("*") if f.is_file())
    print(f"""
======================================================================
 DONE — {ok}/{len(jobs)} tiles ({size / 1e6:.1f} MB) in:
   {tile_dir}

 Serve:   ./scripts/serve_tiles.sh {args.name}
 mapviz:  http://localhost:8000/{args.name}/tiles/{{level}}/{{x}}/{{y}}.{args.ext}
          Max Zoom: {args.zoom[1]}
======================================================================""")
    return 0 if ok == len(jobs) else 1


if __name__ == "__main__":
    sys.exit(main())
