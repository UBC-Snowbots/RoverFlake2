"""Report-map exporter: offline tiles + track.csv + waypoints.yaml ->
report/route_map.png, mission.geojson (all tags + track, GIS-ready), poi.csv.

Pillow only — no gdal, no matplotlib, no bag reading (mission_manager logs
the track to CSV while recording). Degrades to a plain background when the
site imagery hasn't been fetched; the track and waypoints still render.

    ros2 run rover_mapping export_map            # newest mission
    ros2 run rover_mapping export_map -- field2  # by name
"""
from __future__ import annotations

import csv
import math
import os
import sys
from typing import List, Optional, Tuple

import yaml
from PIL import Image, ImageDraw, ImageFont

from rover_mapping import paths
from rover_mapping.geojson_export import mission_geojson, write_geojson
from rover_mapping.mission_io import CATEGORY_COLORS, load_yaml_list

MAX_PX = 3000          # cap of the stitched canvas' long side
MIN_PX = 800           # small missions are upscaled to at least this
MARGIN = 0.18          # bbox padding as a fraction of the data span
BG = (24, 24, 37)      # canvas where tiles are missing
TRACK = (137, 220, 235)
ATTRIBUTION = 'Imagery: Esri, Maxar, Earthstar Geographics'


def load_track(mission_dir: str) -> List[Tuple[float, float, float]]:
    """track.csv -> [(stamp, lat, lon)]."""
    path = os.path.join(mission_dir, 'track.csv')
    if not os.path.exists(path):
        return []
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append((float(row['stamp']), float(row['lat']),
                         float(row['lon'])))
    return rows


def data_bbox(lats: List[float], lons: List[float],
              margin: float = MARGIN,
              min_span_deg: float = 0.0002) -> Tuple[float, float, float, float]:
    """(lat_min, lon_min, lat_max, lon_max): proportional padding around the
    data, with a minimum ground span (~22 m) so tiny or single-point missions
    still fill the frame instead of shrinking to a speck."""
    lat_c = (min(lats) + max(lats)) / 2
    lon_c = (min(lons) + max(lons)) / 2
    cos = max(0.2, math.cos(math.radians(lat_c)))
    lat_span = max(max(lats) - min(lats), min_span_deg) * (1 + 2 * margin)
    lon_span = max(max(lons) - min(lons), min_span_deg / cos) * (1 + 2 * margin)
    return (lat_c - lat_span / 2, lon_c - lon_span / 2,
            lat_c + lat_span / 2, lon_c + lon_span / 2)


def pick_zoom(bbox: Tuple[float, float, float, float], zooms: List[int],
              max_px: int = MAX_PX) -> int:
    """Highest available zoom whose stitched bbox stays within max_px."""
    lat_min, lon_min, lat_max, lon_max = bbox
    for z in sorted(zooms, reverse=True):
        x0, y1 = paths.deg2tile(lat_min, lon_min, z)
        x1, y0 = paths.deg2tile(lat_max, lon_max, z)
        if max(abs(x1 - x0), abs(y1 - y0)) * 256 <= max_px:
            return z
    return min(zooms)


def _font(size: int) -> ImageFont.ImageFont:
    for cand in ('/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf',
                 '/usr/share/fonts/TTF/DejaVuSansMono-Bold.ttf'):
        if os.path.exists(cand):
            return ImageFont.truetype(cand, size)
    return ImageFont.load_default()


def _rgb(cat: str) -> Tuple[int, int, int]:
    r, g, b, _ = CATEGORY_COLORS.get(cat, (1, 1, 1, 1))
    return int(r * 255), int(g * 255), int(b * 255)


def export_mission(mission_dir: str, site: Optional[str] = None,
                   out: Optional[str] = None) -> str:
    track = load_track(mission_dir)
    waypoints = load_yaml_list(os.path.join(mission_dir, 'waypoints.yaml'))
    lats = [p[1] for p in track] + [w['lat'] for w in waypoints]
    lons = [p[2] for p in track] + [w['lon'] for w in waypoints]
    if not lats:
        raise ValueError(f'{mission_dir}: no track and no waypoints to draw')

    meta = {}
    mpath = os.path.join(mission_dir, 'mission.yaml')
    if os.path.exists(mpath):
        with open(mpath) as f:
            meta = yaml.safe_load(f) or {}
    if site is None:
        site = (meta.get('site')
                or paths.site_covering(sum(lats) / len(lats),
                                       sum(lons) / len(lons))
                or paths.default_site())

    tiles = paths.tiles_dir(site) if site else ''
    zooms = paths.zoom_levels(tiles)
    bbox = data_bbox(lats, lons)
    # No imagery: still pick the sharpest zoom under the cap so bench-scale
    # missions don't collapse to a handful of pixels.
    z = pick_zoom(bbox, zooms or list(range(10, 23)))

    lat_min, lon_min, lat_max, lon_max = bbox
    x0f, y1f = paths.deg2tile(lat_min, lon_min, z)
    x1f, y0f = paths.deg2tile(lat_max, lon_max, z)
    tx0, tx1 = int(x0f), int(x1f)
    ty0, ty1 = int(y0f), int(y1f)

    img = Image.new('RGB', ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256), BG)
    missing = 0
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            tile = None
            for ext in ('jpg', 'jpeg', 'png'):
                p = os.path.join(tiles, str(z), str(tx), f'{ty}.{ext}')
                if os.path.exists(p):
                    tile = Image.open(p).convert('RGB')
                    break
            if tile is None:
                missing += 1
                continue
            img.paste(tile, ((tx - tx0) * 256, (ty - ty0) * 256))

    def px(lat: float, lon: float) -> Tuple[float, float]:
        x, y = paths.deg2tile(lat, lon, z)
        return (x - tx0) * 256, (y - ty0) * 256

    # Crop to the padded bbox, upscale tiny crops, then draw overlays (so
    # text and markers stay crisp at their fixed pixel sizes).
    left, top = px(lat_max, lon_min)
    right, bottom = px(lat_min, lon_max)
    img = img.crop((int(left), int(top), int(right), int(bottom)))
    scale = max(1, math.ceil(MIN_PX / max(img.width, img.height)))
    if scale > 1:
        img = img.resize((img.width * scale, img.height * scale),
                         Image.BILINEAR)
    draw = ImageDraw.Draw(img)

    def cpx(lat: float, lon: float) -> Tuple[float, float]:
        x, y = px(lat, lon)
        return (x - int(left)) * scale, (y - int(top)) * scale

    if len(track) > 1:
        draw.line([cpx(p[1], p[2]) for p in track], fill=TRACK, width=4,
                  joint='curve')

    font = _font(16)
    if track:
        sx, sy = cpx(track[0][1], track[0][2])
        ex, ey = cpx(track[-1][1], track[-1][2])
        draw.ellipse((sx - 6, sy - 6, sx + 6, sy + 6),
                     outline=(255, 255, 255), width=3)
        draw.text((sx + 10, sy - 20), 'start', fill=(255, 255, 255),
                  font=font, stroke_width=2, stroke_fill=(0, 0, 0))
        draw.rectangle((ex - 6, ey - 6, ex + 6, ey + 6),
                       fill=(255, 255, 255), outline=(0, 0, 0), width=2)
        draw.text((ex + 10, ey + 4), 'end', fill=(255, 255, 255),
                  font=font, stroke_width=2, stroke_fill=(0, 0, 0))
    for w in waypoints:
        x, y = cpx(w['lat'], w['lon'])
        col = _rgb(w['category'])
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill=col,
                     outline=(0, 0, 0), width=2)
        draw.text((x + 11, y - 9), w['label'], fill=col, font=font,
                  stroke_width=2, stroke_fill=(0, 0, 0))

    # Legend (categories present), top-right
    cats = sorted({w['category'] for w in waypoints},
                  key=list(CATEGORY_COLORS).index)
    if cats:
        lx, ly = img.width - 150, 12
        draw.rectangle((lx - 10, ly - 6, img.width - 8, ly + 22 * len(cats)),
                       fill=(0, 0, 0, 180), outline=(90, 90, 110))
        for i, c in enumerate(cats):
            y = ly + 22 * i + 8
            draw.ellipse((lx, y - 6, lx + 12, y + 6), fill=_rgb(c))
            draw.text((lx + 20, y - 9), c, fill=(230, 230, 230), font=font)

    # Scale bar, bottom-left — a round length near 1/5 of the image width
    mpp = paths.meters_per_pixel((lat_min + lat_max) / 2, z) / scale
    target = mpp * img.width / 5
    length = min((s for s in (10, 20, 50, 100, 200, 500, 1000, 2000)
                  if s >= target), default=2000)
    bar = length / mpp
    bx, by = 14, img.height - 24
    draw.line((bx, by, bx + bar, by), fill=(255, 255, 255), width=3)
    for end in (bx, bx + bar):
        draw.line((end, by - 6, end, by + 6), fill=(255, 255, 255), width=3)
    draw.text((bx + 4, by - 26), f'{length} m', fill=(255, 255, 255),
              font=font, stroke_width=2, stroke_fill=(0, 0, 0))

    # North arrow, top-left
    draw.polygon([(26, 16), (18, 44), (26, 36), (34, 44)],
                 fill=(255, 255, 255), outline=(0, 0, 0))
    draw.text((20, 46), 'N', fill=(255, 255, 255), font=font,
              stroke_width=2, stroke_fill=(0, 0, 0))

    # Title top-left, attribution bottom-right (clear of the scale bar)
    title = os.path.basename(mission_dir)
    draw.text((46, 14), title, fill=(255, 255, 255), font=_font(20),
              stroke_width=2, stroke_fill=(0, 0, 0))
    attr_font = _font(12)
    attr_w = draw.textlength(ATTRIBUTION, font=attr_font)
    draw.text((img.width - attr_w - 10, img.height - 22), ATTRIBUTION,
              fill=(210, 210, 210), font=attr_font,
              stroke_width=2, stroke_fill=(0, 0, 0))

    out = out or os.path.join(mission_dir, 'report', 'route_map.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img.save(out)
    write_poi_csv(os.path.join(os.path.dirname(out), 'poi.csv'),
                  waypoints, track)
    segments = load_yaml_list(os.path.join(mission_dir, 'segments.yaml'))
    write_geojson(os.path.join(os.path.dirname(out), 'mission.geojson'),
                  mission_geojson(waypoints, track, segments,
                                  meta=dict(meta, mission=os.path.basename(mission_dir))))
    if missing and zooms:
        print(f'warning: {missing} tiles missing (site imagery incomplete)',
              file=sys.stderr)
    return out


def write_poi_csv(path: str, waypoints: List[dict], track) -> str:
    """Report-ready POI table: every waypoint plus the track endpoints."""
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['id', 'label', 'category', 'lat', 'lon', 'alt', 'notes'])
        if track:
            w.writerow(['track_start', 'Track start', 'track',
                        f'{track[0][1]:.8f}', f'{track[0][2]:.8f}', '', ''])
        for wp in waypoints:
            w.writerow([wp.get('id'), wp.get('label'), wp.get('category'),
                        f"{wp['lat']:.8f}", f"{wp['lon']:.8f}",
                        wp.get('alt', ''), wp.get('notes', '')])
        if track:
            w.writerow(['track_end', 'Track end', 'track',
                        f'{track[-1][1]:.8f}', f'{track[-1][2]:.8f}', '', ''])
    return path


def main(argv=None) -> int:
    ref = (argv or sys.argv[1:] or ['last'])[0]
    try:
        out = export_mission(paths.resolve_mission(ref))
    except (FileNotFoundError, ValueError) as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1
    print(out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
