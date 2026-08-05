# CLAUDE.md — Rover Path Recording, Replay & Reporting (ROS 2 Humble)

Implementation spec. The authoritative copy of this spec was provided at
repo root during implementation; this is the package-local copy required by
the repo layout. See PROGRESS.md for phase status and deliberate deviations.

## Environment

- Ubuntu 22.04, ROS 2 Humble, Python 3.10, colcon workspace at `~/RoverFlake2`
- mapviz stack: `ros-humble-mapviz ros-humble-mapviz-plugins
  ros-humble-tile-map ros-humble-swri-transform-util`
- GDAL for imagery tooling: `gdal-bin python3-gdal`
- GPS driver publishes `sensor_msgs/msg/NavSatFix` on `/gnss_fix`
  (`src/rover_gnss/rover_gnss/nmea_reader.py`); `fake_gps` for bench tests
- Bags: sqlite3 (mcap storage plugin not installed)

## Mission

Competition field task: travel to 3 sites of interest (≥10 m apart); at each
site record its GPS coordinates and the route taken. Report deliverable: a
map of the route with the starting location, the 3 sites, other
landmarks/obstacles/features, and where the sample was taken.

Priorities: (1) clear organizational structure of everything saved,
(2) easy to save clearly-labeled distinct points in the field, (3) built on
mapviz for live view and replay, (4) easy to view points and runs afterwards.

## Core design (do not deviate without asking)

**`ros2 bag` is the source of truth; mapviz is the viewer.** One bag per run
plus small human-readable YAML sidecars. Replay = `ros2 bag play` with a
saved mapviz config. Sub-paths = timestamp ranges into the bag. Report map =
rendered offline from the bag + sidecars over the same imagery GeoTIFF.

## Sidecar schemas (stable contract)

`waypoints.yaml`:

```yaml
- id: site_1                # auto: <category>_<n>, unique per run
  label: "Site 1 — rock outcrop"
  category: site            # start|site|sample|obstacle|landmark
  lat: 51.44012
  lon: -112.61234
  alt: 720.4
  stamp: 1754300000.12      # unix epoch (ROS time from fix header)
  notes: ""
```

`segments.yaml`:

```yaml
- name: route_to_site_2
  start_stamp: 1754300100.0
  end_stamp:   1754300420.5
```

Marker color map (mapviz markers AND report map): start=green, site=red,
sample=purple, obstacle=orange, landmark=blue.

## Layout

```
src/rover_mapping/
├── CLAUDE.md / README.md / PROGRESS.md / requirements.txt
├── imagery_pipeline/          # Phase 1, vendored unchanged (ROS-free)
├── rover_mapping/             # ament_python modules
│   ├── mission_manager.py fake_gps.py export_report_map.py
│   ├── tag_cli.py mission_io.py local_xy.py launch_helpers.py
├── launch/  config/  scripts/  test/
└── missions/                  # generated per run (gitignored)
src/rover_mapping_interfaces/  # sibling pkg (colcon can't nest packages)
└── srv/TagPoint.srv  srv/Segment.srv
```

## Conventions & constraints

- Type hints; node loggers, no prints in nodes; small pure geodetic
  functions with pytest coverage (local-XY, id auto-increment, atomic YAML,
  segment offset math).
- Tagger never loses data on Ctrl-C: every tag is a synchronous atomic
  YAML write (temp + rename).
- `/waypoints` published TRANSIENT_LOCAL, full MarkerArray republished on
  every change (late joiners + replay see all markers).
- Shell scripts: `set -euo pipefail`, work from any CWD.
- Deps beyond ROS debs: pip `rasterio matplotlib pyproj pyyaml`
  (exporter only).
- If a mapviz Humble package lacks something, STOP and report options
  rather than silently substituting.
