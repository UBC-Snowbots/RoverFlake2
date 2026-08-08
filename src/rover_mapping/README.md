# rover_mapping — GNSS Mission Recording & Report Maps

Record a run's GPS track, tag labeled points of interest as you drive, and
export a report-ready route map. The HMI's **GNSS Mission** module is the
front end (embedded offline map, live path, tag buttons); this package is
the backend. Everything is ROS — no shell scripts.

## ROS API

One node, `mission_manager`, owns the mission lifecycle:

| Service           | Type                                    | Does                                     |
|-------------------|-----------------------------------------|------------------------------------------|
| `/mission/start`  | `rover_mapping_interfaces/StartMission` | create `missions/<date>_<name>/`, start bag + track log |
| `/mission/stop`   | `std_srvs/Trigger`                      | finalize the bag, close the mission      |
| `/tag_point`      | `rover_mapping_interfaces/TagPoint`     | save a labeled waypoint — the current GPS fix, or `manual_lat`/`manual_lon` when `use_manual` |
| `/segment_start`  | `rover_mapping_interfaces/Segment`      | open a named route leg                   |
| `/segment_end`    | `rover_mapping_interfaces/Segment`      | close a leg (empty name = last open)     |
| `/mission/export` | `rover_mapping_interfaces/ExportMap`    | render `report/` (PNG + GeoJSON + CSV)   |

Subscribes `/gnss_fix` (NavSatFix). Publishes the active mission name
latched on `/mission/active` (std_msgs/String, empty = none) so UIs that
join late recover state.

A mission directory is self-describing, human-readable, crash-safe:

```
missions/2026-08-05_field1/
  mission.yaml     name, site, date
  track.csv        stamp,lat,lon,alt (2 Hz, streamed during the run)
  waypoints.yaml   tagged points (atomic writes)
  segments.yaml    named legs as timestamp ranges
  rosbag2/         /gnss_fix bag — the raw source of truth
  report/
    route_map.png
    mission.geojson  every tag as a placed Point + track/segment LineStrings
    poi.csv
```

`mission.geojson` is standard RFC 7946 (coordinates `[lon, lat, alt]`) and
opens directly in QGIS, geojson.io, or Google Earth.

## Data locations & portability

Missions and imagery live in the source tree under `src/rover_mapping/`
(both gitignored). Paths resolve via `$ROVERFLAKE_ROOT` (set in the
container and rover env), `$ROVER_MAPPING_ROOT` to override, or the source
tree itself when built with `--symlink-install`. Dependencies are apt-only:
`python3-pil` and `python3-yaml` (no gdal, no matplotlib, no mapviz).

## Offline imagery for a site

```bash
ros2 run rover_mapping fetch_tiles -- --name circ --center 51.453361 -112.722667 --km 1.5
```

Esri World Imagery, zooms 13–19 (auto-steps below placeholder zooms), into
`imagery/<name>/tiles/{z}/{x}/{y}.jpg` — the layout the HMI map and the
exporter read. Keep the provider's attribution on report maps.

The HMI map composites tiles from **all** sites (z/x/y is a global grid, so
overlapping fetches mesh exactly; missing zooms fall back to a magnified
coarser tile), and its **Fetch tiles** button downloads ~1 km more coverage
around the current view center into the site under the view — existing tiles
are skipped, so repeated fetches just extend the map.

## Bench test without hardware

```bash
ros2 run rover_mapping mission_manager     # terminal 1
ros2 run rover_mapping fake_gps            # terminal 2
```

Then drive it from the HMI's GNSS Mission module, or from a terminal:

```bash
ros2 run rover_mapping tag site "rock outcrop"
ros2 run rover_mapping export_map          # newest mission -> route_map.png
```

## Manual coordinates

Points the rover isn't standing on — a target radioed in by the judges, a
coordinate read off another map — are tagged by coordinate instead of by
fix. They land in `waypoints.yaml` alongside fix tags (marked
`source: manual`), so the report map and POI CSV pick them up for free.

```bash
ros2 run rover_mapping tag site "given target" --at 51.453361 -112.722667
```

In the HMI's **GNSS Mission** module, the lat/lon row under the tag buttons
does the same: type the two numbers (or paste `51.453361, -112.722667` whole
into the lat box), pick a category, hit **Add point**. The map jumps to the
point and draws it as a diamond — fix tags stay circles, so a planned target
never reads as somewhere the rover has been. A manual point with no mission
recording is still plotted, and the status line says it wasn't saved.
Manual points survive a new mission's map reset; **Clear points** drops them.
