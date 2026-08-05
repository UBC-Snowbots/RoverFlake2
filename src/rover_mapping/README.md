# rover_mapping — Path Recording, Replay & Report Maps

Record a run's GPS track in a rosbag, tag labeled points of interest as you
drive, replay the run over offline satellite imagery in mapviz, and export
a report-ready route map.

**`ros2 bag` is the source of truth; mapviz is just the viewer.** Every run
is one folder under `missions/` holding the bag plus small human-readable
YAML files — nothing hidden in a database or a GUI.

## ROS API (this is the real interface)

One long-running stack (`mapviz_offline.launch.py`) contains the
`mission_manager` node. Missions are driven entirely through its services,
so the HMI or any other node can control them:

| Service          | Type                                     | Does                                        |
|------------------|------------------------------------------|---------------------------------------------|
| `/mission/start` | `rover_mapping_interfaces/StartMission`  | create `missions/<date>_<name>/`, start bag recording (in-process rosbag2) |
| `/mission/stop`  | `std_srvs/Trigger`                       | finalize the bag, close the mission         |
| `/tag_point`     | `rover_mapping_interfaces/TagPoint`      | save current GPS fix as a labeled waypoint  |
| `/segment_start` | `rover_mapping_interfaces/Segment`       | open a named route leg                      |
| `/segment_end`   | `rover_mapping_interfaces/Segment`       | close a leg (empty name = last open)        |

Topics: subscribes `/gnss_fix` (NavSatFix) and `/local_xy_origin`;
publishes `/waypoints` (MarkerArray, latched). Recorded topics:
`/gnss_fix /waypoints /tf /tf_static /local_xy_origin /diagnostics`.

Tags are atomic YAML writes and the bag is finalized on any shutdown path
— a crash or Ctrl-C never loses data. Tagging is rejected (ok=false) when
no mission is active or the fix is missing/stale (>2 s).

## One-time setup

```bash
cd ~/RoverFlake2
colcon build --symlink-install --packages-select \
    rover_mapping_interfaces rover_mapping tile_map
pip install -r src/rover_mapping/requirements.txt   # report exporter only
```

(`tile_map` is built from patched source because the Humble deb is broken —
see PROGRESS.md.)

## The `./rover` CLI (thin sugar over the ROS API)

```bash
cd src/rover_mapping
./rover                              # help
./rover view drumheller              # bring the stack up, leave it up
./rover start run1                   # /mission/start — recording begins
./rover tag site "rock outcrop"      # /tag_point (any terminal)
./rover seg start route_to_site_2    # /segment_start ... /segment_end
./rover stop                         # /mission/stop — bag finalized
./rover ls                           # list missions
./rover replay last 4.0              # replay over imagery (or replay-seg)
./rover export last                  # report/route_map.png
```

`rover` sources ROS + the workspace itself. Missions are addressed by
name, path, or `last`.

### Bench test without hardware

```bash
./rover view                        # terminal 1 (ubc_test imagery)
ros2 run rover_mapping fake_gps     # terminal 2
```

A cyan path grows over the imagery; practice the whole flow indoors.

## Imagery for a new site (e.g. the real Drumheller course)

Right-click the course center in Google Maps, copy the coordinates, then:

```bash
./rover fetch circ 51.4530 -112.7120 1.5   # Esri World Imagery, offline
./rover view circ                          # verify with wifi off
```

`drumheller` (placeholder center) and `ubc_test` are already fetched.
The local-XY origin is anchored at the imagery center, so mapviz has a
valid wgs84↔map transform before the first GPS fix (origin:=auto restores
first-fix anchoring).

## Competition day

1. `./rover view circ` — leave it running.
2. GPS lock: `ros2 topic echo /gnss_fix --once`.
3. `./rover start run1` — drive; `tag start` once, `tag site` at each site,
   `tag sample`, `seg start`/`seg end` per leg. `./rover stop` when done.
4. `./rover replay last` to review, `./rover export last` for the report.

| Competition requirement            | Where it's satisfied                    |
|------------------------------------|-----------------------------------------|
| 3 sites ≥10 m apart, coords logged | `tag site` ×3 → `waypoints.yaml`        |
| Route taken                        | bag `/gnss_fix` → navsat path           |
| Start location                     | `tag start` → green star                |
| Landmarks / obstacles              | `tag landmark` / `tag obstacle`         |
| Sample location                    | `tag sample` → purple X                 |
| Map deliverable                    | `rover export` → `report/route_map.png` |
| Review any leg                     | `rover replay-seg`                      |

Categories/colors everywhere (mapviz + report): `start`=green, `site`=red,
`sample`=purple, `obstacle`=orange, `landmark`=blue.

## What's in a mission folder

```
missions/2026-08-05_run1/
├── rosbag2/          # the bag — source of truth
├── mission.yaml      # name, site, date (replay/export read the site here)
├── waypoints.yaml    # tagged points: id, label, category, lat/lon, stamp
├── segments.yaml     # named legs as timestamp ranges into the bag
├── origin.yaml       # local-XY anchor (offset replays line up exactly)
└── report/route_map.png
```

## Gotchas

- mapviz 2.6.5 has no config-file parameter: the launch renders
  `config/<site>.mvc` and symlinks `~/.mapviz_config` to it. GUI changes
  autosave back into that file — commit keepers.
- GPS topic is `/gnss_fix` (rover_gnss `nmea_reader`); everything takes
  `fix_topic:=...` if that changes.
- Humble's `ros2 bag play` has no `--playback-duration`: leg replay prints
  when to Ctrl-C.
- Esri imagery requires attribution — `rover export` adds it.
- Own GeoTIFFs (e.g. UBC Abacus orthos):
  `imagery_pipeline/scripts/prepare_imagery.sh` — see that README.
