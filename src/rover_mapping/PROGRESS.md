# PROGRESS

## Phase status

| Phase | Status | Verified how |
|-------|--------|--------------|
| 1 — Offline imagery | DONE | `imagery_pipeline/` (pruned to the 3 scripts actually used); Esri sets fetched for ubc_test + drumheller; tile server wired into launch |
| 2 — Live path in mapviz | DONE | verified on-screen: tile_map + navsat + marker displays load from the rendered config, fake-GPS path draws over the ubc_test imagery |
| 3 — Record & replay | DONE (ROS-native) | mission lifecycle is services on `mission_manager` (/mission/start, /mission/stop) with in-process rosbag2_py recording; verified end-to-end incl. bag finalization on stop |
| 4 — Waypoint tagger | DONE | folded into `mission_manager`; tags verified live in mapviz (colored spheres + labels, latched); rejected when no mission/fix; bogus category rejected |
| 5 — Sub-path replay | DONE (launch-native) | `replay.launch.py segment:=<name>` computes the bag offset itself (verified: 1.8 s offset, 4.0 s duration) |
| 6 — Report exporter | DONE | route_map.png @300 dpi from a recorded fake run: route, start star, numbered sites, sample X, obstacle, legend, scale bar, north arrow, attribution |
| 7 — Docs | DONE | README rewritten around the single `./rover` CLI (fetch/sites/view/start/tag/seg/ls/replay/replay-seg/export); missions carry a `mission.yaml` so replay/export find their site automatically |

## Verified in-sim (fake_gps, headless)

- unit tests: 11 passed (`pytest src/rover_mapping/test/`)
- `colcon build --symlink-install` + `colcon test`: clean
- tag CLI end-to-end (`tag start/site/sample/obstacle`, `tag seg start/end`)
- waypoints.yaml / segments.yaml / origin.yaml atomic writes, id auto-increment
- /waypoints TRANSIENT_LOCAL latch (late joiner gets full set)
- bag record topic set; segment offset math vs metadata.yaml
- exporter output non-blank, all report elements present

## Needs real hardware

- real `/gnss_fix` from `rover_gnss` nmea_reader (topic name matches)
- marker appearance in mapviz during replay
- Drumheller imagery fetch (`fetch_tiles.py` bbox, zoom 13–19)

## tile_map deb is broken — built from source

`ros-humble-tile-map` 2.6.5 ships a `libtile_map.so` whose Q_OBJECT
vtables/metaObjects were never moc'd (upstream CMake bug: `QT_HEADERS`
listed as sources but no `qt5_wrap_cpp` for the core lib), so mapviz fails
with `undefined symbol: _ZTVN8tile_map12StadiaSourceE`. Fixed by vendoring
`src/external_pkgs/mapviz` (tag 2.6.5) with two local CMake patches in
`tile_map/CMakeLists.txt` (add moc for QT_HEADERS; link OpenGL::GLU) and
building only `tile_map` (siblings have COLCON_IGNORE — debs cover them).
The workspace overlay shadows the broken deb. Re-source after building.

## Field simulation at the real task site (2026-08-05)

Task site 51°27'12.1"N 112°43'21.6"W (= 51.453361, -112.722667) fetched as
site `circ`. Full task flow recorded against it with fake GPS
(mission `2026-08-05_field-sim1`): start → 3 sites on separate legs
(~40 m apart) → obstacle + landmark en route → sample at site 3 →
report/route_map.png. Usability fixes found by doing it: `last` now means
newest by mtime (was alphabetical), missions resolve by bare name
(`field-sim1`), and `rover fetch` probes Esri coverage and steps the max
zoom down past "Map data not yet available" placeholder tiles (this site
tops out at z18; z19 is fake). The stale `drumheller` placeholder set was
removed — `circ` supersedes it.

## Architecture note (2026-08-05 refactor)

Mission control moved from bash orchestration into ROS proper:
`mission_manager` node owns start/stop/tag/segments; bags are recorded
in-process via `rosbag2_py.Recorder` (Humble's Python API — no subprocess).
`./rover` is thin sugar over `ros2 launch` + service calls; only imagery
fetching stays in scripts. The local-XY origin defaults to the imagery
center (slippy-math in `launch_helpers.tiles_center`, unit-tested), which
gives mapviz a valid wgs84↔map transform immediately — this removed the
"Wgs84Transformer not initialized" warning spam.

## Deviations from spec (deliberate)

- `rover_mapping_interfaces` lives at `src/rover_mapping_interfaces/`
  (sibling, not nested) — colcon does not discover packages nested inside
  another package.
- Added `Segment.srv` (spec listed only TagPoint.srv but requires
  /segment_start & /segment_end services with a name).
- Added `origin.yaml` sidecar so segment replay (--start-offset skips the
  bag's one-shot /local_xy_origin message) re-anchors identically.
- Fix topic default is `/gnss_fix` (what rover_gnss actually publishes),
  not `/fix`; configurable via `fix_topic` everywhere.
- mcap storage not used (ros-humble-rosbag2-storage-mcap not installed);
  sqlite3 default works with the exporter's SequentialReader.
