# Camera dwindle grid — design

Branch: `feat/aaronrhim/hmi_cam_ui` (worktree from main). 2026-08-08.

## Goal

Manage the camera grid like the panel tiling: move and resize individual camera
cells with the same dwindle algorithm and the same Alt chords, and select which
cameras appear in the grid. Everything else about the Cameras module (single
view, click-to-zoom, subscriptions, liveness) stays as it is.

## Approved decisions

- Same Alt chords, context-dependent: when the Cameras panel is focused and in
  grid view, Alt+Arrow / Alt+Shift+Arrow / Alt+Ctrl+Shift+Arrow / Alt+J act on
  camera cells; otherwise they act on panels as today. A camera-wall monitor
  typically shows only the Cameras panel, so the context switch is natural.
- Selection: keep Shift+1..9 membership toggles, add a browsable picker overlay
  in grid view.
- Approach A: extract one generic dwindle tree core; both the panel tiling and
  the camera grid use it. The old tree code inside `tiling_container.cpp` is
  removed, not kept alongside.
- Click on a grid cell still zooms to single view (unchanged).

## Components

### 1. `dwindle_tree.{h,cpp}` (new, extracted core)

Generic BSP tree over `QWidget*` leaves — the mechanism currently embedded in
`TilingContainer` (`DwindleNode`, `dwindleAdd/Remove/Swap`, `splitAncestor`,
resize, recalc, serialize), with `TilePanel*` generalized to `QWidget*`.

API (one tree instance per user):
- `addLeaf(QWidget* w, QWidget* split_target)` — bisect target leaf, smart
  split (taller box → top/bottom, wider → left/right)
- `removeLeaf(QWidget*)` — collapse parent, sibling takes its box
- `swap(QWidget*, QWidget*)`, `resize(QWidget*, dx, dy)` (ratio step, clamped
  0.1..1.9), `toggleSplit(QWidget*)`
- `nearestInDirection(QWidget*, dx, dy)` — directional focus hit-test
- `recalc(QRect area, int gap)` — walk tree, `setGeometry` each leaf widget
- `serialize(idFor)` / `deserialize(json, widgetFor)` — leaf identity via
  caller-supplied name↔widget mapping
- `leaves()`, `clear()`

`TilingContainer` keeps everything panel-specific (TilePanel visuals, hint
partitioning / `buildHintTree`, drag overlays, sidebar, layout store) and
delegates all tree structure/geometry to a `DwindleTree` member. Its saved
layout JSON format must not change — existing saved layouts keep loading.

### 2. Tiling-op routing hook (`TilingContainer` + `GuiModule`)

New op enum `{Focus, Resize, Swap, ToggleSplit}`. `addPanel()` gains one
optional callback `std::function<bool(TilingOp, int dx, int dy)>`; each Alt
binding first offers the op to the focused panel's callback and stops if it
returns true. `GuiModule` gets a matching virtual (default: none), wired in
`hmi_host.cpp` alongside `toggleCallback()`. No behavior change for modules
that don't opt in.

### 3. `cameras/camera_grid.{h,cpp}` (new widget)

Replaces the fixed 2-column `QGridLayout` page in `CameraModule`. Owns a
`DwindleTree` of the in-grid `CameraViewport` cells.

- Membership changes (`toggleInGrid`, restore) add/remove leaves; adding splits
  the focused cell; never empties (existing rule kept).
- Tracks the focused cell; `CameraViewport` gains a focused flag drawn as a
  border highlight in its existing overlay pass.
- Exposes `focusDirection / swap / resize / toggleSplit` for the module's
  tiling-op callback, and tree (de)serialization for saved state.
- `CameraModule` consumes tiling ops only while grid view is showing.

### 4. `cameras/camera_picker_overlay.{h,cpp}` (new overlay)

Grid-view picker in the same idiom as `CameraListOverlay` / layout manager:
toggled with `P` (module-focused shortcut), rows "N label" with in-grid
checkmark and live/dead dimming, Up/Down navigate, Enter/Space toggle
membership, Esc closes. Shift+1..9 keeps working with the overlay open or
closed. Module keybindings list updated so Alt+/ documents all of this.

### 5. Persistence

`CameraModule::saveState()` adds `"grid_tree"` (camera names at leaves +
split direction + ratio per internal node) next to the existing keys.
`restoreState()`: rebuild tree from names; unknown names dropped, cameras
missing from the tree appended via normal dwindle add; absent `"grid_tree"`
(old saved layouts) → default tree from membership list.

## Testing

- New `test_dwindle_tree.cpp` beside `test_camera_config.cpp`: pure-geometry
  unit tests — add/remove/swap/resize invariants, serialize round-trip,
  collapse-on-remove. Runs headless (offscreen QWidget, no display needed).
- Build + tests via the rover Docker container.
- Panel tiling regression: existing saved layouts load unchanged; Alt chords
  unchanged outside the camera grid. Verified by inspection + Aaron's bench
  pass (visual behavior has no automated coverage).

## Out of scope

Single-view behavior, subscription/QoS logic, camera_map.json format, ROS
interfaces, drag modes (Alt+Z/X stay panel-level for now).
