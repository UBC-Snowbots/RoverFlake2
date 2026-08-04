# Wall snapshots: save/restore all HMI windows as one named wall

**Date:** 2026-08-03
**Status:** Approved direction. Builds on 2026-08-03-hmi-wall-design.md (shipped).

## Goal

Arrange the three wall windows by hand, press save-wall once in any window →
every window persists its own state under one wall name. Reopen the whole wall
with `hmi_wall.launch.py wall:=<name>`, or live via `/hmi/wall/load` (so CBS
buttons can eventually switch walls). No YAML editing, no per-window flags.

## Design decisions (settled with Aaron)

- Identity comes from the existing `instance` param — wall features are active
  only when it is non-empty. A plain single-window HMI ignores wall topics.
- Save-then-rename pattern, matching single-layout save: save-wall auto-names
  by timestamp (`yyyy-MM-dd HH:mm`), rename later in the overlay.
- Storage: one directory per wall, one file per instance —
  `config/layouts/walls/<wall-slug>/<instance>.json` — repo-tracked (git-synced)
  like layouts. Per-instance files mean three processes never write one file.

## Mechanism

```
any window: press W in Alt+P ──publish──► /hmi/wall/save  "2026-08-03 21:40"
                                              │ (every instance, incl. sender)
                                              ▼
                              write own state → walls/<slug>/<instance>.json

launch wall:=<name>  ──or──  /hmi/wall/load "<name>"
                                              │ (every instance)
                                              ▼
                              read walls/<slug>/<own instance>.json → apply
```

Missing per-instance file on load → WARN, window keeps its current state.

## 1. LayoutStore wall methods (rover_hmi_core)

Same class/file (dir resolution is shared). New:

- `struct WallEntry { QString name; QString dir_path; QString saved_at; }`
- `std::vector<WallEntry> listWalls() const` — subdirs of `<layouts>/walls/`,
  name read from any member file's `"wall"` field (fallback: dir name),
  sorted by saved_at.
- `bool saveWallInstance(const QString& wall_name, const QString& instance, const QJsonObject& layout)`
  — creates `walls/<slug-of-name>/`, writes `<instance>.json`
  (`{wall, saved_at, tree, visible}`).
- `QJsonObject loadWallInstance(const QString& wall_name, const QString& instance) const`
  — empty object if absent.
- `bool removeWall(const QString& wall_name)`, `bool renameWall(const QString& old_name, const QString& new_name)`
  — operate on the directory (rename rewrites each member file's `"wall"` field).

## 2. TilingContainer: state JSON + wall hooks + overlay section

- Refactor: extract `QJsonObject currentLayoutJson() const` (tree + visible,
  no name) from `saveCurrentLayout`, and `void applyLayoutJson(const QJsonObject&)`
  from `loadLayout(int)` — both existing bodies become thin wrappers. External
  wall code uses these; single-layout behavior unchanged.
- New public hooks, set by the host (all null in single-window mode):
  - `std::function<void(const QString&)> onWallSaveRequested` — overlay's W key
    publishes the (timestamp) name via this.
  - `std::function<void(const QString&)> onWallLoadRequested` — overlay Enter
    on a wall entry publishes via this.
  - When both are null, the overlay renders no WALLS section (single-window
    UX untouched).
- `LayoutManagerOverlay` gains a WALLS section below the layouts list:
  - listing from `layoutStore().listWalls()`; arrow keys move through both
    sections continuously.
  - `Enter` on wall → `onWallLoadRequested(name)`. `D` → `removeWall`.
    `R` → rename (existing rename editor, routed to `renameWall`).
  - `W` (anywhere in overlay, wall mode only) → `onWallSaveRequested(timestamp)`;
    overlay refreshes after save (small delay not needed — own write is
    synchronous; other instances' files appear by next refresh).
  - Keybindings cheatsheet (Alt+/) gains the W entry when wall mode is on.
- `applyLayoutJson` does NOT invoke `onLayoutChanged` — naming is the caller's
  job. `loadLayout(int)` keeps firing it with the layout entry's name (behavior
  unchanged); the wall-load path in the host fires it with the wall name (§3).

## 3. hmi_host wiring

Only when `instance` is non-empty:

- Publishers: `/hmi/wall/save`, `/hmi/wall/load` (`std_msgs/String`).
  Hooks: `onWallSaveRequested` publishes name on `/hmi/wall/save`;
  `onWallLoadRequested` publishes on `/hmi/wall/load` (self included — DDS
  loops the message back to the sender's own subscription; one code path).
- Subscriptions:
  - `/hmi/wall/save` → `store.saveWallInstance(name, instance, tiling->currentLayoutJson())`;
    log INFO with the written path.
  - `/hmi/wall/load` → `json = store.loadWallInstance(name, instance)`;
    empty → WARN (list known walls); else `tiling->applyLayoutJson(json)` and
    set title to `Rover HMI [<instance>] — <wall-name>`.
- Param `wall` (string, default ""): startup wall load, same code path as the
  topic. Precedence: `wall` > `layout` > `panels`.

## 4. hmi_wall.launch.py

New launch arg `wall` (default ""), passed as the `wall` param to all three
instances. Result: `ros2 launch rover_launchers hmi_wall.launch.py wall:=ops`
= the whole ensemble, no other flags.

## Error handling

- Save with layouts dir unresolved (read-only store): existing
  `statusMessage()` WARN path; nothing written.
- `/hmi/wall/load` unknown wall or missing instance file: WARN listing
  `listWalls()` names; state unchanged.
- Slug collisions (two walls whose names slug identically): second save lands
  in the same dir — same behavior class as layout save today; acceptable.
- Wall topics received by an empty-instance HMI: not subscribed (wall subs
  only created when instance set) — inert by construction.

## Verification

- Two instances on the dev box, arrange differently, W in one → two files
  under `walls/<ts>/`. Relaunch with `wall:=<ts>` → both restored. `/hmi/wall/load`
  live-switches both. Rename + delete from the overlay.
- Single-window HMI: Alt+P shows no WALLS section; wall topics absent
  (`ros2 topic list`).
- CBS box (Aaron): full 3-monitor loop — arrange, save, relaunch, restore.

## Out of scope

- Button map `wall:<name>` entries for the router (natural follow-up; the
  `/hmi/wall/load` topic already makes it a ~5-line router change later).
- Cross-machine wall portability beyond git (files are already repo-tracked).
- Capturing window geometry/monitor (i3 owns placement; instance pins monitor).
