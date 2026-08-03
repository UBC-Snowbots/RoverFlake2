# HMI deployment layouts: launch args, live switching, CBS buttons

**Date:** 2026-08-02
**Status:** Approved approach (A) — single always-running HMI, topic-driven switching, router node for CBS buttons.

## Goal

Deploy-time ergonomics for `rover_hmi` (rover_hmi_core):

1. Launch files can start the HMI in a named saved layout, or with an explicit set of panels open.
2. The window title reflects the active layout.
3. The 8 CBS buttons switch layouts live (1 button = 1 layout), without restarting anything.

The HMI stays a single process. Layout switching is always the existing
`TilingContainer` load path; launch args and CBS buttons are just two callers of it.

## Architecture

```
launch arg (layout / panels)  ──► rover_hmi params ──┐
                                                     ├──► TilingContainer::loadLayoutByName / showPanels
/cbs/left_panel_a ──► hmi_router ──► /hmi/load_layout ┘         (window title follows)
        (GenericPanel)   (edge detect + YAML map)  (std_msgs/String)
```

The HMI never learns what a CBS is. Anything that can publish a string
(terminal, router, future tooling) can drive it.

## 1. HMI control surface (rover_hmi_core)

### Startup parameters (read in `hmi_host.cpp` after `finalize()`, before `show`)

- `layout` (string, default ""): name of a saved layout in `config/layouts/`.
  Loaded via the new by-name path. Unknown name → WARN log, default tiling.
- `panels` (string array, default []): panel titles to show, replacing the
  default-visible set. Tree is auto-built (dwindle order), no saved tree needed.
  Unknown titles → WARN per title, rest still shown. `layout` wins if both set.

### Runtime topics (subscribed on the existing shared node)

- `/hmi/load_layout` (`std_msgs/String`): saved-layout name → `loadLayoutByName()`.
- `/hmi/show_panels` (`std_msgs/String`): comma-separated panel titles → `showPanels()`.

Callbacks run inside the Qt event loop already (the 20 ms `spin_some` timer),
so they call TilingContainer directly — no queuing/threading needed.

### TilingContainer additions

- `bool loadLayoutByName(const QString& name)` — resolves name → entry via
  `LayoutStore::list()`, delegates to the existing `loadLayout` body.
  Buttons/launch bind to *names*, never indices (indices shift on save).
- `void showPanels(const std::vector<std::string>& titles)` — sets visibility
  like `loadLayout` does (silent `syncCheckboxes`, same as today), clears the
  tree, rebuilds it from the visible panels using the same partition logic
  `finalize()` uses (extract that into a shared helper rather than duplicating).
- `std::function<void(QString)> onLayoutChanged` — invoked with the layout name
  (or a "Panel1 + Panel2" style string for `showPanels`) after every successful
  load, including Alt+P loads. `hmi_host` sets it to update the window title:
  `Rover HMI — <name>`. Default title stays `Rover HMI`.

## 2. Launch files (rover_launchers)

One parameterized file, `hmi.launch.py`:

```
ros2 launch rover_launchers hmi.launch.py layout:=science
ros2 launch rover_launchers hmi.launch.py panels:="Spectrometer,Cameras"
```

Declares `layout` and `panels` launch args, passes them as node parameters.
Optional per-layout wrappers (`hmi_science.launch.py`, …) are 5-line includes
with a fixed `layout` value — added only when actually wanted, since the arg
form already covers it.

## 3. Button router (rover_manager)

Flesh out the existing `router_A.cpp` skeleton (its stated purpose):

- Subscribes `/cbs/left_panel_a` (`rover_msgs/GenericPanel`).
- Rising-edge detection per button: previous-state array, fire on 0→1 only.
  The panel streams state continuously, so edge detection is the whole
  debounce story at ROS level (firmware already handles contact bounce).
- On edge of button *i*: look up `buttons[i]` in the map; if a layout name is
  bound, publish it on `/hmi/load_layout`. Unbound → ignore silently.
- Map lives in a repo-tracked YAML param file,
  `src/rover_manager/config/cbs_button_map.yaml`:

```yaml
hmi_router:
  ros__parameters:
    button_layouts: ["drive", "arm", "science", "", "", ""]  # index = button
```

Remapping buttons = edit YAML, restart router. No code changes.
The left panel exposes 6 buttons today (`$gen_left_A` has 6); the map is
sized by the message's `num_buttons`, so an 8-button panel needs only a
longer YAML list.

## 4. Layout naming

Saved layouts default to timestamp names (`2026-08-02 10:12`). Button/launch
binding requires stable names: rename via Alt+P to `drive`, `arm`, `science`,
etc. Name matching is exact (case-sensitive) — keep names lowercase-simple.

## Error handling

- Unknown layout name (param or topic): WARN log with the list of known names;
  HMI state unchanged.
- Unknown panel title in `panels`: WARN per title, show the rest.
- `LayoutStore` unresolved (read-only store): existing `statusMessage()`
  behavior; `layout` param WARNs and falls back to default tiling.
- Router with missing/short YAML list: buttons beyond the list are unbound.

## Testing

All testable without CBS hardware:

- `ros2 launch … layout:=science` → correct panels + title.
- `ros2 topic pub --once /hmi/load_layout std_msgs/String "{data: science}"`
  → live switch + title change.
- Same for `/hmi/show_panels` with a panel list.
- Router: `ros2 topic pub` a fake `GenericPanel` sequence (0→1→1→0) and verify
  exactly one `/hmi/load_layout` message per press.
- Hardware pass (Aaron, on the bench): real CBS presses end-to-end.

## Out of scope

- Multiple HMI windows / process-per-layout (rejected).
- CBS switches (5 state-carrying inputs) — separate future mapping.
- Routing buttons to non-HMI actions — the router/YAML shape allows it later.
- Firing module toggle callbacks on layout load (existing silent behavior kept).
