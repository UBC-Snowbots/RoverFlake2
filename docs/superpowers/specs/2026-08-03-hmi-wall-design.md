# HMI wall: multi-monitor instances, screen placement, button scenes

**Date:** 2026-08-03
**Status:** Approved direction. Builds on 2026-08-02-hmi-layout-launch-cbs-design.md (shipped).
**Target:** CBS control base = Ubuntu + i3 on X11 (3 monitors). Placement is
i3 config keyed on the window title's `[instance]` prefix — no Qt placement code.

## Goal

One launch file opens three `rover_hmi` windows, one per monitor, each with its
own layout. CBS buttons switch whole-wall "scenes" (per-window layouts) live.

## Design decisions (settled with Aaron)

- **Instance pins monitor; layout is free to change.** Each window has an
  `instance` name (`left`/`center`/`right`); the monitor binding is
  instance→output, in i3 config (the CBS WM is i3, a tiling WM that overrides
  app-requested placement — WM rules are the mechanism that actually works,
  and the dev-machine Hyprland equivalent uses the same title prefix). Layout
  JSONs stay portable and carry no screen info.
- **Wall scenes:** one button reconfigures all windows via per-instance topics.
- Single-window usage (empty `instance`) must behave exactly as today.

## 1. rover_hmi additions (rover_hmi_core)

### Param `instance` (string, default "")

- Non-empty: window title becomes `Rover HMI [<instance>] — <layout>`
  (`Rover HMI [<instance>]` before any layout loads). Titles stay unique even
  when two windows show the same layout.
- Non-empty: subscribes `/hmi/<instance>/load_layout` and
  `/hmi/<instance>/show_panels` **in addition to** the global
  `/hmi/load_layout` + `/hmi/show_panels` (global pub still drives every
  window — preserves the bench workflow).
- Empty: exactly current behavior (global topics only, title `Rover HMI — <layout>`).

Node-name uniqueness comes from the launch file (`name='rover_hmi_<instance>'`),
no code needed. There is deliberately NO screen/placement code in Qt: i3
overrides tiled-window placement, so a `screen` param would be dead weight.

## 1b. i3 placement snippet (repo-tracked)

New file `setup_scripts/i3/rover_hmi.conf`, output names to be filled from
`xrandr --listmonitors` on the CBS box:

```
# Pin each HMI instance to a monitor (edit output names for this machine).
# i3 >= 4.20: add to ~/.config/i3/config:  include <repo>/setup_scripts/i3/rover_hmi.conf
# older i3: paste these lines into the config. Reload with $mod+Shift+r.
for_window [title="^Rover HMI \[left\]"]   move to output HDMI-1
for_window [title="^Rover HMI \[center\]"] move to output DP-1
for_window [title="^Rover HMI \[right\]"]  move to output DP-2
```

`for_window` (not `assign`) so the rule also fires on title changes, not only
at map time. Title regex anchors on the `[instance]` prefix, so layout
switches never re-trigger a move (prefix is stable).

## 2. hmi_wall.launch.py (rover_launchers)

Three `rover_hmi` nodes. Launch args, all optional:

```
left:=camera  center:=drive  right:=arm   # layout per instance ("" = default tiling)
```

Each node gets `name='rover_hmi_<instance>'`, params `{instance, layout: <arg>}`.
Existing `hmi.launch.py` untouched. Monitor routing is the i3 snippet's job.

## 3. Router scenes (rover_manager)

`button_layouts` entry format extended, backward compatible. Each entry is:

- bare layout name → publish to global `/hmi/load_layout` (today's behavior), or
- comma-separated `instance:layout` pairs → publish each layout to
  `/hmi/<instance>/load_layout`.

```yaml
hmi_router:
  ros__parameters:
    button_layouts:
      - "left:camera, center:drive, right:telemetry"   # scene
      - "arm"                                          # plain global switch
      - ""                                             # unbound
```

Router creates per-instance publishers lazily on first use and caches them.
Whitespace around pairs/colons is trimmed. A malformed pair (empty instance or
layout) is WARNed once and skipped; the rest of the scene still fires.

## Error handling

- Scene naming a nonexistent instance → messages publish to a topic nobody
  subscribes; harmless (same class as unknown layout names, caught on the HMI
  side when the instance does exist).
- All existing single-window error paths unchanged.

## Verification

- No-hardware: start two instances (`instance:=left`, `instance:=right`); pub to
  `/hmi/left/load_layout` → only left window switches; pub global
  `/hmi/load_layout` → both switch. Titles show `[left]`/`[right]`.
- Router: fake GenericPanel press on a scene button → one String on each
  instance topic (echo both).
- CBS bench (Aaron): fill real output names into the i3 snippet
  (`xrandr --listmonitors`), include it in the i3 config, run
  `hmi_wall.launch.py` — windows land on correct monitors, scene buttons
  reconfigure the wall.

## Out of scope

- Saving screen into layout JSON (rejected: instance owns the monitor).
- Qt-side placement (rejected: i3 overrides it).
- Hyprland windowrules for dev machines (same title-prefix scheme works;
  users add their own).
- Per-instance CBS switch/pot semantics.
- Cost note: 3 processes × all plugins is accepted; hidden camera tiles
  already unsubscribe, so camera bandwidth follows visible panels only.
