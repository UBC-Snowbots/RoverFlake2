# HMI Core Consolidation — Design

**Date:** 2026-05-20
**Scope:** Fold every Qt5 `GuiModule` plugin into `rover_hmi_core`. Delete duplicate plugin packages. Leave legacy GTK `rover_hmi` untouched.

## Background

The HMI plugin system is sprawled across five packages:

- **`rover_hmi_core/`** — hub. Has the host executable (`rover_hmi`), the `GuiModule` base class, `TilingContainer`, `PlotWidget`, and 14 modules (arm/drive/science/electrical). Plugins registered under `rover_hmi_core/*`.
- **`rover_drive_hmi/`** — single `DriveModule`. Registered under `rover_drive_hmi/DriveModule`. **Code is a layout-only variant** of the canonical copy in `rover_hmi_core/src/drive/drive_module.cpp` (external version uses a separate Q_OBJECT header; core version inlines it for AUTOMOC with private include dirs).
- **`rover_science_hmi/`** — single `ScienceModule`. Registered under `rover_science_hmi/ScienceModule`. **Code is byte-identical** to `rover_hmi_core/src/science/science_module.cpp`.
- **`arm_hardware_interface/`** — 5 modules (motor_status, plotting, motor_config, send_command, command_log) registered under `arm_hardware_interface/*`. Four files byte-identical to the `rover_hmi_core/src/arm/*` canonical copies; `send_command_module.cpp` is the same layout-only Q_OBJECT variant as drive. The package also contains `moteus_driver_node` (non-HMI, stays).
- **`dev_arm_moveit_config_v3/`** — single `RvizModule` (embedded RViz). Registered under `dev_arm_moveit_config_v3/RvizModule`. **Not duplicated** in `rover_hmi_core`. The package also contains MoveIt config and launch files (stays).

Because pluginlib loads every registered class it finds in the ament index, the current state causes every duplicated module to load **twice** at HMI startup (e.g. two `MotorStatusModule` panels in the sidebar). This is a latent bug fixed as a side effect of consolidation.

The legacy GTK package `src/rover_hmi/` (`gtkmm-3.0`, 9700+ LOC of glade/CSS/cpp, executables `main_hmi_node`/`dashboard_hmi_node`/`science_module_hmi_node`, used by 3 launchers in `rover_launchers/`) is **out of scope** for this consolidation.

## Goal

Every `GuiModule` plugin lives in `rover_hmi_core/`. Each module loads exactly once. The five external packages either disappear (drive/science HMI shells) or shed their HMI scaffolding (arm hardware interface, moveit config).

## Final layout

```
src/
├── rover_hmi_core/                    # one hub, one .so for plugins
│   ├── plugins.xml                    # every GuiModule registered here
│   ├── src/
│   │   ├── hmi_host.cpp               # unchanged
│   │   ├── plot_widget.cpp            # unchanged
│   │   ├── tiling_container.cpp       # unchanged
│   │   ├── arm/                       # unchanged (canonical copies stay)
│   │   ├── drive/                     # unchanged (canonical copies stay)
│   │   ├── science/                   # unchanged (canonical copies stay)
│   │   ├── electrical/                # unchanged (canonical copies stay)
│   │   └── moveit/                    # NEW — holds rviz_module.cpp
│   └── include/rover_hmi_core/
│       └── moveit/                    # NEW — holds rviz_module.h
│
├── arm_hardware_interface/            # keeps moteus_driver only
│   ├── plugins.xml                    # DELETE
│   ├── CMakeLists.txt                 # prune HMI library + Qt5/rover_hmi_core/pluginlib deps
│   ├── package.xml                    # drop rover_hmi_core, pluginlib, Qt5 deps
│   ├── src/
│   │   ├── moteus_driver_node.cpp     # keep
│   │   └── *_module.cpp               # DELETE all 5
│   └── include/
│       ├── arm_hardware_interface/    # keep
│       ├── moteus_driver_node.h       # keep
│       ├── moteus_protocol.h          # keep
│       ├── axis_5_6_differential.h    # keep
│       └── *_module.h                 # DELETE all 5
│
├── dev_arm_moveit_config_v3/          # keeps launch/, config/, README
│   ├── plugins.xml                    # DELETE
│   ├── CMakeLists.txt                 # prune HMI library + Qt5/rover_hmi_core/pluginlib deps
│   ├── package.xml                    # drop rover_hmi_core, pluginlib, Qt5 deps
│   ├── src/rviz_module.cpp            # MOVE to rover_hmi_core/src/moveit/
│   └── include/rviz_module.h          # MOVE to rover_hmi_core/include/rover_hmi_core/moveit/
│
├── rover_drive_hmi/                   # DELETE entire package
└── rover_science_hmi/                 # DELETE entire package
```

## `plugins.xml` consolidation

The canonical copies of all duplicated modules already exist in `rover_hmi_core/plugins.xml` under the `rover_hmi_core/*` class name. After consolidation, only the `rover_hmi_core/*` registrations remain. The only addition is one new entry for the moveit RViz panel:

```xml
<!-- MoveIt embedded view -->
<class name="rover_hmi_core/RvizModule"
       type="RvizModule"
       base_class_type="rover_hmi_core::GuiModule">
  <description>Embedded RViz 3D robot visualization</description>
</class>
```

The class string change (`dev_arm_moveit_config_v3/RvizModule` → `rover_hmi_core/RvizModule`) has no observable effect: the HMI host iterates whatever pluginlib finds, and the sidebar tile label comes from the module's `name()` override, not the class string.

The external `plugins.xml` files in `rover_drive_hmi/`, `rover_science_hmi/`, `arm_hardware_interface/`, and `dev_arm_moveit_config_v3/` are deleted along with the rest of their HMI scaffolding.

## CMakeLists & package.xml changes

### `rover_hmi_core/`

`CMakeLists.txt`:
1. Add `find_package(ament_index_cpp REQUIRED)` (used by `rviz_module.cpp` for package-share resolution).
2. Add `find_package` calls for the exact RViz deps that `rviz_module.cpp` references — the implementation plan will read the file and pin the list (likely `rviz_common`, `rviz_default_plugins`, `rviz_rendering`).
3. Append `src/moveit/rviz_module.cpp` and `include/rover_hmi_core/moveit/rviz_module.h` to `add_library(rover_hmi_modules SHARED ...)`.
4. Add `include/rover_hmi_core/moveit` to the `target_include_directories(rover_hmi_modules PRIVATE ...)` block.
5. Add `ament_index_cpp` and the rviz deps to `ament_target_dependencies(rover_hmi_modules ...)`.

`package.xml`: add `<depend>ament_index_cpp</depend>` plus the rviz `<depend>`s confirmed in step 2.

### `arm_hardware_interface/`

`CMakeLists.txt` — remove:
- `find_package(rover_hmi_core REQUIRED)`, `find_package(pluginlib REQUIRED)`, `find_package(Qt5 REQUIRED COMPONENTS Widgets)`
- `set(CMAKE_AUTOMOC ON)`
- The `add_library(arm_hardware_interface_hmi SHARED ...)` block and its `target_link_libraries` / `ament_target_dependencies`
- `pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)`
- `install(TARGETS arm_hardware_interface_hmi ...)`
- The `rover_hmi_core` entry in `ament_target_dependencies(moteus_driver ...)` — verify by grep during implementation that `moteus_driver_node.cpp` does not include any HMI headers; remove the dep if confirmed unused.

`package.xml`: drop `<depend>rover_hmi_core</depend>`, `<depend>pluginlib</depend>`, and any Qt5/qt deps.

### `dev_arm_moveit_config_v3/`

`CMakeLists.txt` — remove:
- `find_package(rover_hmi_core REQUIRED)`, `find_package(pluginlib REQUIRED)`, `find_package(ament_index_cpp REQUIRED)`, `find_package(Qt5 REQUIRED COMPONENTS Widgets)`
- `set(CMAKE_AUTOMOC ON)`
- `include_directories(include)` (no longer relevant once the only thing under `include/` is moved out)
- The `add_library(dev_arm_moveit_config_v3_hmi SHARED ...)` block, its link/dependency rules, and `pluginlib_export_plugin_description_file(...)`.
- `install(TARGETS dev_arm_moveit_config_v3_hmi ...)`

`package.xml`: drop `<depend>rover_hmi_core</depend>`, `<depend>pluginlib</depend>`, `<depend>ament_index_cpp</depend>`, any Qt5 deps.

### `rover_drive_hmi/` and `rover_science_hmi/`

`git rm -r` both directories.

## Verification

Each step must pass before the next:

1. `colcon build --packages-select rover_hmi_core` — moveit RViz module compiles inside the core library; appended `plugins.xml` entry is well-formed.
2. `colcon build --packages-select arm_hardware_interface` — moteus driver still builds with Qt5/pluginlib/rover_hmi_core deps removed.
3. `colcon build --packages-select dev_arm_moveit_config_v3` — package still builds (mostly launch/config now).
4. Full clean build: `rm -rf build install log && colcon build` — catches any remaining `find_package(rover_drive_hmi)` / `find_package(rover_science_hmi)` references anywhere in the tree.
5. Runtime smoke test: `ros2 run rover_hmi_core rover_hmi` — every module appears **exactly once** in the sidebar; the RViz panel renders the robot.
6. `ros2 pkg list | grep -iE 'rover_drive_hmi|rover_science_hmi'` returns nothing.

### Edge cases & risks

- **Stale `install/` artifacts**: pluginlib reads its registry from the ament index in `install/`. Deleted packages can leave stale entries. The implementation plan starts with `rm -rf build install log` to force a clean state before any verification.
- **External consumers of `rover_drive_hmi` / `rover_science_hmi`**: grepped — none. Only the launchers in `rover_launchers/` reference `rover_hmi` (legacy GTK, out of scope).
- **`moteus_driver` leftover dep**: confirm-by-grep step during implementation; drop if unused.
- **Exact RViz dep list**: read `rviz_module.cpp` to pin the include set before editing CMakeLists.
- **Git history**: use `git mv` for the rviz module (preserves history); `git rm -r` for deleted packages.

### Out of scope

- Legacy GTK `src/rover_hmi/` — left untouched. Its 3 launchers in `rover_launchers/` continue to work.
- The 6 unfinished tasks/work from the prior drive-hookup branch are unrelated.
- Any module-level refactoring (file splits, renames, etc.) beyond moving and CMake hygiene.

## Affected files

| File | Action |
|---|---|
| `src/rover_hmi_core/plugins.xml` | append RvizModule entry |
| `src/rover_hmi_core/CMakeLists.txt` | add deps + moveit sources to `rover_hmi_modules` |
| `src/rover_hmi_core/package.xml` | add `ament_index_cpp` + rviz deps |
| `src/rover_hmi_core/src/moveit/rviz_module.cpp` | NEW (git mv from `dev_arm_moveit_config_v3/src/`) |
| `src/rover_hmi_core/include/rover_hmi_core/moveit/rviz_module.h` | NEW (git mv from `dev_arm_moveit_config_v3/include/`) |
| `src/arm_hardware_interface/CMakeLists.txt` | strip HMI library + HMI deps |
| `src/arm_hardware_interface/package.xml` | drop HMI deps |
| `src/arm_hardware_interface/plugins.xml` | DELETE |
| `src/arm_hardware_interface/src/*_module.cpp` (5 files) | DELETE |
| `src/arm_hardware_interface/include/*_module.h` (5 files) | DELETE |
| `src/dev_arm_moveit_config_v3/CMakeLists.txt` | strip HMI library + HMI deps |
| `src/dev_arm_moveit_config_v3/package.xml` | drop HMI deps |
| `src/dev_arm_moveit_config_v3/plugins.xml` | DELETE |
| `src/rover_drive_hmi/` | DELETE entire directory |
| `src/rover_science_hmi/` | DELETE entire directory |
