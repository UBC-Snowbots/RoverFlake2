# HMI Core Consolidation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fold every Qt5 `GuiModule` plugin into `rover_hmi_core`. Delete duplicate plugin packages (`rover_drive_hmi`, `rover_science_hmi`). Strip HMI scaffolding from `arm_hardware_interface` and `dev_arm_moveit_config_v3` while keeping their non-HMI code. Legacy GTK `rover_hmi` is out of scope.

**Architecture:** Pure restructuring — no semantic code changes. The canonical copy of every duplicated module already lives in `rover_hmi_core/src/{arm,drive,science,electrical}/`; the only file that physically moves is `rviz_module.{cpp,h}` from `dev_arm_moveit_config_v3` into a new `rover_hmi_core/src/moveit/` directory. The other external packages just shed their HMI plumbing (libraries, plugins.xml, Qt5/rover_hmi_core/pluginlib deps).

**Tech Stack:** ROS 2 (rclcpp + pluginlib), C++17, Qt5, colcon, git.

**Note on testing:** Per the spec, no automated tests are added — this is restructuring, not behavior change. Verification is `colcon build` (per-package and full clean build) plus a runtime smoke test (`ros2 run rover_hmi_core rover_hmi` and confirming each module appears exactly once in the sidebar).

**Spec:** `docs/superpowers/specs/2026-05-20-hmi-core-consolidation-design.md`

---

## File Structure

| File / directory | Action |
|---|---|
| `src/rover_hmi_core/plugins.xml` | append RvizModule entry |
| `src/rover_hmi_core/CMakeLists.txt` | add `ament_index_cpp` find_package, append `src/moveit/rviz_module.cpp` to `rover_hmi_modules` library, add `include/rover_hmi_core/moveit` to private include dirs, add `ament_index_cpp` to `ament_target_dependencies` |
| `src/rover_hmi_core/package.xml` | add `<depend>ament_index_cpp</depend>` |
| `src/rover_hmi_core/src/moveit/rviz_module.cpp` | NEW (`git mv` from `dev_arm_moveit_config_v3/src/`) |
| `src/rover_hmi_core/include/rover_hmi_core/moveit/rviz_module.h` | NEW (`git mv` from `dev_arm_moveit_config_v3/include/`) |
| `src/dev_arm_moveit_config_v3/plugins.xml` | DELETE |
| `src/dev_arm_moveit_config_v3/CMakeLists.txt` | strip HMI library + HMI find_packages + AUTOMOC |
| `src/dev_arm_moveit_config_v3/package.xml` | drop `rover_hmi_core`, `pluginlib`, `ament_index_cpp` deps |
| `src/arm_hardware_interface/src/*_module.cpp` (5 files) | DELETE |
| `src/arm_hardware_interface/include/*_module.h` (5 files) | DELETE |
| `src/arm_hardware_interface/plugins.xml` | DELETE |
| `src/arm_hardware_interface/CMakeLists.txt` | strip HMI library + HMI find_packages + AUTOMOC; drop unused `rover_hmi_core` from `moteus_driver` deps |
| `src/arm_hardware_interface/package.xml` | drop `rover_hmi_core`, `pluginlib` deps |
| `src/rover_drive_hmi/` | DELETE entire directory |
| `src/rover_science_hmi/` | DELETE entire directory |

Total: ~13 files modified, ~14 files deleted, 2 files moved.

---

## Task 1: Move `RvizModule` into `rover_hmi_core`

Pull the RViz embed plugin into the hub package. After this task it lives in `rover_hmi_core` and is registered there, but `dev_arm_moveit_config_v3` still has its now-orphaned plugins.xml and CMake scaffolding (cleaned up in Task 2).

**Files:**
- Create: `src/rover_hmi_core/src/moveit/rviz_module.cpp` (via `git mv`)
- Create: `src/rover_hmi_core/include/rover_hmi_core/moveit/rviz_module.h` (via `git mv`)
- Modify: `src/rover_hmi_core/plugins.xml`
- Modify: `src/rover_hmi_core/CMakeLists.txt`
- Modify: `src/rover_hmi_core/package.xml`

- [ ] **Step 1: Create the moveit subdirectories**

```bash
mkdir -p src/rover_hmi_core/src/moveit
mkdir -p src/rover_hmi_core/include/rover_hmi_core/moveit
```

- [ ] **Step 2: Move the rviz module files preserving git history**

```bash
git mv src/dev_arm_moveit_config_v3/src/rviz_module.cpp \
       src/rover_hmi_core/src/moveit/rviz_module.cpp
git mv src/dev_arm_moveit_config_v3/include/rviz_module.h \
       src/rover_hmi_core/include/rover_hmi_core/moveit/rviz_module.h
```

- [ ] **Step 3: Append RvizModule entry to `rover_hmi_core/plugins.xml`**

Open `src/rover_hmi_core/plugins.xml`. Find the closing `</library>` tag (last line before the closing `</library>`). Insert this block immediately before it:

```xml
  <!-- MoveIt -->
  <class name="rover_hmi_core/RvizModule"
         type="RvizModule"
         base_class_type="rover_hmi_core::GuiModule">
    <description>Embedded RViz 3D robot visualization</description>
  </class>
```

- [ ] **Step 4: Update `rover_hmi_core/CMakeLists.txt`**

Open `src/rover_hmi_core/CMakeLists.txt`. Find the existing `find_package` block:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
```

Insert `find_package(ament_index_cpp REQUIRED)` immediately after `find_package(geometry_msgs REQUIRED)`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
```

Then find the `add_library(rover_hmi_modules SHARED ...)` block. Append `src/moveit/rviz_module.cpp` after the existing Drive line:

```cmake
add_library(rover_hmi_modules SHARED
    # Arm
    src/arm/motor_status_module.cpp
    src/arm/motor_config_module.cpp
    src/arm/plotting_module.cpp
    src/arm/send_command_module.cpp
    src/arm/command_log_module.cpp
    # Science
    src/science/science_module.cpp
    src/science/science_pipeline_module.cpp
    src/science/science_analysis_module.cpp
    # Electrical
    src/electrical/power_summary_module.cpp
    src/electrical/drivetrain_stop_module.cpp
    src/electrical/wheel_telemetry_module.cpp
    src/electrical/lighting_module.cpp
    src/electrical/vaw_tree_module.cpp
    src/electrical/power_timeline_module.cpp
    # Drive
    src/drive/drive_module.cpp
    # MoveIt
    src/moveit/rviz_module.cpp
)
```

Find the `target_include_directories(rover_hmi_modules PRIVATE ...)` block and append `include/rover_hmi_core/moveit`:

```cmake
target_include_directories(rover_hmi_modules PRIVATE
    include/rover_hmi_core/arm
    include/rover_hmi_core/science
    include/rover_hmi_core/electrical
    include/rover_hmi_core/drive
    include/rover_hmi_core/moveit
)
```

Find the `ament_target_dependencies(rover_hmi_modules ...)` line and add `ament_index_cpp`:

```cmake
ament_target_dependencies(rover_hmi_modules rclcpp rover_msgs std_msgs sensor_msgs geometry_msgs pluginlib ament_index_cpp)
```

- [ ] **Step 5: Update `rover_hmi_core/package.xml`**

Open `src/rover_hmi_core/package.xml`. Find the existing `<depend>` block. Insert `<depend>ament_index_cpp</depend>` after `<depend>geometry_msgs</depend>`:

```xml
  <depend>rclcpp</depend>
  <depend>pluginlib</depend>
  <depend>rover_msgs</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>ament_index_cpp</depend>
```

- [ ] **Step 6: Build rover_hmi_core**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select rover_hmi_core --symlink-install 2>&1"
```

Expected: build succeeds. The pre-existing `Failed to extract project name from 'CMakeLists.txt'` warning at the workspace root is unrelated — ignore it.

If the build fails with `rviz_module.h not found`: the include directory `include/rover_hmi_core/moveit` is missing from `target_include_directories`. Re-check Step 4.

If it fails with an `ament_index_cpp` undefined symbol: the `ament_target_dependencies` update in Step 4 didn't land.

- [ ] **Step 7: Commit**

```bash
git add src/rover_hmi_core/plugins.xml \
        src/rover_hmi_core/CMakeLists.txt \
        src/rover_hmi_core/package.xml \
        src/rover_hmi_core/src/moveit/rviz_module.cpp \
        src/rover_hmi_core/include/rover_hmi_core/moveit/rviz_module.h \
        src/dev_arm_moveit_config_v3/src/rviz_module.cpp \
        src/dev_arm_moveit_config_v3/include/rviz_module.h
git commit -m "refactor(rover_hmi_core): absorb RvizModule from dev_arm_moveit_config_v3

Move rviz_module.{cpp,h} into rover_hmi_core/src/moveit/ and register
the RvizModule class under rover_hmi_core/RvizModule. dev_arm_moveit_
config_v3 still has stale plugin scaffolding — cleaned up in next
commit."
```

---

## Task 2: Strip HMI scaffolding from `dev_arm_moveit_config_v3`

Now the package becomes a pure MoveIt config — launches, configs, README, package metadata. No more HMI library, no Qt5 dep, no pluginlib export.

**Files:**
- Delete: `src/dev_arm_moveit_config_v3/plugins.xml`
- Modify: `src/dev_arm_moveit_config_v3/CMakeLists.txt`
- Modify: `src/dev_arm_moveit_config_v3/package.xml`

- [ ] **Step 1: Delete the plugins descriptor**

```bash
git rm src/dev_arm_moveit_config_v3/plugins.xml
```

- [ ] **Step 2: Rewrite `dev_arm_moveit_config_v3/CMakeLists.txt`**

Replace the entire contents of `src/dev_arm_moveit_config_v3/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.22)
project(dev_arm_moveit_config_v3)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
```

This removes: the `rover_hmi_core` / `pluginlib` / `ament_index_cpp` / `Qt5` find_packages, `set(CMAKE_AUTOMOC ON)`, `include_directories(include)`, the entire `add_library(dev_arm_moveit_config_v3_hmi SHARED ...)` block and its `target_link_libraries` / `ament_target_dependencies`, `pluginlib_export_plugin_description_file(...)`, and `install(TARGETS dev_arm_moveit_config_v3_hmi ...)`. The remaining lines preserve the launch/config install rules verbatim.

- [ ] **Step 3: Edit `dev_arm_moveit_config_v3/package.xml`**

Remove the three HMI `<depend>` lines. Find:

```xml
  <depend>rover_hmi_core</depend>
  <depend>pluginlib</depend>
  <depend>ament_index_cpp</depend>
```

Delete all three lines. The surrounding `<exec_depend>` lines for MoveIt/rviz2 stay — those are runtime deps for the launch files and are unaffected.

- [ ] **Step 4: Build dev_arm_moveit_config_v3**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select dev_arm_moveit_config_v3 --symlink-install 2>&1"
```

Expected: build succeeds. Package is now pure config — should be very fast.

- [ ] **Step 5: Commit**

```bash
git add src/dev_arm_moveit_config_v3/CMakeLists.txt \
        src/dev_arm_moveit_config_v3/package.xml \
        src/dev_arm_moveit_config_v3/plugins.xml
git commit -m "refactor(dev_arm_moveit_config_v3): drop HMI plugin scaffolding

RvizModule moved to rover_hmi_core in the previous commit. The package
is now a pure MoveIt config: launch/, config/, and package metadata
only. Removes Qt5/rover_hmi_core/pluginlib build deps and the HMI
shared library target."
```

---

## Task 3: Delete HMI files from `arm_hardware_interface`

Strip the 5 duplicate HMI modules and their plugin scaffolding. Moteus driver and protocol headers stay.

**Files:**
- Delete: `src/arm_hardware_interface/src/{motor_status,plotting,motor_config,send_command,command_log}_module.cpp` (5 files)
- Delete: `src/arm_hardware_interface/include/{motor_status,plotting,motor_config,send_command,command_log}_module.h` (5 files)
- Delete: `src/arm_hardware_interface/plugins.xml`
- Modify: `src/arm_hardware_interface/CMakeLists.txt`
- Modify: `src/arm_hardware_interface/package.xml`

- [ ] **Step 1: Delete the 10 HMI source/header files**

```bash
git rm src/arm_hardware_interface/src/motor_status_module.cpp \
       src/arm_hardware_interface/src/plotting_module.cpp \
       src/arm_hardware_interface/src/motor_config_module.cpp \
       src/arm_hardware_interface/src/send_command_module.cpp \
       src/arm_hardware_interface/src/command_log_module.cpp \
       src/arm_hardware_interface/include/motor_status_module.h \
       src/arm_hardware_interface/include/plotting_module.h \
       src/arm_hardware_interface/include/motor_config_module.h \
       src/arm_hardware_interface/include/send_command_module.h \
       src/arm_hardware_interface/include/command_log_module.h
```

- [ ] **Step 2: Delete plugins.xml**

```bash
git rm src/arm_hardware_interface/plugins.xml
```

- [ ] **Step 3: Rewrite `arm_hardware_interface/CMakeLists.txt`**

Replace the entire contents of `src/arm_hardware_interface/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external_pkgs/moteus moteus_build)

# CAN-FD driver executable
add_executable(moteus_driver src/moteus_driver_node.cpp)
target_link_libraries(moteus_driver moteus::cpp)
ament_target_dependencies(moteus_driver rclcpp rover_msgs std_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include/)

install(TARGETS moteus_driver
    DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
```

This removes: `find_package(rover_hmi_core REQUIRED)`, `find_package(pluginlib REQUIRED)`, `find_package(Qt5 REQUIRED COMPONENTS Widgets)`, `set(CMAKE_AUTOMOC ON)`, the entire `add_library(arm_hardware_interface_hmi SHARED ...)` block with its link/dependency rules, `pluginlib_export_plugin_description_file(rover_hmi_core plugins.xml)`, and `install(TARGETS arm_hardware_interface_hmi ...)`. It also drops the unused `rover_hmi_core` entry from `ament_target_dependencies(moteus_driver ...)` — verified by reading `moteus_driver_node.cpp`, which has zero HMI includes.

- [ ] **Step 4: Edit `arm_hardware_interface/package.xml`**

Remove the two HMI `<depend>` lines. Find:

```xml
  <depend>rover_hmi_core</depend>
  <depend>pluginlib</depend>
```

Delete both lines.

- [ ] **Step 5: Build arm_hardware_interface**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select arm_hardware_interface --symlink-install 2>&1"
```

Expected: build succeeds. The moteus_driver executable should link with `moteus::cpp` and the rover_msgs typesupports.

If it fails with `undefined reference` errors related to `rover_hmi_core` or Qt — the find_package deletions in Step 3 didn't land cleanly. Re-check the CMakeLists.

- [ ] **Step 6: Commit**

```bash
git add src/arm_hardware_interface/CMakeLists.txt \
        src/arm_hardware_interface/package.xml \
        src/arm_hardware_interface/plugins.xml \
        src/arm_hardware_interface/src/motor_status_module.cpp \
        src/arm_hardware_interface/src/plotting_module.cpp \
        src/arm_hardware_interface/src/motor_config_module.cpp \
        src/arm_hardware_interface/src/send_command_module.cpp \
        src/arm_hardware_interface/src/command_log_module.cpp \
        src/arm_hardware_interface/include/motor_status_module.h \
        src/arm_hardware_interface/include/plotting_module.h \
        src/arm_hardware_interface/include/motor_config_module.h \
        src/arm_hardware_interface/include/send_command_module.h \
        src/arm_hardware_interface/include/command_log_module.h
git commit -m "refactor(arm_hardware_interface): drop HMI plugins, keep moteus_driver

The 5 arm HMI modules (motor_status, plotting, motor_config,
send_command, command_log) lived as byte-identical (or layout-only)
duplicates of the canonical copies already in rover_hmi_core/src/arm/.
Removed them along with the Qt5/pluginlib/rover_hmi_core build deps
and the HMI plugin descriptor. moteus_driver_node and the moteus
protocol headers stay."
```

---

## Task 4: Delete `rover_drive_hmi` and `rover_science_hmi`

Both packages have already been replaced by canonical copies inside `rover_hmi_core`. Remove them outright.

**Files:**
- Delete: `src/rover_drive_hmi/` (entire directory)
- Delete: `src/rover_science_hmi/` (entire directory)

- [ ] **Step 1: Confirm no other package depends on these by name**

```bash
grep -rE "rover_drive_hmi|rover_science_hmi" src/ \
    --include="CMakeLists.txt" --include="package.xml" --include="*.launch.py" \
    | grep -v "^src/rover_drive_hmi/" \
    | grep -v "^src/rover_science_hmi/"
```

Expected: no output. If anything appears, that file must be updated first — surface it before deleting.

- [ ] **Step 2: Delete the two directories**

```bash
git rm -r src/rover_drive_hmi src/rover_science_hmi
```

- [ ] **Step 3: Commit**

```bash
git commit -m "refactor: delete rover_drive_hmi and rover_science_hmi packages

Both packages held a single GuiModule plugin (DriveModule and
ScienceModule respectively) that already exists as the canonical copy
in rover_hmi_core/src/{drive,science}/. The duplicate registrations
caused the HMI host to load each module twice at startup. Deletion
fixes that duplicate-load bug as a side effect."
```

---

## Task 5: Clean rebuild + smoke test

No more code changes. This task validates the entire consolidation worked end-to-end.

- [ ] **Step 1: Wipe and rebuild the entire workspace**

```bash
docker compose --compatibility exec rover bash -c "cd /RoverFlake2 && rm -rf build install log 2>&1"
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --symlink-install 2>&1" | tail -40
```

Expected: every package builds. The pre-existing root-CMakeLists `Failed to extract project name` warning is unrelated. If any package fails with `Could not find a package configuration file provided by "rover_drive_hmi"` or similar, a leftover `find_package` reference exists somewhere — grep the whole tree for it, fix, rebuild.

- [ ] **Step 2: Confirm the deleted packages are gone from the ament index**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 pkg list 2>&1 | grep -iE 'rover_drive_hmi|rover_science_hmi'"
```

Expected: no output.

- [ ] **Step 3: Confirm pluginlib registers the moveit RvizModule under the new name**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && grep -rE 'RvizModule' /RoverFlake2/install 2>&1 | head -5"
```

Expected: at least one match showing `rover_hmi_core/RvizModule` in `rover_hmi_core/share/.../plugins.xml`. No matches for `dev_arm_moveit_config_v3/RvizModule`.

- [ ] **Step 4: HMI smoke test**

```bash
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 run rover_hmi_core rover_hmi 2>&1"
```

Expected when the window opens:
- The sidebar lists each module **exactly once** (no duplicate `MotorStatusModule`, `DriveModule`, `ScienceModule` entries — those duplicate-loads were the latent bug this consolidation fixes).
- The RViz panel is initially hidden (it's `defaultVisible() == false`). When toggled on it spawns rviz2 as a child process and embeds the window.
- Every other module renders the same as before (the canonical code didn't change; we only removed duplicates of it).

If a previously-working module is missing entirely from the sidebar, the most likely cause is a stale build artifact in `install/` — repeat Step 1.

- [ ] **Step 5: No commit**

This task is verification only. Nothing to add.

---

## Self-review

- **Spec coverage:** Every spec section maps to a task.
  - "Final layout" → Tasks 1 (rover_hmi_core additions), 2 (moveit config strip), 3 (arm HW strip), 4 (delete drive/science HMI packages).
  - "plugins.xml consolidation" → Task 1 Step 3 (append entry) + Tasks 2, 3 (delete external plugins.xml files).
  - "CMakeLists & package.xml changes" → Tasks 1 Steps 4–5, 2 Steps 2–3, 3 Steps 3–4.
  - "Verification" → Task 5 Steps 1–4 (clean build, ament index check, pluginlib registry check, runtime smoke test).
  - "Edge cases & risks":
    - Stale install/ artifacts → Task 5 Step 1 (rm -rf build install log before rebuild).
    - External consumers grep → Task 4 Step 1 (defensive re-grep before deletion).
    - moteus_driver leftover dep → Task 3 Step 3 (confirmed during plan-writing that moteus_driver_node.cpp has no HMI includes; drop is unconditional).
    - Exact RViz dep list → confirmed during plan-writing: only `ament_index_cpp` (rviz2 is a runtime subprocess, not a build dep).
    - Git history preservation → Task 1 Step 2 (`git mv`), Tasks 3 & 4 (`git rm`).
  - "Out of scope" (legacy GTK rover_hmi, prior drive work, in-module refactors) → not touched by any task.
- **Placeholder scan:** No "TBD" / "TODO" / "add appropriate" / "similar to" in any task. Every CMakeLists rewrite shows the complete final file. Every `<depend>` change cites the exact lines to delete. Every commit message is written out verbatim. The "RViz deps" question raised in the spec is resolved inline ("only ament_index_cpp").
- **Type consistency:** Class string `rover_hmi_core/RvizModule` used identically in plugins.xml entry (Task 1 Step 3) and the pluginlib registry check (Task 5 Step 3). File paths `src/rover_hmi_core/src/moveit/rviz_module.cpp` and `src/rover_hmi_core/include/rover_hmi_core/moveit/rviz_module.h` used identically across Task 1 Steps 1, 2, 4 (in CMakeLists `add_library` and `target_include_directories`).
