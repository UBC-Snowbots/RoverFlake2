# HMI Deployment Layouts (launch args + live switch + CBS buttons) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `rover_hmi` start in a named layout or explicit panel set from a launch file, switch layouts live via ROS topics, show the active layout in the window title, and map CBS panel buttons (1 button = 1 layout) to those topics.

**Architecture:** Single HMI process. `TilingContainer` gains a by-name layout load, an explicit-panel-set load, and an `onLayoutChanged` callback. `hmi_host.cpp` exposes both as startup params (`layout`, `panels`) and runtime topics (`/hmi/load_layout`, `/hmi/show_panels`) and wires the callback to the window title. A router node in rover_manager converts button rising edges on `/cbs/left_panel_a` into `/hmi/load_layout` publishes, mapped by a repo-tracked YAML.

**Tech Stack:** ROS2 Humble (C++), Qt5, pluginlib, launch_ros. Spec: `docs/superpowers/specs/2026-08-02-hmi-layout-launch-cbs-design.md`.

## Global Constraints

- Work on a new branch off `main` (suggested: `feat/aaronrhim/hmi_deploy_layouts`). Current checkout is on `fix/aaronrhim/drilling_pipeline` — do not mix.
- **Never commit, push, or open a PR without Aaron's explicit sign-off.** Commit steps below execute only if Aaron pre-approved per-task commits at plan handoff; otherwise leave changes uncommitted at each checkpoint.
- Commit messages: repo conventional style (`feat(rover_hmi_core): …`). **No Co-Authored-By / Claude trailers.**
- Code style: compact, low-line-count comments, no banner spacers beyond what the file already uses; match surrounding idiom.
- Build command (from repo root, container workspace is `/RoverFlake2`):
  `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select <pkg> 2>&1"`
- No unit-test framework exists in these packages; each task's test cycle is compile + runtime verification via `ros2 topic pub`/`ros2 param`. GUI-visible checks and anything touching the physical CBS panel are run by Aaron (suggest `!` prefix commands), never autonomously.
- `panels` is a comma-separated string in both the param and the topic (launch args are strings; keeping one format everywhere). This supersedes the spec's "string array" wording for the param.
- The topic message type for both HMI topics is `std_msgs/msg/String`.
- Panel titles are the `GuiModule::name()` strings (e.g. "Motor Telemetry"); layout names are `LayoutStore` entry names, matched exactly (case-sensitive).
- `docs/superpowers/` specs/plans get deleted before the branch is finalized (Aaron's standing rule).

---

### Task 1: TilingContainer — loadLayoutByName + onLayoutChanged

**Files:**
- Modify: `src/rover_hmi_core/include/rover_hmi_core/tiling_container.h` (public section of `TilingContainer`, near `loadLayout`, ~line 292)
- Modify: `src/rover_hmi_core/src/tiling_container.cpp` (`loadLayout` at ~line 1522)

**Interfaces:**
- Produces: `bool TilingContainer::loadLayoutByName(const QString& name)` — true if a saved layout with that exact name was found and loaded.
- Produces: `std::function<void(const QString&)> TilingContainer::onLayoutChanged` — public member, invoked with the layout name after every successful `loadLayout` (Alt+P loads included). Task 3 assigns it.

- [ ] **Step 1: Declare the new API in tiling_container.h**

In the public section of `TilingContainer`, next to the existing layout-persistence methods:

```cpp
    // Layout persistence (called by LayoutManagerOverlay)
    void saveCurrentLayout();
    void loadLayout(int index);
    bool loadLayoutByName(const QString& name);  // external callers bind to names, not indices
    void deleteLayout(int index);
    void renameLayout(int index, const QString& name);
    LayoutStore& layoutStore() { return layout_store_; }

    // Fired with the layout name after every successful layout load.
    std::function<void(const QString&)> onLayoutChanged;
```

- [ ] **Step 2: Implement in tiling_container.cpp**

At the end of `loadLayout(int index)` (after `hideLayoutManagerOverlay();`):

```cpp
    if (onLayoutChanged) onLayoutChanged(entries[index].name);
```

Directly below `loadLayout`, add:

```cpp
bool TilingContainer::loadLayoutByName(const QString& name) {
    auto entries = layout_store_.list();
    for (int i = 0; i < (int)entries.size(); ++i)
        if (entries[i].name == name) { loadLayout(i); return true; }
    return false;
}
```

- [ ] **Step 3: Build**

Run: `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select rover_hmi_core 2>&1"`
Expected: clean build (warnings pre-existing at worst).

- [ ] **Step 4: Checkpoint / commit (only if pre-approved)**

```bash
git add src/rover_hmi_core/include/rover_hmi_core/tiling_container.h src/rover_hmi_core/src/tiling_container.cpp
git commit -m "feat(rover_hmi_core): by-name layout load + layout-changed callback"
```

---

### Task 2: TilingContainer — buildHintTree extraction + showPanels

**Files:**
- Modify: `src/rover_hmi_core/include/rover_hmi_core/tiling_container.h`
- Modify: `src/rover_hmi_core/src/tiling_container.cpp` (`finalize()` tree-build block at ~lines 948–1000)

**Interfaces:**
- Consumes: `onLayoutChanged` from Task 1.
- Produces: `void TilingContainer::showPanels(const std::vector<std::string>& titles)` — shows exactly those panels (silently ignores titles that match no panel; caller pre-validates), auto-builds the tree, fires `onLayoutChanged` with `"A + B"` joined titles.
- Produces (private): `DwindleNode* buildHintTree(const std::function<bool(const PanelInfo&)>& include)`.

- [ ] **Step 1: Declare in tiling_container.h**

Public, after `loadLayoutByName`:

```cpp
    // Show exactly these panels (titles = GuiModule::name()); tree is rebuilt
    // from layout hints. Unknown titles are ignored — callers pre-validate.
    void showPanels(const std::vector<std::string>& titles);
```

Private, next to `buildColumn`:

```cpp
    // Build the hint-partitioned initial tree (left/right/bottom) from panels
    // selected by `include`. Shared by finalize() and showPanels().
    DwindleNode* buildHintTree(const std::function<bool(const PanelInfo&)>& include);
```

- [ ] **Step 2: Extract buildHintTree from finalize()**

Move the block in `finalize()` from `std::vector<TilePanel*> left_panels, …` through the `root_ = …` assignment into the new method. The method returns the root instead of assigning `root_`; the `pi.default_visible` test becomes `include(pi)`:

```cpp
DwindleNode* TilingContainer::buildHintTree(
        const std::function<bool(const PanelInfo&)>& include) {
    // Groups: hint=="main"|"left" → left column, "right" → right column,
    //         anything else → bottom row
    std::vector<TilePanel*> left_panels, right_panels, bottom_panels;
    for (auto& pi : panels_) {
        if (!include(pi)) continue;
        if (pi.hint == "main" || pi.hint == "left")
            left_panels.push_back(pi.panel);
        else if (pi.hint == "right")
            right_panels.push_back(pi.panel);
        else
            bottom_panels.push_back(pi.panel);
    }

    auto* left_tree   = buildColumn(left_panels,   true);   // vertical stack
    auto* right_tree  = buildColumn(right_panels,  true);   // vertical stack
    auto* bottom_tree = buildColumn(bottom_panels, false);  // horizontal row

    DwindleNode* top_tree = nullptr;
    if (left_tree && right_tree) {
        auto* lr = new DwindleNode();
        lr->isNode = true;
        lr->splitTop = false;
        lr->splitRatio = 1.0f;
        lr->children[0] = left_tree;
        lr->children[1] = right_tree;
        left_tree->parent  = lr;
        right_tree->parent = lr;
        all_nodes_.push_back(lr);
        top_tree = lr;
    } else {
        top_tree = left_tree ? left_tree : right_tree;
    }

    if (top_tree && bottom_tree) {
        auto* tb = new DwindleNode();
        tb->isNode = true;
        tb->splitTop = true;
        tb->splitRatio = TOP_BOTTOM_SPLIT_RATIO;
        tb->children[0] = top_tree;
        tb->children[1] = bottom_tree;
        top_tree->parent    = tb;
        bottom_tree->parent = tb;
        all_nodes_.push_back(tb);
        return tb;
    }
    return top_tree ? top_tree : bottom_tree;
}
```

In `finalize()`, the removed block becomes:

```cpp
    root_ = buildHintTree([](const PanelInfo& pi) { return pi.default_visible; });
```

Keep the surrounding comment banner in `finalize()` only if it still reads correctly; the group-mapping comment moves with the code.

- [ ] **Step 3: Implement showPanels**

Mirrors `loadLayout`'s clear/show/sync/rebuild sequence:

```cpp
void TilingContainer::showPanels(const std::vector<std::string>& titles) {
    for (auto* n : all_nodes_) delete n;
    all_nodes_.clear();
    root_          = nullptr;
    focused_panel_ = nullptr;

    std::vector<std::string> visible_titles;
    for (auto& pi : panels_) {
        bool vis = std::find(titles.begin(), titles.end(),
                             pi.panel->title()) != titles.end();
        pi.panel->setVisible(vis);
        if (vis) visible_titles.push_back(pi.panel->title());
    }
    sidebar_->syncCheckboxes(visible_titles);

    root_ = buildHintTree([&titles](const PanelInfo& pi) {
        return std::find(titles.begin(), titles.end(),
                         pi.panel->title()) != titles.end();
    });
    recalculate();

    for (auto& pi : panels_)
        if (pi.panel->isVisible()) { setFocusedPanel(pi.panel); break; }

    QStringList shown;
    for (const auto& t : visible_titles) shown << QString::fromStdString(t);
    if (onLayoutChanged) onLayoutChanged(shown.join(" + "));
}
```

`QStringList` needs `#include <QStringList>` in tiling_container.cpp if not already transitively available.

- [ ] **Step 4: Build**

Run: `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select rover_hmi_core 2>&1"`
Expected: clean build.

- [ ] **Step 5: Regression check (Aaron, GUI)**

Suggest Aaron run `! docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 run rover_hmi_core rover_hmi"` and confirm the default tiling looks unchanged (the finalize refactor must be behavior-neutral).

- [ ] **Step 6: Checkpoint / commit (only if pre-approved)**

```bash
git add src/rover_hmi_core/include/rover_hmi_core/tiling_container.h src/rover_hmi_core/src/tiling_container.cpp
git commit -m "feat(rover_hmi_core): showPanels + shared hint-tree builder"
```

---

### Task 3: hmi_host — startup params, control topics, window title

**Files:**
- Modify: `src/rover_hmi_core/src/hmi_host.cpp`
- Modify: `src/rover_hmi_core/CMakeLists.txt:102` (`ament_target_dependencies(rover_hmi rclcpp pluginlib)` → add `std_msgs`)

**Interfaces:**
- Consumes: `loadLayoutByName`, `showPanels`, `onLayoutChanged` (Tasks 1–2).
- Produces: node params `layout` (string, default ""), `panels` (comma-separated string, default ""); topics `/hmi/load_layout`, `/hmi/show_panels` (`std_msgs/msg/String`). Task 4's launch file and Task 5's router rely on these exact names.

- [ ] **Step 1: Add includes and a panel-list splitter to hmi_host.cpp**

With the other includes: `#include <std_msgs/msg/string.hpp>` and `#include <sstream>`, `#include <algorithm>`, `#include <string>`. Above `main()`:

```cpp
// "A, B ,C" → {"A","B","C"} (trimmed, empties dropped)
static std::vector<std::string> splitPanelList(const std::string& csv) {
    std::vector<std::string> out;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
        size_t a = item.find_first_not_of(" \t");
        if (a == std::string::npos) continue;
        size_t b = item.find_last_not_of(" \t");
        out.push_back(item.substr(a, b - a + 1));
    }
    return out;
}
```

- [ ] **Step 2: Wire title callback, params, and subscriptions**

In `main()`, after `tiling->finalize();` (currently line ~108) and before `window.setCentralWidget(tiling);`, insert:

```cpp
    // Window title follows the active layout / panel set.
    tiling->onLayoutChanged = [&window](const QString& name) {
        window.setWindowTitle(name.isEmpty() ? QString("Rover HMI")
                                             : "Rover HMI — " + name);
    };

    // Known panel titles, for validating panel-set requests.
    std::vector<std::string> module_names;
    for (auto& m : modules) module_names.push_back(m->name());

    auto applyPanels = [&node, &module_names, tiling](const std::string& csv) {
        std::vector<std::string> known;
        for (const auto& t : splitPanelList(csv)) {
            if (std::find(module_names.begin(), module_names.end(), t)
                    != module_names.end())
                known.push_back(t);
            else
                RCLCPP_WARN(node->get_logger(),
                            "show_panels: unknown panel '%s'", t.c_str());
        }
        if (!known.empty()) tiling->showPanels(known);
    };
    auto applyLayout = [&node, tiling](const std::string& name) {
        if (tiling->loadLayoutByName(QString::fromStdString(name))) return;
        std::string names;
        for (const auto& e : tiling->layoutStore().list())
            names += (names.empty() ? "" : ", ") + e.name.toStdString();
        RCLCPP_WARN(node->get_logger(),
                    "load_layout: unknown layout '%s' (known: %s)",
                    name.c_str(), names.c_str());
    };

    // Startup selection: layout name wins over explicit panel list.
    const auto layout_param = node->declare_parameter<std::string>("layout", "");
    const auto panels_param = node->declare_parameter<std::string>("panels", "");
    if (!layout_param.empty())      applyLayout(layout_param);
    else if (!panels_param.empty()) applyPanels(panels_param);

    // Runtime control topics. Callbacks run on the Qt thread via the
    // spin_some timer, so they may touch widgets directly.
    auto load_layout_sub = node->create_subscription<std_msgs::msg::String>(
        "/hmi/load_layout", 10,
        [applyLayout](const std_msgs::msg::String& msg) { applyLayout(msg.data); });
    auto show_panels_sub = node->create_subscription<std_msgs::msg::String>(
        "/hmi/show_panels", 10,
        [applyPanels](const std_msgs::msg::String& msg) { applyPanels(msg.data); });
```

Note: `applyLayout`/`applyPanels` capture `node` and `module_names` by reference; both outlive `app.exec()`. The subscription shared_ptrs are named locals so they stay alive for the app's lifetime.

- [ ] **Step 3: Add std_msgs to the rover_hmi target**

`src/rover_hmi_core/CMakeLists.txt` line ~102:

```cmake
ament_target_dependencies(rover_hmi rclcpp pluginlib std_msgs)
```

(`package.xml` already depends on std_msgs.)

- [ ] **Step 4: Build**

Run: `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select rover_hmi_core 2>&1"`
Expected: clean build.

- [ ] **Step 5: Runtime verification (Aaron, GUI)**

Aaron starts the HMI, then from a second terminal:

```
! docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 topic pub --once /hmi/load_layout std_msgs/msg/String '{data: <a-saved-layout-name>}'"
```

Expected: layout switches live, title becomes `Rover HMI — <name>`. Then:

```
! docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 topic pub --once /hmi/show_panels std_msgs/msg/String '{data: \"Motor Telemetry, Plots\"}'"
```

Expected: exactly those panels shown, title `Rover HMI — Motor Telemetry + Plots`. Also pub an unknown layout name and confirm the WARN lists known names and the GUI is untouched.

- [ ] **Step 6: Checkpoint / commit (only if pre-approved)**

```bash
git add src/rover_hmi_core/src/hmi_host.cpp src/rover_hmi_core/CMakeLists.txt
git commit -m "feat(rover_hmi_core): layout/panels startup params, /hmi control topics, dynamic title"
```

---

### Task 4: hmi.launch.py

**Files:**
- Create: `src/rover_launchers/launch/hmi.launch.py`

**Interfaces:**
- Consumes: `rover_hmi` params `layout` / `panels` from Task 3.
- Produces: `ros2 launch rover_launchers hmi.launch.py layout:=<name>` / `panels:="A,B"`.

- [ ] **Step 1: Write the launch file**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Single parameterized HMI entry point: pick a saved layout by name, or an
# explicit comma-separated panel list. Both empty = default tiling.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='',
                              description='Saved layout name to load at startup'),
        DeclareLaunchArgument('panels', default_value='',
                              description='Comma-separated panel titles to show'),
        Node(
            package='rover_hmi_core',
            executable='rover_hmi',
            name='rover_hmi',
            parameters=[{
                'layout': LaunchConfiguration('layout'),
                'panels': LaunchConfiguration('panels'),
            }],
        ),
    ])
```

- [ ] **Step 2: Build + verify install**

Run: `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select rover_launchers 2>&1 && ls install/rover_launchers/share/rover_launchers/launch/ | grep hmi"`
Expected: `hmi.launch.py` listed (launch dir already installed to share/<pkg> root per recent fix).

- [ ] **Step 3: Launch verification (Aaron, GUI)**

```
! docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 launch rover_launchers hmi.launch.py layout:=<a-saved-layout-name>"
```

Expected: HMI opens directly in that layout with the matching title. Repeat once with `panels:="Motor Telemetry"`.

- [ ] **Step 4: Checkpoint / commit (only if pre-approved)**

```bash
git add src/rover_launchers/launch/hmi.launch.py
git commit -m "feat(rover_launchers): parameterized hmi.launch.py (layout/panels args)"
```

---

### Task 5: CBS button router

**Files:**
- Modify: `src/rover_manager/src/router_A.cpp` (replace skeleton)
- Create: `src/rover_manager/config/cbs_button_map.yaml`
- Modify: `src/rover_manager/CMakeLists.txt` (std_msgs dep on `router_a`, add to install TARGETS)
- Modify: `src/rover_manager/package.xml` (add `<depend>std_msgs</depend>`)
- Modify: `src/rover_manager/launch/cbs_system_bringup.launch.py` (start router with the YAML)

**Interfaces:**
- Consumes: `/cbs/left_panel_a` (`rover_msgs/msg/GenericPanel`: `int16[] buttons`), `/hmi/load_layout` contract from Task 3.
- Produces: node `hmi_router` (executable name stays `router_a`), param `button_layouts` (string array indexed by button).

- [ ] **Step 1: Rewrite router_A.cpp**

Full replacement (the skeleton's commented-out Autoware remnants go away):

```cpp
// hmi_router — maps CBS panel button presses to HMI layout switches.
// Rising-edge detect on /cbs/left_panel_a buttons; button i publishes
// button_layouts[i] on /hmi/load_layout. Empty/missing entry = unbound.

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/generic_panel.hpp"
#include "std_msgs/msg/string.hpp"

class HmiRouter : public rclcpp::Node {
public:
    HmiRouter() : Node("hmi_router") {
        button_layouts_ = declare_parameter<std::vector<std::string>>(
            "button_layouts", std::vector<std::string>{});
        layout_pub_ = create_publisher<std_msgs::msg::String>("/hmi/load_layout", 10);
        panel_sub_  = create_subscription<rover_msgs::msg::GenericPanel>(
            "/cbs/left_panel_a", 10,
            std::bind(&HmiRouter::panelCallback, this, std::placeholders::_1));
    }

private:
    void panelCallback(const rover_msgs::msg::GenericPanel::SharedPtr msg) {
        // First message (or size change) only sets the baseline — a button
        // already held at startup must not fire.
        if (prev_buttons_.size() != msg->buttons.size()) {
            prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
            return;
        }
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            bool rising = msg->buttons[i] && !prev_buttons_[i];
            prev_buttons_[i] = msg->buttons[i];
            if (!rising || i >= button_layouts_.size() || button_layouts_[i].empty())
                continue;
            std_msgs::msg::String out;
            out.data = button_layouts_[i];
            layout_pub_->publish(out);
            RCLCPP_INFO(get_logger(), "button %zu → layout '%s'", i, out.data.c_str());
        }
    }

    std::vector<std::string> button_layouts_;
    std::vector<int16_t> prev_buttons_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr layout_pub_;
    rclcpp::Subscription<rover_msgs::msg::GenericPanel>::SharedPtr panel_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HmiRouter>());
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Write cbs_button_map.yaml**

`src/rover_manager/config/cbs_button_map.yaml` — placeholder names; Aaron renames saved layouts (Alt+P) to match:

```yaml
# CBS button → HMI layout map. Index = button index in /cbs/left_panel_a.
# Empty string = button unbound. Names must exactly match saved layout names.
hmi_router:
  ros__parameters:
    button_layouts: ["drive", "arm", "science", "", "", ""]
```

- [ ] **Step 3: Build plumbing**

`src/rover_manager/CMakeLists.txt`:

```cmake
ament_target_dependencies(router_a rclcpp rover_utils rover_msgs std_msgs)
```

and add `router_a` to the `install(TARGETS …)` list (it is currently built but never installed, so `ros2 run` can't find it).

`src/rover_manager/package.xml`, with the other depends:

```xml
  <depend>std_msgs</depend>
```

- [ ] **Step 4: Start the router from CBS bringup**

In `src/rover_manager/launch/cbs_system_bringup.launch.py` (add `import os` — `get_package_share_directory` is already imported):

```python
    hmi_router = Node(
        package='rover_manager',
        executable='router_a',
        name='hmi_router',
        parameters=[os.path.join(
            get_package_share_directory('rover_manager'),
            'config', 'cbs_button_map.yaml')]
    )
```

and add `hmi_router` to the returned `LaunchDescription` list.

- [ ] **Step 5: Build**

Run: `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select rover_manager 2>&1"`
Expected: clean build.

- [ ] **Step 6: Edge-detection verification (no hardware needed)**

Terminal 1 — run the router with the map:

```
docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && source /RoverFlake2/install/setup.bash && ros2 run rover_manager router_a --ros-args --params-file /RoverFlake2/install/rover_manager/share/rover_manager/config/cbs_button_map.yaml"
```

Terminal 2 — echo the output: `ros2 topic echo /hmi/load_layout`

Terminal 3 — fake a press-and-hold of button 0 (baseline, press, hold, release):

```
ros2 topic pub --once /cbs/left_panel_a rover_msgs/msg/GenericPanel "{num_buttons: 6, num_switches: 5, buttons: [0,0,0,0,0,0], switches: [0,0,0,0,0]}"
ros2 topic pub --once /cbs/left_panel_a rover_msgs/msg/GenericPanel "{num_buttons: 6, num_switches: 5, buttons: [1,0,0,0,0,0], switches: [0,0,0,0,0]}"
ros2 topic pub --once /cbs/left_panel_a rover_msgs/msg/GenericPanel "{num_buttons: 6, num_switches: 5, buttons: [1,0,0,0,0,0], switches: [0,0,0,0,0]}"
ros2 topic pub --once /cbs/left_panel_a rover_msgs/msg/GenericPanel "{num_buttons: 6, num_switches: 5, buttons: [0,0,0,0,0,0], switches: [0,0,0,0,0]}"
```

Expected: exactly **one** `data: drive` on `/hmi/load_layout` (press edge only — not on hold, not on release, not on baseline). Then press button 3 (unbound) the same way → nothing published.

- [ ] **Step 7: Checkpoint / commit (only if pre-approved)**

```bash
git add src/rover_manager/src/router_A.cpp src/rover_manager/config/cbs_button_map.yaml src/rover_manager/CMakeLists.txt src/rover_manager/package.xml src/rover_manager/launch/cbs_system_bringup.launch.py
git commit -m "feat(rover_manager): hmi_router — CBS buttons drive HMI layout switching"
```

---

### Task 6: End-to-end hardware pass (Aaron drives)

**Files:** none — verification only.

- [ ] **Step 1: Aaron renames layouts to stable names**

In the HMI: Alt+P, rename saved layouts to the names in `cbs_button_map.yaml` (`drive`, `arm`, `science`). Layout JSONs are repo-tracked, so the renames land in git.

- [ ] **Step 2: Full stack on the bench (Aaron, real CBS panel)**

Aaron launches CBS bringup + `hmi.launch.py`, presses each mapped button, confirms: single instant layout switch per press, correct title, unbound buttons inert. Watch router log lines for the button→layout mapping.

- [ ] **Step 3: Record outcome honestly**

Whatever was not hardware-verified stays flagged as such in the PR description.
