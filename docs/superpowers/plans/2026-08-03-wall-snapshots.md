# Wall Snapshots Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Save every wall window's state under one name with a single keypress; restore the whole wall via `hmi_wall.launch.py wall:=<name>` or `/hmi/wall/load`.

**Architecture:** `/hmi/wall/save` broadcast → each instance writes its own `config/layouts/walls/<slug>/<instance>.json` (no cross-process state exchange, no shared-file writes). `/hmi/wall/load` → each instance reads its own file and applies. LayoutStore gains wall methods; TilingContainer gains `currentLayoutJson()`/`applyLayoutJson()` (refactored out of save/load) plus two host-settable hooks; the Alt+P overlay gains a WALLS section visible only in wall mode. Spec: `docs/superpowers/specs/2026-08-03-wall-snapshots-design.md`.

**Tech Stack:** ROS2 Humble (C++), Qt5.

## Global Constraints

- Branch `feat/aaronrhim/hmi_deploy_layouts`. Local commits only; no push/PR without Aaron. No Claude trailers.
- The working tree may contain unrelated uncommitted edits (e.g. ptz_camera_module files) — commit ONLY the files each task names.
- Build: `docker compose --compatibility exec rover bash -c "source /opt/ros/humble/setup.bash && cd /RoverFlake2 && colcon build --packages-select <pkg> 2>&1"`
- Topics verbatim: `/hmi/wall/save`, `/hmi/wall/load` (`std_msgs/msg/String` = wall name). Created ONLY when `instance` param non-empty.
- Storage verbatim: `<layouts-dir>/walls/<wall-slug>/<instance>.json`, schema `{wall, saved_at, tree, visible}` (`tree`/`visible` exactly as single layouts).
- Wall-name→dir resolution MUST be deterministic across racing processes: an existing wall dir whose display name matches wins; otherwise the plain slug (suffix only when a differently-named wall owns that slug).
- Startup precedence: `wall` > `layout` > `panels`. `wall` with empty `instance` → WARN, ignored.
- Single-window regression-neutrality: empty `instance` → no wall topics, no WALLS overlay section, `loadLayout`/`saveCurrentLayout` observable behavior unchanged.
- Wall load sets the title to `<title_base> — <wall name>`; `applyLayoutJson` itself never fires `onLayoutChanged`.
- Overlay keys in wall mode: `W` save wall (timestamp name `yyyy-MM-dd HH:mm`), `Enter` on a wall row → load-wall broadcast + close overlay, `R`/`D` rename/delete wall. Read-only store inerts `W`/`R`/`D` exactly as for layouts.
- Code style: compact comments, match file idiom. No test framework; test cycle = clean build; GUI/runtime checks deferred to Aaron.

---

### Task 1: LayoutStore wall methods

**Files:**
- Modify: `src/rover_hmi_core/include/rover_hmi_core/layout_store.h`
- Modify: `src/rover_hmi_core/src/layout_store.cpp`

**Interfaces (produced, used by Tasks 2–3):**
- `struct LayoutStore::WallEntry { QString name; QString dir_path; QString saved_at; }`
- `std::vector<WallEntry> listWalls() const` — sorted by saved_at, oldest first
- `bool saveWallInstance(const QString& wall_name, const QString& instance, const QJsonObject& layout)`
- `QJsonObject loadWallInstance(const QString& wall_name, const QString& instance) const` — empty object when absent
- `bool removeWall(const QString& wall_name)`
- `bool renameWall(const QString& old_name, const QString& new_name)` — rewrites members' `"wall"` field; dir keeps its original slug (display name is authoritative)

- [ ] **Step 1: Header additions** — inside `class LayoutStore`, after the existing `Entry` API:

```cpp
    // ── Walls: one dir per wall, one file per instance ──────────────────────
    struct WallEntry {
        QString name;      // display name ("wall" field of member files)
        QString dir_path;  // absolute walls/<slug>/ directory
        QString saved_at;  // newest member's timestamp
    };
    std::vector<WallEntry> listWalls() const;      // sorted by saved_at
    bool saveWallInstance(const QString& wall_name, const QString& instance,
                          const QJsonObject& layout);
    QJsonObject loadWallInstance(const QString& wall_name,
                                 const QString& instance) const;
    bool removeWall(const QString& wall_name);
    // Rewrites members' "wall" field; dir keeps its slug (name is authoritative).
    bool renameWall(const QString& old_name, const QString& new_name);
```

and in the private section: `QString wallDirFor(const QString& wall_name) const;`

- [ ] **Step 2: Extract the slug normalizer** — in layout_store.cpp, split `slugFor` so walls reuse the normalization:

```cpp
// "Arm Teleop" → "arm-teleop"
static QString slugify(const QString& name, const QString& fallback) {
    QString slug;
    for (QChar c : name.toLower()) {
        if (c.isLetterOrNumber()) slug += c;
        else if (!slug.endsWith('-')) slug += '-';
    }
    while (slug.endsWith('-')) slug.chop(1);
    while (slug.startsWith('-')) slug.remove(0, 1);
    return slug.isEmpty() ? fallback : slug;
}
```

`slugFor` keeps its signature and collision loop but starts from `QString slug = slugify(name, "layout");`.

- [ ] **Step 3: Wall implementation** — append to layout_store.cpp (add `#include <QDateTime>`):

```cpp
static const char* WALLS_SUBDIR = "walls";

// Display name of a wall dir: "wall" field of any member, else dir basename.
static QString wallDisplayName(const QString& wall_dir) {
    for (const QFileInfo& fi : QDir(wall_dir).entryInfoList({"*.json"}, QDir::Files, QDir::Name)) {
        QFile f(fi.absoluteFilePath());
        if (!f.open(QIODevice::ReadOnly)) continue;
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        if (doc.isObject() && doc.object().contains("wall"))
            return doc.object()["wall"].toString();
    }
    return QFileInfo(wall_dir).fileName();
}

// Deterministic name→dir: racing instances must all resolve one wall name to
// one directory. An existing wall with the name wins; else the plain slug,
// suffixed only while a differently-named wall occupies it.
QString LayoutStore::wallDirFor(const QString& wall_name) const {
    if (dir_.isEmpty()) return {};
    QDir walls(dir_ + "/" + WALLS_SUBDIR);
    for (const QFileInfo& fi : walls.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name))
        if (wallDisplayName(fi.absoluteFilePath()) == wall_name)
            return fi.absoluteFilePath();

    QString slug = slugify(wall_name, "wall");
    QString candidate = slug;
    for (int i = 2; walls.exists(candidate) &&
                    wallDisplayName(walls.absoluteFilePath(candidate)) != wall_name; ++i)
        candidate = slug + "-" + QString::number(i);
    return walls.absoluteFilePath(candidate);
}

std::vector<LayoutStore::WallEntry> LayoutStore::listWalls() const {
    std::vector<WallEntry> out;
    if (dir_.isEmpty()) return out;
    for (const QFileInfo& fi : QDir(dir_ + "/" + WALLS_SUBDIR)
             .entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name)) {
        WallEntry e;
        e.dir_path = fi.absoluteFilePath();
        e.name     = wallDisplayName(e.dir_path);
        for (const QFileInfo& mf : QDir(e.dir_path).entryInfoList({"*.json"}, QDir::Files)) {
            QFile f(mf.absoluteFilePath());
            if (!f.open(QIODevice::ReadOnly)) continue;
            QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
            if (doc.isObject())
                e.saved_at = std::max(e.saved_at, doc.object()["saved_at"].toString());
        }
        out.push_back(std::move(e));
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const WallEntry& a, const WallEntry& b) { return a.saved_at < b.saved_at; });
    return out;
}

bool LayoutStore::saveWallInstance(const QString& wall_name, const QString& instance,
                                   const QJsonObject& layout) {
    if (!writable()) return false;
    QString wdir = wallDirFor(wall_name);
    if (wdir.isEmpty() || !QDir().mkpath(wdir)) return false;
    QJsonObject obj = layout;
    obj["wall"]     = wall_name;
    obj["saved_at"] = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm");
    return writeJson(wdir + "/" + instance + ".json", obj);
}

QJsonObject LayoutStore::loadWallInstance(const QString& wall_name,
                                          const QString& instance) const {
    QString wdir = wallDirFor(wall_name);
    if (wdir.isEmpty()) return {};
    QFile f(wdir + "/" + instance + ".json");
    if (!f.open(QIODevice::ReadOnly)) return {};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    return doc.isObject() ? doc.object() : QJsonObject{};
}

bool LayoutStore::removeWall(const QString& wall_name) {
    if (!writable()) return false;
    QDir wdir(wallDirFor(wall_name));
    return wdir.exists() && wdir.removeRecursively();
}

bool LayoutStore::renameWall(const QString& old_name, const QString& new_name) {
    if (!writable()) return false;
    QDir wdir(wallDirFor(old_name));
    if (!wdir.exists()) return false;
    for (const QFileInfo& fi : wdir.entryInfoList({"*.json"}, QDir::Files)) {
        QFile f(fi.absoluteFilePath());
        if (!f.open(QIODevice::ReadOnly)) return false;
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        f.close();
        if (!doc.isObject()) continue;
        QJsonObject obj = doc.object();
        obj["wall"] = new_name;
        if (!writeJson(fi.absoluteFilePath(), obj)) return false;
    }
    return true;
}
```

- [ ] **Step 4: Build** — rover_hmi_core, expect clean.
- [ ] **Step 5: Commit**

```bash
git add src/rover_hmi_core/include/rover_hmi_core/layout_store.h src/rover_hmi_core/src/layout_store.cpp
git commit -m "feat(rover_hmi_core): LayoutStore wall snapshots — per-instance files under walls/<slug>/"
```

---

### Task 2: TilingContainer refactor, wall hooks, overlay WALLS section

**Files:**
- Modify: `src/rover_hmi_core/include/rover_hmi_core/tiling_container.h`
- Modify: `src/rover_hmi_core/src/tiling_container.cpp`

**Interfaces:**
- Consumes: Task 1's wall methods.
- Produces: `QJsonObject TilingContainer::currentLayoutJson() const`; `void TilingContainer::applyLayoutJson(const QJsonObject&)` (never fires `onLayoutChanged`); public members `std::function<void(const QString&)> onWallSaveRequested, onWallLoadRequested` (both null ⇒ no WALLS UI). Task 3 sets the hooks and calls the two methods.

- [ ] **Step 1: Header** — in `TilingContainer` public section, next to `onLayoutChanged`:

```cpp
    // Wall mode (host sets these when an instance name is configured; both
    // null in single-window mode — the overlay then shows no WALLS section).
    std::function<void(const QString&)> onWallSaveRequested;
    std::function<void(const QString&)> onWallLoadRequested;

    // Current tree+visible state / apply a saved state. applyLayoutJson never
    // fires onLayoutChanged — naming is the caller's concern.
    QJsonObject currentLayoutJson() const;
    void applyLayoutJson(const QJsonObject& layout);
```

In `LayoutManagerOverlay` private section: `std::vector<LayoutStore::WallEntry> walls_;` and helpers `bool wallMode() const; int rowCount() const; bool isWallRow(int idx) const;`.

- [ ] **Step 2: Refactor save/load through the new methods** (behavior-neutral):

```cpp
QJsonObject TilingContainer::currentLayoutJson() const {
    QJsonObject layout;
    if (root_) layout["tree"] = serializeTree(root_);
    QJsonArray visible;
    for (auto& pi : panels_)
        if (pi.panel->isVisible())
            visible.append(QString::fromStdString(pi.panel->title()));
    layout["visible"] = visible;
    return layout;
}
```

`saveCurrentLayout` becomes: `QJsonObject layout = currentLayoutJson();` plus the existing `name`/`saved_at` lines and `layout_store_.save(layout);`.

`applyLayoutJson(const QJsonObject& layout)` receives the current `loadLayout` body from the `visible_set` construction through the focus loop (verbatim move). `loadLayout(int index)` becomes:

```cpp
void TilingContainer::loadLayout(int index) {
    auto entries = layout_store_.list();
    if (index < 0 || index >= (int)entries.size()) return;
    applyLayoutJson(entries[index].json);
    hideLayoutManagerOverlay();
    if (onLayoutChanged) onLayoutChanged(entries[index].name);
}
```

- [ ] **Step 3: Overlay WALLS section.** Row model: indices `0..snapshots_.size()-1` are layouts, `snapshots_.size()..snapshots_.size()+walls_.size()-1` are walls; a non-focusable section header band (height `LAYOUT_ROW_H`) is painted between the two groups when the walls group is shown. Implement:

```cpp
bool LayoutManagerOverlay::wallMode() const { return (bool)tc_->onWallSaveRequested; }
int  LayoutManagerOverlay::rowCount() const {
    return (int)snapshots_.size() + (wallMode() ? (int)walls_.size() : 0);
}
bool LayoutManagerOverlay::isWallRow(int idx) const {
    return idx >= (int)snapshots_.size();
}
```

- `refresh()`: additionally `walls_ = wallMode() ? tc_->layoutStore().listWalls() : {};` and clamp `focused_idx_` to `rowCount()-1`.
- `paintEvent`: after the layouts loop, when `wallMode() && !walls_.empty()` draw the header band (dim text `WALLS — load applies to every window`, same fonts/colors as the hint line) then one row per wall in the exact visual style of layout rows (name left, `saved_at` right, green focus highlight, rename editor reuse). All scroll math (`clampScroll`, wheel, scrollbar) switches from `(int)snapshots_.size() * LAYOUT_ROW_H` to `rowCount() * LAYOUT_ROW_H + (walls shown ? LAYOUT_ROW_H : 0)` for total height, and row-y calculations add `LAYOUT_ROW_H` for wall rows (the header band).
- Hint line gains `  W save wall` when `wallMode()` (writable store only).
- `keyPressEvent` changes:
  - Up/Down clamp to `rowCount()-1`.
  - `Enter`: layout row → `tc_->loadLayout(focused_idx_)` (unchanged); wall row → `tc_->onWallLoadRequested(walls_[focused_idx_ - snapshots_.size()].name); tc_->hideLayoutManagerOverlay();`.
  - `W` (normal mode, writable, `wallMode()`): `tc_->onWallSaveRequested(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm")); refresh(); update();`
  - `R`: wall row → start rename with the wall's name; on confirm route to `tc_->layoutStore().renameWall(old, new)` (layout rows keep `tc_->renameLayout`).
  - `D`: wall row → `tc_->layoutStore().removeWall(name); refresh();` with the existing index re-clamp.
  - Read-only store: `W` joins `S`/`R`/`D` in the inert-keys branch.
- `#include <QDateTime>` in tiling_container.cpp if absent.

- [ ] **Step 4: Build** — rover_hmi_core, expect clean.
- [ ] **Step 5: Commit**

```bash
git add src/rover_hmi_core/include/rover_hmi_core/tiling_container.h src/rover_hmi_core/src/tiling_container.cpp
git commit -m "feat(rover_hmi_core): wall hooks + WALLS overlay section; extract currentLayoutJson/applyLayoutJson"
```

---

### Task 3: hmi_host wall wiring

**Files:**
- Modify: `src/rover_hmi_core/src/hmi_host.cpp`

**Interfaces:**
- Consumes: Tasks 1–2 (`saveWallInstance`, `loadWallInstance`, `listWalls`, `currentLayoutJson`, `applyLayoutJson`, the two hooks, existing `instance`, `title_base`, `applyLayout`, `applyPanels`).
- Produces: topics `/hmi/wall/save` + `/hmi/wall/load`; param `wall` (string, ""). Task 4 passes the param.

- [ ] **Step 1: Wall lambdas + pub/sub.** Inside `main()`, after the existing per-instance subscriptions block, add:

```cpp
    // Wall snapshots: broadcast save/load — every instance persists/applies
    // its OWN state file, sender included (DDS loops messages back).
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wall_save_pub, wall_load_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wall_save_sub, wall_load_sub;
    auto applyWall = [&node, &window, tiling, instance, title_base](const std::string& name) {
        QJsonObject json = tiling->layoutStore().loadWallInstance(
            QString::fromStdString(name), QString::fromStdString(instance));
        if (json.isEmpty()) {
            std::string known;
            for (const auto& w : tiling->layoutStore().listWalls())
                known += (known.empty() ? "" : ", ") + w.name.toStdString();
            RCLCPP_WARN(node->get_logger(),
                        "wall '%s': no state for instance '%s' (known walls: %s)",
                        name.c_str(), instance.c_str(), known.c_str());
            return;
        }
        tiling->applyLayoutJson(json);
        window.setWindowTitle(title_base + " — " + QString::fromStdString(name));
    };
    if (!instance.empty()) {
        wall_save_pub = node->create_publisher<std_msgs::msg::String>("/hmi/wall/save", 10);
        wall_load_pub = node->create_publisher<std_msgs::msg::String>("/hmi/wall/load", 10);
        tiling->onWallSaveRequested = [wall_save_pub](const QString& name) {
            std_msgs::msg::String m; m.data = name.toStdString(); wall_save_pub->publish(m);
        };
        tiling->onWallLoadRequested = [wall_load_pub](const QString& name) {
            std_msgs::msg::String m; m.data = name.toStdString(); wall_load_pub->publish(m);
        };
        wall_save_sub = node->create_subscription<std_msgs::msg::String>(
            "/hmi/wall/save", 10, [&node, tiling, instance](const std_msgs::msg::String& msg) {
                bool ok = tiling->layoutStore().saveWallInstance(
                    QString::fromStdString(msg.data), QString::fromStdString(instance),
                    tiling->currentLayoutJson());
                if (ok) RCLCPP_INFO(node->get_logger(), "wall '%s': instance state saved", msg.data.c_str());
                else    RCLCPP_WARN(node->get_logger(), "wall '%s': save failed — %s", msg.data.c_str(),
                                    tiling->layoutStore().statusMessage().toStdString().c_str());
            });
        wall_load_sub = node->create_subscription<std_msgs::msg::String>(
            "/hmi/wall/load", 10,
            [applyWall](const std_msgs::msg::String& msg) { applyWall(msg.data); });
    }
```

- [ ] **Step 2: `wall` param + precedence.** Replace the startup-selection block:

```cpp
    // Startup selection: wall name wins over layout, layout over panels.
    const auto wall_param   = node->declare_parameter<std::string>("wall", "");
    const auto layout_param = node->declare_parameter<std::string>("layout", "");
    const auto panels_param = node->declare_parameter<std::string>("panels", "");
    if (!wall_param.empty()) {
        if (instance.empty())
            RCLCPP_WARN(node->get_logger(), "wall param requires an instance name — ignored");
        else applyWall(wall_param);
    }
    else if (!layout_param.empty())  applyLayout(layout_param);
    else if (!panels_param.empty())  applyPanels(panels_param);
```

Ordering note: `applyWall` must be defined before this block — place the Step 1 code between the per-instance subscriptions and the startup-selection block, then move the startup-selection block after it (the subscriptions may stay where they are; only the param block moves).

- [ ] **Step 3: Build** — rover_hmi_core, expect clean.
- [ ] **Step 4: Commit**

```bash
git add src/rover_hmi_core/src/hmi_host.cpp
git commit -m "feat(rover_hmi_core): /hmi/wall save+load topics and wall startup param"
```

---

### Task 4: hmi_wall.launch.py wall arg

**Files:**
- Modify: `src/rover_launchers/launch/hmi_wall.launch.py`

- [ ] **Step 1:** Add to the args loop section a fourth declaration and pass-through:

```python
    actions.append(DeclareLaunchArgument(
        'wall', default_value='',
        description='Wall snapshot to restore on startup ("" = none)'))
```

and in each node's `parameters` dict add `'wall': LaunchConfiguration('wall'),`.

- [ ] **Step 2: Build + verify** — rover_launchers; `ros2 launch rover_launchers hmi_wall.launch.py --show-args` inside the container should list `wall`.
- [ ] **Step 3: Commit**

```bash
git add src/rover_launchers/launch/hmi_wall.launch.py
git commit -m "feat(rover_launchers): wall arg — restore a saved wall at startup"
```

---

### Task 5: Bench pass (Aaron drives)

- [ ] Two instances on the dev box: arrange differently → `W` in one → `walls/<ts>/left.json` + `right.json` exist; relaunch `wall:=<ts>` restores both; `/hmi/wall/load` live-switches both; rename + delete from the overlay work.
- [ ] Single-window HMI: no WALLS section, no `/hmi/wall/*` topics in `ros2 topic list`.
- [ ] CBS box: full 3-monitor arrange → save → relaunch → restore loop.
- [ ] Record honestly what was and wasn't verified.
