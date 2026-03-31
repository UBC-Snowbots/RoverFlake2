// =============================================================================
// tiling_container.h — Hyprland-style dwindle tiling layout for Qt widgets
//
// Panels are organised in a binary space partition (BSP) tree whose structure
// mirrors Hyprland's SDwindleNodeData.  Every internal node splits its
// rectangle either horizontally or vertically at a configurable ratio; every
// leaf wraps a single TilePanel.  Adding a panel bisects the leaf that
// currently holds focus, so the layout grows organically without any manual
// geometry management.
//
// Key classes:
//   DwindleNode          — one node in the BSP tree (internal split or leaf)
//   TilePanel            — titled widget wrapper shown at each leaf
//   DragOverlay          — translucent floating snapshot rendered during drag
//   KeybindingsOverlay   — Alt+/ keybindings cheatsheet, arrow-key browsable
//   LayoutManagerOverlay — Alt+P saved-layout manager (save/load/delete)
//   ModuleSidebar        — checkbox list that hides/shows panels
//   TilingContainer      — root widget; owns the tree, overlays, and input
//
// Lifecycle expected by the host (hmi_host.cpp in this same package):
//   1. Call addPanel() for each module (pre-finalize registration).
//   2. Call finalize() once — builds the initial tree from the accumulated
//      panels, partitioned by their layout hint ("main", "right", "bottom").
//   3. Show the window.  All subsequent layout changes are driven by the
//      keyboard shortcuts documented on TilingContainer below.
// =============================================================================

#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QShortcut>
#include <QCheckBox>
#include <QPixmap>
#include <QKeyEvent>
#include <QPoint>
#include <QRect>
#include <QJsonObject>

#include <vector>
#include <string>
#include <functional>
#include <array>

// ---------------------------------------------------------------------------
// Overlay data types
// ---------------------------------------------------------------------------
struct KeybindEntry    { QString keys; QString description; };
struct KeybindCategory { QString name; std::vector<KeybindEntry> entries; };

struct LayoutSnapshot {
    QString name;
    QString saved_at;   // ISO date string
    QString json;       // full serialized layout
};

class TilePanel;
class TilingContainer;

// ---------------------------------------------------------------------------
// Dwindle binary tree node — mirrors Hyprland's SDwindleNodeData
//   isNode=true  → internal node, children[0/1] are sub-trees
//   isNode=false → leaf, panel points to the TilePanel widget
//   splitTop     → true = top/bottom split, false = left/right split
//   splitRatio   → 0.1..1.9; 1.0 means 50/50
// ---------------------------------------------------------------------------
struct DwindleNode {
    DwindleNode* parent = nullptr;
    bool isNode = false;
    std::array<DwindleNode*, 2> children = {nullptr, nullptr};
    bool splitTop = false;
    float splitRatio = 1.0f;
    QRect box;                   // in tiling_area_ local coordinates
    TilePanel* panel = nullptr;  // non-null only for leaf nodes

    DwindleNode* sibling() const {
        if (!parent) return nullptr;
        return parent->children[0] == this ? parent->children[1] : parent->children[0];
    }

    DwindleNode* leafFor(TilePanel* p);
    void recalcSizePosRecursive(int gap = 3);
};


// ---------------------------------------------------------------------------
// TilePanel — a single titled panel in the tiling area
// ---------------------------------------------------------------------------
class TilePanel : public QWidget {
    Q_OBJECT
public:
    TilePanel(const std::string& title, QWidget* content, QWidget* parent = nullptr);

    void setFocused(bool focused);
    void setDropTarget(bool target);
    bool isFocused() const { return focused_; }
    QWidget* content() const { return content_; }
    std::string title() const { return title_; }

signals:
    void clicked();
    void hovered();

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void enterEvent(QEvent*) override;

private:
    std::string title_;
    QWidget* content_;
    bool focused_ = false;
    bool drop_target_ = false;
};


enum class DragMode { None, Move, Resize };


// ---------------------------------------------------------------------------
// DragOverlay — translucent floating snapshot during Alt+Z drag
// ---------------------------------------------------------------------------
class DragOverlay : public QWidget {
public:
    explicit DragOverlay(QWidget* parent = nullptr);
    void setSnapshot(const QPixmap& pixmap);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QPixmap snapshot_;
};


// ---------------------------------------------------------------------------
// KeybindingsOverlay — Alt+/ cheatsheet, arrow-key / scroll browsable
// ---------------------------------------------------------------------------
class KeybindingsOverlay : public QWidget {
    Q_OBJECT
public:
    explicit KeybindingsOverlay(TilingContainer* tc, QWidget* parent = nullptr);
    void setCategories(std::vector<KeybindCategory> cats);

protected:
    void paintEvent(QPaintEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    TilingContainer* tc_;
    std::vector<KeybindCategory> categories_;
    int focused_idx_   = 0;
    int scroll_offset_ = 0;

    int  totalEntries() const;
    int  yOfEntry(int flat_idx) const;
    void scrollToFocused(int list_h);
};


// ---------------------------------------------------------------------------
// LayoutManagerOverlay — Alt+P layout save/load/delete panel
// ---------------------------------------------------------------------------
class LayoutManagerOverlay : public QWidget {
    Q_OBJECT
public:
    explicit LayoutManagerOverlay(TilingContainer* tc, QWidget* parent = nullptr);
    void refresh();   // reload snapshots from persistent storage

protected:
    void paintEvent(QPaintEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    TilingContainer* tc_;
    std::vector<LayoutSnapshot> snapshots_;
    int focused_idx_   = 0;
    int scroll_offset_ = 0;
};


// ---------------------------------------------------------------------------
// ModuleSidebar — section-grouped checkbox list that controls panel visibility
//
//   Modules are organised into named sections (e.g. "Arm", "Science").
//   Alt+1..9 applies only to modules in the *active* section.
//   Alt+[ / Alt+] cycle through sections.
// ---------------------------------------------------------------------------
class ModuleSidebar : public QWidget {
    Q_OBJECT
public:
    explicit ModuleSidebar(QWidget* parent = nullptr);

    // Add a new section heading.  Call before addModule() for that section.
    void addSection(const std::string& name);

    // Add a module to the most-recently-added section.
    void addModule(const std::string& name, TilePanel* panel,
                   bool default_visible = true,
                   std::function<void(bool)> on_toggle = nullptr);

    // Toggle the Nth module (0-based) within the currently active section.
    void toggleModule(int index);

    // Cycle the active section forward (+1) or backward (-1).
    void switchSection(int delta);

    // Silent checkbox update used when loading a saved layout.
    void syncCheckboxes(const std::vector<std::string>& visible_titles);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void setActiveSection(int idx);
    void refreshIndexLabels();

    QVBoxLayout* layout_;
    QLabel*      section_indicator_ = nullptr;  // shows "▸ SECTION" in header

    struct Entry { QCheckBox* check; TilePanel* panel; QLabel* idx_lbl; };
    struct Section {
        std::string name;
        QLabel*           header = nullptr;
        std::vector<Entry> entries;
    };
    std::vector<Section> sections_;
    int active_section_ = 0;
};


// ---------------------------------------------------------------------------
// TilingContainer — Hyprland-style dwindle layout for dashboard modules
//
//   Keyboard shortcuts:
//     Alt+Left/Right/Up/Down        — focus adjacent panel
//     Alt+Shift+Left/Right/Up/Down  — resize focused panel
//     Alt+Ctrl+Shift+Arrow          — swap focused panel in direction
//     Alt+Z (hold) + mouse move     — drag-to-swap mode
//     Alt+X (hold) + mouse move     — drag-to-resize mode
//     Alt+Tab                       — cycle focus
//     Alt+J                         — toggle split direction of focused node
//     Alt+1..9                      — toggle module by sidebar index
//     Alt+/                         — toggle keybindings overlay
//     Alt+P                         — toggle layout manager overlay
// ---------------------------------------------------------------------------
class TilingContainer : public QWidget {
    Q_OBJECT
public:
    explicit TilingContainer(QWidget* parent = nullptr);
    ~TilingContainer();

    // Register a panel before finalize()
    void addPanel(const std::string& title, QWidget* content,
                  const std::string& layout_hint = "right",
                  bool default_visible = true,
                  std::function<void(bool)> on_toggle = nullptr,
                  std::vector<std::pair<std::string,std::string>> module_keybinds = {},
                  const std::string& section = "General");

    // Build the initial layout (call after all addPanel calls)
    void finalize();

    // Navigation / manipulation
    void focusNext();
    void focusPrev();
    void focusDirection(int dx, int dy);
    void swapWithFocused(int dx, int dy);
    void resizeFocused(int dx, int dy);

    // Overlay control (called by overlay classes)
    void toggleKeybindingsOverlay();
    void hideKeybindingsOverlay();
    void toggleLayoutManagerOverlay();
    void hideLayoutManagerOverlay();

    // Layout persistence (called by LayoutManagerOverlay)
    void saveCurrentLayout();
    void loadLayout(int index);
    void deleteLayout(int index);

    ModuleSidebar* sidebar() const { return sidebar_; }

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    // Dwindle tree
    void dwindleAdd(TilePanel* panel);
    void dwindleRemove(TilePanel* panel);
    void dwindleSwap(TilePanel* a, TilePanel* b);
    DwindleNode* getLeafFor(TilePanel* panel);
    void recalculate();

    // Build a column/row sub-tree from a list of panels
    DwindleNode* buildColumn(std::vector<TilePanel*>& panels, bool vertical);

    void setFocusedPanel(TilePanel* panel);
    void enterMoveMode();
    void exitDragMode();

    // Tree serialization helpers (used by layout save/load)
    QJsonObject serializeTree(DwindleNode* node) const;
    DwindleNode* deserializeTree(const QJsonObject& obj);

    bool anyOverlayVisible() const;

    struct PanelInfo {
        TilePanel* panel;
        std::string hint;
        std::string section;
        bool default_visible = true;
        std::function<void(bool)> on_toggle;
        std::vector<std::pair<std::string,std::string>> module_keybinds;
    };

    std::vector<PanelInfo> panels_;
    TilePanel* focused_panel_ = nullptr;

    // Dwindle tree data
    DwindleNode* root_ = nullptr;
    std::vector<DwindleNode*> all_nodes_;

    QWidget* tiling_area_ = nullptr;
    ModuleSidebar* sidebar_ = nullptr;

    // Drag/resize state
    DragMode drag_mode_ = DragMode::None;
    QPoint last_mouse_global_;
    DragOverlay* drag_overlay_ = nullptr;
    TilePanel* drag_source_ = nullptr;
    TilePanel* drag_target_ = nullptr;
    QPoint drag_grab_offset_;

    // Overlays
    KeybindingsOverlay*  keybindings_overlay_  = nullptr;
    LayoutManagerOverlay* layout_overlay_       = nullptr;
    std::vector<KeybindCategory> base_categories_;
};
