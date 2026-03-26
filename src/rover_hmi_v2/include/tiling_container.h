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

#include <vector>
#include <string>
#include <functional>
#include <array>

class TilePanel;

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
// ModuleSidebar — checkbox list that controls panel visibility
// ---------------------------------------------------------------------------
class ModuleSidebar : public QWidget {
    Q_OBJECT
public:
    explicit ModuleSidebar(QWidget* parent = nullptr);
    void addModule(const std::string& name, TilePanel* panel, bool default_visible = true,
                   std::function<void(bool)> on_toggle = nullptr);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QVBoxLayout* layout_;
    struct Entry { QCheckBox* check; TilePanel* panel; };
    std::vector<Entry> entries_;
};


// ---------------------------------------------------------------------------
// TilingContainer — Hyprland-style dwindle layout for dashboard modules
//
//   Keyboard shortcuts (when a panel is focused):
//     Alt+Left/Right/Up/Down        — focus adjacent panel
//     Alt+Shift+Left/Right/Up/Down  — resize focused panel
//     Alt+Ctrl+Shift+Arrow          — swap focused panel in direction
//     Alt+Z (hold) + mouse move     — drag-to-swap mode
//     Alt+X (hold) + mouse move     — drag-to-resize mode
//     Alt+Tab                       — cycle focus
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
                  std::function<void(bool)> on_toggle = nullptr);

    // Build the initial layout (call after all addPanel calls)
    void finalize();

    // Navigation / manipulation
    void focusNext();
    void focusPrev();
    void focusDirection(int dx, int dy);
    void swapWithFocused(int dx, int dy);
    void resizeFocused(int dx, int dy);

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

    struct PanelInfo {
        TilePanel* panel;
        std::string hint;
        bool default_visible = true;
        std::function<void(bool)> on_toggle;
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
};
