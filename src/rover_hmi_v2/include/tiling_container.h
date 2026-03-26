#pragma once

#include <QWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QKeyEvent>
#include <QPainter>
#include <QPainterPath>
#include <QShortcut>
#include <QPoint>
#include <QCheckBox>
#include <QPixmap>

#include <vector>
#include <string>
#include <functional>

// A single panel in the tiling layout
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
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void enterEvent(QEvent* event) override;

private:
    std::string title_;
    QWidget* content_;
    bool focused_ = false;
    bool drop_target_ = false;
};


enum class DragMode { None, Move, Resize };


// Floating overlay that shows a translucent snapshot of the dragged panel
class DragOverlay : public QWidget {
public:
    explicit DragOverlay(QWidget* parent = nullptr);
    void setSnapshot(const QPixmap& pixmap);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPixmap snapshot_;
};


// Sidebar for toggling module visibility
class ModuleSidebar : public QWidget {
    Q_OBJECT
public:
    explicit ModuleSidebar(QWidget* parent = nullptr);

    void addModule(const std::string& name, TilePanel* panel, bool default_visible = true,
                    std::function<void(bool)> on_toggle = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QVBoxLayout* layout_;
    struct Entry {
        QCheckBox* check;
        TilePanel* panel;
    };
    std::vector<Entry> entries_;
};


// Hyprland-style tiling container
class TilingContainer : public QWidget {
    Q_OBJECT
public:
    explicit TilingContainer(QWidget* parent = nullptr);

    void addPanel(const std::string& title, QWidget* content,
                  const std::string& layout_hint = "right",
                  bool default_visible = true,
                  std::function<void(bool)> on_toggle = nullptr);

    void finalize();

    void focusNext();
    void focusPrev();
    void focusDirection(int dx, int dy);
    void swapWithFocused(int dx, int dy);
    void resizeFocused(int dx, int dy);

    ModuleSidebar* sidebar() const { return sidebar_; }

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    void setFocusedIndex(int idx);
    int panelAtPos(const QPoint& globalPos);
    void enterMoveMode();
    void enterResizeMode();
    void exitDragMode();
    void performSwap(int source_idx, int target_idx);

    struct PanelInfo {
        TilePanel* panel;
        std::string hint;
        int grid_col;
        int grid_row;
        bool default_visible = true;
        std::function<void(bool)> on_toggle;
    };

    std::vector<PanelInfo> panels_;
    int focused_idx_ = 0;

    QSplitter* main_splitter_ = nullptr;
    QSplitter* right_splitter_ = nullptr;
    QSplitter* bottom_splitter_ = nullptr;

    ModuleSidebar* sidebar_ = nullptr;

    // Drag state
    DragMode drag_mode_ = DragMode::None;
    QPoint last_mouse_global_;

    // Move-mode floating drag state
    DragOverlay* drag_overlay_ = nullptr;
    int drag_source_idx_ = -1;
    int drag_target_idx_ = -1;
    QPoint drag_grab_offset_;   // offset from panel top-left to mouse at grab time
};
