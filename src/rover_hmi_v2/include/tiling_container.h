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
#include <QScrollArea>

#include <vector>
#include <string>
#include <functional>

// A single panel in the tiling layout
class TilePanel : public QWidget {
    Q_OBJECT
public:
    TilePanel(const std::string& title, QWidget* content, QWidget* parent = nullptr);

    void setFocused(bool focused);
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
};


// Hyprland-style tiling mode
// Alt+Z hold = move mode (mouse movement swaps windows)
// Alt+X hold = resize mode (mouse movement resizes)
enum class DragMode { None, Move, Resize };


// Sidebar for toggling module visibility
class ModuleSidebar : public QWidget {
    Q_OBJECT
public:
    explicit ModuleSidebar(QWidget* parent = nullptr);

    void addModule(const std::string& name, TilePanel* panel);

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
                  const std::string& layout_hint = "right");

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

    struct PanelInfo {
        TilePanel* panel;
        std::string hint;
        int grid_col;
        int grid_row;
    };

    std::vector<PanelInfo> panels_;
    int focused_idx_ = 0;

    QSplitter* main_splitter_ = nullptr;
    QSplitter* right_splitter_ = nullptr;
    QSplitter* bottom_splitter_ = nullptr;

    ModuleSidebar* sidebar_ = nullptr;

    // Hyprland drag state
    DragMode drag_mode_ = DragMode::None;
    QPoint last_mouse_global_;
};
