#pragma once

#include <QWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QLabel>
#include <QKeyEvent>
#include <QPainter>
#include <QPainterPath>
#include <QShortcut>
#include <QPoint>

#include <vector>
#include <string>
#include <functional>

// A single panel in the tiling layout — rounded border, title bar, content area
class TilePanel : public QWidget {
    Q_OBJECT
public:
    TilePanel(const std::string& title, QWidget* content, QWidget* parent = nullptr);

    void setFocused(bool focused);
    bool isFocused() const { return focused_; }
    QWidget* content() const { return content_; }

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


// Drag mode for Alt+Z (move) and Alt+X (resize)
enum class DragMode { None, Move, Resize };


// Hyprland-style tiling container with keyboard navigation
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

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void setFocusedIndex(int idx);
    int panelAtPos(const QPoint& globalPos);

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

    // Alt+Z / Alt+X drag state
    bool alt_held_ = false;
    DragMode drag_mode_ = DragMode::None;
    bool dragging_ = false;
    QPoint drag_start_;
    int drag_panel_idx_ = -1;
};
