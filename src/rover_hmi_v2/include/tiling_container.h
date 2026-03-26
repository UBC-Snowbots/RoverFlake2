#pragma once

#include <QWidget>
#include <QSplitter>
#include <QVBoxLayout>
#include <QLabel>
#include <QKeyEvent>
#include <QPainter>
#include <QPainterPath>
#include <QShortcut>

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

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    std::string title_;
    QWidget* content_;
    bool focused_ = false;
};


// Hyprland-style tiling container with keyboard navigation
// Manages a grid of TilePanels in a nested splitter layout
class TilingContainer : public QWidget {
    Q_OBJECT
public:
    explicit TilingContainer(QWidget* parent = nullptr);

    // Add a panel. layout_hint: "left", "right", "bottom"
    // First panel added becomes the main (left) panel.
    void addPanel(const std::string& title, QWidget* content,
                  const std::string& layout_hint = "right");

    // Call after all panels are added to set up the layout and keybindings
    void finalize();

    // Focus navigation
    void focusNext();
    void focusPrev();
    void focusDirection(int dx, int dy);
    void swapWithFocused(int dx, int dy);

protected:
    void keyPressEvent(QKeyEvent* event) override;

private:
    void setFocusedIndex(int idx);

    struct PanelInfo {
        TilePanel* panel;
        std::string hint;
        int grid_col;  // logical column (0=left, 1=right)
        int grid_row;  // logical row within column
    };

    std::vector<PanelInfo> panels_;
    int focused_idx_ = 0;

    QSplitter* main_splitter_ = nullptr;
    QSplitter* right_splitter_ = nullptr;
};
