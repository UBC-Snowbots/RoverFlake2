#include "tiling_container.h"
#include "catppuccin.h"

#include <QHBoxLayout>
#include <QApplication>
#include <algorithm>

// ---------------------------------------------------------------------------
// TilePanel
// ---------------------------------------------------------------------------

TilePanel::TilePanel(const std::string& title, QWidget* content, QWidget* parent)
    : QWidget(parent), title_(title), content_(content) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(theme::BorderWidth + 4, theme::BorderWidth + 4,
                                theme::BorderWidth + 4, theme::BorderWidth + 4);
    layout->setSpacing(0);
    // Reserve space for the title painted in paintEvent
    layout->setContentsMargins(8, 32, 8, 8);
    content->setParent(this);
    layout->addWidget(content);
    setFocusPolicy(Qt::ClickFocus);
}

void TilePanel::setFocused(bool focused) {
    focused_ = focused;
    update();
}

void TilePanel::mousePressEvent(QMouseEvent*) {
    emit clicked();
}

void TilePanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    int r = theme::BorderRadius;
    int bw = theme::BorderWidth;
    QRectF outer(bw / 2.0, bw / 2.0, width() - bw, height() - bw);

    // Panel background
    QPainterPath path;
    path.addRoundedRect(outer, r, r);
    p.fillPath(path, QColor(theme::BgPanel));

    // Border — white when focused, dim when not
    QPen pen(QColor(focused_ ? theme::Border : theme::BorderDim), bw);
    p.setPen(pen);
    p.drawRoundedRect(outer, r, r);

    // Title bar area
    QRectF titleRect(bw + 8, bw + 4, width() - 2 * bw - 16, 24);
    QFont font("monospace", theme::FontSizeLg, QFont::Bold);
    p.setFont(font);
    p.setPen(QColor(focused_ ? theme::Text : theme::TextDim));
    p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter,
               QString::fromStdString(title_));
}


// ---------------------------------------------------------------------------
// TilingContainer
// ---------------------------------------------------------------------------

TilingContainer::TilingContainer(QWidget* parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
}

void TilingContainer::addPanel(const std::string& title, QWidget* content,
                                const std::string& layout_hint) {
    auto* panel = new TilePanel(title, content, this);
    connect(panel, &TilePanel::clicked, [this, panel]() {
        for (int i = 0; i < (int)panels_.size(); i++) {
            if (panels_[i].panel == panel) {
                setFocusedIndex(i);
                break;
            }
        }
    });

    PanelInfo info;
    info.panel = panel;
    info.hint = layout_hint;
    info.grid_col = 0;
    info.grid_row = 0;
    panels_.push_back(info);
}

void TilingContainer::finalize() {
    if (panels_.empty()) return;

    // Assign grid positions based on hints
    // "left"/"main" -> col 0, "right"/"bottom" -> col 1
    int left_row = 0, right_row = 0;
    for (auto& pi : panels_) {
        if (pi.hint == "left" || pi.hint == "main") {
            pi.grid_col = 0;
            pi.grid_row = left_row++;
        } else {
            pi.grid_col = 1;
            pi.grid_row = right_row++;
        }
    }

    // Build splitter layout
    auto* outer_layout = new QHBoxLayout(this);
    outer_layout->setContentsMargins(4, 4, 4, 4);
    outer_layout->setSpacing(0);

    // Collect left and right panels
    std::vector<TilePanel*> left_panels, right_panels;
    for (auto& pi : panels_) {
        if (pi.grid_col == 0) left_panels.push_back(pi.panel);
        else right_panels.push_back(pi.panel);
    }

    main_splitter_ = new QSplitter(Qt::Horizontal, this);
    main_splitter_->setHandleWidth(4);
    main_splitter_->setStyleSheet(
        "QSplitter::handle { background: #222222; }"
    );

    if (left_panels.size() == 1) {
        main_splitter_->addWidget(left_panels[0]);
    } else if (left_panels.size() > 1) {
        auto* left_split = new QSplitter(Qt::Vertical);
        left_split->setHandleWidth(4);
        left_split->setStyleSheet("QSplitter::handle { background: #222222; }");
        for (auto* p : left_panels) left_split->addWidget(p);
        main_splitter_->addWidget(left_split);
    }

    if (right_panels.size() == 1) {
        main_splitter_->addWidget(right_panels[0]);
    } else if (right_panels.size() > 1) {
        right_splitter_ = new QSplitter(Qt::Vertical);
        right_splitter_->setHandleWidth(4);
        right_splitter_->setStyleSheet("QSplitter::handle { background: #222222; }");
        for (auto* p : right_panels) right_splitter_->addWidget(p);
        main_splitter_->addWidget(right_splitter_);
    }

    // Default split: left gets 60%, right gets 40%
    main_splitter_->setSizes({600, 400});
    outer_layout->addWidget(main_splitter_);

    // Set up keybindings (matching hyprland: Super+Arrow = focus, Super+Shift+Arrow = swap)
    auto bind = [this](QKeySequence seq, std::function<void()> fn) {
        auto* sc = new QShortcut(seq, this);
        connect(sc, &QShortcut::activated, fn);
    };

    // Super+Arrow: focus direction
    bind(QKeySequence("Meta+Left"),  [this]() { focusDirection(-1, 0); });
    bind(QKeySequence("Meta+Right"), [this]() { focusDirection(1, 0); });
    bind(QKeySequence("Meta+Up"),    [this]() { focusDirection(0, -1); });
    bind(QKeySequence("Meta+Down"),  [this]() { focusDirection(0, 1); });

    // Super+Shift+Arrow: swap panels
    bind(QKeySequence("Meta+Shift+Left"),  [this]() { swapWithFocused(-1, 0); });
    bind(QKeySequence("Meta+Shift+Right"), [this]() { swapWithFocused(1, 0); });
    bind(QKeySequence("Meta+Shift+Up"),    [this]() { swapWithFocused(0, -1); });
    bind(QKeySequence("Meta+Shift+Down"),  [this]() { swapWithFocused(0, 1); });

    // Tab / Shift+Tab: cycle focus
    bind(QKeySequence("Tab"),       [this]() { focusNext(); });
    bind(QKeySequence("Shift+Tab"), [this]() { focusPrev(); });

    setFocusedIndex(0);
}

void TilingContainer::setFocusedIndex(int idx) {
    if (idx < 0 || idx >= (int)panels_.size()) return;
    for (int i = 0; i < (int)panels_.size(); i++)
        panels_[i].panel->setFocused(i == idx);
    focused_idx_ = idx;
}

void TilingContainer::focusNext() {
    setFocusedIndex((focused_idx_ + 1) % panels_.size());
}

void TilingContainer::focusPrev() {
    setFocusedIndex((focused_idx_ - 1 + panels_.size()) % panels_.size());
}

void TilingContainer::focusDirection(int dx, int dy) {
    if (panels_.empty()) return;
    auto& cur = panels_[focused_idx_];
    int target_col = cur.grid_col + dx;
    int target_row = cur.grid_row + dy;

    // Find the closest panel in that direction
    int best = -1;
    int best_dist = 9999;
    for (int i = 0; i < (int)panels_.size(); i++) {
        if (i == focused_idx_) continue;
        auto& p = panels_[i];

        if (dx != 0 && p.grid_col == target_col) {
            int dist = std::abs(p.grid_row - cur.grid_row);
            if (dist < best_dist) { best = i; best_dist = dist; }
        }
        if (dy != 0 && p.grid_col == cur.grid_col && p.grid_row == target_row) {
            best = i;
            break;
        }
        // If vertical nav crosses column boundary (only 1 panel in current col)
        if (dy != 0 && best == -1 && p.grid_col != cur.grid_col) {
            int dist = std::abs(p.grid_row - target_row) + 1;
            if (dist < best_dist) { best = i; best_dist = dist; }
        }
    }

    if (best >= 0) setFocusedIndex(best);
}

void TilingContainer::swapWithFocused(int dx, int dy) {
    if (panels_.empty()) return;
    auto& cur = panels_[focused_idx_];
    int target_col = cur.grid_col + dx;
    int target_row = cur.grid_row + dy;

    int target = -1;
    int best_dist = 9999;
    for (int i = 0; i < (int)panels_.size(); i++) {
        if (i == focused_idx_) continue;
        auto& p = panels_[i];

        if (dx != 0 && p.grid_col == target_col) {
            int dist = std::abs(p.grid_row - cur.grid_row);
            if (dist < best_dist) { target = i; best_dist = dist; }
        }
        if (dy != 0 && p.grid_col == cur.grid_col && p.grid_row == target_row) {
            target = i;
            break;
        }
    }

    if (target < 0) return;

    // Swap grid positions
    std::swap(panels_[focused_idx_].grid_col, panels_[target].grid_col);
    std::swap(panels_[focused_idx_].grid_row, panels_[target].grid_row);

    // Swap widgets in splitters
    // We need to rebuild the splitter layout
    // Simpler approach: swap the panel widgets within their parent splitters
    auto* panelA = panels_[focused_idx_].panel;
    auto* panelB = panels_[target].panel;

    auto* parentA = qobject_cast<QSplitter*>(panelA->parentWidget());
    auto* parentB = qobject_cast<QSplitter*>(panelB->parentWidget());

    if (parentA && parentB) {
        int idxA = parentA->indexOf(panelA);
        int idxB = parentB->indexOf(panelB);

        if (parentA == parentB) {
            // Same splitter — swap indices
            parentA->insertWidget(idxA, panelB);
            parentA->insertWidget(idxB, panelA);
        } else {
            // Different splitters
            parentA->insertWidget(idxA, panelB);
            parentB->insertWidget(idxB, panelA);
        }
    }
}

void TilingContainer::keyPressEvent(QKeyEvent* event) {
    QWidget::keyPressEvent(event);
}
