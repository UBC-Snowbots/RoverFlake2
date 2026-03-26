#include "tiling_container.h"
#include "catppuccin.h"

#include <QHBoxLayout>
#include <QApplication>
#include <QMouseEvent>
#include <algorithm>

static constexpr int RESIZE_STEP = 30;

// ---------------------------------------------------------------------------
// TilePanel
// ---------------------------------------------------------------------------

TilePanel::TilePanel(const std::string& title, QWidget* content, QWidget* parent)
    : QWidget(parent), title_(title), content_(content) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 38, 10, 10);
    layout->setSpacing(0);
    content->setParent(this);
    layout->addWidget(content);
    setFocusPolicy(Qt::ClickFocus);
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover, true);
}

void TilePanel::setFocused(bool focused) {
    focused_ = focused;
    update();
}

void TilePanel::mousePressEvent(QMouseEvent*) {
    emit clicked();
}

void TilePanel::enterEvent(QEvent*) {
    emit hovered();
}

void TilePanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    int r = theme::BorderRadius;
    int bw = theme::BorderWidth;
    QRectF outer(bw / 2.0, bw / 2.0, width() - bw, height() - bw);

    QPainterPath path;
    path.addRoundedRect(outer, r, r);
    p.fillPath(path, QColor(theme::BgPanel));

    QPen pen(QColor(focused_ ? theme::Border : theme::BorderDim), bw);
    p.setPen(pen);
    p.drawRoundedRect(outer, r, r);

    QRectF titleRect(bw + 12, bw + 6, width() - 2 * bw - 24, 28);
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
    setMouseTracking(true);
}

void TilingContainer::addPanel(const std::string& title, QWidget* content,
                                const std::string& layout_hint) {
    auto* panel = new TilePanel(title, content, this);

    connect(panel, &TilePanel::clicked, [this, panel]() {
        for (int i = 0; i < (int)panels_.size(); i++) {
            if (panels_[i].panel == panel) { setFocusedIndex(i); break; }
        }
    });

    // Mouse hover -> focus (like hyprland focus_on_hover)
    connect(panel, &TilePanel::hovered, [this, panel]() {
        for (int i = 0; i < (int)panels_.size(); i++) {
            if (panels_[i].panel == panel) { setFocusedIndex(i); break; }
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

    int left_row = 0, right_row = 0, bottom_row = 0;
    for (auto& pi : panels_) {
        if (pi.hint == "main" || pi.hint == "left") {
            pi.grid_col = 0;
            pi.grid_row = left_row++;
        } else if (pi.hint == "right") {
            pi.grid_col = 1;
            pi.grid_row = right_row++;
        } else {
            pi.grid_col = 2;
            pi.grid_row = bottom_row++;
        }
    }

    std::vector<TilePanel*> left_panels, right_panels, bottom_panels;
    for (auto& pi : panels_) {
        if (pi.grid_col == 0) left_panels.push_back(pi.panel);
        else if (pi.grid_col == 1) right_panels.push_back(pi.panel);
        else bottom_panels.push_back(pi.panel);
    }

    auto* outer_layout = new QVBoxLayout(this);
    outer_layout->setContentsMargins(6, 6, 6, 6);
    outer_layout->setSpacing(0);

    auto ss = QStringLiteral("QSplitter::handle { background: #111111; }");

    // Top: horizontal split of left | right
    auto* top_splitter = new QSplitter(Qt::Horizontal);
    top_splitter->setHandleWidth(4);
    top_splitter->setStyleSheet(ss);
    main_splitter_ = top_splitter;

    if (left_panels.size() == 1) {
        top_splitter->addWidget(left_panels[0]);
    } else if (left_panels.size() > 1) {
        auto* s = new QSplitter(Qt::Vertical);
        s->setHandleWidth(4); s->setStyleSheet(ss);
        for (auto* p : left_panels) s->addWidget(p);
        top_splitter->addWidget(s);
    }

    if (right_panels.size() == 1) {
        top_splitter->addWidget(right_panels[0]);
    } else if (right_panels.size() > 1) {
        right_splitter_ = new QSplitter(Qt::Vertical);
        right_splitter_->setHandleWidth(4);
        right_splitter_->setStyleSheet(ss);
        for (auto* p : right_panels) right_splitter_->addWidget(p);
        top_splitter->addWidget(right_splitter_);
    }

    top_splitter->setSizes({580, 420});

    if (bottom_panels.empty()) {
        outer_layout->addWidget(top_splitter);
    } else {
        auto* vert_splitter = new QSplitter(Qt::Vertical);
        vert_splitter->setHandleWidth(4);
        vert_splitter->setStyleSheet(ss);
        vert_splitter->addWidget(top_splitter);

        if (bottom_panels.size() == 1) {
            vert_splitter->addWidget(bottom_panels[0]);
        } else {
            bottom_splitter_ = new QSplitter(Qt::Horizontal);
            bottom_splitter_->setHandleWidth(4);
            bottom_splitter_->setStyleSheet(ss);
            for (auto* p : bottom_panels) bottom_splitter_->addWidget(p);
            vert_splitter->addWidget(bottom_splitter_);
        }

        vert_splitter->setSizes({750, 250});
        outer_layout->addWidget(vert_splitter);
    }

    // -----------------------------------------------------------------------
    // Keybindings — hyprland $mainMod remapped to Alt
    // -----------------------------------------------------------------------
    auto bind = [this](const char* seq, std::function<void()> fn) {
        auto* sc = new QShortcut(QKeySequence(seq), this);
        sc->setContext(Qt::ApplicationShortcut);
        sc->setAutoRepeat(true);
        connect(sc, &QShortcut::activated, fn);
    };

    // Alt+Arrow: focus direction (movefocus l/r/u/d)
    bind("Alt+Left",  [this]() { focusDirection(-1, 0); });
    bind("Alt+Right", [this]() { focusDirection(1, 0); });
    bind("Alt+Up",    [this]() { focusDirection(0, -1); });
    bind("Alt+Down",  [this]() { focusDirection(0, 1); });

    // Alt+Shift+Arrow: resize active (resizeactive 30 0, etc.)
    bind("Alt+Shift+Right", [this]() { resizeFocused(1, 0); });
    bind("Alt+Shift+Left",  [this]() { resizeFocused(-1, 0); });
    bind("Alt+Shift+Up",    [this]() { resizeFocused(0, -1); });
    bind("Alt+Shift+Down",  [this]() { resizeFocused(0, 1); });

    // Alt+Shift+Ctrl+Arrow: move window (movewindow l/r/u/d)
    bind("Alt+Shift+Ctrl+Left",  [this]() { swapWithFocused(-1, 0); });
    bind("Alt+Shift+Ctrl+Right", [this]() { swapWithFocused(1, 0); });
    bind("Alt+Shift+Ctrl+Up",    [this]() { swapWithFocused(0, -1); });
    bind("Alt+Shift+Ctrl+Down",  [this]() { swapWithFocused(0, 1); });

    // Alt+J: toggle split (swap horizontal/vertical orientation of focused splitter)
    bind("Alt+J", [this]() {
        auto* panel = panels_[focused_idx_].panel;
        auto* parent = qobject_cast<QSplitter*>(panel->parentWidget());
        if (parent) {
            parent->setOrientation(
                parent->orientation() == Qt::Horizontal ? Qt::Vertical : Qt::Horizontal);
        }
    });

    // Alt+Tab: cycle focus
    bind("Alt+Tab", [this]() { focusNext(); });

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

    int best = -1;
    int best_dist = 9999;
    for (int i = 0; i < (int)panels_.size(); i++) {
        if (i == focused_idx_) continue;
        auto& p = panels_[i];

        if (dx != 0 && p.grid_col == target_col) {
            int dist = std::abs(p.grid_row - cur.grid_row);
            if (dist < best_dist) { best = i; best_dist = dist; }
        }
        if (dy != 0) {
            // Same column
            if (p.grid_col == cur.grid_col && p.grid_row == target_row) {
                best = i; break;
            }
            // Top <-> Bottom
            if (dy > 0 && cur.grid_col < 2 && p.grid_col == 2) {
                int dist = std::abs(p.grid_row - cur.grid_row);
                if (dist < best_dist) { best = i; best_dist = dist; }
            }
            if (dy < 0 && cur.grid_col == 2 && p.grid_col < 2) {
                int dist = std::abs(p.grid_row - cur.grid_row) + std::abs(p.grid_col);
                if (dist < best_dist) { best = i; best_dist = dist; }
            }
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
            target = i; break;
        }
    }

    if (target < 0) return;

    std::swap(panels_[focused_idx_].grid_col, panels_[target].grid_col);
    std::swap(panels_[focused_idx_].grid_row, panels_[target].grid_row);

    auto* panelA = panels_[focused_idx_].panel;
    auto* panelB = panels_[target].panel;
    auto* parentA = qobject_cast<QSplitter*>(panelA->parentWidget());
    auto* parentB = qobject_cast<QSplitter*>(panelB->parentWidget());

    if (parentA && parentB) {
        int idxA = parentA->indexOf(panelA);
        int idxB = parentB->indexOf(panelB);
        if (parentA == parentB) {
            parentA->insertWidget(idxA, panelB);
            parentA->insertWidget(idxB, panelA);
        } else {
            parentA->insertWidget(idxA, panelB);
            parentB->insertWidget(idxB, panelA);
        }
    }
}

void TilingContainer::resizeFocused(int dx, int dy) {
    auto* panel = panels_[focused_idx_].panel;
    auto* parent = qobject_cast<QSplitter*>(panel->parentWidget());
    if (!parent) return;

    int idx = parent->indexOf(panel);
    QList<int> sizes = parent->sizes();

    if (parent->orientation() == Qt::Horizontal && dx != 0) {
        if (idx < sizes.size()) {
            sizes[idx] += dx * RESIZE_STEP;
            if (idx + 1 < sizes.size()) sizes[idx + 1] -= dx * RESIZE_STEP;
            parent->setSizes(sizes);
        }
    }
    if (parent->orientation() == Qt::Vertical && dy != 0) {
        if (idx < sizes.size()) {
            sizes[idx] += dy * RESIZE_STEP;
            if (idx + 1 < sizes.size()) sizes[idx + 1] -= dy * RESIZE_STEP;
            parent->setSizes(sizes);
        }
    }

    // Also resize the parent splitter for cross-axis
    auto* gp = qobject_cast<QSplitter*>(parent->parentWidget());
    if (gp) {
        int pidx = gp->indexOf(parent);
        QList<int> ps = gp->sizes();
        if (gp->orientation() == Qt::Horizontal && dx != 0 && pidx < ps.size()) {
            ps[pidx] += dx * RESIZE_STEP;
            if (pidx + 1 < ps.size()) ps[pidx + 1] -= dx * RESIZE_STEP;
            gp->setSizes(ps);
        }
        if (gp->orientation() == Qt::Vertical && dy != 0 && pidx < ps.size()) {
            ps[pidx] += dy * RESIZE_STEP;
            if (pidx + 1 < ps.size()) ps[pidx + 1] -= dy * RESIZE_STEP;
            gp->setSizes(ps);
        }
    }
}

void TilingContainer::keyPressEvent(QKeyEvent* event) {
    QWidget::keyPressEvent(event);
}
