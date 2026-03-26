#include "tiling_container.h"
#include "catppuccin.h"

#include <QHBoxLayout>
#include <QApplication>
#include <QMouseEvent>
#include <QCursor>
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

void TilePanel::mousePressEvent(QMouseEvent* e) {
    emit clicked();
    // Let parent handle it for drag operations
    e->ignore();
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

    connect(panel, &TilePanel::hovered, [this, panel]() {
        // Only hover-focus when not in drag mode
        if (drag_mode_ == DragMode::None) {
            for (int i = 0; i < (int)panels_.size(); i++) {
                if (panels_[i].panel == panel) { setFocusedIndex(i); break; }
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

int TilingContainer::panelAtPos(const QPoint& globalPos) {
    for (int i = 0; i < (int)panels_.size(); i++) {
        auto* p = panels_[i].panel;
        QRect r(p->mapToGlobal(QPoint(0, 0)), p->size());
        if (r.contains(globalPos)) return i;
    }
    return -1;
}

void TilingContainer::finalize() {
    if (panels_.empty()) return;

    int left_row = 0, right_row = 0, bottom_row = 0;
    for (auto& pi : panels_) {
        if (pi.hint == "main" || pi.hint == "left") {
            pi.grid_col = 0; pi.grid_row = left_row++;
        } else if (pi.hint == "right") {
            pi.grid_col = 1; pi.grid_row = right_row++;
        } else {
            pi.grid_col = 2; pi.grid_row = bottom_row++;
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

    // Alt+Arrow: focus direction
    bind("Alt+Left",  [this]() { focusDirection(-1, 0); });
    bind("Alt+Right", [this]() { focusDirection(1, 0); });
    bind("Alt+Up",    [this]() { focusDirection(0, -1); });
    bind("Alt+Down",  [this]() { focusDirection(0, 1); });

    // Alt+Shift+Arrow: resize (resizeactive)
    bind("Alt+Shift+Right", [this]() { resizeFocused(1, 0); });
    bind("Alt+Shift+Left",  [this]() { resizeFocused(-1, 0); });
    bind("Alt+Shift+Up",    [this]() { resizeFocused(0, -1); });
    bind("Alt+Shift+Down",  [this]() { resizeFocused(0, 1); });

    // Alt+Shift+Ctrl+Arrow: move/swap window
    bind("Alt+Shift+Ctrl+Left",  [this]() { swapWithFocused(-1, 0); });
    bind("Alt+Shift+Ctrl+Right", [this]() { swapWithFocused(1, 0); });
    bind("Alt+Shift+Ctrl+Up",    [this]() { swapWithFocused(0, -1); });
    bind("Alt+Shift+Ctrl+Down",  [this]() { swapWithFocused(0, 1); });

    // Alt+J: toggle split orientation
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

// ---------------------------------------------------------------------------
// Alt+Z (move) / Alt+X (resize) — hold key + mouse drag
// ---------------------------------------------------------------------------

void TilingContainer::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Alt) {
        alt_held_ = true;
    }

    // Alt+Z -> enter move mode
    if (alt_held_ && event->key() == Qt::Key_Z && drag_mode_ == DragMode::None) {
        drag_mode_ = DragMode::Move;
        setCursor(Qt::SizeAllCursor);
        return;
    }

    // Alt+X -> enter resize mode
    if (alt_held_ && event->key() == Qt::Key_X && drag_mode_ == DragMode::None) {
        drag_mode_ = DragMode::Resize;
        setCursor(Qt::SizeFDiagCursor);
        return;
    }

    QWidget::keyPressEvent(event);
}

void TilingContainer::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Alt) {
        alt_held_ = false;
    }

    // Release Z or X or Alt -> end drag mode
    if (event->key() == Qt::Key_Z || event->key() == Qt::Key_X || event->key() == Qt::Key_Alt) {
        if (drag_mode_ != DragMode::None) {
            drag_mode_ = DragMode::None;
            dragging_ = false;
            drag_panel_idx_ = -1;
            unsetCursor();
        }
    }

    QWidget::keyReleaseEvent(event);
}

void TilingContainer::mousePressEvent(QMouseEvent* event) {
    if (drag_mode_ != DragMode::None && event->button() == Qt::LeftButton) {
        dragging_ = true;
        drag_start_ = event->globalPos();
        drag_panel_idx_ = panelAtPos(event->globalPos());
        if (drag_panel_idx_ >= 0)
            setFocusedIndex(drag_panel_idx_);
        return;
    }
    QWidget::mousePressEvent(event);
}

void TilingContainer::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_ || drag_panel_idx_ < 0) {
        QWidget::mouseMoveEvent(event);
        return;
    }

    QPoint delta = event->globalPos() - drag_start_;

    if (drag_mode_ == DragMode::Move) {
        // Move: check if mouse is now over a different panel -> swap
        int target = panelAtPos(event->globalPos());
        if (target >= 0 && target != drag_panel_idx_) {
            // Swap the two panels
            auto& a = panels_[drag_panel_idx_];
            auto& b = panels_[target];

            std::swap(a.grid_col, b.grid_col);
            std::swap(a.grid_row, b.grid_row);

            auto* panelA = a.panel;
            auto* panelB = b.panel;
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

            drag_panel_idx_ = target;
            setFocusedIndex(target);
            drag_start_ = event->globalPos();
        }
    } else if (drag_mode_ == DragMode::Resize) {
        // Resize: adjust splitter sizes based on mouse delta
        auto* panel = panels_[drag_panel_idx_].panel;
        auto* parent = qobject_cast<QSplitter*>(panel->parentWidget());
        if (parent) {
            int idx = parent->indexOf(panel);
            QList<int> sizes = parent->sizes();

            if (parent->orientation() == Qt::Horizontal && std::abs(delta.x()) > 2) {
                if (idx < sizes.size()) {
                    sizes[idx] += delta.x();
                    if (idx + 1 < sizes.size()) sizes[idx + 1] -= delta.x();
                    parent->setSizes(sizes);
                }
            }
            if (parent->orientation() == Qt::Vertical && std::abs(delta.y()) > 2) {
                if (idx < sizes.size()) {
                    sizes[idx] += delta.y();
                    if (idx + 1 < sizes.size()) sizes[idx + 1] -= delta.y();
                    parent->setSizes(sizes);
                }
            }

            // Also resize parent splitter for cross-axis
            auto* gp = qobject_cast<QSplitter*>(parent->parentWidget());
            if (gp) {
                int pidx = gp->indexOf(parent);
                QList<int> ps = gp->sizes();
                if (gp->orientation() == Qt::Horizontal && std::abs(delta.x()) > 2) {
                    if (pidx < ps.size()) {
                        ps[pidx] += delta.x();
                        if (pidx + 1 < ps.size()) ps[pidx + 1] -= delta.x();
                        gp->setSizes(ps);
                    }
                }
                if (gp->orientation() == Qt::Vertical && std::abs(delta.y()) > 2) {
                    if (pidx < ps.size()) {
                        ps[pidx] += delta.y();
                        if (pidx + 1 < ps.size()) ps[pidx + 1] -= delta.y();
                        gp->setSizes(ps);
                    }
                }
            }
        }

        drag_start_ = event->globalPos();
    }
}

void TilingContainer::mouseReleaseEvent(QMouseEvent* event) {
    if (dragging_) {
        dragging_ = false;
        // Stay in drag mode until key is released (hold behavior)
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

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
            if (p.grid_col == cur.grid_col && p.grid_row == target_row) {
                best = i; break;
            }
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
