#include "tiling_container.h"
#include "catppuccin.h"

#include <QApplication>
#include <QMouseEvent>
#include <QCursor>
#include <algorithm>

static constexpr int RESIZE_STEP = 30;
static constexpr int SIDEBAR_WIDTH = 180;

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
    setMouseTracking(true);
    setAttribute(Qt::WA_Hover, true);
}

void TilePanel::setFocused(bool focused) {
    focused_ = focused;
    update();
}

void TilePanel::mousePressEvent(QMouseEvent* e) {
    emit clicked();
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
// ModuleSidebar
// ---------------------------------------------------------------------------

ModuleSidebar::ModuleSidebar(QWidget* parent) : QWidget(parent) {
    setFixedWidth(SIDEBAR_WIDTH);
    layout_ = new QVBoxLayout(this);
    layout_->setContentsMargins(12, 16, 12, 16);
    layout_->setSpacing(6);

    auto* title = new QLabel("Modules");
    title->setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    title->setStyleSheet(QString("color: %1;").arg(theme::Text));
    layout_->addWidget(title);

    auto* sep = new QWidget();
    sep->setFixedHeight(1);
    sep->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
    layout_->addWidget(sep);

    layout_->addStretch();
}

void ModuleSidebar::addModule(const std::string& name, TilePanel* panel) {
    auto* check = new QCheckBox(QString::fromStdString(name));
    check->setChecked(true);
    check->setFont(QFont("monospace", theme::FontSize));
    check->setStyleSheet(
        QString("QCheckBox { color: %1; spacing: 8px; }"
                "QCheckBox::indicator { width: 16px; height: 16px; }"
                "QCheckBox::indicator:checked { background: %2; border: 1px solid %3; border-radius: 3px; }"
                "QCheckBox::indicator:unchecked { background: %4; border: 1px solid %5; border-radius: 3px; }")
        .arg(theme::Text).arg(theme::Green).arg(theme::Border)
        .arg(theme::Bg).arg(theme::BorderDim));

    QObject::connect(check, &QCheckBox::toggled, [panel](bool visible) {
        panel->setVisible(visible);
    });

    // Insert before the stretch
    layout_->insertWidget(layout_->count() - 1, check);
    entries_.push_back({check, panel});
}

void ModuleSidebar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor("#080808"));

    // Right edge line
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(width() - 1, 0, width() - 1, height());
}


// ---------------------------------------------------------------------------
// TilingContainer
// ---------------------------------------------------------------------------

TilingContainer::TilingContainer(QWidget* parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);

    // Install global event filter to catch Alt+Z / Alt+X anywhere
    qApp->installEventFilter(this);
}

void TilingContainer::addPanel(const std::string& title, QWidget* content,
                                const std::string& layout_hint) {
    auto* panel = new TilePanel(title, content, this);

    connect(panel, &TilePanel::clicked, [this, panel]() {
        for (int i = 0; i < (int)panels_.size(); i++) {
            if (panels_[i].panel == panel) { setFocusedIndex(i); break; }
        }
    });

    // Hyprland follow_mouse=1: hover focuses the window
    connect(panel, &TilePanel::hovered, [this, panel]() {
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
        if (!p->isVisible()) continue;
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

    // Main layout: sidebar | tiling area
    auto* outer_layout = new QHBoxLayout(this);
    outer_layout->setContentsMargins(0, 0, 0, 0);
    outer_layout->setSpacing(0);

    // Sidebar (fixed, non-tileable)
    sidebar_ = new ModuleSidebar(this);
    outer_layout->addWidget(sidebar_);

    // Register all panels in sidebar
    for (auto& pi : panels_) {
        sidebar_->addModule(pi.panel->title(), pi.panel);
    }

    // Tiling area
    auto* tiling_area = new QWidget();
    auto* tiling_layout = new QVBoxLayout(tiling_area);
    tiling_layout->setContentsMargins(6, 6, 6, 6);
    tiling_layout->setSpacing(0);

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
        tiling_layout->addWidget(top_splitter);
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
        tiling_layout->addWidget(vert_splitter);
    }

    outer_layout->addWidget(tiling_area, 1);

    // -----------------------------------------------------------------------
    // Keyboard shortcuts (hyprland $mainMod = Alt)
    // -----------------------------------------------------------------------
    auto bind = [this](const char* seq, std::function<void()> fn) {
        auto* sc = new QShortcut(QKeySequence(seq), this);
        sc->setContext(Qt::ApplicationShortcut);
        sc->setAutoRepeat(true);
        connect(sc, &QShortcut::activated, fn);
    };

    // Alt+Arrow: focus direction (movefocus)
    bind("Alt+Left",  [this]() { focusDirection(-1, 0); });
    bind("Alt+Right", [this]() { focusDirection(1, 0); });
    bind("Alt+Up",    [this]() { focusDirection(0, -1); });
    bind("Alt+Down",  [this]() { focusDirection(0, 1); });

    // Alt+Shift+Arrow: resize (resizeactive)
    bind("Alt+Shift+Right", [this]() { resizeFocused(1, 0); });
    bind("Alt+Shift+Left",  [this]() { resizeFocused(-1, 0); });
    bind("Alt+Shift+Up",    [this]() { resizeFocused(0, -1); });
    bind("Alt+Shift+Down",  [this]() { resizeFocused(0, 1); });

    // Alt+Shift+Ctrl+Arrow: swap window
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
// Hyprland Alt+Z (move) / Alt+X (resize) — hold key, mouse movement acts
// ---------------------------------------------------------------------------
// In hyprland: hold SUPER + press LMB + move mouse = move window
// Here: hold Alt+Z = enter move mode, mouse movement over another panel swaps
//        hold Alt+X = enter resize mode, mouse movement adjusts splitter sizes
// No click needed — just holding the keys and moving the mouse.
// ---------------------------------------------------------------------------

bool TilingContainer::eventFilter(QObject* /*obj*/, QEvent* event) {
    if (event->type() == QEvent::KeyPress) {
        auto* ke = static_cast<QKeyEvent*>(event);

        // Alt+Z -> move mode
        if (ke->key() == Qt::Key_Z && (ke->modifiers() & Qt::AltModifier)
            && !ke->isAutoRepeat()) {
            enterMoveMode();
            return true;
        }
        // Alt+X -> resize mode
        if (ke->key() == Qt::Key_X && (ke->modifiers() & Qt::AltModifier)
            && !ke->isAutoRepeat()) {
            enterResizeMode();
            return true;
        }
    }

    if (event->type() == QEvent::KeyRelease) {
        auto* ke = static_cast<QKeyEvent*>(event);
        if (ke->isAutoRepeat()) return false;

        // Releasing Z, X, or Alt exits drag mode
        if (drag_mode_ != DragMode::None &&
            (ke->key() == Qt::Key_Z || ke->key() == Qt::Key_X || ke->key() == Qt::Key_Alt)) {
            exitDragMode();
            return true;
        }
    }

    // Mouse move during drag mode — no click needed, just mouse movement
    if (event->type() == QEvent::MouseMove && drag_mode_ != DragMode::None) {
        auto* me = static_cast<QMouseEvent*>(event);
        QPoint globalPos = me->globalPos();
        QPoint delta = globalPos - last_mouse_global_;

        if (drag_mode_ == DragMode::Move) {
            // Check if mouse is now over a different panel -> swap
            int target = panelAtPos(globalPos);
            if (target >= 0 && target != focused_idx_) {
                auto& a = panels_[focused_idx_];
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

                setFocusedIndex(target);
            }
        } else if (drag_mode_ == DragMode::Resize) {
            // Resize focused panel's splitter based on mouse delta
            auto* panel = panels_[focused_idx_].panel;
            auto* parent = qobject_cast<QSplitter*>(panel->parentWidget());
            if (parent) {
                int idx = parent->indexOf(panel);
                QList<int> sizes = parent->sizes();

                if (parent->orientation() == Qt::Horizontal && std::abs(delta.x()) > 1) {
                    if (idx < sizes.size()) {
                        sizes[idx] += delta.x();
                        if (idx + 1 < sizes.size()) sizes[idx + 1] -= delta.x();
                        parent->setSizes(sizes);
                    }
                }
                if (parent->orientation() == Qt::Vertical && std::abs(delta.y()) > 1) {
                    if (idx < sizes.size()) {
                        sizes[idx] += delta.y();
                        if (idx + 1 < sizes.size()) sizes[idx + 1] -= delta.y();
                        parent->setSizes(sizes);
                    }
                }

                // Also adjust parent splitter for cross-axis
                auto* gp = qobject_cast<QSplitter*>(parent->parentWidget());
                if (gp) {
                    int pidx = gp->indexOf(parent);
                    QList<int> ps = gp->sizes();
                    if (gp->orientation() == Qt::Horizontal && std::abs(delta.x()) > 1) {
                        if (pidx < ps.size()) {
                            ps[pidx] += delta.x();
                            if (pidx + 1 < ps.size()) ps[pidx + 1] -= delta.x();
                            gp->setSizes(ps);
                        }
                    }
                    if (gp->orientation() == Qt::Vertical && std::abs(delta.y()) > 1) {
                        if (pidx < ps.size()) {
                            ps[pidx] += delta.y();
                            if (pidx + 1 < ps.size()) ps[pidx + 1] -= delta.y();
                            gp->setSizes(ps);
                        }
                    }
                }
            }
        }

        last_mouse_global_ = globalPos;
        return true;  // Consume the event during drag
    }

    return false;  // Don't consume other events
}

void TilingContainer::enterMoveMode() {
    if (drag_mode_ != DragMode::None) return;
    drag_mode_ = DragMode::Move;
    last_mouse_global_ = QCursor::pos();
    setCursor(Qt::SizeAllCursor);
}

void TilingContainer::enterResizeMode() {
    if (drag_mode_ != DragMode::None) return;
    drag_mode_ = DragMode::Resize;
    last_mouse_global_ = QCursor::pos();
    setCursor(Qt::SizeFDiagCursor);
}

void TilingContainer::exitDragMode() {
    drag_mode_ = DragMode::None;
    unsetCursor();
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
    int start = focused_idx_;
    for (int n = 1; n <= (int)panels_.size(); n++) {
        int i = (start + n) % panels_.size();
        if (panels_[i].panel->isVisible()) {
            setFocusedIndex(i);
            return;
        }
    }
}

void TilingContainer::focusPrev() {
    int start = focused_idx_;
    for (int n = 1; n <= (int)panels_.size(); n++) {
        int i = (start - n + panels_.size()) % panels_.size();
        if (panels_[i].panel->isVisible()) {
            setFocusedIndex(i);
            return;
        }
    }
}

void TilingContainer::focusDirection(int dx, int dy) {
    if (panels_.empty()) return;
    auto& cur = panels_[focused_idx_];
    int target_col = cur.grid_col + dx;
    int target_row = cur.grid_row + dy;

    int best = -1;
    int best_dist = 9999;
    for (int i = 0; i < (int)panels_.size(); i++) {
        if (i == focused_idx_ || !panels_[i].panel->isVisible()) continue;
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
        if (i == focused_idx_ || !panels_[i].panel->isVisible()) continue;
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
