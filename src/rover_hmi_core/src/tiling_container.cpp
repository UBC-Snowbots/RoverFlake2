#include <rover_hmi_core/tiling_container.h>
#include <rover_hmi_core/catppuccin.h>

#include <QApplication>
#include <QMouseEvent>
#include <QCursor>
#include <QJsonDocument>
#include <QJsonArray>
#include <QSettings>
#include <QDateTime>
#include <algorithm>
#include <cmath>

static constexpr int SIDEBAR_WIDTH = 180;
static constexpr int PANEL_GAP = 3;
static constexpr float RESIZE_RATIO_STEP = 0.08f;


// ---------------------------------------------------------------------------
// DwindleNode
// ---------------------------------------------------------------------------

DwindleNode* DwindleNode::leafFor(TilePanel* p) {
    if (!isNode) return (panel == p) ? this : nullptr;
    auto* r = children[0] ? children[0]->leafFor(p) : nullptr;
    if (r) return r;
    return children[1] ? children[1]->leafFor(p) : nullptr;
}

void DwindleNode::recalcSizePosRecursive(int gap) {
    if (!isNode) {
        if (panel)
            panel->setGeometry(box.adjusted(gap, gap, -gap, -gap));
        return;
    }

    auto* c0 = children[0];
    auto* c1 = children[1];
    if (!c0 || !c1) return;

    if (!splitTop) {
        // Left/right split
        int w0 = (int)(box.width() / 2.0f * splitRatio);
        w0 = std::max(80, std::min(w0, box.width() - 80));
        c0->box = QRect(box.x(), box.y(), w0, box.height());
        c1->box = QRect(box.x() + w0, box.y(), box.width() - w0, box.height());
    } else {
        // Top/bottom split
        int h0 = (int)(box.height() / 2.0f * splitRatio);
        h0 = std::max(60, std::min(h0, box.height() - 60));
        c0->box = QRect(box.x(), box.y(), box.width(), h0);
        c1->box = QRect(box.x(), box.y() + h0, box.width(), box.height() - h0);
    }

    c0->recalcSizePosRecursive(gap);
    c1->recalcSizePosRecursive(gap);
}


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

void TilePanel::setDropTarget(bool target) {
    drop_target_ = target;
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

    if (drop_target_) {
        QPen pen(QColor(theme::Green), 3);
        p.setPen(pen);
        p.drawRoundedRect(outer, r, r);
        QColor overlay(theme::Green);
        overlay.setAlpha(20);
        p.fillPath(path, overlay);
    } else {
        QPen pen(QColor(focused_ ? theme::Border : theme::BorderDim), bw);
        p.setPen(pen);
        p.drawRoundedRect(outer, r, r);
    }

    QRectF titleRect(bw + 12, bw + 6, width() - 2 * bw - 24, 28);
    QFont font("monospace", theme::FontSizeLg, QFont::Bold);
    p.setFont(font);
    p.setPen(QColor(focused_ ? theme::Text : theme::TextDim));
    p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter,
               QString::fromStdString(title_));
}


// ---------------------------------------------------------------------------
// DragOverlay
// ---------------------------------------------------------------------------

DragOverlay::DragOverlay(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_TransparentForMouseEvents);
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
}

void DragOverlay::setSnapshot(const QPixmap& pixmap) {
    snapshot_ = pixmap;
    setFixedSize(pixmap.size());
}

void DragOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setOpacity(0.7);
    p.drawPixmap(0, 0, snapshot_);
    p.setOpacity(1.0);
    p.setPen(QPen(QColor(theme::Green), 3));
    p.drawRoundedRect(QRectF(1, 1, width() - 2, height() - 2),
                      theme::BorderRadius, theme::BorderRadius);
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

void ModuleSidebar::addModule(const std::string& name, TilePanel* panel, bool default_visible,
                              std::function<void(bool)> on_toggle) {
    auto* check = new QCheckBox(QString::fromStdString(name));
    check->setChecked(default_visible);
    panel->setVisible(default_visible);
    check->setFont(QFont("monospace", theme::FontSize));
    check->setStyleSheet(
        QString("QCheckBox { color: %1; spacing: 8px; }"
                "QCheckBox::indicator { width: 16px; height: 16px; }"
                "QCheckBox::indicator:checked { background: %2; border: 1px solid %3; border-radius: 3px; }"
                "QCheckBox::indicator:unchecked { background: %4; border: 1px solid %5; border-radius: 3px; }")
        .arg(theme::Text).arg(theme::Green).arg(theme::Border)
        .arg(theme::Bg).arg(theme::BorderDim));

    QObject::connect(check, &QCheckBox::toggled, [on_toggle](bool visible) {
        if (on_toggle)
            on_toggle(visible);
    });

    layout_->insertWidget(layout_->count() - 1, check);
    entries_.push_back({check, panel});
}

void ModuleSidebar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor("#080808"));
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(width() - 1, 0, width() - 1, height());
}

void ModuleSidebar::toggleModule(int index) {
    if (index < 0 || index >= (int)entries_.size()) return;
    entries_[index].check->toggle();
}

void ModuleSidebar::syncCheckboxes(const std::vector<std::string>& visible_titles) {
    for (auto& e : entries_) {
        bool vis = std::find(visible_titles.begin(), visible_titles.end(),
                             e.panel->title()) != visible_titles.end();
        e.check->blockSignals(true);
        e.check->setChecked(vis);
        e.check->blockSignals(false);
    }
}


// ---------------------------------------------------------------------------
// KeybindingsOverlay
// ---------------------------------------------------------------------------

static constexpr int CAT_H = 38;
static constexpr int ROW_H = 30;

KeybindingsOverlay::KeybindingsOverlay(TilingContainer* tc, QWidget* parent)
    : QWidget(parent), tc_(tc) {
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
    setFocusPolicy(Qt::StrongFocus);
    hide();
}

void KeybindingsOverlay::setCategories(std::vector<KeybindCategory> cats) {
    categories_ = std::move(cats);
    focused_idx_   = 0;
    scroll_offset_ = 0;
}

int KeybindingsOverlay::totalEntries() const {
    int n = 0;
    for (auto& cat : categories_) n += (int)cat.entries.size();
    return n;
}

int KeybindingsOverlay::yOfEntry(int flat_idx) const {
    int y = 0, cur = 0;
    for (auto& cat : categories_) {
        y += CAT_H;
        for (int e = 0; e < (int)cat.entries.size(); ++e) {
            if (cur == flat_idx) return y;
            y += ROW_H;
            cur++;
        }
    }
    return y;
}

void KeybindingsOverlay::scrollToFocused(int list_h) {
    int y = yOfEntry(focused_idx_);
    if (y - scroll_offset_ < ROW_H)
        scroll_offset_ = std::max(0, y - ROW_H);
    else if (y + ROW_H * 2 - scroll_offset_ > list_h)
        scroll_offset_ = y + ROW_H * 2 - list_h;
}

void KeybindingsOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0, 0, 0, 210));

    int pw = std::min(680, width()  - 48);
    int ph = std::min(700, height() - 48);
    int px = (width()  - pw) / 2;
    int py = (height() - ph) / 2;

    QPainterPath panelPath;
    panelPath.addRoundedRect(QRect(px, py, pw, ph), 12, 12);
    p.fillPath(panelPath, QColor(theme::BgPanel));
    p.setPen(QPen(QColor(theme::Border), 2));
    p.drawRoundedRect(QRect(px, py, pw, ph), 12, 12);

    int margin = 24;
    QRect titleRect(px + margin, py + margin, pw - 2 * margin, 30);
    p.setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    p.setPen(QColor(theme::Text));
    p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter, "Keybindings");
    p.setFont(QFont("monospace", theme::FontSizeSm));
    p.setPen(QColor(theme::TextDim));
    p.drawText(titleRect, Qt::AlignRight | Qt::AlignVCenter, "Esc · Alt+/ to close");

    int sepY = py + margin + 34;
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(px + margin, sepY, px + pw - margin, sepY);

    if (categories_.empty()) return;

    int listX = px + margin;
    int listY = sepY + 12;
    int listW = pw - 2 * margin;
    int listH = ph - (listY - py) - margin;

    p.setClipRect(listX, listY, listW, listH);
    p.translate(0, listY - scroll_offset_);

    int y = 0, flat = 0;
    for (auto& cat : categories_) {
        p.setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        p.setPen(QColor(theme::TextDim));
        p.drawText(QRect(listX, y + 8, listW, 22),
                   Qt::AlignLeft | Qt::AlignVCenter, cat.name.toUpper());
        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.drawLine(listX, y + CAT_H - 2, listX + listW, y + CAT_H - 2);
        y += CAT_H;

        p.setFont(QFont("monospace", theme::FontSizeSm));
        for (auto& entry : cat.entries) {
            bool focused = (flat == focused_idx_);
            if (focused) {
                QColor hi(theme::Green); hi.setAlpha(38);
                p.fillRect(QRect(listX, y, listW, ROW_H), hi);
            }
            p.setPen(focused ? QColor(theme::Green) : QColor(theme::Text));
            p.drawText(QRect(listX + 8, y, listW * 45 / 100, ROW_H),
                       Qt::AlignLeft | Qt::AlignVCenter, entry.keys);
            p.setPen(focused ? QColor(theme::Green).lighter(130) : QColor(theme::TextDim));
            p.drawText(QRect(listX + listW * 45 / 100, y, listW * 55 / 100, ROW_H),
                       Qt::AlignLeft | Qt::AlignVCenter, entry.description);
            y += ROW_H;
            flat++;
        }
    }

    p.resetTransform();
    p.setClipping(false);
    int totalH = y;
    if (totalH > listH) {
        int barH = std::max(20, listH * listH / totalH);
        int barY = listY + (listH - barH) * scroll_offset_ / std::max(1, totalH - listH);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(theme::BorderDim));
        p.drawRoundedRect(px + pw - margin + 6, barY, 4, barH, 2, 2);
    }
}

void KeybindingsOverlay::keyPressEvent(QKeyEvent* ke) {
    if (categories_.empty()) return;

    int ph     = std::min(700, height() - 48);
    int py     = (height() - ph) / 2;
    int margin = 24;
    int listY  = py + margin + 34 + 12;
    int listH  = ph - (listY - py) - margin;

    if (ke->key() == Qt::Key_Up) {
        focused_idx_ = std::max(0, focused_idx_ - 1);
        scrollToFocused(listH);
        update();
    } else if (ke->key() == Qt::Key_Down) {
        focused_idx_ = std::min(totalEntries() - 1, focused_idx_ + 1);
        scrollToFocused(listH);
        update();
    } else if (ke->key() == Qt::Key_Escape ||
               (ke->key() == Qt::Key_Slash && (ke->modifiers() & Qt::AltModifier))) {
        tc_->hideKeybindingsOverlay();
    }
}

void KeybindingsOverlay::wheelEvent(QWheelEvent* we) {
    int ph     = std::min(700, height() - 48);
    int py     = (height() - ph) / 2;
    int margin = 24;
    int listY  = py + margin + 34 + 12;
    int listH  = ph - (listY - py) - margin;

    int steps = we->angleDelta().y() / 40;
    scroll_offset_ = std::max(0, scroll_offset_ - steps * ROW_H);

    int totalH = 0;
    for (auto& cat : categories_)
        totalH += CAT_H + (int)cat.entries.size() * ROW_H;
    scroll_offset_ = std::min(scroll_offset_, std::max(0, totalH - listH));
    update();
}


// ---------------------------------------------------------------------------
// LayoutManagerOverlay
// ---------------------------------------------------------------------------

static constexpr int LAYOUT_ROW_H = 52;

LayoutManagerOverlay::LayoutManagerOverlay(TilingContainer* tc, QWidget* parent)
    : QWidget(parent), tc_(tc) {
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
    setFocusPolicy(Qt::StrongFocus);
    hide();
}

void LayoutManagerOverlay::refresh() {
    snapshots_.clear();
    QSettings s("rover_hmi", "layouts");
    int count = s.value("count", 0).toInt();
    for (int i = 0; i < count; ++i) {
        s.beginGroup(QString("layout_%1").arg(i));
        if (!s.value("deleted", false).toBool()) {
            LayoutSnapshot snap;
            snap.name     = s.value("name").toString();
            snap.saved_at = s.value("saved_at").toString();
            snap.json     = s.value("json").toString();
            snapshots_.push_back(snap);
        }
        s.endGroup();
    }
    focused_idx_   = std::min(focused_idx_, std::max(0, (int)snapshots_.size() - 1));
    scroll_offset_ = 0;
}

void LayoutManagerOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0, 0, 0, 210));

    int pw = std::min(680, width()  - 48);
    int ph = std::min(700, height() - 48);
    int px = (width()  - pw) / 2;
    int py = (height() - ph) / 2;

    QPainterPath panelPath;
    panelPath.addRoundedRect(QRect(px, py, pw, ph), 12, 12);
    p.fillPath(panelPath, QColor(theme::BgPanel));
    p.setPen(QPen(QColor(theme::Border), 2));
    p.drawRoundedRect(QRect(px, py, pw, ph), 12, 12);

    int margin = 24;
    // Title
    p.setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    p.setPen(QColor(theme::Text));
    p.drawText(QRect(px + margin, py + margin, pw - 2 * margin, 30),
               Qt::AlignLeft | Qt::AlignVCenter, "Saved Layouts");
    p.setFont(QFont("monospace", theme::FontSizeSm));
    p.setPen(QColor(theme::TextDim));
    p.drawText(QRect(px + margin, py + margin, pw - 2 * margin, 30),
               Qt::AlignRight | Qt::AlignVCenter, "S save  D delete  Enter load  Esc close");

    int sepY = py + margin + 34;
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(px + margin, sepY, px + pw - margin, sepY);

    int listX = px + margin;
    int listY = sepY + 12;
    int listW = pw - 2 * margin;
    int listH = ph - (listY - py) - margin;

    if (snapshots_.empty()) {
        p.setFont(QFont("monospace", theme::FontSize));
        p.setPen(QColor(theme::TextDim));
        p.drawText(QRect(listX, listY, listW, listH),
                   Qt::AlignCenter, "No saved layouts.\nPress S to save the current layout.");
        return;
    }

    p.setClipRect(listX, listY, listW, listH);
    p.translate(0, listY - scroll_offset_);

    int y = 0;
    for (int i = 0; i < (int)snapshots_.size(); ++i) {
        const auto& snap = snapshots_[i];
        bool focused = (i == focused_idx_);

        if (focused) {
            QColor hi(theme::Green); hi.setAlpha(38);
            p.fillRect(QRect(listX, y, listW, LAYOUT_ROW_H), hi);
        }

        // Name
        p.setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        p.setPen(focused ? QColor(theme::Green) : QColor(theme::Text));
        p.drawText(QRect(listX + 8, y + 6, listW - 16, 24),
                   Qt::AlignLeft | Qt::AlignVCenter, snap.name);

        // Date (right-aligned on name row)
        p.setFont(QFont("monospace", theme::FontSizeSm));
        p.setPen(focused ? QColor(theme::Green).lighter(130) : QColor(theme::TextDim));
        p.drawText(QRect(listX + 8, y + 6, listW - 16, 24),
                   Qt::AlignRight | Qt::AlignVCenter, snap.saved_at);

        // Separator
        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.drawLine(listX, y + LAYOUT_ROW_H - 1, listX + listW, y + LAYOUT_ROW_H - 1);

        y += LAYOUT_ROW_H;
    }

    p.resetTransform();
    p.setClipping(false);
    int totalH = (int)snapshots_.size() * LAYOUT_ROW_H;
    if (totalH > listH) {
        int barH = std::max(20, listH * listH / totalH);
        int barY = listY + (listH - barH) * scroll_offset_ / std::max(1, totalH - listH);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(theme::BorderDim));
        p.drawRoundedRect(px + pw - margin + 6, barY, 4, barH, 2, 2);
    }
}

void LayoutManagerOverlay::keyPressEvent(QKeyEvent* ke) {
    int ph     = std::min(700, height() - 48);
    int py     = (height() - ph) / 2;
    int margin = 24;
    int listY  = py + margin + 34 + 12;
    int listH  = ph - (listY - py) - margin;

    auto clampScroll = [&]() {
        int totalH = (int)snapshots_.size() * LAYOUT_ROW_H;
        int maxScroll = std::max(0, totalH - listH);
        // Keep focused row visible
        int rowY = focused_idx_ * LAYOUT_ROW_H;
        if (rowY - scroll_offset_ < 0)
            scroll_offset_ = rowY;
        else if (rowY + LAYOUT_ROW_H - scroll_offset_ > listH)
            scroll_offset_ = rowY + LAYOUT_ROW_H - listH;
        scroll_offset_ = std::clamp(scroll_offset_, 0, maxScroll);
    };

    if (ke->key() == Qt::Key_Up) {
        focused_idx_ = std::max(0, focused_idx_ - 1);
        clampScroll();
        update();
    } else if (ke->key() == Qt::Key_Down) {
        focused_idx_ = std::min((int)snapshots_.size() - 1, focused_idx_ + 1);
        clampScroll();
        update();
    } else if (ke->key() == Qt::Key_Return || ke->key() == Qt::Key_Enter) {
        if (!snapshots_.empty()) tc_->loadLayout(focused_idx_);
    } else if (ke->key() == Qt::Key_S) {
        tc_->saveCurrentLayout();
        refresh();
        focused_idx_ = std::max(0, (int)snapshots_.size() - 1);
        update();
    } else if (ke->key() == Qt::Key_D) {
        if (!snapshots_.empty()) {
            tc_->deleteLayout(focused_idx_);
            refresh();
            focused_idx_ = std::min(focused_idx_, std::max(0, (int)snapshots_.size() - 1));
            update();
        }
    } else if (ke->key() == Qt::Key_Escape) {
        tc_->hideLayoutManagerOverlay();
    }
}

void LayoutManagerOverlay::wheelEvent(QWheelEvent* we) {
    int ph     = std::min(700, height() - 48);
    int py     = (height() - ph) / 2;
    int margin = 24;
    int listY  = py + margin + 34 + 12;
    int listH  = ph - (listY - py) - margin;

    int steps = we->angleDelta().y() / 40;
    scroll_offset_ -= steps * LAYOUT_ROW_H;
    int totalH = (int)snapshots_.size() * LAYOUT_ROW_H;
    scroll_offset_ = std::clamp(scroll_offset_, 0, std::max(0, totalH - listH));
    update();
}


// ---------------------------------------------------------------------------
// TilingContainer
// ---------------------------------------------------------------------------

TilingContainer::TilingContainer(QWidget* parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    qApp->installEventFilter(this);
}

TilingContainer::~TilingContainer() {
    for (auto* n : all_nodes_) delete n;
}

void TilingContainer::addPanel(const std::string& title, QWidget* content,
                                const std::string& layout_hint,
                                bool default_visible,
                                std::function<void(bool)> on_toggle,
                                std::vector<std::pair<std::string,std::string>> module_keybinds) {
    auto* panel = new TilePanel(title, content, this);

    connect(panel, &TilePanel::clicked, [this, panel]() { setFocusedPanel(panel); });
    connect(panel, &TilePanel::hovered, [this, panel]() {
        if (drag_mode_ == DragMode::None) setFocusedPanel(panel);
    });

    panels_.push_back({panel, layout_hint, default_visible, on_toggle, std::move(module_keybinds)});
}

// Build a vertical (vertical=true) or horizontal column of panels
DwindleNode* TilingContainer::buildColumn(std::vector<TilePanel*>& panels, bool vertical) {
    if (panels.empty()) return nullptr;

    if (panels.size() == 1) {
        auto* leaf = new DwindleNode();
        leaf->panel = panels[0];
        all_nodes_.push_back(leaf);
        return leaf;
    }

    auto* node = new DwindleNode();
    node->isNode = true;
    node->splitTop = vertical;  // true = top/bottom stacking
    all_nodes_.push_back(node);

    // First panel → first child
    auto* leaf0 = new DwindleNode();
    leaf0->panel = panels[0];
    leaf0->parent = node;
    all_nodes_.push_back(leaf0);

    // Remaining panels → second child (recurse)
    std::vector<TilePanel*> rest(panels.begin() + 1, panels.end());
    auto* rest_node = buildColumn(rest, vertical);
    rest_node->parent = node;

    node->children[0] = leaf0;
    node->children[1] = rest_node;
    return node;
}

void TilingContainer::finalize() {
    auto* outer_layout = new QHBoxLayout(this);
    outer_layout->setContentsMargins(0, 0, 0, 0);
    outer_layout->setSpacing(0);

    sidebar_ = new ModuleSidebar(this);
    outer_layout->addWidget(sidebar_);

    tiling_area_ = new QWidget(this);
    outer_layout->addWidget(tiling_area_, 1);

    // Wire up sidebar toggles: each calls dwindleAdd/Remove + any module callback
    for (auto& pi : panels_) {
        pi.panel->setParent(tiling_area_);

        auto* self = this;
        auto* panel = pi.panel;
        auto module_toggle = pi.on_toggle;

        std::function<void(bool)> vis_cb = [self, panel, module_toggle](bool visible) {
            if (module_toggle) module_toggle(visible);
            if (visible) {
                panel->setVisible(true);
                self->dwindleAdd(panel);
            } else {
                self->dwindleRemove(panel);
                panel->setVisible(false);
            }
        };

        sidebar_->addModule(pi.panel->title(), pi.panel, pi.default_visible, vis_cb);
    }

    // -----------------------------------------------------------------------
    // Build initial dwindle tree for default-visible panels.
    // Groups: hint=="main"|"left" → left column
    //         hint=="right"       → right column
    //         anything else       → bottom row
    // -----------------------------------------------------------------------
    std::vector<TilePanel*> left_panels, right_panels, bottom_panels;
    for (auto& pi : panels_) {
        if (!pi.default_visible) continue;
        if (pi.hint == "main" || pi.hint == "left")
            left_panels.push_back(pi.panel);
        else if (pi.hint == "right")
            right_panels.push_back(pi.panel);
        else
            bottom_panels.push_back(pi.panel);
    }

    auto* left_tree   = buildColumn(left_panels,   true);   // vertical stack
    auto* right_tree  = buildColumn(right_panels,  true);   // vertical stack
    auto* bottom_tree = buildColumn(bottom_panels, false);  // horizontal row

    // Combine left+right into a horizontal split
    DwindleNode* top_tree = nullptr;
    if (left_tree && right_tree) {
        auto* lr = new DwindleNode();
        lr->isNode = true;
        lr->splitTop = false;   // left/right
        lr->splitRatio = 1.0f;  // 50/50
        lr->children[0] = left_tree;
        lr->children[1] = right_tree;
        left_tree->parent  = lr;
        right_tree->parent = lr;
        all_nodes_.push_back(lr);
        top_tree = lr;
    } else {
        top_tree = left_tree ? left_tree : right_tree;
    }

    // Combine top+bottom into a vertical split (80% / 20%)
    if (top_tree && bottom_tree) {
        auto* tb = new DwindleNode();
        tb->isNode = true;
        tb->splitTop = true;    // top/bottom
        tb->splitRatio = 1.6f;  // 80% top, 20% bottom
        tb->children[0] = top_tree;
        tb->children[1] = bottom_tree;
        top_tree->parent    = tb;
        bottom_tree->parent = tb;
        all_nodes_.push_back(tb);
        root_ = tb;
    } else {
        root_ = top_tree ? top_tree : bottom_tree;
    }

    // Recalculate when tiling_area_ resizes
    tiling_area_->installEventFilter(this);

    // -----------------------------------------------------------------------
    // Overlays
    // -----------------------------------------------------------------------
    keybindings_overlay_ = new KeybindingsOverlay(this, window());
    layout_overlay_      = new LayoutManagerOverlay(this, window());

    base_categories_ = {
        { "Focus",   { {"Alt + Arrow",          "Focus adjacent panel"  },
                       {"Alt + Tab",            "Cycle focus"           } }},
        { "Layout",  { {"Alt + J",              "Toggle split direction" },
                       {"Alt+Shift+Arrow",      "Resize panel (hold)"   },
                       {"Alt + X + drag",       "Free resize"           },
                       {"Alt+Ctrl+Shift+Arrow", "Swap panel"            },
                       {"Alt + Z + drag",       "Drag to swap"          } }},
        { "Modules", { {"Alt + 1 … 9",          "Toggle module"         },
                       {"Sidebar checkbox",     "Toggle visibility"     } }},
        { "Layouts", { {"Alt + P",              "Open layout manager"   },
                       {"S",                   "Save current layout"   },
                       {"D",                   "Delete selected"       },
                       {"Enter",               "Load selected"         } }},
        { "Help",    { {"Alt + /",              "Toggle this overlay"   },
                       {"Escape",              "Dismiss overlay"       } }},
    };
    keybindings_overlay_->setCategories(base_categories_);

    // -----------------------------------------------------------------------
    // Keyboard shortcuts
    // -----------------------------------------------------------------------
    auto bind = [this](const char* seq, std::function<void()> fn, bool repeat = false) {
        auto* sc = new QShortcut(QKeySequence(seq), this);
        sc->setContext(Qt::ApplicationShortcut);
        sc->setAutoRepeat(repeat);
        connect(sc, &QShortcut::activated, fn);
    };

    bind("Alt+Left",  [this]() { if (anyOverlayVisible()) return; focusDirection(-1, 0); });
    bind("Alt+Right", [this]() { if (anyOverlayVisible()) return; focusDirection(1, 0); });
    bind("Alt+Up",    [this]() { if (anyOverlayVisible()) return; focusDirection(0, -1); });
    bind("Alt+Down",  [this]() { if (anyOverlayVisible()) return; focusDirection(0, 1); });

    bind("Alt+Shift+Right", [this]() { if (anyOverlayVisible()) return; resizeFocused(1, 0); },  true);
    bind("Alt+Shift+Left",  [this]() { if (anyOverlayVisible()) return; resizeFocused(-1, 0); }, true);
    bind("Alt+Shift+Up",    [this]() { if (anyOverlayVisible()) return; resizeFocused(0, -1); }, true);
    bind("Alt+Shift+Down",  [this]() { if (anyOverlayVisible()) return; resizeFocused(0, 1); },  true);

    bind("Alt+Ctrl+Shift+Left",  [this]() { if (anyOverlayVisible()) return; swapWithFocused(-1, 0); });
    bind("Alt+Ctrl+Shift+Right", [this]() { if (anyOverlayVisible()) return; swapWithFocused(1, 0); });
    bind("Alt+Ctrl+Shift+Up",    [this]() { if (anyOverlayVisible()) return; swapWithFocused(0, -1); });
    bind("Alt+Ctrl+Shift+Down",  [this]() { if (anyOverlayVisible()) return; swapWithFocused(0, 1); });

    bind("Alt+Tab", [this]() { if (anyOverlayVisible()) return; focusNext(); });

    bind("Alt+J", [this]() {
        if (anyOverlayVisible()) return;
        if (!focused_panel_) return;
        auto* leaf = getLeafFor(focused_panel_);
        if (leaf && leaf->parent) {
            leaf->parent->splitTop = !leaf->parent->splitTop;
            recalculate();
        }
    });

    bind("Alt+/", [this]() { toggleKeybindingsOverlay(); });
    bind("Alt+P", [this]() { toggleLayoutManagerOverlay(); });

    for (int i = 0; i < 9; ++i) {
        auto* sc = new QShortcut(QKeySequence(QString("Alt+%1").arg(i + 1)), this);
        sc->setContext(Qt::ApplicationShortcut);
        int idx = i;
        connect(sc, &QShortcut::activated, [this, idx]() {
            if (anyOverlayVisible()) return;
            sidebar_->toggleModule(idx);
        });
    }

    // Focus first visible panel
    for (auto& pi : panels_) {
        if (pi.panel->isVisible()) {
            setFocusedPanel(pi.panel);
            break;
        }
    }
}


// ---------------------------------------------------------------------------
// Dwindle tree operations
// ---------------------------------------------------------------------------

void TilingContainer::dwindleAdd(TilePanel* panel) {
    auto* newLeaf = new DwindleNode();
    newLeaf->panel = panel;
    all_nodes_.push_back(newLeaf);

    if (!root_) {
        root_ = newLeaf;
        recalculate();
        setFocusedPanel(panel);
        return;
    }

    // Split the focused leaf (or the first available leaf)
    DwindleNode* target = focused_panel_ ? getLeafFor(focused_panel_) : nullptr;
    if (!target) {
        target = root_;
        while (target->isNode && target->children[0]) target = target->children[0];
    }

    // Create an internal node to replace target
    auto* newParent = new DwindleNode();
    newParent->isNode = true;
    newParent->parent = target->parent;
    // Smart split: taller box → top/bottom, wider box → left/right
    newParent->splitTop = (target->box.height() >= target->box.width());

    if (target->parent) {
        if (target->parent->children[0] == target)
            target->parent->children[0] = newParent;
        else
            target->parent->children[1] = newParent;
    } else {
        root_ = newParent;
    }

    target->parent  = newParent;
    newLeaf->parent = newParent;
    newParent->children[0] = target;
    newParent->children[1] = newLeaf;
    all_nodes_.push_back(newParent);

    recalculate();
    setFocusedPanel(panel);
}

void TilingContainer::dwindleRemove(TilePanel* panel) {
    auto* leaf = getLeafFor(panel);
    if (!leaf) return;

    if (!leaf->parent) {
        // Last panel in the tree
        all_nodes_.erase(std::remove(all_nodes_.begin(), all_nodes_.end(), leaf), all_nodes_.end());
        delete leaf;
        root_ = nullptr;
        return;
    }

    auto* parent  = leaf->parent;
    auto* sibling = leaf->sibling();

    // Sibling takes parent's place
    sibling->parent = parent->parent;
    if (parent->parent) {
        if (parent->parent->children[0] == parent)
            parent->parent->children[0] = sibling;
        else
            parent->parent->children[1] = sibling;
    } else {
        root_ = sibling;
    }

    all_nodes_.erase(std::remove(all_nodes_.begin(), all_nodes_.end(), leaf),   all_nodes_.end());
    all_nodes_.erase(std::remove(all_nodes_.begin(), all_nodes_.end(), parent), all_nodes_.end());
    delete leaf;
    delete parent;

    recalculate();

    // Refocus on sibling's first leaf if we removed the focused panel
    if (focused_panel_ == panel) {
        DwindleNode* next = sibling;
        while (next && next->isNode) next = next->children[0];
        if (next && next->panel) setFocusedPanel(next->panel);
    }
}

void TilingContainer::dwindleSwap(TilePanel* a, TilePanel* b) {
    auto* nodeA = getLeafFor(a);
    auto* nodeB = getLeafFor(b);
    if (!nodeA || !nodeB) return;
    std::swap(nodeA->panel, nodeB->panel);
    recalculate();
}

DwindleNode* TilingContainer::getLeafFor(TilePanel* panel) {
    return (root_ && panel) ? root_->leafFor(panel) : nullptr;
}

void TilingContainer::recalculate() {
    if (!root_ || !tiling_area_) return;
    QRect area = tiling_area_->rect();
    if (area.isEmpty()) return;
    root_->box = area;
    root_->recalcSizePosRecursive(PANEL_GAP);
}


// ---------------------------------------------------------------------------
// Event filter — resize detection + Alt+Z/X drag modes
// ---------------------------------------------------------------------------

bool TilingContainer::eventFilter(QObject* obj, QEvent* event) {
    // Recalculate on tiling area resize; keep overlay geometries in sync
    if (obj == tiling_area_ && event->type() == QEvent::Resize) {
        recalculate();
        if (keybindings_overlay_ && keybindings_overlay_->isVisible())
            keybindings_overlay_->setGeometry(tiling_area_->geometry());
        if (layout_overlay_ && layout_overlay_->isVisible())
            layout_overlay_->setGeometry(tiling_area_->geometry());
        return false;
    }

    // Enter drag/resize mode — blocked while any overlay is open
    if (event->type() == QEvent::KeyPress) {
        auto* ke = static_cast<QKeyEvent*>(event);

        if (anyOverlayVisible()) return false;

        if (ke->key() == Qt::Key_Z && (ke->modifiers() & Qt::AltModifier)
            && !ke->isAutoRepeat() && drag_mode_ == DragMode::None) {
            // Embedded X11 windows (e.g. RViz) swallow hover events so
            // focused_panel_ may be stale. Re-derive from cursor position.
            if (tiling_area_) {
                QPoint local = tiling_area_->mapFromGlobal(QCursor::pos());
                for (auto& pi : panels_) {
                    if (pi.panel->isVisible() && pi.panel->geometry().contains(local)) {
                        setFocusedPanel(pi.panel);
                        break;
                    }
                }
            }
            enterMoveMode();
            return true;
        }
        if (ke->key() == Qt::Key_X && (ke->modifiers() & Qt::AltModifier)
            && !ke->isAutoRepeat() && drag_mode_ == DragMode::None) {
            // Same cursor-position fix for resize mode.
            if (tiling_area_) {
                QPoint local = tiling_area_->mapFromGlobal(QCursor::pos());
                for (auto& pi : panels_) {
                    if (pi.panel->isVisible() && pi.panel->geometry().contains(local)) {
                        setFocusedPanel(pi.panel);
                        break;
                    }
                }
            }
            drag_mode_ = DragMode::Resize;
            last_mouse_global_ = QCursor::pos();
            setCursor(Qt::SizeFDiagCursor);
            return true;
        }
    }

    // Exit drag/resize mode
    if (event->type() == QEvent::KeyRelease) {
        auto* ke = static_cast<QKeyEvent*>(event);
        if (ke->isAutoRepeat()) return false;
        if (drag_mode_ != DragMode::None &&
            (ke->key() == Qt::Key_Z || ke->key() == Qt::Key_X || ke->key() == Qt::Key_Alt)) {
            exitDragMode();
            return true;
        }
    }

    // If the window loses focus while a drag/move is in progress, the KeyRelease
    // event is never delivered — exit drag mode here so panels don't stay disabled.
    if (event->type() == QEvent::WindowDeactivate && drag_mode_ != DragMode::None) {
        exitDragMode();
        return false;
    }

    // Mouse move during drag/resize
    if (event->type() == QEvent::MouseMove && drag_mode_ != DragMode::None) {
        auto* me = static_cast<QMouseEvent*>(event);
        QPoint globalPos = me->globalPos();
        QPoint delta     = globalPos - last_mouse_global_;

        if (drag_mode_ == DragMode::Move) {
            // Move the floating overlay
            if (drag_overlay_) {
                QWidget* topLevel = window();
                drag_overlay_->move(topLevel->mapFromGlobal(globalPos - drag_grab_offset_));
                drag_overlay_->raise();
            }

            // Highlight panel under cursor as drop target
            TilePanel* newTarget = nullptr;
            if (tiling_area_) {
                QPoint local = tiling_area_->mapFromGlobal(globalPos);
                for (auto& pi : panels_) {
                    if (pi.panel == drag_source_ || !pi.panel->isVisible()) continue;
                    if (pi.panel->geometry().contains(local)) { newTarget = pi.panel; break; }
                }
            }
            if (newTarget != drag_target_) {
                if (drag_target_) drag_target_->setDropTarget(false);
                drag_target_ = newTarget;
                if (drag_target_) drag_target_->setDropTarget(true);
            }

        } else if (drag_mode_ == DragMode::Resize && focused_panel_) {
            // Adjust splitRatio of closest ancestor with matching orientation
            auto* leaf = getLeafFor(focused_panel_);
            if (leaf) {
                if (std::abs(delta.x()) > 0) {
                    for (auto* cur = leaf; cur->parent; cur = cur->parent) {
                        auto* p = cur->parent;
                        if (!p->splitTop) {
                            float rel = (float)delta.x() * 2.0f / std::max(1, p->box.width());
                            p->splitRatio = std::clamp(p->splitRatio + rel, 0.1f, 1.9f);
                            recalculate();
                            break;
                        }
                    }
                }
                if (std::abs(delta.y()) > 0) {
                    for (auto* cur = leaf; cur->parent; cur = cur->parent) {
                        auto* p = cur->parent;
                        if (p->splitTop) {
                            float rel = (float)delta.y() * 2.0f / std::max(1, p->box.height());
                            p->splitRatio = std::clamp(p->splitRatio + rel, 0.1f, 1.9f);
                            recalculate();
                            break;
                        }
                    }
                }
            }
        }

        last_mouse_global_ = globalPos;
        return true;
    }

    return false;
}


// ---------------------------------------------------------------------------
// Drag mode enter/exit
// ---------------------------------------------------------------------------

void TilingContainer::enterMoveMode() {
    if (!focused_panel_) return;

    drag_mode_   = DragMode::Move;
    drag_source_ = focused_panel_;
    drag_target_ = nullptr;
    last_mouse_global_ = QCursor::pos();

    QPixmap snapshot = drag_source_->grab();
    QPixmap scaled   = snapshot.scaled(snapshot.size() * 0.9,
                                       Qt::KeepAspectRatio, Qt::SmoothTransformation);

    QPoint panelTopLeft = drag_source_->mapToGlobal(QPoint(0, 0));
    drag_grab_offset_   = (last_mouse_global_ - panelTopLeft) * 0.9;

    QWidget* topLevel = window();
    drag_overlay_     = new DragOverlay(topLevel);
    drag_overlay_->setSnapshot(scaled);
    drag_overlay_->move(topLevel->mapFromGlobal(last_mouse_global_ - drag_grab_offset_));
    drag_overlay_->show();
    drag_overlay_->raise();

    drag_source_->setEnabled(false);
    setCursor(Qt::ClosedHandCursor);
}

void TilingContainer::exitDragMode() {
    if (drag_mode_ == DragMode::Move) {
        if (drag_source_ && drag_target_ && drag_source_ != drag_target_)
            dwindleSwap(drag_source_, drag_target_);

        for (auto& pi : panels_) pi.panel->setDropTarget(false);

        if (drag_overlay_) {
            drag_overlay_->hide();
            drag_overlay_->deleteLater();
            drag_overlay_ = nullptr;
        }
        drag_target_ = nullptr;
    }

    // Always re-enable the source panel — enterMoveMode() disables it and
    // we must undo that regardless of how drag mode is exited (key release,
    // window focus loss, etc.).
    if (drag_source_) {
        drag_source_->setEnabled(true);
        drag_source_ = nullptr;
    }

    drag_mode_ = DragMode::None;
    unsetCursor();
}


// ---------------------------------------------------------------------------
// Overlay helpers
// ---------------------------------------------------------------------------

bool TilingContainer::anyOverlayVisible() const {
    return (keybindings_overlay_ && keybindings_overlay_->isVisible()) ||
           (layout_overlay_      && layout_overlay_->isVisible());
}

void TilingContainer::toggleKeybindingsOverlay() {
    if (drag_mode_ != DragMode::None) return;
    if (keybindings_overlay_->isVisible()) {
        hideKeybindingsOverlay();
        return;
    }
    // Build categories: focused module's keybinds first, then global
    std::vector<KeybindCategory> cats;
    if (focused_panel_) {
        for (auto& pi : panels_) {
            if (pi.panel == focused_panel_ && !pi.module_keybinds.empty()) {
                KeybindCategory modCat;
                modCat.name = QString::fromStdString(focused_panel_->title());
                for (auto& [k, d] : pi.module_keybinds)
                    modCat.entries.push_back({QString::fromStdString(k),
                                              QString::fromStdString(d)});
                cats.push_back(std::move(modCat));
                break;
            }
        }
    }
    cats.insert(cats.end(), base_categories_.begin(), base_categories_.end());
    keybindings_overlay_->setCategories(std::move(cats));

    keybindings_overlay_->setGeometry(tiling_area_->geometry());
    keybindings_overlay_->raise();
    keybindings_overlay_->QWidget::show();
    keybindings_overlay_->setFocus();
}

void TilingContainer::hideKeybindingsOverlay() {
    keybindings_overlay_->hide();
}

void TilingContainer::toggleLayoutManagerOverlay() {
    if (drag_mode_ != DragMode::None) return;
    if (layout_overlay_->isVisible()) {
        hideLayoutManagerOverlay();
        return;
    }
    layout_overlay_->refresh();
    layout_overlay_->setGeometry(tiling_area_->geometry());
    layout_overlay_->raise();
    layout_overlay_->QWidget::show();
    layout_overlay_->setFocus();
}

void TilingContainer::hideLayoutManagerOverlay() {
    layout_overlay_->hide();
}


// ---------------------------------------------------------------------------
// Layout persistence
// ---------------------------------------------------------------------------

QJsonObject TilingContainer::serializeTree(DwindleNode* node) const {
    QJsonObject obj;
    obj["isNode"] = node->isNode;
    if (node->isNode) {
        obj["splitTop"]   = node->splitTop;
        obj["splitRatio"] = (double)node->splitRatio;
        QJsonArray children;
        if (node->children[0]) children.append(serializeTree(node->children[0]));
        if (node->children[1]) children.append(serializeTree(node->children[1]));
        obj["children"] = children;
    } else {
        obj["panel"] = node->panel ? QString::fromStdString(node->panel->title()) : "";
    }
    return obj;
}

DwindleNode* TilingContainer::deserializeTree(const QJsonObject& obj) {
    auto* node = new DwindleNode();
    all_nodes_.push_back(node);
    node->isNode = obj["isNode"].toBool();
    if (node->isNode) {
        node->splitTop   = obj["splitTop"].toBool();
        node->splitRatio = (float)obj["splitRatio"].toDouble(1.0);
        auto children    = obj["children"].toArray();
        if (children.size() >= 2) {
            node->children[0] = deserializeTree(children[0].toObject());
            node->children[1] = deserializeTree(children[1].toObject());
            node->children[0]->parent = node;
            node->children[1]->parent = node;
        }
    } else {
        QString title = obj["panel"].toString();
        for (auto& pi : panels_) {
            if (QString::fromStdString(pi.panel->title()) == title) {
                node->panel = pi.panel;
                break;
            }
        }
    }
    return node;
}

void TilingContainer::saveCurrentLayout() {
    QJsonObject layout;
    QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm");
    layout["name"]     = ts;
    layout["saved_at"] = ts;

    if (root_) layout["tree"] = serializeTree(root_);

    QJsonArray hidden;
    for (auto& pi : panels_)
        if (!pi.panel->isVisible())
            hidden.append(QString::fromStdString(pi.panel->title()));
    layout["hidden"] = hidden;

    QSettings s("rover_hmi", "layouts");
    int count = s.value("count", 0).toInt();
    s.beginGroup(QString("layout_%1").arg(count));
    s.setValue("name",     ts);
    s.setValue("saved_at", ts);
    s.setValue("json",     QString(QJsonDocument(layout).toJson(QJsonDocument::Compact)));
    s.setValue("deleted",  false);
    s.endGroup();
    s.setValue("count", count + 1);
}

void TilingContainer::loadLayout(int index) {
    // index is into the non-deleted snapshot list — resolve to settings key
    QSettings s("rover_hmi", "layouts");
    int count = s.value("count", 0).toInt();
    int found = 0;
    int settingsIdx = -1;
    for (int i = 0; i < count; ++i) {
        s.beginGroup(QString("layout_%1").arg(i));
        bool deleted = s.value("deleted", false).toBool();
        s.endGroup();
        if (!deleted) {
            if (found == index) { settingsIdx = i; break; }
            found++;
        }
    }
    if (settingsIdx < 0) return;

    s.beginGroup(QString("layout_%1").arg(settingsIdx));
    QString json = s.value("json").toString();
    s.endGroup();

    QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8());
    if (doc.isNull()) return;
    QJsonObject layout = doc.object();

    // Collect hidden panel titles
    QSet<QString> hidden_set;
    for (auto h : layout["hidden"].toArray())
        hidden_set.insert(h.toString());

    // Clear tree
    for (auto* n : all_nodes_) delete n;
    all_nodes_.clear();
    root_         = nullptr;
    focused_panel_ = nullptr;

    // Set panel visibility
    std::vector<std::string> visible_titles;
    for (auto& pi : panels_) {
        bool vis = !hidden_set.contains(QString::fromStdString(pi.panel->title()));
        pi.panel->setVisible(vis);
        if (vis) visible_titles.push_back(pi.panel->title());
    }
    sidebar_->syncCheckboxes(visible_titles);

    // Reconstruct tree
    if (layout.contains("tree") && !layout["tree"].toObject().isEmpty())
        root_ = deserializeTree(layout["tree"].toObject());

    recalculate();

    // Focus first visible panel
    for (auto& pi : panels_) {
        if (pi.panel->isVisible()) { setFocusedPanel(pi.panel); break; }
    }

    hideLayoutManagerOverlay();
}

void TilingContainer::deleteLayout(int index) {
    QSettings s("rover_hmi", "layouts");
    int count = s.value("count", 0).toInt();
    int found = 0;
    for (int i = 0; i < count; ++i) {
        s.beginGroup(QString("layout_%1").arg(i));
        bool deleted = s.value("deleted", false).toBool();
        if (!deleted) {
            if (found == index) {
                s.setValue("deleted", true);
                s.endGroup();
                return;
            }
            found++;
        }
        s.endGroup();
    }
}


// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void TilingContainer::setFocusedPanel(TilePanel* panel) {
    if (focused_panel_ == panel) return;
    focused_panel_ = panel;
    for (auto& pi : panels_)
        pi.panel->setFocused(pi.panel == panel);
}

void TilingContainer::focusNext() {
    bool found = false;
    for (auto& pi : panels_) {
        if (found && pi.panel->isVisible()) { setFocusedPanel(pi.panel); return; }
        if (pi.panel == focused_panel_) found = true;
    }
    for (auto& pi : panels_) {
        if (pi.panel->isVisible()) { setFocusedPanel(pi.panel); return; }
    }
}

void TilingContainer::focusPrev() {
    TilePanel* prev = nullptr;
    for (auto& pi : panels_) {
        if (pi.panel == focused_panel_ && prev) { setFocusedPanel(prev); return; }
        if (pi.panel->isVisible()) prev = pi.panel;
    }
    for (auto it = panels_.rbegin(); it != panels_.rend(); ++it) {
        if (it->panel->isVisible()) { setFocusedPanel(it->panel); return; }
    }
}

// Navigate by geometry center: find nearest panel in the given direction
void TilingContainer::focusDirection(int dx, int dy) {
    if (!focused_panel_) return;
    QPoint cur = focused_panel_->geometry().center();

    TilePanel* best = nullptr;
    double bestDist = 1e18;

    for (auto& pi : panels_) {
        auto* p = pi.panel;
        if (p == focused_panel_ || !p->isVisible()) continue;
        QPoint pc = p->geometry().center();
        int relX = pc.x() - cur.x();
        int relY = pc.y() - cur.y();

        if (dx > 0 && relX <= 0) continue;
        if (dx < 0 && relX >= 0) continue;
        if (dy > 0 && relY <= 0) continue;
        if (dy < 0 && relY >= 0) continue;

        double dist = (double)(relX * relX + relY * relY);
        if (dist < bestDist) { bestDist = dist; best = p; }
    }

    if (best) setFocusedPanel(best);
}

void TilingContainer::swapWithFocused(int dx, int dy) {
    if (!focused_panel_) return;
    QPoint cur = focused_panel_->geometry().center();

    TilePanel* target = nullptr;
    double bestDist = 1e18;

    for (auto& pi : panels_) {
        auto* p = pi.panel;
        if (p == focused_panel_ || !p->isVisible()) continue;
        QPoint pc = p->geometry().center();
        int relX = pc.x() - cur.x();
        int relY = pc.y() - cur.y();

        if (dx > 0 && relX <= 0) continue;
        if (dx < 0 && relX >= 0) continue;
        if (dy > 0 && relY <= 0) continue;
        if (dy < 0 && relY >= 0) continue;

        double dist = (double)(relX * relX + relY * relY);
        if (dist < bestDist) { bestDist = dist; target = p; }
    }

    if (target) dwindleSwap(focused_panel_, target);
}

// Adjust splitRatio of the nearest ancestor with the matching split orientation
void TilingContainer::resizeFocused(int dx, int dy) {
    if (!focused_panel_) return;
    auto* leaf = getLeafFor(focused_panel_);
    if (!leaf) return;

    if (dx != 0) {
        for (auto* cur = leaf; cur->parent; cur = cur->parent) {
            auto* p = cur->parent;
            if (!p->splitTop) {
                float delta = (p->children[0] == cur) ? dx * RESIZE_RATIO_STEP
                                                       : -dx * RESIZE_RATIO_STEP;
                p->splitRatio = std::clamp(p->splitRatio + delta, 0.1f, 1.9f);
                recalculate();
                return;
            }
        }
    }

    if (dy != 0) {
        for (auto* cur = leaf; cur->parent; cur = cur->parent) {
            auto* p = cur->parent;
            if (p->splitTop) {
                float delta = (p->children[0] == cur) ? dy * RESIZE_RATIO_STEP
                                                       : -dy * RESIZE_RATIO_STEP;
                p->splitRatio = std::clamp(p->splitRatio + delta, 0.1f, 1.9f);
                recalculate();
                return;
            }
        }
    }
}
