// tiling_container.cpp — Hyprland-style dwindle tiling container.
// Architecture and keybindings are documented in tiling_container.h.

#include <rover_hmi_core/tiling_container.h>
#include <rover_hmi_core/catppuccin.h>

#include <QApplication>
#include <QMouseEvent>
#include <QCursor>
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QStringList>
#include <algorithm>
#include <cmath>


// ---- Tunables (px unless noted) --------------------------------------------
// Saved layouts: repo-tracked JSON files, see layout_store.h

// Tiling geometry
static constexpr int SIDEBAR_WIDTH     = 180;  // initial; user can drag the right edge
static constexpr int SIDEBAR_MIN_WIDTH = 140;
static constexpr int SIDEBAR_MAX_WIDTH = 420;
static constexpr int SIDEBAR_GRIP_PX   = 8;    // grab zone along the right edge
static constexpr int PANEL_GAP       = 3;
static constexpr int MIN_PANE_WIDTH  = 80;   // splits never squeeze a side below these
static constexpr int MIN_PANE_HEIGHT = 60;
static const char*   SIDEBAR_BG      = "#080808";  // other colors: catppuccin.h

// splitRatio: 1.0 = 50/50 (see DwindleNode); clamped so a side can't collapse
static constexpr float SPLIT_RATIO_MIN        = 0.1f;
static constexpr float SPLIT_RATIO_MAX        = 1.9f;
static constexpr float RESIZE_RATIO_STEP      = 0.08f;  // per Alt+Shift+Arrow press
static constexpr float TOP_BOTTOM_SPLIT_RATIO = 1.6f;   // initial top/bottom = 80/20

// TilePanel chrome
static constexpr int PANEL_PADDING    = 10;  // content inset
static constexpr int PANEL_TITLEBAR_H = 60;  // painted title strip

// Modal overlays (keybindings + layout manager)
static constexpr int OVERLAY_MAX_W        = 680;
static constexpr int OVERLAY_MAX_H        = 700;
static constexpr int OVERLAY_SCREEN_PAD   = 48;   // min gap to widget edge
static constexpr int OVERLAY_MARGIN       = 24;
static constexpr int OVERLAY_RADIUS       = 12;
static constexpr int OVERLAY_SCRIM_ALPHA  = 210;
static constexpr int OVERLAY_LIST_TOP_PAD = 12;   // gap under header rule
static constexpr int KEYBIND_HEADER_H     = 34;   // title line
static constexpr int LAYOUT_HEADER_H      = 52;   // title + hint lines
static constexpr int KEYBIND_CAT_H        = 38;   // category heading row
static constexpr int KEYBIND_ROW_H        = 30;
static constexpr int LAYOUT_ROW_H         = 52;

// Lets the wrapping name label toggle its checkbox — QCheckBox cannot
// word-wrap its own text, so module names live in a separate QLabel.
class ToggleOnClick : public QObject {
public:
    ToggleOnClick(QCheckBox* box, QObject* parent) : QObject(parent), box_(box) {}
    bool eventFilter(QObject*, QEvent* ev) override {
        if (ev->type() == QEvent::MouseButtonRelease) {
            box_->toggle();
            return true;
        }
        return false;
    }
private:
    QCheckBox* box_;
};


// ---------------------------------------------------------------------------
// DwindleNode
// ---------------------------------------------------------------------------

// DFS for the leaf holding panel `p`; nullptr if absent
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
        // max(min()) not std::clamp: bounds may cross when box < 2*MIN_PANE_WIDTH
        w0 = std::max(MIN_PANE_WIDTH, std::min(w0, box.width() - MIN_PANE_WIDTH));
        c0->box = QRect(box.x(), box.y(), w0, box.height());
        c1->box = QRect(box.x() + w0, box.y(), box.width() - w0, box.height());
    } else {
        // Top/bottom split
        int h0 = (int)(box.height() / 2.0f * splitRatio);
        h0 = std::max(MIN_PANE_HEIGHT, std::min(h0, box.height() - MIN_PANE_HEIGHT));
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
    // top margin reserves room for the title strip painted in paintEvent()
    layout->setContentsMargins(PANEL_PADDING, PANEL_TITLEBAR_H,
                               PANEL_PADDING, PANEL_PADDING);
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

    QRectF titleRect(bw + 12, bw + 6, width() - 2 * bw - 24, PANEL_TITLEBAR_H - 12);
    QFont font("monospace");
    font.setPixelSize(theme::px(theme::FontSizeTitle));
    font.setBold(true);
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
    setMouseTracking(true);  // hover cursor feedback on the resize edge
    layout_ = new QVBoxLayout(this);
    layout_->setContentsMargins(10, 12, 10, 12);
    layout_->setSpacing(4);

    auto* title = new QLabel("Modules");
    title->setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    title->setStyleSheet(QString("color: %1;").arg(theme::Text));
    layout_->addWidget(title);

    section_indicator_ = new QLabel("");
    section_indicator_->setFont(QFont("monospace", theme::FontSizeSm));
    section_indicator_->setStyleSheet(
        QString("color: %1; padding-bottom: 4px;").arg(theme::Cyan));
    layout_->addWidget(section_indicator_);

    auto* sep = new QWidget();
    sep->setFixedHeight(1);
    sep->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
    layout_->addWidget(sep);

    layout_->addStretch();

    // Pinned below the stretch so it always sits at the sidebar's bottom.
    // Sections/modules insert at count()-2, i.e. above the stretch.
    // The shortcut reads as one token ("Alt+/"), separated from its meaning
    // by color rather than punctuation.
    auto* help_hint = new QLabel();
    help_hint->setFont(QFont("monospace", theme::FontSizeSm));
    help_hint->setTextFormat(Qt::RichText);
    help_hint->setWordWrap(true);
    help_hint->setText(
        QString("<span style='color:%1;'>Alt+/</span>"
                "&nbsp;&nbsp;<span style='color:%2;'>keybindings</span>"
                "&nbsp;&nbsp;&nbsp;<span style='color:%1;'>Alt+C</span>"
                "&nbsp;&nbsp;<span style='color:%2;'>clear panels</span>")
        .arg(theme::Text).arg(theme::TextDim));
    help_hint->setStyleSheet("padding-top: 6px;");
    layout_->addWidget(help_hint);
}

void ModuleSidebar::addSection(const std::string& name) {
    // Section separator + header
    if (!sections_.empty()) {
        auto* line = new QWidget();
        line->setFixedHeight(1);
        line->setStyleSheet(QString("background: %1;").arg(theme::BorderDim));
        layout_->insertWidget(layout_->count() - 2, line);
    }

    auto* hdr = new QLabel(QString::fromStdString(name).toUpper());
    hdr->setFont(QFont("monospace", theme::FontSizeSm, QFont::Bold));
    hdr->setWordWrap(true);
    hdr->setStyleSheet(
        QString("color: %1; padding: 6px 2px 2px 2px; letter-spacing: 1px;")
        .arg(theme::BorderDim));  // dim initially; setActiveSection brightens it
    layout_->insertWidget(layout_->count() - 2, hdr);

    sections_.push_back({ name, hdr, {} });
}

void ModuleSidebar::addModule(const std::string& name, TilePanel* panel,
                               bool default_visible,
                               std::function<void(bool)> on_toggle) {
    if (sections_.empty()) addSection("General");
    auto& sec = sections_.back();

    // Index label — shows "[N]" when this section is active
    auto* idx_lbl = new QLabel("");
    idx_lbl->setFont(QFont("monospace", theme::FontSizeSm));
    idx_lbl->setStyleSheet(QString("color: %1; min-width: 22px;").arg(theme::TextDim));
    idx_lbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Indicator-only checkbox; the name lives in a separate QLabel because
    // QCheckBox cannot word-wrap and long names clipped at the sidebar edge.
    auto* check = new QCheckBox();
    check->setChecked(default_visible);
    panel->setVisible(default_visible);
    check->setStyleSheet(
        QString("QCheckBox::indicator { width: 14px; height: 14px; }"
                "QCheckBox::indicator:checked { background: %1; border: 1px solid %2; border-radius: 3px; }"
                "QCheckBox::indicator:unchecked { background: %3; border: 1px solid %4; border-radius: 3px; }")
        .arg(theme::Green).arg(theme::Border)
        .arg(theme::Bg).arg(theme::BorderDim));

    QObject::connect(check, &QCheckBox::toggled, [on_toggle](bool visible) {
        if (on_toggle) on_toggle(visible);
    });

    auto* name_lbl = new QLabel(QString::fromStdString(name));
    name_lbl->setFont(QFont("monospace", theme::FontSize));
    name_lbl->setWordWrap(true);
    name_lbl->setStyleSheet(QString("color: %1;").arg(theme::Text));
    name_lbl->installEventFilter(new ToggleOnClick(check, name_lbl));

    // Row: [idx_lbl] [check] [name]
    auto* row = new QHBoxLayout();
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(6);
    row->addWidget(idx_lbl);
    row->addWidget(check);
    row->addWidget(name_lbl, 1);

    auto* row_w = new QWidget();
    row_w->setLayout(row);
    layout_->insertWidget(layout_->count() - 2, row_w);

    // Default width: wide enough that no module name wraps (user can still
    // drag-resize afterwards). Measured via font metrics because a wrapping
    // QLabel's sizeHint() does not report the unwrapped text width.
    int need = 22 + row->spacing()                       // idx_lbl min-width
             + check->sizeHint().width() + row->spacing()
             + QFontMetrics(name_lbl->font()).horizontalAdvance(name_lbl->text())
             + layout_->contentsMargins().left()
             + layout_->contentsMargins().right()
             + SIDEBAR_GRIP_PX + 4;
    content_width_ = std::max(content_width_, need);
    setFixedWidth(std::clamp(content_width_, SIDEBAR_MIN_WIDTH, SIDEBAR_MAX_WIDTH));

    sec.entries.push_back({ check, panel, idx_lbl });

    // Refresh labels for the active section after adding
    if ((int)(sections_.size() - 1) == active_section_)
        refreshIndexLabels();
}

void ModuleSidebar::setActiveSection(int idx) {
    if (idx < 0 || idx >= (int)sections_.size()) return;

    // Dim the old header
    if (active_section_ < (int)sections_.size())
        sections_[active_section_].header->setStyleSheet(
            QString("color: %1; padding: 6px 2px 2px 2px; letter-spacing: 1px;")
            .arg(theme::BorderDim));

    active_section_ = idx;

    // Brighten the new header
    sections_[active_section_].header->setStyleSheet(
        QString("color: %1; padding: 6px 2px 2px 2px; letter-spacing: 1px; font-weight: bold;")
        .arg(theme::Cyan));

    section_indicator_->setText(
        QString("▸ %1  (Alt+[/])").arg(
            QString::fromStdString(sections_[active_section_].name).toUpper()));

    refreshIndexLabels();
}

void ModuleSidebar::refreshIndexLabels() {
    for (int s = 0; s < (int)sections_.size(); s++) {
        bool is_active = (s == active_section_);
        auto& sec = sections_[s];
        for (int i = 0; i < (int)sec.entries.size(); i++) {
            auto& e = sec.entries[i];
            if (is_active && i < 9) {
                e.idx_lbl->setText(QString("[%1]").arg(i + 1));
                e.idx_lbl->setStyleSheet(
                    QString("color: %1; min-width: 22px;").arg(theme::TextDim));
            } else {
                e.idx_lbl->setText("");
            }
        }
    }
}

void ModuleSidebar::switchSection(int delta) {
    if (sections_.empty()) return;
    int n = (int)sections_.size();
    setActiveSection((active_section_ + delta % n + n) % n);
}

void ModuleSidebar::toggleModule(int index) {
    if (active_section_ < 0 || active_section_ >= (int)sections_.size()) return;
    auto& entries = sections_[active_section_].entries;
    if (index < 0 || index >= (int)entries.size()) return;
    entries[index].check->toggle();
}

void ModuleSidebar::syncCheckboxes(const std::vector<std::string>& visible_titles) {
    for (auto& sec : sections_) {
        for (auto& e : sec.entries) {
            bool vis = std::find(visible_titles.begin(), visible_titles.end(),
                                 e.panel->title()) != visible_titles.end();
            e.check->blockSignals(true);
            e.check->setChecked(vis);
            e.check->blockSignals(false);
        }
    }
}

void ModuleSidebar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(SIDEBAR_BG));
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(width() - 1, 0, width() - 1, height());
    // Drag-handle affordance on the resizable right edge
    p.setPen(QPen(QColor(theme::Border), 2));
    p.drawLine(width() - 3, height() / 2 - 12, width() - 3, height() / 2 + 12);
}

// ---------------------------------------------------------------------------
// Right-edge width drag — the layout honours setFixedWidth, so dragging just
// updates it (clamped) and the tiling area reflows automatically.
// ---------------------------------------------------------------------------

void ModuleSidebar::mousePressEvent(QMouseEvent* e) {
    if (e->pos().x() >= width() - SIDEBAR_GRIP_PX) {
        resizing_edge_ = true;
        e->accept();
        return;
    }
    QWidget::mousePressEvent(e);
}

void ModuleSidebar::mouseMoveEvent(QMouseEvent* e) {
    if (resizing_edge_) {
        setFixedWidth(std::clamp(e->pos().x(), SIDEBAR_MIN_WIDTH, SIDEBAR_MAX_WIDTH));
        return;
    }
    setCursor(e->pos().x() >= width() - SIDEBAR_GRIP_PX ? Qt::SizeHorCursor
                                                        : Qt::ArrowCursor);
    QWidget::mouseMoveEvent(e);
}

void ModuleSidebar::mouseReleaseEvent(QMouseEvent* e) {
    resizing_edge_ = false;
    QWidget::mouseReleaseEvent(e);
}

void ModuleSidebar::leaveEvent(QEvent* e) {
    if (!resizing_edge_) setCursor(Qt::ArrowCursor);
    QWidget::leaveEvent(e);
}


// ---------------------------------------------------------------------------
// Shared overlay helpers — scrim/panel chrome, geometry, scrollbar
// ---------------------------------------------------------------------------

struct OverlayFrame {
    int px, py, pw, ph;              // centred panel rect
    int listX, listY, listW, listH;  // scrollable list rect, below the header
};

static OverlayFrame overlayFrame(const QWidget* w, int header_h) {
    OverlayFrame f;
    f.pw = std::min(OVERLAY_MAX_W, w->width()  - OVERLAY_SCREEN_PAD);
    f.ph = std::min(OVERLAY_MAX_H, w->height() - OVERLAY_SCREEN_PAD);
    f.px = (w->width()  - f.pw) / 2;
    f.py = (w->height() - f.ph) / 2;
    f.listX = f.px + OVERLAY_MARGIN;
    f.listY = f.py + OVERLAY_MARGIN + header_h + OVERLAY_LIST_TOP_PAD;
    f.listW = f.pw - 2 * OVERLAY_MARGIN;
    f.listH = f.ph - (f.listY - f.py) - OVERLAY_MARGIN;
    return f;
}

static void drawOverlayChrome(QPainter& p, const QWidget* w, const OverlayFrame& f) {
    p.fillRect(w->rect(), QColor(0, 0, 0, OVERLAY_SCRIM_ALPHA));
    QPainterPath panelPath;
    panelPath.addRoundedRect(QRect(f.px, f.py, f.pw, f.ph),
                             OVERLAY_RADIUS, OVERLAY_RADIUS);
    p.fillPath(panelPath, QColor(theme::BgPanel));
    p.setPen(QPen(QColor(theme::Border), 2));
    p.drawRoundedRect(QRect(f.px, f.py, f.pw, f.ph),
                      OVERLAY_RADIUS, OVERLAY_RADIUS);
}

// no-op when the list fits
static void drawOverlayScrollbar(QPainter& p, const OverlayFrame& f,
                                 int total_h, int scroll_offset) {
    if (total_h <= f.listH) return;
    int barH = std::max(20, f.listH * f.listH / total_h);
    int barY = f.listY + (f.listH - barH) * scroll_offset
                             / std::max(1, total_h - f.listH);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(theme::BorderDim));
    p.drawRoundedRect(f.px + f.pw - OVERLAY_MARGIN + 6, barY, 4, barH, 2, 2);
}


// ---------------------------------------------------------------------------
// KeybindingsOverlay
// ---------------------------------------------------------------------------

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

// y of the entry in unscrolled list coordinates
int KeybindingsOverlay::yOfEntry(int flat_idx) const {
    int y = 0, cur = 0;
    for (auto& cat : categories_) {
        y += KEYBIND_CAT_H;
        for (int e = 0; e < (int)cat.entries.size(); ++e) {
            if (cur == flat_idx) return y;
            y += KEYBIND_ROW_H;
            cur++;
        }
    }
    return y;
}

// keep the focused entry visible with one row of context
void KeybindingsOverlay::scrollToFocused(int list_h) {
    int y = yOfEntry(focused_idx_);
    if (y - scroll_offset_ < KEYBIND_ROW_H)
        scroll_offset_ = std::max(0, y - KEYBIND_ROW_H);
    else if (y + KEYBIND_ROW_H * 2 - scroll_offset_ > list_h)
        scroll_offset_ = y + KEYBIND_ROW_H * 2 - list_h;
}

void KeybindingsOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const OverlayFrame f = overlayFrame(this, KEYBIND_HEADER_H);
    drawOverlayChrome(p, this, f);

    QRect titleRect(f.px + OVERLAY_MARGIN, f.py + OVERLAY_MARGIN, f.listW, 30);
    p.setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    p.setPen(QColor(theme::Text));
    p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter, "Keybindings");
    p.setFont(QFont("monospace", theme::FontSizeSm));
    p.setPen(QColor(theme::TextDim));
    p.drawText(titleRect, Qt::AlignRight | Qt::AlignVCenter, "Esc · Alt+/ to close");

    int sepY = f.listY - OVERLAY_LIST_TOP_PAD;
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(f.listX, sepY, f.px + f.pw - OVERLAY_MARGIN, sepY);

    if (categories_.empty()) return;

    p.setClipRect(f.listX, f.listY, f.listW, f.listH);
    p.translate(0, f.listY - scroll_offset_);

    int y = 0, flat = 0;
    for (auto& cat : categories_) {
        p.setFont(QFont("monospace", theme::FontSize, QFont::Bold));
        p.setPen(QColor(theme::TextDim));
        p.drawText(QRect(f.listX, y + 8, f.listW, 22),
                   Qt::AlignLeft | Qt::AlignVCenter, cat.name.toUpper());
        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.drawLine(f.listX, y + KEYBIND_CAT_H - 2,
                   f.listX + f.listW, y + KEYBIND_CAT_H - 2);
        y += KEYBIND_CAT_H;

        // rows: key sequence (left 45%) / description (right 55%)
        p.setFont(QFont("monospace", theme::FontSizeSm));
        for (auto& entry : cat.entries) {
            bool focused = (flat == focused_idx_);
            if (focused) {
                QColor hi(theme::Green); hi.setAlpha(38);
                p.fillRect(QRect(f.listX, y, f.listW, KEYBIND_ROW_H), hi);
            }
            p.setPen(focused ? QColor(theme::Green) : QColor(theme::Text));
            p.drawText(QRect(f.listX + 8, y, f.listW * 45 / 100, KEYBIND_ROW_H),
                       Qt::AlignLeft | Qt::AlignVCenter, entry.keys);
            p.setPen(focused ? QColor(theme::Green).lighter(130) : QColor(theme::TextDim));
            p.drawText(QRect(f.listX + f.listW * 45 / 100, y,
                             f.listW * 55 / 100, KEYBIND_ROW_H),
                       Qt::AlignLeft | Qt::AlignVCenter, entry.description);
            y += KEYBIND_ROW_H;
            flat++;
        }
    }

    p.resetTransform();
    p.setClipping(false);
    drawOverlayScrollbar(p, f, y, scroll_offset_);
}

void KeybindingsOverlay::keyPressEvent(QKeyEvent* ke) {
    if (categories_.empty()) return;
    const OverlayFrame f = overlayFrame(this, KEYBIND_HEADER_H);

    if (ke->key() == Qt::Key_Up) {
        focused_idx_ = std::max(0, focused_idx_ - 1);
        scrollToFocused(f.listH);
        update();
    } else if (ke->key() == Qt::Key_Down) {
        focused_idx_ = std::min(totalEntries() - 1, focused_idx_ + 1);
        scrollToFocused(f.listH);
        update();
    } else if (ke->key() == Qt::Key_Escape ||
               (ke->key() == Qt::Key_Slash && (ke->modifiers() & Qt::AltModifier))) {
        tc_->hideKeybindingsOverlay();
    }
}

void KeybindingsOverlay::wheelEvent(QWheelEvent* we) {
    const OverlayFrame f = overlayFrame(this, KEYBIND_HEADER_H);

    int steps = we->angleDelta().y() / 40;
    scroll_offset_ = std::max(0, scroll_offset_ - steps * KEYBIND_ROW_H);

    int totalH = 0;
    for (auto& cat : categories_)
        totalH += KEYBIND_CAT_H + (int)cat.entries.size() * KEYBIND_ROW_H;
    scroll_offset_ = std::min(scroll_offset_, std::max(0, totalH - f.listH));
    update();
}


// ---------------------------------------------------------------------------
// LayoutManagerOverlay
// ---------------------------------------------------------------------------

LayoutManagerOverlay::LayoutManagerOverlay(TilingContainer* tc, QWidget* parent)
    : QWidget(parent), tc_(tc) {
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
    setFocusPolicy(Qt::StrongFocus);
    hide();
}

void LayoutManagerOverlay::cancelRename() {
    if (renaming_) {
        renaming_ = false;
        update();
    }
}

void LayoutManagerOverlay::refresh() {
    snapshots_ = tc_->layoutStore().list();
    focused_idx_   = std::min(focused_idx_, std::max(0, (int)snapshots_.size() - 1));
    scroll_offset_ = 0;
}

void LayoutManagerOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const OverlayFrame f = overlayFrame(this, LAYOUT_HEADER_H);
    drawOverlayChrome(p, this, f);

    // title line, then hint line (red store status when saving is unavailable)
    const bool writable = tc_->layoutStore().writable();
    p.setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
    p.setPen(QColor(theme::Text));
    p.drawText(QRect(f.px + OVERLAY_MARGIN, f.py + OVERLAY_MARGIN, f.listW, 24),
               Qt::AlignLeft | Qt::AlignVCenter, "Saved Layouts");
    p.setFont(QFont("monospace", theme::FontSizeSm));
    p.setPen(QColor(writable ? theme::TextDim : theme::Red));
    p.drawText(QRect(f.px + OVERLAY_MARGIN, f.py + OVERLAY_MARGIN + 26, f.listW, 18),
               Qt::AlignRight | Qt::AlignVCenter,
               !writable  ? tc_->layoutStore().statusMessage()
               : renaming_ ? "Enter confirm  Esc cancel"
                           : "S save  R rename  D delete  Enter load  Esc close");

    int sepY = f.listY - OVERLAY_LIST_TOP_PAD;
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(f.listX, sepY, f.px + f.pw - OVERLAY_MARGIN, sepY);

    if (snapshots_.empty()) {
        p.setFont(QFont("monospace", theme::FontSize));
        p.setPen(QColor(theme::TextDim));
        p.drawText(QRect(f.listX, f.listY, f.listW, f.listH), Qt::AlignCenter,
                   writable ? "No saved layouts.\nPress S to save the current layout."
                            : "No saved layouts.");
        return;
    }

    p.setClipRect(f.listX, f.listY, f.listW, f.listH);
    p.translate(0, f.listY - scroll_offset_);

    int y = 0;
    for (int i = 0; i < (int)snapshots_.size(); ++i) {
        const auto& snap = snapshots_[i];
        bool focused = (i == focused_idx_);

        if (focused) {
            QColor hi(theme::Green); hi.setAlpha(38);
            p.fillRect(QRect(f.listX, y, f.listW, LAYOUT_ROW_H), hi);
        }

        // name, or inline rename field when renaming this row
        if (focused && renaming_) {
            QRect editRect(f.listX + 8, y + 6, f.listW - 16, 24);
            p.setPen(QPen(QColor(theme::Cyan), 1));
            p.setBrush(QColor(theme::Bg));
            p.drawRoundedRect(editRect.adjusted(-2, -2, 2, 2), 3, 3);

            p.setFont(QFont("monospace", theme::FontSize, QFont::Bold));
            p.setPen(QColor(theme::Cyan));
            p.drawText(editRect, Qt::AlignLeft | Qt::AlignVCenter, rename_buf_);

            // static text cursor at end of buffer — no blink timer
            QFontMetrics fm(p.font());
            int cx = editRect.x() + fm.horizontalAdvance(rename_buf_);
            p.drawLine(cx, editRect.top() + 3, cx, editRect.bottom() - 3);
        } else {
            p.setFont(QFont("monospace", theme::FontSize, QFont::Bold));
            p.setPen(focused ? QColor(theme::Green) : QColor(theme::Text));
            p.drawText(QRect(f.listX + 8, y + 6, f.listW - 16, 24),
                       Qt::AlignLeft | Qt::AlignVCenter, snap.name);
        }

        // save date, right-aligned; hidden while renaming
        if (!(focused && renaming_)) {
            p.setFont(QFont("monospace", theme::FontSizeSm));
            p.setPen(focused ? QColor(theme::Green).lighter(130) : QColor(theme::TextDim));
            p.drawText(QRect(f.listX + 8, y + 6, f.listW - 16, 24),
                       Qt::AlignRight | Qt::AlignVCenter, snap.saved_at);
        }

        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.drawLine(f.listX, y + LAYOUT_ROW_H - 1,
                   f.listX + f.listW, y + LAYOUT_ROW_H - 1);

        y += LAYOUT_ROW_H;
    }

    p.resetTransform();
    p.setClipping(false);
    drawOverlayScrollbar(p, f, (int)snapshots_.size() * LAYOUT_ROW_H, scroll_offset_);
}

void LayoutManagerOverlay::keyPressEvent(QKeyEvent* ke) {
    const OverlayFrame f = overlayFrame(this, LAYOUT_HEADER_H);
    const int listH = f.listH;

    // keep the focused row inside the visible list
    auto clampScroll = [&]() {
        int totalH = (int)snapshots_.size() * LAYOUT_ROW_H;
        int maxScroll = std::max(0, totalH - listH);
        int rowY = focused_idx_ * LAYOUT_ROW_H;
        if (rowY - scroll_offset_ < 0)
            scroll_offset_ = rowY;
        else if (rowY + LAYOUT_ROW_H - scroll_offset_ > listH)
            scroll_offset_ = rowY + LAYOUT_ROW_H - listH;
        scroll_offset_ = std::clamp(scroll_offset_, 0, maxScroll);
    };

    // ── Rename mode ──────────────────────────────────────────────────────────
    if (renaming_) {
        if (ke->key() == Qt::Key_Return || ke->key() == Qt::Key_Enter) {
            QString name = rename_buf_.trimmed();
            if (!name.isEmpty() && focused_idx_ < (int)snapshots_.size())
                tc_->renameLayout(focused_idx_, name);
            renaming_ = false;
            refresh();
            update();
        } else if (ke->key() == Qt::Key_Escape) {
            renaming_ = false;
            update();
        } else if (ke->key() == Qt::Key_Backspace) {
            if (!rename_buf_.isEmpty())
                rename_buf_.chop(1);
            update();
        } else if (!ke->text().isEmpty() && ke->text()[0].isPrint()) {
            rename_buf_ += ke->text();
            update();
        }
        return;
    }

    // ── Normal mode ──────────────────────────────────────────────────────────
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
    } else if (!tc_->layoutStore().writable() &&
               (ke->key() == Qt::Key_S || ke->key() == Qt::Key_R || ke->key() == Qt::Key_D)) {
        // read-only store: mutating keys are inert (banner explains why)
    } else if (ke->key() == Qt::Key_S) {
        tc_->saveCurrentLayout();
        refresh();
        focused_idx_ = std::max(0, (int)snapshots_.size() - 1);
        update();
    } else if (ke->key() == Qt::Key_R) {
        if (!snapshots_.empty()) {
            rename_buf_ = snapshots_[focused_idx_].name;
            renaming_   = true;
            update();
        }
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
    const OverlayFrame f = overlayFrame(this, LAYOUT_HEADER_H);

    int steps = we->angleDelta().y() / 40;
    scroll_offset_ -= steps * LAYOUT_ROW_H;
    int totalH = (int)snapshots_.size() * LAYOUT_ROW_H;
    scroll_offset_ = std::clamp(scroll_offset_, 0, std::max(0, totalH - f.listH));
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
                                std::vector<std::pair<std::string,std::string>> module_keybinds,
                                const std::string& section,
                                std::function<QJsonObject()> save_state,
                                std::function<void(const QJsonObject&)> restore_state) {
    auto* panel = new TilePanel(title, content, this);

    connect(panel, &TilePanel::clicked, [this, panel]() { setFocusedPanel(panel); });
    connect(panel, &TilePanel::hovered, [this, panel]() {
        if (drag_mode_ == DragMode::None) setFocusedPanel(panel);
    });

    panels_.push_back({panel, layout_hint, section, default_visible, on_toggle,
                       std::move(save_state), std::move(restore_state),
                       std::move(module_keybinds)});
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

DwindleNode* TilingContainer::buildHintTree(
        const std::function<bool(const PanelInfo&)>& include) {
    // Groups: hint=="main"|"left" → left column, "right" → right column,
    //         anything else → bottom row
    std::vector<TilePanel*> left_panels, right_panels, bottom_panels;
    for (auto& pi : panels_) {
        if (!include(pi)) continue;
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

    DwindleNode* top_tree = nullptr;
    if (left_tree && right_tree) {
        auto* lr = new DwindleNode();
        lr->isNode = true;
        lr->splitTop = false;
        lr->splitRatio = 1.0f;
        lr->children[0] = left_tree;
        lr->children[1] = right_tree;
        left_tree->parent  = lr;
        right_tree->parent = lr;
        all_nodes_.push_back(lr);
        top_tree = lr;
    } else {
        top_tree = left_tree ? left_tree : right_tree;
    }

    if (top_tree && bottom_tree) {
        auto* tb = new DwindleNode();
        tb->isNode = true;
        tb->splitTop = true;
        tb->splitRatio = TOP_BOTTOM_SPLIT_RATIO;
        tb->children[0] = top_tree;
        tb->children[1] = bottom_tree;
        top_tree->parent    = tb;
        bottom_tree->parent = tb;
        all_nodes_.push_back(tb);
        return tb;
    }
    return top_tree ? top_tree : bottom_tree;
}

void TilingContainer::finalize() {
    auto* outer_layout = new QHBoxLayout(this);
    outer_layout->setContentsMargins(0, 0, 0, 0);
    outer_layout->setSpacing(0);

    sidebar_ = new ModuleSidebar(this);
    outer_layout->addWidget(sidebar_);

    tiling_area_ = new QWidget(this);
    outer_layout->addWidget(tiling_area_, 1);

    // Wire up sidebar toggles: grouped by section
    // Collect unique sections in insertion order
    std::vector<std::string> section_order;
    for (auto& pi : panels_) {
        const std::string& sec = pi.section.empty() ? "General" : pi.section;
        if (std::find(section_order.begin(), section_order.end(), sec) == section_order.end())
            section_order.push_back(sec);
    }

    for (const auto& sec_name : section_order) {
        sidebar_->addSection(sec_name);
        for (auto& pi : panels_) {
            const std::string& sec = pi.section.empty() ? "General" : pi.section;
            if (sec != sec_name) continue;

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
    }

    // Build initial dwindle tree for default-visible panels.
    root_ = buildHintTree([](const PanelInfo& pi) { return pi.default_visible; });

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
                       {"Alt + C",              "Clear all panels"      },
                       {"Alt+Shift+Arrow",      "Resize panel (hold)"   },
                       {"Alt + X + drag",       "Free resize"           },
                       {"Alt+Ctrl+Shift+Arrow", "Swap panel"            },
                       {"Alt + Z + drag",       "Drag to swap"          } }},
        { "Modules", { {"Alt + 1 … 9",          "Toggle module (active section)" },
                       {"Alt + [",             "Previous section"               },
                       {"Alt + ]",             "Next section"                   },
                       {"Sidebar checkbox",    "Toggle visibility"              } }},
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
    bind("Alt+C", [this]() { if (anyOverlayVisible()) return; clearAllPanels(); });
    bind("Alt+P", [this]() { toggleLayoutManagerOverlay(); });

    bind("Alt+[", [this]() { if (anyOverlayVisible()) return; sidebar_->switchSection(-1); });
    bind("Alt+]", [this]() { if (anyOverlayVisible()) return; sidebar_->switchSection(1); });

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

// Nearest ancestor of `leaf` split in the given orientation; child_on_path
// receives that ancestor's child on the path from leaf (resize direction).
static DwindleNode* splitAncestor(DwindleNode* leaf, bool split_top,
                                  DwindleNode** child_on_path = nullptr) {
    for (auto* cur = leaf; cur && cur->parent; cur = cur->parent) {
        if (cur->parent->splitTop == split_top) {
            if (child_on_path) *child_on_path = cur;
            return cur->parent;
        }
    }
    return nullptr;
}

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
            if (auto* under = panelUnderCursor()) setFocusedPanel(under);
            enterMoveMode();
            return true;
        }
        if (ke->key() == Qt::Key_X && (ke->modifiers() & Qt::AltModifier)
            && !ke->isAutoRepeat() && drag_mode_ == DragMode::None) {
            // Same cursor-position fix for resize mode.
            if (auto* under = panelUnderCursor()) setFocusedPanel(under);
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
            // per axis, resize the closest matching-orientation split;
            // *2 maps a full-box drag to the full 0..2 ratio range
            auto* leaf = getLeafFor(focused_panel_);
            if (leaf) {
                if (delta.x() != 0) {
                    if (auto* split = splitAncestor(leaf, /*split_top=*/false)) {
                        float rel = (float)delta.x() * 2.0f / std::max(1, split->box.width());
                        split->splitRatio = std::clamp(split->splitRatio + rel,
                                                       SPLIT_RATIO_MIN, SPLIT_RATIO_MAX);
                        recalculate();
                    }
                }
                if (delta.y() != 0) {
                    if (auto* split = splitAncestor(leaf, /*split_top=*/true)) {
                        float rel = (float)delta.y() * 2.0f / std::max(1, split->box.height());
                        split->splitRatio = std::clamp(split->splitRatio + rel,
                                                       SPLIT_RATIO_MIN, SPLIT_RATIO_MAX);
                        recalculate();
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
    layout_overlay_->cancelRename();
    layout_overlay_->hide();
}


// ---------------------------------------------------------------------------
// Layout persistence — repo-tracked JSON files via LayoutStore.
// UI indices are into LayoutStore::list(), same ordering everywhere.
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

void TilingContainer::clearAllPanels() {
    for (auto& pi : panels_) {
        if (!pi.panel->isVisible()) continue;
        if (pi.on_toggle) pi.on_toggle(false);
        pi.panel->setVisible(false);
    }
    for (auto* n : all_nodes_) delete n;
    all_nodes_.clear();
    root_          = nullptr;
    focused_panel_ = nullptr;
    sidebar_->syncCheckboxes({});
    recalculate();
}

void TilingContainer::saveCurrentLayout() {
    QJsonObject layout;
    QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm");
    layout["name"]     = ts;  // renameable later
    layout["saved_at"] = ts;

    if (root_) layout["tree"] = serializeTree(root_);

    QJsonArray visible;
    for (auto& pi : panels_)
        if (pi.panel->isVisible())
            visible.append(QString::fromStdString(pi.panel->title()));
    layout["visible"] = visible;

    // Module-specific state (e.g. camera grid membership), keyed by title.
    QJsonObject modules;
    for (auto& pi : panels_) {
        if (!pi.save_state) continue;
        QJsonObject st = pi.save_state();
        if (!st.isEmpty())
            modules[QString::fromStdString(pi.panel->title())] = st;
    }
    if (!modules.isEmpty()) layout["modules"] = modules;

    layout_store_.save(layout);
}

void TilingContainer::loadLayout(int index) {
    auto entries = layout_store_.list();
    if (index < 0 || index >= (int)entries.size()) return;
    const QJsonObject& layout = entries[index].json;

    QSet<QString> visible_set;
    for (auto v : layout["visible"].toArray())
        visible_set.insert(v.toString());

    // Clear tree
    for (auto* n : all_nodes_) delete n;
    all_nodes_.clear();
    root_         = nullptr;
    focused_panel_ = nullptr;

    // Set panel visibility — only show panels explicitly in the saved visible set
    std::vector<std::string> visible_titles;
    for (auto& pi : panels_) {
        bool vis = visible_set.contains(QString::fromStdString(pi.panel->title()));
        bool was = pi.panel->isVisible();
        pi.panel->setVisible(vis);
        if (vis != was && pi.on_toggle) pi.on_toggle(vis);  // let modules react
        if (vis) visible_titles.push_back(pi.panel->title());
    }
    sidebar_->syncCheckboxes(visible_titles);

    // Apply saved module state after visibility settles.
    QJsonObject modules = layout["modules"].toObject();
    for (auto& pi : panels_) {
        if (!pi.restore_state) continue;
        auto it = modules.constFind(QString::fromStdString(pi.panel->title()));
        if (it != modules.constEnd()) pi.restore_state(it->toObject());
    }

    // Reconstruct tree
    if (layout.contains("tree") && !layout["tree"].toObject().isEmpty())
        root_ = deserializeTree(layout["tree"].toObject());

    recalculate();

    // Focus first visible panel
    for (auto& pi : panels_) {
        if (pi.panel->isVisible()) { setFocusedPanel(pi.panel); break; }
    }

    hideLayoutManagerOverlay();
    if (onLayoutChanged) onLayoutChanged(entries[index].name);
}

bool TilingContainer::loadLayoutByName(const QString& name) {
    auto entries = layout_store_.list();
    for (int i = 0; i < (int)entries.size(); ++i)
        if (entries[i].name == name) { loadLayout(i); return true; }
    return false;
}

void TilingContainer::showPanels(const std::vector<std::string>& titles) {
    for (auto* n : all_nodes_) delete n;
    all_nodes_.clear();
    root_          = nullptr;
    focused_panel_ = nullptr;

    std::vector<std::string> visible_titles;
    for (auto& pi : panels_) {
        bool vis = std::find(titles.begin(), titles.end(),
                             pi.panel->title()) != titles.end();
        pi.panel->setVisible(vis);
        if (vis) visible_titles.push_back(pi.panel->title());
    }
    sidebar_->syncCheckboxes(visible_titles);

    root_ = buildHintTree([&titles](const PanelInfo& pi) {
        return std::find(titles.begin(), titles.end(),
                         pi.panel->title()) != titles.end();
    });
    recalculate();

    for (auto& pi : panels_)
        if (pi.panel->isVisible()) { setFocusedPanel(pi.panel); break; }

    hideLayoutManagerOverlay();
    QStringList shown;
    for (const auto& t : visible_titles) shown << QString::fromStdString(t);
    if (onLayoutChanged) onLayoutChanged(shown.join(" + "));
}

void TilingContainer::deleteLayout(int index) {
    auto entries = layout_store_.list();
    if (index >= 0 && index < (int)entries.size())
        layout_store_.remove(entries[index].file_path);
}

void TilingContainer::renameLayout(int index, const QString& name) {
    auto entries = layout_store_.list();
    if (index >= 0 && index < (int)entries.size())
        layout_store_.rename(entries[index].file_path, name);
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

// Hit-test at the cursor: embedded X11 windows (RViz) swallow hover events,
// leaving focused_panel_ stale when a drag mode starts.
TilePanel* TilingContainer::panelUnderCursor() const {
    if (!tiling_area_) return nullptr;
    QPoint local = tiling_area_->mapFromGlobal(QCursor::pos());
    for (const auto& pi : panels_)
        if (pi.panel->isVisible() && pi.panel->geometry().contains(local))
            return pi.panel;
    return nullptr;
}

// Nearest visible panel strictly in the given direction (arrow semantics:
// one of dx/dy non-zero); nullptr if none.
TilePanel* TilingContainer::nearestPanelInDirection(int dx, int dy) const {
    if (!focused_panel_) return nullptr;
    QPoint cur = focused_panel_->geometry().center();

    TilePanel* best = nullptr;
    double best_dist = 1e18;

    for (const auto& pi : panels_) {
        auto* p = pi.panel;
        if (p == focused_panel_ || !p->isVisible()) continue;
        QPoint rel = p->geometry().center() - cur;
        if (dx > 0 && rel.x() <= 0) continue;
        if (dx < 0 && rel.x() >= 0) continue;
        if (dy > 0 && rel.y() <= 0) continue;
        if (dy < 0 && rel.y() >= 0) continue;

        double dist = (double)rel.x() * rel.x() + (double)rel.y() * rel.y();
        if (dist < best_dist) { best_dist = dist; best = p; }
    }
    return best;
}

void TilingContainer::focusDirection(int dx, int dy) {
    if (auto* best = nearestPanelInDirection(dx, dy))
        setFocusedPanel(best);
}

void TilingContainer::swapWithFocused(int dx, int dy) {
    if (auto* target = nearestPanelInDirection(dx, dy))
        dwindleSwap(focused_panel_, target);
}

// Resize along the arrow axis via the nearest matching-orientation split;
// the sign flips by which side the panel is on so it grows toward the arrow.
void TilingContainer::resizeFocused(int dx, int dy) {
    if (!focused_panel_) return;
    auto* leaf = getLeafFor(focused_panel_);
    if (!leaf) return;

    int arrow = (dx != 0) ? dx : dy;
    bool split_top = (dx == 0);  // vertical arrow → top/bottom split

    DwindleNode* child = nullptr;
    auto* split = splitAncestor(leaf, split_top, &child);
    if (!split) return;

    float delta = (split->children[0] == child ? arrow : -arrow) * RESIZE_RATIO_STEP;
    split->splitRatio = std::clamp(split->splitRatio + delta,
                                   SPLIT_RATIO_MIN, SPLIT_RATIO_MAX);
    recalculate();
}
