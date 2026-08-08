#include "camera_picker_overlay.h"

#include <rover_hmi_core/catppuccin.h>

#include <QKeyEvent>
#include <QPainter>
#include <QPainterPath>

namespace {
constexpr int ROW_H    = 34;
constexpr int HEADER_H = 34;
constexpr int MARGIN   = 20;
constexpr int PANEL_W  = 380;
constexpr int RADIUS   = 10;
}

CameraPickerOverlay::CameraPickerOverlay(QStringList labels, QWidget* parent)
    : QWidget(parent), labels_(std::move(labels)) {
    setAttribute(Qt::WA_NoSystemBackground);
    setAttribute(Qt::WA_TranslucentBackground);
    setFocusPolicy(Qt::StrongFocus);
    hide();
}

void CameraPickerOverlay::setState(const std::vector<bool>& in_grid,
                                   const std::vector<bool>& alive) {
    in_grid_ = in_grid;
    alive_   = alive;
    if (isVisible()) update();
}

void CameraPickerOverlay::toggle() {
    if (isVisible()) { hide(); return; }
    if (parentWidget()) setGeometry(parentWidget()->rect());
    raise();
    show();
    setFocus();
}

void CameraPickerOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0, 0, 0, 170));

    const int ph = HEADER_H + 2 * MARGIN + ROW_H * int(labels_.size());
    const int pw = qMin(PANEL_W, width() - 2 * MARGIN);
    const QRect panel((width() - pw) / 2, (height() - qMin(ph, height())) / 2,
                      pw, qMin(ph, height() - 2 * MARGIN));

    QPainterPath path;
    path.addRoundedRect(panel, RADIUS, RADIUS);
    p.fillPath(path, QColor(theme::BgPanel));
    p.setPen(QPen(QColor(theme::Border), 2));
    p.drawRoundedRect(panel, RADIUS, RADIUS);

    p.setFont(QFont("monospace", theme::FontSize, QFont::Bold));
    p.setPen(QColor(theme::Text));
    p.drawText(QRect(panel.x() + MARGIN, panel.y() + 8, panel.width() - 2 * MARGIN, 22),
               Qt::AlignLeft | Qt::AlignVCenter, "Grid cameras");
    p.setFont(QFont("monospace", theme::FontSizeSm));
    p.setPen(QColor(theme::TextDim));
    p.drawText(QRect(panel.x() + MARGIN, panel.y() + 8, panel.width() - 2 * MARGIN, 22),
               Qt::AlignRight | Qt::AlignVCenter, "Enter toggle · Esc close");

    int y = panel.y() + HEADER_H + MARGIN / 2;
    for (int i = 0; i < labels_.size(); ++i) {
        const bool focused = (i == focused_idx_);
        const bool in      = i < int(in_grid_.size()) && in_grid_[size_t(i)];
        const bool alive   = i < int(alive_.size()) && alive_[size_t(i)];
        if (focused) {
            QColor hi(theme::Green); hi.setAlpha(38);
            p.fillRect(QRect(panel.x() + 6, y, panel.width() - 12, ROW_H), hi);
        }
        p.setFont(QFont("monospace", theme::FontSize));
        p.setPen(in ? QColor(theme::Green) : QColor(theme::TextDim));
        p.drawText(QRect(panel.x() + MARGIN, y, 24, ROW_H),
                   Qt::AlignLeft | Qt::AlignVCenter, in ? "✓" : "·");
        p.setPen(focused ? QColor(theme::Green)
                         : alive ? QColor(theme::Text) : QColor(theme::TextDim));
        p.drawText(QRect(panel.x() + MARGIN + 24, y, panel.width() - 2 * MARGIN - 24, ROW_H),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 %2%3").arg(i + 1).arg(labels_[i])
                       .arg(alive ? QString() : QStringLiteral("  (no publisher)")));
        y += ROW_H;
    }
}

void CameraPickerOverlay::keyPressEvent(QKeyEvent* ke) {
    switch (ke->key()) {
    case Qt::Key_Up:
        focused_idx_ = qMax(0, focused_idx_ - 1);
        update();
        break;
    case Qt::Key_Down:
        focused_idx_ = qMin(int(labels_.size()) - 1, focused_idx_ + 1);
        update();
        break;
    case Qt::Key_Return:
    case Qt::Key_Enter:
    case Qt::Key_Space:
        if (onToggle) onToggle(focused_idx_);
        break;
    case Qt::Key_Escape:
    case Qt::Key_P:
        hide();
        break;
    default:
        QWidget::keyPressEvent(ke);
    }
}
