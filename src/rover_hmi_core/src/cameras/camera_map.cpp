#include "camera_map.h"
#include <rover_hmi_core/catppuccin.h>

#include <QMouseEvent>
#include <QPainter>

namespace {
constexpr double kDotRadius = 14.0;
constexpr double kHitRadius = 26.0;
}

CameraMap::CameraMap(std::vector<rover_hmi_core::camera_config::Camera> cams,
                     QWidget* parent)
    : QWidget(parent), cams_(std::move(cams)), alive_(cams_.size(), false)
{
    setMinimumSize(220, 300);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
}

void CameraMap::setActive(int idx)
{
    active_ = idx;
    update();
}

void CameraMap::setAlive(const std::vector<bool>& alive)
{
    if (alive != alive_) {
        alive_ = alive;
        update();
    }
}

QRect CameraMap::bodyRect() const
{
    // 2:3 rover footprint centered in the widget with margin for labels.
    int m = 30;
    QRect avail = rect().adjusted(m, m, -m, -m);
    int w = qMin(avail.width(), avail.height() * 2 / 3);
    int h = w * 3 / 2;
    if (h > avail.height()) { h = avail.height(); w = h * 2 / 3; }
    return QRect(avail.center().x() - w / 2, avail.center().y() - h / 2, w, h);
}

QPointF CameraMap::dotPos(int idx) const
{
    QRect b = bodyRect();
    const auto& pos = cams_[size_t(idx)].pos;
    return { b.left() + pos.x() * b.width(), b.top() + pos.y() * b.height() };
}

void CameraMap::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(theme::Bg));

    QRect b = bodyRect();
    // Chassis + 6 wheels, plain shapes in dim outline.
    p.setPen(QPen(QColor(theme::BorderDim), 2));
    p.setBrush(QColor(theme::BgPanel));
    p.drawRoundedRect(b, 18, 18);
    int ww = b.width() / 5, wh = b.height() / 7;
    for (int i = 0; i < 3; ++i) {
        int y = b.top() + (i + 1) * b.height() / 4 - wh / 2;
        p.drawRoundedRect(QRect(b.left() - ww / 2, y, ww, wh), 4, 4);
        p.drawRoundedRect(QRect(b.right() - ww / 2, y, ww, wh), 4, 4);
    }

    p.setFont(QFont("monospace", theme::px(theme::FontSizeSm)));
    for (int i = 0; i < int(cams_.size()); ++i) {
        QPointF c = dotPos(i);
        bool alive = i < int(alive_.size()) && alive_[size_t(i)];
        QColor col = (i == active_) ? QColor(theme::Cyan)
                   : alive          ? QColor(theme::Text)
                                    : QColor(theme::TextDim);
        p.setPen(QPen(col, 2));
        p.setBrush(i == active_ ? QColor(theme::Cyan).darker(200) : QColor(theme::Bg));
        p.drawEllipse(c, kDotRadius, kDotRadius);
        p.setPen(col);
        p.drawText(QRectF(c.x() - 70, c.y() + kDotRadius + 2, 140, 24),
                   Qt::AlignHCenter | Qt::AlignTop,
                   QStringLiteral("%1 %2").arg(i + 1).arg(cams_[size_t(i)].label));
    }
}

void CameraMap::mousePressEvent(QMouseEvent* e)
{
    for (int i = 0; i < int(cams_.size()); ++i) {
        QPointF d = dotPos(i) - QPointF(e->pos());
        if (d.x() * d.x() + d.y() * d.y() <= kHitRadius * kHitRadius) {
            if (onSelect) onSelect(i);
            return;
        }
    }
}
