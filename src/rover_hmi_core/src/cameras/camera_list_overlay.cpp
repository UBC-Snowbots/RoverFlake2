#include "camera_list_overlay.h"
#include <rover_hmi_core/catppuccin.h>

#include <QEvent>
#include <QFontMetrics>
#include <QPainter>

namespace { const QString kHint = QStringLiteral("↑↓ switch   G grid"); }

CameraListOverlay::CameraListOverlay(std::vector<QString> labels, QWidget* parent)
    : QWidget(parent), labels_(std::move(labels)), alive_(labels_.size(), false)
{
    setAttribute(Qt::WA_TransparentForMouseEvents);
    parent->installEventFilter(this);
    reposition();
}

void CameraListOverlay::setActive(int idx)
{
    active_ = idx;
    update();
}

void CameraListOverlay::setAlive(const std::vector<bool>& alive)
{
    if (alive != alive_) {
        alive_ = alive;
        update();
    }
}

int CameraListOverlay::rowHeight() const
{
    return QFontMetrics(QFont("monospace", theme::px(theme::FontSizeSm))).height() + 10;
}

void CameraListOverlay::reposition()
{
    QFontMetrics fm(QFont("monospace", theme::px(theme::FontSizeSm)));
    int w = fm.horizontalAdvance(kHint);
    for (size_t i = 0; i < labels_.size(); ++i)
        w = qMax(w, fm.horizontalAdvance(QStringLiteral("%1 %2").arg(i + 1).arg(labels_[i])));
    w += 36;
    int h = rowHeight() * (int(labels_.size()) + 1) + 12;  // +1: hint footer
    resize(w, h);
    if (parentWidget())
        move(parentWidget()->width() - w - 12, parentWidget()->height() - h - 12);
}

bool CameraListOverlay::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == parentWidget() && ev->type() == QEvent::Resize) reposition();
    return QWidget::eventFilter(obj, ev);
}

void CameraListOverlay::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0, 0, 0, 170));
    p.drawRoundedRect(rect(), 8, 8);

    p.setFont(QFont("monospace", theme::px(theme::FontSizeSm)));
    int rh = rowHeight();
    for (int i = 0; i < int(labels_.size()); ++i) {
        QRect row(6, 6 + i * rh, width() - 12, rh);
        if (i == active_) {
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(theme::Cyan).darker(300));
            p.drawRoundedRect(row, 4, 4);
        }
        bool alive = i < int(alive_.size()) && alive_[size_t(i)];
        p.setPen(i == active_ ? QColor(theme::Cyan)
               : alive        ? QColor(theme::Text)
                              : QColor(theme::TextDim));
        p.drawText(row.adjusted(8, 0, 0, 0), Qt::AlignVCenter,
                   QStringLiteral("%1 %2").arg(i + 1).arg(labels_[size_t(i)]));
    }
    QRect footer(6, 6 + int(labels_.size()) * rh, width() - 12, rh);
    p.setPen(QColor(theme::TextDim));
    p.drawText(footer.adjusted(8, 0, 0, 0), Qt::AlignVCenter, kHint);
}
