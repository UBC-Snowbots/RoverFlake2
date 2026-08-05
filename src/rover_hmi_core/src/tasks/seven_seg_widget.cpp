// seven_seg_widget.cpp — clickable 7-segment digit

#include "seven_seg_widget.h"
#include <rover_hmi_core/catppuccin.h>

#include <QMouseEvent>
#include <QPainter>

SevenSegWidget::SevenSegWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(110, 170);
    setMaximumSize(160, 250);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    setCursor(Qt::PointingHandCursor);
}

void SevenSegWidget::setBits(quint8 b) {
    bits_ = b & 0x7f;
    update();
}

QRectF SevenSegWidget::segRect(int i) const {
    const qreal m = 8;
    const qreal W = width() - 2 * m;
    const qreal H = height() - 2 * m;
    const qreal t = qMin(W, H) * 0.18;
    const qreal midY = (H - t) / 2;
    switch (i) {
        case 0: return {m + t, m, W - 2 * t, t};                            // a
        case 1: return {m + W - t, m + t, t, midY - t};                     // b
        case 2: return {m + W - t, m + midY + t, t, H - midY - 2 * t};      // c
        case 3: return {m + t, m + H - t, W - 2 * t, t};                    // d
        case 4: return {m, m + midY + t, t, H - midY - 2 * t};              // e
        case 5: return {m, m + t, t, midY - t};                             // f
        case 6: return {m + t, m + midY, W - 2 * t, t};                     // g
    }
    return {};
}

void SevenSegWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    for (int i = 0; i < 7; ++i) {
        const bool lit = bits_ & (1 << i);
        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.setBrush(QColor(lit ? theme::Red : "#1c1c1c"));
        p.drawRoundedRect(segRect(i), 3, 3);
    }
}

void SevenSegWidget::mousePressEvent(QMouseEvent* ev) {
    for (int i = 0; i < 7; ++i) {
        if (segRect(i).adjusted(-3, -3, 3, 3).contains(ev->pos())) {
            bits_ ^= (1 << i);
            update();
            if (onChanged) onChanged();
            return;
        }
    }
}
