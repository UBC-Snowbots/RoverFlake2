#include "camera_viewport.h"
#include <rover_hmi_core/catppuccin.h>

#include <QPainter>
#include <QRandomGenerator>

CameraViewport::CameraViewport(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(320, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    static_timer_ = new QTimer(this);
    QObject::connect(static_timer_, &QTimer::timeout, this, [this]() { update(); });
    static_timer_->start(100);
}

void CameraViewport::mousePressEvent(QMouseEvent* ev)
{
    if (onClick) onClick();
    QWidget::mousePressEvent(ev);
}

void CameraViewport::setLabel(const QString& label)
{
    label_ = label;
    update();
}

void CameraViewport::setFrame(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (!msg) { setNoSignal(); return; }
    if (frame_clock_.isValid()) {
        qint64 dt = frame_clock_.restart();
        if (dt > 0) fps_ = fps_ * 0.9 + (1000.0 / dt) * 0.1;
    } else {
        frame_clock_.start();
    }
    msg_ = std::move(msg);
    error_.clear();
    if (static_timer_->isActive()) static_timer_->stop();
    update();
}

void CameraViewport::setNoSignal()
{
    msg_.reset();
    fps_ = 0.0;
    frame_clock_.invalidate();
    // The noise animation only runs when there is no placeholder picture.
    if (placeholder_.isNull() && !static_timer_->isActive()) static_timer_->start(100);
    update();
}

void CameraViewport::setPlaceholder(const QPixmap& pm)
{
    placeholder_ = pm;
    if (!placeholder_.isNull() && static_timer_->isActive()) static_timer_->stop();
    else if (placeholder_.isNull() && !msg_ && !static_timer_->isActive())
        static_timer_->start(100);
    update();
}

void CameraViewport::setError(const QString& msg)
{
    error_ = msg;
    setNoSignal();
}

void CameraViewport::setFocused(bool on)
{
    if (focused_ == on) return;
    focused_ = on;
    update();
}

void CameraViewport::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), QColor(theme::Bg));
    if (msg_) drawFrame(p);
    else      drawStatic(p);
    drawOverlay(p);
    if (focused_) {
        p.setPen(QPen(QColor(theme::Green), 2));
        p.drawRect(rect().adjusted(1, 1, -1, -1));
    }
}

void CameraViewport::drawFrame(QPainter& p)
{
    QImage::Format fmt;
    size_t bpp = 0;
    const auto& enc = msg_->encoding;
    if      (enc == "rgb8")  { fmt = QImage::Format_RGB888; bpp = 3; }
    else if (enc == "bgr8")  { fmt = QImage::Format_BGR888; bpp = 3; }
    else if (enc == "mono8") { fmt = QImage::Format_Grayscale8; bpp = 1; }
    else {
        p.setPen(QColor(theme::Red));
        p.drawText(rect(), Qt::AlignCenter,
                   QStringLiteral("unsupported encoding: %1").arg(enc.c_str()));
        return;
    }
    if (msg_->step < size_t(msg_->width) * bpp
        || msg_->data.size() < size_t(msg_->step) * msg_->height) {
        p.setPen(QColor(theme::Red));
        p.drawText(rect(), Qt::AlignCenter,
                   QStringLiteral("malformed image: truncated data"));
        return;
    }
    // Zero-copy wrap; msg_ keeps the buffer alive until the next frame replaces it.
    QImage img(msg_->data.data(), int(msg_->width), int(msg_->height),
               qsizetype(msg_->step), fmt);
    QSize scaled = img.size().scaled(size(), Qt::KeepAspectRatio);
    QRect target(QPoint((width() - scaled.width()) / 2,
                        (height() - scaled.height()) / 2), scaled);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.drawImage(target, img);
}

void CameraViewport::drawStatic(QPainter& p)
{
    QString text = error_.isEmpty()
        ? QStringLiteral("NO SIGNAL — %1").arg(label_)
        : error_;
    if (!placeholder_.isNull()) {
        // Small centered square, PTZ-idle style: center-cropped, capped size,
        // caption directly underneath.
        int ch = theme::px(theme::FontSize) + 10;
        int side = qMin(qMin(width(), height()) * 2 / 5, 300);
        int top = (height() - side - ch) / 2;
        QRect target((width() - side) / 2, top, side, side);
        int s = qMin(placeholder_.width(), placeholder_.height());
        QRect src((placeholder_.width() - s) / 2, (placeholder_.height() - s) / 2, s, s);
        p.setRenderHint(QPainter::SmoothPixmapTransform);
        p.drawPixmap(target, placeholder_, src);
        p.setFont(QFont("monospace", theme::px(theme::FontSize), QFont::Bold));
        p.setPen(QColor(theme::Red));
        p.drawText(QRect(0, top + side + 4, width(), ch),
                   Qt::AlignHCenter | Qt::AlignTop, text);
        return;
    }
    auto* rng = QRandomGenerator::global();
    const int cell = 6;
    for (int y = 0; y < height(); y += cell)
        for (int x = 0; x < width(); x += cell) {
            int g = 20 + int(rng->bounded(70));
            p.fillRect(x, y, cell, cell, QColor(g, g, g));
        }
    p.setFont(QFont("monospace", theme::px(theme::FontSizeLg), QFont::Bold));
    p.setPen(QColor(theme::Red));
    p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap, text);
}

void CameraViewport::drawOverlay(QPainter& p)
{
    QString info = label_;
    if (msg_)
        info += QStringLiteral("  %1x%2  %3 fps")
                    .arg(msg_->width).arg(msg_->height).arg(fps_, 0, 'f', 0);
    p.setFont(QFont("monospace", theme::px(theme::FontSizeSm)));
    QRect box = p.fontMetrics().boundingRect(info).adjusted(-8, -4, 8, 4);
    box.moveTopLeft(QPoint(8, 8));
    p.fillRect(box, QColor(0, 0, 0, 170));
    p.setPen(QColor(theme::Green));
    p.drawText(box, Qt::AlignCenter, info);
}
