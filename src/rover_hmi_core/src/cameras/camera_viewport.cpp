#include "camera_viewport.h"
#include <rover_hmi_core/catppuccin.h>
#include <rover_hmi_core/cameras/camera_config.h>

#include <QDateTime>
#include <QDir>
#include <QPainter>
#include <QPushButton>
#include <QRandomGenerator>

namespace {
// Zero-copy QImage wrap of the ROS buffer; invalid + reason set on bad input.
// The wrap stays valid only while msg's buffer lives.
QImage wrapFrame(const sensor_msgs::msg::Image& msg, QString* why)
{
    QImage::Format fmt;
    size_t bpp = 0;
    const auto& enc = msg.encoding;
    if      (enc == "rgb8")  { fmt = QImage::Format_RGB888; bpp = 3; }
    else if (enc == "bgr8")  { fmt = QImage::Format_BGR888; bpp = 3; }
    else if (enc == "mono8") { fmt = QImage::Format_Grayscale8; bpp = 1; }
    else {
        *why = QStringLiteral("unsupported encoding: %1").arg(enc.c_str());
        return {};
    }
    if (msg.step < size_t(msg.width) * bpp
        || msg.data.size() < size_t(msg.step) * msg.height) {
        *why = QStringLiteral("malformed image: truncated data");
        return {};
    }
    return QImage(msg.data.data(), int(msg.width), int(msg.height),
                  qsizetype(msg.step), fmt);
}
}  // namespace

CameraViewport::CameraViewport(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(320, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    static_timer_ = new QTimer(this);
    QObject::connect(static_timer_, &QTimer::timeout, this, [this]() { update(); });
    static_timer_->start(100);

    snap_btn_ = new QPushButton(QStringLiteral("📷"), this);
    snap_btn_->setFocusPolicy(Qt::NoFocus);  // keys stay with the module
    snap_btn_->setToolTip(QStringLiteral("Save screenshot"));
    snap_btn_->setStyleSheet(QStringLiteral(
        "QPushButton { background: rgba(0,0,0,170); border: 1px solid %1;"
        " padding: 2px 8px; }").arg(theme::BorderDim));
    snap_btn_->hide();
    QObject::connect(snap_btn_, &QPushButton::clicked,
                     this, [this]() { saveSnapshot(); });
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
    snap_btn_->show();
    update();
}

void CameraViewport::setNoSignal()
{
    msg_.reset();
    fps_ = 0.0;
    frame_clock_.invalidate();
    snap_btn_->hide();
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

void CameraViewport::resizeEvent(QResizeEvent*)
{
    snap_btn_->adjustSize();
    snap_btn_->move(width() - snap_btn_->width() - 8, 8);
}

void CameraViewport::saveSnapshot()
{
    if (!msg_) return;
    QString why;
    QImage img = wrapFrame(*msg_, &why);
    if (img.isNull()) return;  // the paint path already shows why on screen

    QDir dir(rover_hmi_core::camera_config::screenshotDir());
    dir.mkpath(QStringLiteral("."));
    const QString file = dir.filePath(
        QStringLiteral("%1_%2.png")
            .arg(label_.toLower().replace(' ', '_'),
                 QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")));

    flash_ = img.save(file) ? QStringLiteral("saved %1").arg(file)
                            : QStringLiteral("screenshot failed: %1").arg(file);
    QTimer::singleShot(3000, this, [this]() { flash_.clear(); update(); });
    update();
}

void CameraViewport::drawFrame(QPainter& p)
{
    QString why;
    // Zero-copy wrap; msg_ keeps the buffer alive until the next frame replaces it.
    QImage img = wrapFrame(*msg_, &why);
    if (img.isNull()) {
        p.setPen(QColor(theme::Red));
        p.drawText(rect(), Qt::AlignCenter, why);
        return;
    }
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

    if (!flash_.isEmpty()) {
        QRect fb = p.fontMetrics().boundingRect(flash_).adjusted(-8, -4, 8, 4);
        fb.moveBottomLeft(QPoint(8, height() - 8));
        p.fillRect(fb, QColor(0, 0, 0, 170));
        p.setPen(QColor(theme::Yellow));
        p.drawText(fb, Qt::AlignCenter, flash_);
    }
}
