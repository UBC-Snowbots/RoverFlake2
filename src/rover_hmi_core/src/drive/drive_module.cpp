#include "drive_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSlider>
#include <QFont>
#include <QPainter>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QShortcut>
#include <QSizePolicy>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

// VirtualJoystick — defined here so Q_OBJECT is in the .cpp (avoids AUTOMOC
// header-scanning issues with private include directories).
class VirtualJoystick : public QWidget {
    Q_OBJECT
public:
    explicit VirtualJoystick(QWidget* parent = nullptr);
    float axisX() const { return axis_x_; }
    float axisY() const { return axis_y_; }
    void setAxisFromKey(int dx, int dy);
    void resetAxes();
signals:
    void axisChanged(float x, float y);
protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
private:
    void updateFromPos(const QPointF& pos);
    float axis_x_ = 0.0f;
    float axis_y_ = 0.0f;
    bool  pressed_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// VirtualJoystick implementation
// ─────────────────────────────────────────────────────────────────────────────

VirtualJoystick::VirtualJoystick(QWidget* parent) : QWidget(parent) {
    setMinimumSize(180, 180);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
}

void VirtualJoystick::updateFromPos(const QPointF& pos) {
    QPointF center(width() / 2.0, height() / 2.0);
    float radius = std::min(width(), height()) / 2.0f - 10;
    QPointF delta = pos - center;
    float dist = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
    if (dist > radius) {
        delta *= (radius / dist);
    }
    axis_x_ =  delta.x() / radius;
    axis_y_ = -delta.y() / radius;  // screen Y is flipped
    update();
    emit axisChanged(axis_x_, axis_y_);
}

void VirtualJoystick::resetAxes() {
    axis_x_ = 0.0f;
    axis_y_ = 0.0f;
    pressed_ = false;
    update();
    emit axisChanged(0.0f, 0.0f);
}

void VirtualJoystick::setAxisFromKey(int dx, int dy) {
    axis_x_ = std::max(-1.0f, std::min(1.0f, axis_x_ + dx * 0.2f));
    axis_y_ = std::max(-1.0f, std::min(1.0f, axis_y_ + dy * 0.2f));
    update();
    emit axisChanged(axis_x_, axis_y_);
}

void VirtualJoystick::mousePressEvent(QMouseEvent* e) {
    pressed_ = true;
    updateFromPos(e->pos());
}

void VirtualJoystick::mouseMoveEvent(QMouseEvent* e) {
    if (pressed_) updateFromPos(e->pos());
}

void VirtualJoystick::mouseReleaseEvent(QMouseEvent*) {
    resetAxes();
}

void VirtualJoystick::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    QPointF center(width() / 2.0, height() / 2.0);
    float radius = std::min(width(), height()) / 2.0f - 10;

    // Outer ring
    p.setPen(QPen(QColor(theme::Border), 2));
    p.setBrush(QColor(theme::BgPanel));
    p.drawEllipse(center, radius, radius);

    // Crosshair
    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawLine(QPointF(center.x() - radius, center.y()),
               QPointF(center.x() + radius, center.y()));
    p.drawLine(QPointF(center.x(), center.y() - radius),
               QPointF(center.x(), center.y() + radius));

    // Direction zones (forward = top half of circle, green tint)
    if (axis_y_ > 0.05f) {
        p.setBrush(QColor(0, 255, 136, 30));
        p.setPen(Qt::NoPen);
        p.drawEllipse(center, radius - 2, radius - 2);
    } else if (axis_y_ < -0.05f) {
        p.setBrush(QColor(255, 68, 102, 30));
        p.setPen(Qt::NoPen);
        p.drawEllipse(center, radius - 2, radius - 2);
    }

    // Knob
    float kx = center.x() + axis_x_ * radius;
    float ky = center.y() - axis_y_ * radius;
    float krad = 20.0f;

    p.setPen(QPen(QColor(theme::Cyan), 2));
    p.setBrush(QColor(theme::Cyan).darker(200));
    p.drawEllipse(QPointF(kx, ky), krad, krad);

    // Speed vector line
    p.setPen(QPen(QColor(theme::Cyan), 2));
    p.drawLine(center, QPointF(kx, ky));
}


// ─────────────────────────────────────────────────────────────────────────────
// DriveModule
// ─────────────────────────────────────────────────────────────────────────────

QWidget* DriveModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* outer  = new QVBoxLayout(widget);
    outer->setContentsMargins(12, 12, 12, 12);
    outer->setSpacing(10);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);
    QFont monoLg("monospace", theme::FontSizeLg, QFont::Bold);

    // ── Status bar ────────────────────────────────────────────────────────────
    status_lbl_ = new QLabel("Drive ready");
    status_lbl_->setFont(monoBold);
    status_lbl_->setStyleSheet(
        QString("color: %1; padding: 4px;").arg(theme::TextDim));
    status_lbl_->setAlignment(Qt::AlignCenter);
    outer->addWidget(status_lbl_);

    // ── Main row: joystick + speed controls ──────────────────────────────────
    auto* main_row = new QHBoxLayout();
    outer->addLayout(main_row, 1);

    // Joystick
    joystick_ = new VirtualJoystick(widget);
    main_row->addWidget(joystick_, 2);

    // Right panel: speed + live readouts
    auto* right_col = new QVBoxLayout();
    right_col->setSpacing(8);
    main_row->addLayout(right_col, 1);

    auto* speed_header = new QLabel("Max Speed");
    speed_header->setFont(monoBold);
    speed_header->setStyleSheet(QString("color: %1;").arg(theme::Text));
    speed_header->setAlignment(Qt::AlignCenter);
    right_col->addWidget(speed_header);

    auto* slider = new QSlider(Qt::Vertical, widget);
    slider->setRange(1, 20);   // 0.1 to 2.0 m/s in steps of 0.1
    slider->setValue(5);       // default 0.5 m/s
    slider->setStyleSheet(
        QString("QSlider::groove:vertical { background: %1; width: 8px; border-radius: 4px; }"
                "QSlider::handle:vertical { background: %2; height: 18px; width: 18px;"
                "  margin: -5px -5px; border-radius: 9px; }"
                "QSlider::sub-page:vertical { background: %3; border-radius: 4px; }")
        .arg(theme::BgPanel).arg(theme::Cyan).arg(theme::Cyan));
    right_col->addWidget(slider, 1, Qt::AlignHCenter);

    speed_lbl_ = new QLabel("0.5 m/s");
    speed_lbl_->setFont(monoLg);
    speed_lbl_->setStyleSheet(QString("color: %1;").arg(theme::Cyan));
    speed_lbl_->setAlignment(Qt::AlignCenter);
    right_col->addWidget(speed_lbl_);

    QObject::connect(slider, &QSlider::valueChanged, [this](int v) {
        max_linear_ = v * 0.1f;
        speed_lbl_->setText(QString("%1 m/s").arg(max_linear_, 0, 'f', 1));
    });

    // ── Emergency stop button ─────────────────────────────────────────────────
    auto* estop = new QPushButton("■  STOP", widget);
    estop->setFont(monoBold);
    estop->setStyleSheet(
        QString("QPushButton { background: %1; color: %2; border: 2px solid %2;"
                "  border-radius: 8px; padding: 14px; font-size: %3px; }"
                "QPushButton:hover { background: #3d1b1b; }"
                "QPushButton:pressed { background: #5a1e1e; }")
        .arg(theme::Bg).arg(theme::Red).arg(theme::FontSizeLg));
    outer->addWidget(estop);

    QObject::connect(estop, &QPushButton::clicked, [this]() { onKeyStop(); });

    // ── Keyboard shortcuts (W/A/S/D + Space) ─────────────────────────────────
    // These are local shortcuts — only active when the module's widget is in focus.
    // We install them on the parent window with ApplicationShortcut scope but
    // check enabled_ so they don't interfere with other modules when Drive is
    // hidden.
    auto bind = [widget](const char* seq, std::function<void()> fn) {
        auto* sc = new QShortcut(QKeySequence(seq), widget);
        sc->setContext(Qt::ApplicationShortcut);
        QObject::connect(sc, &QShortcut::activated, fn);
    };

    bind("W",     [this]() { joystick_->setAxisFromKey(0, 1);  });
    bind("S",     [this]() { joystick_->setAxisFromKey(0, -1); });
    bind("A",     [this]() { joystick_->setAxisFromKey(-1, 0); });
    bind("D",     [this]() { joystick_->setAxisFromKey(1, 0);  });
    bind("Space", [this]() { onKeyStop(); });

    // ── Wire joystick to publisher ────────────────────────────────────────────
    QObject::connect(joystick_, &VirtualJoystick::axisChanged, [this](float x, float y) {
        if (!enabled_ || !pub_) return;
        if (status_lbl_) {
            QString dir;
            if      (y > 0.1f)  dir = QString("▲ Forward  %.0f%%").arg(y * 100);
            else if (y < -0.1f) dir = QString("▼ Backward  %.0f%%").arg(-y * 100);
            else if (x > 0.1f)  dir = QString("▶ Turning right  %.0f%%").arg(x * 100);
            else if (x < -0.1f) dir = QString("◀ Turning left  %.0f%%").arg(-x * 100);
            else                dir = "Idle";
            status_lbl_->setText(dir);
            status_lbl_->setStyleSheet(
                QString("color: %1; padding: 4px;")
                .arg((std::abs(x) > 0.05f || std::abs(y) > 0.05f) ? theme::Green : theme::TextDim));
        }
    });

    // Publish timer — 20 Hz
    publish_timer_ = new QTimer(widget);
    QObject::connect(publish_timer_, &QTimer::timeout, [this]() { onPublishTimer(); });

    enabled_ = true;
    return widget;
}

void DriveModule::setNode(rclcpp::Node::SharedPtr node) {
    pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void DriveModule::start() {
    if (publish_timer_) publish_timer_->start(50);  // 20 Hz
}

void DriveModule::stop() {
    enabled_ = false;
    if (publish_timer_) publish_timer_->stop();
    // Send zero velocity on shutdown
    if (pub_) publishTwist(0.0f, 0.0f);
}

void DriveModule::onPublishTimer() {
    if (!enabled_ || !pub_ || !joystick_) return;
    float x = joystick_->axisX();
    float y = joystick_->axisY();
    if (std::abs(x) < 0.02f && std::abs(y) < 0.02f) {
        // Dead-zone: publish one zero Twist on entry — otherwise the last
        // non-zero command stays live on the wire after joystick release and
        // the rover only stops if the consumer happens to run a watchdog.
        if (was_moving_) {
            was_moving_ = false;
            publishTwist(0.0f, 0.0f);
        }
        return;
    }
    was_moving_ = true;
    publishTwist(y * max_linear_, -x * max_angular_);
}

void DriveModule::onKeyStop() {
    if (joystick_) joystick_->resetAxes();
    if (pub_)      publishTwist(0.0f, 0.0f);
    if (status_lbl_) {
        status_lbl_->setText("STOPPED");
        status_lbl_->setStyleSheet(
            QString("color: %1; padding: 4px; font-weight: bold;").arg(theme::Red));
    }
}

void DriveModule::publishTwist(float linear_x, float angular_z) {
    if (!pub_) return;
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = linear_x;
    msg.angular.z = angular_z;
    pub_->publish(msg);
}

PLUGINLIB_EXPORT_CLASS(DriveModule, rover_hmi_core::GuiModule)
#include "drive_module.moc"
