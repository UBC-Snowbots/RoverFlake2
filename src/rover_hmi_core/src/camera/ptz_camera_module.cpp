// ptz_camera_module.cpp — "PTZ Camera"
//
// Control contract inherited from the old rover_hmi dashboard: pan, tilt AND
// zoom all ride one geometry_msgs/Vector3 on /ptz/control (x=pan, y=tilt,
// z=zoom; sign = direction, 0 = stop). Press starts a continuous move,
// release stops it.

#include "ptz_camera_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QSlider>
#include <QPainter>
#include <QKeyEvent>
#include <QApplication>
#include <QLineEdit>
#include <QTextEdit>
#include <QAbstractSpinBox>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// PtzPreview — aspect-fit canvas for the latest camera frame
// ─────────────────────────────────────────────────────────────────────────────
class PtzPreview : public QWidget {
public:
    explicit PtzPreview(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumSize(320, 240);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }
    void setFrame(const QImage& img) { frame_ = img; stale_ = false; update(); }
    void setStale() { stale_ = true; update(); }
    void setIdleImage(const QImage& img) { idle_ = img; update(); }
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.fillRect(rect(), QColor(theme::Bg));
        if (frame_.isNull()) {
            // Camera never connected: wallpaper centered at natural size
            // (deliberately not scaled to fit).
            if (!idle_.isNull())
                p.drawImage(QPoint((width() - idle_.width()) / 2,
                                   (height() - idle_.height()) / 2), idle_);
            p.setPen(QColor(theme::TextDim));
            p.setFont(QFont("monospace", theme::FontSizeSm, QFont::Bold));
            p.drawText(rect().adjusted(0, 0, 0, -6),
                       Qt::AlignBottom | Qt::AlignHCenter, "NO SIGNAL");
        } else {
            QSize s = frame_.size().scaled(size(), Qt::KeepAspectRatio);
            QRect target(QPoint((width() - s.width()) / 2, (height() - s.height()) / 2), s);
            p.setRenderHint(QPainter::SmoothPixmapTransform);
            p.drawImage(target, frame_);
            if (stale_) {
                p.fillRect(target, QColor(0, 0, 0, 160));  // dim the frozen frame
                p.setPen(QColor(theme::Yellow));
                p.setFont(QFont("monospace", theme::FontSizeLg, QFont::Bold));
                p.drawText(rect(), Qt::AlignCenter, "FEED STALE");
            }
        }
        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.drawRect(rect().adjusted(0, 0, -1, -1));
    }
private:
    QImage frame_;
    QImage idle_;
    bool   stale_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// PtzKeyFilter — app-wide key press AND release events.
// QShortcut cannot report key release, which continuous motor moves need
// (release = stop). Auto-repeat is filtered so holding a key is one move.
// ─────────────────────────────────────────────────────────────────────────────
class PtzKeyFilter : public QObject {
public:
    using Handler = std::function<bool(int key, bool pressed)>;
    PtzKeyFilter(QWidget* gate, Handler h)
        : QObject(gate), gate_(gate), handler_(std::move(h)) {}
protected:
    bool eventFilter(QObject*, QEvent* ev) override {
        const bool press = ev->type() == QEvent::KeyPress;
        if (!press && ev->type() != QEvent::KeyRelease) return false;
        auto* ke = static_cast<QKeyEvent*>(ev);
        if (ke->isAutoRepeat() || ke->modifiers() != Qt::NoModifier) return false;
        if (!gate_ || !gate_->isVisible()) return false;
        QWidget* fw = QApplication::focusWidget();  // let text entry keep its keys
        if (qobject_cast<QLineEdit*>(fw) || qobject_cast<QTextEdit*>(fw) ||
            qobject_cast<QAbstractSpinBox*>(fw)) return false;
        return handler_(ke->key(), press);  // true = consumed
    }
private:
    QWidget* gate_;
    Handler  handler_;
};

// ─────────────────────────────────────────────────────────────────────────────
// PtzCameraModule
// ─────────────────────────────────────────────────────────────────────────────

QWidget* PtzCameraModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* outer  = new QVBoxLayout(widget);
    outer->setContentsMargins(12, 12, 12, 12);
    outer->setSpacing(8);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);
    QFont monoSm("monospace", theme::FontSizeSm);

    // ── Status bar ──
    status_lbl_ = new QLabel("PTZ ready");
    status_lbl_->setFont(monoBold);
    status_lbl_->setAlignment(Qt::AlignCenter);
    status_lbl_->setStyleSheet(QString("color: %1; padding: 2px;").arg(theme::TextDim));
    outer->addWidget(status_lbl_);

    // ── Main row: preview | controls ──
    auto* main_row = new QHBoxLayout();
    main_row->setSpacing(10);
    outer->addLayout(main_row, 1);

    auto* feed_col = new QVBoxLayout();
    preview_ = new PtzPreview(widget);
    // Installed by rover_hmi_core's CMake; missing file just leaves a black idle canvas.
    try {
        const auto share = ament_index_cpp::get_package_share_directory("rover_hmi_core");
        preview_->setIdleImage(QImage(QString::fromStdString(share + "/assets/cats/cat_square.png")));
    } catch (const std::exception&) {}
    feed_col->addWidget(preview_, 1);
    feed_lbl_ = new QLabel("/ip_camera/image_raw — waiting for frames");
    feed_lbl_->setFont(monoSm);
    feed_lbl_->setAlignment(Qt::AlignCenter);
    feed_lbl_->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    feed_col->addWidget(feed_lbl_);
    main_row->addLayout(feed_col, 3);

    auto* ctl_col = new QVBoxLayout();
    ctl_col->setSpacing(8);
    main_row->addLayout(ctl_col, 1);

    // Press-and-hold button: press starts the move, release stops it.
    auto hold_btn = [&](const QString& text, std::function<void()> on_press,
                        std::function<void()> on_release) {
        auto* b = new QPushButton(text, widget);
        b->setFont(monoBold);
        b->setMinimumHeight(48);
        b->setFocusPolicy(Qt::NoFocus);  // keep keyboard focus off buttons so Space doesn't retrigger
        QObject::connect(b, &QPushButton::pressed,  on_press);
        QObject::connect(b, &QPushButton::released, on_release);
        return b;
    };

    // ── Pan/tilt pad — same arrangement as the old dashboard's ptz_grid:
    //    pan buttons flank the stacked tilt pair ──
    auto* pad = new QGridLayout();
    pad->setSpacing(6);
    auto pt = [&](const QString& t, int dx, int dy) {
        return hold_btn(t, [this, dx, dy]() { panTiltEvent(dx, dy, true);  },
                           [this, dx, dy]() { panTiltEvent(dx, dy, false); });
    };
    auto* pan_dec = pt("− PAN −", -1, 0);
    auto* pan_inc = pt("+ PAN +", +1, 0);
    // Buttons default to a Fixed vertical policy, which leaves the two-row pan
    // cells half-empty; Expanding stretches them to the full span height.
    pan_dec->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    pan_inc->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    pad->addWidget(pan_dec, 0, 0, 2, 1);
    pad->addWidget(pt("+ TILT +",  0, +1), 0, 1);
    pad->addWidget(pt("− TILT −",  0, -1), 1, 1);
    pad->addWidget(pan_inc, 0, 2, 2, 1);
    ctl_col->addLayout(pad);

    // ── Zoom row — directly under the pad, as in the old dashboard ──
    auto* zoom_row = new QHBoxLayout();
    zoom_row->setSpacing(6);
    zoom_row->addWidget(hold_btn("ZOOM OUT", [this]() { zoomEvent(-1, true);  },
                                             [this]() { zoomEvent(-1, false); }));
    zoom_row->addWidget(hold_btn("ZOOM IN",  [this]() { zoomEvent(+1, true);  },
                                             [this]() { zoomEvent(+1, false); }));
    ctl_col->addLayout(zoom_row);

    // ── Speed multiplier for /ptz/control pan/tilt (driver adds its own serial scaling) ──
    auto* speed_row = new QHBoxLayout();
    auto* speed_lbl = new QLabel("Speed ×1");
    speed_lbl->setFont(mono);
    auto* slider = new QSlider(Qt::Horizontal, widget);
    slider->setRange(1, 5);
    slider->setValue(1);
    QObject::connect(slider, &QSlider::valueChanged, [this, speed_lbl](int v) {
        speed_ = float(v);
        speed_lbl->setText(QString("Speed ×%1").arg(v));
        if (pan_dir_ || tilt_dir_) publishControl();  // live-update an in-progress move
    });
    speed_row->addWidget(speed_lbl);
    speed_row->addWidget(slider, 1);
    ctl_col->addLayout(speed_row);

    // ── Stop-all ──
    auto* stop_btn = new QPushButton("■ STOP", widget);
    stop_btn->setFont(monoBold);
    stop_btn->setMinimumHeight(48);
    stop_btn->setFocusPolicy(Qt::NoFocus);
    stop_btn->setStyleSheet(QString("color: %1; border-color: %1;").arg(theme::Red));
    QObject::connect(stop_btn, &QPushButton::clicked, [this]() { stopAll(); });
    ctl_col->addWidget(stop_btn);
    ctl_col->addStretch(1);

    // ── Keyboard: I/J/K/L pan-tilt, Z/X zoom — press/release ──
    key_filter_ = new PtzKeyFilter(widget, [this](int key, bool pressed) {
        return onKey(key, pressed);
    });

    // Stale-feed / fps watchdog
    health_timer_ = new QTimer(widget);
    QObject::connect(health_timer_, &QTimer::timeout, [this]() { onHealthTick(); });

    return widget;
}

void PtzCameraModule::setNode(rclcpp::Node::SharedPtr node) {
    pt_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>("/ptz/control", 10);
    // Best-effort keeps the HMI happy over the lossy radio link (compatible with
    // gscam's reliable publisher either way).
    img_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/ip_camera/image_raw", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });
}

void PtzCameraModule::start() {
    enabled_ = true;
    if (key_filter_) qApp->installEventFilter(key_filter_);
    if (health_timer_) health_timer_->start(1000);
}

void PtzCameraModule::stop() {
    enabled_ = false;
    if (key_filter_) qApp->removeEventFilter(key_filter_);
    if (health_timer_) health_timer_->stop();
    // Leave the camera parked: zero every axis (also stops any zoom move).
    pan_dir_ = tilt_dir_ = zoom_dir_ = 0;
    if (pt_pub_) publishControl();
}

// ── Video feed ──

// gscam is configured for rgb8; accept the other common raw encodings too.
// .copy() detaches from the ROS buffer, which dies with the message.
QImage PtzCameraModule::imageMsgToQImage(const sensor_msgs::msg::Image& msg) {
    QImage::Format fmt;
    if      (msg.encoding == "rgb8")  fmt = QImage::Format_RGB888;
    else if (msg.encoding == "bgr8")  fmt = QImage::Format_BGR888;
    else if (msg.encoding == "mono8") fmt = QImage::Format_Grayscale8;
    else return {};
    return QImage(msg.data.data(), msg.width, msg.height, msg.step, fmt).copy();
}

void PtzCameraModule::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!enabled_ || !preview_) return;
    QImage frame = imageMsgToQImage(*msg);
    if (frame.isNull()) return;
    preview_->setFrame(frame);
    frame_w_ = msg->width;
    frame_h_ = msg->height;
    ++frames_;
    last_frame_ = std::chrono::steady_clock::now();
}

void PtzCameraModule::onHealthTick() {
    using namespace std::chrono;
    if (last_frame_ == steady_clock::time_point{}) return;  // never had a frame
    const auto age = duration_cast<seconds>(steady_clock::now() - last_frame_).count();
    if (age >= 2) {
        preview_->setStale();
        feed_lbl_->setText(QString("/ip_camera/image_raw — stale (%1s)").arg(age));
        feed_lbl_->setStyleSheet(QString("color: %1;").arg(theme::Yellow));
    } else {
        feed_lbl_->setText(QString("/ip_camera/image_raw — %1×%2 @ %3 fps")
                               .arg(frame_w_).arg(frame_h_).arg(frames_));
        feed_lbl_->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    }
    frames_ = 0;
}

// ── Controls ──

bool PtzCameraModule::onKey(int key, bool pressed) {
    if (!enabled_) return false;
    switch (key) {
        case Qt::Key_I: panTiltEvent(0, +1, pressed); return true;
        case Qt::Key_K: panTiltEvent(0, -1, pressed); return true;
        case Qt::Key_J: panTiltEvent(-1, 0, pressed); return true;
        case Qt::Key_L: panTiltEvent(+1, 0, pressed); return true;
        case Qt::Key_Z: zoomEvent(+1, pressed); return true;
        case Qt::Key_X: zoomEvent(-1, pressed); return true;
        default: return false;
    }
}

void PtzCameraModule::panTiltEvent(int dx, int dy, bool pressed) {
    // Track the held direction per axis so diagonal holds and out-of-order
    // releases (press J, press I, release J) behave correctly.
    if (pressed) { if (dx) pan_dir_ = dx;                  if (dy) tilt_dir_ = dy; }
    else         { if (dx && pan_dir_ == dx) pan_dir_ = 0; if (dy && tilt_dir_ == dy) tilt_dir_ = 0; }
    publishControl();
}

void PtzCameraModule::zoomEvent(int dz, bool pressed) {
    if (pressed) zoom_dir_ = dz;
    else if (zoom_dir_ == dz) zoom_dir_ = 0;
    publishControl();
}

void PtzCameraModule::publishControl() {
    if (!pt_pub_) return;
    // sign = direction, 0 = stop; pitch_tilt_node applies its own per-axis
    // serial scaling (pan ×2, tilt ×15). z stays at ±1 — the zoom node
    // matches the exact value, so the speed slider must not scale it.
    geometry_msgs::msg::Vector3 msg;
    msg.x = pan_dir_  * speed_;
    msg.y = tilt_dir_ * speed_;
    msg.z = zoom_dir_;
    pt_pub_->publish(msg);

    QString dir;
    if      (tilt_dir_ > 0) dir = "▲ tilt up";
    else if (tilt_dir_ < 0) dir = "▼ tilt down";
    if      (pan_dir_ > 0)  dir += (dir.isEmpty() ? "" : " + ") + QString("▶ pan right");
    else if (pan_dir_ < 0)  dir += (dir.isEmpty() ? "" : " + ") + QString("◀ pan left");
    if      (zoom_dir_ > 0) dir += (dir.isEmpty() ? "" : " + ") + QString("⊕ zoom in");
    else if (zoom_dir_ < 0) dir += (dir.isEmpty() ? "" : " + ") + QString("⊖ zoom out");
    setStatus(dir.isEmpty() ? "ptz stopped" : dir,
              dir.isEmpty() ? theme::TextDim : theme::Green);
}

void PtzCameraModule::stopAll() {
    pan_dir_ = tilt_dir_ = zoom_dir_ = 0;
    publishControl();
}

void PtzCameraModule::setStatus(const QString& text, const char* color) {
    if (!status_lbl_) return;
    status_lbl_->setText(text);
    status_lbl_->setStyleSheet(QString("color: %1; padding: 2px;").arg(color));
}

PLUGINLIB_EXPORT_CLASS(PtzCameraModule, rover_hmi_core::GuiModule)
