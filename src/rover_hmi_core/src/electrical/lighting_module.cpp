// lighting_module.cpp — "Lighting"
//
// Spatial top-down rover view for five LED boards. Any interaction publishes
// the whole desired array to /lights/cmd; /lights/feedback drives the display
// (sliders synced with signals blocked so echoes never re-publish).

#include "lighting_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QRadialGradient>
#include <QSizePolicy>
#include <QTimer>

#include <pluginlib/class_list_macros.hpp>

static const char* CLUSTER_NAMES[]  = { "FRONT", "LEFT", "RIGHT", "BACK" };
static const char* CLUSTER_ACCENT[] = { theme::Cyan, theme::Yellow, theme::Green, "#ff9944" };

// Which wire board(s) each UI cluster drives (-1 = unused slot).
// The FRONT cluster gangs both front boards behind one control.
const int LightingModule::CLUSTER_BOARDS[LightingModule::NUM_CLUSTERS][2] = {
    { BOARD_FRONT_LEFT, BOARD_FRONT_RIGHT },
    { BOARD_LEFT,  -1 },
    { BOARD_RIGHT, -1 },
    { BOARD_BACK,  -1 },
};

// Shared by cluster + master sliders; oversized handle for glove-friendly clicks.
static QString sliderStyle() {
    return QString(
        "QSlider { min-height: 34px; }"
        "QSlider::groove:horizontal { height: 10px; background: %1; border-radius: 5px; }"
        "QSlider::handle:horizontal { width: 26px; height: 26px; background: %2;"
        " border-radius: 13px; margin: -8px 0; }"
        "QSlider::sub-page:horizontal { background: %3; border-radius: 5px; }")
        .arg(theme::BgPanel).arg(theme::Text).arg(theme::Cyan);
}

// ---------------------------------------------------------------- rover view

RoverLightingView::RoverLightingView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(200, 240);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void RoverLightingView::setPercents(const std::array<double, 5>& percents,
                                    bool have_feedback) {
    percents_ = percents;
    have_feedback_ = have_feedback;
    update();
}

void RoverLightingView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(theme::Bg));

    // Square rover body centered in the widget, with margin around it for
    // glow halos (the physical rover is square).
    const double side = qMin(width(), height()) * 0.62;
    QRectF body((width() - side) / 2.0, (height() - side) / 2.0, side, side);
    p.setPen(QPen(QColor(have_feedback_ ? theme::Border : theme::BorderDim), 2));
    p.setBrush(QColor(theme::BgPanel));
    p.drawRoundedRect(body, 14, 14);

    // Chevron above the body marks the front
    p.setPen(QPen(QColor(theme::TextDim), 2));
    const double cx = body.center().x();
    const double cy = body.top() - (height() - side) * 0.25;
    p.drawLine(QPointF(cx - 10, cy + 5), QPointF(cx, cy - 5));
    p.drawLine(QPointF(cx, cy - 5), QPointF(cx + 10, cy + 5));

    // Glyph positions, indexed by board number. The two front lights are
    // forward-facing floodlights mounted along the front edge (not corner
    // lights); sides are mid-edge, back is center-rear.
    const QPointF pos[5] = {
        QPointF(cx - side * 0.18, body.top()),       // 0 front-left floodlight
        QPointF(cx + side * 0.18, body.top()),       // 1 front-right floodlight
        QPointF(body.left(),  body.center().y()),    // 2 left
        QPointF(body.right(), body.center().y()),    // 3 right
        QPointF(cx,           body.bottom()),        // 4 back
    };
    const double r = qMin(width(), height()) * 0.05;

    for (int i = 0; i < 5; i++) {
        // Glyphs show HARDWARE-CONFIRMED state only (from /lights/feedback) —
        // deliberately not the commanded value, so a lit glyph means the light
        // is really on. Until the first feedback, everything draws off/unknown.
        const double pct = have_feedback_ ? qBound(0.0, percents_[i], 100.0) : 0.0;
        if (pct > 0.0) {
            // Lit: warm radial halo whose radius and alpha grow with brightness,
            // plus a solid core dot.
            const double glow_r = r * (2.0 + 3.0 * pct / 100.0);
            QRadialGradient grad(pos[i], glow_r);
            QColor warm(0xff, 0xf2, 0xb0);
            warm.setAlphaF(0.10 + 0.50 * pct / 100.0);
            grad.setColorAt(0.0, warm);
            warm.setAlphaF(0.0);
            grad.setColorAt(1.0, warm);
            p.setPen(Qt::NoPen);
            p.setBrush(grad);
            p.drawEllipse(pos[i], glow_r, glow_r);

            p.setPen(QPen(QColor(theme::Border), 1));
            p.setBrush(QColor(0xff, 0xf6, 0xd0));
            p.drawEllipse(pos[i], r, r);
        } else {
            // Off/unknown: dim outline only.
            p.setPen(QPen(QColor(theme::BorderDim), 2));
            p.setBrush(QColor(theme::BgPanel));
            p.drawEllipse(pos[i], r, r);
        }
    }
}

// ------------------------------------------------------------------- module

QWidget* LightingModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));

    auto* grid = new QGridLayout(widget);
    grid->setSpacing(8);
    grid->setContentsMargins(8, 8, 8, 8);

    view_ = new RoverLightingView();
    view_->setPercents({}, false);

    // Grid mirrors the physical layout: FRONT across the top, LEFT/RIGHT
    // flanking the rover canvas, BACK across the bottom, then MASTER + status.
    grid->addWidget(makeCluster(0, CLUSTER_NAMES[0], CLUSTER_ACCENT[0]), 0, 0, 1, 3);
    grid->addWidget(makeCluster(1, CLUSTER_NAMES[1], CLUSTER_ACCENT[1]), 1, 0);
    grid->addWidget(view_,                                               1, 1);
    grid->addWidget(makeCluster(2, CLUSTER_NAMES[2], CLUSTER_ACCENT[2]), 1, 2);
    grid->addWidget(makeCluster(3, CLUSTER_NAMES[3], CLUSTER_ACCENT[3]), 2, 0, 1, 3);

    // Master row: slider + ALL ON / ALL OFF
    auto* master = new QWidget();
    master->setStyleSheet(
        QString("background: %1; border: 1px solid %2; border-radius: 6px;")
        .arg(theme::BgPanel).arg(theme::BorderDim));
    auto* mrow = new QHBoxLayout(master);
    mrow->setSpacing(8);
    mrow->setContentsMargins(8, 6, 8, 6);

    auto* mtitle = new QLabel("MASTER");
    mtitle->setStyleSheet(
        QString("color: %1; border: none; font-weight: bold;").arg(theme::Text));
    mrow->addWidget(mtitle);

    master_slider_ = new QSlider(Qt::Horizontal);
    master_slider_->setRange(0, 100);
    master_slider_->setValue(0);
    master_slider_->setStyleSheet(sliderStyle());
    mrow->addWidget(master_slider_, 1);

    master_lbl_ = new QLabel("0%");
    master_lbl_->setMinimumWidth(56);
    master_lbl_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    master_lbl_->setStyleSheet(QString("color: %1; border: none;").arg(theme::Text));
    mrow->addWidget(master_lbl_);

    // Master slider overrides every cluster to the same value, syncing their
    // widgets (signals blocked so they don't each publish) and sending one
    // combined command.
    QObject::connect(master_slider_, &QSlider::valueChanged, [this](int val) {
        master_lbl_->setText(QString("%1%").arg(val));
        for (int c = 0; c < NUM_CLUSTERS; c++) {
            sliders_[c]->blockSignals(true);
            sliders_[c]->setValue(val);
            sliders_[c]->blockSignals(false);
            value_lbls_[c]->setText(QString("%1%").arg(val));
            cluster_on_[c] = (val > 0);
            if (val > 0) remembered_[c] = val;
            applyToggleStyle(c, cluster_on_[c]);
        }
        for (int b = 0; b < NUM_BOARDS; b++) desired_[b] = val;
        publishCmd();
    });

    // ALL ON restores each cluster to its own remembered brightness
    // (not one uniform value).
    auto* all_on = new QPushButton("ALL ON");
    QObject::connect(all_on, &QPushButton::clicked, [this]() {
        for (int c = 0; c < NUM_CLUSTERS; c++) {
            cluster_on_[c] = true;
            sliders_[c]->blockSignals(true);
            sliders_[c]->setValue(int(remembered_[c]));
            sliders_[c]->blockSignals(false);
            value_lbls_[c]->setText(QString("%1%").arg(int(remembered_[c])));
            applyToggleStyle(c, true);
            for (int k = 0; k < 2; k++) {
                const int b = CLUSTER_BOARDS[c][k];
                if (b >= 0) desired_[b] = remembered_[c];
            }
        }
        publishCmd();
    });
    mrow->addWidget(all_on);

    // ALL OFF zeroes every board but keeps each cluster's brightness remembered
    // so ALL ON / toggles can bring the same levels back.
    auto* all_off = new QPushButton("ALL OFF");
    QObject::connect(all_off, &QPushButton::clicked, [this]() {
        for (int c = 0; c < NUM_CLUSTERS; c++) {
            if (sliders_[c]->value() > 0) remembered_[c] = sliders_[c]->value();
            cluster_on_[c] = false;
            applyToggleStyle(c, false);
        }
        for (int b = 0; b < NUM_BOARDS; b++) desired_[b] = 0.0;
        publishCmd();
    });
    mrow->addWidget(all_off);

    grid->addWidget(master, 3, 0, 1, 3);

    status_ = new QLabel("Waiting for /lights/feedback...");
    status_->setStyleSheet(
        QString("color: %1; padding: 4px 6px; border: 1px solid %2;")
        .arg(theme::TextDim).arg(theme::BorderDim));
    grid->addWidget(status_, 4, 0, 1, 3);

    // No-feedback watchdog (single-shot, re-armed by every publish, cancelled
    // by every feedback). Firing means commands are going unanswered — the
    // glyphs are intentionally dark, so tell the operator why.
    confirm_timer_ = new QTimer(widget);
    confirm_timer_->setSingleShot(true);
    QObject::connect(confirm_timer_, &QTimer::timeout, [this]() {
        if (status_) {
            status_->setText(
                "\xe2\x9a\xa0 No response on /lights/feedback \xc2\xb7 "
                "commands sent but unconfirmed \xc2\xb7 glyphs show confirmed state only");
            status_->setStyleSheet(
                QString("color: %1; padding: 4px 6px; border: 1px solid %1; font-weight: bold;")
                .arg(theme::Yellow));
        }
    });

    grid->setColumnStretch(0, 2);
    grid->setColumnStretch(1, 3);
    grid->setColumnStretch(2, 2);
    grid->setRowStretch(1, 1);

    return widget;
}

QWidget* LightingModule::makeCluster(int cluster, const char* title,
                                     const char* accent) {
    auto* box = new QWidget();
    box->setStyleSheet(
        QString("background: %1; border: 1px solid %2; border-radius: 6px;")
        .arg(theme::BgPanel).arg(theme::BorderDim));
    auto* lay = new QVBoxLayout(box);
    lay->setSpacing(6);
    lay->setContentsMargins(8, 6, 8, 6);

    auto* title_lbl = new QLabel(title);
    title_lbl->setAlignment(Qt::AlignCenter);
    title_lbl->setStyleSheet(
        QString("color: %1; border: none; font-weight: bold; font-size: %2px;")
        .arg(accent).arg(theme::FontSize));
    lay->addWidget(title_lbl);

    // Toggle works like a dimmer switch: there is no on/off bit on the wire,
    // so OFF just commands 0% (remembering the slider), ON re-commands the
    // remembered brightness.
    toggle_btns_[cluster] = new QPushButton("OFF");
    applyToggleStyle(cluster, false);
    QObject::connect(toggle_btns_[cluster], &QPushButton::clicked, [this, cluster]() {
        cluster_on_[cluster] = !cluster_on_[cluster];
        if (cluster_on_[cluster]) {
            sliders_[cluster]->blockSignals(true);
            sliders_[cluster]->setValue(int(remembered_[cluster]));
            sliders_[cluster]->blockSignals(false);
            value_lbls_[cluster]->setText(QString("%1%").arg(int(remembered_[cluster])));
            setClusterValue(cluster, remembered_[cluster]);
        } else {
            if (sliders_[cluster]->value() > 0)
                remembered_[cluster] = sliders_[cluster]->value();
            setClusterValue(cluster, 0.0);
        }
        applyToggleStyle(cluster, cluster_on_[cluster]);
    });
    lay->addWidget(toggle_btns_[cluster]);

    auto* row = new QHBoxLayout();
    row->setSpacing(6);

    sliders_[cluster] = new QSlider(Qt::Horizontal);
    sliders_[cluster]->setRange(0, 100);
    sliders_[cluster]->setValue(0);
    sliders_[cluster]->setStyleSheet(sliderStyle());
    // Dragging publishes live; any nonzero position counts as "on" and becomes
    // the value the toggle will restore.
    QObject::connect(sliders_[cluster], &QSlider::valueChanged, [this, cluster](int val) {
        value_lbls_[cluster]->setText(QString("%1%").arg(val));
        cluster_on_[cluster] = (val > 0);
        if (val > 0) remembered_[cluster] = val;
        applyToggleStyle(cluster, cluster_on_[cluster]);
        setClusterValue(cluster, val);
    });
    row->addWidget(sliders_[cluster], 1);

    value_lbls_[cluster] = new QLabel("0%");
    value_lbls_[cluster]->setMinimumWidth(56);
    value_lbls_[cluster]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value_lbls_[cluster]->setStyleSheet(
        QString("color: %1; border: none;").arg(theme::Text));
    row->addWidget(value_lbls_[cluster]);

    lay->addLayout(row);
    return box;
}

void LightingModule::applyToggleStyle(int cluster, bool on) {
    if (!toggle_btns_[cluster]) return;
    toggle_btns_[cluster]->setText(on ? "ON" : "OFF");
    if (on) {
        toggle_btns_[cluster]->setStyleSheet(
            QString("background: #1b3d2a; color: %1; border: 1px solid %1;"
                    " border-radius: 4px; padding: 6px; font-weight: bold;")
            .arg(theme::Green));
    } else {
        toggle_btns_[cluster]->setStyleSheet(
            QString("background: %1; color: %2; border: 1px solid %2;"
                    " border-radius: 4px; padding: 6px; font-weight: bold;")
            .arg(theme::BgPanel).arg(theme::TextDim));
    }
}

// Write one cluster's board(s) into the desired array and broadcast it.
void LightingModule::setClusterValue(int cluster, double pct) {
    for (int k = 0; k < 2; k++) {
        const int b = CLUSTER_BOARDS[cluster][k];
        if (b >= 0) desired_[b] = pct;
    }
    publishCmd();
}

// Wire contract: every command carries the full 5-board array (never a delta).
void LightingModule::publishCmd() {
    if (!pub_) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(desired_, desired_ + NUM_BOARDS);
    pub_->publish(msg);

    // Glyphs deliberately do NOT light up here — they only show what the
    // hardware confirms. Instead, arm the no-feedback watchdog: if nothing
    // answers within the window, the status line turns into a warning.
    if (confirm_timer_) confirm_timer_->start(FEEDBACK_TIMEOUT_MS);
}

void LightingModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    // Reliable QoS: lighting commands are sparse and must not be dropped.
    auto qos = rclcpp::QoS(10).reliable();
    pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/lights/cmd", qos);
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/lights/feedback", qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            onFeedback(msg);
        });
}

// Hardware-confirmed state from the CAN bridge. Runs on the Qt thread (the
// host spins ROS from a QTimer), so touching widgets directly is safe.
void LightingModule::onFeedback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < static_cast<size_t>(NUM_BOARDS)) return;  // malformed

    std::array<double, 5> fb{};
    for (int b = 0; b < NUM_BOARDS; b++) {
        // Clamp so a bad feedback value can never be shown or re-published.
        fb[b] = qBound(0.0, msg->data[b], 100.0);
        // Adopt confirmed state so the next publish doesn't resurrect stale
        // values for boards the user hasn't touched since.
        desired_[b] = fb[b];
    }
    if (view_) view_->setPercents(fb, true);

    // Sync each cluster's widgets to the confirmed value (front shows FL).
    // Signals are blocked so this sync can never trigger a re-publish loop.
    for (int c = 0; c < NUM_CLUSTERS; c++) {
        const int ival = int(fb[CLUSTER_BOARDS[c][0]] + 0.5);
        if (sliders_[c]) {
            sliders_[c]->blockSignals(true);
            sliders_[c]->setValue(ival);
            sliders_[c]->blockSignals(false);
        }
        if (value_lbls_[c]) value_lbls_[c]->setText(QString("%1%").arg(ival));
        cluster_on_[c] = (ival > 0);
        if (ival > 0) remembered_[c] = ival;
        applyToggleStyle(c, cluster_on_[c]);
    }

    // Feedback arrived: disarm the watchdog and clear any warning styling.
    if (confirm_timer_) confirm_timer_->stop();
    if (status_) {
        status_->setText("Live  \xc2\xb7  /lights/feedback");
        status_->setStyleSheet(
            QString("color: %1; padding: 4px 6px; border: 1px solid %2;")
            .arg(theme::Green).arg(theme::BorderDim));
    }
}

PLUGINLIB_EXPORT_CLASS(LightingModule, rover_hmi_core::GuiModule)
