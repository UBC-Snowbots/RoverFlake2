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
// The FRONT cluster gangs both front boards behind one control (when linked).
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

// Big squarish touch-target buttons (generous vertical padding, modest radius).
static QString buttonStyle(const QString& bg, const QString& fg) {
    return QString(
        "QPushButton { background: %1; color: %2; border: 2px solid %2;"
        " border-radius: 6px; padding: 16px 20px; font-weight: bold;"
        " font-size: %3px; }")
        .arg(bg).arg(fg).arg(theme::FontSizeLg);
}

// Value/tag label styling at the enlarged control font size.
static QString labelStyle(const QString& color) {
    return QString("color: %1; border: none; font-size: %2px;")
        .arg(color).arg(theme::FontSizeLg);
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
        QString("color: %1; border: none; font-weight: bold; font-size: %2px;")
        .arg(theme::Text).arg(theme::FontSizeLg));
    mrow->addWidget(mtitle);

    master_slider_ = new QSlider(Qt::Horizontal);
    master_slider_->setRange(0, 100);
    master_slider_->setValue(0);
    master_slider_->setStyleSheet(sliderStyle());
    mrow->addWidget(master_slider_, 1);

    master_lbl_ = new QLabel("0%");
    master_lbl_->setMinimumWidth(72);
    master_lbl_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    master_lbl_->setStyleSheet(labelStyle(theme::Text));
    mrow->addWidget(master_lbl_);

    // Master slider overrides every cluster (and both split front sliders) to
    // the same value, syncing their widgets with signals blocked so they don't
    // each publish, then sends one combined command.
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
        if (fr_slider_) {
            fr_slider_->blockSignals(true);
            fr_slider_->setValue(val);
            fr_slider_->blockSignals(false);
            fr_lbl_->setText(QString("%1%").arg(val));
            if (val > 0) remembered_fr_ = val;
        }
        for (int b = 0; b < NUM_BOARDS; b++) desired_[b] = val;
        publishCmd();
    });

    // ALL ON restores each cluster to its own remembered brightness
    // (not one uniform value).
    auto* all_on = new QPushButton("ALL ON");
    all_on->setStyleSheet(buttonStyle(theme::BgPanel, theme::Green));
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
        // Split front pair keeps its own FR memory.
        if (!front_linked_ && fr_slider_) {
            fr_slider_->blockSignals(true);
            fr_slider_->setValue(int(remembered_fr_));
            fr_slider_->blockSignals(false);
            fr_lbl_->setText(QString("%1%").arg(int(remembered_fr_)));
            desired_[BOARD_FRONT_RIGHT] = remembered_fr_;
        }
        publishCmd();
    });
    mrow->addWidget(all_on);

    // ALL OFF zeroes every board but keeps each cluster's brightness remembered
    // so ALL ON / toggles can bring the same levels back.
    auto* all_off = new QPushButton("ALL OFF");
    all_off->setStyleSheet(buttonStyle(theme::BgPanel, theme::Red));
    QObject::connect(all_off, &QPushButton::clicked, [this]() {
        for (int c = 0; c < NUM_CLUSTERS; c++) {
            if (sliders_[c]->value() > 0) remembered_[c] = sliders_[c]->value();
            cluster_on_[c] = false;
            applyToggleStyle(c, false);
        }
        if (fr_slider_ && fr_slider_->value() > 0) remembered_fr_ = fr_slider_->value();
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
        .arg(accent).arg(theme::FontSizeLg));

    if (cluster == 0) {
        // Front title row also hosts the LINKED/SPLIT mode button, which flips
        // between driving both floodlights together and independently.
        auto* trow = new QHBoxLayout();
        trow->setSpacing(6);
        trow->addWidget(title_lbl, 1);
        link_btn_ = new QPushButton("LINKED");
        link_btn_->setToolTip("Toggle between one control for both front lights (LINKED)\n"
                              "and separate FL / FR sliders (SPLIT)");
        link_btn_->setStyleSheet(buttonStyle(theme::BgPanel, theme::Cyan));
        QObject::connect(link_btn_, &QPushButton::clicked, [this]() {
            setFrontLinked(!front_linked_);
        });
        trow->addWidget(link_btn_);
        lay->addLayout(trow);
    } else {
        lay->addWidget(title_lbl);
    }

    // Toggle works like a dimmer switch: there is no on/off bit on the wire,
    // so OFF just commands 0% (remembering the slider), ON re-commands the
    // remembered brightness. In split mode it drives both floodlights, each
    // restoring its own memory.
    toggle_btns_[cluster] = new QPushButton("OFF");
    applyToggleStyle(cluster, false);
    QObject::connect(toggle_btns_[cluster], &QPushButton::clicked, [this, cluster]() {
        cluster_on_[cluster] = !cluster_on_[cluster];
        const bool split_front = (cluster == 0 && !front_linked_);
        if (cluster_on_[cluster]) {
            sliders_[cluster]->blockSignals(true);
            sliders_[cluster]->setValue(int(remembered_[cluster]));
            sliders_[cluster]->blockSignals(false);
            value_lbls_[cluster]->setText(QString("%1%").arg(int(remembered_[cluster])));
            if (split_front) {
                fr_slider_->blockSignals(true);
                fr_slider_->setValue(int(remembered_fr_));
                fr_slider_->blockSignals(false);
                fr_lbl_->setText(QString("%1%").arg(int(remembered_fr_)));
                desired_[BOARD_FRONT_LEFT]  = remembered_[cluster];
                desired_[BOARD_FRONT_RIGHT] = remembered_fr_;
                publishCmd();
            } else {
                setClusterValue(cluster, remembered_[cluster]);
            }
        } else {
            if (sliders_[cluster]->value() > 0)
                remembered_[cluster] = sliders_[cluster]->value();
            if (split_front) {
                if (fr_slider_->value() > 0) remembered_fr_ = fr_slider_->value();
                desired_[BOARD_FRONT_LEFT]  = 0.0;
                desired_[BOARD_FRONT_RIGHT] = 0.0;
                publishCmd();
            } else {
                setClusterValue(cluster, 0.0);
            }
        }
        applyToggleStyle(cluster, cluster_on_[cluster]);
    });
    lay->addWidget(toggle_btns_[cluster]);

    auto* row = new QHBoxLayout();
    row->setSpacing(6);

    if (cluster == 0) {
        // "FL" tag appears only in split mode, when this slider is FL-only.
        fl_tag_ = new QLabel("FL");
        fl_tag_->setStyleSheet(labelStyle(theme::TextDim) + "font-weight: bold;");
        fl_tag_->setVisible(false);
        row->addWidget(fl_tag_);
    }

    // Dragging publishes live; any nonzero position counts as "on" and becomes
    // the value the toggle will restore. In split mode this slider is FL only.
    sliders_[cluster] = new QSlider(Qt::Horizontal);
    sliders_[cluster]->setRange(0, 100);
    sliders_[cluster]->setValue(0);
    sliders_[cluster]->setStyleSheet(sliderStyle());
    QObject::connect(sliders_[cluster], &QSlider::valueChanged, [this, cluster](int val) {
        value_lbls_[cluster]->setText(QString("%1%").arg(val));
        if (val > 0) remembered_[cluster] = val;
        if (cluster == 0 && !front_linked_) {
            cluster_on_[0] = (val > 0 || (fr_slider_ && fr_slider_->value() > 0));
            applyToggleStyle(0, cluster_on_[0]);
            setBoardValue(BOARD_FRONT_LEFT, val);
        } else {
            cluster_on_[cluster] = (val > 0);
            applyToggleStyle(cluster, cluster_on_[cluster]);
            setClusterValue(cluster, val);
        }
    });
    row->addWidget(sliders_[cluster], 1);

    value_lbls_[cluster] = new QLabel("0%");
    value_lbls_[cluster]->setMinimumWidth(72);
    value_lbls_[cluster]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value_lbls_[cluster]->setStyleSheet(labelStyle(theme::Text));
    row->addWidget(value_lbls_[cluster]);

    lay->addLayout(row);

    if (cluster == 0) {
        // Second slider row for the front-right floodlight, hidden while the
        // pair is linked.
        fr_row_ = new QWidget();
        fr_row_->setStyleSheet("border: none;");
        auto* frl = new QHBoxLayout(fr_row_);
        frl->setContentsMargins(0, 0, 0, 0);
        frl->setSpacing(6);

        auto* fr_tag = new QLabel("FR");
        fr_tag->setStyleSheet(labelStyle(theme::TextDim) + "font-weight: bold;");
        frl->addWidget(fr_tag);

        fr_slider_ = new QSlider(Qt::Horizontal);
        fr_slider_->setRange(0, 100);
        fr_slider_->setValue(0);
        fr_slider_->setStyleSheet(sliderStyle());
        QObject::connect(fr_slider_, &QSlider::valueChanged, [this](int val) {
            fr_lbl_->setText(QString("%1%").arg(val));
            if (val > 0) remembered_fr_ = val;
            cluster_on_[0] = (val > 0 || (sliders_[0] && sliders_[0]->value() > 0));
            applyToggleStyle(0, cluster_on_[0]);
            setBoardValue(BOARD_FRONT_RIGHT, val);
        });
        frl->addWidget(fr_slider_, 1);

        fr_lbl_ = new QLabel("0%");
        fr_lbl_->setMinimumWidth(72);
        fr_lbl_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        fr_lbl_->setStyleSheet(labelStyle(theme::Text));
        frl->addWidget(fr_lbl_);

        fr_row_->setVisible(false);
        lay->addWidget(fr_row_);
    }

    return box;
}

void LightingModule::applyToggleStyle(int cluster, bool on) {
    if (!toggle_btns_[cluster]) return;
    toggle_btns_[cluster]->setText(on ? "ON" : "OFF");
    toggle_btns_[cluster]->setStyleSheet(
        on ? buttonStyle("#1b3d2a", theme::Green)
           : buttonStyle(theme::BgPanel, theme::TextDim));
}

// Switch the front pair between one ganged control and independent FL/FR
// sliders. Re-linking snaps both floodlights to the FL slider's value.
void LightingModule::setFrontLinked(bool linked) {
    front_linked_ = linked;
    if (link_btn_) {
        link_btn_->setText(linked ? "LINKED" : "SPLIT");
        // Yellow while split as a visual cue that front is in the non-default mode.
        link_btn_->setStyleSheet(
            buttonStyle(theme::BgPanel, linked ? theme::Cyan : theme::Yellow));
    }
    if (fl_tag_) fl_tag_->setVisible(!linked);
    if (fr_row_) fr_row_->setVisible(!linked);

    if (linked) {
        setClusterValue(0, sliders_[0] ? sliders_[0]->value() : 0.0);
    } else {
        // Split starts from the current per-board desired values.
        const int fr = int(desired_[BOARD_FRONT_RIGHT] + 0.5);
        if (fr_slider_) {
            fr_slider_->blockSignals(true);
            fr_slider_->setValue(fr);
            fr_slider_->blockSignals(false);
        }
        if (fr_lbl_) fr_lbl_->setText(QString("%1%").arg(fr));
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

// Split-mode write: one board only, then broadcast the full array.
void LightingModule::setBoardValue(int board, double pct) {
    desired_[board] = pct;
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

    // Sync each cluster's widgets to the confirmed value (front shows FL, and
    // its FR slider syncs separately when split). Signals are blocked so this
    // sync can never trigger a re-publish loop.
    for (int c = 0; c < NUM_CLUSTERS; c++) {
        const int ival = int(fb[CLUSTER_BOARDS[c][0]] + 0.5);
        if (sliders_[c]) {
            sliders_[c]->blockSignals(true);
            sliders_[c]->setValue(ival);
            sliders_[c]->blockSignals(false);
        }
        if (value_lbls_[c]) value_lbls_[c]->setText(QString("%1%").arg(ival));
        bool on = (ival > 0);
        if (ival > 0) remembered_[c] = ival;
        if (c == 0) {
            const int fr = int(fb[BOARD_FRONT_RIGHT] + 0.5);
            if (fr_slider_) {
                fr_slider_->blockSignals(true);
                fr_slider_->setValue(fr);
                fr_slider_->blockSignals(false);
            }
            if (fr_lbl_) fr_lbl_->setText(QString("%1%").arg(fr));
            if (fr > 0) remembered_fr_ = fr;
            on = on || (fr > 0);
        }
        cluster_on_[c] = on;
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
