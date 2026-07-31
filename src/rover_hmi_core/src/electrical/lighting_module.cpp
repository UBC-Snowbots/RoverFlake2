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

#include <pluginlib/class_list_macros.hpp>

static const char* CLUSTER_NAMES[]  = { "FRONT", "LEFT", "RIGHT", "BACK" };
static const char* CLUSTER_ACCENT[] = { theme::Cyan, theme::Yellow, theme::Green, "#ff9944" };

const int LightingModule::CLUSTER_BOARDS[LightingModule::NUM_CLUSTERS][2] = {
    { BOARD_FRONT_LEFT, BOARD_FRONT_RIGHT },
    { BOARD_LEFT,  -1 },
    { BOARD_RIGHT, -1 },
    { BOARD_BACK,  -1 },
};

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

    // Rover body, with margin left around it for glow halos
    const double mx = width() * 0.20, my = height() * 0.16;
    QRectF body(mx, my, width() - 2 * mx, height() - 2 * my);
    p.setPen(QPen(QColor(have_feedback_ ? theme::Border : theme::BorderDim), 2));
    p.setBrush(QColor(theme::BgPanel));
    p.drawRoundedRect(body, 14, 14);

    // Chevron above the body marks the front
    p.setPen(QPen(QColor(theme::TextDim), 2));
    const double cx = body.center().x();
    const double cy = body.top() - my * 0.5;
    p.drawLine(QPointF(cx - 10, cy + 5), QPointF(cx, cy - 5));
    p.drawLine(QPointF(cx, cy - 5), QPointF(cx + 10, cy + 5));

    // Glyph positions, indexed by board number
    const QPointF pos[5] = {
        QPointF(body.left(),  body.top()),           // 0 front-left
        QPointF(body.right(), body.top()),           // 1 front-right
        QPointF(body.left(),  body.center().y()),    // 2 left
        QPointF(body.right(), body.center().y()),    // 3 right
        QPointF(body.center().x(), body.bottom()),   // 4 back
    };
    const double r = qMin(width(), height()) * 0.05;

    for (int i = 0; i < 5; i++) {
        const double pct = have_feedback_ ? qBound(0.0, percents_[i], 100.0) : 0.0;
        if (pct > 0.0) {
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

void LightingModule::setClusterValue(int cluster, double pct) {
    for (int k = 0; k < 2; k++) {
        const int b = CLUSTER_BOARDS[cluster][k];
        if (b >= 0) desired_[b] = pct;
    }
    publishCmd();
}

void LightingModule::publishCmd() {
    if (!pub_) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(desired_, desired_ + NUM_BOARDS);
    pub_->publish(msg);
}

void LightingModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    auto qos = rclcpp::QoS(10).reliable();
    pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/lights/cmd", qos);
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/lights/feedback", qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            onFeedback(msg);
        });
}

void LightingModule::onFeedback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < static_cast<size_t>(NUM_BOARDS)) return;

    std::array<double, 5> fb{};
    for (int b = 0; b < NUM_BOARDS; b++) {
        fb[b] = qBound(0.0, msg->data[b], 100.0);
        // Adopt confirmed state so the next publish doesn't resurrect stale
        // values for boards the user hasn't touched since.
        desired_[b] = fb[b];
    }
    if (view_) view_->setPercents(fb, true);

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

    if (status_) status_->setText("Live  \xc2\xb7  /lights/feedback");
}

PLUGINLIB_EXPORT_CLASS(LightingModule, rover_hmi_core::GuiModule)
