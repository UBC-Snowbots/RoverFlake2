// lighting_module.cpp — "Lighting"
//
// Zone rows: Front(cyan), Back(orange), Left(yellow), Right(green).
// On/Off toggle + brightness slider + value label + color swatch + auto indicator.

#include "lighting_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QScrollArea>
#include <QFont>
#include <QSizePolicy>
#include <QColor>

#include <pluginlib/class_list_macros.hpp>

static const char* ZONE_NAMES[]   = { "Front", "Back", "Left", "Right" };
static const char* ZONE_COLORS[]  = { theme::Cyan, "#ff9944", theme::Yellow, theme::Green };
static const char* COL_HEADERS[]  = { "Zone", "On/Off", "Brightness", "Value%", "Color", "Auto" };

QWidget* LightingModule::createWidget(QWidget* parent) {
    auto* scroll = new QScrollArea(parent);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll->setStyleSheet(
        QString("QScrollArea { background: %1; }"
                "QScrollArea > QWidget > QWidget { background: %1; }")
        .arg(theme::Bg));

    auto* widget = new QWidget();
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(2);
    grid->setContentsMargins(4, 4, 4, 4);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString(
        "background: %1; color: %2; padding: 4px 8px; border-bottom: 1px solid %3; font-weight: bold;")
        .arg(theme::HeaderBg).arg(theme::Text).arg(theme::BorderDim);

    // Column headers
    for (int c = 0; c < 6; c++) {
        auto* lbl = new QLabel(COL_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        grid->addWidget(lbl, 0, c);
    }

    for (int z = 0; z < NUM_ZONES; z++) {
        zone_enabled_[z] = false;

        // Zone name label
        auto* zone_lbl = new QLabel(ZONE_NAMES[z]);
        zone_lbl->setFont(monoBold);
        zone_lbl->setAlignment(Qt::AlignCenter);
        zone_lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        zone_lbl->setStyleSheet(
            QString("background: %1; color: %2; padding: 4px 8px; border: 1px solid %3; font-weight: bold;")
            .arg(theme::Bg).arg(ZONE_COLORS[z]).arg(theme::BorderDim));
        grid->addWidget(zone_lbl, z + 1, 0);

        // On/Off toggle button
        toggle_btns_[z] = new QPushButton("OFF");
        toggle_btns_[z]->setFont(monoBold);
        toggle_btns_[z]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        toggle_btns_[z]->setStyleSheet(
            QString("background: %1; color: %2; border: 1px solid %2;"
                    " border-radius: 4px; padding: 6px; font-weight: bold;")
            .arg(theme::BgPanel).arg(theme::TextDim));
        int zone_idx = z;
        QObject::connect(toggle_btns_[z], &QPushButton::clicked, [this, zone_idx]() {
            zone_enabled_[zone_idx] = !zone_enabled_[zone_idx];
            int bright = sliders_[zone_idx] ? sliders_[zone_idx]->value() : 50;
            publishControl(zone_idx, zone_enabled_[zone_idx], bright);
            // Update button appearance immediately
            if (toggle_btns_[zone_idx]) {
                if (zone_enabled_[zone_idx]) {
                    toggle_btns_[zone_idx]->setText("ON");
                    toggle_btns_[zone_idx]->setStyleSheet(
                        QString("background: #1b3d2a; color: %1; border: 1px solid %1;"
                                " border-radius: 4px; padding: 6px; font-weight: bold;")
                        .arg(theme::Green));
                } else {
                    toggle_btns_[zone_idx]->setText("OFF");
                    toggle_btns_[zone_idx]->setStyleSheet(
                        QString("background: %1; color: %2; border: 1px solid %2;"
                                " border-radius: 4px; padding: 6px; font-weight: bold;")
                        .arg(theme::BgPanel).arg(theme::TextDim));
                }
            }
        });
        grid->addWidget(toggle_btns_[z], z + 1, 1);

        // Brightness slider
        sliders_[z] = new QSlider(Qt::Horizontal);
        sliders_[z]->setRange(0, 100);
        sliders_[z]->setValue(50);
        sliders_[z]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sliders_[z]->setStyleSheet(
            QString("QSlider::groove:horizontal { height: 6px; background: %1; border-radius: 3px; }"
                    "QSlider::handle:horizontal { width: 14px; height: 14px; background: %2;"
                    " border-radius: 7px; margin: -4px 0; }"
                    "QSlider::sub-page:horizontal { background: %3; border-radius: 3px; }")
            .arg(theme::BgPanel).arg(theme::Text).arg(theme::Cyan));
        QObject::connect(sliders_[z], &QSlider::valueChanged, [this, zone_idx](int val) {
            if (value_lbls_[zone_idx])
                value_lbls_[zone_idx]->setText(QString("%1%").arg(val));
            publishControl(zone_idx, zone_enabled_[zone_idx], val);
        });
        grid->addWidget(sliders_[z], z + 1, 2);

        // Value% label
        value_lbls_[z] = new QLabel("50%");
        value_lbls_[z]->setFont(mono);
        value_lbls_[z]->setAlignment(Qt::AlignCenter);
        value_lbls_[z]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        value_lbls_[z]->setStyleSheet(
            QString("color: %1; padding: 4px 6px; border: 1px solid %2;")
            .arg(theme::Text).arg(theme::BorderDim));
        grid->addWidget(value_lbls_[z], z + 1, 3);

        // Color swatch
        swatch_lbls_[z] = new QLabel();
        swatch_lbls_[z]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        swatch_lbls_[z]->setStyleSheet(
            QString("background: %1; border: 1px solid %2; border-radius: 3px;")
            .arg(theme::BgPanel).arg(theme::BorderDim));
        grid->addWidget(swatch_lbls_[z], z + 1, 4);

        // Auto indicator
        auto_lbls_[z] = new QLabel("--");
        auto_lbls_[z]->setFont(mono);
        auto_lbls_[z]->setAlignment(Qt::AlignCenter);
        auto_lbls_[z]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto_lbls_[z]->setStyleSheet(
            QString("color: %1; padding: 4px 6px; border: 1px solid %2;")
            .arg(theme::TextDim).arg(theme::BorderDim));
        grid->addWidget(auto_lbls_[z], z + 1, 5);
    }

    // Status row
    status_ = new QLabel("Waiting for lighting status...");
    status_->setFont(mono);
    status_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_->setStyleSheet(
        QString("color: %1; padding: 4px 6px; border: 1px solid %2;")
        .arg(theme::TextDim).arg(theme::BorderDim));
    grid->addWidget(status_, NUM_ZONES + 1, 0, 1, 6);

    // Equal stretches
    for (int c = 0; c < 6; c++) grid->setColumnStretch(c, 1);
    for (int r = 0; r <= NUM_ZONES + 1; r++) grid->setRowStretch(r, 1);

    scroll->setWidget(widget);
    return scroll;
}

void LightingModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    auto qos = rclcpp::QoS(10).reliable();
    pub_ = node->create_publisher<rover_msgs::msg::LightControl>("/lighting/control", qos);
    sub_ = node->create_subscription<rover_msgs::msg::LightStatus>(
        "/lighting/status", qos,
        [this](const rover_msgs::msg::LightStatus::SharedPtr msg) {
            onLightStatus(msg);
        });
}

void LightingModule::onLightStatus(const rover_msgs::msg::LightStatus::SharedPtr msg) {
    for (int z = 0; z < NUM_ZONES; z++) {
        bool on   = msg->enabled[z];
        int  brt  = msg->brightness[z];
        bool autom = msg->auto_mode[z];
        int  r    = msg->color_r[z];
        int  g    = msg->color_g[z];
        int  b    = msg->color_b[z];

        zone_enabled_[z] = on;

        if (toggle_btns_[z]) {
            toggle_btns_[z]->setText(on ? "ON" : "OFF");
            if (on) {
                toggle_btns_[z]->setStyleSheet(
                    QString("background: #1b3d2a; color: %1; border: 1px solid %1;"
                            " border-radius: 4px; padding: 6px; font-weight: bold;")
                    .arg(theme::Green));
            } else {
                toggle_btns_[z]->setStyleSheet(
                    QString("background: %1; color: %2; border: 1px solid %2;"
                            " border-radius: 4px; padding: 6px; font-weight: bold;")
                    .arg(theme::BgPanel).arg(theme::TextDim));
            }
        }

        if (sliders_[z]) {
            sliders_[z]->blockSignals(true);
            sliders_[z]->setValue(brt);
            sliders_[z]->blockSignals(false);
        }

        if (value_lbls_[z])
            value_lbls_[z]->setText(QString("%1%").arg(brt));

        if (swatch_lbls_[z]) {
            if (autom) {
                QColor c(r, g, b);
                swatch_lbls_[z]->setStyleSheet(
                    QString("background: %1; border: 1px solid %2; border-radius: 3px;")
                    .arg(c.name()).arg(theme::BorderDim));
            } else {
                swatch_lbls_[z]->setStyleSheet(
                    QString("background: %1; border: 1px solid %2; border-radius: 3px;")
                    .arg(theme::BgPanel).arg(theme::BorderDim));
            }
        }

        if (auto_lbls_[z]) {
            auto_lbls_[z]->setText(autom ? "AUTO" : "--");
            auto_lbls_[z]->setStyleSheet(
                QString("color: %1; padding: 4px 6px; border: 1px solid %2;")
                .arg(autom ? theme::Cyan : theme::TextDim).arg(theme::BorderDim));
        }
    }

    if (status_)
        status_->setText("Live  \xc2\xb7  Lighting status received");
}

void LightingModule::publishControl(int zone, bool enabled, int brightness) {
    if (!pub_) return;
    rover_msgs::msg::LightControl msg;
    msg.zone       = zone;
    msg.enabled    = enabled;
    msg.brightness = brightness;
    pub_->publish(msg);
}

PLUGINLIB_EXPORT_CLASS(LightingModule, rover_hmi_core::GuiModule)
