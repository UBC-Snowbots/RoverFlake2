// motor_config_module.cpp
// See motor_config_module.h for module overview.

#include "motor_config_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QFont>

#include <pluginlib/class_list_macros.hpp>

static const char* FIELD_HEADERS[] = {
    "Kp", "Kd", "Max I (A)", "Max Vel (rev/s)",
    "Pos Min (rev)", "Pos Max (rev)", "Max V (V)", "Gear Ratio"
};

QWidget* MotorConfigModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(2);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::HeaderBg).arg(theme::Text);
    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);

    auto* corner = new QLabel("Joint");
    corner->setFont(monoBold);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    grid->addWidget(corner, 0, 0);

    for (int c = 0; c < NUM_FIELDS; c++) {
        auto* lbl = new QLabel(FIELD_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(lbl, 0, c + 1);
    }

    for (int r = 0; r < NUM_MOTORS; r++) {
        auto* jname = new QLabel(ARM_JOINTS[r].hardware_name);
        jname->setFont(monoBold);
        jname->setStyleSheet(
            QString("background: %1; color: %2; padding: 8px;")
            .arg(theme::Bg).arg(theme::MotorColors[r]));
        grid->addWidget(jname, r + 1, 0);

        for (int c = 0; c < NUM_FIELDS; c++) {
            auto* lbl = new QLabel("--");
            lbl->setFont(mono);
            lbl->setStyleSheet(cellStyle);
            lbl->setAlignment(Qt::AlignCenter);
            grid->addWidget(lbl, r + 1, c + 1);
            cells_[r][c] = lbl;
        }
    }

    status_ = new QLabel("Waiting for config from driver...");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_FIELDS + 1);

    for (int c = 0; c <= NUM_FIELDS; c++)
        grid->setColumnStretch(c, 1);

    return widget;
}

void MotorConfigModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
            onFeedback(msg);
        });
}

void MotorConfigModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    // config_received_ is checked first so this function becomes a no-op after
    // the table has been populated. The driver sends config in every feedback
    // message; we only need it once.
    if (config_received_ || !cells_[0][0]) return;
    if ((int)msg->config.size() < NUM_MOTORS) return;

    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);

    for (int r = 0; r < NUM_MOTORS; r++) {
        const auto& c = msg->config[r];
        cells_[r][0]->setText(QString::number(c.kp, 'f', 0));
        cells_[r][1]->setText(QString::number(c.kd, 'f', 0));
        cells_[r][2]->setText(QString::number(c.max_current_amps, 'f', 1));
        cells_[r][3]->setText(QString::number(c.max_velocity, 'f', 3));
        cells_[r][4]->setText(QString::number(c.min_position, 'f', 2));
        cells_[r][5]->setText(QString::number(c.max_position, 'f', 2));
        cells_[r][6]->setText(QString::number(c.max_voltage_volts, 'f', 1));
        cells_[r][7]->setText(c.gear_reduction > 0
            ? QString("1/%1").arg(qRound(1.0f / c.gear_reduction))
            : "--");

        for (int col = 0; col < NUM_FIELDS; col++)
            cells_[r][col]->setStyleSheet(cellStyle);
    }

    if (status_) {
        status_->setText("Config received from driver");
        status_->setStyleSheet(
            QString("background: %1; color: %2; padding: 8px;")
            .arg(theme::Bg).arg(theme::Green));
    }

    config_received_ = true;
}

PLUGINLIB_EXPORT_CLASS(MotorConfigModule, rover_hmi_core::GuiModule)
