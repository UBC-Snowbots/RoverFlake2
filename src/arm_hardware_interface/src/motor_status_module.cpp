// motor_status_module.cpp
//
// HMI plugin that displays a live per-motor status table for the arm.
//
// This module lives in arm_hardware_interface because the motor status display
// is tightly coupled to the driver's data — it subscribes directly to
// /arm/moteus_feedback which is published by the moteus_driver node in this
// same package. It uses motor_addressing.h from this package for NUM_MOTORS
// and joint names.

#include "motor_status_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QFont>

#include <pluginlib/class_list_macros.hpp>

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

static const char* FIELD_HEADERS[] = {
    "Mode", "Fault", "Position (rev)", "Velocity (rev/s)",
    "Torque (Nm)", "Voltage (V)", "Temp (C)"
};

QWidget* MotorStatusModule::createWidget(QWidget* parent) {
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
        auto* jname = new QLabel(JOINT_NAMES[r]);
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

    status_ = new QLabel("Waiting for data...");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_FIELDS + 1);

    for (int c = 0; c <= NUM_FIELDS; c++)
        grid->setColumnStretch(c, 1);

    return widget;
}

void MotorStatusModule::setNode(rclcpp::Node::SharedPtr node) {
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
            onFeedback(msg);
        });
}

void MotorStatusModule::onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
    if (!cells_[0][0]) return;

    auto faultStyle = QString("background: %1; color: %2; padding: 8px; font-weight: bold;")
        .arg(theme::Bg).arg(theme::Red);
    auto okStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Green);
    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);

    int active = 0;
    for (int i = 0; i < NUM_MOTORS && i < (int)msg->status.size(); i++) {
        const auto& s = msg->status[i];
        active++;

        cells_[i][0]->setText(QString::number(s.moteus_mode));
        cells_[i][1]->setText(QString::number(s.moteus_fault));
        cells_[i][2]->setText(QString::number(s.curr_position, 'f', 3));
        cells_[i][3]->setText(QString::number(s.curr_velocity, 'f', 3));
        cells_[i][4]->setText(QString::number(s.curr_torque, 'f', 3));
        cells_[i][5]->setText(QString::number(s.curr_voltage_volts, 'f', 1));
        cells_[i][6]->setText(QString::number(s.driver_temp_degreesc, 'f', 1));

        // Only the Fault column (index 1) gets red/green treatment to draw
        // immediate attention to fault conditions. All other columns use plain
        // white text so the color signal remains unambiguous.
        cells_[i][1]->setStyleSheet(s.moteus_fault != 0 ? faultStyle : okStyle);
        for (int c = 0; c < NUM_FIELDS; c++)
            if (c != 1) cells_[i][c]->setStyleSheet(cellStyle);
    }

    if (status_)
        status_->setText(QString("Receiving from %1 motors").arg(active));
}

PLUGINLIB_EXPORT_CLASS(MotorStatusModule, rover_hmi_core::GuiModule)
