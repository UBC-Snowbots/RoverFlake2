// drivetrain_stop_module.cpp — "Drivetrain Stop"

#include "drivetrain_stop_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QFont>
#include <QSizePolicy>

#include <pluginlib/class_list_macros.hpp>



QWidget* DrivetrainStopModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));

    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(16);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->addStretch(1);

    QFont monoLg("monospace", theme::FontSizeLg, QFont::Bold);
    QFont mono("monospace", theme::FontSize);
    QFont monoSm("monospace", theme::FontSizeSm);

    // Status label
    status_lbl_ = new QLabel("RUNNING");
    status_lbl_->setFont(monoLg);
    status_lbl_->setAlignment(Qt::AlignCenter);
    status_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_lbl_->setStyleSheet(QString("color: %1; font-size: 28px; font-weight: bold;").arg(theme::Green));
    layout->addWidget(status_lbl_);

    // Action button
    action_btn_ = new QPushButton("STOP");
    action_btn_->setFont(monoLg);
    action_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    action_btn_->setMinimumHeight(80);
    action_btn_->setStyleSheet(
        QString("background: #3d1b1b; color: %1; border: 2px solid %1;"
                " border-radius: 8px; padding: 16px; font-size: 24px; font-weight: bold;")
        .arg(theme::Red));
    QObject::connect(action_btn_, &QPushButton::clicked, [this]() {
        stopped_ = !stopped_;
        publishStop(stopped_);
        updateUi();
    });
    layout->addWidget(action_btn_);

    // Warning label
    warning_lbl_ = new QLabel("\u26a0 Drivetrain is remotely stopped");
    warning_lbl_->setFont(mono);
    warning_lbl_->setAlignment(Qt::AlignCenter);
    warning_lbl_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    warning_lbl_->setStyleSheet(QString("color: %1;").arg(theme::Yellow));
    warning_lbl_->setVisible(false);
    layout->addWidget(warning_lbl_);

    layout->addStretch(1);

    // Blink timer for STOPPED label
    blink_timer_ = new QTimer(widget);
    blink_timer_->setInterval(500);
    QObject::connect(blink_timer_, &QTimer::timeout, [this]() {
        blink_state_ = !blink_state_;
        if (status_lbl_ && stopped_) {
            status_lbl_->setStyleSheet(
                QString("color: %1; font-size: 28px; font-weight: bold;")
                .arg(blink_state_ ? theme::Red : theme::TextDim));
        }
    });

    return widget;
}

void DrivetrainStopModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    auto qos = rclcpp::QoS(10).reliable();
    pub_ = node->create_publisher<std_msgs::msg::Bool>("/drivetrain/remote_stop", qos);
    sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "/drivetrain/remote_stop_status", qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            onStopStatus(msg);
        });
}

void DrivetrainStopModule::onStopStatus(const std_msgs::msg::Bool::SharedPtr msg) {
    stopped_ = msg->data;
    updateUi();
}

void DrivetrainStopModule::publishStop(bool stopped) {
    if (!pub_) return;
    std_msgs::msg::Bool msg;
    msg.data = stopped;
    pub_->publish(msg);
}

void DrivetrainStopModule::updateUi() {
    if (stopped_) {
        if (status_lbl_) {
            status_lbl_->setText("STOPPED");
            status_lbl_->setStyleSheet(
                QString("color: %1; font-size: 28px; font-weight: bold;").arg(theme::Red));
        }
        if (action_btn_) {
            action_btn_->setText("RELEASE");
            action_btn_->setStyleSheet(
                QString("background: #1b3d2a; color: %1; border: 2px solid %1;"
                        " border-radius: 8px; padding: 16px; font-size: 24px; font-weight: bold;")
                .arg(theme::Green));
        }
        if (warning_lbl_) warning_lbl_->setVisible(true);
        if (blink_timer_) blink_timer_->start();
    } else {
        if (status_lbl_) {
            status_lbl_->setText("RUNNING");
            status_lbl_->setStyleSheet(
                QString("color: %1; font-size: 28px; font-weight: bold;").arg(theme::Green));
        }
        if (action_btn_) {
            action_btn_->setText("STOP");
            action_btn_->setStyleSheet(
                QString("background: #3d1b1b; color: %1; border: 2px solid %1;"
                        " border-radius: 8px; padding: 16px; font-size: 24px; font-weight: bold;")
                .arg(theme::Red));
        }
        if (warning_lbl_) warning_lbl_->setVisible(false);
        if (blink_timer_) blink_timer_->stop();
        blink_state_ = false;
    }
}

PLUGINLIB_EXPORT_CLASS(DrivetrainStopModule, rover_hmi_core::GuiModule)
