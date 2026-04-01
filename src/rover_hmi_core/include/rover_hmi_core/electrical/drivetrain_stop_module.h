// drivetrain_stop_module.h — "Drivetrain Stop"
//
// Large stop/release button with status indicator.
// Publishes to /drivetrain/remote_stop (std_msgs::Bool).
// Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class DrivetrainStopModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Drivetrain Stop"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override {}

private:
    void onStopStatus(const std_msgs::msg::Bool::SharedPtr msg);
    void publishStop(bool stopped);
    void updateUi();

    bool stopped_ = false;

    QLabel*      status_lbl_  = nullptr;
    QPushButton* action_btn_  = nullptr;
    QLabel*      warning_lbl_ = nullptr;
    QTimer*      blink_timer_ = nullptr;
    bool         blink_state_ = false;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};
