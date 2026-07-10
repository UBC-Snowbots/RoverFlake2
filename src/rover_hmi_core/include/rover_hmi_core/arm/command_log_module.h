// command_log_module.h
// Scrolling log panel. Subscribes to two topics:
//   /arm/hmi_log    — commands sent by HMI modules (SendCommandModule et al.)
//   /arm/config_log — informational messages from the arm driver
//
// Entries are color-coded by content: stop commands in red, position commands
// in green, driver/config messages in cyan, and everything else in the dim
// default text color.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QTextEdit>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CommandLogModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Command Log"; }
    std::string layoutHint() const override { return "bottom"; }
    std::string sectionName() const override { return "Arm"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void appendLog(const QString& text, const QString& color);

    QTextEdit* log_ = nullptr;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hmi_log_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_log_sub_;
};
