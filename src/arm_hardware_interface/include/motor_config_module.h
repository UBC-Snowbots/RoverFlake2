// motor_config_module.h
// Displays the per-motor PID gains and limits pushed to firmware at startup.
//
// Lives in arm_hardware_interface because config data is embedded in
// /arm/moteus_feedback, published by moteus_driver_node in this same package.
// Config is read once from the first feedback message — the driver embeds it
// in every message, but we only need to populate the table once.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include "motor_addressing.h"

#include <QLabel>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"

class MotorConfigModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Motor Config"; }
    std::string layoutHint() const override { return "main"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);

    // 8 columns: Kp, Kd, Max I, Max Vel, Pos Min, Pos Max, Max V, Gear Ratio
    static constexpr int NUM_FIELDS = 8;
    QLabel* cells_[NUM_MOTORS][NUM_FIELDS] = {};
    QLabel* status_ = nullptr;
    // Set to true after the first valid config message is processed. Subsequent
    // feedback messages are ignored so the table isn't re-written on every tick.
    bool config_received_ = false;

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr sub_;
};
