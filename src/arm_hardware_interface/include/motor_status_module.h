// motor_status_module.h
//
// HMI plugin that displays a live per-motor status table for the arm.
//
// This module lives in arm_hardware_interface because the motor status display
// is tightly coupled to the driver's data — it subscribes directly to
// /arm/moteus_feedback which is published by the moteus_driver node in this
// same package. It uses motor_addressing.h from this package for NUM_MOTORS
// and joint names.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include "motor_addressing.h"  // NUM_MOTORS

#include <QLabel>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"

class MotorStatusModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Motor Status"; }
    std::string layoutHint() const override { return "main"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);

    // NUM_FIELDS corresponds to the columns: Mode, Fault, Position, Velocity,
    // Torque, Voltage, Temp.
    static constexpr int NUM_FIELDS = 7;
    QLabel* cells_[NUM_MOTORS][NUM_FIELDS] = {};
    QLabel* status_ = nullptr;

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr sub_;
};
