// motor_status_module.h  —  "Motor Telemetry"
//
// Read-only live telemetry table for all arm motors.
// Subscribes to /arm/moteus_feedback (BldcServoStatus per motor).
//
// Columns (all read from hardware each poll cycle):
//   Mode | Fault | Pos (curr) | Pos (des) | Vel (curr) | Vel (des)
//   Torque | Current (A) | Power (W) | Voltage (V) | Temp (°C)
//
// Row background is color-coded by motor state; temperature cell is
// additionally color-coded by value (>60°C red, 40–60°C yellow).

#pragma once

#include <rover_hmi_core/gui_module.h>
#include "motor_addressing.h"  // NUM_MOTORS

#include <QLabel>
#include <QScrollArea>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"

class MotorStatusModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Motor Telemetry"; }
    std::string layoutHint()  const override { return "main"; }
    std::string sectionName() const override { return "Arm"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);

    // 11 columns — see FIELD_HEADERS in .cpp
    static constexpr int NUM_FIELDS = 11;
    QLabel* cells_[NUM_MOTORS][NUM_FIELDS] = {};
    QLabel* row_labels_[NUM_MOTORS] = {};
    QLabel* status_ = nullptr;

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr sub_;
};
