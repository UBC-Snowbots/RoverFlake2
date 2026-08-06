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
#include <rover_arm_common/motor_addressing.h>  // NUM_MOTORS

#include <QLabel>
#include <QPushButton>
#include <QGraphicsOpacityEffect>

#include "driver_process.h"
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

    // 12 columns — see FIELD_HEADERS in .cpp
    static constexpr int NUM_FIELDS = 12; // must equal NUM_COLS in motor_status_module.cpp

private:
    friend struct MotorStatusTestAccess;  // test seam (test/motor_status_display_test.cpp)
    void onFeedback(const rover_msgs::msg::MoteusArmStatus::SharedPtr msg);
    void setBanner(const QString& text, const char* fg, const char* bg, bool bold = true);
    void updateDriverBtn();

    QLabel* cells_[NUM_MOTORS][NUM_FIELDS] = {};
    QLabel* row_labels_[NUM_MOTORS] = {};
    QLabel* status_ = nullptr;
    qint64  last_msg_ms_ = 0;   // staleness watchdog: no telemetry -> driver down
    QGraphicsOpacityEffect* dim_ = nullptr;  // stale data must LOOK stale
    QPushButton*  drv_btn_ = nullptr;
    DriverProcess drv_;         // bench-only HMI-owned driver (see driver_process.h)

    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr sub_;
};
