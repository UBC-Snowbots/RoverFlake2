// wheel_telemetry_module.h — "Wheel Telemetry"
//
// Grid table: 6 wheels x (Speed | Torque | Temp | Power | Enabled).
// Subscribes to /drivetrain/wheel_states (rover_msgs::WheelStates).
// Section: Electricals. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>
#include <QScrollArea>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/wheel_states.hpp"

class WheelTelemetryModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Wheel Telemetry"; }
    std::string sectionName() const override { return "Electricals"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     setNode(rclcpp::Node::SharedPtr node) override;
    void     start() override {}
    void     stop() override {}

private:
    void onWheelStates(const rover_msgs::msg::WheelStates::SharedPtr msg);

    static constexpr int NUM_WHEELS = 6;
    // Columns: Speed | Torque | Temp | Power | Enabled
    static constexpr int NUM_COLS   = 5;

    QLabel* row_labels_[NUM_WHEELS]         = {};
    QLabel* cells_[NUM_WHEELS][NUM_COLS]    = {};
    QLabel* status_                         = nullptr;

    rclcpp::Subscription<rover_msgs::msg::WheelStates>::SharedPtr sub_;
};
