// send_command_module.h
// Motor command panel — send position/stop commands, hold-to-jog, and
// multi-axis zero.
//
// Lives in arm_hardware_interface because commands are published to
// /arm/command, which is subscribed by moteus_driver_node in this same package.
//
// JogButton implements a hold-to-move pattern: jogPressed fires when the mouse
// button goes down and triggers a continuous velocity command; jogReleased fires
// on mouse-up and sends velocity 0 to stop the motor. Standard QPushButton
// clicked() is not used because it only fires on release.
//
// Commands are also published to /arm/hmi_log as formatted strings so
// CommandLogModule can display them without any direct coupling between modules.

#pragma once

#include <rover_hmi_core/gui_module.h>
#include "motor_addressing.h"
#include "arm_commands.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QMouseEvent>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "std_msgs/msg/string.hpp"

constexpr int NUM_ZERO_AXES = NUM_MOTORS;

class JogButton : public QPushButton {
    Q_OBJECT
public:
    JogButton(const QString& text, QWidget* parent = nullptr);
signals:
    void jogPressed();
    void jogReleased();
protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
};

class SendCommandModule : public rover_hmi_core::GuiModule {
public:
    std::string name() const override { return "Send Command"; }
    std::string layoutHint() const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }
    QWidget* createWidget(QWidget* parent) override;
    void setNode(rclcpp::Node::SharedPtr node) override;
    void start() override {}
    void stop() override {}

private:
    void sendPosition(int motor_id, double pos, double vel, double max_torque);
    void sendVelocity(int motor_id, double velocity);
    void sendStop(int motor_id);
    void sendStopAll();
    void sendZero(int motor_id);
    void logCmd(const QString& cmd);

    rclcpp::Publisher<rover_msgs::msg::ArmCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;

    QComboBox* motor_select_ = nullptr;
    QComboBox* cmd_type_ = nullptr;
    QDoubleSpinBox* position_spin_ = nullptr;
    QDoubleSpinBox* velocity_spin_ = nullptr;
    QDoubleSpinBox* torque_spin_ = nullptr;
    QDoubleSpinBox* jog_speed_spin_ = nullptr;
    QCheckBox* pos_enable_ = nullptr;
    QCheckBox* vel_enable_ = nullptr;
    QCheckBox* torque_enable_ = nullptr;
    std::array<QCheckBox*, NUM_ZERO_AXES> zero_checks_{};
};
