// arm_view_module.h — "Digital Twin" and "Sim Arm" panels
//
// Both panels render the arm as a painted 2-projection FK view (top + side)
// using the kinematic chain from dev_arm_description_v2 and the shared
// ARM_JOINTS calibration. They differ only in where the angles come from:
//
//   ArmTwinModule  — the PHYSICAL arm, from motor telemetry: /arm/feedback
//                    (axis-space output revs decoded from CAN each driver
//                    cycle) converted through motorRevToJointRad(). Shows
//                    exactly what the arm is doing, not what it was told.
//   ArmSimModule   — the simulation: /joint_states (URDF radians), i.e.
//                    mock_arm or anything else publishing joint states.
//
// Run both side by side and any divergence between the two silhouettes IS
// the twin-vs-sim error (numerically on /arm/twin_error via arm_twin_node).

#pragma once

#include <rover_hmi_core/gui_module.h>
#include <rover_arm_common/motor_addressing.h>

#include <QLabel>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/arm_command.hpp"
#include "rover_msgs/msg/moteus_arm_status.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmViewWidget;  // painted FK view — defined in arm_view_module.cpp

// Shared scaffolding: FK widget + per-joint angle readout + status line.
class ArmViewModuleBase : public rover_hmi_core::GuiModule {
public:
    QWidget* createWidget(QWidget* parent) override;
    void start() override {}
    void stop() override {}

protected:
    virtual const char* statusHint() const = 0;   // shown until data arrives
    void applyAngles(const std::array<double, 6>& rad, bool fresh);
    void setStatus(const QString& text, const char* color);

    ArmViewWidget* view_ = nullptr;
    QLabel* status_ = nullptr;
    std::array<QLabel*, 6> angle_labels_{};
};

class ArmTwinModule : public ArmViewModuleBase {
public:
    std::string name()        const override { return "Digital Twin"; }
    std::string layoutHint()  const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }
    void setNode(rclcpp::Node::SharedPtr node) override;

protected:
    const char* statusHint() const override {
        return "waiting for /arm/feedback (moteus_driver telemetry)";
    }

private:
    rclcpp::Subscription<rover_msgs::msg::ArmCommand>::SharedPtr feedback_sub_;
    rclcpp::Subscription<rover_msgs::msg::MoteusArmStatus>::SharedPtr status_sub_;
    rclcpp::Time last_msg_;
    rclcpp::Node::SharedPtr node_;
};

class ArmSimModule : public ArmViewModuleBase {
public:
    std::string name()        const override { return "Sim Arm"; }
    std::string layoutHint()  const override { return "right"; }
    std::string sectionName() const override { return "Arm"; }
    bool defaultVisible()     const override { return false; }
    void setNode(rclcpp::Node::SharedPtr node) override;

protected:
    const char* statusHint() const override {
        return "waiting for /joint_states (run: ros2 run mock mock_arm_node)";
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
};
